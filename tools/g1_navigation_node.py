#!/usr/bin/env python3
"""Onboard G1 PosNav execution, idle until Start Navigation is requested."""
import argparse
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from unitree_api.msg import Request
from skimage.morphology import binary_dilation, disk
from g1_navigation_core import Navigation
from stg_error_recorder import STGErrorRecorder, STGErrorLog
from experiment_recorder import TrialRecorder, ExperimentSTGRecorder
from posnav_fmm.planner import plan_snapshot
from posnav_fmm.errors import PlanningRetry
from posnav_fmm.visibility import line_is_collision_free


def planar(message):
    p, q = message.pose.position, message.pose.orientation
    values = (p.x, p.y, q.x, q.y, q.z, q.w)
    if message.header.frame_id != 'map' or not all(math.isfinite(v) for v in values):
        raise ValueError('Expected finite map pose')
    norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    if norm < 1e-9:
        raise ValueError('Invalid pose quaternion')
    x, y, z, w = (v/norm for v in (q.x, q.y, q.z, q.w))
    return (p.x, p.y, math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


class NavigationNode(Node):
    def __init__(self, config, record_stg_errors=False, stg_error_dir=None, experiment_dir=None):
        super().__init__('g1_navigation')
        self.cfg = yaml.safe_load(Path(config).read_text())
        self.error_log = None
        self.recording_error = None
        if record_stg_errors:
            directory = stg_error_dir or Path(__file__).resolve().parents[1]/'stg_error_results'
            self.error_log = STGErrorLog(directory)
            self.get_logger().info('STG error records: '+str(self.error_log.path))
        self.control = Navigation(self.plan, self.clear,
                                  STGErrorRecorder() if record_stg_errors else None)
        self.experiment = None
        if experiment_dir is not None:
            parameters = {key: getattr(self.control, key) for key, value in vars(Navigation).items()
                          if isinstance(value, (int, float))}
            self.experiment = TrialRecorder(experiment_dir, dict(control=parameters, planning=self.cfg))
            self.control.recorder = ExperimentSTGRecorder(self.experiment)
        self.goal = None
        self.goal_pose = None
        self.pose_source_stamp = None
        self.record_grid = self.record_grid_metadata = None
        self.last_health_time = -math.inf
        self.last_health_key = None
        self.occupancy = self.free = None
        self.map_stamp = None
        self.last_status = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.commands = self.create_publisher(Request, '/api/sport/request', 1)
        self.status = self.create_publisher(String, '/g1/navigation/status', latched)
        self.stg_display = self.create_publisher(Marker, '/g1/live_bev/stg_macro', latched)
        self.create_subscription(PoseStamped, '/g1/live_bev/robot', self.on_pose, 1)
        self.create_subscription(OccupancyGrid, '/g1/bev/task', self.on_map, latched)
        self.create_subscription(PoseStamped, '/g1/live_bev/goal_2d', self.on_goal, 10)
        self.create_subscription(Empty, '/g1/navigation/cancel', lambda _: self.cancel('Localization stopped or map reset'), 10)
        if self.experiment:
            self.create_subscription(String, '/g1/live_bev/status', self.on_health, latched)
        self.create_service(Trigger, '/g1/navigation/start', self.start)
        self.create_service(Trigger, '/g1/navigation/stop', self.stop)
        from g1_action_server import NavigationAction
        self.action = NavigationAction(self)
        self.create_timer(1./self.control.control_hz, self.tick)
        self.publish_status()

    def send(self, velocity):
        request = Request()
        request.header.identity.id = time.monotonic_ns()
        request.header.identity.api_id = 7105
        request.header.policy.noreply = True
        request.parameter = json.dumps({'velocity': [velocity[0], 0., velocity[1]],
                                        'duration': self.control.duration})
        self.commands.publish(request)
        if self.experiment:
            self.experiment.command(velocity, self.control.duration)

    def cancel(self, reason):
        if self.control.active:
            self.send((0., 0.))
        if self.experiment:
            self.experiment.event('cancel', reason=reason, was_active=self.control.active)
        self.control.stop(reason)
        self.flush_recordings()
        self.publish_status()
        if hasattr(self, 'action'):
            self.action.update()

    def snapshot(self):
        if self.experiment:
            return self.experiment.snapshot(self.record_grid, self.record_grid_metadata)
        return None

    def on_health(self, msg):
        try:
            health = json.loads(msg.data)
        except (ValueError, TypeError):
            self.experiment.event('invalid_health_message', raw=msg.data)
            return
        key = (health.get('running'), str(health.get('message', '')).split(':')[0])
        now = time.monotonic()
        if now-self.last_health_time >= 1. or key != self.last_health_key:
            self.experiment.event('localization_health', health=health)
            self.last_health_time, self.last_health_key = now, key

    def flush_recordings(self):
        if self.experiment and self.experiment.writer.error and self.recording_error is None:
            self.recording_error = 'Experiment recording failed: '+self.experiment.writer.error
            self.get_logger().error(self.recording_error)
            self.cancel(self.recording_error)
        if self.experiment and self.error_log is None:
            self.control.recorder.completed.clear()
        if self.error_log is None or self.recording_error is not None:
            return
        try:
            self.error_log.flush(self.control.recorder)
        except (OSError, ValueError) as exc:
            self.recording_error = 'STG recording failed: '+str(exc)
            self.get_logger().error(self.recording_error)
            self.cancel(self.recording_error)

    def on_goal(self, msg):
        # Even a redraw of the same XY/yaw ends the old task.
        self.cancel('Goal redrawn; press Start Navigation')
        try:
            self.goal_pose = planar(msg)
            self.goal = self.goal_pose[:2]
            if self.experiment:
                self.experiment.event('goal_selected', source_stamp_ns=stamp_ns(msg), goal=self.goal_pose)
        except ValueError as exc:
            self.goal = None
            self.goal_pose = None
            self.control.reason = str(exc)
            if self.experiment:
                self.experiment.event('goal_rejected', reason=str(exc))
        self.publish_status()

    def on_pose(self, msg):
        try:
            self.control.pose = planar(msg)
            # Deliberately no source stamp age, duplicate or rollback checks.
            self.control.pose_time = time.monotonic()
            self.pose_source_stamp = stamp_ns(msg)
            if self.experiment:
                self.experiment.pose(self.control.pose, self.pose_source_stamp, self.control.state)
        except ValueError as exc:
            self.control.pose_time = -math.inf
            if self.experiment:
                self.experiment.event('pose_invalid', reason=str(exc), source_stamp_ns=stamp_ns(msg))

    def on_map(self, msg):
        try:
            stamp = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
            if msg.header.frame_id != 'map' or stamp == 0:
                self.control.map_time = -math.inf
                self.map_stamp = None
                if self.experiment:
                    self.experiment.event('map_invalid', reason='Expected nonzero map timestamp in map frame',
                                          source_stamp_ns=stamp)
                return
            if stamp == self.map_stamp:
                return  # Republishing retained maps must not refresh validity.
            age = max(0., (self.get_clock().now().nanoseconds-stamp)/1e9)
            self.control.map_time = time.monotonic()-age
            self.map_stamp = stamp
            data = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
            if data.size == 0 or not math.isclose(msg.info.resolution, .05, abs_tol=1e-8):
                raise ValueError('Expected 5 cm task grid')
            self.occupancy = data != 0
            self.origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
            self.resolution = .05
            self.free = ~binary_dilation(self.occupancy, disk(self.cfg['fmm_inflation_cells']))
            if self.experiment:
                self.record_grid = data.astype(np.int8)
                p, q = msg.info.origin.position, msg.info.origin.orientation
                self.record_grid_metadata = dict(source_stamp_ns=stamp, frame_id=msg.header.frame_id,
                    resolution_m=msg.info.resolution, width=msg.info.width, height=msg.info.height,
                    origin_position=[p.x, p.y, p.z], origin_quaternion=[q.x, q.y, q.z, q.w])
        except (ValueError, TypeError) as exc:
            self.control.map_time = -math.inf
            if self.experiment:
                self.experiment.event('map_invalid', reason=str(exc))

    def cell(self, xy):
        col, row = np.floor((np.asarray(xy)-self.origin)/self.resolution).astype(int)
        return (row, col)

    def clear(self, start, end, limit):
        start, end = np.asarray(start), np.asarray(end)
        distance = np.linalg.norm(end-start)
        if distance > limit:
            end = start + (end-start)*(limit/distance)
        clear = line_is_collision_free(self.cell(start), self.cell(end), self.free)
        if not clear and self.experiment:
            self.experiment.obstacles += 1
            self.experiment.event('passage_blocked', start_xy=start, checked_end_xy=end,
                                  limit_m=limit if math.isfinite(limit) else None, map_ref=self.snapshot())
        return clear

    def plan(self, pose, goal):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'stg_macro'
        marker.id = 0
        marker.action = Marker.DELETE
        map_ref = self.snapshot()
        planning_started = time.monotonic()
        if self.experiment:
            self.experiment.event('planning_started', attempt_id=self.experiment.attempt+1,
                                  start_xy=pose, goal_xy=goal, map_ref=map_ref)
        try:
            plan = plan_snapshot(self.occupancy, self.origin, self.resolution, pose, goal,
                inflation_cells=self.cfg['fmm_inflation_cells'], step_cells=self.cfg['fmm_step_cells'],
                max_stgs=self.cfg['fmm_max_stgs'], min_step_cells=self.cfg['fmm_min_step_cells'])
        except Exception as exc:
            if self.experiment:
                self.experiment.planning(pose, goal, map_ref,
                    'retry' if isinstance(exc, PlanningRetry) else 'error', str(exc),
                    duration_s=time.monotonic()-planning_started)
            if isinstance(exc, PlanningRetry):
                self.stg_display.publish(marker)
            raise
        if self.experiment:
            self.experiment.planning(pose, goal, map_ref,
                'selected' if plan.stg_xy is not None else 'no_stg', plan.reason,
                stg=plan.stg_xy, rollout_rc=plan.rollout_rc,
                duration_s=time.monotonic()-planning_started)
        if plan.stg_xy is not None:
            marker.action = Marker.ADD
            marker.type = Marker.SPHERE
            marker.pose.orientation.w = 1.
            marker.pose.position.x, marker.pose.position.y = plan.stg_xy
            marker.pose.position.z = .1
            marker.scale.x = marker.scale.y = marker.scale.z = .15
            marker.color.r = marker.color.b = marker.color.a = 1.
        self.stg_display.publish(marker)
        return plan.stg_xy

    def start(self, _, response):
        try:
            if self.action.busy:
                raise ValueError('An Action goal owns navigation')
            self.begin_navigation(self.goal_pose)
            response.success = True
            response.message = 'Navigation started'
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
        if self.experiment:
            self.experiment.event('start_request', accepted=response.success, message=response.message)
        self.publish_status()
        return response

    def begin_navigation(self, goal_pose):
        """Shared entry; caller holds the node's mutually exclusive callback group."""
        if self.experiment and self.experiment.writer.error:
            raise ValueError('Experiment recording failed: '+self.experiment.writer.error)
        if self.recording_error is not None:
            raise ValueError(self.recording_error)
        if goal_pose is None:
            raise ValueError('Draw the goal arrow first')
        self.control.start(goal_pose[:2], time.monotonic())
        self.goal_pose = tuple(goal_pose)
        self.goal = self.goal_pose[:2]
        if self.experiment:
            self.experiment.begin(self.goal_pose, self.control.pose, self.pose_source_stamp, self.control.goal_tol)
            self.experiment.event('initial_task_map', map_ref=self.snapshot())

    def stop(self, _, response):
        if self.experiment:
            self.experiment.event('stop_request')
        self.cancel('Stopped by user')
        response.success = True
        response.message = 'Navigation stopped'
        return response

    def tick(self):
        self.action.update()
        try:
            velocity = self.control.tick(time.monotonic())
            if velocity is not None:
                self.send(velocity)
        except Exception as exc:
            self.cancel('Navigation stopped: '+str(exc))
        # Disk I/O follows command publication, never precedes the stop command.
        self.flush_recordings()
        self.publish_status()
        self.action.update()

    def destroy_node(self):
        self.action.close()
        return super().destroy_node()

    def publish_status(self):
        c = self.control
        status = json.dumps(dict(active=c.active, state=c.state, message=c.paused or c.reason,
                                 stg=c.stg))
        if status != self.last_status:
            self.status.publish(String(data=status))
            self.last_status = status
            if self.experiment:
                self.experiment.event('navigation_status', status=json.loads(status))
        if self.experiment and self.experiment.trial_id and not c.active:
            age = time.monotonic()-c.pose_time
            self.experiment.finish(c.reason, c.pose, age if math.isfinite(age) else None,
                                   c.input_problem(time.monotonic()), self.snapshot())

    def close_recordings(self):
        if self.error_log is not None:
            self.error_log.close()
        if self.experiment is not None:
            self.experiment.close()


def stamp_ns(message):
    return message.header.stamp.sec*1000000000 + message.header.stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1]/'fastlio/config/live_bev.yaml')
    parser.add_argument('--record-stg-errors', action='store_true',
                        help='Save per-STG poses, anomalies and valid XY L2 errors as JSONL')
    parser.add_argument('--experiment-dir', type=Path, help='Internal: existing experiment session directory')
    args = parser.parse_args()
    rclpy.init()
    node = NavigationNode(args.config, record_stg_errors=args.record_stg_errors,
                          experiment_dir=args.experiment_dir)
    def interrupted(*_):
        raise KeyboardInterrupt
    # Keep ROS alive until the final zero velocity has been published.
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel('Navigation process stopped')
        node.close_recordings()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
