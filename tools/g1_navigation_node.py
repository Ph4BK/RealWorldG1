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
from posnav_fmm.planner import plan_snapshot
from posnav_fmm.errors import GoalUnreachable
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
    def __init__(self, config):
        super().__init__('g1_navigation')
        self.cfg = yaml.safe_load(Path(config).read_text())
        self.control = Navigation(self.plan, self.clear)
        self.goal = None
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
        self.create_service(Trigger, '/g1/navigation/start', self.start)
        self.create_service(Trigger, '/g1/navigation/stop', self.stop)
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

    def cancel(self, reason):
        if self.control.active:
            self.send((0., 0.))
        self.control.stop(reason)
        self.publish_status()

    def on_goal(self, msg):
        # Even a redraw of the same XY/yaw ends the old task.
        self.cancel('Goal redrawn; press Start Navigation')
        try:
            self.goal = planar(msg)[:2]
        except ValueError as exc:
            self.goal = None
            self.control.reason = str(exc)
        self.publish_status()

    def on_pose(self, msg):
        try:
            self.control.pose = planar(msg)
            # Deliberately no source stamp age, duplicate or rollback checks.
            self.control.pose_time = time.monotonic()
        except ValueError:
            self.control.pose_time = -math.inf

    def on_map(self, msg):
        try:
            stamp = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
            if msg.header.frame_id != 'map' or stamp == 0:
                self.control.map_time = -math.inf
                self.map_stamp = None
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
        except (ValueError, TypeError):
            self.control.map_time = -math.inf

    def cell(self, xy):
        col, row = np.floor((np.asarray(xy)-self.origin)/self.resolution).astype(int)
        return (row, col)

    def clear(self, start, end, limit):
        start, end = np.asarray(start), np.asarray(end)
        distance = np.linalg.norm(end-start)
        if distance > limit:
            end = start + (end-start)*(limit/distance)
        return line_is_collision_free(self.cell(start), self.cell(end), self.free)

    def plan(self, pose, goal):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'stg_macro'
        marker.id = 0
        marker.action = Marker.DELETE
        try:
            plan = plan_snapshot(self.occupancy, self.origin, self.resolution, pose, goal,
                inflation_cells=self.cfg['fmm_inflation_cells'], step_cells=self.cfg['fmm_step_cells'],
                max_stgs=self.cfg['fmm_max_stgs'], fallback_cells=self.cfg['fmm_fallback_cells'])
        except GoalUnreachable:
            self.stg_display.publish(marker)
            raise
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
            if self.goal is None:
                raise ValueError('Draw the goal arrow first')
            self.control.start(self.goal, time.monotonic())
            response.success = True
            response.message = 'Navigation started'
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
        self.publish_status()
        return response

    def stop(self, _, response):
        self.cancel('Stopped by user')
        response.success = True
        response.message = 'Navigation stopped'
        return response

    def tick(self):
        try:
            velocity = self.control.tick(time.monotonic())
            if velocity is not None:
                self.send(velocity)
        except Exception as exc:
            self.cancel('Navigation stopped: '+str(exc))
        self.publish_status()

    def publish_status(self):
        c = self.control
        status = json.dumps(dict(active=c.active, state=c.state, message=c.paused or c.reason,
                                 stg=c.stg))
        if status != self.last_status:
            self.status.publish(String(data=status))
            self.last_status = status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1]/'fastlio/config/live_bev.yaml')
    args = parser.parse_args()
    rclpy.init()
    node = NavigationNode(args.config)
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
