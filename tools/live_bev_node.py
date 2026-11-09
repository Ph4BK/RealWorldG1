#!/usr/bin/env python3
"""Live localization/BEV interface with an explicitly started navigation child."""
import argparse
from collections import deque
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String, ColorRGBA, Empty
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster
from live_bev_core import RollingBEV, checked_transform, initial_map_from_odom
from posnav_fmm.planner import plan_snapshot
from experiment_recorder import ExperimentSession


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_matrix(pose):
    q = pose.orientation
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    t[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return checked_transform(t)


def fill_pose(pose, t):
    pose.position.x, pose.position.y, pose.position.z = map(float, t[:3, 3])
    q = Rotation.from_matrix(t[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)


class Child:
    def __init__(self, command, log_path, relay_prefixes=()):
        self.relay_prefixes = relay_prefixes
        self.logger = logging.Logger(str(log_path))
        self.handler = RotatingFileHandler(log_path, maxBytes=2_000_000, backupCount=1)
        self.logger.addHandler(self.handler)
        self.proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, start_new_session=True)
        self.thread = threading.Thread(target=self.drain, daemon=True)
        self.thread.start()

    def drain(self):
        for line in self.proc.stdout:
            self.logger.warning(line.rstrip())
            if self.relay_prefixes and line.startswith(self.relay_prefixes):
                print(line.rstrip(), flush=True)
        self.proc.stdout.close()

    def close(self, timeout=5):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGTERM)
                try:
                    self.proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                    self.proc.wait(timeout=2)
        self.thread.join(timeout=2)
        self.handler.close()


class LiveBEV(Node):
    def __init__(self, args):
        super().__init__('g1_live_bev')
        self.args = args
        self.experiment = getattr(args, 'experiment', None)
        self.cfg = yaml.safe_load(args.config.read_text())
        self.map_config = json.loads((args.map_dir / 'bev_config.json').read_text())
        with np.load(args.map_dir / 'bev_occupancy.npz') as d:
            self.core = RollingBEV(d['occ'], d['origin'], float(d['res']),
                window=self.cfg['window_frames'], min_points=self.cfg['min_points'],
                z_min=self.cfg['z_min'], z_max=self.cfg['z_max'],
                radius=self.cfg['update_radius_m'],
                inner_radius=self.cfg['update_inner_radius_m'],
                max_window_age=self.cfg['max_window_age_s'])
        assert np.allclose(self.core.origin, self.map_config['origin_xy_m'])
        assert self.core.res == self.map_config['resolution_m']
        self.z = float(yaml.safe_load((args.map_dir / 'metadata.yaml').read_text())['initial_imu_xyz_map'][2])
        self.imu, self.odom, self.pending = deque(maxlen=1000), deque(maxlen=100), deque(maxlen=10)
        self.received = dict(imu=0., odom=0., cloud=0., registration=0.)
        self.selected = None
        self.goal = None
        self.plan = None
        self.plan_message = 'FMM + STG: not calculated'
        self.selection_odom = None
        self.transform = None
        self.registration_stamp = -math.inf
        self.pending_registrations = deque(maxlen=10)
        self.registration_deferred = 0
        self.registration_wait_timeouts = 0
        self.fitness = 0.
        self.accepted = False
        self.child = None
        self.active = False
        self.initial_clear_pending = False
        self.message = 'Keep G1 stationary; wait for live sensors, then draw an arrow.'
        self.error = ''
        self.registration_error = ''
        self.notice = ''
        self.notice_until = 0.
        self.updates = 0
        self.map_update_stamp = None
        self.last_status = 0.
        self.last_graph_check = 0.
        self.graph_error = ''
        self.display_dirty = False
        self.last_display = -math.inf
        display_hz = float(self.cfg['map_display_hz'])
        self.registration_wait = float(self.cfg['registration_odom_wait_s'])
        if (not np.isfinite(display_hz) or display_hz <= 0 or
                not np.isfinite(self.registration_wait) or self.registration_wait <= 0):
            raise ValueError('Display rate and registration wait must be positive and finite')
        self.display_period = 1. / display_hz
        self.static_tf = StaticTransformBroadcaster(self)
        # Localization uses base_link and motion_link as aliases of the IMU
        # reference here, exactly as the offline pipeline; no body calibration implied.
        transforms = []
        for parent, child in [('odom', self.cfg['odom_frame']), ('base_link', 'imu_link'), ('base_link', 'motion_link')]:
            if parent == child:
                continue
            t = TransformStamped(); t.header.frame_id = parent; t.child_frame_id = child
            t.transform.rotation.w = 1.; transforms.append(t)
        self.static_tf.sendTransform(transforms)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.maps = {name: self.create_publisher(OccupancyGrid, '/g1/bev/' + name, latched)
                     for name in ['static', 'local', 'task']}
        self.cell_points = [Point(x=float(x),y=float(y),z=.004)
                            for x,y in zip(self.core.x.ravel(),self.core.y.ravel())]
        self.full_display = {name:self.create_publisher(MarkerArray, '/g1/bev/'+name+'_display', latched)
                             for name in ['static','task']}
        self.local_display = self.create_publisher(Marker, '/g1/bev/local_display', latched)
        self.status = self.create_publisher(String, '/g1/live_bev/status', latched)
        self.nav_cancel = self.create_publisher(Empty, '/g1/navigation/cancel', 10)
        self.preview = self.create_publisher(PoseStamped, '/g1/live_bev/initial_arrow', latched)
        self.robot = self.create_publisher(PoseStamped, '/g1/live_bev/robot', 5)
        # Humble PoseDisplay can segfault on disable with queued pose messages.
        # Preserve PoseStamped APIs and use independent Marker displays in RViz.
        self.preview_display = self.create_publisher(Marker, '/g1/live_bev/initial_arrow_display', latched)
        self.robot_display = self.create_publisher(Marker, '/g1/live_bev/robot_display', latched)
        self.goal_display = self.create_publisher(Marker, '/g1/live_bev/goal_display', latched)
        self.stg_display = self.create_publisher(Marker, '/g1/live_bev/stg_macro', latched)
        self.boundary = self.create_publisher(Marker, '/g1/live_bev/update_region', latched)
        self.create_subscription(Imu, self.cfg['imu_topic'], self.on_imu, sensor)
        self.create_subscription(Odometry, self.cfg['odom_topic'], self.on_odom, sensor)
        self.create_subscription(PointCloud2, self.cfg['cloud_topic'], self.on_cloud, sensor)
        self.create_subscription(String, '/g1/localization_update', self.on_registration, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/g1/live_bev/initialpose_2d', self.on_arrow, 10)
        self.create_subscription(PoseStamped, '/g1/navigation/goal', self.on_goal, latched)
        self.create_service(Trigger, '/g1/live_bev/start', self.start)
        self.create_service(Trigger, '/g1/live_bev/stop', self.stop)
        self.create_service(Trigger, '/g1/live_bev/reset', self.reset_map)
        self.create_service(Trigger, '/g1/live_bev/compute_stg', self.compute_stg)
        self.create_timer(.05, self.tick)
        self.publish_maps(static=True)

    def on_imu(self, m):
        stamp = stamp_s(m.header.stamp)
        if self.imu and stamp <= self.imu[-1][0]:
            self.imu.clear()
        a, g = m.linear_acceleration, m.angular_velocity
        self.imu.append([stamp, a.x, a.y, a.z, g.x, g.y, g.z])
        while self.imu and stamp - self.imu[0][0] > self.cfg['imu_seconds']:
            self.imu.popleft()
        self.received['imu'] = time.monotonic()

    def on_odom(self, m):
        if m.header.frame_id != self.cfg['odom_frame']:
            self.error = f'Odometry frame {m.header.frame_id!r} != {self.cfg["odom_frame"]!r}'
            return
        try:
            pose = pose_matrix(m.pose.pose)
        except ValueError:
            self.error = 'Invalid odometry pose'; return
        stamp = stamp_s(m.header.stamp)
        if self.odom and stamp <= self.odom[-1][0]:
            if stamp < self.odom[-1][0]:
                self.odom.clear(); self.pending.clear(); self.core.reset()
                self.map_update_stamp = None
                self.accepted = False; self.registration_stamp = -math.inf
                self.pending_registrations.clear()
                self.selected = None; self.active = False
                self.error = 'Odometry clock reset: stop and redraw the initial pose.'
                self.clear_plan()
                self.publish_maps()
            return
        self.odom.append((stamp, pose))
        self.received['odom'] = time.monotonic()
        self.retry_registrations()

    def on_cloud(self, m):
        if m.header.frame_id != self.cfg['odom_frame']:
            self.error = f'Cloud frame {m.header.frame_id!r} != {self.cfg["odom_frame"]!r}'; return
        self.received['cloud'] = time.monotonic()
        if self.active:
            self.pending.append((time.monotonic(), m))

    def quiet_seed(self, x, y, yaw):
        self.require_sensors()
        if not self.odom or not self.imu:
            raise ValueError('Waiting for live IMU and odometry')
        stamp, pose = self.odom[-1]
        if abs(stamp - self.imu[-1][0]) > .15:
            raise ValueError('IMU/odometry timestamps do not match')
        recent = [t for s, t in self.odom if stamp - s <= 2.]
        if len(recent) < 10:
            raise ValueError('Waiting for a stationary odometry window')
        movement = max(np.linalg.norm(t[:3, 3] - pose[:3, 3]) for t in recent)
        turn = max(Rotation.from_matrix(t[:3, :3].T @ pose[:3, :3]).magnitude() for t in recent)
        if movement > .08 or turn > math.radians(5):
            raise ValueError('Keep G1 stationary while selecting and starting')
        return initial_map_from_odom(x, y, yaw, self.z, list(self.imu), pose), pose.copy()

    def require_sensors(self):
        now = time.monotonic()
        for key in ['imu', 'odom', 'cloud']:
            if now - self.received[key] > self.cfg['sensor_timeout_s']:
                raise ValueError(f'Waiting for fresh {key} data from G1')
        for topic in [self.cfg['cloud_topic'], self.cfg['odom_topic']]:
            count = self.count_publishers(topic)
            if count != 1:
                raise ValueError(f'{topic}: expected one publisher, found {count}')

    def on_goal(self, m):
        # Display-only state from navigation (manual or Action), never a command.
        try:
            if m.header.frame_id != 'map':
                raise ValueError('Draw the goal in frame map')
            t = pose_matrix(m.pose)
            yaw = math.atan2(t[1, 0], t[0, 0])
            goal = dict(x=float(t[0, 3]), y=float(t[1, 3]), yaw=yaw)
            path = self.args.run_dir/'goal_pose.json'
            temporary = path.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(goal, indent=2)+'\n')
            temporary.replace(path)
            if self.goal is None or (self.goal['x'],self.goal['y']) != (goal['x'],goal['y']):
                # A delayed display update must not delete an active navigation STG.
                # Manual redraw clears that marker in the navigation input callback.
                self.plan = None
                self.plan_message = 'Goal changed; click Compute FMM + STG macro'
            self.goal = goal
            arrow = PoseStamped(); arrow.header = m.header
            arrow.pose.position.x = goal['x']; arrow.pose.position.y = goal['y']
            arrow.pose.position.z = .07
            arrow.pose.orientation.z = math.sin(yaw/2); arrow.pose.orientation.w = math.cos(yaw/2)
            self.goal_display.publish(self.arrow_marker(arrow, 'posnav_goal', (1., 0., 0.)))
        except (ValueError, OSError) as exc:
            self.notice = f'Goal not recorded: {exc}'
            self.notice_until = time.monotonic()+5.
        self.publish_status()

    def clear_plan(self, message='FMM + STG: not calculated'):
        self.plan = None; self.plan_message = message
        marker=Marker(); marker.header.frame_id='map'; marker.ns='stg_macro'; marker.id=0
        marker.action=Marker.DELETE; self.stg_display.publish(marker)

    def compute_stg(self, request, response):
        self.clear_plan()
        try:
            if self.goal is None:raise ValueError('Draw the red goal arrow first')
            if not self.active or not self.odom:raise ValueError('Start localization before computing STG')
            problem=self.health()
            if problem:raise ValueError(problem)
            # Single-threaded ROS callback: map, goal and pose are a consistent snapshot.
            robot=self.transform @ self.odom[-1][1]
            self.plan=plan_snapshot(self.core.task.copy(),self.core.origin,self.core.res,
                robot[:2,3],(self.goal['x'],self.goal['y']),
                inflation_cells=self.cfg['fmm_inflation_cells'], step_cells=self.cfg['fmm_step_cells'],
                max_stgs=self.cfg['fmm_max_stgs'], min_step_cells=self.cfg['fmm_min_step_cells'])
            if self.plan.stg_xy is not None:
                marker=Marker();marker.header.frame_id='map';marker.header.stamp=self.get_clock().now().to_msg()
                marker.ns='stg_macro';marker.id=0;marker.type=Marker.SPHERE;marker.action=Marker.ADD
                marker.pose.orientation.w=1.;marker.pose.position.x,marker.pose.position.y=self.plan.stg_xy
                marker.pose.position.z=.10
                marker.scale.x=marker.scale.y=marker.scale.z=.15
                marker.color=ColorRGBA(r=1.,g=0.,b=1.,a=1.)
                self.stg_display.publish(marker)
                self.plan_message=f'STG snapshot: x={self.plan.stg_xy[0]:.3f}, y={self.plan.stg_xy[1]:.3f} m'
            else:self.plan_message=self.plan.reason+'; no STG point'
            response.success=True
        except (ValueError, RuntimeError, ImportError, AssertionError) as exc:
            self.clear_plan(f'FMM + STG: {exc}')
            response.success=False
        response.message=self.plan_message
        self.publish_status()
        return response

    def on_arrow(self, m):
        if self.experiment:
            self.experiment.event('initial_arrow_request', frame_id=m.header.frame_id,
                source_stamp_ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec)
        if self.child is not None:
            # A rejected UI action is not a localization/sensor failure.
            self.notice = 'Initial pose unchanged. Stop localization before drawing a new arrow.'
            self.notice_until = time.monotonic() + 5.
            self.publish_status()
            if self.experiment:
                self.experiment.event('initial_arrow_rejected', reason=self.notice)
            return
        try:
            if m.header.frame_id != 'map':
                raise ValueError('Draw the arrow in frame map')
            pose = pose_matrix(m.pose.pose)
            yaw = math.atan2(pose[1, 0], pose[0, 0])
            x, y = pose[:2, 3]
            _, self.selection_odom = self.quiet_seed(x, y, yaw)
            self.selected = dict(x=float(x), y=float(y), yaw_deg=math.degrees(yaw), z=self.z)
            arrow = PoseStamped(); arrow.header = m.header; arrow.pose = m.pose.pose
            arrow.pose.position.z = .03; self.preview.publish(arrow)
            self.preview_display.publish(self.arrow_marker(arrow, 'initial_arrow', (0., 200./255, 0.)))
            self.message = 'Arrow selected (IMU reference). Keep still and press Start.'
            self.error = ''
        except ValueError as exc:
            self.error = str(exc)
        if self.experiment:
            self.experiment.event('initial_arrow_result', accepted=not bool(self.error),
                                  selected=self.selected, error=self.error)
        self.publish_status()

    def start(self, request, response):
        if self.child is not None:
            response.success = False
            response.message = 'Localization is already started; stop it first'
            if self.experiment:
                self.experiment.event('localization_start_request', accepted=False, message=response.message)
            return response
        try:
            if self.selected is None:
                raise ValueError('Draw a valid initial-pose arrow first')
            if self.count_publishers('/g1/localization_update') or self.count_publishers('/odom2map'):
                raise ValueError('Another localization node is running; stop that instance first')
            p = self.selected
            seed, pose = self.quiet_seed(p['x'], p['y'], math.radians(p['yaw_deg']))
            delta = np.linalg.inv(self.selection_odom) @ pose
            if np.linalg.norm(delta[:3, 3]) > .05 or Rotation.from_matrix(delta[:3, :3]).magnitude() > math.radians(3):
                raise ValueError('G1 moved after the arrow was drawn; redraw at its current pose')
            params = dict(use_sim_time=False, path_map=str((self.args.map_dir/'map.pcd').resolve()),
                initialpose=[*map(float, seed[:3, 3]), *Rotation.from_matrix(seed[:3, :3]).as_euler('XYZ', degrees=True).tolist()],
                pcd_queue_maxsize=10, voxelsize_coarse=.1, voxelsize_fine=.1,
                threshold_fitness=.5, threshold_fitness_init=.5, loc_frequence=2.5,
                maxpoints_source=80000, maxpoints_target=400000, filter_odom2map=False,
                confidence_loc_th=.7, dis_updatemap=3.5, save_scan=False,
                **{'kf_baselink2map/x':[.001,.002], 'kf_baselink2map/y':[.001,.005], 'kf_baselink2map/z':[.00001,.04]})
            config = self.args.run_dir/'localization.yaml'
            config.write_text(yaml.safe_dump({'global_localization_node':{'ros__parameters':params}},sort_keys=False))
            (self.args.run_dir/'initial_pose.json').write_text(json.dumps(dict(**p, T_map_from_odom=seed.tolist()),indent=2))
            if self.experiment:
                self.experiment.initialization(dict(**p, T_map_from_odom=seed.tolist()), params)
            command = ['ros2','run','open3d_loc','global_localization_node','--ros-args',
                '-r','__node:=global_localization_node','--params-file',str(config),
                '-r',f'/cloud_registered_1:={self.cfg["cloud_topic"]}',
                '-r',f'/Odometry_loc:={self.cfg["odom_topic"]}',
                '--disable-external-lib-logs','--disable-rosout-logs']
            self.child = Child(command, self.args.run_dir/'localization.log')
            self.active = True; self.transform = None; self.accepted = False
            self.registration_stamp = -math.inf; self.core.reset(); self.pending.clear(); self.updates = 0
            self.initial_clear_pending = True
            self.map_update_stamp = None
            self.pending_registrations.clear()
            self.registration_deferred = self.registration_wait_timeouts = 0
            self.clear_plan()
            self.error = ''; self.registration_error = ''; self.message = 'Waiting for an accepted, fresh ICP registration.'
            self.publish_maps()
            response.success = True; response.message = self.message
        except (ValueError, OSError) as exc:
            response.success = False; response.message = str(exc); self.error = str(exc)
        if self.experiment:
            self.experiment.event('localization_start_request', accepted=response.success, message=response.message)
        self.publish_status()
        return response

    def stop(self, request, response):
        self.nav_cancel.publish(Empty())
        if self.experiment:
            self.experiment.event('localization_stop_request')
        self.map_update_stamp = None
        self.clear_plan()
        self.notice = ''; self.notice_until = 0.
        self.active = False
        self.initial_clear_pending = False
        self.pending_registrations.clear()
        if self.child is not None:
            self.child.close(); self.child = None
        self.accepted = False; self.pending.clear(); self.core.pause()
        self.selected = None; self.selection_odom = None
        self.error = ''; self.registration_error = ''; self.message = 'Stopped. Draw a new arrow to start a new session.'
        self.publish_maps(); self.publish_status()
        response.success = True; response.message = self.message
        return response

    def reset_map(self, request, response):
        self.nav_cancel.publish(Empty())
        if self.experiment:
            self.experiment.event('static_map_reset_request')
        self.map_update_stamp = None
        self.clear_plan()
        self.core.reset(); self.pending.clear(); self.updates = 0
        self.initial_clear_pending = False
        self.publish_maps()
        response.success = True; response.message = f'Static map restored; collecting {self.core.window} new frames.'
        return response

    def on_registration(self, msg):
        if not self.active:
            return
        try:
            data = json.loads(msg.data)
            stamp, fitness = float(data['stamp']), float(data['fitness'])
            t = checked_transform(data['transform'])
            if not np.isfinite([stamp, fitness]).all() or not 0 <= fitness <= 1:
                raise ValueError('Invalid ICP result')
            if stamp <= self.registration_stamp:
                return  # Republishing an old fit must not extend freshness.
            if any(item[1] == stamp for item in self.pending_registrations):
                return  # A duplicate must not renew the original waiting deadline.
            received = time.monotonic()
            if not self.odom or stamp > self.odom[-1][0] + .2:
                if len(self.pending_registrations) == self.pending_registrations.maxlen:
                    raise ValueError('Registration waiting queue is full')
                self.pending_registrations.append((received, stamp, fitness, t, data['accepted']))
                self.registration_deferred += 1
                return  # Keep the previous fit subject to its existing health limits.
            self.apply_registration(received, stamp, fitness, t, data['accepted'])
        except (ValueError, KeyError, TypeError) as exc:
            self.reject_registration(str(exc))

    def reject_registration(self, reason):
        self.accepted = False
        self.registration_error = f'Invalid localization update: {reason}'
        self.core.pause(); self.pending.clear(); self.publish_maps()

    def retry_registrations(self):
        if not self.active:
            return
        now = time.monotonic()
        waiting = list(self.pending_registrations)
        self.pending_registrations.clear()
        for received, stamp, fitness, transform, accepted in waiting:
            if stamp <= self.registration_stamp:
                continue
            if now - received >= self.registration_wait:
                self.registration_wait_timeouts += 1
                self.reject_registration('Registration timestamp does not match the odometry clock '
                                         f'after waiting {self.registration_wait:.2f} s')
            elif self.odom and stamp <= self.odom[-1][0] + .2:
                self.apply_registration(received, stamp, fitness, transform, accepted)
            else:
                self.pending_registrations.append((received, stamp, fitness, transform, accepted))

    def apply_registration(self, received, stamp, fitness, transform, accepted):
        self.registration_error = ''
        self.registration_stamp = stamp
        # Waiting must not make an old result look freshly received.
        self.received['registration'] = received
        self.fitness = fitness
        self.accepted = accepted is True and fitness >= self.cfg['registration_min_fitness']
        if self.transform is not None and self.accepted:
            delta = np.linalg.inv(self.transform) @ transform
            if (np.linalg.norm(delta[:3, 3]) > self.cfg['transform_jump_translation_m'] or
                    Rotation.from_matrix(delta[:3, :3]).magnitude() > math.radians(self.cfg['transform_jump_rotation_deg'])):
                self.map_update_stamp = None
                self.core.reset(); self.pending.clear(); self.clear_plan(); self.publish_maps()
        if self.accepted:
            self.transform = transform
        else:
            self.core.pause(); self.pending.clear(); self.publish_maps()

    def health(self):
        now = time.monotonic()
        if self.error:
            return self.error
        if self.registration_error:
            return self.registration_error
        if now - self.last_graph_check > 1.:
            self.last_graph_check = now
            self.graph_error = ''
            for topic in [self.cfg['cloud_topic'], self.cfg['odom_topic']]:
                if self.count_publishers(topic) != 1:
                    self.graph_error = f'Expected exactly one publisher on {topic}'
        if self.graph_error:
            return self.graph_error
        for key in ['cloud', 'odom']:
            if now - self.received[key] > self.cfg['sensor_timeout_s']:
                return f'Paused: {key} data is stale'
        if not self.accepted or self.transform is None:
            return f'Waiting for accepted localization (fitness={self.fitness:.3f})'
        if (now - self.received['registration'] > self.cfg['registration_timeout_s'] or
                self.odom[-1][0] - self.registration_stamp > self.cfg['registration_timeout_s']):
            return 'Paused: no fresh ICP registration'
        return ''

    def tick(self):
        if self.child is not None and self.child.proc.poll() is not None:
            if self.active:
                self.core.pause(); self.pending.clear(); self.publish_maps()
            self.active = False; self.error = f'Localization exited ({self.child.proc.returncode}); press Stop and inspect the log.'
            self.pending_registrations.clear()
        if self.active:
            self.retry_registrations()
            problem = self.health()
            if problem:
                if self.core.frames or self.core.mask.any():
                    self.core.pause(); self.publish_maps()
                self.pending.clear(); self.message = problem
            else:
                while self.pending:
                    received, cloud = self.pending[0]
                    stamp = stamp_s(cloud.header.stamp) + self.cfg['cloud_stamp_offset_s']
                    odom_stamp, pose = min(self.odom, key=lambda v: abs(v[0] - stamp))
                    if abs(odom_stamp - stamp) > self.cfg['pose_sync_tolerance_s']:
                        if time.monotonic() - received < .3:
                            break
                        self.pending.popleft(); self.core.pause(); self.publish_maps()
                        self.message = 'Paused: cloud/odometry timestamps do not match; check cloud_stamp_offset_s.'
                        continue
                    self.pending.popleft()
                    # Reject delayed/backlogged observations relative to the sensor clock.
                    if self.odom[-1][0] - stamp > self.cfg['sensor_timeout_s']:
                        self.core.pause(); self.publish_maps(); continue
                    try:
                        points = point_cloud2.read_points_numpy(cloud, field_names=('x','y','z'), skip_nans=True)
                        points = np.asarray(points).reshape(-1,3)
                        mapped = points @ self.transform[:3,:3].T + self.transform[:3,3]
                        robot = self.transform @ pose
                        yaw = math.atan2(robot[1,0], robot[0,0])
                        updated = self.core.add(stamp, mapped, [robot[0,3], robot[1,3], yaw])
                        initial_cleared = self.initial_clear_pending
                        if initial_cleared:
                            self.core.clear_initial_region(robot[0,3], robot[1,3])
                            self.initial_clear_pending = False
                        self.publish_pose(robot, cloud.header.stamp)
                        if updated:
                            self.map_update_stamp = self.get_clock().now().to_msg()
                            self.updates += 1; self.publish_maps(force_display=False)
                            self.message = f'Updating; {self.updates} updates; fitness={self.fitness:.3f}'
                        else:
                            if initial_cleared or not self.core.frames:
                                self.publish_maps()
                            self.message = f'Collecting valid frames: {len(self.core.frames)}/{self.core.window}'
                    except (ValueError, AssertionError, KeyError) as exc:
                        self.core.pause(); self.publish_maps(); self.message = f'Paused: invalid cloud: {exc}'
        self.publish_displays()
        if time.monotonic() - self.last_status > .5:
            self.publish_status()

    def grid(self, data):
        msg = OccupancyGrid(); msg.header.frame_id = 'map'; msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.core.res
        msg.info.height, msg.info.width = self.core.static.shape
        msg.info.origin.position.x, msg.info.origin.position.y = map(float,self.core.origin)
        msg.info.origin.orientation.w = 1.
        msg.data = data.astype(np.int8).ravel().tolist()
        return msg

    def publish_maps(self, static=False, force_display=True):
        if static:
            self.maps['static'].publish(self.grid(self.core.static.astype(np.int8)*100))
            self.publish_full_display('static', self.core.static)
        task = self.grid(self.core.task.astype(np.int8)*100)
        # Only successful fusion advances this stamp. Republishing a retained
        # grid during a pause must not make it fresh for navigation.
        if self.map_update_stamp is None:
            task.header.stamp.sec = task.header.stamp.nanosec = 0
        else:
            task.header.stamp = self.map_update_stamp
        self.maps['task'].publish(task)
        self.maps['local'].publish(self.grid(np.where(self.core.mask, self.core.local*100, -1)))
        self.display_dirty = True
        if force_display:
            self.publish_displays(force=True)

    def publish_displays(self, force=False):
        if not self.display_dirty or (not force and time.monotonic() - self.last_display < self.display_period):
            return
        self.publish_full_display('task', self.core.task)

        # OccupancyGrid -1 is usually gray in RViz, not transparent. Render only
        # selected cells as black/white tiles, so the exterior is truly absent.
        marker = Marker(); marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'local_binary'; marker.id = 0; marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD; marker.pose.orientation.w = 1.
        marker.scale.x = self.core.res; marker.scale.y = self.core.res; marker.scale.z = .002
        marker.color.a = 1.
        rows, cols = np.where(self.core.mask)
        for row, col in zip(rows, cols):
            marker.points.append(Point(x=float(self.core.x[row,col]), y=float(self.core.y[row,col]), z=.01))
            value = 0. if self.core.local[row,col] else 1.
            marker.colors.append(ColorRGBA(r=value,g=value,b=value,a=1.))
        if not marker.points: marker.action = Marker.DELETE
        self.local_display.publish(marker)
        self.display_dirty = False
        self.last_display = time.monotonic()

    def publish_full_display(self, name, occupied):
        # Marker rendering avoids the indexed-texture shader failure seen with
        # the RViz Map display on this WSL software-OpenGL environment.
        background = Marker(); background.header.frame_id='map'
        background.header.stamp=self.get_clock().now().to_msg()
        background.ns=name; background.id=0; background.type=Marker.CUBE
        background.action=Marker.ADD; background.pose.orientation.w=1.
        height,width=occupied.shape
        background.pose.position.x=float(self.core.origin[0]+width*self.core.res/2)
        background.pose.position.y=float(self.core.origin[1]+height*self.core.res/2)
        background.scale.x=width*self.core.res; background.scale.y=height*self.core.res; background.scale.z=.002
        background.color=ColorRGBA(r=1.,g=1.,b=1.,a=1.)
        obstacles=Marker(); obstacles.header=background.header; obstacles.ns=name; obstacles.id=1
        obstacles.type=Marker.CUBE_LIST; obstacles.action=Marker.ADD; obstacles.pose.orientation.w=1.
        obstacles.scale.x=self.core.res; obstacles.scale.y=self.core.res; obstacles.scale.z=.002
        obstacles.color=ColorRGBA(r=0.,g=0.,b=0.,a=1.)
        obstacles.points=[self.cell_points[i] for i in np.flatnonzero(occupied)]
        if not obstacles.points: obstacles.action=Marker.DELETE
        self.full_display[name].publish(MarkerArray(markers=[background,obstacles]))

    @staticmethod
    def arrow_marker(pose, namespace, color):
        marker = Marker(); marker.header = pose.header; marker.pose = pose.pose
        marker.ns = namespace; marker.id = 0; marker.type = Marker.ARROW; marker.action = Marker.ADD
        marker.scale.x = .7; marker.scale.y = .05; marker.scale.z = .14
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.)
        return marker

    def publish_pose(self, transform, stamp):
        pose = PoseStamped(); pose.header.frame_id='map'; pose.header.stamp=stamp
        fill_pose(pose.pose, transform); pose.pose.position.z=.05; self.robot.publish(pose)
        self.robot_display.publish(self.arrow_marker(pose, 'robot', (0., 100./255, 1.)))
        marker = Marker(); marker.header=pose.header; marker.ns='update_region'; marker.id=0
        marker.type=Marker.LINE_LIST; marker.action=Marker.ADD; marker.pose=pose.pose
        marker.pose.orientation.x=0.; marker.pose.orientation.y=0.
        marker.pose.orientation.z=0.; marker.pose.orientation.w=1.
        marker.scale.x=.025; marker.color.r=1.; marker.color.g=.55; marker.color.a=1.
        angles=np.linspace(0.,2*math.pi,100)
        for radius in (self.core.inner_radius, self.core.radius):
            ring=[Point(x=float(radius*np.cos(a)), y=float(radius*np.sin(a)), z=.02) for a in angles]
            for start,end in zip(ring[:-1],ring[1:]):
                marker.points.extend([start,end])
        self.boundary.publish(marker)

    def publish_status(self):
        self.last_status=time.monotonic()
        message = self.error or self.registration_error or self.message
        if self.last_status < self.notice_until:
            message += '\n' + self.notice
        self.status.publish(String(data=json.dumps(dict(pose=self.selected,goal=self.goal,running=self.child is not None,
            plan_message=self.plan_message,
            pending_registrations=len(self.pending_registrations),
            registration_deferred=self.registration_deferred,
            registration_wait_timeouts=self.registration_wait_timeouts,
            message=message,frames=len(self.core.frames),window_frames=self.core.window,updates=self.updates,fitness=self.fitness))))


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir',type=Path,default=root/'maps/full_lab_v1')
    parser.add_argument('--config',type=Path,default=root/'fastlio/config/live_bev.yaml')
    parser.add_argument('--run-dir',type=Path,default=root/'.cache/live_bev')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--record-stg-errors',action='store_true')
    parser.add_argument('--record-experiment',action='store_true',
                        help='Save a self-contained timestamped experiment bundle on G1')
    args=parser.parse_args(); args.run_dir.mkdir(parents=True,exist_ok=True)
    experiment = None
    if args.record_experiment:
        experiment = ExperimentSession(root, args.map_dir, args.config)
        args.experiment = experiment
        # Load the captured map/config so the evidence matches actual runtime inputs.
        args.map_dir = experiment.path/'assets/map'
        args.config = experiment.path/'assets/config/live_bev.yaml'
        args.run_dir = experiment.path/'component_logs'
        print('EXPERIMENT_DIR: '+str(experiment.path), flush=True)
    rclpy.init(); node=None; rviz=None; navigation=None
    clean = False
    failure = None
    try:
        node=LiveBEV(args)
        navigation=Child([sys.executable,str(root/'tools/g1_navigation_node.py'),
                          '--config',str(args.config)]+
                         (['--record-stg-errors'] if args.record_stg_errors else [])+
                         (['--experiment-dir',str(experiment.path)] if experiment else []),
                         args.run_dir/'navigation.log')
        if not args.headless:
            rviz=Child(['rviz2','-d',str(root/'fastlio/rviz/live_bev.rviz'),'--ros-args','-p','use_sim_time:=false',
                        '--disable-external-lib-logs','--disable-rosout-logs'],args.run_dir/'rviz.log')
        print('READY: live BEV UI; navigation stays idle until Start Navigation.',flush=True)
        while rclpy.ok() and (rviz is None or rviz.proc.poll() is None):
            rclpy.spin_once(node,timeout_sec=.1)
            if navigation.proc.poll() is not None:
                raise RuntimeError('Navigation process exited; inspect navigation.log')
            if experiment and experiment.writer.error:
                raise RuntimeError('Experiment recording failed: '+experiment.writer.error)
        if rviz is not None and rviz.proc.poll() not in (None, 0):
            raise RuntimeError('RViz exited with an error; inspect rviz.log')
        clean = True
    except (KeyboardInterrupt,ExternalShutdownException):
        clean = True
    except Exception as exc:
        failure = str(exc)
        raise
    finally:
        if navigation is not None: navigation.close(timeout=20 if experiment else 5)
        if node is not None and node.child is not None: node.child.close()
        if rviz is not None: rviz.close()
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if experiment:
            experiment.close(clean=clean, error=failure)
            print('EXPERIMENT_SAVED: '+str(experiment.path), flush=True)


if __name__=='__main__':
    main()
