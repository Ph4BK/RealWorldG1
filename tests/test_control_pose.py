"""Real node inputs on isolated loopback ROS; command publishers are intercepted."""
from copy import deepcopy
import json
import math
import os
import statistics
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from g1_control_pose import LatestControlPose, monotonic_clock_id
from g1_navigation_node import NavigationNode
from live_bev_node import LiveBEV


def reference(now=10., **updates):
    data = dict(schema_version=1, clock_id=monotonic_clock_id(), session_id='session-one',
        map_epoch=0, revision=1, sent_monotonic=now, valid_until_monotonic=now+1.,
        valid=True, reason='', transform=np.eye(4).tolist(), registration_stamp=100.,
        latest_odom_stamp=100., odom_received_monotonic=now, map_stamp_ns=1)
    data.update(updates)
    return data


class ControlSourceTest(unittest.TestCase):
    def setUp(self):
        self.source = LatestControlPose(.3, 6.)
        self.source.set_reference(reference(), 10.)
        self.source.set_odometry(100., np.eye(4), 10.)

    def test_full_rigid_transform_preserves_roll_pitch_and_reference(self):
        a = np.eye(4); a[:3, :3] = Rotation.from_euler('xyz', [3., .1, .4]).as_matrix()
        a[:3, 3] = [1., 2., 1.24]
        b = np.eye(4); b[:3, :3] = Rotation.from_euler('xyz', [.02, -.04, .3]).as_matrix()
        b[:3, 3] = [.1, .2, .3]
        self.source.set_reference(reference(now=10.01, transform=a.tolist(), revision=2), 10.01)
        self.source.set_odometry(100.02, b, 10.02)
        np.testing.assert_allclose(self.source.snapshot(10.02)['transform'], a @ b)

    def test_transform_change_without_revision_is_rejected(self):
        changed = np.eye(4); changed[0, 3] = .01
        self.assertFalse(self.source.set_reference(reference(now=10.01, transform=changed.tolist()), 10.01))
        self.assertIn('revision', self.source.snapshot(10.02)['reason'])

    def test_duplicate_source_does_not_refresh_time(self):
        self.assertFalse(self.source.set_odometry(100., np.eye(4), 10.2))
        self.assertEqual(self.source.odom_received, 10.)
        self.assertFalse(self.source.snapshot(10.31)['valid'])

    def test_no_subtraction_of_unix_and_sensor_clocks(self):
        # Sensor and monotonic clocks differ by ~1.8 billion seconds.
        s = LatestControlPose(.3, 6.)
        s.set_reference(reference(latest_odom_stamp=1795257000., registration_stamp=1795257000.), 10.)
        s.set_odometry(1795257000., np.eye(4), 10.)
        self.assertTrue(s.snapshot(10.1)['valid'])

    def test_advancing_headers_cannot_hide_queue_delay(self):
        self.source.set_odometry(100.1, np.eye(4), 10.7)
        state = self.source.snapshot(10.7)
        self.assertFalse(state['valid'])
        self.assertIn('backlogged', state['reason'])

    def test_first_queued_frame_is_bounded_by_reference(self):
        s = LatestControlPose(.3, 6.)
        s.set_reference(reference(latest_odom_stamp=100.7), 10.)
        s.set_odometry(100., np.eye(4), 10.)
        self.assertFalse(s.snapshot(10.)['valid'])

    def test_reference_transport_delay_does_not_renew_deadline(self):
        s = LatestControlPose(.3, 6.)
        s.set_reference(reference(), 11.1)
        s.set_odometry(101.1, np.eye(4), 11.1)
        self.assertFalse(s.snapshot(11.1)['valid'])

    def test_duplicate_reference_does_not_renew_deadline(self):
        self.assertFalse(self.source.set_reference(reference(valid_until_monotonic=99.), 10.2))
        self.assertEqual(self.source.reference['valid_until_monotonic'], 11.)

    def test_invalid_localization_never_falls_back_to_old_transform(self):
        self.source.set_reference(reference(now=10.01, valid=False, reason='ICP rejected'), 10.01)
        self.assertFalse(self.source.snapshot(10.02)['valid'])
        self.assertEqual(self.source.snapshot(10.02)['reason'], 'ICP rejected')

    def test_registration_also_expires_in_sensor_time(self):
        self.source.set_odometry(107., np.eye(4), 10.1)
        self.assertIn('ICP', self.source.snapshot(10.1)['reason'])

    def test_bad_host_clock_and_future_packet_are_rejected(self):
        for updates in (dict(clock_id='another-host'), dict(sent_monotonic=11.),
                        dict(transform=np.zeros((4, 4)).tolist())):
            with self.subTest(updates=updates):
                s = LatestControlPose(.3, 6.)
                self.assertFalse(s.set_reference(reference(**updates), 10.))
                self.assertFalse(s.snapshot(10.)['valid'])

    def test_clock_reversal_requires_new_localization_session(self):
        self.source.set_odometry(99., np.eye(4), 10.1)
        self.assertFalse(self.source.snapshot(10.1)['valid'])
        self.source.set_reference(reference(now=10.2), 10.2)
        self.source.set_odometry(100.2, np.eye(4), 10.2)
        self.assertFalse(self.source.snapshot(10.2)['valid'])
        self.source.set_reference(reference(now=10.3, session_id='session-two'), 10.3)
        self.source.set_odometry(100.3, np.eye(4), 10.3)
        self.assertTrue(self.source.snapshot(10.3)['valid'])

    def test_new_session_recovers_even_after_malformed_reference(self):
        self.source.set_odometry(99., np.eye(4), 10.1)
        self.source.set_reference({}, 10.2)
        self.source.set_reference(reference(now=10.3, session_id='session-two'), 10.3)
        self.source.set_odometry(100.3, np.eye(4), 10.3)
        self.assertTrue(self.source.snapshot(10.3)['valid'])


class ControlNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID', '0') == '0' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Tests need a nonzero ROS domain on localhost only')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.nav = NavigationNode(ROOT/'fastlio/config/live_bev.yaml')
        self.commands = []
        self.command_patch = patch.object(self.nav.commands, 'publish', side_effect=self.commands.append)
        self.command_patch.start()
        self.now = time.monotonic()
        self.grid = OccupancyGrid(); self.grid.header.frame_id = 'map'
        self.grid.header.stamp = self.nav.get_clock().now().to_msg()
        self.grid.info.resolution = .05; self.grid.info.width = self.grid.info.height = 200
        self.grid.info.origin.position.x = self.grid.info.origin.position.y = -2.
        self.grid.data = [0]*40000
        self.nav.on_map(self.grid)

    def tearDown(self):
        self.nav.destroy_node(); self.command_patch.stop()

    def ref(self, now=None, **updates):
        now = self.now if now is None else now
        fields = dict(map_stamp_ns=self.nav.map_stamp)
        fields.update(updates)
        data = reference(now=now, **fields)
        with patch('g1_navigation_node.time.monotonic', return_value=now):
            self.nav.on_control_reference(String(data=json.dumps(data)))
        return data

    def odom(self, stamp=100., x=0., yaw=0., now=None, frame='camera_init'):
        msg = Odometry(); msg.header.frame_id = frame
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.orientation.z = math.sin(yaw/2); msg.pose.pose.orientation.w = math.cos(yaw/2)
        with patch('g1_navigation_node.time.monotonic', return_value=self.now if now is None else now):
            self.nav.on_control_odom(msg)
        return msg

    def test_production_subscription_no_longer_consumes_historical_robot_pose(self):
        topics = [s.topic_name for s in self.nav.subscriptions]
        self.assertIn('/Odometry_loc', topics)
        self.assertIn('/g1/live_bev/control_reference', topics)
        self.assertNotIn('/g1/live_bev/robot', topics)

    def test_raw_odometry_without_accepted_reference_cannot_start(self):
        self.odom()
        self.nav.goal_pose = (1., 0., 0.)
        self.assertFalse(self.nav.start(None, Trigger.Response()).success)

    def test_latest_odometry_updates_turn_stop_through_real_controller(self):
        self.ref(); self.odom(yaw=0.)
        c = self.nav.control
        c.active = True; c.state = 'turn'; c.stg = (math.cos(1.), math.sin(1.))
        c.turn_sign = 1; c.previous_error = c.turn_error = 1.
        self.odom(stamp=100.1, yaw=.95, now=self.now+.1)
        with patch('g1_navigation_node.time.monotonic', return_value=self.now+.1):
            self.nav.tick()
        self.assertEqual(c.state, 'turn_settle')
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., 0.])
        self.assertAlmostEqual(c.pose[2], .95)

    def test_control_pose_diagnostic_preserves_full_mapped_3d_pose(self):
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler('xyz', [math.pi, .1, .3]).as_matrix()
        transform[:3, 3] = [1., 2., 1.24]
        self.ref(transform=transform.tolist())
        with patch.object(self.nav.control_pose_output, 'publish') as output:
            self.odom()
        message = output.call_args.args[0]
        self.assertAlmostEqual(message.pose.position.z, 1.24)
        q = message.pose.orientation
        np.testing.assert_allclose(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), transform[:3, :3], atol=1e-12)

    def test_output_preserves_original_unix_nanoseconds_exactly(self):
        message = Odometry(); message.header.frame_id = 'camera_init'
        message.header.stamp.sec = 1795257194; message.header.stamp.nanosec = 123456789
        message.pose.pose.orientation.w = 1.
        t = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        self.ref(latest_odom_stamp=t, registration_stamp=t)
        with patch.object(self.nav.control_pose_output, 'publish') as output, \
             patch('g1_navigation_node.time.monotonic', return_value=self.now):
            self.nav.on_control_odom(message)
        self.assertEqual(output.call_args.args[0].header.stamp, message.header.stamp)
        self.assertEqual(self.nav.pose_source_stamp, 1795257194123456789)

    def test_latest_odometry_updates_straight_stop_through_real_controller(self):
        self.ref(); self.odom()
        c = self.nav.control
        c.active = True; c.state = 'walk'; c.stg = (1., 0.)
        c.origin = (0., 0.); c.direction = (1., 0.); c.goal = (3., 0.)
        c.final_goal = (3., 0., 0.)
        self.odom(stamp=100.1, x=.90, now=self.now+.1)
        with patch('g1_navigation_node.time.monotonic', return_value=self.now+.1):
            self.nav.tick()
        self.assertEqual(c.state, 'walk_settle')
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., 0.])

    def test_repeated_timer_and_duplicate_odom_do_not_make_pose_fresh(self):
        self.ref(); self.odom()
        original = self.nav.control.pose_time
        self.odom(now=self.now+.1)
        self.assertEqual(self.nav.control.pose_time, original)
        with patch('g1_navigation_node.time.monotonic', return_value=self.now+.31):
            self.nav.refresh_control_pose()
        self.assertEqual(self.nav.control.pose_time, -math.inf)

    def test_stale_map_is_not_bypassed_by_fresh_control_odometry(self):
        self.ref(); self.odom()
        self.nav.control.map_time -= 5.
        self.nav.goal_pose = (1., 0., 0.)
        self.assertFalse(self.nav.start(None, Trigger.Response()).success)

    def test_reset_epoch_cancels_active_task_and_requires_new_map(self):
        self.ref(); self.odom()
        self.nav.goal_pose = (1., 0., 0.)
        self.assertTrue(self.nav.start(None, Trigger.Response()).success)
        self.ref(now=self.now+.01, map_epoch=1, valid=False, reason='Map reset')
        self.assertFalse(self.nav.control.active)
        self.assertEqual(self.nav.control.map_time, -math.inf)
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., 0.])

    def test_session_switch_after_bad_packet_still_cancels_active_task(self):
        self.ref(); self.odom()
        self.nav.goal_pose = (1., 0., 0.)
        self.assertTrue(self.nav.start(None, Trigger.Response()).success)
        self.nav.on_control_reference(String(data='invalid json'))
        self.ref(now=self.now+.01, session_id='session-two')
        self.assertFalse(self.nav.control.active)

    def test_bad_frame_invalidates_control_pose(self):
        self.ref(); self.odom()
        self.odom(stamp=100.1, frame='wrong')
        self.assertEqual(self.nav.control.pose_time, -math.inf)

    def test_renewed_reference_does_not_refresh_original_odom_sample(self):
        self.ref(); self.odom()
        original = self.nav.control.pose_time
        self.ref(now=self.now+.2, revision=2)
        self.assertEqual(self.nav.control.pose_time, original)

    def test_selected_map_must_match_reference_generation(self):
        self.ref(map_stamp_ns=self.nav.map_stamp+1); self.odom()
        self.assertFalse(self.nav.control_pose_status['valid'])
        self.assertIn('matching', self.nav.control_pose_status['reason'])

    def test_real_dds_reference_and_raw_odom_reach_control_without_cloud_node(self):
        from rclpy.qos import QoSProfile, DurabilityPolicy
        peer = rclpy.create_node('control_pose_local_transport_test')
        ref_pub = peer.create_publisher(String, '/g1/live_bev/control_reference',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        odom_pub = peer.create_publisher(Odometry, '/Odometry_loc', 1)
        try:
            message = Odometry(); message.header.frame_id = 'camera_init'
            message.pose.pose.orientation.w = 1.; message.pose.pose.position.x = .7
            end = time.monotonic()+3.
            while self.nav.control.pose is None and time.monotonic() < end:
                now = time.monotonic()
                data = reference(now=now, map_stamp_ns=self.nav.map_stamp)
                ref_pub.publish(String(data=json.dumps(data)))
                sensor_time = 100.+now-self.now
                message.header.stamp.sec = int(sensor_time)
                message.header.stamp.nanosec = round((sensor_time-int(sensor_time))*1e9)
                odom_pub.publish(message)
                # Drain the reference and timer as well as odometry; flooding
                # the first subscription on every spin would starve the others.
                for _ in range(4):
                    rclpy.spin_once(self.nav, timeout_sec=.01)
                rclpy.spin_once(peer, timeout_sec=0.)
            self.assertIsNotNone(self.nav.control.pose, dict(
                status=self.nav.control_pose_status, error=self.nav.latest_control_pose.error,
                reference=self.nav.latest_control_pose.reference,
                odom_stamp=self.nav.latest_control_pose.odom_stamp))
            self.assertAlmostEqual(self.nav.control.pose[0], .7)
            self.assertTrue(self.nav.control_pose_status['valid'])
            self.assertFalse(self.commands)  # No task and no motion command.
        finally:
            peer.destroy_node()

    def test_real_cloud_chain_keeps_historical_pose_but_control_uses_latest(self):
        with tempfile.TemporaryDirectory() as directory:
            live = LiveBEV(SimpleNamespace(config=ROOT/'fastlio/config/live_bev.yaml',
                map_dir=ROOT/'maps/full_lab_v1', run_dir=Path(directory)))
            try:
                live.count_publishers = lambda _: 1
                live.active = live.accepted = True; live.transform = np.eye(4)
                live.registration_stamp = 99.; live.received['registration'] = time.monotonic()
                def odom(t, x=0., yaw=0.):
                    msg = self.odom(stamp=t, x=x, yaw=yaw, now=time.monotonic())
                    live.on_odom(msg)
                def cloud(t):
                    h = Header(frame_id='camera_init'); h.stamp.sec = int(t)
                    h.stamp.nanosec = round((t-int(t))*1e9)
                    live.on_cloud(create_cloud_xyz32(h, np.array([[1., 0., .5], [1.01, 0., .6]], np.float32)))
                with patch.object(live.maps['task'], 'publish', side_effect=self.nav.on_map), \
                     patch.object(live.control_reference, 'publish', side_effect=self.nav.on_control_reference):
                    for t in np.linspace(99.1, 100., 10):
                        odom(float(t)); cloud(float(t)); live.tick()
                    self.assertGreater(live.updates, 0)
                    odom(100.1, .1, .1)
                    odom(100.8, .8, .8)
                    cloud(100.1)
                    with patch.object(live.robot, 'publish') as historical, \
                         patch.object(live.core, 'add', wraps=live.core.add) as fusion:
                        live.tick()
                    old = historical.call_args.args[0]
                    self.assertAlmostEqual(old.pose.position.x, .1)
                    self.assertAlmostEqual(fusion.call_args.args[2][0], .1)
                    self.assertAlmostEqual(self.nav.control.pose[0], .8)
                    self.assertAlmostEqual(self.nav.control.pose[2], .8)
                    self.assertAlmostEqual(self.nav.pose_source_stamp/1e9, 100.8)
            finally:
                live.destroy_node()

    def test_recorded_L3_source_selection_without_changing_original_stamps(self):
        # Real recorded headers/XYZ/ICP, not a closed-loop movement simulation.
        # The recording has only odom yaw, so reconstructed orientation is planar;
        # verify source selection and mapped XYZ, not full-3D attitude accuracy.
        directory = Path(os.environ.get('G1_L3_EVIDENCE_DIR',
            str(ROOT.parent/'_scratch/20261121_交付/l3_result')))
        if not all((directory/name).is_file() for name in ('l3_track.jsonl', 'l3_goal.jsonl')):
            self.skipTest('External recorded L3 evidence missing; set G1_L3_EVIDENCE_DIR')
        records = [json.loads(line) for line in (directory/'l3_track.jsonl').read_text().splitlines()]
        goals = [json.loads(line) for line in (directory/'l3_goal.jsonl').read_text().splitlines()]
        start = next(r['wall'] for r in goals if r['kind'] == 'accepted')
        end = next(r['wall'] for r in goals if r['kind'] == 'result')
        origin = records[0]['wall']
        latest = fit = None
        revision = 0
        latest_received = fit_received = None
        baseline_lags, new_lags = [], []
        for row in records:
            now = 1000.+row['wall']-origin
            with patch('g1_navigation_node.time.monotonic', return_value=now):
                if row['kind'] == 'odom':
                    latest, latest_received = row, now
                    m = Odometry(); m.header.frame_id = self.nav.cfg['odom_frame']
                    t = row['stamp']; m.header.stamp.sec = int(t)
                    m.header.stamp.nanosec = round((t-int(t))*1e9)
                    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = row['x'], row['y'], row['z']
                    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(row['yaw']/2), math.cos(row['yaw']/2)
                    self.nav.on_control_odom(m)
                elif row['kind'] == 'localization_update':
                    fit = json.loads(row['raw']); fit_received = now
                    revision += 1
                elif row['kind'] == 'pose' and start <= row['wall'] <= end and latest and fit:
                    # Recreate the new producer lease using recorded receipt
                    # times. These packets were not present in the old run.
                    packet = reference(now=now, session_id='recorded-L3-offline',
                        revision=revision,
                        valid=fit['accepted'] and fit['fitness'] >= self.nav.cfg['registration_min_fitness'],
                        transform=np.asarray(fit['transform']).reshape(4, 4).tolist(),
                        registration_stamp=fit['stamp'], latest_odom_stamp=latest['stamp'],
                        odom_received_monotonic=latest_received,
                        valid_until_monotonic=min(fit_received+6., latest_received+1.),
                        map_stamp_ns=self.nav.map_stamp)
                    self.nav.on_control_reference(String(data=json.dumps(packet)))
                    self.assertTrue(self.nav.control_pose_status['valid'], self.nav.control_pose_status)
                    new_lags.append(latest['stamp']-self.nav.pose_source_stamp/1e9)
                    baseline_lags.append(latest['stamp']-row['stamp'])
                    point = np.array([latest['x'], latest['y'], latest['z'], 1.])
                    expected = np.asarray(fit['transform']).reshape(4, 4) @ point
                    np.testing.assert_allclose(self.nav.control.pose[:2], expected[:2], atol=1e-9)
        self.assertGreater(len(new_lags), 50)
        self.assertGreater(statistics.median(baseline_lags), .5)
        self.assertLess(max(abs(v) for v in new_lags), 1e-6)
        self.assertFalse(self.commands)


if __name__ == '__main__':
    unittest.main()
