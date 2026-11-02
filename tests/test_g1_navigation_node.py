"""Run in an isolated ROS domain on loopback only; never on the robot domain."""
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
from unitree_api.msg import Request
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from g1_navigation_node import NavigationNode
from stg_error_recorder import STGErrorRecorder, STGErrorLog


class NavigationNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID', '0') == '0' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Tests require a nonzero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = NavigationNode(ROOT/'fastlio/config/live_bev.yaml')
        self.commands = []
        self.publisher = patch.object(self.node.commands, 'publish', side_effect=self.commands.append)
        self.publisher.start()

    def tearDown(self):
        self.publisher.stop()
        if self.node.error_log is not None:
            self.node.error_log.close()
        self.node.destroy_node()

    def enable_recording(self, directory):
        self.node.control.recorder = STGErrorRecorder()
        self.node.error_log = STGErrorLog(directory)

    def test_recording_disabled_by_default(self):
        self.assertIsNone(self.node.error_log)
        self.assertIsNone(self.node.control.recorder)

    def test_recording_option_creates_session_file_without_starting_navigation(self):
        with tempfile.TemporaryDirectory() as directory:
            node = NavigationNode(ROOT/'fastlio/config/live_bev.yaml',
                                  record_stg_errors=True, stg_error_dir=directory)
            try:
                self.assertIsInstance(node.control.recorder, STGErrorRecorder)
                self.assertEqual(node.error_log.path.parent, Path(directory))
                self.assertEqual(node.error_log.path.read_text(), '')
                self.assertFalse(node.control.active)
            finally:
                node.error_log.close()
                node.destroy_node()

    def test_recording_flush_follows_stop_publication(self):
        with tempfile.TemporaryDirectory() as directory:
            self.enable_recording(directory)
            self.ready()
            c = self.node.control
            c.plan = lambda *_: (1., 0.)
            self.node.tick()
            c.until = 0.
            self.node.tick()
            self.node.on_pose(self.pose(.95, .03))
            self.node.tick()
            self.node.on_pose(self.pose(1.03, .04, .2))
            c.until = 0.
            events = []
            with patch.object(self.node, 'send', side_effect=lambda v: events.append(('send', v))), \
                 patch.object(self.node.error_log, 'flush', wraps=self.node.error_log.flush) as flush:
                flush.side_effect = lambda r: (events.append(('flush', None)), STGErrorLog.flush(self.node.error_log, r))
                self.node.tick()
            self.assertEqual(events, [('send', (0., 0.)), ('flush', None)])
            row = json.loads(self.node.error_log.path.read_text())
            self.assertAlmostEqual(row['error_m'], .05)
            for actual, expected in zip(row['walk_settle_pose'], [1.03, .04, .2]):
                self.assertAlmostEqual(actual, expected)
            self.assertEqual(c.state, 'plan')

    def test_redraw_records_cancelled_segment(self):
        with tempfile.TemporaryDirectory() as directory:
            self.enable_recording(directory)
            self.ready()
            self.node.tick()
            self.node.on_goal(self.pose(2., 0.))
            row = json.loads(self.node.error_log.path.read_text())
            self.assertEqual(row['anomalies'], ['Goal redrawn; press Start Navigation'])
            self.assertIsNone(row['walk_settle_pose'])
            self.assertNotIn('error_m', row)

    def test_execution_error_records_partial_segment(self):
        with tempfile.TemporaryDirectory() as directory:
            self.enable_recording(directory)
            self.ready()
            self.node.control.plan = lambda *_: (1., 0.)
            self.node.tick()
            c = self.node.control
            c.until = 0.
            with patch.object(c, 'clear', side_effect=ValueError('test execution failure')):
                self.node.tick()
            row = json.loads(self.node.error_log.path.read_text())
            self.assertIn('Navigation stopped: test execution failure', row['anomalies'])
            self.assertIsNotNone(row['turn_settle_pose'])
            self.assertIsNone(row['walk_settle_pose'])
            self.assertNotIn('error_m', row)
            self.assertFalse(c.active)

    def test_storage_failure_stops_and_refuses_new_task(self):
        with tempfile.TemporaryDirectory() as directory:
            self.enable_recording(directory)
            self.ready()
            with patch.object(self.node.error_log, 'flush', side_effect=OSError('disk full')):
                self.node.tick()
            self.assertFalse(self.node.control.active)
            self.assertIn('disk full', self.node.control.reason)
            response = self.node.start(None, Trigger.Response())
            self.assertFalse(response.success)
            self.assertIn('disk full', response.message)
            self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., 0.])

    def pose(self, x=0., y=0., yaw=0.):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return p

    def grid(self):
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.info.resolution = .05
        m.info.width = m.info.height = 200
        m.info.origin.position.x = m.info.origin.position.y = -2.
        m.data = [0]*40000
        return m

    def ready(self):
        self.node.on_goal(self.pose(3., 0.))
        self.node.on_pose(self.pose())
        self.node.on_map(self.grid())
        r = self.node.start(None, Trigger.Response())
        self.assertTrue(r.success, r.message)

    def test_idle_and_goal_selection_never_send_commands(self):
        self.node.tick()
        self.node.on_goal(self.pose(3., 0.))
        self.node.tick()
        self.assertEqual(self.commands, [])
        self.assertFalse(self.node.start(None, Trigger.Response()).success)

    def test_command_matches_official_api_without_response(self):
        self.node.send((.5, 0.))
        r = self.commands[-1]
        self.assertEqual(r.header.identity.api_id, 7105)
        self.assertTrue(r.header.policy.noreply)
        self.assertEqual(json.loads(r.parameter), {'velocity': [.5, 0., 0.], 'duration': .1})
        self.assertGreater(r.header.identity.id, 0)
        self.node.send((0., -1.))
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., -1.])
        self.assertAlmostEqual(self.node.control.control_hz, 50.)

    def test_pose_source_stamp_is_not_checked(self):
        p = self.pose()
        p.header.stamp.sec = 100
        self.node.on_pose(p)
        t = self.node.control.pose_time
        p.header.stamp.sec = 0
        with patch('g1_navigation_node.time.monotonic', return_value=t+1.):
            self.node.on_pose(p)
        self.assertEqual(self.node.control.pose_time, t+1.)
        with patch('g1_navigation_node.time.monotonic', return_value=t+2.):
            self.node.on_pose(p)
        self.assertEqual(self.node.control.pose_time, t+2.)

    def test_republishing_map_does_not_refresh_fusion_age(self):
        m = self.grid()
        self.node.on_map(m)
        before = self.node.control.map_time
        with patch('g1_navigation_node.time.monotonic', return_value=before+2.):
            self.node.on_map(m)
        self.assertEqual(self.node.control.map_time, before)
        m.header.stamp.sec = m.header.stamp.nanosec = 0
        self.node.on_map(m)
        self.assertEqual(self.node.control.map_time, -math.inf)

    def test_latched_old_map_is_not_fresh_at_startup(self):
        m = self.grid()
        m.header.stamp.sec -= 5
        self.node.on_map(m)
        self.assertGreater(time.monotonic()-self.node.control.map_time, 4.9)

    def test_same_goal_redraw_stops_and_requires_restart(self):
        self.ready()
        self.node.on_goal(self.pose(3., 0.))
        self.assertFalse(self.node.control.active)
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., 0.])
        count = len(self.commands)
        self.node.tick()
        self.assertEqual(len(self.commands), count)

    def test_planning_and_forward_command_without_any_ack(self):
        self.ready()
        self.node.on_pose(self.pose(0., 0., -.1))
        self.node.tick()
        self.assertTrue(self.node.control.active, self.node.control.reason)
        c = self.node.control
        self.assertEqual(c.state, 'turn_out')
        sign = c.turn_sign
        self.node.tick()
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., float(sign)])
        self.node.on_pose(self.pose(0., 0., -.1 + sign*.41))
        self.node.tick()
        self.assertEqual(c.state, 'turn_out_settle')
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., 0.])
        c.until = 0.
        self.node.tick()
        self.node.tick()
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., float(-sign)])
        heading = math.atan2(c.stg[1], c.stg[0])
        self.node.on_pose(self.pose(0., 0., heading))
        self.node.tick()
        self.assertEqual(c.state, 'turn_settle')
        c.until = 0.
        self.node.tick()
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [.5, 0., 0.])
        self.node.stop(None, Trigger.Response())
        self.assertFalse(self.node.control.active)
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., 0.])

    def test_aligned_stg_starts_walking_next_cycle_without_settle_timer(self):
        self.ready()
        self.node.tick()
        self.assertEqual(self.node.control.state, 'walk_start')
        self.node.tick()
        self.assertEqual(self.node.control.state, 'walk')
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [.5, 0., 0.])

    def test_inflated_corridor_clips_at_one_meter_and_stg(self):
        m = self.grid()
        data = np.zeros((200,200), dtype=np.int8)
        data[40,70] = 100  # 1.5 m ahead: inflated edge remains beyond 1 m
        m.data = data.ravel().tolist()
        self.node.on_map(m)
        self.assertTrue(self.node.clear((0.,0.), (2.,0.), 1.))
        self.assertFalse(self.node.clear((0.,0.), (2.,0.), math.inf))
        self.assertTrue(self.node.clear((0.,0.), (.5,0.), math.inf))
        self.assertFalse(self.node.clear((.4,0.), (2.,0.), 1.))

    def test_unreachable_goal_stays_active_and_new_map_resumes(self):
        self.node.on_goal(self.pose(3., 0.))
        self.node.on_pose(self.pose())
        m = self.grid()
        data = np.zeros((200,200), dtype=np.int8)
        # Enclose the goal without occupying its cell after inflation.
        data[25:56, 85] = data[25:56, 115] = 100
        data[25, 85:116] = data[55, 85:116] = 100
        m.data = data.ravel().tolist()
        self.node.on_map(m)
        self.assertTrue(self.node.start(None, Trigger.Response()).success)
        self.node.tick()
        c = self.node.control
        self.assertTrue(c.active)
        self.assertEqual(c.state, 'plan_wait')
        status = json.loads(self.node.last_status)
        self.assertEqual(status['message'], 'Goal is unreachable from the current position')
        self.assertIsNone(status['stg'])
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0.,0.,0.])
        deadline = c.until
        with patch('g1_navigation_node.time.monotonic', return_value=deadline-.01):
            self.node.on_map(self.grid())  # Barrier removed, but keep the 1 s interval.
            self.node.on_pose(self.pose())
            self.node.tick()
        self.assertEqual(c.state, 'plan_wait')
        with patch('g1_navigation_node.time.monotonic', return_value=deadline):
            self.node.tick()
        self.assertTrue(c.active)
        self.assertEqual(c.state, 'walk_start')
        self.assertIsNotNone(c.stg)
        self.assertNotIn('unreachable', json.loads(self.node.last_status)['message'])

    def test_redraw_during_unreachable_wait_does_not_auto_restart(self):
        from posnav_fmm.errors import GoalUnreachable
        self.ready()
        with patch.object(self.node.control, 'plan', side_effect=GoalUnreachable('Goal is unreachable from the current position')):
            self.node.tick()
        self.assertEqual(self.node.control.state, 'plan_wait')
        self.node.on_goal(self.pose(2., 1.))
        self.assertFalse(self.node.control.active)
        count = len(self.commands)
        self.node.tick()
        self.assertEqual(len(self.commands), count)

    def test_inflated_start_waits_and_recovers_when_map_clears(self):
        self.node.on_goal(self.pose(3., 0.))
        self.node.on_pose(self.pose())
        m = self.grid()
        data = np.zeros((200,200), dtype=np.int8)
        data[45,40] = 100  # Inflation reaches the robot's start cell (40,40).
        m.data = data.ravel().tolist()
        self.node.on_map(m)
        self.assertTrue(self.node.start(None, Trigger.Response()).success)
        self.node.tick()
        c = self.node.control
        self.assertEqual(c.state, 'plan_wait')
        self.assertIsNone(c.stg)
        self.assertIn('unreachable', c.reason)
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0.,0.,0.])
        with patch('g1_navigation_node.time.monotonic', return_value=c.until):
            self.node.on_map(self.grid())
            self.node.on_pose(self.pose())
            self.node.tick()
        self.assertEqual(c.state, 'walk_start')
        self.assertIsNotNone(c.stg)

    def test_blocked_segment_status_and_retry_recovery(self):
        self.ready()
        c = self.node.control
        with patch.object(c, 'clear', return_value=False):
            self.node.tick()
        self.assertTrue(c.active)
        self.assertEqual(c.state, 'plan_wait')
        status = json.loads(self.node.last_status)
        self.assertEqual(status['message'], 'STG segment is blocked')
        self.assertTrue(status['active'])
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0.,0.,0.])
        deadline = c.until
        with patch('g1_navigation_node.time.monotonic', return_value=deadline-.01):
            self.node.on_map(self.grid())
            self.node.on_pose(self.pose())
            self.node.tick()
        self.assertEqual(c.state, 'plan_wait')
        with patch('g1_navigation_node.time.monotonic', return_value=deadline):
            self.node.tick()
        self.assertTrue(c.active)
        self.assertEqual(c.state, 'walk_start')
        self.assertNotIn('blocked', json.loads(self.node.last_status)['message'])

    def test_local_stg_exhaustion_retries_without_creating_experiment(self):
        from posnav_fmm.errors import NoVisibleSTG
        self.node.control.recorder = STGErrorRecorder()
        self.ready()
        c = self.node.control
        with patch('g1_navigation_node.plan_snapshot',side_effect=NoVisibleSTG('No visible progressing STG')) as plan:
            self.node.tick()
            self.assertEqual(c.state,'plan_wait')
            self.assertTrue(c.active)
            self.assertIsNone(c.stg)
            self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'],[0.,0.,0.])
            self.assertIn('No visible',json.loads(self.node.last_status)['message'])
            with patch('g1_navigation_node.time.monotonic',return_value=c.until-.01):
                self.node.on_pose(self.pose());self.node.on_map(self.grid());self.node.tick()
            self.assertEqual(plan.call_count,1)
        with patch('g1_navigation_node.time.monotonic',return_value=c.until):
            self.node.on_pose(self.pose());self.node.on_map(self.grid());self.node.tick()
        self.assertEqual(c.state,'walk_start')
        self.assertEqual(len(c.recorder.completed),0)
        self.assertEqual(c.recorder.current['anomalies'],[])

    def test_corner_short_stg_selected_without_retry_wait(self):
        self.node.control.recorder = STGErrorRecorder()
        self.node.on_goal(self.pose(5.025,3.025))
        self.node.on_pose(self.pose(1.025,3.025))
        m=self.grid();m.info.width=m.info.height=120
        m.info.origin.position.x=m.info.origin.position.y=0.
        data=np.zeros((120,120),dtype=np.int8);data[48:58,27]=100
        m.data=data.ravel().tolist();self.node.on_map(m)
        self.assertTrue(self.node.start(None,Trigger.Response()).success)
        self.node.tick()
        c=self.node.control
        self.assertIn(c.state,('turn','turn_settle'))
        np.testing.assert_allclose(c.stg,[1.325,3.175])
        self.assertTrue(self.node.clear(c.pose[:2],c.stg,math.inf))
        self.assertEqual(len(c.recorder.completed),0)
        self.assertEqual(c.recorder.current['anomalies'],[])

    def test_publisher_delivers_without_response_subscriber(self):
        # Verify actual DDS serialization, still isolated from physical G1.
        self.publisher.stop()
        received = []
        sink = self.node.create_subscription(Request, '/api/sport/request', received.append, 1)
        deadline = time.monotonic()+3.
        while self.node.commands.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.02)
        self.node.send((0., 1.))
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.02)
        self.assertTrue(received)
        self.assertEqual(json.loads(received[0].parameter), {'velocity': [0.,0.,1.], 'duration': .1})
        self.node.destroy_subscription(sink)
        self.publisher.start()


if __name__ == '__main__':
    unittest.main()
