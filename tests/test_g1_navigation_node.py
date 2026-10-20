"""Run in an isolated ROS domain on loopback only; never on the robot domain."""
import json
import math
import os
from pathlib import Path
import sys
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
        self.node.destroy_node()

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
        self.node.tick()
        self.assertTrue(self.node.control.active, self.node.control.reason)
        self.assertEqual(self.node.control.state, 'turn_settle')
        self.node.control.until = 0.
        self.node.tick()
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [.5, 0., 0.])
        self.node.stop(None, Trigger.Response())
        self.assertFalse(self.node.control.active)
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., 0.])

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
        self.assertEqual(c.state, 'turn_settle')
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
        self.assertEqual(c.state, 'turn_settle')
        self.assertNotIn('blocked', json.loads(self.node.last_status)['message'])

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
