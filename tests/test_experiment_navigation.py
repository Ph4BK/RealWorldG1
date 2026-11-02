"""Real ROS callbacks with the Unitree command publisher replaced by an in-memory sink.

Requires an isolated localhost ROS domain. These tests never publish Unitree commands
and do not claim to verify Unitree DDS serialization or physical motion.
"""
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from experiment_recorder import ExperimentSession
from posnav_fmm.errors import GoalUnreachable


class RequestStub:
    def __init__(self):
        self.header = SimpleNamespace(identity=SimpleNamespace(id=0, api_id=0),
                                      policy=SimpleNamespace(noreply=False))
        self.parameter = ''


# Only the unavailable Unitree type is substituted, during this isolated import.
stub = ModuleType('unitree_api.msg')
stub.Request = RequestStub
spec = importlib.util.spec_from_file_location('navigation_evidence_under_test', ROOT/'tools/g1_navigation_node.py')
nav = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'unitree_api': ModuleType('unitree_api'), 'unitree_api.msg': stub}):
    spec.loader.exec_module(nav)


class NavigationEvidenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID', '0') == '0' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Use a nonzero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.session = ExperimentSession(ROOT, ROOT/'maps/full_lab_v1', ROOT/'fastlio/config/live_bev.yaml', self.tmp.name)
        self.commands = []
        original = Node.create_publisher
        def publisher(node, message_type, topic, qos, **kwargs):
            if topic == '/api/sport/request':
                return SimpleNamespace(publish=self.commands.append)
            return original(node, message_type, topic, qos, **kwargs)
        with patch.object(Node, 'create_publisher', publisher):
            self.node = nav.NavigationNode(ROOT/'fastlio/config/live_bev.yaml', experiment_dir=self.session.path)

    def tearDown(self):
        self.node.cancel('Navigation process stopped')
        self.node.close_recordings()
        self.node.destroy_node()
        self.session.close()
        self.tmp.cleanup()

    def pose(self, x=0., y=0.):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.w = 1.
        return msg

    def grid(self, blocked=False):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.info.resolution = .05
        msg.info.width = msg.info.height = 200
        msg.info.origin.position.x = msg.info.origin.position.y = -2.
        msg.info.origin.orientation.w = 1.
        data = np.zeros((200, 200), dtype=np.int8)
        if blocked:
            data[25:56, 85] = data[25:56, 115] = 100
            data[25, 85:116] = data[55, 85:116] = 100
        msg.data = data.ravel().tolist()
        return msg

    def ready(self, goal=3., blocked=False):
        self.node.on_goal(self.pose(goal))
        self.node.on_pose(self.pose())
        self.node.on_map(self.grid(blocked))
        response = self.node.start(None, Trigger.Response())
        self.assertTrue(response.success, response.message)

    def read(self, relative):
        self.node.experiment.writer.queue.join()
        return json.loads((self.session.path/relative).read_text())

    def events(self):
        self.node.experiment.writer.queue.join()
        return [json.loads(line) for line in (self.session.path/'events.jsonl').read_text().splitlines()]

    def test_rejected_request_and_idle_never_create_trial_or_send_motion(self):
        response = self.node.start(None, Trigger.Response())
        self.assertFalse(response.success)
        self.node.tick()
        self.assertEqual(self.commands, [])
        self.assertIsNone(self.node.experiment.trial_id)
        self.assertTrue(any(e['event']=='start_request' and not e['accepted'] for e in self.events()))

    def test_arrival_and_second_goal_have_separate_results(self):
        self.ready(goal=.2)
        self.node.tick()
        result = self.read('trials/trial_0001/result.json')
        self.assertTrue(result['success'])
        self.assertEqual(result['planning_attempts'], 0)
        self.assertTrue((self.session.path/result['final_map_ref']).exists())
        self.ready(goal=3.)
        self.node.tick()
        self.node.on_goal(self.pose(4.))
        result = self.read('trials/trial_0002/result.json')
        self.assertEqual(result['outcome'], 'cancelled')
        self.assertFalse(result['success'])
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., 0.])

    def test_real_fmm_unreachable_attempt_then_recovery(self):
        self.ready(blocked=True)
        self.node.tick()
        self.assertEqual(self.node.control.state, 'plan_wait')
        self.node.experiment.writer.queue.join()
        trial = self.session.path/'trials/trial_0001'
        first = json.loads((trial/'planning.jsonl').read_text())
        self.assertEqual(first['outcome'], 'retry')
        self.assertIsNone(first['selected_stg'])
        self.assertEqual((trial/'stg_segments.jsonl').read_text(), '')
        self.node.on_map(self.grid())
        self.node.on_pose(self.pose())
        self.node.control.until = 0.
        self.node.tick()
        self.assertEqual(self.node.control.state, 'walk_start')
        self.node.stop(None, Trigger.Response())
        self.node.experiment.writer.queue.join()
        attempts = [json.loads(line) for line in (trial/'planning.jsonl').read_text().splitlines()]
        self.assertEqual([a['outcome'] for a in attempts], ['retry', 'selected'])
        self.assertNotEqual(attempts[0]['map_ref'], attempts[1]['map_ref'])
        segment = json.loads((trial/'stg_segments.jsonl').read_text())
        self.assertIn('Stopped by user', segment['anomalies'])

    def test_stale_input_does_not_count_as_planning_attempt(self):
        self.ready()
        self.node.control.pose_time = -float('inf')
        with patch.object(nav, 'plan_snapshot', wraps=nav.plan_snapshot) as planner:
            self.node.tick()
            planner.assert_not_called()
        self.assertEqual(self.node.experiment.attempt, 0)
        self.assertTrue(any(e.get('status', {}).get('message')=='Waiting for pose' for e in self.events()))

    def test_no_stg_return_preserves_planner_reason_and_terminates_without_segment(self):
        self.ready()
        plan = SimpleNamespace(stg_xy=None, reason='FMM distance ratio stop', rollout_rc=[])
        with patch.object(nav, 'plan_snapshot', return_value=plan):
            self.node.tick()
        self.assertFalse(self.node.control.active)
        attempt = self.read('trials/trial_0001/planning.jsonl')
        result = self.read('trials/trial_0001/result.json')
        self.assertEqual(attempt['outcome'], 'no_stg')
        self.assertEqual(attempt['reason'], plan.reason)
        self.assertFalse(result['success'])
        self.assertEqual(result['stg_count'], 0)

    def test_writer_failure_stops_and_rejects_next_start(self):
        self.ready()
        self.node.experiment.writer.queue.join()
        with patch.object(self.node.experiment.writer, '_write', side_effect=OSError('disk full')):
            self.node.experiment.event('fail_write')
            self.node.experiment.writer.queue.join()
        self.node.tick()
        self.assertFalse(self.node.control.active)
        self.assertIn('disk full', self.node.control.reason)
        self.assertFalse(self.node.start(None, Trigger.Response()).success)
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., 0.])

    def test_full_recording_and_old_stg_file_can_coexist(self):
        from stg_error_recorder import STGErrorLog
        self.node.error_log = STGErrorLog(Path(self.tmp.name)/'legacy')
        legacy_path = self.node.error_log.path
        self.ready()
        self.node.tick()
        self.node.tick()
        self.node.stop(None, Trigger.Response())
        legacy = json.loads(legacy_path.read_text())
        self.assertNotIn('trial_id', legacy)
        self.assertIn('Stopped by user', legacy['anomalies'])
        self.node.experiment.writer.queue.join()
        evidence = json.loads((self.session.path/'trials/trial_0001/stg_segments.jsonl').read_text())
        self.assertEqual(evidence['trial_id'], 'trial_0001')
        self.assertEqual(evidence['initial_pose'], legacy['initial_pose'])


if __name__ == '__main__':
    unittest.main()
