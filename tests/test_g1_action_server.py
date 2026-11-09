"""Real Action transport on an isolated localhost domain; motion sink is replaced."""
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
import socket
import tempfile
import threading
import uuid
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_interfaces.action import NavigateTo
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
# Keep core/error class identity when the temporary Unitree import stub is removed.
from g1_navigation_core import Navigation


class RequestStub:
    def __init__(self):
        self.header = SimpleNamespace(identity=SimpleNamespace(id=0, api_id=0),
                                      policy=SimpleNamespace(noreply=False))
        self.parameter = ''


stub = ModuleType('unitree_api.msg')
stub.Request = RequestStub
spec = importlib.util.spec_from_file_location('action_navigation_under_test', ROOT/'tools/g1_navigation_node.py')
nav = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'unitree_api': ModuleType('unitree_api'), 'unitree_api.msg': stub}):
    spec.loader.exec_module(nav)


class ActionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID', '0') == '0' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Tests require nonzero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.commands = []
        original = Node.create_publisher
        def publisher(node, message_type, topic, qos, **kwargs):
            if topic == '/api/sport/request':
                return SimpleNamespace(publish=self.commands.append)
            return original(node, message_type, topic, qos, **kwargs)
        with patch.object(Node, 'create_publisher', publisher):
            self.node = nav.NavigationNode(ROOT/'fastlio/config/live_bev.yaml')
        self.peer = Node('g1_action_test_client')
        self.client = ActionClient(self.peer, NavigateTo, '/nav/navigate_to')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.peer)
        self.xy_yaw = (0., 0., 0.)
        self.feedback = []
        self.assertTrue(self.client.wait_for_server(timeout_sec=3.))
        self.node.control.plan = lambda *_: (1., 0.)
        self.node.control.clear = lambda *_: True
        self.ready()

    def tearDown(self):
        self.node.cancel('Test stopped')
        for _ in range(5):
            self.executor.spin_once(timeout_sec=.01)
        self.client.destroy()
        self.node.destroy_node()
        self.peer.destroy_node()
        self.executor.shutdown()

    def ready(self):
        self.refresh()
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.node.get_clock().now().to_msg()
        grid.info.width = grid.info.height = 200
        grid.info.resolution = .05
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*40000
        self.node.on_map(grid)

    def refresh(self):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.node.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = self.xy_yaw[:2]
        p.pose.orientation.z = math.sin(self.xy_yaw[2]/2)
        p.pose.orientation.w = math.cos(self.xy_yaw[2]/2)
        self.node.on_pose(p)
        self.node.control.map_time = time.monotonic()

    def spin(self, future, refresh=True):
        deadline = time.monotonic()+4.
        while not future.done() and time.monotonic() < deadline:
            if refresh:
                self.refresh()
            self.executor.spin_once(timeout_sec=.01)
        self.assertTrue(future.done(), 'Action operation timed out')
        return future.result()

    def goal(self, x=3., timeout=120.):
        g = NavigateTo.Goal()
        g.g_x = x
        g.pos_tol, g.yaw_tol, g.timeout, g.max_speed = .05, .17, timeout, .5
        return g

    def send(self, goal=None, refresh=True):
        return self.spin(self.client.send_goal_async(goal or self.goal(),
            feedback_callback=lambda msg: self.feedback.append(msg.feedback)), refresh)

    def stopped(self):
        self.assertFalse(self.node.control.active)
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., 0.])

    def wait_until(self, predicate):
        deadline = time.monotonic()+4.
        while not predicate() and time.monotonic() < deadline:
            self.refresh()
            self.executor.spin_once(timeout_sec=.01)
        self.assertTrue(predicate(), 'Display update timed out')

    def test_action_goal_reaches_late_display_without_redraw_or_stg_deletion(self):
        from live_bev_node import LiveBEV
        self.node.action.transform = (1., 2., math.pi/2)
        g = self.goal(3.)
        g.g_yaw = .3
        handle = self.send(g)
        self.assertTrue(handle.accepted)
        expected = self.node.action.map_pose((g.g_x, g.g_y, g.g_yaw))
        # Start the display after acceptance: the retained goal must be delivered.
        with tempfile.TemporaryDirectory() as tmp:
            view = LiveBEV(SimpleNamespace(config=ROOT/'fastlio/config/live_bev.yaml',
                map_dir=ROOT/'maps/full_lab_v1', run_dir=Path(tmp)))
            self.executor.add_node(view)
            try:
                with patch.object(view.stg_display, 'publish') as stg:
                    self.wait_until(lambda: view.goal is not None)
                    stg.assert_not_called()
                self.assertTrue(self.node.control.active)
                self.assertEqual(bytes(self.node.action.current.handle.goal_id.uuid),
                                 bytes(handle.goal_id.uuid))
                for key, value in zip(('x', 'y', 'yaw'), expected):
                    self.assertAlmostEqual(view.goal[key], value)
                self.assertEqual(json.loads((Path(tmp)/'goal_pose.json').read_text()), view.goal)
                # RViz may attach later still; its marker must also be retained.
                markers = []
                qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                sub = self.peer.create_subscription(Marker, '/g1/live_bev/goal_display', markers.append, qos)
                try:
                    self.wait_until(lambda: bool(markers))
                    m = markers[-1]
                    self.assertEqual(m.header.frame_id, 'map')
                    self.assertEqual(m.ns, 'posnav_goal')
                    self.assertAlmostEqual(m.pose.position.x, expected[0])
                    self.assertAlmostEqual(m.pose.position.y, expected[1])
                    self.assertAlmostEqual(m.pose.orientation.z, math.sin(expected[2]/2))
                    self.assertEqual((m.color.r, m.color.g, m.color.b), (1., 0., 0.))
                    self.assertTrue(self.node.control.active)
                finally:
                    self.peer.destroy_subscription(sub)
                self.spin(handle.cancel_goal_async())
                self.assertEqual(self.spin(handle.get_result_async()).status, 5)
            finally:
                self.executor.remove_node(view)
                view.destroy_node()

    def test_goal_state_rejects_busy_updates_and_tracks_action_manual_action_order(self):
        goals = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.peer.create_subscription(PoseStamped, '/g1/navigation/goal', goals.append, qos)
        try:
            first = self.send(self.goal(3.))
            self.assertTrue(first.accepted)
            self.wait_until(lambda: bool(goals))
            self.assertEqual(goals[-1].pose.position.x, 3.)
            with patch.object(self.node.goal_state, 'publish') as publish:
                self.assertFalse(self.send(self.goal(4.)).accepted)
                publish.assert_not_called()
            manual = PoseStamped()
            manual.header.frame_id = 'map'
            manual.pose.position.x = 2.
            manual.pose.orientation.w = 1.
            pub = self.peer.create_publisher(PoseStamped, '/g1/live_bev/goal_2d', 10)
            try:
                self.wait_until(lambda: pub.get_subscription_count() > 0)
                pub.publish(manual)
                self.wait_until(lambda: goals[-1].pose.position.x == 2.)
                self.assertEqual(self.spin(first.get_result_async()).result.reason, 'goal_redrawn')
                self.assertFalse(self.node.control.active)
                second = self.send(self.goal(4.))
                self.assertTrue(second.accepted)
                self.wait_until(lambda: goals[-1].pose.position.x == 4.)
                self.assertTrue(self.node.control.active)
                self.spin(second.cancel_goal_async())
                self.spin(second.get_result_async())
            finally:
                self.peer.destroy_publisher(pub)
        finally:
            self.peer.destroy_subscription(sub)

    def test_fine_completion_and_actual_scene_pose(self):
        self.xy_yaw = (.1, .02, .4)
        self.refresh()
        goal = self.goal(.2)
        goal.g_yaw = .4
        h = self.send(goal)
        self.assertTrue(h.accepted)
        self.wait_until(lambda: self.node.control.state == 'fine_approach')
        self.xy_yaw = (.21, .02, .4)
        r = self.spin(h.get_result_async())
        self.assertEqual(r.status, 4)
        self.assertTrue(r.result.success)
        self.assertEqual(r.result.reason, 'arrived')
        self.assertEqual(r.result.final_pose.header.frame_id, 'scene')
        self.assertAlmostEqual(r.result.final_pose.pose.position.x, .21)
        self.assertAlmostEqual(r.result.internal_error, math.hypot(.01, .02), places=5)
        self.assertEqual(r.result.uncertainty, -1.)
        self.stopped()

    def test_cancel_and_feedback(self):
        h = self.send()
        for _ in range(15):
            self.refresh()
            self.executor.spin_once(timeout_sec=.01)
        self.assertTrue(self.feedback)
        self.assertEqual(self.feedback[-1].current_pose.header.frame_id, 'scene')
        self.assertAlmostEqual(self.feedback[-1].remaining_dist, 3.)
        ack = self.spin(h.cancel_goal_async())
        self.assertTrue(ack.goals_canceling)
        r = self.spin(h.get_result_async())
        self.assertEqual(r.status, 5)
        self.assertEqual(r.result.reason, 'cancelled')
        self.stopped()

    def test_recovery_sectors_use_uninflated_cell_centers_and_body_yaw(self):
        import numpy as np
        occ = np.zeros((41, 41), dtype=bool)
        origin = (-1.025, -1.025)
        for x, y in ((.1, 0.), (.3, 0.), (.75, 0.), (.8, 0.),
                     (0., .1), (-.1, 0.), (0., -.1), (.1, .1), (-.1, -.1)):
            occ[round(y/.05)+20, round(x/.05)+20] = True
        result = nav.recovery_occupancy(occ, origin, .05, (0., 0., 0.))
        self.assertEqual(result['near_count'], 7)
        self.assertEqual(result['sector_counts'], [3, 2, 1, 2])
        rotated = nav.recovery_occupancy(occ, origin, .05, (0., 0., math.pi/2))
        self.assertEqual(rotated['sector_counts'], [2, 1, 2, 3])
        translated = nav.recovery_occupancy(occ, (origin[0]+2., origin[1]-3.),
                                            .05, (2., -3., 0.))
        self.assertEqual(translated, result)

    def test_recovery_feedback_and_lateral_command_keep_original_action_alive(self):
        from posnav_fmm.errors import GoalUnreachable
        self.node.control.plan = lambda *_: (_ for _ in ()).throw(GoalUnreachable('near-field blocked'))
        self.node.control.recovery = lambda _: dict(near_count=1, sector_counts=[4, 0, 2, 3])
        handle = self.send()
        self.wait_until(lambda: self.node.control.state == 'recovery_left')
        self.wait_until(lambda: any(f.stage == 'recovery_left (1/3)' for f in self.feedback))
        self.assertTrue(self.node.control.active)
        self.assertTrue(any(json.loads(c.parameter)['velocity'] == [0., .5, 0.] for c in self.commands))
        status = json.loads(self.node.last_status)
        self.assertEqual(status['recovery_count'], 1)
        self.assertEqual(status['recovery_sector_counts'], [4, 0, 2, 3])
        self.assertAlmostEqual(self.feedback[-1].remaining_dist, 3.)
        self.spin(handle.cancel_goal_async())
        self.assertEqual(self.spin(handle.get_result_async()).status, 5)
        self.stopped()

    def test_recovery_exhaustion_and_timeout_return_real_terminal_results(self):
        from posnav_fmm.errors import GoalUnreachable
        self.node.control.plan = lambda *_: (_ for _ in ()).throw(GoalUnreachable('near-field blocked'))
        self.node.control.recovery = lambda _: dict(near_count=1, sector_counts=[0]*4)
        h = self.send()
        self.node.control.recovery_count = 3
        self.node.control.state = 'plan'
        self.node.tick()
        r = self.spin(h.get_result_async())
        self.assertEqual(r.status, 6)
        self.assertEqual(r.result.reason, 'recovery_exhausted')
        self.assertFalse(r.result.success)
        self.stopped()
        h = self.send(self.goal(timeout=.12))
        self.assertTrue(h.accepted)
        self.wait_until(lambda: self.node.control.state.startswith('recovery_'))
        r = self.spin(h.get_result_async())
        self.assertEqual(r.status, 6)
        self.assertEqual(r.result.reason, 'timeout')
        self.stopped()

    def test_timeout_aborts_and_stops(self):
        h = self.send(self.goal(timeout=.08))
        r = self.spin(h.get_result_async())
        self.assertEqual(r.status, 6)
        self.assertEqual(r.result.reason, 'timeout')
        self.stopped()

    def test_pre_selection_falls_back_for_occupied_pre_and_blocked_segment(self):
        import numpy as np
        goal = (3., 1., 0.)
        self.assertIsNotNone(self.node.navigation_route(goal)['pre'])
        for col, reason in ((48, 'occupied'), (54, 'segment')):
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.header.stamp = self.node.get_clock().now().to_msg()
            grid.info.width = grid.info.height = 200
            grid.info.resolution = .05
            grid.info.origin.orientation.w = 1.
            data = np.zeros((200,200),dtype=np.int8)
            data[20,col] = 100
            grid.data = data.ravel().tolist()
            self.node.on_map(grid)
            route = self.node.navigation_route(goal)
            self.assertIsNone(route['pre'])
            self.assertIn(reason, route['reason'])
            # Selection never clears any task-map occupancy to make pre fit.
            self.assertTrue(self.node.occupancy[20,col])

    def test_fine_yaw_feedback_and_cancel_use_original_goal_yaw(self):
        goal = self.goal(0.)
        goal.g_yaw = .4
        handle = self.send(goal)
        self.wait_until(lambda: self.node.control.state == 'heading_align')
        self.wait_until(lambda: any(f.stage=='heading_align' for f in self.feedback))
        self.wait_until(lambda: any(abs(json.loads(c.parameter)['velocity'][2]-.6)<1e-6 for c in self.commands))
        self.assertAlmostEqual(self.feedback[-1].remaining_yaw,.4,places=5)
        self.spin(handle.cancel_goal_async())
        self.assertEqual(self.spin(handle.get_result_async()).status,5)
        self.stopped()

    def test_fine_position_wait_is_included_in_overall_timeout(self):
        handle = self.send(self.goal(0.,timeout=.12))
        self.assertTrue(handle.accepted)
        result = self.spin(handle.get_result_async())
        self.assertEqual(result.status,6)
        self.assertEqual(result.result.reason,'timeout')
        self.stopped()

    def test_second_goal_and_rviz_start_rejected_while_action_owns_control(self):
        first = self.send()
        self.assertTrue(first.accepted)
        second = self.send()
        self.assertFalse(second.accepted)
        self.assertFalse(self.node.start(None, Trigger.Response()).success)
        self.spin(first.cancel_goal_async())
        self.spin(first.get_result_async())

    def test_rviz_task_blocks_action_and_still_works(self):
        self.node.begin_navigation((3., 0., 0.))
        self.assertFalse(self.send().accepted)
        self.node.stop(None, Trigger.Response())
        self.node.goal_pose = (0., 0., 0.)
        self.assertTrue(self.node.start(None, Trigger.Response()).success)
        for _ in range(4):
            self.refresh()
            self.node.tick()
            if self.node.control.state.endswith('settle'):
                self.node.control.until = 0.
        self.assertEqual(self.node.control.reason, 'Final approach completed')

    def test_goal_redraw_aborts_action(self):
        h = self.send()
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = 2.
        p.pose.orientation.w = 1.
        self.node.on_goal(p)
        r = self.spin(h.get_result_async())
        self.assertEqual(r.status, 6)
        self.assertFalse(r.result.success)
        self.assertEqual(self.node.goal, (2., 0.))
        self.stopped()

    def test_invalid_and_unready_goals_rejected(self):
        for field, value in [('g_x', math.nan), ('max_speed', .1), ('g_z', 1.), ('timeout', 0.)]:
            g = self.goal()
            setattr(g, field, value)
            self.assertFalse(self.send(g).accepted)
        self.node.control.pose_time = -math.inf
        self.assertFalse(self.send(refresh=False).accepted)
        self.assertEqual(self.commands, [])

    def test_fixed_transform_used_for_goal_and_terminal_pose(self):
        self.node.action.transform = (1., 2., math.pi/2)
        self.xy_yaw = (1., 2.1, math.pi/2)
        self.refresh()
        h = self.send(self.goal(.1))
        r = self.spin(h.get_result_async())
        self.assertEqual(r.status, 4)
        self.assertAlmostEqual(self.node.control.final_goal[0], 1.)
        self.assertAlmostEqual(self.node.control.final_goal[1], 2.1)
        self.assertAlmostEqual(r.result.final_pose.pose.position.x, .1)
        self.assertAlmostEqual(r.result.final_pose.pose.position.y, 0.)

    def test_stale_pose_never_reported_as_arrived(self):
        h = self.send()
        self.node.control.pose_time = -math.inf
        self.node.cancel('Stopped by user')
        r = self.spin(h.get_result_async(), refresh=False)
        self.assertEqual(r.status, 6)
        self.assertEqual(r.result.reason, 'pose_stale')
        self.assertEqual(r.result.internal_error, -1.)

    def test_planning_retry_is_nonterminal(self):
        from posnav_fmm.errors import PlanningRetry
        def retry(*_):
            raise PlanningRetry('Temporarily unreachable')
        self.node.control.plan = retry
        h = self.send()
        for _ in range(12):
            self.refresh()
            self.executor.spin_once(timeout_sec=.01)
        self.assertTrue(self.node.control.active, self.node.control.reason)
        self.assertEqual(self.node.control.state, 'plan_wait')
        self.spin(h.cancel_goal_async())
        self.assertEqual(self.spin(h.get_result_async()).status, 5)

    def test_experiment_events_keep_uuid_and_trial_association(self):
        from experiment_recorder import ExperimentSession, TrialRecorder, ExperimentSTGRecorder
        with tempfile.TemporaryDirectory() as tmp:
            session = ExperimentSession(ROOT, ROOT/'maps/full_lab_v1', ROOT/'fastlio/config/live_bev.yaml', tmp)
            recorder = TrialRecorder(session.path, {})
            self.node.experiment = recorder
            self.node.control.recorder = ExperimentSTGRecorder(recorder)
            try:
                h = self.send(self.goal(0.))
                result = self.spin(h.get_result_async())
                self.assertTrue(result.result.success)
                recorder.writer.queue.join()
                rows = [json.loads(s) for s in (session.path/'events.jsonl').read_text().splitlines()]
                started = next(r for r in rows if r['event']=='action_goal_started')
                finished = next(r for r in rows if r['event']=='action_result')
                self.assertEqual(started['action_goal_uuid'], bytes(h.goal_id.uuid).hex())
                self.assertEqual(started['trial_id'], 'trial_0001')
                self.assertEqual(finished['action_trial_id'], 'trial_0001')
                self.assertEqual(finished['action_goal_uuid'], started['action_goal_uuid'])
            finally:
                recorder.close()
                self.node.experiment = None
                self.node.control.recorder = None
                session.close()

    def test_tcp_bridge_forwards_real_action_result(self):
        sys.path.insert(0, str(ROOT/'posnav/bridge'))
        from tcp_bridge import BridgeCore, Journal, RosDriver, Server, Handler
        from tcp_protocol import PROTOCOL, FIELD_LIMITS, GOAL_TYPES, encode
        from rosidl_runtime_py.convert import message_to_ordereddict
        self.assertEqual(NavigateTo.Goal.get_fields_and_field_types(), GOAL_TYPES)
        driver = RosDriver.__new__(RosDriver)
        driver.client, driver.message, driver.serialize = self.client, NavigateTo, message_to_ordereddict
        rows, errors = [], []
        with tempfile.TemporaryDirectory() as tmp:
            core = BridgeCore(driver, self.node.action.scene_frame, True, Journal(tmp))
            server = Server(('127.0.0.1', 0), Handler)
            server.core, server.token = core, 'test-token-at-least-24-characters'
            serving = threading.Thread(target=server.serve_forever, daemon=True)
            serving.start()
            def client():
                try:
                    with socket.create_connection(server.server_address, timeout=3.) as sock:
                        sock.settimeout(3.)
                        stream = sock.makefile('rb')
                        sock.sendall(encode(dict(protocol=PROTOCOL, type='hello', token=server.token)))
                        rows.append(json.loads(stream.readline()))
                        fields = {p+k: 0. for p in ('s_', 'g_') for k in ('x','y','z','roll','pitch','yaw')}
                        fields.update(FIELD_LIMITS)
                        fields['g_x'] = 0.
                        sock.sendall(encode(dict(protocol=PROTOCOL, type='navigate',
                            request_id=str(uuid.uuid4()), source_action_id='test-navigation',
                            coordinate_frame=self.node.action.scene_frame,
                            operator_confirmed=True, goal=fields)))
                        while True:
                            row = json.loads(stream.readline())
                            rows.append(row)
                            if row['type'] == 'result':
                                break
                except Exception as exc:
                    errors.append(str(exc))
            worker = threading.Thread(target=client, daemon=True)
            worker.start()
            try:
                deadline = time.monotonic()+4.
                while worker.is_alive() and time.monotonic()<deadline:
                    self.refresh()
                    self.executor.spin_once(timeout_sec=.01)
                self.assertFalse(worker.is_alive())
                self.assertEqual(errors, [])
                self.assertTrue(rows[0]['action_available'])
                result = rows[-1]
                self.assertEqual(result['type'], 'result')
                self.assertEqual(result['data']['ros_goal_status'], 4)
                self.assertEqual(result['data']['result']['reason'], 'arrived')
                self.assertEqual(result['data']['result']['final_pose']['header']['frame_id'], 'scene')
                self.assertFalse(core.journal.pending.exists())
            finally:
                server.shutdown()
                server.server_close()
                worker.join(timeout=1.)
                serving.join(timeout=1.)


if __name__ == '__main__':
    unittest.main()
