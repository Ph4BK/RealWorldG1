"""Real ROS message types, isolated node and mocked child process; no robot I/O."""
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from live_bev_node import LiveBEV


def timestamp(header,value):
    header.stamp.sec=int(value);header.stamp.nanosec=round((value-int(value))*1e9)


class NodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.node=LiveBEV(SimpleNamespace(config=ROOT/'fastlio/config/live_bev.yaml',map_dir=ROOT/'maps/full_lab_v1',run_dir=Path(self.tmp.name)))
        self.node.count_publishers=lambda topic: 1 if topic in ['/cloud_registered_1','/Odometry_loc'] else 0
    def tearDown(self):self.node.destroy_node();self.tmp.cleanup()
    def odom(self,t):
        m=Odometry();m.header.frame_id='camera_init';timestamp(m.header,t)
        m.pose.pose.orientation.w=1.;self.node.on_odom(m)
    def cloud(self,t):
        h=Header(frame_id='camera_init');timestamp(h,t)
        m=create_cloud_xyz32(h,np.array([[-1.,0.,.5],[-1.,0.,.6],[0,0,0]],np.float32))
        self.node.on_cloud(m)
    def seed(self):
        for t in np.linspace(10,12,401):
            m=Imu();timestamp(m.header,float(t));m.linear_acceleration.z=1.;self.node.on_imu(m)
        for t in np.linspace(10,12,21):self.odom(float(t))
        self.cloud(12.)
    def registration(self,t,fitness=.9):
        self.node.on_registration(String(data=json.dumps(dict(stamp=t,fitness=fitness,accepted=True,transform=np.eye(4).ravel().tolist()))))
    def start_session(self):
        self.seed()
        arrow=PoseWithCovarianceStamped();arrow.header.frame_id='map';arrow.pose.pose.orientation.w=1.
        self.node.on_arrow(arrow)
        child=SimpleNamespace(proc=SimpleNamespace(poll=lambda:None),close=lambda:None)
        with patch('live_bev_node.Child',return_value=child):
            response=self.node.start(None,Trigger.Response())
        self.assertTrue(response.success,response.message)

    def test_experiment_preserves_each_localization_initialization_and_operations(self):
        from experiment_recorder import ExperimentSession
        experiment=ExperimentSession(ROOT,ROOT/'maps/full_lab_v1',
            ROOT/'fastlio/config/live_bev.yaml',Path(self.tmp.name)/'experiments')
        self.node.experiment=experiment
        try:
            self.start_session()
            self.node.reset_map(None,Trigger.Response())
            self.node.stop(None,Trigger.Response())
            self.start_session()
            experiment.writer.queue.join()
            first=json.loads((experiment.path/'assets/config/localization_0001.json').read_text())
            second=json.loads((experiment.path/'assets/config/localization_0002.json').read_text())
            self.assertIn('T_map_from_odom',first['initial'])
            self.assertIn('path_map',second['parameters'])
            events=[json.loads(line)['event'] for line in
                    (experiment.path/'localization_events.jsonl').read_text().splitlines()]
            self.assertEqual(events.count('localization_initialized'),2)
            self.assertIn('static_map_reset_request',events)
            self.assertIn('localization_stop_request',events)
        finally:
            experiment.close()
            self.node.experiment=None

    def test_initial_clear_uses_first_output_pose_once_and_draws_annulus(self):
        n=self.node;n.core.static[:]=True
        self.start_session()
        self.assertTrue(n.initial_clear_pending)
        self.assertTrue(n.core.task.all())
        self.odom(12.1);self.cloud(12.1);n.tick()  # No accepted localization yet.
        self.assertTrue(n.core.task.all())
        self.assertTrue(n.initial_clear_pending)
        transform=np.eye(4);transform[0,3]=1.  # ICP differs from the drawn arrow.
        n.on_registration(String(data=json.dumps(dict(stamp=12.1,fitness=.9,
            accepted=True,transform=transform.ravel().tolist()))))
        with patch.object(n.maps['task'],'publish') as maps, patch.object(n.boundary,'publish') as boundary:
            self.odom(12.2);self.cloud(12.2);n.tick()
            self.assertFalse(n.initial_clear_pending)
            inner=np.hypot(n.core.x-1.,n.core.y)<=.3
            np.testing.assert_array_equal(n.core.task,~inner)
            self.assertTrue(n.core.static.all())
            self.assertEqual(n.updates,0)  # First pose precedes a full window.
            self.assertEqual(maps.call_args.args[0].header.stamp.sec,0)
            marker=boundary.call_args.args[0]
            radii=np.array([np.hypot(p.x,p.y) for p in marker.points])
            self.assertEqual(len(radii),396)
            np.testing.assert_allclose(radii[:198],.3)
            np.testing.assert_allclose(radii[198:],2.)
        # Even after new observations/pause/recovery, this is not a moving clear disk.
        n.core.task[inner]=True
        n.core.pause()
        for i in range(n.core.window):
            t=12.3+.1*i;self.odom(t);self.cloud(t);n.tick()
        self.assertTrue(n.core.task[inner].all())
        self.assertGreater(n.updates,0)
        n.reset_map(None,Trigger.Response())
        self.assertTrue(n.core.task.all())
        self.odom(13.4);self.cloud(13.4);n.tick()
        self.assertTrue(n.core.task[inner].all())
        self.assertFalse(n.initial_clear_pending)
        n.stop(None,Trigger.Response())
        self.start_session()
        self.assertTrue(n.initial_clear_pending)
        self.registration(12.)
        self.odom(12.1);self.cloud(12.1);n.tick()
        inner=np.hypot(n.core.x,n.core.y)<=.3
        self.assertFalse(n.core.task[inner].any())
        self.assertFalse(n.initial_clear_pending)

    def test_static_restore_before_first_pose_disables_initial_clear(self):
        n=self.node;n.core.static[:]=True
        self.start_session()
        n.reset_map(None,Trigger.Response())
        self.assertFalse(n.initial_clear_pending)
        self.registration(12.)
        self.odom(12.1);self.cloud(12.1);n.tick()
        self.assertTrue(n.core.task.all())
    def test_arrow_start_stop_and_live_fusion(self):
        n=self.node
        response=n.start(None,Trigger.Response());self.assertFalse(response.success)
        self.seed()
        arrow=PoseWithCovarianceStamped();arrow.header.frame_id='map';arrow.pose.pose.orientation.w=1.
        n.on_arrow(arrow);self.assertIsNotNone(n.selected);self.assertIsNone(n.child)
        child=SimpleNamespace(proc=SimpleNamespace(poll=lambda:None),close=lambda:None)
        with patch('live_bev_node.Child',return_value=child):
            response=n.start(None,Trigger.Response());self.assertTrue(response.success,response.message)
        self.assertEqual(n.updates,0)
        self.registration(12.)
        for i in range(n.core.window):
            self.assertEqual(n.updates,0)
            t=12.+.1*(i+1)
            self.odom(t);self.cloud(t);n.tick()
        self.assertEqual(n.updates,1,n.message)
        self.assertEqual(len(n.core.frames),n.core.window)
        local=np.asarray(n.grid(np.where(n.core.mask,n.core.local*100,-1)).data).reshape(n.core.static.shape)
        self.assertTrue(np.all(local[~n.core.mask]==-1))
        snapshot=n.core.task.copy()
        self.registration(t,.2);n.tick();self.assertEqual(len(n.core.frames),0)
        np.testing.assert_array_equal(snapshot,n.core.task)
        self.assertFalse(n.start(None,Trigger.Response()).success)
        self.assertTrue(n.stop(None,Trigger.Response()).success)
        self.assertFalse(n.active);self.assertIsNone(n.selected)
    def test_stale_duplicate_fit_and_transform_jump(self):
        n=self.node;self.seed();n.active=True
        self.registration(12.)
        n.received['registration']=time.monotonic()-10
        self.registration(12.)
        self.assertIn('fresh ICP',n.health())
        n.core.task[:]=True
        self.odom(12.1)
        transform=np.eye(4);transform[0,3]=1
        n.on_registration(String(data=json.dumps(dict(stamp=12.1,fitness=.9,accepted=True,transform=transform.ravel().tolist()))))
        np.testing.assert_array_equal(n.core.task,n.core.static)
        n.received['cloud']=time.monotonic()-5
        self.assertIn('cloud data is stale',n.health())
    def test_moved_after_arrow_rejected(self):
        n=self.node;self.seed()
        arrow=PoseWithCovarianceStamped();arrow.header.frame_id='map';arrow.pose.pose.orientation.w=1.;n.on_arrow(arrow)
        for _,pose in n.odom:pose[0,3]=.2
        response=n.start(None,Trigger.Response());self.assertFalse(response.success);self.assertIn('redraw',response.message)

    def test_redraw_while_running_does_not_pause_updates(self):
        n=self.node;self.seed()
        arrow=PoseWithCovarianceStamped();arrow.header.frame_id='map';arrow.pose.pose.orientation.w=1.
        n.on_arrow(arrow)
        selected=dict(n.selected)
        child=SimpleNamespace(proc=SimpleNamespace(poll=lambda:None),close=lambda:None)
        with patch('live_bev_node.Child',return_value=child):
            self.assertTrue(n.start(None,Trigger.Response()).success)
        self.registration(12.)
        for i in range(n.core.window):
            t=12.+.1*(i+1);self.odom(t);self.cloud(t);n.tick()
        self.assertEqual(n.updates,1)
        arrow.pose.pose.position.x=99.
        n.on_arrow(arrow)
        self.assertEqual(n.selected,selected)
        self.assertEqual(n.health(),'')
        self.assertEqual(len(n.core.frames),n.core.window)
        self.assertIn('Initial pose unchanged',n.notice)
        self.assertFalse(n.start(None,Trigger.Response()).success)
        self.assertEqual(n.health(),'')
        t+=.1;self.odom(t);self.cloud(t);n.tick()
        self.assertEqual(n.updates,2)
        self.assertTrue(n.stop(None,Trigger.Response()).success)

    def test_registration_clock_mismatch_recovers_without_clearing_other_errors(self):
        n=self.node;self.seed();n.active=True
        self.registration(12.)
        n.core.task[:]=True
        self.registration(13.)  # Deferred first; only a persistent mismatch fails.
        self.assertTrue(n.accepted)
        deadline=n.pending_registrations[0][0]+n.registration_wait+.01
        with patch('live_bev_node.time.monotonic',return_value=deadline):
            n.tick()
        self.assertFalse(n.accepted)
        self.assertIn('timestamp',n.health())
        self.assertEqual(len(n.core.frames),0)
        self.assertTrue(n.core.task.all())
        self.odom(13.);self.cloud(13.);self.registration(13.)
        self.assertEqual(n.health(),'')
        n.tick()  # the matching cloud at 13.0 is the first recovered frame
        for i in range(n.core.window-1):
            t=13.+.1*(i+1);self.odom(t);self.cloud(t);n.tick()
        self.assertEqual(n.updates,1)
        n.error='Odometry clock reset: stop and redraw the initial pose.'
        self.registration(t)
        self.assertIn('clock reset',n.health())

    def test_registration_waits_for_odom_without_clearing_window(self):
        n=self.node;self.seed();n.active=True;self.registration(12.)
        for i in range(10):
            t=12.+.1*(i+1);self.odom(t);self.cloud(t);n.tick()
        before=n.core.task.copy();received=time.monotonic()
        with patch('live_bev_node.time.monotonic',return_value=received):
            self.registration(13.5)
        self.assertEqual(n.registration_stamp,12.)
        self.assertEqual(len(n.core.frames),10)
        self.assertEqual(n.health(),'')
        with patch('live_bev_node.time.monotonic',return_value=received+.4):
            self.odom(13.5)  # No second registration message is required.
        self.assertEqual(n.registration_stamp,13.5)
        self.assertEqual(n.received['registration'],received)
        self.assertEqual(len(n.pending_registrations),0)
        self.assertEqual(len(n.core.frames),10)
        self.assertEqual(n.registration_wait_timeouts,0)
        np.testing.assert_array_equal(before,n.core.task)

    def test_duplicate_does_not_extend_registration_wait(self):
        n=self.node;self.seed();n.active=True;self.registration(12.)
        begin=time.monotonic()
        with patch('live_bev_node.time.monotonic',return_value=begin):self.registration(13.)
        with patch('live_bev_node.time.monotonic',return_value=begin+.8):self.registration(13.)
        self.assertEqual(len(n.pending_registrations),1)
        self.assertEqual(n.pending_registrations[0][0],begin)
        with patch('live_bev_node.time.monotonic',return_value=begin+1.01):n.tick()
        self.assertEqual(n.registration_wait_timeouts,1)
        self.assertFalse(n.accepted)
        self.assertEqual(len(n.pending_registrations),0)

    def test_wait_queue_is_bounded_and_cleared_on_stop_or_clock_reset(self):
        n=self.node;self.seed();n.active=True;self.registration(12.)
        for i in range(11):self.registration(20.+i)
        self.assertEqual(len(n.pending_registrations),10)
        self.assertIn('queue is full',n.health())
        n.stop(None,Trigger.Response())
        self.assertEqual(len(n.pending_registrations),0)
        n.active=True;self.registration(15.)
        self.assertEqual(len(n.pending_registrations),1)
        self.odom(11.)
        self.assertFalse(n.active)
        self.assertEqual(len(n.pending_registrations),0)

    def test_newer_valid_registration_supersedes_older_waiting_result(self):
        n=self.node;self.seed();n.active=True;self.registration(12.)
        self.registration(13.)
        # Simulate odometry already available before a newer result callback.
        n.odom.append((14.,np.eye(4)))
        self.registration(14.)
        n.retry_registrations()
        self.assertEqual(n.registration_stamp,14.)
        self.assertEqual(len(n.pending_registrations),0)
        self.assertEqual(n.registration_wait_timeouts,0)

    def test_display_throttle_preserves_fusion_and_grid_publication(self):
        n=self.node;self.seed();n.active=True;self.registration(12.)
        begin=time.monotonic();n.last_display=begin
        with patch.object(n.maps['task'],'publish') as grids, \
                patch.object(n,'publish_full_display') as displays:
            for i in range(20):
                t=12.+.1*(i+1)
                with patch('live_bev_node.time.monotonic',return_value=begin+.1*(i+1)):
                    self.odom(t);self.cloud(t);n.tick()
            self.assertEqual(n.updates,11)  # Still commits every valid cloud after warm-up.
            self.assertEqual(grids.call_count,11)
            self.assertGreaterEqual(displays.call_count,4)
            self.assertLessEqual(displays.call_count,6)
            n.core.task[:]=False
            n.publish_maps(force_display=False)
            with patch('live_bev_node.time.monotonic',return_value=begin+2.5):
                n.publish_displays()
            np.testing.assert_array_equal(displays.call_args.args[1],n.core.task)
            self.assertFalse(n.display_dirty)
            before=displays.call_count
            n.reset_map(None,Trigger.Response())  # User resets must display immediately.
            self.assertEqual(displays.call_count,before+1)

    def test_manual_stg_and_goal_yaw_independence(self):
        n=self.node
        self.assertFalse(n.compute_stg(None,Trigger.Response()).success)
        self.assertEqual(n.error,'')
        self.seed();n.active=True;self.registration(12.)
        n.core.task[:]=False
        n.goal=dict(x=2.,y=0.,yaw=0.)
        before=n.core.task.copy()
        self.assertTrue(n.compute_stg(None,Trigger.Response()).success,n.plan_message)
        first=n.plan;self.assertIsNotNone(first.stg_xy)
        n.goal['yaw']=2.7
        self.assertTrue(n.compute_stg(None,Trigger.Response()).success)
        np.testing.assert_array_equal(first.distance_m,n.plan.distance_m)
        self.assertEqual(first.stg_xy,n.plan.stg_xy)
        np.testing.assert_array_equal(before,n.core.task)
        self.assertEqual(n.updates,0);self.assertEqual(n.error,'')
        n.received['odom']=time.monotonic()-5
        self.assertFalse(n.compute_stg(None,Trigger.Response()).success)
        self.assertIsNone(n.plan);self.assertEqual(n.error,'')

    def test_task_map_stamp_only_advances_on_successful_fusion(self):
        n=self.node
        with patch.object(n.maps['task'],'publish') as grids:
            n.publish_maps()
            self.assertEqual(grids.call_args.args[0].header.stamp.sec,0)
            self.seed();n.active=True;self.registration(12.)
            for i in range(n.core.window):
                t=12.+.1*(i+1);self.odom(t);self.cloud(t);n.tick()
            stamp=grids.call_args.args[0].header.stamp
            self.assertGreater(stamp.sec,0)
            n.registration_error='simulated lost localization';n.tick()
            self.assertEqual(grids.call_args.args[0].header.stamp,stamp)
            n.publish_maps()
            self.assertEqual(grids.call_args.args[0].header.stamp,stamp)
            n.reset_map(None,Trigger.Response())
            self.assertEqual(grids.call_args.args[0].header.stamp.sec,0)

    def test_goal_annotation_is_independent_of_localization(self):
        n=self.node
        goal=PoseStamped();goal.header.frame_id='map'
        goal.pose.position.x=2.;goal.pose.position.y=-1.
        goal.pose.orientation.z=np.sin(.3);goal.pose.orientation.w=np.cos(.3)
        n.on_goal(goal)  # no sensors, no initial arrow required
        self.assertAlmostEqual(n.goal['yaw'],.6)
        self.assertIsNone(n.selected);self.assertIsNone(n.child)
        self.assertFalse(n.active);self.assertEqual(n.error,'')
        self.assertEqual(json.loads((n.args.run_dir/'goal_pose.json').read_text()),n.goal)
        self.assertFalse(n.start(None,Trigger.Response()).success)  # goal is not an initial pose
        self.seed()
        arrow=PoseWithCovarianceStamped();arrow.header.frame_id='map';arrow.pose.pose.orientation.w=1.
        n.on_arrow(arrow)
        child=SimpleNamespace(proc=SimpleNamespace(poll=lambda:None),close=lambda:None)
        with patch('live_bev_node.Child',return_value=child):
            self.assertTrue(n.start(None,Trigger.Response()).success)
        self.registration(12.)
        for i in range(n.core.window):
            t=12.+.1*(i+1);self.odom(t);self.cloud(t);n.tick()
        selected=dict(n.selected);task=n.core.task.copy()
        goal.pose.position.x=3.;n.on_goal(goal)
        self.assertEqual(n.goal['x'],3.)
        self.assertEqual(n.selected,selected);self.assertEqual(n.health(),'')
        self.assertEqual(n.updates,1);self.assertEqual(len(n.core.frames),n.core.window)
        np.testing.assert_array_equal(n.core.task,task)
        goal.header.frame_id='wrong';n.on_goal(goal)
        self.assertEqual(n.goal['x'],3.);self.assertEqual(n.health(),'')
        t+=.1;self.odom(t);self.cloud(t);n.tick();self.assertEqual(n.updates,2)
        self.assertTrue(n.stop(None,Trigger.Response()).success)
        self.assertEqual(n.goal['x'],3.)

if __name__=='__main__':unittest.main()
