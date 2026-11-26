"""Production node entries on an isolated loopback ROS domain; no robot I/O."""
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from live_bev_core import obstacle_map_topic, room_no_go, STATIC_TEST_WARNING
from live_bev_node import LiveBEV, navigation_command
from g1_navigation_node import NavigationNode


def stamp(header, value):
    header.stamp.sec = int(value)
    header.stamp.nanosec = round((value-int(value))*1e9)


class ObstacleModeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID', '0') == '0' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Require nonzero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.directory = Path(self.tmp.name)/'map'
        self.directory.mkdir()
        self.origin = np.array([-4., -4.])
        self.static = np.zeros((160, 160), dtype=bool)
        self.static[100, 80] = True  # A real wall at (0.025, 1.025).
        self.no_go = room_no_go(self.static.shape, self.origin, .05, [-3.5, 3.5, -3.5, 3.5])
        np.savez(self.directory/'bev_occupancy.npz', occ=self.static, origin=self.origin, res=.05)
        np.savez(self.directory/'fixed_no_go.npz', no_go=self.no_go, origin=self.origin, res=.05)
        config = dict(origin_xy_m=self.origin.tolist(), resolution_m=.05,
                      fixed_no_go=dict(file='fixed_no_go.npz',
                        sha256=hashlib.sha256((self.directory/'fixed_no_go.npz').read_bytes()).hexdigest(),
                        source='go2_room_bounds_grid_anchor_inclusive',
                        room_bounds_xmin_xmax_ymin_ymax_m=[-3.5, 3.5, -3.5, 3.5]))
        (self.directory/'bev_config.json').write_text(json.dumps(config))
        (self.directory/'metadata.yaml').write_text('initial_imu_xyz_map: [0, 0, 1.24]\n')
        self.nodes, self.patches, self.commands = [], [], []

    def tearDown(self):
        for node in reversed(self.nodes):
            if isinstance(node, NavigationNode):
                node.close_recordings()
            node.destroy_node()
        for p in reversed(self.patches):
            p.stop()
        self.tmp.cleanup()

    def live(self, mode='static_test', experiment=None):
        args = SimpleNamespace(config=ROOT/'fastlio/config/live_bev.yaml', map_dir=self.directory,
            run_dir=self.directory, obstacle_mode=mode, experiment=experiment)
        node = LiveBEV(args)
        node.count_publishers = lambda topic: 1 if topic in ('/cloud_registered_1', '/Odometry_loc') else 0
        self.nodes.append(node)
        return node

    def nav(self, mode='static_test', experiment_dir=None):
        node = NavigationNode(ROOT/'fastlio/config/live_bev.yaml', obstacle_mode=mode,
                              experiment_dir=experiment_dir)
        self.nodes.append(node)
        p = patch.object(node.commands, 'publish', side_effect=self.commands.append)
        p.start(); self.patches.append(p)
        return node

    def packet(self, live, mode='static_test'):
        messages = []
        with patch.object(live.maps[mode if mode == 'static_test' else 'task'], 'publish',
                          side_effect=lambda m: messages.append(deepcopy(m))):
            live.publish_maps(force_display=False)
        return messages[-1]

    def pair(self, mode='static_test'):
        live, nav = self.live(mode), self.nav(mode)
        live.map_update_stamp = live.get_clock().now().to_msg()
        live.core.task[80, 100] = True  # A realtime-only point one metre ahead.
        nav.on_map(self.packet(live, mode))
        self.pose(nav)
        return live, nav

    def pose(self, nav, x=0., y=0.):
        p = PoseStamped(); p.header.frame_id = 'map'
        p.pose.orientation.w = 1.; p.pose.position.x = x; p.pose.position.y = y
        nav.on_pose(p)

    def test_mode_validation_and_child_forwarding(self):
        self.assertEqual(obstacle_map_topic('dynamic'), '/g1/bev/task')
        self.assertEqual(obstacle_map_topic('static_test'), '/g1/bev/static_test')
        with self.assertRaises(ValueError):
            obstacle_map_topic('static')
        args = SimpleNamespace(config=Path('config.yaml'), record_stg_errors=True)
        legacy = navigation_command(ROOT, args)
        self.assertNotIn('--obstacle-mode', legacy)
        args.obstacle_mode = 'static_test'
        command = navigation_command(ROOT, args, SimpleNamespace(path=Path('session')))
        self.assertEqual(command[-2:], ['--obstacle-mode', 'static_test'])
        self.assertIn('--experiment-dir', command)

    def test_unknown_mode_rejected_before_node_initialization(self):
        with self.assertRaises(ValueError):
            self.nav('typo')
        with self.assertRaises(ValueError):
            self.live('typo')

    def test_candidate_options_forward_and_preview_execution_match(self):
        args=SimpleNamespace(config=ROOT/'fastlio/config/live_bev.yaml',map_dir=self.directory,
            run_dir=self.directory,obstacle_mode='static_test',record_stg_errors=False,
            walk_guard=True,clearance_weight=1.,clearance_safe_distance=.60,clip_stg_goal=True)
        live=LiveBEV(args); self.nodes.append(live)
        nav=NavigationNode(args.config,obstacle_mode='static_test',walk_guard=True,
                           clearance_weight=1.,clip_stg_goal=True)
        self.nodes.append(nav)
        command=navigation_command(ROOT,args)
        for flag in ('--walk-guard','--clearance-weight','--clearance-safe-distance','--clip-stg-goal'):
            self.assertIn(flag,command)
        self.assertEqual(live.navigation_options,nav.navigation_options)
        self.assertIsNotNone(nav.control.walk_guard)
        self.assertFalse(nav.control.active)
        live.map_update_stamp=live.get_clock().now().to_msg()
        nav.on_map(self.packet(live)); self.pose(nav)
        live.active=True; live.transform=np.eye(4); live.odom.append((1.,np.eye(4)))
        live.health=lambda:''; live.goal=dict(x=2.,y=0.,yaw=0.)
        response=live.compute_stg(Trigger.Request(),Trigger.Response())
        self.assertTrue(response.success,response.message)
        executed=nav.plan((0.,0.),(2.,0.))
        self.assertEqual(live.plan.stg_xy,executed.stg_xy)
        np.testing.assert_array_equal(live.plan.cost_to_goal,executed.cost_to_goal)
        np.testing.assert_array_equal(live.plan.traversible,executed.traversible)
        self.assertFalse(nav.control.active)

    def test_round1_real_parent_child_share_overrides_maps_and_runtime_identity(self):
        from g1_navigation_options import options_from_args
        args=SimpleNamespace(config=ROOT/'fastlio/config/live_bev.yaml',map_dir=self.directory,
            run_dir=self.directory,obstacle_mode='static_test',record_stg_errors=False,
            walk_guard=True,near_stg_no_realign=True,coarse_turn_profile=True,
            guard_reenter_deg=20.,fmm_inflation_cells=7,fmm_step_cells=20,
            coarse_omega=.5,no_turn_out=True,clearance_weight=3.,clip_stg_goal=True)
        live=LiveBEV(args);self.nodes.append(live)
        nav=NavigationNode(args.config,obstacle_mode='static_test',**options_from_args(args));self.nodes.append(nav)
        command=navigation_command(ROOT,args)
        for flag in ('--near-stg-no-realign','--coarse-turn-profile','--guard-reenter-deg',
                     '--fmm-inflation-cells','--fmm-step-cells','--coarse-omega','--no-turn-out'):
            self.assertIn(flag,command)
        self.assertEqual(live.cfg,nav.cfg)
        self.assertEqual((live.cfg['fmm_inflation_cells'],live.cfg['fmm_step_cells']),(7,20))
        self.assertEqual(nav.control.walk_guard.config.reenter_deg,20.)
        self.assertEqual(nav.control.omega,.5)
        self.assertFalse(nav.control.turn_out_enabled)
        live.map_update_stamp=live.get_clock().now().to_msg()
        nav.on_map(self.packet(live));self.pose(nav)
        live.active=True;live.transform=np.eye(4);live.odom.append((1.,np.eye(4)))
        live.health=lambda:'';live.goal=dict(x=2.,y=0.,yaw=0.)
        response=live.compute_stg(Trigger.Request(),Trigger.Response())
        self.assertTrue(response.success,response.message)
        executed=nav.plan((0.,0.),(2.,0.))
        self.assertEqual(live.plan.stg_xy,executed.stg_xy)
        np.testing.assert_array_equal(live.plan.traversible,executed.traversible)
        np.testing.assert_array_equal(live.plan.cost_to_goal,executed.cost_to_goal)
        with patch.object(live.status,'publish') as published:
            live.publish_status()
        preview=json.loads(published.call_args.args[0].data)
        nav.publish_status();actual=json.loads(nav.last_status)
        self.assertEqual(preview['runtime_parameters']['planning'],actual['runtime_parameters']['planning'])
        self.assertEqual(preview['task_map_identity'],actual['task_map_identity'])
        self.assertAlmostEqual(preview['start_check_inflation_m'],.35)
        self.assertAlmostEqual(actual['runtime_parameters']['stop_check_inflation_m'],.35)
        self.assertEqual(actual['runtime_parameters']['walk_guard']['reenter_deg'],20.)
        self.assertTrue(actual['runtime_parameters']['motion_policy']['near_stg_no_realign'])
        self.assertEqual(preview['runtime_parameters']['startup_source_sha256_lf'],
                         actual['runtime_parameters']['startup_source_sha256_lf'])
        self.assertFalse(nav.control.active)

    def test_stop_band_preview_child_and_fine_share_actual_policy(self):
        from g1_navigation_options import options_from_args
        self.static[:]=False;self.static[:,80]=True
        np.savez(self.directory/'bev_occupancy.npz',occ=self.static,origin=self.origin,res=.05)
        args=SimpleNamespace(config=ROOT/'fastlio/config/live_bev.yaml',map_dir=self.directory,
            run_dir=self.directory,record_stg_errors=False,obstacle_mode='static_test',
            fmm_inflation_cells=7,fmm_stop_inflation_cells=4,allow_goal_in_inflation=True,
            clip_stg_goal=True,stg_clearance_m=.4)
        live=LiveBEV(args);self.nodes.append(live)
        nav=NavigationNode(args.config,obstacle_mode='static_test',**options_from_args(args));self.nodes.append(nav)
        command=navigation_command(ROOT,args)
        self.assertEqual(command[command.index('--fmm-stop-inflation-cells')+1],'4')
        self.assertEqual(command.count('--allow-goal-in-inflation'),1)
        live.map_update_stamp=live.get_clock().now().to_msg();nav.on_map(self.packet(live))
        live.active=live.accepted=True;live.health=lambda:''
        for start,goal in [((.325,0.),(1.025,0.)),((1.025,0.),(.325,.6))]:
            with self.subTest(start=start,goal=goal):
                transform=np.eye(4);transform[:2,3]=start
                live.transform=transform;live.odom.clear();live.odom.append((1.,np.eye(4)))
                live.received['odom']=time.monotonic();live.start_check_free=None
                live.goal=dict(x=goal[0],y=goal[1],yaw=0.)
                nav.control.pose=(*start,0.)
                nav.control.goal=goal
                response=live.compute_stg(None,Trigger.Response())
                self.assertTrue(response.success,response.message)
                result=nav.plan(start,goal)
                self.assertEqual(live.plan.stg_xy,result.stg_xy)
                self.assertEqual(live.plan.inflation_connections,result.inflation_connections)
                np.testing.assert_array_equal(live.plan.traversible,result.traversible)
                with patch.object(live.status,'publish') as published:live.publish_status()
                preview=json.loads(published.call_args.args[0].data)
                nav.publish_status();actual=json.loads(nav.last_status)
                for key in ('start_free','planning_start_free','stop_start_free','start_requires_connector'):
                    self.assertEqual(preview[key],actual[key])
                self.assertAlmostEqual(preview['start_check_inflation_m'],.2)
                for key in ('planning_check_inflation_m','stop_check_inflation_m','allow_goal_in_inflation',
                            'endpoint_connection_policy','endpoint_connection_max_length_m','stop_check_scope'):
                    self.assertEqual(preview['runtime_parameters'][key],actual['runtime_parameters'][key])
                self.assertEqual(preview['navigation_options'],actual['navigation_options'])
                self.assertEqual(preview['inflation_connections'],actual['inflation_connections'])
                nav.control.final_goal=(*goal,0.)
                self.assertTrue(nav.control._goal_clear())
        self.assertFalse(nav.control.active)

    def test_invalid_candidate_cost_rejected_before_child_spawn(self):
        args=SimpleNamespace(config=Path('config.yaml'),record_stg_errors=False,
                             clearance_weight=float('nan'))
        with self.assertRaises(ValueError):navigation_command(ROOT,args)
        with self.assertRaises(ValueError):
            NavigationNode(ROOT/'fastlio/config/live_bev.yaml',clearance_weight=-1)

    def test_stg_gate_reaches_actual_preview_child_and_selection_status(self):
        # Wall at x=.025; this chord stays .325m away, above hard inflation
        # but below the user 40cm shortcut gate.
        self.static[:,:]=False;self.static[20:140,80]=True
        np.savez(self.directory/'bev_occupancy.npz',occ=self.static,origin=self.origin,res=.05)
        args=SimpleNamespace(config=ROOT/'fastlio/config/live_bev.yaml',map_dir=self.directory,
            run_dir=self.directory,record_stg_errors=False,obstacle_mode='static_test',
            stg_clearance_m=.4,clearance_weight=3.)
        from g1_navigation_options import options_from_args
        options=options_from_args(args)
        live=LiveBEV(args);self.nodes.append(live)
        nav=NavigationNode(args.config,obstacle_mode='static_test',**options);self.nodes.append(nav)
        command=navigation_command(ROOT,args)
        self.assertEqual(command[command.index('--stg-clearance-m')+1],'0.4')
        live.map_update_stamp=live.get_clock().now().to_msg();nav.on_map(self.packet(live))
        robot=np.eye(4);robot[:2,3]=(-.325,-2.)
        live.active=True;live.transform=np.eye(4);live.odom.append((1.,robot))
        live.health=lambda:'';live.goal=dict(x=-.325,y=2.,yaw=0.)
        response=live.compute_stg(Trigger.Request(),Trigger.Response())
        self.assertTrue(response.success,response.message)
        executed=nav.plan((-.325,-2.),(-.325,2.))
        self.assertEqual(live.plan.stg_xy,executed.stg_xy)
        self.assertEqual(live.plan.stg_selection,executed.stg_selection)
        self.assertEqual(executed.stg_selection['selected_macro_index'],1)
        self.assertTrue(executed.stg_selection['first_macro_exception'])
        with patch.object(live.status,'publish') as pub:live.publish_status()
        preview=json.loads(pub.call_args.args[0].data)
        nav.publish_status();actual=json.loads(nav.last_status)
        self.assertEqual(preview['runtime_parameters']['clearance'],actual['runtime_parameters']['clearance'])
        self.assertEqual(preview['stg_selection']['candidates'],actual['stg_selection']['candidates'])
        self.assertEqual(preview['task_map_identity'],actual['task_map_identity'])
        self.assertFalse(nav.control.active)

    def test_fine_options_reach_actual_child_and_status(self):
        args = SimpleNamespace(config=ROOT/'fastlio/config/live_bev.yaml', map_dir=self.directory,
            run_dir=self.directory, record_stg_errors=False, fine_stop_layered=True,
            fine_goal_behind=True, fine_pre_fallback=True, fine_align_wz=.2,
            position_settle_fresh=True, fine_stop_radius=.09, position_settle_samples=5)
        live = LiveBEV(args); self.nodes.append(live)
        from g1_navigation_options import options_from_args
        options = options_from_args(args)
        nav = NavigationNode(args.config, **options); self.nodes.append(nav)
        self.assertEqual(live.navigation_options, nav.navigation_options)
        command = navigation_command(ROOT, args)
        for flag in ('--fine-stop-layered', '--fine-goal-behind', '--fine-pre-fallback',
                     '--fine-align-wz', '--position-settle-fresh', '--position-settle-samples'):
            self.assertIn(flag, command)
        self.assertTrue(nav.control.fine.fine_stop_layered)
        self.assertEqual(nav.control.fine.fine_align_wz, .2)
        nav.last_status = None
        with patch.object(nav.status, 'publish') as published:
            nav.publish_status()
        state = json.loads(published.call_args.args[0].data)
        self.assertEqual(state['navigation_options'], options)

    def test_v3_options_reach_real_child_and_entry_diagnostics(self):
        args = SimpleNamespace(config=ROOT/'fastlio/config/live_bev.yaml', map_dir=self.directory,
            run_dir=self.directory, record_stg_errors=False, fine_entry_align=True,
            fine_lateral_guard=True, fine_entry_align_deg=35., fine_entry_done_deg=9.,
            fine_entry_align_timeout=5., fine_lateral_max=.11)
        from g1_navigation_options import options_from_args
        live = LiveBEV(args); self.nodes.append(live)
        options = options_from_args(args)
        nav = NavigationNode(args.config, **options); self.nodes.append(nav)
        self.assertEqual(live.navigation_options, nav.navigation_options)
        command = navigation_command(ROOT, args)
        for flag in ('--fine-entry-align', '--fine-lateral-guard', '--fine-entry-align-deg',
                     '--fine-entry-done-deg', '--fine-entry-align-timeout', '--fine-lateral-max'):
            self.assertIn(flag, command)
        self.assertTrue(nav.control.fine.fine_entry_align)
        state = json.loads(nav.last_status)
        self.assertEqual(state['navigation_options'], options)
        self.assertEqual(state['fine_entry_diagnostics']['attempts'], 0)
        self.assertFalse(nav.control.active)

    def test_observation_switches_execute_actual_live_cloud_path(self):
        args=SimpleNamespace(config=ROOT/'fastlio/config/live_bev.yaml',map_dir=self.directory,
            run_dir=self.directory,ground_filter=True,speckle_filter=True,obstacle_mode='static_test')
        live=LiveBEV(args);self.nodes.append(live)
        live.active=True;live.transform=np.eye(4);live.accepted=True;live.health=lambda:''
        x,y=np.meshgrid(np.arange(-1.85,1.86,.1),np.arange(-1.85,1.86,.1))
        p=np.column_stack((x.ravel(),y.ravel(),.06+.006*x.ravel()))
        for i in range(15):
            t=100.+i*.1;m=Odometry();m.header.frame_id='camera_init';stamp(m.header,t)
            m.pose.pose.orientation.w=1.;live.on_odom(m)
            h=Header(frame_id='camera_init');stamp(h,t)
            live.on_cloud(create_cloud_xyz32(h,p.astype(np.float32)));live.tick()
        self.assertTrue(live.core.observation_filter.active)
        self.assertGreater(live.core.observation_filter.diagnostics['ground_removed_points'],100)
        self.assertTrue(live.core.task[live.core.no_go].all())
        np.testing.assert_array_equal(live.navigation_occupancy(),self.static|self.no_go)
        with patch.object(live.status,'publish') as published:live.publish_status()
        status=json.loads(published.call_args.args[0].data)
        self.assertTrue(status['observation_filters']['ground'])
        self.assertTrue(status['observation_filters']['speckles'])
        self.assertFalse(live.navigation_options['walk_guard'])

    def test_default_live_and_nav_use_dynamic_topic(self):
        live = LiveBEV(SimpleNamespace(config=ROOT/'fastlio/config/live_bev.yaml',
            map_dir=self.directory, run_dir=self.directory))
        self.nodes.append(live)
        nav = NavigationNode(ROOT/'fastlio/config/live_bev.yaml'); self.nodes.append(nav)
        self.assertEqual(live.obstacle_mode, 'dynamic')
        self.assertNotIn('static_test', live.maps)
        self.assertEqual(nav.navigation_map_topic, '/g1/bev/task')
        self.assertIn('/g1/bev/task', [s.topic_name for s in nav.subscriptions])
        self.assertFalse(nav.control.active)

    def test_static_source_is_exact_union_and_never_mutates(self):
        live, nav = self.pair()
        topics = [s.topic_name for s in nav.subscriptions]
        self.assertIn('/g1/bev/static_test', topics)
        self.assertNotIn('/g1/bev/task', topics)
        np.testing.assert_array_equal(nav.occupancy, self.static | self.no_go)
        self.assertFalse(nav.occupancy[80, 100])
        self.assertTrue(live.core.task[80, 100])
        with self.assertRaises(ValueError):
            live.navigation_occupancy()[80, 80] = True
        live.core.reset()
        np.testing.assert_array_equal(live.navigation_occupancy(), self.static | self.no_go)

    def test_real_619_static_source_preserves_all_fixed_policy_cells(self):
        args = SimpleNamespace(config=ROOT/'maps/619_go2base/runtime_live_bev.yaml',
            map_dir=ROOT/'maps/619_go2base', run_dir=self.directory, obstacle_mode='static_test')
        node = LiveBEV(args); self.nodes.append(node)
        self.assertEqual(node.core.no_go_cells, 87894)
        self.assertTrue(node.navigation_occupancy()[node.core.no_go].all())
        np.testing.assert_array_equal(node.navigation_occupancy(), node.core.static | node.core.no_go)

    def test_both_clis_expose_mode_without_initializing_ros(self):
        from contextlib import redirect_stdout
        from io import StringIO
        from live_bev_node import main as live_main
        from g1_navigation_node import main as nav_main
        for main in (live_main, nav_main):
            with self.subTest(main=main.__module__), patch.object(sys, 'argv', ['node', '--help']), \
                 patch('rclpy.init') as init, redirect_stdout(StringIO()) as output:
                with self.assertRaises(SystemExit) as caught:
                    main()
                self.assertEqual(caught.exception.code, 0)
                self.assertIn('--obstacle-mode {dynamic,static_test}', output.getvalue())
                init.assert_not_called()

    def test_static_corridor_and_pre_use_static_source(self):
        _, nav = self.pair()
        self.assertTrue(nav.clear((0., 0.), (2., 0.), math.inf))
        self.assertIsNotNone(nav.navigation_route((1.3, 0., 0.))['pre'])
        result = nav.plan((0., 0.), (2., 0.))
        self.assertIsNotNone(result.stg_xy)

    def test_dynamic_corridor_and_pre_still_block_realtime_point(self):
        _, nav = self.pair('dynamic')
        self.assertFalse(nav.clear((0., 0.), (2., 0.), math.inf))
        # Endpoints outside inflation, but the complete pre-to-goal line is blocked.
        route = nav.navigation_route((1.4, 0., 0.))
        self.assertIsNone(route['pre'])

    def test_static_wall_and_no_go_still_block(self):
        _, nav = self.pair()
        self.assertFalse(nav.clear((0., 0.), (0., 1.5), math.inf))
        self.pose(nav, 0., 1.)
        self.assertFalse(nav.start_status()['start_free'])
        nav.goal_pose = (2., 0., 0.)
        self.assertFalse(nav.start(None, Trigger.Response()).success)
        self.pose(nav, 3.6, 0.)
        self.assertFalse(nav.start_status()['start_free'])
        self.assertFalse(nav.start(None, Trigger.Response()).success)

    def test_running_current_cell_guard_still_stops_in_static_mode(self):
        _, nav = self.pair()
        nav.goal_pose = (2., 0., 0.)
        self.assertTrue(nav.start(None, Trigger.Response()).success)
        self.pose(nav, 0., 1.)
        nav.tick()
        self.assertFalse(nav.control.active)
        self.assertEqual(nav.control.reason, 'start_in_obstacle')
        self.assertEqual(json.loads(self.commands[-1].parameter)['velocity'], [0., 0., 0.])

    def test_start_requires_nonzero_genuine_fusion_stamp(self):
        live, nav = self.live(), self.nav()
        nav.on_map(self.packet(live)); self.pose(nav)
        self.assertIsNone(nav.occupancy)
        self.assertEqual(nav.control.map_time, -math.inf)
        nav.goal_pose = (2., 0., 0.)
        self.assertFalse(nav.start(None, Trigger.Response()).success)

    def test_republish_never_refreshes_stamp_or_navigation_age(self):
        live, nav = self.pair()
        when = nav.control.map_time
        msg = self.packet(live)
        time.sleep(.005)
        nav.on_map(self.packet(live))
        self.assertEqual(nav.control.map_time, when)
        self.assertEqual(self.packet(live).header.stamp, msg.header.stamp)
        self.assertIsNotNone(live.static_test_grid)

    def test_stale_map_and_stale_pose_still_reject_start(self):
        _, nav = self.pair()
        nav.goal_pose = (2., 0., 0.)
        nav.control.map_time -= 20.
        self.assertFalse(nav.start(None, Trigger.Response()).success)
        nav.control.map_time = time.monotonic()
        nav.control.pose_time -= 20.
        self.assertFalse(nav.start(None, Trigger.Response()).success)

    def test_reset_invalidates_selected_map(self):
        live, nav = self.pair()
        live.reset_map(None, Trigger.Response())
        nav.on_map(self.packet(live))
        self.assertEqual(nav.control.map_time, -math.inf)
        np.testing.assert_array_equal(live.navigation_occupancy(), self.static | self.no_go)

    def test_status_and_rviz_identify_source_without_rewriting_reason(self):
        live, nav = self.pair()
        nav.control.reason = 'test_reason'
        for node in (live, nav):
            with patch.object(node.status, 'publish') as published:
                node.publish_status()
            status = json.loads(published.call_args.args[0].data)
            self.assertEqual(status['obstacle_mode'], 'static_test')
            self.assertFalse(status['dynamic_obstacles_used'])
            self.assertIn(STATIC_TEST_WARNING, status['message'])
        self.assertEqual(nav.control.reason, 'test_reason')
        with patch.object(live.full_display['task'], 'publish') as published:
            live.publish_displays(force=True)
        markers = published.call_args.args[0].markers
        self.assertEqual(len(markers[1].points), int((self.static | self.no_go).sum()))
        self.assertEqual(markers[2].action, Marker.ADD)
        self.assertEqual(markers[2].text, STATIC_TEST_WARNING)

    def test_dynamic_rviz_deletes_previous_static_warning(self):
        live = self.live('dynamic')
        with patch.object(live.full_display['task'], 'publish') as published:
            live.publish_full_display('task', live.navigation_occupancy())
        self.assertEqual(published.call_args.args[0].markers[-1].action, Marker.DELETE)

    def test_preview_calls_real_planner_with_selected_source(self):
        live, _ = self.pair()
        live.active = True; live.transform = np.eye(4)
        live.odom.append((12., np.eye(4))); live.goal = dict(x=2., y=0.)
        from posnav_fmm.planner import plan_snapshot
        with patch.object(live, 'health', return_value=''), \
             patch('live_bev_node.plan_snapshot', wraps=plan_snapshot) as plan:
            result = live.compute_stg(None, Trigger.Response())
        self.assertTrue(result.success, result.message)
        np.testing.assert_array_equal(plan.call_args.args[0], self.static | self.no_go)

    def test_dynamic_shadow_still_fuses_via_real_callbacks(self):
        live, nav = self.live(), self.nav()
        def odom(t):
            m = Odometry(); m.header.frame_id = 'camera_init'; stamp(m.header, t)
            m.pose.pose.orientation.w = 1.; live.on_odom(m)
        def cloud(t):
            h = Header(frame_id='camera_init'); stamp(h, t)
            live.on_cloud(create_cloud_xyz32(h, np.array([[1., 0., .5], [1.01, 0., .6],
                                                         [0., 0., 0.]], np.float32)))
        for t in np.linspace(10., 12., 401):
            m = Imu(); stamp(m.header, float(t)); m.linear_acceleration.z = 1.; live.on_imu(m)
        for t in np.linspace(10., 12., 21):
            odom(float(t))
        cloud(12.)
        arrow = PoseWithCovarianceStamped(); arrow.header.frame_id = 'map'; arrow.pose.pose.orientation.w = 1.
        live.on_arrow(arrow)
        child = SimpleNamespace(proc=SimpleNamespace(poll=lambda: None), close=lambda: None)
        with patch('live_bev_node.Child', return_value=child):
            result = live.start(None, Trigger.Response())
            self.assertTrue(result.success, result.message)
        live.on_registration(String(data=json.dumps(dict(stamp=12., fitness=.99,
            accepted=True, transform=np.eye(4).ravel().tolist()))))
        for i in range(live.core.window-1):
            t = 12.1 + i*.1; odom(t); cloud(t); live.tick()
        self.assertIsNone(live.map_update_stamp)
        t = 13.0; odom(t); cloud(t); live.tick()
        self.assertGreater(live.updates, 0)
        self.assertTrue(live.core.task[80, 100])
        self.assertFalse(live.navigation_occupancy()[80, 100])
        nav.on_map(self.packet(live))
        self.assertTrue(math.isfinite(nav.control.map_time))
        # A rejected fit must not advance either map stamp or pose publication.
        previous = deepcopy(live.map_update_stamp)
        live.on_registration(String(data=json.dumps(dict(stamp=13., fitness=.1,
            accepted=False, transform=np.eye(4).ravel().tolist()))))
        odom(13.1); cloud(13.1)
        with patch.object(live.robot, 'publish') as poses:
            live.tick()
        self.assertEqual(live.map_update_stamp, previous)
        self.assertFalse(poses.called)
        self.assertEqual(self.packet(live).header.stamp, previous)

    def test_experiment_records_mode_and_selected_grid(self):
        from experiment_recorder import ExperimentSession
        session = ExperimentSession(ROOT, self.directory, ROOT/'fastlio/config/live_bev.yaml',
                                    Path(self.tmp.name)/'experiments')
        try:
            live = self.live(experiment=session)
            nav = self.nav(experiment_dir=session.path)
            live.map_update_stamp = live.get_clock().now().to_msg()
            nav.on_map(self.packet(live)); self.pose(nav)
            nav.goal_pose = (2., 0., 0.)
            self.assertTrue(nav.start(None, Trigger.Response()).success)
            nav.cancel('Stopped by user')
            nav.close_recordings()
            session.writer.queue.join()
            events = (session.path/'localization_events.jsonl').read_text()
            self.assertIn('obstacle_mode_selected', events)
            self.assertIn('static_test', events)
            self.assertEqual(nav.record_grid_metadata['navigation_map_topic'], '/g1/bev/static_test')
            np.testing.assert_array_equal(nav.record_grid != 0, self.static | self.no_go)
        finally:
            session.close()


if __name__ == '__main__':
    unittest.main()
