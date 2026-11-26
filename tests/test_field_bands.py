"""Field stop/planning split, endpoint connectivity and real controller wiring."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from skimage.morphology import binary_dilation,disk

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from g1_navigation_options import add_navigation_options,options_from_args,forward_navigation_options,checked_options
from g1_motion_policy import effective_planning_config,band_planning_options
from g1_navigation_node import NavigationNode
from posnav_fmm.planner import plan_snapshot
from posnav_fmm.errors import StartInObstacle,GoalUnreachable
from posnav_fmm.visibility import line_is_collision_free,segment_clearance


class OptionsTest(unittest.TestCase):
    def test_new_options_roundtrip_zero_and_default(self):
        p=argparse.ArgumentParser();add_navigation_options(p)
        for args in ([],['--fmm-inflation-cells','7','--fmm-stop-inflation-cells','4','--allow-goal-in-inflation'],
                     ['--fmm-stop-inflation-cells','0']):
            with self.subTest(args=args):
                a=p.parse_args(args);child=forward_navigation_options(a)
                self.assertEqual(options_from_args(a),options_from_args(p.parse_args(child)))
                if '0' in args:self.assertIn('0',child)
        self.assertEqual(forward_navigation_options(p.parse_args([])),[])

    def test_invalid_bands_fail_before_runtime(self):
        for bad in (dict(fmm_stop_inflation_cells=-1),dict(fmm_stop_inflation_cells=True),
                    dict(fmm_stop_inflation_cells=4.5),dict(allow_goal_in_inflation=1),
                    dict(fmm_inflation_cells=4,fmm_stop_inflation_cells=7)):
            with self.subTest(bad=bad),self.assertRaises(ValueError):checked_options(**bad)
        cfg=dict(fmm_inflation_cells=5,fmm_step_cells=10,fmm_min_step_cells=4,fmm_max_stgs=4)
        with self.assertRaises(ValueError):effective_planning_config(cfg,checked_options(fmm_stop_inflation_cells=6))
        followed=effective_planning_config(cfg,checked_options(fmm_inflation_cells=7))
        self.assertEqual(followed['fmm_stop_inflation_cells'],7)
        self.assertEqual(band_planning_options(followed,checked_options()),{})


class PlannerTest(unittest.TestCase):
    def setUp(self):
        self.grid=np.zeros((100,100),bool);self.grid[:,20]=True
        self.band=(1.325,2.025);self.normal=(2.025,2.025)
        self.opts=dict(inflation_cells=7,stop_inflation_cells=4,allow_start_in_inflation=True,
                       allow_goal_in_inflation=True,clip_stg_goal=True,stg_clearance_m=.4)

    def test_real_start_and_goal_counterexamples_connect_without_clearing_stop_band(self):
        original=self.grid.copy();self.grid.flags.writeable=False
        for start,goal,kind in [(self.band,self.normal,'start'),(self.normal,self.band,'goal')]:
            with self.subTest(kind=kind):
                result=plan_snapshot(self.grid,[0,0],.05,start,goal,**self.opts)
                self.assertIsNotNone(result.stg_xy)
                self.assertEqual(result.inflation_connections[0]['endpoint'],kind)
                stop=~binary_dilation(self.grid,disk(4))
                self.assertFalse(np.any(result.traversible & ~stop))
                for c in result.inflation_connections:
                    self.assertLessEqual(c['length_m'],c['max_length_m']+1e-12)
                    self.assertTrue(c['opened_cells_rc'])
                    for p in c['opened_cells_rc']:self.assertTrue(stop[tuple(p)])
                if kind=='start':
                    self.assertEqual(result.stg_selection['selected_macro_index'],1)
                    self.assertTrue(result.stg_selection['first_macro_exception'])
                    self.assertGreaterEqual(np.linalg.norm(np.asarray(result.stg_xy)-start),.2-1e-12)
                    self.assertGreater(segment_clearance(np.asarray(start)[::-1]/.05,
                        np.asarray(result.stg_xy)[::-1]/.05,~stop,.05,.05),0)
        np.testing.assert_array_equal(original,self.grid)

    def test_permissions_and_stop_band_cannot_be_bypassed(self):
        for start,goal,overrides,error in [
            (self.band,self.normal,dict(allow_start_in_inflation=False),StartInObstacle),
            (self.normal,self.band,dict(allow_goal_in_inflation=False),ValueError),
            ((1.175,2.025),self.normal,{},StartInObstacle),
            (self.normal,(1.175,2.025),{},ValueError),
            ((1.025,2.025),self.normal,dict(stop_inflation_cells=0),StartInObstacle),
            (self.normal,(1.025,2.025),dict(stop_inflation_cells=0),ValueError)]:
            with self.subTest(start=start,goal=goal),self.assertRaises(error):
                plan_snapshot(self.grid,[0,0],.05,start,goal,**dict(self.opts,**overrides))
        with self.assertRaises(StartInObstacle):
            plan_snapshot(self.grid,[0,0],.05,self.band,self.normal,inflation_cells=7,allow_start_in_inflation=True)

    def test_narrow_space_without_planning_anchor_rejects(self):
        grid=np.zeros((100,100),bool);grid[:,20]=True;grid[:,32]=True
        with self.assertRaisesRegex(GoalUnreachable,'connection_unavailable'):
            plan_snapshot(grid,[0,0],.05,(1.325,2.025),(1.325,3.025),**self.opts)

    def test_subcell_endpoints_and_out_of_bounds_are_checked(self):
        for xy in [(1.301,2.001),(1.349,2.049)]:
            result=plan_snapshot(self.grid,[0,0],.05,xy,self.normal,**self.opts)
            self.assertIsNotNone(result.stg_xy)
            self.assertEqual(result.inflation_connections[0]['endpoint_xy'],list(xy))
        for xy in [(-.01,2.),(5.,2.),(float('nan'),2.)]:
            with self.assertRaises(ValueError):plan_snapshot(self.grid,[0,0],.05,xy,self.normal,**self.opts)

    def test_normal_grid_and_numerical_results_match_field_baseline(self):
        from frozen_baselines import baseline_path
        field=(Path(os.environ['G1_FIELD_SNAPSHOT'])/'posnav_fmm/planner.py'
               if os.environ.get('G1_FIELD_SNAPSHOT') else baseline_path('G1_FIELD_BASELINE', 'field_planner.py'))
        spec=importlib.util.spec_from_file_location('posnav_fmm._field_baseline',field)
        module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
        for weight in (0.,1.):
            for gate in (0.,.4):
                with self.subTest(weight=weight,gate=gate):
                    opts=dict(inflation_cells=7,clearance_weight=weight,clip_stg_goal=True,stg_clearance_m=gate)
                    a=module.plan_snapshot(self.grid,[0,0],.05,self.normal,(2.025,4.025),**opts)
                    b=plan_snapshot(self.grid,[0,0],.05,self.normal,(2.025,4.025),**opts)
                    for key in ('stg_xy','rollout_rc','reason','stopped','final_stg','stg_selection'):
                        self.assertEqual(getattr(a,key),getattr(b,key))
                    for key in ('distance_m','cost_to_goal','traversible'):
                        np.testing.assert_array_equal(getattr(a,key),getattr(b,key))


class ActualNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID','0')=='0' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise RuntimeError('Require isolated ROS domain')
        rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):self.nodes=[];self.temp=tempfile.TemporaryDirectory()
    def tearDown(self):
        for n in self.nodes:n.close_recordings();n.destroy_node()
        self.temp.cleanup()
    def node(self,**kwargs):
        n=NavigationNode(ROOT/'fastlio/config/live_bev.yaml',fmm_inflation_cells=7,
            fmm_stop_inflation_cells=4,**kwargs);self.nodes.append(n)
        m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg()
        m.info.width=m.info.height=100;m.info.resolution=.05;m.info.origin.orientation.w=1.
        occ=np.zeros((100,100),np.int8);occ[:,20]=100;m.data=occ.ravel().tolist();n.on_map(m)
        n.control.pose=(2.025,2.025,0.)
        return n

    def test_goal_allowance_reaches_route_and_fine_and_rejects_real_stop_band(self):
        for enabled in (False,True):
            n=self.node(allow_goal_in_inflation=enabled)
            self.assertIs(n.control.allow_goal_in_inflation,enabled)
            n.control.final_goal=(1.325,2.025,0.)
            if enabled:self.assertIsNone(n.navigation_route(n.control.final_goal)['pre'])
            else:
                with self.assertRaises(ValueError):n.navigation_route(n.control.final_goal)
            self.assertIs(n.control._goal_clear(),enabled)
            n.control.final_goal=(1.175,2.025,0.)
            self.assertFalse(n.control._goal_clear())
            with self.assertRaises(ValueError):n.navigation_route(n.control.final_goal)

    def test_normal_goal_fine_passage_uses_stop_band_for_admitted_start(self):
        n=self.node(allow_goal_in_inflation=False);n.control.pose=(1.325,2.025,0.)
        n.control.final_goal=(2.025,2.025,0.)
        self.assertTrue(n.control._goal_clear())
        self.assertFalse(n.clear(n.control.pose[:2],n.control.final_goal[:2],float('inf')))

    def test_constructor_records_effective_parameters_before_first_map(self):
        (Path(self.temp.name)/'session.json').write_text(json.dumps(dict(
            session_id='field_bands_test',monotonic_origin=time.monotonic())))
        n=self.node(allow_goal_in_inflation=True,experiment_dir=Path(self.temp.name))
        n.experiment.writer.queue.join()
        params=json.loads((Path(self.temp.name)/'assets/config/navigation_parameters.json').read_text())
        runtime=params['runtime_parameters']
        self.assertAlmostEqual(runtime['stop_check_inflation_m'],.2)
        self.assertAlmostEqual(runtime['planning_check_inflation_m'],.35)
        self.assertTrue(params['navigation_options']['allow_goal_in_inflation'])
        self.assertEqual(params['navigation_options']['fmm_stop_inflation_cells'],4)
        self.assertEqual(runtime['stop_check_scope'],'entire_stg_segment')

    def test_runtime_status_connections_and_whole_segment_horizon(self):
        n=self.node(allow_goal_in_inflation=True,stg_clearance_m=.4)
        n.control.pose=(1.325,2.025,0.)
        n.require_start_free();n.plan(n.control.pose[:2],(2.025,2.025));n.publish_status()
        status=json.loads(n.last_status)
        self.assertTrue(status['start_free']);self.assertFalse(status['planning_start_free'])
        self.assertTrue(status['start_requires_connector'])
        self.assertTrue(status['inflation_connections'])
        n.occupancy[:]=False;n.occupancy[70,40]=True
        from g1_motion_policy import inflation_masks
        n.free,n.stop_free=inflation_masks(n.occupancy,n.cfg)
        self.assertTrue(n.clear((2.025,2.025),(2.025,4.025),1.))
        self.assertFalse(n.control.passage_clear((2.025,2.025),(2.025,4.025)))

    def test_connector_enters_real_coarse_walk_with_original_arrival_and_near_gate(self):
        n=self.node(allow_goal_in_inflation=True,stg_clearance_m=.4,walk_guard=True,
                    near_stg_no_realign=True,coarse_turn_profile=True)
        c=n.control;c.pose=(1.325,2.025,0.);c.pose_time=c.map_time=0.
        c.start((3.025,2.025,0.),0.)
        moving=False
        for t in np.arange(0.,1.,.02):
            c.pose_time=c.map_time=float(t)
            command=c.tick(float(t))
            if command[0]>0:
                moving=True;break
        self.assertTrue(moving,(c.state,c.reason,c.stg))
        self.assertEqual(c.epsilon,.15)
        self.assertEqual(n.last_inflation_connections[0]['endpoint'],'start')
        self.assertGreater(c.remaining(),c.epsilon)

    def test_connector_admission_is_specific_and_invalidated_by_map_updates(self):
        n=self.node(allow_goal_in_inflation=True,stg_clearance_m=.4)
        start=(1.325,2.025);p=n.plan(start,(3.025,2.025))
        self.assertTrue(n.control.stg_segment_clear(start,p.stg_xy))
        self.assertFalse(n.control.stg_segment_clear(start,(p.stg_xy[0],p.stg_xy[1]+.3)))
        self.assertFalse(n.control.stg_segment_clear((start[0]+.001,start[1]),p.stg_xy))
        n.map_stamp+=1
        self.assertFalse(n.control.stg_segment_clear(start,p.stg_xy))
        n.map_stamp-=1
        n.stop_free[n.cell(p.stg_xy)]=False
        self.assertFalse(n.control.stg_segment_clear(start,p.stg_xy))
        m=OccupancyGrid();m.header.frame_id='map'
        m.header.stamp.sec=n.map_stamp//1000000000;m.header.stamp.nanosec=n.map_stamp%1000000000
        m.info.width=m.info.height=100;m.info.resolution=.05;m.info.origin.orientation.w=1.
        m.data=np.zeros(10000,dtype=np.int8).tolist()
        n.on_map(m)
        self.assertIsNone(n.stg_corridor)


if __name__=='__main__':unittest.main()
