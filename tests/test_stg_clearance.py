"""Hard STG chord clearance through geometry, selection and plan_snapshot."""
import argparse
import importlib
import importlib.util
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
from skimage.morphology import binary_dilation,disk

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from posnav_fmm.planner import plan_snapshot,_ClearanceGate
from posnav_fmm.visibility import segment_clearance,get_stg,get_cost_stg
from posnav_fmm.fmm_planner import FMMPlanner
from posnav_fmm.errors import NoVisibleSTG,StartInObstacle
from g1_navigation_options import add_navigation_options,options_from_args,forward_navigation_options,planning_options,checked_options


class GeometryTest(unittest.TestCase):
    def test_cell_edges_and_exact_forty_cm_not_cell_centres(self):
        grid=np.zeros((100,100),bool);grid[:,20]=True
        for x,expected in ((.599,.401),(.600,.400),(.601,.399)):
            with self.subTest(x=x):
                actual=segment_clearance((20.5,x/.05),(60.5,x/.05),grid,.05,1.)
                self.assertAlmostEqual(actual,expected,places=12)

    def test_intersections_corners_and_zero_length(self):
        grid=np.zeros((100,100),bool);grid[20,20]=True
        for a,b in (((20.5,10.),(20.5,30.)),((10.,30.),(30.,10.)),((20.5,20.5),(20.5,20.5))):
            with self.subTest(a=a,b=b):self.assertEqual(segment_clearance(a,b,grid,.05,1.),0.)
        self.assertAlmostEqual(segment_clearance((30.,30.),(30.,30.),grid,.05,1.),math.sqrt(162)*.05)

    def test_oblique_exact_clearance_and_reversal(self):
        grid=np.zeros((100,100),bool);grid[20,20]=True
        total=40.-8.*math.sqrt(2.)
        a=(10.,total-10.);b=(total-10.,10.)
        self.assertAlmostEqual(segment_clearance(a,b,grid,.05,1.),.4,places=12)
        self.assertAlmostEqual(segment_clearance(b,a,grid,.05,1.),.4,places=12)

    def test_whole_chord_including_start_and_map_boundary(self):
        grid=np.zeros((100,100),bool)
        self.assertAlmostEqual(segment_clearance((20.,6.),(60.,6.),grid,.05,.4),.3)
        self.assertEqual(segment_clearance((20.,-1.),(60.,6.),grid,.05,.4),0.)
        grid[20,20]=True
        self.assertAlmostEqual(segment_clearance((20.5,13.),(20.5,5.),grid,.05,.4),.25)

    def test_distance_agrees_with_independent_dense_point_queries(self):
        grid=np.zeros((100,100),bool);grid[40,40]=True
        rng=np.random.default_rng(426)
        for _ in range(16):
            a,b=rng.uniform(28,52,(2,2))
            points=np.linspace(a,b,5001)
            d=np.maximum(np.maximum(40.-points,points-41.),0.)
            sampled=float(np.linalg.norm(d,axis=1).min())*.05
            exact=segment_clearance(a,b,grid,.05,3.)
            spacing=float(np.linalg.norm(b-a))*.05/5000
            self.assertLessEqual(exact,sampled+1e-12)
            self.assertGreaterEqual(exact,sampled-spacing/2-1e-12)


class SelectionTest(unittest.TestCase):
    def gate(self,grid,start=(20,20),inflation=5):
        free=~binary_dilation(grid,disk(inflation));state=(start[0]+1,start[1]+1)
        gate=_ClearanceGate(grid,free,np.pad(free,1,constant_values=True),state,np.zeros(2),.05,
            ((start[1]+.5)*.05,(start[0]+.5)*.05),(4.,4.),(81,81),False,.4)
        return gate

    def test_endpoints_clear_but_middle_grazes_and_soft_cost_cannot_override(self):
        grid=np.zeros((100,100),bool);grid[27,40]=True;gate=self.gate(grid)
        path=[(21,21),(21,31),(21,61)]
        start=np.array((20.5,20.5));end=np.array((20.5,60.5))
        self.assertEqual(segment_clearance(start,start,grid,.05,.4),.4)
        self.assertEqual(segment_clearance(end,end,grid,.05,.4),.4)
        self.assertAlmostEqual(segment_clearance(start,end,grid,.05,.4),.325)
        self.assertEqual(get_stg(path,gate.padded),path[-1])
        self.assertEqual(get_stg(path,gate.padded,candidate_filter=gate.allows),path[1])
        cost=np.ones(gate.padded.shape);field=np.full_like(cost,1000.)
        field[path[-1]]=0.
        self.assertEqual(get_cost_stg(path,gate.padded,cost,field),path[-1])
        self.assertEqual(get_cost_stg(path,gate.padded,cost,field,candidate_filter=gate.allows),path[1])

    def test_blocked_first_does_not_make_second_the_clearance_exception(self):
        grid=np.zeros((100,100),bool);grid[20,30]=True;gate=self.gate(grid,inflation=0)
        path=[(21,21),(21,41),(41,41)]
        first=gate.check(1,path[1]);later=gate.check(2,path[2])
        self.assertFalse(first['hard_clear'])
        self.assertTrue(later['hard_clear']);self.assertFalse(later['clearance_ok'])
        self.assertIsNone(get_stg(path,gate.padded,candidate_filter=gate.allows))
        cost=np.ones(gate.padded.shape)
        self.assertIsNone(get_cost_stg(path,gate.padded,cost,cost,candidate_filter=gate.allows))

    def test_subcell_start_is_used_instead_of_rounded_cell_centre(self):
        grid=np.zeros((100,100),bool);grid[:,20]=True
        free=~binary_dilation(grid,disk(5))
        for x,eligible in ((.599,True),(.600,True),(.601,False)):
            with self.subTest(x=x):
                gate=_ClearanceGate(grid,free,np.pad(free,1,constant_values=True),(21,12),np.zeros(2),.05,
                    (x,1.025),(x,4.),(81,12),False,.4)
                self.assertEqual(gate.check(2,(41,12))['eligible'],eligible)


class PlannerTest(unittest.TestCase):
    def corridor(self,wide=False):
        grid=np.zeros((140,140),bool)
        walls=(40,70) if wide else (50,65)
        grid[5:130,list(walls)]=True
        x=2.775 if wide else 2.925
        return grid,(x,1.025),(x,5.025)

    def test_narrow_corridor_selects_only_first_with_both_cost_branches(self):
        grid,start,goal=self.corridor();original=grid.copy();grid.flags.writeable=False
        for weight in (0.,1.,3.):
            with self.subTest(weight=weight):
                old=plan_snapshot(grid,[0,0],.05,start,goal,clearance_weight=weight)
                new=plan_snapshot(grid,[0,0],.05,start,goal,clearance_weight=weight,stg_clearance_m=.4)
                self.assertGreater(old.stg_xy[1],new.stg_xy[1]+1.)
                s=new.stg_selection
                self.assertEqual(s['selected_macro_index'],1);self.assertTrue(s['first_macro_exception'])
                self.assertEqual(s['fallback_reason'],'first_stg_only')
                self.assertAlmostEqual(s['line_clearance_m_capped'],.325)
                self.assertTrue(all(not row['eligible'] for row in s['candidates'][1:]))
                np.testing.assert_array_equal(new.traversible,old.traversible)
                np.testing.assert_array_equal(new.distance_m,old.distance_m)
                if weight:np.testing.assert_array_equal(new.cost_to_goal,old.cost_to_goal)
                self.assertEqual(new.rollout_rc,old.rollout_rc)
        np.testing.assert_array_equal(grid,original)

    def test_wide_corridor_keeps_ranking_and_long_chords(self):
        grid,start,goal=self.corridor(wide=True)
        for weight in (0.,3.):
            old=plan_snapshot(grid,[0,0],.05,start,goal,clearance_weight=weight)
            new=plan_snapshot(grid,[0,0],.05,start,goal,clearance_weight=weight,stg_clearance_m=.4)
            self.assertEqual(new.stg_xy,old.stg_xy)
            self.assertEqual(new.stg_selection['selected_macro_index'],4)
            self.assertFalse(new.stg_selection['first_macro_exception'])

    def test_exact_threshold_and_start_prefix_not_skipped(self):
        grid=np.zeros((100,100),bool);grid[:,20]=True
        for x,selected in ((.599,4),(.600,4),(.601,1)):
            with self.subTest(x=x):
                plan=plan_snapshot(grid,[0,0],.05,(x,1.025),(x,4.025),stg_clearance_m=.4)
                self.assertEqual(plan.stg_selection['selected_macro_index'],selected)

    def test_direct_goal_is_first_macro_but_later_clipped_goal_cannot_bypass(self):
        grid,start,_=self.corridor()
        direct=plan_snapshot(grid,[0,0],.05,start,(start[0]+.003,start[1]+.2),stg_clearance_m=.4)
        self.assertTrue(direct.final_stg)
        self.assertEqual(direct.stg_selection['kind'],'direct_goal')
        self.assertTrue(direct.stg_selection['first_macro_exception'])
        self.assertEqual(direct.stg_xy,(start[0]+.003,start[1]+.2))
        target=(start[0],start[1]+1.8)
        far=plan_snapshot(grid,[0,0],.05,start,target,clip_stg_goal=True,stg_clearance_m=.4)
        self.assertFalse(far.final_stg)
        self.assertEqual(far.stg_selection['selected_macro_index'],1)
        self.assertFalse(far.stg_selection['candidates'][-1]['eligible'])

    def test_radius_retry_keeps_hard_check_and_first_only_exception(self):
        grid=np.zeros((120,120),bool);grid[48:58,27]=True
        p=plan_snapshot(grid,[0,0],.05,[1.025,3.025],[5.025,3.025],stg_clearance_m=.4)
        self.assertIn('search radius',p.reason)
        self.assertEqual(len(p.rollout_rc),2)
        self.assertEqual(p.stg_selection['selected_macro_index'],1)
        self.assertTrue(p.stg_selection['candidates'][0]['hard_clear'])
        with patch.object(FMMPlanner,'get_short_term_goal',return_value=(63,31,False,False)):
            with self.assertRaises(NoVisibleSTG):
                plan_snapshot(grid,[0,0],.05,[1.025,3.025],[5.025,3.025],max_stgs=1,stg_clearance_m=.4)

    def test_start_and_goal_rejection_are_not_relaxed(self):
        grid,start,goal=self.corridor()
        with self.assertRaises(StartInObstacle):plan_snapshot(grid,[0,0],.05,(2.525,1.025),goal,stg_clearance_m=.4)
        with self.assertRaisesRegex(ValueError,'occupied'):plan_snapshot(grid,[0,0],.05,start,(2.525,5.025),stg_clearance_m=.4)

    def test_disabled_gate_is_identical_to_frozen_round_one(self):
        from frozen_baselines import baseline_path
        directory=baseline_path('G1_STG_BASELINE', 'round1_posnav_fmm')
        name='round1_posnav_fmm'
        spec=importlib.util.spec_from_file_location(name,directory/'__init__.py',submodule_search_locations=[str(directory)])
        module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
        baseline=importlib.import_module(name+'.planner').plan_snapshot
        for wide in (False,True):
            grid,start,goal=self.corridor(wide)
            for weight in (0.,3.):
                for clip in (False,True):
                    for step in (10,20):
                        with self.subTest(wide=wide,weight=weight,clip=clip,step=step):
                            options=dict(clearance_weight=weight,clip_stg_goal=clip,step_cells=step)
                            old=baseline(grid,[0,0],.05,start,goal,**options)
                            new=plan_snapshot(grid,[0,0],.05,start,goal,stg_clearance_m=0.,**options)
                            for key in ('stg_xy','rollout_rc','final_stg','reason','start_rc','goal_rc','stopped'):
                                self.assertEqual(getattr(old,key),getattr(new,key))
                            for key in ('distance_m','traversible','cost_to_goal'):
                                np.testing.assert_array_equal(getattr(old,key),getattr(new,key))
                            self.assertIsNone(new.stg_selection)

    def test_cli_validation_and_shared_forwarding(self):
        parser=argparse.ArgumentParser();add_navigation_options(parser)
        args=parser.parse_args(['--stg-clearance-m','0.40'])
        options=options_from_args(args)
        child=parser.parse_args(forward_navigation_options(args))
        self.assertEqual(options_from_args(child),options)
        self.assertEqual(planning_options(options)['stg_clearance_m'],.4)
        self.assertEqual(planning_options(checked_options()),{})
        for value in (-.4,float('nan'),float('inf'),True,2.01):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):checked_options(stg_clearance_m=value)
                with self.assertRaises(ValueError):plan_snapshot(self.corridor()[0],[0,0],.05,(1.,1.),(2.,2.),stg_clearance_m=value)


if __name__=='__main__':unittest.main()
