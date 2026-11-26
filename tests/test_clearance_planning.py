"""Cost propagation and actual plan_snapshot rollout, no robot interfaces."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.ndimage import distance_transform_edt
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from posnav_fmm.planner import plan_snapshot
from posnav_fmm.fmm_planner import FMMPlanner
from posnav_fmm.visibility import get_cost_stg, supercover_line
from posnav_fmm.errors import StartInObstacle


class ClearanceTest(unittest.TestCase):
    def setUp(self):
        self.grid = np.zeros((160, 160), bool)
        self.grid[70:100, 70:90] = True

    def test_zero_weight_is_exact_legacy_path_and_distance(self):
        args=(self.grid,[0,0],.05,[1.025,3.025],[6.025,3.025])
        a=plan_snapshot(*args); b=plan_snapshot(*args,clearance_weight=0.)
        np.testing.assert_array_equal(a.distance_m,b.distance_m)
        self.assertEqual(a.rollout_rc,b.rollout_rc)
        self.assertEqual(a.stg_xy,b.stg_xy)
        self.assertIsNone(b.cost_to_goal)

    def test_weighted_cost_retains_geometric_distance_and_hard_map(self):
        args=(self.grid,[0,0],.05,[1.025,3.025],[6.025,3.025])
        original=self.grid.copy(); a=plan_snapshot(*args); b=plan_snapshot(*args,clearance_weight=1.)
        np.testing.assert_array_equal(a.distance_m,b.distance_m)
        np.testing.assert_array_equal(a.traversible,b.traversible)
        np.testing.assert_array_equal(original,self.grid)
        self.assertIsNotNone(b.cost_to_goal)
        self.assertGreaterEqual(b.cost_to_goal[b.start_rc], b.distance_m[b.start_rc]-1e-7)

    def test_weight_does_not_allow_occupied_start_or_goal(self):
        for weight in (0.,1.,10.):
            with self.subTest(weight=weight), self.assertRaises(StartInObstacle):
                plan_snapshot(self.grid,[0,0],.05,[3.525,3.525],[6.025,3.025],clearance_weight=weight)
            with self.assertRaisesRegex(ValueError,'occupied'):
                plan_snapshot(self.grid,[0,0],.05,[1.025,3.025],[3.525,3.525],clearance_weight=weight)

    def test_invalid_cost_options(self):
        for options in [dict(clearance_weight=-1),dict(clearance_weight=float('nan')),
                        dict(clearance_weight=11),dict(clearance_safe_distance=0),
                        dict(clearance_safe_distance=float('inf')),dict(clip_stg_goal=1)]:
            with self.subTest(options=options), self.assertRaises(ValueError):
                plan_snapshot(self.grid,[0,0],.05,[1.025,3.025],[6.025,3.025],**options)

    def test_uniform_high_cost_does_not_trigger_legacy_gradient_rejection(self):
        p=FMMPlanner(np.ones((100,100),bool)); goals=np.zeros((100,100));goals[50,80]=1
        p.set_multi_goal(goals,cost=np.full((100,100),5.))
        r,c,replan,stop=p.get_short_term_goal((50,20),10)
        self.assertFalse(replan);self.assertGreater(c,20)

    def test_empty_map_does_not_invent_obstacle_clearance_penalty(self):
        p=plan_snapshot(np.zeros((120,120),bool),[0,0],.05,
                        [1.025,3.025],[5.025,3.025],clearance_weight=2.)
        np.testing.assert_allclose(p.cost_to_goal,p.distance_m,rtol=1e-10,atol=1e-10)

    def test_full_candidate_loop_completes_with_ten_hz_sensor_samples(self):
        from g1_final_approach import PosNav
        from g1_navigation_core import velocity_xyz
        from g1_walk_guard import WalkGuardConfig
        grid=np.zeros((160,160),bool); goal=(5.,4.,1.2)
        def route(g):
            return dict(pre=(g[0]-.6*math.cos(g[2]),g[1]-.6*math.sin(g[2])),reason='clear')
        n=PosNav(lambda p,g:plan_snapshot(grid,[0,0],.05,p,g,
                    clearance_weight=1.,clip_stg_goal=True),lambda *_:True,route,
                    walk_guard=WalkGuardConfig())
        physical=np.array([1.,4.,-.8]); n.pose=tuple(physical); n.pose_time=n.map_time=0.
        n.start(goal,0.);states=set()
        for i in range(10000):
            t=i*.02
            if i%5==0:
                n.pose=tuple(physical);n.pose_time=n.map_time=t
            command=n.tick(t);states.add(n.state)
            if not n.active:break
            vx,vy,w=velocity_xyz(command)
            c,s=math.cos(physical[2]),math.sin(physical[2])
            physical+=np.array([c*vx-s*vy,s*vx+c*vy,w])*.02
        self.assertFalse(n.active,n.state)
        self.assertEqual(n.reason,'Final approach completed')
        self.assertIn('guard_align',states)
        self.assertLess(n.final_error_m,.05)

    def test_macro_goal_clip_avoids_20cm_overshoot_and_marks_final(self):
        grid=np.zeros((120,120),bool); args=(grid,[0,0],.05,[1.025,3.025],[2.825,3.025])
        legacy=plan_snapshot(*args); clipped=plan_snapshot(*args,clip_stg_goal=True)
        self.assertAlmostEqual(legacy.stg_xy[0]-2.825,.2)
        self.assertEqual(clipped.stg_xy,(2.825,3.025))
        self.assertTrue(clipped.final_stg)
        self.assertFalse(legacy.final_stg)

    def test_clip_does_not_shortcut_through_obstacle(self):
        self.grid[60,40]=True
        p=plan_snapshot(self.grid,[0,0],.05,[1.025,3.025],[3.025,3.025],clip_stg_goal=True)
        col,row=np.floor(np.asarray(p.stg_xy)/.05).astype(int)
        end=(row,col)  # Check the selected executable chord, not discarded rollout suffixes.
        for r,c in supercover_line(p.start_rc,end):self.assertTrue(p.traversible[r,c])

    def test_expensive_chord_is_not_selected_just_because_visible(self):
        free=np.ones((40,40),bool);cost=np.ones_like(free,dtype=float)
        cost[10,11:19]=5.; field=np.full_like(cost,100.)
        path=[(10,10),(15,10),(15,20),(10,20)]
        field[15,10]=15.;field[15,20]=5.;field[10,20]=0.
        self.assertNotEqual(get_cost_stg(path,free,cost,field),path[-1])


if __name__ == '__main__':unittest.main()
