import sys,unittest
from pathlib import Path
import numpy as np
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from posnav_fmm.planner import plan_snapshot
from posnav_fmm.visibility import supercover_line,line_is_collision_free,get_stg
from posnav_fmm.errors import GoalUnreachable, NoVisibleSTG
from posnav_fmm.fmm_planner import FMMPlanner
from skimage.morphology import binary_dilation,disk

class FMMTest(unittest.TestCase):
    def test_occupied_or_inflated_start_is_not_cleared(self):
        for obstacle in [(60,20),(65,20)]:
            with self.subTest(obstacle=obstacle):
                grid=np.zeros((120,120),bool);grid[obstacle]=True
                original=grid.copy()
                with self.assertRaisesRegex(GoalUnreachable,'unreachable'):
                    plan_snapshot(grid,[0,0],.05,[1.025,3.025],[5.025,3.025])
                np.testing.assert_array_equal(grid,original)

    def test_free_start_preserves_inflated_neighbor_cells(self):
        grid=np.zeros((120,120),bool);grid[66,20]=True
        expected=~binary_dilation(grid,disk(5))
        self.assertTrue(expected[60,20]);self.assertFalse(expected[61,20])
        p=plan_snapshot(grid,[0,0],.05,[1.025,3.025],[5.025,3.025])
        np.testing.assert_array_equal(p.traversible,expected)
        self.assertIsNotNone(p.stg_xy)

    def test_open_space_rollout_and_single_goal(self):
        grid=np.zeros((120,120),bool); original=grid.copy()
        p=plan_snapshot(grid,[0,0],.05,[1.025,3.025],[5.025,3.025])
        self.assertEqual(len(p.rollout_rc),5)
        self.assertEqual(p.goal_rc,(60,100))
        self.assertEqual(np.count_nonzero(p.distance_m==0),1)
        self.assertAlmostEqual(p.stg_xy[0],3.025)
        self.assertAlmostEqual(p.stg_xy[1],3.025)
        self.assertFalse(p.stopped)
        self.assertAlmostEqual(p.distance_m[p.start_rc],4.,delta=.1)
        np.testing.assert_array_equal(grid,original)
    def test_inflation_and_failure_cases(self):
        grid=np.zeros((120,120),bool);grid[60,60]=True
        p=plan_snapshot(grid,[0,0],.05,[1,1],[4,1])
        self.assertFalse(p.traversible[60,65]);self.assertTrue(p.traversible[60,66])
        with self.assertRaisesRegex(ValueError,'occupied'):plan_snapshot(grid,[0,0],.05,[1,1],[3,3])
        with self.assertRaisesRegex(ValueError,'outside'):plan_snapshot(grid,[0,0],.05,[1,1],[7,3])
        grid[30:91,30]=True;grid[30:91,90]=True;grid[30,30:91]=True;grid[90,30:91]=True
        with self.assertRaisesRegex(ValueError,'unreachable'):plan_snapshot(grid,[0,0],.05,[1,1],[3.5,3.5])
    def test_stop_distance_unchanged(self):
        p=plan_snapshot(np.zeros((60,60),bool),[0,0],.05,[1.025,1.025],[1.125,1.025])
        self.assertTrue(p.stopped);self.assertIsNone(p.stg_xy)
    def test_supercover_corner_and_furthest_visible(self):
        free=np.ones((30,30),bool);free[11,12]=False
        self.assertIn((11,12),supercover_line((11,11),(12,12)))
        self.assertFalse(line_is_collision_free((11,11),(12,12),free))
        path=[(10,10),(10,15),(15,15)]
        free[12,12]=False
        self.assertEqual(get_stg(path,free),(10,15))
    def test_row_col_and_negative_origin(self):
        p=plan_snapshot(np.zeros((120,120),bool),[-2,-3],.05,[-.975,-1.975],[-.975,2.025])
        self.assertAlmostEqual(p.stg_xy[0],-.975)
        self.assertAlmostEqual(p.stg_xy[1],.025)

    def test_first_macro_candidate_must_be_visible(self):
        free=np.ones((30,30),bool);free[10,12]=False
        self.assertIsNone(get_stg([(10,10),(10,15)],free))
        self.assertIsNone(get_stg([(10,10),(10,15),(10,20)],free))

    def test_wall_corner_shrinks_in_one_snapshot(self):
        grid=np.zeros((120,120),bool);grid[48:58,27]=True
        calls=[]
        original=FMMPlanner.get_short_term_goal
        def observed(planner,state,radius):
            calls.append((tuple(state),radius))
            return original(planner,state,radius)
        with patch.object(FMMPlanner,'get_short_term_goal',observed), \
             patch.object(FMMPlanner,'set_multi_goal',autospec=True,
                          side_effect=FMMPlanner.set_multi_goal) as field:
            p=plan_snapshot(grid,[0,0],.05,[1.025,3.025],[5.025,3.025])
        field.assert_called_once()
        self.assertEqual(calls[-3:],[((61,21),9),((61,21),8),((61,21),7)])
        self.assertEqual(len(p.rollout_rc),2)  # Short target is not extended by rollout.
        np.testing.assert_allclose(p.stg_xy,[1.325,3.175])
        self.assertTrue(line_is_collision_free(p.start_rc,p.rollout_rc[-1],p.traversible))
        self.assertIn('0.35 m',p.reason)
        # A later open-space segment starts again at the normal radius.
        q=plan_snapshot(np.zeros_like(grid),[0,0],.05,[1.025,3.025],[5.025,3.025])
        self.assertAlmostEqual(q.stg_xy[0],3.025)

    def test_replan_exhausts_radii_once_without_10cm_fallback(self):
        calls=[]
        def failed(planner,state,radius):
            calls.append((tuple(state),radius))
            return (*state,True,False)
        with patch.object(FMMPlanner,'get_short_term_goal',failed):
            with self.assertRaises(NoVisibleSTG):
                plan_snapshot(np.zeros((120,120),bool),[0,0],.05,
                              [1.025,3.025],[5.025,3.025])
        self.assertEqual(calls,[((61,21),r) for r in range(10,3,-1)])

    def test_invisible_best_candidate_does_not_try_other_candidates(self):
        grid=np.zeros((120,120),bool);grid[48:58,27]=True
        calls=[]
        def invisible(planner,state,radius):
            calls.append((tuple(state),radius))
            # This valid endpoint has a blocked chord; other visible choices
            # exist on the same map but must not be searched at this radius.
            return (63,31,False,False)
        with patch.object(FMMPlanner,'get_short_term_goal',invisible):
            with self.assertRaises(NoVisibleSTG):
                plan_snapshot(grid,[0,0],.05,[1.025,3.025],[5.025,3.025],max_stgs=1)
        self.assertEqual(calls,[((61,21),r) for r in range(10,3,-1)])

    def test_min_radius_success_without_minimum_actual_distance(self):
        calls=[]
        def choose(planner,state,radius):
            calls.append(radius)
            return (state[0],state[1]+radius,radius>4,False)
        with patch.object(FMMPlanner,'get_short_term_goal',choose):
            p=plan_snapshot(np.zeros((120,120),bool),[0,0],.05,
                            [1.049,3.025],[5.025,3.025])
        self.assertEqual(calls,list(range(10,3,-1)))
        self.assertAlmostEqual(p.stg_xy[0],1.225)
        self.assertLess(np.linalg.norm(np.asarray(p.stg_xy)-[1.049,3.025]),.20)

    def test_later_rollout_replan_keeps_valid_prefix(self):
        with patch.object(FMMPlanner,'get_short_term_goal',
                          side_effect=[(61,31,False,False),(61,41,True,False)]) as choose:
            p=plan_snapshot(np.zeros((120,120),bool),[0,0],.05,
                            [1.025,3.025],[5.025,3.025])
        self.assertEqual(choose.call_count,2)
        np.testing.assert_allclose(p.stg_xy,[1.525,3.025])

    def test_no_progress_does_not_return_current_cell(self):
        with patch.object(FMMPlanner,'get_short_term_goal',return_value=(61,21,False,False)):
            with self.assertRaises(NoVisibleSTG):
                plan_snapshot(np.zeros((120,120),bool),[0,0],.05,
                              [1.025,3.025],[5.025,3.025])

    def test_invalid_radius_configuration(self):
        for params in [dict(min_step_cells=0),dict(min_step_cells=11),dict(min_step_cells=4.5)]:
            with self.subTest(params=params),self.assertRaisesRegex(ValueError,'parameters'):
                plan_snapshot(np.zeros((120,120),bool),[0,0],.05,
                              [1.025,3.025],[5.025,3.025],**params)

if __name__=='__main__':unittest.main()
