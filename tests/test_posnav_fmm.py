import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from posnav_fmm.planner import plan_snapshot
from posnav_fmm.visibility import supercover_line,line_is_collision_free,get_stg

class FMMTest(unittest.TestCase):
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

if __name__=='__main__':unittest.main()
