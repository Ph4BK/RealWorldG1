import math
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from live_bev_core import RollingBEV, initial_map_from_odom


class FusionTest(unittest.TestCase):
    def make(self):
        return RollingBEV(np.zeros((100,100),bool),[-2.5,-2.5],.05,window=5,max_window_age=1.)

    def cell(self,c,x,y):
        col,row=np.floor((np.array([x,y])-c.origin)/c.res).astype(int)
        return row,col

    def test_threshold_window_eviction_and_outside_preservation(self):
        c=self.make(); cell=self.cell(c,-1.,0.)
        front=self.cell(c,2.3,0.);c.static[front]=True;c.task[front]=True
        for i in range(5):
            points=[[-1.,0.,.5]] if i<2 else [[0.,0.,0.]]
            self.assertEqual(c.add(i*.1,points,[0,0,0]),i==4)
        self.assertTrue(c.task[cell]);self.assertTrue(c.task[front])
        c.add(.5,[[0,0,0]],[0,0,0])
        self.assertFalse(c.task[cell]);self.assertTrue(c.task[front])
        # Region moves away; old modified cells outside new region remain unchanged.
        c.task[cell]=True
        c.add(.6,[[0,0,0]],[2,2,0])
        self.assertTrue(c.task[cell])

    def test_height_inclusive_and_bounds_do_not_clip(self):
        c=self.make();cell=self.cell(c,-1.,0.)
        points=[[-1.,0.,.05],[-1.,0.,1.3],[-1.,0.,.049],[-1.,0.,1.301],[-100,0,.5]]
        c.add(0,points,[0,0,0]);counts=c.frames[-1][1]
        self.assertEqual(counts[cell],2);self.assertEqual(counts.sum(),2)

    def test_empty_stale_duplicate_and_reset(self):
        c=self.make();c.task[:]=True
        for i in range(5): c.add(i*.1,[[0,0,0]],[0,0,0])
        saved=c.task.copy()
        self.assertFalse(c.add(.5,np.zeros((0,3)),[0,0,0]))
        self.assertEqual(len(c.frames),0);np.testing.assert_array_equal(c.task,saved)
        for _ in range(5):c.add(.6,[[0,0,0]],[0,0,0])
        self.assertEqual(len(c.frames),1)
        c.add(3.,[[0,0,0]],[0,0,0]);self.assertEqual(len(c.frames),1)
        c.add(1.,[[0,0,0]],[0,0,0]);np.testing.assert_array_equal(c.task,c.static)

    def test_full_disk_clears_stale_nearby_obstacles_and_preserves_exterior(self):
        c=RollingBEV(np.ones((100,100),bool),[-2.5,-2.5],.05,
                     window=10,max_window_age=2.)
        radius=np.hypot(c.x,c.y)
        expected=radius<=2.
        np.testing.assert_array_equal(c.region(0,0),expected)
        for i in range(10):
            self.assertEqual(c.add(i*.1,[[0.,0.,0.]],[0,0,.7]),i==9)
        self.assertFalse(c.task[self.cell(c,1.,0.)])
        self.assertFalse(c.task[self.cell(c,-1.,0.)])
        self.assertFalse(c.task[self.cell(c,0.,0.)])
        self.assertFalse(c.task[self.cell(c,.25,0.)])
        self.assertTrue(c.task[self.cell(c,2.3,0.)])
        self.assertTrue(c.static.all())

    def test_nearby_live_obstacle_is_retained_in_full_disk(self):
        c=RollingBEV(np.ones((100,100),bool),[-2.5,-2.5],.05)
        for i in range(10):
            c.add(i*.1,[[.2,0.,.5],[.2,0.,.6]],[0,0,0])
        self.assertTrue(c.task[self.cell(c,.2,0.)])
        self.assertFalse(c.task[self.cell(c,0.,0.)])

    def test_disk_boundary_is_inclusive_and_moves_with_robot(self):
        c=RollingBEV(np.zeros((7,7),bool),[-3.5,-3.5],1.)
        region=c.region(0,0)
        self.assertTrue(region[3,3])
        self.assertTrue(region[3,5])  # Exactly 2 m.
        self.assertFalse(region[4,5])
        moved=c.region(1,0)
        self.assertTrue(moved[3,6])
        self.assertFalse(moved[3,1])

    def test_window_max_age(self):
        c=self.make()
        for i in range(5): self.assertFalse(c.add(i*.4,[[0,0,0]],[0,0,0]))
        self.assertLess(len(c.frames),5)

    def test_current_arrow_is_not_odom_origin(self):
        samples=np.zeros((401,7));samples[:,0]=np.linspace(10,12,401);samples[:,3]=1.
        odom=np.eye(4);odom[:3,:3]=Rotation.from_euler('z',.7).as_matrix();odom[:3,3]=[3,-2,.1]
        seed=initial_map_from_odom(5,6,1.2,1.26,samples,odom)
        actual=seed@odom
        np.testing.assert_allclose(actual[:3,3],[5,6,1.26],atol=1e-12)
        self.assertAlmostEqual(math.atan2(actual[1,0],actual[0,0]),1.2)
        samples[:,4]=1
        with self.assertRaises(ValueError):initial_map_from_odom(5,6,1.2,1.26,samples,odom)

if __name__=='__main__':unittest.main()
