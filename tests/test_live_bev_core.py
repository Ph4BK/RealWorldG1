import math
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from live_bev_core import RollingBEV, initial_map_from_odom, grid_start_status, room_no_go


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

    def test_annulus_updates_and_preserves_inner_and_outer_history(self):
        c=RollingBEV(np.ones((100,100),bool),[-2.5,-2.5],.05,
                     window=10,max_window_age=2.)
        radius=np.hypot(c.x,c.y)
        expected=(radius>=.3)&(radius<=2.)
        np.testing.assert_array_equal(c.region(0,0),expected)
        for i in range(10):
            self.assertEqual(c.add(i*.1,[[0.,0.,0.]],[0,0,.7]),i==9)
        self.assertFalse(c.task[self.cell(c,1.,0.)])
        self.assertFalse(c.task[self.cell(c,-1.,0.)])
        self.assertTrue(c.task[self.cell(c,0.,0.)])
        self.assertTrue(c.task[self.cell(c,.25,0.)])
        self.assertTrue(c.task[self.cell(c,2.3,0.)])
        self.assertTrue(c.static.all())

    def test_observed_obstacle_is_preserved_when_robot_moves_close(self):
        c=RollingBEV(np.zeros((100,100),bool),[-2.5,-2.5],.05,
                     window=1,min_points=1)
        cell=self.cell(c,1.,0.)
        c.add(0,[[1.,0.,.5]],[0,0,0])
        self.assertTrue(c.task[cell])
        c.add(.1,[[0.,0.,0.]],[.9,0,0])
        self.assertFalse(c.mask[cell])
        self.assertTrue(c.task[cell])
        c.add(.2,[[0.,0.,0.]],[0,0,0])
        self.assertFalse(c.task[cell])

    def test_annulus_boundaries_and_translation(self):
        c=RollingBEV(np.zeros((7,7),bool),[-3.5,-3.5],1.,inner_radius=1.)
        region=c.region(0,0)
        self.assertFalse(region[3,3])
        self.assertTrue(region[3,4])  # Exactly inner radius.
        self.assertTrue(region[3,5])  # Exactly 2 m.
        self.assertFalse(region[4,5])
        moved=c.region(1,0)
        self.assertFalse(moved[3,4])
        self.assertTrue(moved[3,6])
        self.assertFalse(moved[3,1])

    def test_initial_clear_changes_only_task_disk_and_reset_restores_static(self):
        c=RollingBEV(np.ones((100,100),bool),[-2.5,-2.5],.05)
        c.clear_initial_region(1.,.5)
        inner=np.hypot(c.x-1.,c.y-.5)<=.3
        np.testing.assert_array_equal(c.task,~inner)
        self.assertTrue(c.static.all())
        c.pause()
        np.testing.assert_array_equal(c.task,~inner)
        c.reset()
        self.assertTrue(c.task.all())

    def test_invalid_inner_radius_rejected(self):
        for radius in [-.1,2.,3.,float('nan'),float('inf')]:
            with self.subTest(radius=radius), self.assertRaises(ValueError):
                RollingBEV(np.zeros((2,2),bool),[0,0],.05,inner_radius=radius)

    def test_fixed_no_go_survives_annulus_clear_pause_reset_and_clock_reset(self):
        no_go = room_no_go((100,100), [-2.5,-2.5], .05, [-1.5,1.5,-1.5,1.5])
        c = RollingBEV(np.ones((100,100),bool), [-2.5,-2.5], .05,
                       window=10, no_go=no_go)
        expected = int(no_go.sum())
        for i in range(10):
            c.add(i*.1, [[0.,0.,0.]], [1.4,0.,0.])
        self.assertEqual(int(c.task[c.no_go].sum()), expected)
        self.assertGreater(c.last_no_go_protected_cells, 0)
        self.assertFalse(c.task[self.cell(c,.8,0.)])  # Ordinary static obstacle can clear.
        c.clear_initial_region(1.5,0.)
        self.assertEqual(int(c.task[c.no_go].sum()), expected)
        for operation in (c.pause, c.reset):
            operation()
            self.assertEqual(int(c.task[c.no_go].sum()), expected)
        c.add(10., [[0.,0.,0.]], [1.4,0.,0.])
        c.add(1., [[0.,0.,0.]], [1.4,0.,0.])
        self.assertTrue(c.task[c.no_go].all())
        self.assertEqual(c.no_go_cells, expected)

    def test_no_go_is_defensively_copied_and_rejects_wrong_geometry_or_dtype(self):
        policy = np.zeros((10,10),bool); policy[0] = True
        c = RollingBEV(np.zeros((10,10),bool), [0,0], .05, no_go=policy)
        policy[:] = False
        self.assertTrue(c.task[0].all())
        self.assertTrue(c.no_go[0].all())
        with self.assertRaises(ValueError):
            c.no_go[0,0] = False
        for bad in (np.zeros((2,2),bool), np.zeros((10,10),np.uint8)):
            with self.assertRaises(ValueError):
                RollingBEV(np.zeros((10,10),bool), [0,0], .05, no_go=bad)

    def test_start_status_reports_current_cell_and_nearest_without_snapping(self):
        free = np.zeros((4,4),bool); free[2,3] = True
        before = free.copy()
        xy = np.array([.025,.025])
        result = grid_start_status(free, np.array([0.,0.]), .05, xy)
        self.assertIs(result['start_free'], False)
        self.assertEqual(result['start_rejection_reason'], 'start_in_obstacle')
        np.testing.assert_allclose(result['nearest_free_xy_m'], [.175,.125])
        self.assertAlmostEqual(result['nearest_free_cell_distance_m'], math.hypot(.15,.10))
        np.testing.assert_array_equal(free,before)
        np.testing.assert_array_equal(xy,[.025,.025])
        self.assertTrue(grid_start_status(free,[0,0],.05,[.175,.125])['start_free'])
        self.assertEqual(grid_start_status(free,[0,0],.05,[-.01,0])['start_rejection_reason'],
                         'start_outside_map')
        self.assertIsNone(grid_start_status(np.zeros_like(free),[0,0],.05,xy)['nearest_free_cell_distance_m'])
        self.assertIsNone(grid_start_status(None,None,None,None)['start_free'])

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
