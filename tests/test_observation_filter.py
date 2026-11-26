"""Actual RollingBEV entry: ground confidence, missing IO and weak/real obstacles."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from g1_observation_filter import ObservationFilterConfig, observation_options
from live_bev_core import RollingBEV


def floor(shift=0.):
    x,y=np.meshgrid(np.arange(-1.85,1.86,.10),np.arange(-1.85,1.86,.10))
    return np.column_stack((x.ravel(),y.ravel(),.06+.006*x.ravel()-.003*y.ravel()+shift))


class FilterTest(unittest.TestCase):
    def core(self, **options):
        return RollingBEV(np.zeros((120,120),bool),[-3,-3],.05,window=3,
            observation_filter=ObservationFilterConfig(**options) if options else None)

    def cell(self,c,xy):
        col,row=np.floor((np.asarray(xy)-c.origin)/c.res).astype(int)
        return int(row),int(col)

    def feed(self,c,points,n=12,start=0.):
        for i in range(n):c.add(start+i*.1,points,[0.,0.,0.])

    def test_disabled_exact_legacy_arrays(self):
        a=self.core();b=self.core(ground=False,speckles=False)
        for i in range(15):
            p=np.vstack((floor(),[[1.025,.025,.3],[1.025,.025,.5]]))
            self.assertEqual(a.add(i*.1,p,[0,0,0]),b.add(i*.1,p,[0,0,0]))
            np.testing.assert_array_equal(a.local,b.local);np.testing.assert_array_equal(a.task,b.task)

    def test_ground_removed_and_vertical_obstacle_preserved(self):
        c=self.core(ground=True);obj=np.array([[1.025,.025,.3],[1.025,.025,.5]])
        self.feed(c,np.vstack((floor(),obj)))
        self.assertEqual(c.observation_filter.diagnostics['ground_state'],'ready')
        self.assertGreater(c.observation_filter.diagnostics['ground_removed_points'],100)
        self.assertTrue(c.task[self.cell(c,obj[0,:2])])
        self.assertLess(int(c.task.sum()),10)

    def test_height_reference_offsets_adapt_without_changing_configuration(self):
        for shift in (-.10,.10):
            with self.subTest(shift=shift):
                c=self.core(ground=True);obj=np.array([[1.025,.025,.3+shift],[1.025,.025,.5+shift]])
                self.feed(c,np.vstack((floor(shift),obj)))
                self.assertEqual(c.observation_filter.diagnostics['ground_state'],'ready')
                self.assertAlmostEqual(c.observation_filter.diagnostics['floor_z_at_robot_m'],.06+shift,places=4)
                self.assertTrue(c.task[self.cell(c,obj[0,:2])])
                self.assertEqual(c.z_min,.05)

    def test_sparse_ground_unknown_preserves_original_points(self):
        a=self.core();b=self.core(ground=True);p=np.array([[1.025,.025,.06],[1.025,.025,.06]])
        self.feed(a,p);self.feed(b,p)
        np.testing.assert_array_equal(a.task,b.task)
        self.assertEqual(b.observation_filter.diagnostics['ground_state'],'insufficient_floor_points')

    def test_collinear_floor_is_not_a_plane(self):
        c=self.core(ground=True,min_cells=20);x=np.arange(-1.8,1.8,.02);p=np.column_stack((x,np.zeros(len(x)),.06+0*x))
        self.feed(c,p)
        self.assertFalse(c.observation_filter.active)
        self.assertEqual(c.observation_filter.diagnostics['ground_removed_points'],0)
        self.assertEqual(c.observation_filter.diagnostics['ground_state'],'degenerate_floor')

    def test_points_outside_observed_floor_hull_are_not_filtered(self):
        c=self.core(ground=True);p=floor();p[:,2]=.15+.06*p[:,0]
        # Here the extrapolated plane is above the fitting search band's .20m.
        # Actual observed floor support ends earlier; do not extend it to this point.
        target=np.array([[1.725,.025,.15+.06*1.725]]*2)
        self.feed(c,np.vstack((p,target)))
        self.assertTrue(c.observation_filter.active)
        self.assertTrue(c.task[self.cell(c,target[0,:2])])

    def test_sparse_support_does_not_wrap_at_map_edges(self):
        from g1_observation_filter import ObservationFilter
        f=ObservationFilter(ObservationFilterConfig(speckles=True))
        count=np.zeros((5,5),np.int32);count[0,0]=2;count[-1,-1]=10
        supported=f.supported_cells([(0.,count)],count,np.zeros_like(count,bool),np.zeros_like(count,bool),2)
        self.assertFalse(supported[0,0]);self.assertTrue(supported[-1,-1])

    def test_excessive_tilt_falls_back(self):
        c=self.core(ground=True);p=floor();p[:,2]=.06+.2*p[:,0]
        self.feed(c,p)
        self.assertFalse(c.observation_filter.active)
        self.assertEqual(c.observation_filter.diagnostics['ground_removed_points'],0)

    def test_duplicates_cannot_confirm_ground(self):
        c=self.core(ground=True)
        for _ in range(20):c.add(0.,floor(),[0,0,0])
        self.assertEqual(len(c.observation_filter.fits),1)
        self.assertFalse(c.observation_filter.active)

    def test_lost_floor_does_not_reuse_old_plane(self):
        c=self.core(ground=True);self.feed(c,floor())
        self.assertTrue(c.observation_filter.active)
        c.add(1.3,[[1.025,.025,.06],[1.025,.025,.06]],[0,0,0])
        self.assertFalse(c.observation_filter.active)
        self.assertEqual(len(c.frames),1)
        self.assertIsNone(c.observation_filter.diagnostics['plane_coefficients'])

    def test_empty_cloud_keeps_map_and_resets_confidence(self):
        c=self.core(ground=True);self.feed(c,np.vstack((floor(),[[1.025,.025,.4]]*4)))
        before=c.task.copy()
        self.assertFalse(c.add(1.3,np.empty((0,3)),[0,0,0]))
        np.testing.assert_array_equal(c.task,before)
        self.assertFalse(c.observation_filter.active)
        self.assertEqual(len(c.frames),0)

    def test_long_gap_and_clock_rewind_reset_ground(self):
        c=self.core(ground=True);self.feed(c,floor())
        c.add(10.,floor(),[0,0,0]);self.assertEqual(len(c.observation_filter.fits),1)
        c.add(1.,floor(),[0,0,0]);self.assertEqual(len(c.observation_filter.fits),0)

    def test_protected_cells_never_lose_points_due_to_filter(self):
        a=self.core();b=self.core(ground=True,speckles=True)
        cell=self.cell(a,[1.05,.05]);a.static[cell]=True;b.static[cell]=True
        no_go=self.cell(a,[-1.05,.05]);a.no_go.setflags(write=True);b.no_go.setflags(write=True)
        a.no_go[no_go]=True;b.no_go[no_go]=True
        p=np.vstack((floor(),[[1.05,.05,.06]]*2))
        self.feed(a,p);self.feed(b,p)
        self.assertTrue(b.task[cell]);self.assertTrue(b.task[no_go])
        self.assertEqual(a.frames[-1][1][cell],b.frames[-1][1][cell])

    def test_one_frame_two_points_is_not_two_frames(self):
        c=self.core(speckles=True)
        for i in range(3):c.add(i*.1,[[1.025,.025,.4]]*2 if i==0 else [[0,0,0]],[0,0,0])
        self.assertFalse(c.task[self.cell(c,[1.025,.025])])
        self.assertEqual(c.observation_filter.diagnostics['speckle_removed_cells'],1)

    def test_thin_obstacle_with_two_independent_hits_survives(self):
        c=self.core(speckles=True)
        for i in range(3):c.add(i*.1,[[1.025,.025,.4]] if i<2 else [[0,0,0]],[0,0,0])
        self.assertTrue(c.task[self.cell(c,[1.025,.025])])

    def test_strong_single_frame_survives(self):
        c=self.core(speckles=True)
        for i in range(3):c.add(i*.1,[[1.025,.025,.4]]*4 if i==0 else [[0,0,0]],[0,0,0])
        self.assertTrue(c.task[self.cell(c,[1.025,.025])])

    def test_accepted_obstacle_is_not_forgotten_faster_by_speckle_filter(self):
        c=self.core(speckles=True);target=self.cell(c,[1.025,.025])
        for i in range(3):c.add(i*.1,[[1.025,.025,.4]]*4,[0,0,0])
        for i,n in enumerate([1,1,0],3):c.add(i*.1,[[1.025,.025,.4]]*n if n else [[0,0,0]],[0,0,0])
        self.assertTrue(c.task[target])
        c.add(.6,[[0,0,0]],[0,0,0]);c.add(.7,[[0,0,0]],[0,0,0])
        self.assertFalse(c.task[target])

    def test_options_validate_parameters(self):
        from types import SimpleNamespace
        self.assertIsNone(observation_options(SimpleNamespace()))
        for args in [dict(ground='yes'),dict(min_cells=2),dict(ground_band_m=0),
                     dict(min_quadrants=5),dict(min_inlier_fraction=1.1),dict(confirmation_frames=1.5)]:
            with self.subTest(args=args),self.assertRaises(ValueError):ObservationFilterConfig(**args)


if __name__=='__main__':unittest.main()
