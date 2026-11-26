"""Round-one regressions through actual Navigation entry points; no robot IO."""
import argparse
import importlib.util
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from g1_navigation_core import Navigation,velocity_xyz
from g1_motion_policy import MotionPolicy,effective_planning_config
from g1_walk_guard import WalkGuardConfig
from g1_navigation_options import add_navigation_options,checked_options,options_from_args,forward_navigation_options


class RoundOneTest(unittest.TestCase):
    def controller(self, angle=0., **policy):
        n=Navigation(Mock(return_value=(2.,0.)),Mock(return_value=True),
            walk_guard=WalkGuardConfig(),motion=MotionPolicy(**policy))
        n.pose=(0.,0.,-math.radians(angle));n.pose_time=n.map_time=0.
        n.pose_source_stamp_ns=1_000_000_000
        n.start((5.,0.),0.);n.tick(0.)
        return n

    def tick(self,n,t,pose=None,source=None,fresh=True,map_fresh=True):
        if pose is not None:n.pose=pose
        if fresh:n.pose_time=t
        if map_fresh:n.map_time=t
        n.pose_source_stamp_ns=int((1+t)*1e9) if source is None else source
        command=n.tick(t)
        return None if command is None else velocity_xyz(command)

    def walking(self,**policy):
        n=self.controller(**policy)
        self.assertEqual(self.tick(n,.1),(.5,0.,0.))
        return n

    def test_projection_threshold_is_not_euclidean_and_does_not_stop(self):
        for remaining in (.299,.300,.301):
            for lateral in (-.4,.4):
                with self.subTest(remaining=remaining,lateral=lateral):
                    n=self.walking(near_stg_no_realign=True)
                    pose=(2.-remaining,lateral,1.2)
                    for t in (.2,.3,.4,.5,.6):command=self.tick(n,t,pose)
                    if remaining<=.3:
                        self.assertEqual(command,(.5,0.,0.))
                        self.assertTrue(n.near_stg_latched)
                        self.assertEqual(n.walk_guard.realign_total_count,0)
                    else:
                        self.assertEqual(command,(0.,0.,0.))
                        self.assertEqual(n.state,'guard_before_align')

    def test_near_latch_survives_backward_jitter(self):
        n=self.walking(near_stg_no_realign=True)
        self.tick(n,.2,(1.71,.4,1.2))
        for i in range(3,12):
            self.assertEqual(self.tick(n,i*.1,(1.65,.4,1.2)),(.5,0.,0.))
        self.assertTrue(n.near_stg_latched)
        self.assertAlmostEqual(n.remaining(),.35)

    def test_normal_stop_is_still_fifteen_cm_not_thirty(self):
        n=self.walking(near_stg_no_realign=True)
        self.assertEqual(self.tick(n,.2,(1.75,.4,1.2)),(.5,0.,0.))
        self.assertEqual(n.state,'walk')
        self.assertEqual(self.tick(n,.3,(1.86,.4,1.2)),(0.,0.,0.))
        self.assertEqual(n.state,'walk_settle')

    def test_every_resume_path_uses_same_projection_without_rebasing(self):
        for state in ('guard_before_align','guard_settle','turn_settle','obstacle'):
            with self.subTest(state=state):
                n=self.walking(near_stg_no_realign=True)
                n.state=state;n.until=.3
                for i in range(2,13):command=self.tick(n,i*.1,(1.75,.4,1.2))
                self.assertEqual(n.state,'walk')
                self.assertEqual(command,(.5,0.,0.))
                self.assertAlmostEqual(n.remaining(),.25)
                self.assertEqual(n.motion_direction,(1.,0.))

    def test_active_alignment_stops_then_resumes_without_new_rotation(self):
        n=self.walking(near_stg_no_realign=True)
        n.state='guard_align';n.walk_guard.align_started=0.
        self.assertEqual(self.tick(n,.2,(1.75,.4,1.2)),(0.,0.,0.))
        self.assertEqual(n.state,'guard_settle')
        for i in range(3,13):
            command=self.tick(n,i*.1)
            self.assertEqual(command[2],0.)
        self.assertEqual(command,(.5,0.,0.))

    def test_legacy_excursion_cannot_restart_rotation_inside_latched_zone(self):
        for state in ('turn','turn_out','turn_out_settle'):
            n=self.walking(near_stg_no_realign=True)
            n.state=state;n.until=.3
            for i in range(2,14):
                command=self.tick(n,i*.1,(1.75,.4,1.2))
                self.assertEqual(command[2],0.)
            self.assertEqual(n.state,'walk')

    def test_near_gate_keeps_obstacle_input_and_progress_stops(self):
        n=self.walking(near_stg_no_realign=True)
        self.tick(n,.2,(1.75,.4,1.2));n.clear.return_value=False
        self.assertEqual(self.tick(n,.3),(0.,0.,0.))
        self.assertEqual(n.state,'obstacle')
        n.clear.return_value=True
        self.assertEqual(self.tick(n,5.,fresh=False),(0.,0.,0.))
        self.assertTrue(n.active)
        self.tick(n,5.1)
        self.assertEqual(n.state,'walk')
        for i in range(52,136):command=self.tick(n,i*.1)
        self.assertEqual(command,(0.,0.,0.))
        self.assertIn(n.state,('guard_replan_settle','plan'))
        self.assertEqual(n.walk_guard.realign_total_count,0)

    def test_same_stg_replan_keeps_latch_new_stg_and_new_goal_reset(self):
        n=self.walking(near_stg_no_realign=True)
        self.tick(n,.2,(1.75,.4,1.2));n.state='plan'
        self.tick(n,.3);self.tick(n,.4)
        self.assertTrue(n.near_stg_latched)
        self.assertEqual(n.state,'walk')
        self.assertAlmostEqual(n.remaining(),.25)
        n.plan.return_value=(4.,0.);n.state='plan';self.tick(n,.5)
        self.assertFalse(n.near_stg_latched)
        self.assertEqual(n.motion_stg,(4.,0.))
        n.stop();n.pose_time=n.map_time=1.;n.start((5.,0.),1.)
        self.assertFalse(n.near_stg_latched)
        self.assertIsNone(n.motion_direction)

    def test_coarse_rates_slow_then_stop_and_recheck_before_forward(self):
        n=self.controller(60.,coarse_turn_profile=True)
        self.assertEqual(self.tick(n,.1),(0.,0.,.5))
        self.assertEqual(self.tick(n,.2,(0.,0.,-math.radians(15.))),(0.,0.,.3))
        command=self.tick(n,.3,(0.,0.,-math.radians(8.)))
        self.assertGreater(command[2],.15);self.assertLess(command[2],.3)
        self.assertEqual(self.tick(n,.4,(0.,0.,-math.radians(3.))),(0.,0.,0.))
        self.assertEqual(n.state,'turn_settle')
        for i in range(5,12):
            self.assertEqual(self.tick(n,i*.1),(0.,0.,0.))
        self.assertEqual(self.tick(n,1.2),(.5,0.,0.))

    def test_initial_release_boundary_avoids_unnecessary_turn(self):
        for angle,turn in ((4.9,False),(5.,False),(5.1,True),(-5.1,True)):
            n=self.controller(angle,coarse_turn_profile=True)
            self.assertEqual(n.state=='turn',turn)

    def test_profile_holds_heading_within_attempt_and_never_reverses_before_settle(self):
        n=self.controller(30.,coarse_turn_profile=True)
        self.tick(n,.1)
        self.assertEqual(n.profile_heading,0.)
        command=self.tick(n,.2,(0.,.2,math.radians(6.)))
        self.assertEqual(command,(0.,0.,0.))
        self.assertEqual(n.state,'turn_settle')
        self.assertEqual(n.profile_heading,0.)

    def test_settled_wrong_heading_has_finite_attempts(self):
        n=self.controller(60.,coarse_turn_profile=True,coarse_turn_attempts=1)
        self.tick(n,.1,(0.,0.,0.))
        for i in range(2,25):
            command=self.tick(n,i*.1,(0.,0.,-math.radians(8.)))
            if not n.active:break
        self.assertFalse(n.active)
        self.assertEqual(command,(0.,0.,0.))
        self.assertEqual(n.reason,'coarse_turn_align_limit')

    def test_large_turn_both_signs_is_feasible_in_default_budget_ideal_response(self):
        for angle in (-179.,179.):
            n=self.controller(angle,coarse_turn_profile=True)
            yaw=n.pose[2]
            for i in range(1,130):
                command=self.tick(n,i*.1,(0.,0.,yaw))
                self.assertLessEqual(abs(command[2]),.5)
                yaw+=command[2]*.1
                if n.state=='walk':break
            self.assertTrue(n.active)
            self.assertEqual(n.state,'walk')
            self.assertLess(i*.1,12.)

    def test_turn_budget_uses_valid_source_at_ten_and_five_hz(self):
        for dt in (.1,.2):
            n=self.controller(60.,coarse_turn_profile=True)
            for i in range(1,int(13/dt)):
                command=self.tick(n,i*dt)
                if not n.active:break
            self.assertFalse(n.active)
            self.assertEqual(command,(0.,0.,0.))
            self.assertEqual(n.reason,'coarse_turn_timeout')

    def test_dropout_freezes_budget_and_recovers_without_charging_gap(self):
        n=self.controller(60.,coarse_turn_profile=True)
        self.tick(n,.1);self.tick(n,.2);before=n.profile_samples.settle_fresh_s
        self.assertEqual(self.tick(n,30.,fresh=False),(0.,0.,0.))
        self.assertTrue(n.active)
        self.tick(n,30.1)
        self.assertAlmostEqual(n.profile_samples.settle_fresh_s,before)
        self.tick(n,30.2)
        self.assertAlmostEqual(n.profile_samples.settle_fresh_s,before+.1)

    def test_valid_five_hz_cannot_wait_forever_for_stricter_stop_evidence(self):
        n=self.walking(near_stg_no_realign=True)
        n.state='turn_settle';n.until=.5
        for i in range(1,20):
            command=self.tick(n,i*.2,(1.75,.4,1.2))
            if not n.active:break
        self.assertFalse(n.active)
        self.assertEqual(command,(0.,0.,0.))
        self.assertEqual(n.reason,'walk_guard_stop_timeout')

    def test_duplicate_original_source_cannot_confirm_stop(self):
        n=self.controller(60.,coarse_turn_profile=True)
        self.tick(n,.1,(0.,0.,0.))
        for i in range(2,40):
            self.assertEqual(self.tick(n,i*.1,source=123),(0.,0.,0.))
            self.assertNotEqual(n.state,'walk')
        for i in range(40,45):self.tick(n,i*.1)
        self.assertEqual(n.state,'walk')

    def test_near_gate_overrides_profile_recheck_and_rotating_state(self):
        n=self.controller(60.,coarse_turn_profile=True,near_stg_no_realign=True)
        self.tick(n,.1)
        self.assertEqual(self.tick(n,.2,(1.75,.4,-1.)),(0.,0.,0.))
        for i in range(3,13):command=self.tick(n,i*.1)
        self.assertEqual(command,(.5,0.,0.))
        self.assertEqual(n.profile_attempts,1)
        self.assertAlmostEqual(n.remaining(),.25)

    def test_cancel_never_restarts_old_profile(self):
        n=self.controller(60.,coarse_turn_profile=True)
        self.tick(n,.1);n.stop('Cancelled')
        self.assertIsNone(self.tick(n,1.))
        n.pose_time=n.map_time=1.;n.start((5.,0.),1.)
        self.assertEqual(n.profile_attempts,0)
        self.assertIsNone(n.profile_heading)

    def test_clis_roundtrip_include_guard_planning_and_motion_values(self):
        p=argparse.ArgumentParser();add_navigation_options(p)
        a=p.parse_args(['--walk-guard','--near-stg-no-realign','--coarse-turn-profile',
            '--guard-reenter-deg','20','--fmm-inflation-cells','7','--fmm-step-cells','20',
            '--coarse-omega','.5','--no-turn-out'])
        self.assertEqual(options_from_args(a),options_from_args(p.parse_args(forward_navigation_options(a))))
        self.assertEqual(forward_navigation_options(p.parse_args([])),[])
        cfg=dict(fmm_inflation_cells=5,fmm_step_cells=10,fmm_min_step_cells=4,fmm_max_stgs=4)
        effective=effective_planning_config(cfg,options_from_args(a))
        self.assertEqual((effective['fmm_inflation_cells'],effective['fmm_step_cells']),(7,20))
        self.assertEqual(cfg['fmm_step_cells'],10)

    def test_invalid_options_fail_before_runtime_construction(self):
        for bad in (dict(near_stg_no_realign=True),dict(coarse_turn_profile=True),
            dict(coarse_turn_stop_deg=5),dict(coarse_turn_min_wz=.4),dict(coarse_turn_attempts=1.5),
            dict(near_stg_distance=.15),dict(fmm_inflation_cells=-1),dict(fmm_step_cells=True),
            dict(coarse_omega=float('nan')),dict(guard_reenter_deg=3),dict(no_turn_out=1)):
            with self.subTest(bad=bad),self.assertRaises(ValueError):checked_options(**bad)

    def test_all_new_switches_off_matches_frozen_core_commands(self):
        from frozen_baselines import baseline_path
        path=baseline_path('G1_ROUND1_BASELINE', 'core_v3r2.py')
        spec=importlib.util.spec_from_file_location('frozen_round1_core',path)
        old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
        for enabled in (False,True):
            for angle in (.08,.7):
                nodes=[]
                for cls in (old.Navigation,Navigation):
                    n=cls(lambda p,g:(2.,0.),lambda *_:True,
                          walk_guard=WalkGuardConfig() if enabled else None)
                    n.pose=(0.,0.,-angle);n.pose_time=n.map_time=0.;n.start((5.,0.),0.);nodes.append(n)
                for i in range(601):
                    t=i*.02;sensor=(i//5)*.1;rows=[]
                    for n in nodes:
                        n.pose=(min(2.1,sensor*.2),.01*math.sin(sensor),0. if t>2 else -angle)
                        n.pose_time=sensor;n.map_time=t
                        rows.append((n.tick(t),n.state,n.active,n.reason))
                    self.assertEqual(rows[0],rows[1],(enabled,angle,t))


if __name__=='__main__':unittest.main()
