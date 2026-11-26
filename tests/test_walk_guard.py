"""Actual Navigation.tick/PosNav entry, synthetic sensor time, no ROS/motion."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from g1_navigation_core import Navigation
from g1_final_approach import PosNav
from g1_walk_guard import WalkGuard, WalkGuardConfig


class WalkGuardTest(unittest.TestCase):
    def setUp(self):
        self.plans = []
        self.blocked = False
        def plan(pose, goal):
            self.plans.append((pose, goal))
            return (2., 0.)
        self.n = Navigation(plan, lambda *_: not self.blocked, walk_guard=WalkGuardConfig())
        self.n.pose = (0., 0., 0.)
        self.n.pose_time = self.n.map_time = 0.
        self.n.start((5., 0.), 0.)

    def tick(self, t, pose=None, fresh=True):
        if pose is not None:
            self.n.pose = pose
        self.n.map_time = t
        if fresh:
            self.n.pose_time = t
        return self.n.tick(t)

    def walk(self):
        self.tick(0.)
        self.assertEqual(self.tick(.02), (.5, 0.))

    def settle(self, yaw=0.):
        self.n.stg = (2., 0.)
        self.n.state = 'turn_settle'
        self.n.until = .75
        for i in range(9):
            command = self.tick(i*.1, (0., 0., yaw))
        return command

    def test_unknown_stop_rate_does_not_force_forward(self):
        self.n.state = 'turn_settle'; self.n.stg = (2., 0.); self.n.until = .75
        self.assertEqual(self.tick(2.26), (0., 0.))
        self.assertTrue(self.n.active)
        self.assertEqual(self.n.state, 'turn_settle')
        self.assertEqual(self.n.walk_guard.settle_fresh_s,0.)
        for t in (2.36,2.46,2.56):command=self.tick(t)
        self.assertEqual(command,(.5,0.))

    def test_settle_dropout_across_old_deadline_freezes_observed_budget(self):
        for gap in (.28,.45):
            with self.subTest(gap=gap):
                self.setUp();self.n.state='turn_settle'
                self.n.stg=(2.*math.cos(.4),2.*math.sin(.4));self.n.until=.75
                for i in range(21):self.tick(i*.1,(0.,0.,.02*i))
                # Keep ticking with no new source samples through the interruption.
                for t in (2.1,2.2):self.assertEqual(self.tick(t,fresh=False),(0.,0.))
                before=self.n.walk_guard.settle_fresh_s
                recovered=2.+gap
                self.assertEqual(self.tick(recovered,(0.,0.,.4)),(0.,0.))
                self.assertTrue(self.n.active)
                self.assertEqual(self.n.walk_guard.settle_fresh_s,before)
                for j in (1,2,3):command=self.tick(recovered+j*.1,(0.,0.,.4))
                self.assertEqual(command,(.5,0.))

    def test_duplicate_ticks_do_not_spend_settle_budget(self):
        self.n.state='turn_settle';self.n.stg=(2.,0.);self.n.until=.75
        for i in range(10):self.tick(i*.1,(0.,0.,i*.1))
        before=self.n.walk_guard.settle_fresh_s
        for t in (.92,.94,.96,1.):self.tick(t,fresh=False)
        self.assertEqual(self.n.walk_guard.settle_fresh_s,before)

    def test_settle_evidence_cannot_bridge_packet_gap_below_pose_timeout(self):
        guard=WalkGuard();guard.samples.extend([(0.,0.),(.1,0.),(.2,0.),(.48,0.)])
        self.assertFalse(guard.stopped_since(0.,.48,.3,.15,.15))
        guard.samples.extend([(.58,0.),(.68,0.),(.78,0.)])
        self.assertTrue(guard.stopped_since(0.,.78,.3,.15,.15))

    def test_new_settle_window_resets_its_budget(self):
        guard=WalkGuard()
        for t in (.8,.9,1.):
            guard.observe(t,(0.,0.,t),t,.3);guard.settle_elapsed(.75,t,.15)
        self.assertGreater(guard.settle_fresh_s,0.)
        self.assertEqual(guard.settle_elapsed(1.5,1.,.15),0.)

    def test_continuous_delayed_source_keeps_original_pose_freshness_limit(self):
        self.n.state='turn_settle';self.n.stg=(2.,0.);self.n.until=.75
        for wall in (.8,.9,1.,1.1):
            self.n.pose=(0.,0.,0.);self.n.pose_time=wall-.2;self.n.map_time=wall
            command=self.n.tick(wall)
        self.assertEqual(command,(.5,0.))
        self.assertEqual(self.n.state,'walk')
        # The timeout budget also accepts continuous samples aged 0.2s.
        guard=WalkGuard()
        for source in (.8,.9,1.):
            guard.observe(source,(0.,0.,source),source+.2,.3)
            elapsed=guard.settle_elapsed(.75,source+.2,.3,max_gap=.15)
        self.assertAlmostEqual(elapsed,.2)

    def test_continuing_rotation_never_forces_walk_on_timeout(self):
        self.n.state = 'turn_settle'; self.n.stg = (2., 0.); self.n.until = .75
        for i in range(24):
            command = self.tick(i*.1, (0., 0., .1*i))
            if command is not None:
                self.assertEqual(command[0], 0.)
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.reason, 'walk_guard_stop_timeout')

    def test_stationary_fresh_identical_yaw_samples_confirm_stop(self):
        self.assertEqual(self.settle(), (.5, 0.))
        self.assertEqual(self.n.state, 'walk')
        self.assertIsNone(self.n.yaw_rate())  # Legacy change-only history is insufficient.

    def test_old_stationary_history_does_not_confirm_new_stop(self):
        self.n.state = 'turn_settle'; self.n.stg = (2., 0.); self.n.until = 2.75
        self.n.walk_guard.samples.extend([(0., 0.), (.4, 0.)])
        self.assertEqual(self.tick(2.8), (0., 0.))
        self.assertEqual(self.n.state, 'turn_settle')

    def test_post_turn_heading_is_rechecked_then_bounded_alignment(self):
        self.assertEqual(self.settle(.5), (0., 0.))
        self.assertEqual(self.n.state, 'guard_before_align')
        for i in range(9, 17):
            self.tick(i*.1, (0., 0., .5))
        self.assertEqual(self.n.state, 'guard_align')
        self.assertEqual(self.tick(1.7), (0., -.5))
        self.assertEqual(self.tick(1.8, (0., 0., 0.)), (0., 0.))
        for i in range(19, 27):
            command = self.tick(i*.1)
        self.assertEqual(command, (.5, 0.))

    def test_stalled_realign_is_time_bounded(self):
        self.walk()
        self.n.guard_align(.1, 'fixture')
        for i in range(2, 72):
            self.tick(i*.1, (0., 0., .5))
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.reason, 'walk_guard_align_timeout')

    def test_repeated_heading_failure_is_attempt_bounded(self):
        self.walk()
        for i in range(4):
            self.n.guard_align(float(i), 'fixture')
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.reason, 'walk_guard_align_limit')

    def test_persistent_walk_heading_stops_after_sensor_time_threshold(self):
        for yaw in (-.5, .5):
            with self.subTest(yaw=yaw):
                self.setUp(); self.walk()
                for t in (.1, .2, .3):
                    self.assertEqual(self.tick(t, (0., 0., yaw)), (.5, 0.))
                self.assertEqual(self.tick(.4), (0., 0.))
                self.assertEqual(self.n.state, 'guard_before_align')

    def test_heading_persistence_resets_after_good_sample(self):
        self.walk()
        self.tick(.1, (0., 0., .5))
        self.tick(.2, (0., 0., 0.))
        for t in (.3, .4, .5):
            self.assertEqual(self.tick(t, (0., 0., .5)), (.5, 0.))
        self.assertEqual(self.tick(.6), (0., 0.))

    def test_repeated_pose_tick_cannot_count_as_sensor_persistence(self):
        self.walk(); self.tick(.1, (0., 0., .5))
        for t in (.12, .2, .3):
            self.assertEqual(self.tick(t, fresh=False), (.5, 0.))
        self.assertEqual(self.tick(.41, fresh=False), (0., 0.))
        self.assertEqual(self.n.state, 'walk')

    def test_no_progress_stops_then_replans_from_actual_pose(self):
        self.walk()
        self.assertEqual(self.tick(8.03), (0., 0.))
        self.assertEqual(self.n.state, 'guard_replan_settle')
        self.tick(8.6, (.02, .01, 0.)); self.tick(8.7)
        self.assertEqual(len(self.plans), 2)
        self.assertEqual(self.plans[-1][0], (.02, .01))

    def test_small_jitter_is_not_progress(self):
        self.walk()
        for i in range(1, 81):
            command = self.tick(i*.1, (.02 if i%2 else -.02, 0., 0.))
        self.assertEqual(self.tick(8.1), (0., 0.))
        self.assertEqual(self.n.state, 'guard_replan_settle')

    def test_real_forward_progress_refreshes_timer(self):
        self.walk()
        self.assertEqual(self.tick(7., (.06, 0., 0.)), (.5, 0.))
        self.assertEqual(self.tick(14., (.12, 0., 0.)), (.5, 0.))

    def test_crossing_stg_plane_stops_before_progress_guard(self):
        from unittest.mock import patch
        self.walk()
        with patch.object(self.n.walk_guard,'check_walk',side_effect=AssertionError('must stop first')):
            self.assertEqual(self.tick(.1,(2.1,0.,0.)),(0.,0.))
        self.assertEqual(self.n.state,'walk_settle')

    def test_alignment_status_counts_goal_attempts_and_includes_settling_time(self):
        self.walk();self.n.guard_align(.1,'fixture')
        status=self.n.walk_guard.status(.6)
        self.assertEqual(status['realign_total_count'],1)
        self.assertAlmostEqual(status['realign_total_s'],.5)
        self.n.walk_guard.finish_alignment(1.1)
        self.n.walk_guard.new_segment()
        status=self.n.walk_guard.status(10.)
        self.assertEqual(status['align_count'],0)
        self.assertEqual(status['realign_total_count'],1)
        self.assertAlmostEqual(status['realign_total_s'],1.)
        self.n.walk_guard.reset()
        self.assertEqual(self.n.walk_guard.status(20.)['realign_total_s'],0.)

    def test_repeated_stalls_are_bounded_per_goal(self):
        self.walk()
        for i in range(1,400):
            self.tick(i*.1)
            if not self.n.active:break
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.reason,'walk_guard_no_progress_limit')
        self.assertEqual(len(self.plans),3)

    def test_stale_wait_is_excluded_from_no_progress_time(self):
        self.walk(); self.tick(10., fresh=False)
        self.assertEqual(self.tick(10.1), (.5, 0.))
        self.assertEqual(self.n.walk_guard.diagnostics['no_progress_s'], 0.)

    def test_obstacle_resume_rechecks_heading(self):
        self.walk(); self.blocked=True; self.tick(.1)
        self.blocked=False
        self.assertEqual(self.tick(2.2, (0., 0., .5)), (0., 0.))
        self.assertEqual(self.n.state, 'guard_before_align')

    def test_cancel_and_next_goal_reset_guard_state(self):
        self.walk(); self.n.guard_align(.1, 'fixture')
        self.n.stop('Stopped by user')
        self.assertIsNone(self.tick(.2))
        self.n.start((5., 0.), .2)
        self.assertEqual(self.n.walk_guard.align_count, 0)
        self.assertEqual(len(self.n.walk_guard.samples), 0)

    def test_config_validation(self):
        for args in [dict(progress_s=0), dict(align_deg=20), dict(align_attempts=1.5),
                     dict(far_m=float('nan')), dict(align_max_wz=.1)]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                WalkGuardConfig(**args)

    def test_opt_in_guard_does_not_change_final_completion_contract(self):
        n=PosNav(lambda *_:None, lambda *_:True, lambda *_:dict(pre=None,reason='fixture'),
                 walk_guard=WalkGuardConfig())
        n.final_goal=(1.,0.,0.); n.goal=(1.,0.)
        n.pose=(1.,.5,0.); n.pose_time=n.map_time=1.
        n.active=True; n.state='fine_yaw_settle'; n.until=0.
        self.assertEqual(n.tick(1.),(0.,0.,0.))
        self.assertFalse(n.active)
        self.assertEqual(n.reason,'Final approach completed')
        self.assertAlmostEqual(n.final_error_m,.5)


if __name__ == '__main__':
    unittest.main()
