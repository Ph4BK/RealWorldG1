"""Deterministic timed recovery tests; no ROS, DDS or robot commands."""
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from g1_navigation_core import Navigation, velocity_xyz
from posnav_fmm.errors import GoalUnreachable, NoVisibleSTG


class RecoveryTest(unittest.TestCase):
    def setUp(self):
        self.info = dict(near_count=1, sector_counts=[3, 0, 2, 1])
        self.n = Navigation(Mock(side_effect=GoalUnreachable('unreachable')),
                            Mock(return_value=True), recovery=lambda _: self.info)
        self.n.pose = (0., 0., 0.)
        self.n.pose_time = self.n.map_time = 0.
        self.n.map_revision = 1
        self.n.start((5., 0.), 0.)

    def tick(self, t, revision=None, fresh=True):
        if fresh:
            self.n.pose_time = self.n.map_time = t
        if revision is not None:
            self.n.map_revision = revision
        return velocity_xyz(self.n.tick(t)) if self.n.active else self.n.tick(t)

    def begin(self, t=0.):
        self.assertEqual(self.tick(t), (0., 0., 0.))
        self.assertEqual(self.n.state, 'recovery_prepare')
        self.assertEqual(self.n.recovery_count, 0)
        self.assertEqual(self.tick(t+.02), (0., .5, 0.))
        self.assertEqual(self.n.state, 'recovery_left')

    def test_timed_lateral_motion_bypasses_normal_clearance_and_keeps_goal(self):
        self.n.clear.return_value = False
        self.begin()
        self.n.pose = (1., .5, 1.)  # Deliberately ignore distance/yaw during the timed pulse.
        self.info['sector_counts'] = [0, 8, 9, 10]
        self.assertEqual(self.tick(1.019), (0., .5, 0.))
        self.assertEqual(self.tick(1.02), (0., 0., 0.))
        self.assertEqual(self.n.state, 'recovery_settle')
        self.assertEqual(self.n.goal, (5., 0.))
        self.n.clear.assert_not_called()
        self.assertEqual(self.n.plan.call_count, 1)

    def test_sector_minimum_and_ties_choose_exact_body_axis(self):
        for counts, direction, velocity in [
            ([0, 1, 1, 1], 'forward', (.5, 0., 0.)),
            ([1, 0, 1, 1], 'left', (0., .5, 0.)),
            ([1, 1, 0, 1], 'backward', (-.5, 0., 0.)),
            ([1, 1, 1, 0], 'right', (0., -.5, 0.)),
            ([100]*4, 'forward', (.5, 0., 0.)),
            ([2, 1, 1, 1], 'left', (0., .5, 0.))]:
            with self.subTest(counts=counts):
                self.setUp()
                self.info['sector_counts'] = counts
                self.tick(0.)
                self.assertEqual(self.tick(.02), velocity)
                self.assertEqual(self.n.state, 'recovery_'+direction)
                self.assertEqual(self.n.recovery_count, 1)

    def test_no_near_occupancy_never_triggers_recovery(self):
        self.info['near_count'] = 0
        for t in range(5):
            self.assertEqual(self.tick(float(t), revision=t+1), (0., 0., 0.))
            self.assertEqual(self.n.state, 'plan_wait')
        self.assertEqual(self.n.recovery_count, 0)

    def test_no_visible_stg_needs_three_distinct_failed_maps(self):
        self.n.plan.side_effect = NoVisibleSTG('no visible STG')
        for t in (0., 1., 2., 3.):
            self.tick(t)
            self.assertEqual(self.n.state, 'plan_wait')
        self.assertEqual(self.n.planning_failures, 1)
        self.tick(4., revision=2)
        self.assertEqual(self.n.planning_failures, 2)
        self.tick(5., revision=3)
        self.assertEqual(self.n.state, 'recovery_prepare')
        self.assertEqual(self.n.planning_failures, 3)

    def test_blocked_stg_counts_as_retry_without_fabricating_motion(self):
        self.n.plan.side_effect = None
        self.n.plan.return_value = (1., 0.)
        self.n.clear.return_value = False
        for i in range(3):
            self.assertEqual(self.tick(float(i), revision=i+1), (0., 0., 0.))
        self.assertEqual(self.n.state, 'recovery_prepare')
        self.assertIsNone(self.n.stg)

    def test_input_outage_does_not_count_failures(self):
        self.n.plan.side_effect = NoVisibleSTG('no visible STG')
        self.tick(0.)
        self.tick(2., revision=2, fresh=False)
        self.assertEqual(self.n.planning_failures, 1)
        self.assertEqual(self.n.plan.call_count, 1)
        self.tick(2.1)
        self.assertEqual(self.n.planning_failures, 2)

    def test_prepare_rechecks_near_occupancy_and_inputs_before_moving(self):
        self.tick(0.)
        self.info['near_count'] = 0
        self.assertEqual(self.tick(.02), (0., 0., 0.))
        self.assertEqual(self.n.state, 'plan')
        self.assertEqual(self.n.recovery_count, 0)
        self.setUp()
        self.tick(0.)
        self.assertEqual(self.tick(.31, fresh=False), (0., 0., 0.))
        self.assertEqual(self.n.state, 'plan_wait')
        self.assertEqual(self.n.recovery_count, 0)

    def test_pose_or_map_outage_aborts_pulse_and_never_resumes_remaining_time(self):
        for field in ('pose_time', 'map_time'):
            with self.subTest(field=field):
                self.setUp()
                self.begin()
                self.n.pose_time = self.n.map_time = .2
                setattr(self.n, field, -math.inf)
                self.assertEqual(self.tick(.2, fresh=False), (0., 0., 0.))
                self.assertEqual(self.n.state, 'recovery_settle')
                self.assertEqual(self.n.recovery_count, 1)
                self.n.plan.side_effect = None
                self.n.plan.return_value = (1., 0.)
                self.assertEqual(self.tick(.69, revision=2), (0., 0., 0.))
                self.assertEqual(self.n.state, 'recovery_settle')
                self.tick(.71)
                self.assertEqual(self.n.state, 'walk_start')
                self.assertEqual(self.n.recovery_count, 1)

    def test_settle_requires_half_second_and_a_post_stop_map(self):
        self.begin()
        self.tick(1.02, revision=2)
        self.n.plan.side_effect = None
        self.n.plan.return_value = (1., 0.)
        self.tick(1.51, revision=3)
        self.assertEqual(self.n.state, 'recovery_settle')
        self.tick(1.52)
        self.assertEqual(self.n.state, 'walk_start')
        self.assertEqual(self.n.plan.call_count, 2)
        self.setUp()
        self.begin()
        self.tick(1.02)
        self.tick(1.6)
        self.assertEqual(self.n.state, 'recovery_settle')
        self.assertIn('new task map', self.n.reason)
        self.assertEqual(self.n.plan.call_count, 1)

    def test_third_pulse_allows_replan_but_never_a_fourth_pulse(self):
        t = 0.
        for attempt in range(1, 4):
            self.tick(t, revision=attempt)
            self.assertEqual(self.n.state, 'recovery_prepare')
            self.tick(t+.02)
            self.assertEqual(self.n.recovery_count, attempt)
            self.tick(t+1.02)
            # Replan after the full settle and a new map.
            t += 1.6
        self.assertEqual(self.tick(t, revision=4), (0., 0., 0.))
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.reason, 'recovery_exhausted')
        self.assertEqual(self.n.recovery_count, 3)
        self.assertEqual(self.n.plan.call_count, 4)

    def test_counter_only_resets_after_normal_walk_completion(self):
        self.begin()
        self.tick(1.02)
        self.n.plan.side_effect = None
        self.n.plan.return_value = (1., 0.)
        self.tick(1.6, revision=2)
        self.assertEqual(self.n.recovery_count, 1)  # A selected STG is insufficient.
        self.tick(1.62)
        self.n.pose = (.9, 0., 0.)
        self.tick(1.7)
        self.assertEqual(self.n.state, 'walk_settle')
        self.tick(2.21)
        self.assertEqual(self.n.recovery_count, 0)
        self.assertIn('recovery_counter_reset', [e for e, _ in self.n.recovery_events])

    def test_paused_segment_and_tiny_segment_do_not_reset_counter(self):
        for tiny in (False, True):
            with self.subTest(tiny=tiny):
                self.setUp()
                self.n.recovery_count = 2
                self.n.plan.side_effect = None
                self.n.plan.return_value = (.1 if tiny else 1., 0.)
                self.tick(0.)
                self.tick(.02)
                if not tiny:
                    self.tick(.4, fresh=False)
                    self.n.pose = (.9, 0., 0.)
                    self.tick(.42)
                self.tick(1.)
                self.assertEqual(self.n.recovery_count, 2)

    def test_cancel_at_each_phase_and_new_task_reset(self):
        for phase in ('prepare', 'moving', 'settle'):
            with self.subTest(phase=phase):
                self.setUp()
                self.tick(0.)
                if phase != 'prepare':
                    self.tick(.02)
                if phase == 'settle':
                    self.tick(1.02)
                self.n.stop('Action cancelled')
                self.assertIsNone(self.n.tick(3.))
                self.n.pose_time = self.n.map_time = 3.
                self.n.start((6., 0.), 3.)
                self.assertEqual(self.n.recovery_count, 0)
                self.assertEqual(self.n.planning_failures, 0)

    def test_goal_entered_during_pulse_is_checked_only_after_stopping(self):
        self.begin()
        self.n.pose = (4.8, 0., 0.)
        self.assertEqual(self.tick(.5), (0., .5, 0.))
        self.tick(1.02)
        self.assertTrue(self.n.active)
        self.tick(1.6, revision=2)
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.reason, 'Arrived within 0.50 m')

    def test_feedback_reports_direction_count_and_pause(self):
        self.begin()
        self.assertEqual(self.n.feedback_stage(), 'recovery_left (1/3)')
        self.tick(.5, fresh=False)
        self.assertEqual(self.n.feedback_stage(), 'recovery_settle (1/3): Waiting for pose')


if __name__ == '__main__':
    unittest.main()
