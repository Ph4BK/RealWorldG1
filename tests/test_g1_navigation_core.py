"""Deterministic navigation transitions, with no ROS or robot commands."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from g1_navigation_core import Navigation
from posnav_fmm.errors import GoalUnreachable
from unittest.mock import Mock


class NavigationTest(unittest.TestCase):
    def setUp(self):
        self.plans = []
        self.target = (1., 0.)
        self.blocked = False
        def plan(pose, goal):
            self.plans.append((pose, goal))
            return self.target
        self.n = Navigation(plan, lambda *_: not self.blocked)
        self.n.pose = (0., 0., 0.)
        self.n.pose_time = self.n.map_time = 0.

    def tick(self, t, pose=None, fresh=True):
        if pose is not None:
            self.n.pose = pose
        if fresh:
            self.n.pose_time = self.n.map_time = t
        return self.n.tick(t)

    def start_walk(self):
        self.n.start((5., 0.), 0.)
        self.assertEqual(self.tick(0.), (0., 0.))
        self.assertEqual(self.n.state, 'walk_start')
        self.assertEqual(self.tick(.02), (.5, 0.))

    def test_idle_never_sends_commands(self):
        self.assertIsNone(self.n.tick(100.))

    def test_start_requires_fresh_pose_and_map(self):
        with self.assertRaisesRegex(ValueError, 'pose'):
            self.n.start((5., 0.), .31)
        self.n.pose_time = 2.
        with self.assertRaisesRegex(ValueError, 'map'):
            self.n.start((5., 0.), 2.)

    def test_initial_goal_already_reached(self):
        self.n.start((.4, 0.), 0.)
        self.assertEqual(self.tick(0.), (0., 0.))
        self.assertFalse(self.n.active)
        self.assertEqual(self.plans, [])

    def test_left_threshold_crossing_never_reverses(self):
        self.n.pose = (0., 0., -1.)
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        self.assertEqual(self.tick(.02), (0., 1.))
        self.assertEqual(self.tick(.1, (0., 0., -.13)), (0., 1.))
        self.assertEqual(self.tick(.2, (0., 0., .2)), (0., 0.))
        self.assertEqual(self.n.state, 'turn_settle')
        self.assertEqual(self.tick(.3), (0., 0.))

    def test_right_threshold_is_asymmetric(self):
        self.n.pose = (0., 0., 1.)
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        self.assertEqual(self.tick(.1, (0., 0., .19)), (0., -1.))
        self.assertEqual(self.tick(.2, (0., 0., .17)), (0., 0.))
        self.assertEqual(self.n.state, 'turn_settle')

    def test_initial_turn_selection_boundaries(self):
        cases = [(-.180001, 'turn'), (-.18, 'turn_out'),
                 (-.030001, 'turn_out'), (-.03, 'walk_start'),
                 (0., 'walk_start'), (.03, 'walk_start'),
                 (.030001, 'turn_out'), (.12, 'turn_out'),
                 (.120001, 'turn')]
        for error, state in cases:
            with self.subTest(error=error):
                self.setUp()
                self.n.pose = (0., 0., -error)
                self.n.start((5., 0.), 0.)
                self.assertEqual(self.tick(0.), (0., 0.))
                self.assertEqual(self.n.state, state)
                if state == 'walk_start':
                    self.assertEqual(self.tick(.02), (.5, 0.))
                else:
                    self.assertEqual(self.tick(.02), (0., 1. if error > 0. else -1.))

    def test_small_turn_both_directions_excursion_then_return(self):
        for yaw, sign in [(-.11, 1), (.17, -1)]:
            with self.subTest(sign=sign):
                self.setUp()
                self.n.pose = (0., 0., yaw)
                self.n.start((5., 0.), 0.)
                self.assertEqual(self.tick(0.), (0., 0.))
                self.assertEqual(self.n.state, 'turn_out')
                self.assertEqual(self.tick(.02), (0., float(sign)))
                self.assertEqual(self.tick(.1, (0., 0., yaw + sign*.39)), (0., float(sign)))
                self.assertEqual(self.tick(.2, (0., 0., yaw + sign*.41)), (0., 0.))
                self.assertEqual(self.n.state, 'turn_out_settle')
                self.assertEqual(self.tick(.94), (0., 0.))
                self.assertEqual(self.tick(.96), (0., 0.))
                self.assertEqual(self.n.state, 'turn')
                self.assertEqual(self.tick(.98), (0., float(-sign)))
                self.assertEqual(self.tick(1., (0., 0., sign*.1)), (0., 0.))
                self.assertEqual(self.n.state, 'turn_settle')
                self.assertEqual(self.tick(1.74), (0., 0.))
                self.assertEqual(self.tick(1.75, (0., 0., 0.)), (.5, 0.))
                self.assertEqual(len(self.plans), 1)

    def test_skipped_turn_waits_for_fresh_inputs_and_checks_obstacles(self):
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        self.assertEqual(self.tick(.31, fresh=False), (0., 0.))
        self.assertEqual(self.n.state, 'walk_start')
        self.blocked = True
        self.assertEqual(self.tick(.32, (.02, .01, .01)), (0., 0.))
        self.assertEqual(self.n.origin, (.02, .01))
        self.assertEqual(self.n.state, 'obstacle')

    def test_skipped_turn_can_cancel_before_forward_command(self):
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        self.n.stop('Stopped by user')
        self.assertIsNone(self.tick(.02))

    def test_consecutive_straight_segments_only_wait_for_walk_settle(self):
        self.start_walk()
        self.tick(1., (.9, 0., 0.))
        self.assertEqual(self.n.state, 'walk_settle')
        self.target = (2., 0.)
        self.assertEqual(self.tick(1.49), (0., 0.))
        self.tick(1.5, (1., 0., 0.))
        self.tick(1.52)
        self.assertEqual(self.n.state, 'walk_start')
        self.assertEqual(self.tick(1.54), (.5, 0.))
        self.assertEqual(self.n.origin, (1., 0.))

    def test_return_uses_latest_position_and_heading_after_settle(self):
        self.n.pose = (0., 0., -.1)
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        self.tick(.2, (0., 0., .31))
        # Position and yaw both drift during the first wait.
        self.tick(.96, (.1, .2, .4))
        expected = math.atan2(-.2, .9) - .4
        self.assertAlmostEqual(self.n.turn_error, expected)
        self.assertEqual(self.tick(.98), (0., -1.))
        # Old target heading would stop here; updated bearing still needs turn.
        self.assertEqual(self.tick(1., (.1, .2, .1)), (0., -1.))
        self.assertEqual(self.tick(1.1, (.1, .2, -.1)), (0., 0.))
        self.assertEqual(self.n.state, 'turn_settle')

    def test_excursion_wrap_pause_and_cancel(self):
        for sign in (1, -1):
            with self.subTest(sign=sign):
                self.setUp()
                yaw = sign*3.1
                bearing = yaw + sign*.1
                self.target = (math.cos(bearing), math.sin(bearing))
                self.n.pose = (0., 0., yaw)
                self.n.start((5., 0.), 0.)
                self.tick(0.)
                self.assertEqual(self.tick(.31, fresh=False), (0., 0.))
                angle = yaw + sign*.2
                pose = (0., 0., math.atan2(math.sin(angle), math.cos(angle)))
                self.assertEqual(self.tick(.4, pose), (0., float(sign)))
                self.assertAlmostEqual(self.n.turn_progress, .2)
                angle = yaw + sign*.41
                self.tick(.5, (0., 0., math.atan2(math.sin(angle), math.cos(angle))))
                self.assertEqual(self.n.state, 'turn_out_settle')
                self.assertEqual(self.tick(1.3, fresh=False), (0., 0.))
                self.assertEqual(self.n.state, 'turn_out_settle')
                self.n.stop('Stopped by user')
                self.assertIsNone(self.tick(1.4))

    def test_return_small_error_does_not_repeat_excursion(self):
        self.n.pose = (0., 0., -.1)
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        self.tick(.2, (0., 0., .31))
        self.tick(.96, (0., 0., .05))
        self.assertEqual(self.tick(.98), (0., 0.))
        self.assertEqual(self.n.state, 'turn_settle')

    def test_forward_stops_with_fifteen_cm_remaining(self):
        self.start_walk()
        self.assertEqual(self.tick(1., (.849, .3, 0.)), (.5, 0.))
        self.assertEqual(self.tick(1.02, (.851, .3, 0.)), (0., 0.))
        self.assertEqual(self.n.state, 'walk_settle')

    def test_wrap_boundary_does_not_false_stop(self):
        self.n.pose = (0., 0., -3.13)
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        self.assertEqual(self.tick(.02, (0., 0., 3.13)), (0., 1.))
        self.assertEqual(self.tick(.04, (0., 0., -3.10)), (0., 1.))

    def test_forward_projection_and_overshoot(self):
        self.start_walk()
        self.assertEqual(self.tick(1., (.84, .4, 0.)), (.5, 0.))
        self.assertEqual(self.tick(1.1, (1.05, .4, 0.)), (0., 0.))
        self.assertEqual(self.n.state, 'walk_settle')
        self.assertEqual(self.n.origin, (0., 0.))
        self.assertEqual(self.n.direction, (1., 0.))
        self.tick(1.59)
        self.assertEqual(len(self.plans), 1)
        self.tick(1.61)
        self.assertEqual(self.n.state, 'plan')

    def test_pose_pause_resume_preserves_reference(self):
        self.start_walk()
        self.n.map_time = 1.1
        self.assertEqual(self.tick(1.1, fresh=False), (0., 0.))
        self.assertEqual(self.n.state, 'walk')
        self.assertIn('pose', self.n.paused)
        self.assertEqual(self.tick(1.2, (.2, .1, 0.)), (.5, 0.))
        self.assertEqual(self.n.origin, (0., 0.))
        self.assertEqual(len(self.plans), 1)

    def test_map_pause_resume_requires_both_inputs(self):
        self.start_walk()
        self.n.pose_time = 2.
        self.assertEqual(self.tick(2., fresh=False), (0., 0.))
        self.assertIn('map', self.n.paused)
        self.n.map_time = 2.5
        self.assertEqual(self.tick(2.5, fresh=False), (0., 0.))
        self.assertIn('pose', self.n.paused)
        self.assertEqual(self.tick(2.6, (.95, 0., 0.)), (0., 0.))
        self.assertEqual(self.n.state, 'walk_settle')
        self.assertEqual(len(self.plans), 1)

    def test_turn_pause_keeps_direction(self):
        self.n.pose = (0., 0., -1.)
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        self.assertEqual(self.tick(.31, fresh=False), (0., 0.))
        self.assertEqual(self.tick(.4, (0., 0., -.5)), (0., 1.))
        self.assertEqual(self.tick(.5, (0., 0., .1)), (0., 0.))

    def test_obstacle_removed_after_two_seconds(self):
        self.start_walk()
        self.blocked = True
        self.assertEqual(self.tick(1.), (0., 0.))
        self.blocked = False
        self.assertEqual(self.tick(2.99), (0., 0.))
        self.assertEqual(self.tick(3.), (.5, 0.))
        self.assertEqual(len(self.plans), 1)

    def test_persistent_obstacle_replans_from_actual_pose(self):
        self.start_walk()
        self.blocked = True
        self.tick(1.)
        self.tick(3., (.1, .05, 0.))
        self.assertEqual(self.n.state, 'plan')
        self.blocked = False
        self.target = (.1, 1.)
        self.tick(3.02)
        self.assertEqual(self.plans[-1][0], (.1, .05))
        self.assertEqual(self.n.state, 'turn')

    def test_obstacle_timeout_with_invalid_map_keeps_waiting(self):
        self.start_walk()
        self.blocked = True
        self.tick(1.)
        self.n.pose_time = 4.
        self.tick(4., fresh=False)
        self.assertEqual(self.n.state, 'obstacle')
        self.assertEqual(len(self.plans), 1)
        self.blocked = False
        self.assertEqual(self.tick(4.1), (.5, 0.))

    def test_goal_recheck_after_settle(self):
        self.start_walk()
        self.n.goal = (1.3, 0.)
        self.assertEqual(self.tick(1., (.81, 0., 0.)), (0., 0.))
        self.tick(1.49)
        self.assertTrue(self.n.active)
        self.tick(1.5, (.79, 0., 0.))
        self.assertEqual(self.n.state, 'plan')
        self.assertTrue(self.n.active)
        self.tick(1.52, (.85, 0., 0.))
        self.assertFalse(self.n.active)

    def test_goal_reached_during_turn_wait_skips_walking(self):
        self.n.pose = (0., 0., -1.)
        self.n.start((1., 0.), 0.)
        self.tick(0.)
        self.tick(.02, (0., 0., -.1))
        self.assertEqual(self.tick(.77, (.6, 0., 0.)), (0., 0.))
        self.assertEqual(self.n.state, 'walk_settle')

    def test_no_stg_is_not_success(self):
        self.target = None
        self.n.start((5., 0.), 0.)
        self.assertEqual(self.tick(0.), (0., 0.))
        self.assertFalse(self.n.active)
        self.assertIn('No valid STG', self.n.reason)

    def test_blocked_segment_replans_every_second_and_resumes(self):
        self.blocked = True
        self.n.start((5., 0.), 0.)
        for t, count in [(0., 1), (.5, 1), (.99, 1), (1., 2), (1.99, 2)]:
            self.assertEqual(self.tick(t), (0., 0.))
            self.assertTrue(self.n.active)
            self.assertEqual(self.n.state, 'plan_wait')
            self.assertEqual(self.n.reason, 'STG segment is blocked')
            self.assertEqual(len(self.plans), count)
        self.blocked = False
        self.target = (1.5, 0.)
        self.assertEqual(self.tick(2., (.2, 0., 0.)), (0., 0.))
        self.assertEqual(self.plans[-1][0], (.2, 0.))
        self.assertEqual(self.n.stg, (1.5, 0.))
        self.assertEqual(self.n.state, 'walk_start')
        self.assertEqual(self.tick(2.02), (.5, 0.))

    def test_blocked_segment_retry_pauses_for_stale_input_and_can_cancel(self):
        self.blocked = True
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        self.n.pose_time = 2.
        self.assertEqual(self.tick(2., fresh=False), (0., 0.))
        self.assertEqual(len(self.plans), 1)
        self.tick(2.1)
        self.assertEqual(len(self.plans), 2)
        self.n.stop('Stopped by user')
        self.assertIsNone(self.tick(4.))
        self.assertEqual(len(self.plans), 2)

    def test_no_motion_timeout_or_no_progress_stop(self):
        self.start_walk()
        for t in [10., 100., 1000.]:
            self.assertEqual(self.tick(t), (.5, 0.))
            self.assertTrue(self.n.active)

    def test_cancel_does_not_resume(self):
        self.start_walk()
        self.n.stop('Goal redrawn')
        self.assertIsNone(self.tick(2.))
        self.assertFalse(self.n.active)

    def test_unreachable_retries_once_per_second_and_resumes(self):
        error = GoalUnreachable('Goal is unreachable from the current position')
        self.n.plan = Mock(side_effect=[error, error, (1., 0.)])
        self.n.start((5., 0.), 0.)
        for t, calls in [(0., 1), (.02, 1), (.99, 1), (1., 2), (1.99, 2)]:
            self.assertEqual(self.tick(t), (0., 0.))
            self.assertEqual(self.n.plan.call_count, calls)
            self.assertTrue(self.n.active)
            self.assertEqual(self.n.state, 'plan_wait')
            self.assertEqual(self.n.reason, str(error))
        self.assertEqual(self.tick(2., (.1, 0., 0.)), (0., 0.))
        self.n.plan.assert_called_with((.1, 0.), (5., 0.))
        self.assertEqual(self.n.state, 'walk_start')
        self.assertNotIn('unreachable', self.n.reason)
        self.assertEqual(self.tick(2.02), (.5, 0.))

    def test_retry_waits_for_inputs_then_can_be_cancelled(self):
        self.n.plan = Mock(side_effect=GoalUnreachable('Goal is unreachable from the current position'))
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        self.n.pose_time = 2.
        self.assertEqual(self.tick(2., fresh=False), (0., 0.))
        self.assertEqual(self.n.plan.call_count, 1)
        self.tick(2.1)
        self.assertEqual(self.n.plan.call_count, 2)
        self.n.stop('Goal redrawn')
        self.assertIsNone(self.tick(4.))
        self.assertEqual(self.n.plan.call_count, 2)

    def test_other_planning_errors_do_not_enter_retry(self):
        self.n.plan = Mock(side_effect=ValueError('Robot or goal is outside the map'))
        self.n.start((5., 0.), 0.)
        with self.assertRaisesRegex(ValueError, 'outside'):
            self.tick(0.)
        self.assertNotEqual(self.n.state, 'plan_wait')


if __name__ == '__main__':
    unittest.main()
