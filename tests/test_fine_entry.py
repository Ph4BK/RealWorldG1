"""Actual PosNav handoff/command regressions for v3; no ROS or robot IO."""
import argparse
import importlib.util
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from g1_final_approach import PosNav, FineApproachConfig
from g1_navigation_core import velocity_xyz
from g1_navigation_options import (add_navigation_options, checked_options, options_from_args,
                                   forward_navigation_options)


class FineEntryTest(unittest.TestCase):
    def controller(self, alpha=0., **options):
        n = PosNav(Mock(side_effect=lambda p, g: g), Mock(return_value=True),
            Mock(side_effect=lambda g: dict(pre=None, reason='direct target')),
            fine=FineApproachConfig(**options))
        n.pose = (-.4, 0., -math.radians(alpha))
        n.pose_time = n.map_time = 0.; n.pose_source_stamp_ns = 1_000_000_000
        n.start((0., 0., 0.), 0.)
        n.tick(0.)  # production plan -> direct-goal handoff
        return n

    def tick(self, n, t, pose=None, source=None, sample_time=None, map_time=None):
        if pose is not None:
            n.pose = pose
        n.pose_time = t if sample_time is None else sample_time
        n.map_time = t if map_time is None else map_time
        n.pose_source_stamp_ns = int((1+t)*1e9) if source is None else source
        cmd = n.tick(t)
        return None if cmd is None else velocity_xyz(cmd)

    def events(self, n, name):
        return [d for e, d in n.final_events if e == name]

    def test_entry_threshold_boundaries_and_signed_rotation(self):
        for angle, aligning in ((29.9, False), (30., False), (30.1, True), (-30.1, True)):
            with self.subTest(angle=angle):
                n = self.controller(angle, fine_entry_align=True)
                self.assertEqual(n.state == 'fine_entry_align', aligning)
                if aligning:
                    command = self.tick(n, .1)
                    self.assertEqual(command[:2], (0., 0.))
                    self.assertGreater(command[2]*angle, 0.)
                    self.assertIsNone(n.position_direction)

    def test_ten_degrees_stops_rotation_then_confirms_before_fine(self):
        n = self.controller(89., fine_entry_align=True)
        self.assertEqual(self.tick(n, .1, (-.4, 0., -math.radians(10.))), (0., 0., 0.))
        self.assertEqual(n.state, 'fine_entry_settle')
        for i in range(2, 9):
            self.assertEqual(self.tick(n, i*.1), (0., 0., 0.))
            self.assertNotEqual(n.state, 'fine_approach')
        self.tick(n, .9)
        self.assertEqual(n.state, 'fine_approach')
        self.assertTrue(self.events(n, 'fine_entry_align_completed'))

    def test_rebases_ray_from_latest_translated_pose_after_stopping(self):
        n = self.controller(89., fine_entry_align=True)
        pose = (-.3, -.1, math.atan2(.1, .3))
        self.tick(n, .1, pose)
        for i in range(2, 10):
            self.tick(n, i*.1)
        self.assertEqual(n.state, 'fine_approach')
        self.assertEqual(n.position_origin, pose)
        self.assertAlmostEqual(n.position_remaining(), math.hypot(.3, .1))
        self.assertAlmostEqual(n.evidence()['lateral_m'], 0.)
        self.assertNotEqual(n.position_direction, (1., 0.))

    def test_turning_samples_cannot_confirm_rest_or_start_forward(self):
        n = self.controller(50., fine_entry_align=True)
        self.tick(n, .1, (-.4, 0., 0.))
        for i in range(2, 12):
            command = self.tick(n, i*.1, (-.4, 0., .04 if i%2 else -.04))
            self.assertEqual(command, (0., 0., 0.))
            self.assertEqual(n.state, 'fine_entry_settle')
        for i in range(12, 16):
            self.tick(n, i*.1, (-.4, 0., 0.))
        self.assertEqual(n.state, 'fine_approach')

    def test_pose_dropout_freezes_budget_and_recovery_continues(self):
        n = self.controller(89., fine_entry_align=True)
        self.tick(n, .1); self.tick(n, .2)
        before = n.entry_samples.settle_fresh_s
        self.assertEqual(self.tick(n, 8., sample_time=.2), (0., 0., 0.))
        self.assertEqual(n.state, 'fine_entry_align')
        self.assertTrue(n.active)
        self.tick(n, 8.1)
        self.assertEqual(n.entry_samples.settle_fresh_s, before)
        self.assertGreater(self.tick(n, 8.2)[2], 0.)

    def test_map_dropout_and_duplicate_sources_do_not_spend_budget(self):
        n = self.controller(60., fine_entry_align=True)
        self.assertEqual(self.tick(n, 3., map_time=0.), (0., 0., 0.))
        for i in range(31, 101):
            self.tick(n, i*.1, source=123)
        self.assertTrue(n.active)
        self.assertEqual(n.entry_samples.settle_fresh_s, 0.)

    def test_effective_source_timeout_aborts_with_zero(self):
        n = self.controller(89., fine_entry_align=True)
        for i in range(1, 65):
            command = self.tick(n, i*.1)
            if not n.active:
                break
        self.assertFalse(n.active)
        self.assertEqual(command, (0., 0., 0.))
        self.assertEqual(n.reason, 'fine_entry_align_timeout')
        self.assertEqual(self.events(n, n.reason)[-1]['detail'], 'effective_time_limit')

    def test_slow_but_valid_source_still_spends_active_alignment_budget(self):
        n = self.controller(89., fine_entry_align=True)
        for i in range(1, 34):
            command = self.tick(n, i*.2)  # 5Hz remains valid under the original .3s pose gate.
            if not n.active:
                break
        self.assertFalse(n.active)
        self.assertEqual(command, (0., 0., 0.))
        self.assertEqual(n.reason, 'fine_entry_align_timeout')

    def test_settle_drift_rechecks_heading_and_has_attempt_bound(self):
        n = self.controller(60., fine_entry_align=True, fine_entry_attempts=1)
        self.tick(n, .1, (-.4, 0., 0.))
        for i in range(2, 10):
            command = self.tick(n, i*.1, (-.4, 0., -math.radians(12.)))
        self.assertFalse(n.active)
        self.assertEqual(command, (0., 0., 0.))
        self.assertEqual(self.events(n, 'fine_entry_align_timeout')[-1]['detail'], 'attempt_limit')
        self.assertEqual(n.entry_attempts, 1)

    def test_blocked_entry_stays_zero_without_replan_or_pre_change(self):
        n = self.controller(70., fine_entry_align=True)
        self.tick(n, .1); before = n.entry_samples.settle_fresh_s
        n.clear.return_value = False
        for t in (.2, 3., 10.):
            self.assertEqual(self.tick(n, t), (0., 0., 0.))
        self.assertEqual(n.state, 'fine_entry_obstacle')
        self.assertEqual(n.entry_samples.settle_fresh_s, before)
        n.plan_callback.assert_not_called()
        self.assertEqual(n.route_callback.call_count, 1)
        n.clear.return_value = True
        self.assertEqual(self.tick(n, 10.1), (0., 0., 0.))
        self.assertGreater(self.tick(n, 10.2)[2], 0.)

    def test_obstacle_resume_rechecks_entry_and_preserves_ray_if_no_alignment(self):
        for angle, aligning in ((60., True), (20., False)):
            n = self.controller(0., fine_entry_align=True)
            direction = n.position_direction
            n.clear.return_value = False
            self.tick(n, .1)
            self.assertEqual(n.state, 'fine_obstacle')
            n.clear.return_value = True
            self.tick(n, 2.2, (-.3, .04, -math.radians(angle)))
            self.assertEqual(n.state == 'fine_entry_align', aligning)
            if not aligning:
                self.assertEqual(n.position_direction, direction)

    def test_persistent_fine_obstacle_has_no_new_replan_when_v3_enabled(self):
        # 11-25 review F2: no replan/pre change, but the wait must be bounded and end
        # with a clean failure instead of hanging until the Action timeout.
        for options in (dict(fine_entry_align=True), dict(fine_lateral_guard=True)):
            n = self.controller(**options)
            n.clear.return_value = False
            for t in (.1, 2.2, 4.3, 6.4):
                self.tick(n, t)
            self.assertFalse(n.active)
            self.assertEqual(n.reason, 'fine_passage_blocked')
            self.assertEqual(len(self.events(n, 'fine_passage_blocked')), 1)
            n.plan_callback.assert_not_called()
            self.assertEqual(n.route_callback.call_count, 1)

    def test_blocked_entry_also_ends_after_three_waits(self):
        n = self.controller(70., fine_entry_align=True)
        n.clear.return_value = False
        for t in (.1, .2, 2.2, 4.3):
            self.assertEqual(self.tick(n, t), (0., 0., 0.))
            self.assertTrue(n.active)
        self.assertEqual(self.tick(n, 6.4), (0., 0., 0.))
        self.assertFalse(n.active)
        self.assertEqual(n.reason, 'fine_passage_blocked')
        row = self.events(n, n.reason)[-1]
        self.assertEqual(row['blocked_state'], 'fine_entry_obstacle')
        self.assertEqual(row['waits'], 3)
        self.assertEqual(row['wait_limit'], 3)
        self.assertEqual(n.entry_samples.settle_fresh_s, 0.)
        n.plan_callback.assert_not_called()
        self.assertEqual(n.route_callback.call_count, 1)
        self.assertIsNone(self.tick(n, 9.))
        self.assertEqual(len(self.events(n, 'fine_passage_blocked')), 1)

    def test_resolved_passage_resets_budget_in_both_phases(self):
        for angle in (0., 70.):
            with self.subTest(angle=angle):
                n = self.controller(angle, fine_entry_align=True)
                n.clear.return_value = False
                for t in (.1, 2.2, 4.3):
                    self.tick(n, t)
                self.assertEqual(n.passage_waits, 2)
                n.clear.return_value = True
                self.assertEqual(self.tick(n, 6.4), (0., 0., 0.))
                self.assertTrue(n.active)
                self.assertEqual(n.passage_waits, 0)
                n.clear.return_value = False
                for t in (6.5, 8.6, 10.7):
                    self.assertEqual(self.tick(n, t), (0., 0., 0.))
                    self.assertTrue(n.active)
                self.assertEqual(self.tick(n, 12.8), (0., 0., 0.))
                self.assertEqual(n.reason, 'fine_passage_blocked')

    def test_stale_input_and_fast_ticks_do_not_consume_extra_waits(self):
        for angle in (0., 70.):
            with self.subTest(angle=angle):
                n = self.controller(angle, fine_entry_align=True)
                n.clear.return_value = False
                self.tick(n, .1)
                self.assertEqual(self.tick(n, 20., sample_time=.1), (0., 0., 0.))
                self.assertEqual(self.tick(n, 20.1, map_time=0.), (0., 0., 0.))
                self.assertEqual(n.passage_waits, 0)
                for t in (20.2, 20.3, 20.4):
                    self.tick(n, t)
                self.assertEqual(n.passage_waits, 1)
                self.assertTrue(n.active)
                self.tick(n, 22.3)
                self.assertTrue(n.active)
                self.assertEqual(self.tick(n, 24.4), (0., 0., 0.))
                self.assertEqual(n.reason, 'fine_passage_blocked')

    def test_custom_wait_limit_and_new_goal_reset(self):
        n = self.controller(fine_lateral_guard=True, fine_passage_waits=1)
        n.clear.return_value = False
        self.tick(n, .1)
        self.assertTrue(n.active)
        self.assertEqual(self.tick(n, 2.2), (0., 0., 0.))
        self.assertEqual(n.reason, 'fine_passage_blocked')
        n.clear.return_value = True
        n.pose_time = n.map_time = 3.
        n.start((0., 0., 0.), 3.); self.tick(n, 3.)
        self.assertEqual(n.passage_waits, 0)
        self.assertTrue(n.active)

    def test_default_off_keeps_legacy_obstacle_replan(self):
        n = self.controller()
        n.clear.return_value = False
        self.tick(n, .1)
        self.assertEqual(self.tick(n, 2.2), (0., 0., 0.))
        self.assertEqual(n.state, 'plan')
        self.assertTrue(n.active)
        self.assertFalse(self.events(n, 'fine_passage_blocked'))
        self.assertEqual(len(self.events(n, 'fine_approach_replan')), 1)

    def test_close_goal_keeps_existing_stop_precedence(self):
        n = self.controller()
        n.stop(); n.fine = FineApproachConfig(fine_entry_align=True, fine_stop_layered=True)
        n.pose = (-.03, 0., math.pi); n.pose_time = n.map_time = 1.
        n.start((0., 0., 0.), 1.)
        self.assertEqual(n.state, 'fine_approach')
        self.assertEqual(self.tick(n, 1.), (0., 0., 0.))
        self.assertEqual(n.state, 'fine_position_settle')
        self.assertFalse(self.events(n, 'fine_entry_align_started'))

    def test_exact_goal_never_attempts_bearing_alignment(self):
        n = self.controller(); n.stop()
        n.fine = FineApproachConfig(fine_entry_align=True)
        n.pose = (0., 0., math.pi); n.pose_time = n.map_time = 1.
        n.start((0., 0., 0.), 1.); self.tick(n, 1.)
        self.assertEqual(n.state, 'fine_position_settle')

    def test_entry_uses_target_bearing_not_goal_yaw(self):
        n = self.controller(0., fine_entry_align=True)
        n.stop(); n.pose_time = n.map_time = 1.
        n.start((0., 0., math.pi/2), 1.); self.tick(n, 1.)
        self.assertEqual(n.state, 'fine_approach')

    def test_pre_terminal_handoff_also_passes_entry_gate(self):
        n = self.controller(0., fine_entry_align=True)
        n.state = 'walk_settle'; n.until = .1; n.terminal_stg = True
        n.stg = (-.4, 0.); n.direction = (1., 0.)
        self.tick(n, .1, (-.4, 0., -math.pi/2))
        self.assertEqual(n.state, 'fine_entry_align')

    def test_lateral_gate_boundaries_both_signs(self):
        for lateral in (.099, .100, .101, -.099, -.100, -.101):
            with self.subTest(lateral=lateral):
                n = self.controller(fine_stop_layered=True, fine_lateral_guard=True)
                command = self.tick(n, .1, (-.08, -lateral, 0.))
                if abs(lateral) > .100:
                    self.assertEqual(command, (0., 0., 0.))
                    self.assertFalse(n.active)
                    self.assertEqual(n.reason, 'fine_lateral_exceeded')
                    data = self.events(n, n.reason)[-1]
                    self.assertAlmostEqual(data['lateral_m'], lateral)
                    self.assertIn('alpha_deg', data)
                    self.assertEqual(data['source_stamp_ns'], int(1.1e9))
                else:
                    self.assertEqual(command, (.25, 0., 0.))
                    self.assertTrue(n.active)

    def test_lateral_guard_does_not_apply_outside_euclidean_window(self):
        for layered in (True, False):
            n = self.controller(fine_stop_layered=layered, fine_lateral_guard=True)
            self.tick(n, .1, (-.08, -.16, 0.))
            self.assertTrue(n.active)
            self.assertFalse(self.events(n, 'fine_lateral_exceeded'))

    def test_existing_stop_decision_precedes_lateral_guard(self):
        n = self.controller(fine_stop_layered=True, fine_lateral_guard=True)
        self.assertEqual(self.tick(n, .1, (-.03, -.11, 0.)), (0., 0., 0.))
        self.assertEqual(n.state, 'fine_position_settle')
        self.assertTrue(n.active)
        self.assertFalse(self.events(n, 'fine_lateral_exceeded'))

    def test_stale_input_cannot_trigger_lateral_failure(self):
        n = self.controller(fine_lateral_guard=True)
        self.assertEqual(self.tick(n, 1., (-.08, -.11, 0.), sample_time=0.), (0., 0., 0.))
        self.assertTrue(n.active)
        self.assertFalse(self.events(n, 'fine_lateral_exceeded'))
        self.assertEqual(self.tick(n, 1.1), (0., 0., 0.))
        self.assertEqual(n.reason, 'fine_lateral_exceeded')
        self.assertIsNone(self.tick(n, 2.))

    def test_entry_events_keep_source_and_provisional_error_axes(self):
        n = self.controller(50., fine_entry_align=True)
        row = self.events(n, 'fine_entry_align_started')[-1]
        self.assertEqual(row['source_stamp_ns'], 1_000_000_000)
        self.assertAlmostEqual(row['lateral_m'], 0.)
        self.assertAlmostEqual(row['projected_remaining_m'], .4)
        self.assertAlmostEqual(row['alpha_deg'], 50.)
        self.assertEqual(row['attempts'], 1)
        self.assertEqual(row['position_direction'], row['entry_direction'])

    def test_entry_check_records_below_threshold_without_rebasing(self):
        n = self.controller(20., fine_entry_align=True)
        self.assertEqual(n.state, 'fine_approach')
        row = self.events(n, 'fine_entry_gate_checked')[-1]
        self.assertFalse(row['triggered'])
        self.assertAlmostEqual(row['alpha_deg'], 20.)
        self.assertAlmostEqual(row['projected_remaining_m'], .4)
        self.assertEqual(row['source_stamp_ns'], 1_000_000_000)

    def test_rate_cap_and_six_second_budget_are_independent_constraints(self):
        # 11-25 review F1: entry alignment is a coarse turn and must keep the legacy
        # 1.0 rad/s cap; the heading_align candidate (.2) must not make the 6 s entry
        # budget physically impossible.
        n = self.controller(89., fine_entry_align=True, fine_align_wz=.2)
        first = self.tick(n, .1)
        self.assertEqual(first, (0., 0., 1.))
        self.assertLess(self.events(n, 'fine_entry_align_started')[-1]['minimum_turn_s'], 6.)
        # An ideal 1 rad/s motor removes 79 degrees in ~1.38 s, well inside the budget.
        yaw = -math.radians(89.)
        for i in range(2, 40):
            cmd = self.tick(n, i*.1, (-.4, 0., yaw))
            if cmd is None:
                break
            yaw += cmd[2]*.1
            if n.state == 'fine_approach':
                break
        self.assertNotEqual(n.reason, 'fine_entry_align_timeout')
        self.assertEqual(self.events(n, 'fine_entry_align_completed') != [], True)

    def test_entry_align_ignores_heading_only_rate_cap(self):
        n = self.controller(60., fine_entry_align=True, fine_align_wz=.2)
        self.assertEqual(self.tick(n, .1), (0., 0., 1.))
        # finish entry align, then heading_align must still honour the .2 cap
        yaw = -math.radians(60.)
        t = .2
        while n.state != 'fine_approach' and t < 6.:
            cmd = self.tick(n, t, (-.4, 0., yaw))
            if cmd is None:
                break
            yaw += cmd[2]*.1
            t += .1
        self.assertEqual(n.state, 'fine_approach')

        # Continue through the real position-stop/settle handoff, not a manual
        # heading_align state assignment. The same controller still caps final yaw.
        self.assertEqual(self.tick(n, t+.1, (0., 0., -.5)), (0., 0., 0.))
        self.assertEqual(n.state, 'fine_position_settle')
        self.assertEqual(self.tick(n, t+.7), (0., 0., 0.))
        self.assertEqual(n.state, 'heading_align')
        self.assertEqual(self.tick(n, t+.8), (0., 0., .2))

    def test_entry_timeout_and_attempts_reset_on_new_goal(self):
        n = self.controller(60., fine_entry_align=True)
        self.tick(n, .1); self.tick(n, .2)
        n.stop(); n.pose_time = n.map_time = 1.
        n.start((0., 0., 0.), 1.); self.tick(n, 1.)
        self.assertEqual(n.entry_attempts, 1)
        self.assertEqual(n.entry_samples.settle_fresh_s, 0.)

    def test_multiple_entries_share_effective_budget_without_charging_walk_time(self):
        n = self.controller(60., fine_entry_align=True)
        self.tick(n, .1, (-.4, 0., 0.))
        for i in range(2, 10):
            self.tick(n, i*.1)
        before = n.entry_samples.settle_fresh_s
        n.clear.return_value = False; self.tick(n, 1.)
        n.clear.return_value = True; self.tick(n, 4., (-.3, 0., -math.pi/3))
        self.assertEqual(n.entry_attempts, 2)
        self.tick(n, 4.1)
        self.assertAlmostEqual(n.entry_samples.settle_fresh_s, before)
        self.tick(n, 4.2)
        self.assertAlmostEqual(n.entry_samples.settle_fresh_s, before+.1)

    def test_duplicate_below_lateral_threshold_does_not_accumulate_failure(self):
        n = self.controller(fine_stop_layered=True, fine_lateral_guard=True)
        for i in range(1, 30):
            self.tick(n, i*.1, (-.08, -.1, 0.), source=123)
            self.assertTrue(n.active)
        self.assertFalse(self.events(n, 'fine_lateral_exceeded'))

    def test_v3_off_matches_frozen_v2_with_v2_features_enabled_or_disabled(self):
        from frozen_baselines import baseline_path
        path = baseline_path('G1_ENTRY_BASELINE', 'fine_v2.py')
        spec = importlib.util.spec_from_file_location('frozen_v2_entry_baseline', path)
        old = importlib.util.module_from_spec(spec); sys.modules[spec.name] = old; spec.loader.exec_module(old)
        for enabled in (False, True):
            for pre in (None, (.4, 0.)):
                nodes = []
                for cls in (old.PosNav, PosNav):
                    fine = FineApproachConfig(fine_stop_layered=enabled, fine_goal_behind=enabled,
                        fine_pre_fallback=enabled, position_settle_fresh=enabled,
                        fine_align_wz=.2 if enabled else 1.)
                    n = cls(lambda p, g: g, lambda *_: True,
                            lambda g: dict(pre=pre, reason='same fixture'), fine=fine)
                    n.pose = (0., 0., 0.); n.pose_time = n.map_time = 0.
                    n.pose_source_stamp_ns = 1_000_000_000; n.start((1., 0., .7), 0.)
                    nodes.append(n)
                for i in range(601):
                    t = i*.02; sensor_t = (i//5)*.1
                    row = []
                    for n in nodes:
                        n.pose = (min(1.02, sensor_t*.16), .002*math.sin(sensor_t), 0. if t < 8 else .75)
                        n.pose_time = sensor_t if not 2. < t < 2.5 else 2.
                        n.map_time = t; n.pose_source_stamp_ns = int((1+sensor_t)*1e9)
                        row.append((n.tick(t), n.state, n.active, n.reason, n.pre))
                    self.assertEqual(row[0], row[1], (enabled, pre, t))

    def test_new_cli_options_roundtrip_and_reject_invalid_values(self):
        parser = argparse.ArgumentParser(); add_navigation_options(parser)
        args = parser.parse_args(['--fine-entry-align', '--fine-lateral-guard',
            '--fine-entry-align-deg', '35', '--fine-entry-done-deg', '9', '--fine-lateral-max', '.11',
            '--fine-passage-waits', '4'])
        self.assertEqual(options_from_args(args)['fine_passage_waits'], 4)
        self.assertEqual(options_from_args(args), options_from_args(parser.parse_args(forward_navigation_options(args))))
        self.assertEqual(forward_navigation_options(parser.parse_args([])), [])
        for bad in (dict(fine_entry_align=1), dict(fine_entry_align_deg=10.),
                dict(fine_entry_done_deg=31.), dict(fine_entry_align_timeout=0.),
                dict(fine_entry_attempts=1.5), dict(fine_lateral_max=float('nan')),
                dict(fine_lateral_max=.15), dict(fine_passage_waits=0), dict(fine_passage_waits=33),
                dict(fine_passage_waits=1.5), dict(fine_passage_waits=True)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                checked_options(**bad)


if __name__ == '__main__':
    unittest.main()
