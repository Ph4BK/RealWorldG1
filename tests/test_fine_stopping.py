"""Real PosNav entry/commands; source-sample stopping evidence, no robot IO."""
import argparse
from dataclasses import asdict
import importlib.util
import math
import os
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from g1_final_approach import PosNav, FineApproachConfig
from g1_navigation_core import velocity_xyz
from g1_navigation_options import (add_navigation_options, options_from_args,
                                   forward_navigation_options, checked_options)


class FineStoppingTest(unittest.TestCase):
    def controller(self, **options):
        n = PosNav(lambda p, g: g, lambda *_: True,
                   lambda g: dict(pre=None, reason='no pre'), fine=FineApproachConfig(**options))
        n.pose = (-.4, 0., 0.)
        n.pose_time = n.map_time = 0.
        n.pose_source_stamp_ns = 1_000_000_000
        n.start((0., 0., 0.), 0.)
        n.tick(0.)  # actual direct-goal handoff
        self.assertEqual(n.state, 'fine_approach')
        return n

    def tick(self, n, t, pose=None, source=None, sample_time=None):
        if pose is not None:
            n.pose = pose
        n.map_time = t
        n.pose_time = t if sample_time is None else sample_time
        n.pose_source_stamp_ns = int((1+t)*1e9) if source is None else source
        command = n.tick(t)
        return None if command is None else velocity_xyz(command)

    def events(self, n, name):
        return [d for e, d in n.final_events if e == name]

    def test_circle_boundaries(self):
        for distance, stopped in ((.08, True), (.079, True), (.081, False)):
            with self.subTest(distance=distance):
                n = self.controller(fine_stop_layered=True)
                self.tick(n, .1, (-distance, 0., 0.))
                self.assertEqual(n.state == 'fine_position_settle', stopped)

    def test_projected_gate_boundaries_and_lateral_miss(self):
        for distance, remaining, stopped in ((.15, .04, True), (.150001, .04, False),
                (.149999, .04, True), (.14, .040001, False), (.20, .02, False), (.20, 0., False)):
            with self.subTest(distance=distance, remaining=remaining):
                n = self.controller(fine_stop_layered=True)
                lateral = math.sqrt(distance**2-remaining**2)
                command = self.tick(n, .1, (-remaining, -lateral, 0.))
                self.assertEqual(n.state == 'fine_position_settle', stopped)
                if distance > .15:
                    self.assertFalse(self.events(n, 'fine_position_stop'))
                    self.assertEqual(command[0], 0.)  # no blind straight through a lateral miss
                    self.assertGreater(command[2], 0.)

    def test_passed_plane_is_logged_without_chasing_back(self):
        n = self.controller(fine_stop_layered=True)
        self.assertEqual(self.tick(n, .1, (.06, -.10, 0.)), (0., 0., 0.))
        self.assertEqual(n.state, 'fine_position_settle')
        data = self.events(n, 'fine_passed_goal')[-1]
        self.assertAlmostEqual(data['projected_remaining_m'], -.06)
        self.assertAlmostEqual(data['distance_m'], math.hypot(.06, .10))
        self.assertAlmostEqual(data['lateral_abs_m'], .10)

    def test_passed_plane_diagnostic_survives_stop_circle_priority(self):
        n = self.controller(fine_stop_layered=True)
        self.tick(n, .1, (.06, 0., 0.))
        self.assertEqual(n.state, 'fine_position_settle')
        self.assertTrue(self.events(n, 'fine_passed_goal'))
        self.assertEqual(self.events(n, 'fine_position_stop')[-1]['stop_basis'], 'radius')

    def test_behind_aborts_and_never_rotates_afterwards(self):
        for layered in (False, True):
            n = self.controller(fine_stop_layered=layered, fine_goal_behind=True)
            self.assertEqual(self.tick(n, .1, (-.25, 0., -math.radians(130))), (0., 0., 0.))
            self.assertFalse(n.active)
            self.assertEqual(n.reason, 'final_goal_behind')
            self.assertIsNone(self.tick(n, 2.))
            self.assertFalse(self.events(n, 'fine_position_settled'))
            self.assertAlmostEqual(self.events(n, 'final_goal_behind')[-1]['rel_deg'], 130.)

    def test_behind_has_priority_over_legacy_plane_stop(self):
        n = self.controller(fine_goal_behind=True)
        self.tick(n, .1, (.2, 0., 0.))
        self.assertEqual(n.reason, 'final_goal_behind')

    def test_behind_limits_and_near_goal(self):
        for distance, yaw, state in ((.03, math.pi, 'fine_position_settle'),
                (0., 0., 'fine_position_settle'), (.31, math.pi, 'fine_approach'),
                (.25, math.radians(119.99), 'fine_approach')):
            with self.subTest(distance=distance, yaw=yaw):
                n = self.controller(fine_goal_behind=True)
                self.tick(n, .1, (-distance, 0., yaw))
                self.assertTrue(n.active)
                self.assertEqual(n.state, state)

    def test_l4_side_approach_does_not_use_goal_yaw_projection(self):
        n = self.controller(fine_stop_layered=True)
        n.final_goal = (0., 0., math.pi/3)
        # goal-yaw projection is 3.5cm but actual approach remaining is 24cm.
        self.tick(n, .1, (-.24, .0981495458, 0.))
        self.assertEqual(n.state, 'fine_approach')
        self.assertGreater(n.position_remaining(), .20)
        self.assertLess(n.evidence()['goal_yaw_remaining_m'], .04)

    def test_legacy_lateral_plane_and_blind_segment_unchanged_when_off(self):
        n = self.controller()
        self.assertEqual(self.tick(n, .1, (-.1, -.4, math.pi)), (.25, 0., 0.))
        self.assertEqual(self.tick(n, .2, (.01, -.4, math.pi)), (0., 0., 0.))
        self.assertEqual(n.state, 'fine_position_settle')

    def test_all_disabled_commands_match_frozen_previous_source(self):
        from frozen_baselines import baseline_path
        path = baseline_path('G1_FINE_BASELINE', 'fine_previous.py')
        spec = importlib.util.spec_from_file_location('frozen_fine_baseline', path)
        old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)
        pairs = []
        # Drive the real start/tick, including coarse walking, obstacle, pauses,
        # body-behind blind walk, goal crossing, final yaw and cancellation.
        for pre in (None, (.4, 0.)):
            nodes = []
            for cls in (old.PosNav, PosNav):
                n = cls(lambda p, g: g, lambda *_: True,
                        lambda g: dict(pre=pre, reason='same fixture'))
                n.pose = (0., 0., 0.); n.pose_time = n.map_time = 0.
                n.start((1., 0., .7), 0.)
                nodes.append(n)
            trace = []
            for i in range(501):
                t = i*.02
                # Independent fixed input sequence, never generated by either controller.
                x = min(1.02, t*.16); yaw = 0. if t < 8 else .75
                row = []
                for n in nodes:
                    n.pose = (x, .02*math.sin(t), yaw)
                    n.pose_time = t if not 2. < t < 2.5 else 2.
                    n.map_time = t
                    cmd = n.tick(t)
                    row.append((cmd, n.state, n.active, n.reason, n.pre))
                self.assertEqual(row[0], row[1])
                trace.append(row[1])
            pairs.append(len(trace))
        self.assertEqual(pairs, [501, 501])

    def test_yaw_cap_only_affects_heading_align_and_completes_in_model(self):
        n = self.controller(fine_align_wz=.2)
        n.final_goal = (0., 0., 1.)
        self.assertEqual(self.tick(n, .1, (-.4, 0., -1.))[2], 1.)
        self.tick(n, .2, (.01, 0., 0.)); self.tick(n, .7)
        self.assertEqual(n.state, 'heading_align')
        t, yaw = .72, 0.
        while n.active and t < 8.:
            cmd = self.tick(n, t, (.01, 0., yaw))
            self.assertLessEqual(abs(cmd[2]), .2)
            yaw += cmd[2]*.02
            t += .02
        self.assertFalse(n.active)
        self.assertEqual(n.reason, n.completion_reason)
        self.assertLess(abs(yaw-1.), .101)
        self.assertLess(t, 7.)  # ideal kinematic model only, no motor/dead-zone claim

    def settling(self):
        n = self.controller(fine_stop_layered=True, position_settle_fresh=True)
        self.tick(n, .1, (-.07, 0., 0.))
        return n

    def test_stationary_independent_samples_confirm_after_nominal_wait(self):
        n = self.settling()
        for t in (.2, .3, .4, .5):
            self.tick(n, t, (-.06+(.002 if int(t*10)%2 else 0), 0., 0.))
            self.assertEqual(n.state, 'fine_position_settle')
        self.tick(n, .6)
        self.assertEqual(n.state, 'heading_align')
        self.assertTrue(self.events(n, 'fine_position_settled')[-1]['fresh_confirmation'])

    def test_duplicates_cannot_confirm_even_with_fresh_callback_times(self):
        n = self.settling()
        for i in range(1, 31):
            self.tick(n, .1+i*.1, source=123)
        self.assertEqual(n.state, 'fine_position_settle')
        self.assertEqual(n.position_samples.settle_fresh_s, 0.)

    def test_dropout_freezes_budget_and_recovery_does_not_bridge_gap(self):
        n = self.settling()
        self.tick(n, .2); self.tick(n, .3)
        before = n.position_samples.settle_fresh_s
        self.assertEqual(self.tick(n, 3., sample_time=.3), (0., 0., 0.))
        self.tick(n, 3.1)
        self.assertEqual(n.position_samples.settle_fresh_s, before)
        self.assertEqual(n.state, 'fine_position_settle')
        for t in (3.2, 3.3, 3.4):
            self.tick(n, t)
        self.assertEqual(n.state, 'heading_align')

    def test_sub_pose_timeout_gap_resets_stop_evidence(self):
        n = self.settling()
        for t in (.2, .3, .4, .68):
            self.tick(n, t)
        self.assertEqual(n.state, 'fine_position_settle')
        for t in (.78, .88, .98):
            self.tick(n, t)
        self.assertEqual(n.state, 'heading_align')

    def test_continuous_motion_spends_effective_timeout_and_aborts(self):
        n = self.settling()
        for i in range(2, 24):
            command = self.tick(n, i*.1, (-.07+(.04 if i%2 else 0.), 0., 0.))
            if not n.active:
                self.assertEqual(command, (0., 0., 0.)); break
        self.assertFalse(n.active)
        self.assertEqual(n.reason, 'fine_position_settle_timeout')
        self.assertFalse(self.events(n, 'fine_position_settled'))

    def test_slow_drift_window_is_not_mistaken_for_rest(self):
        n = self.settling()
        for i in range(2, 11):
            self.tick(n, i*.1, (-.07+i*.009, 0., 0.))
        self.assertEqual(n.state, 'fine_position_settle')

    def test_same_source_relocalized_pose_cannot_complete_old_stable_window(self):
        n = self.settling()
        for t in (.2, .3, .4, .5):
            self.tick(n, t)
        self.tick(n, .6, (-.02, 0., 0.), source=int(1.5e9), sample_time=.5)
        self.assertEqual(n.state, 'fine_position_settle')
        self.assertFalse(n.position_samples.samples)

    def test_twenty_hz_samples_use_hold_duration_not_just_sample_count(self):
        n = self.settling()
        for i in range(4, 13):
            self.tick(n, i*.05)
        self.assertEqual(n.state, 'heading_align')

    def test_delayed_continuous_fresh_source_can_confirm(self):
        n = self.settling()
        for i in range(2, 9):
            source_time = i*.1
            self.tick(n, source_time+.20, sample_time=source_time)
            if n.state == 'heading_align':
                break
        self.assertEqual(n.state, 'heading_align')

    def test_all_stage_events_keep_same_source_stamp_and_four_error_axes(self):
        n = self.controller(fine_stop_layered=True)
        for t, pose in ((.1, (-.07, .02, 0.)), (.6, (-.04, .03, 0.)),
                        (.7, (-.04, .03, 0.)), (1.5, (-.03, .04, .05))):
            self.tick(n, t, pose)
        for name in ('fine_position_stop', 'fine_position_settled', 'fine_yaw_stop',
                     'fine_yaw_settled', 'fine_approach_completed'):
            data = self.events(n, name)[-1]
            self.assertEqual(data['source_stamp_ns'], int((1+data['control_time_monotonic'])*1e9))
            self.assertEqual(data['pose_source'], 'latest_fastlio_control_pose')
            self.assertAlmostEqual(data['distance_m'], math.hypot(*data['pose'][:2]))
            self.assertAlmostEqual(data['projected_remaining_m'], -data['pose'][0])
            self.assertAlmostEqual(data['goal_yaw_remaining_m'], -data['pose'][0])
            self.assertAlmostEqual(data['lateral_m'], -data['pose'][1])

    def test_cli_roundtrip_and_invalid_values(self):
        parser = argparse.ArgumentParser(); add_navigation_options(parser)
        self.assertEqual(forward_navigation_options(parser.parse_args([])), [])
        options = ['--fine-stop-layered', '--fine-goal-behind', '--fine-pre-fallback',
                   '--position-settle-fresh', '--fine-align-wz', '.2', '--fine-stop-radius', '.09',
                   '--position-settle-samples', '5']
        parsed = parser.parse_args(options)
        forwarded = parser.parse_args(forward_navigation_options(parsed))
        self.assertEqual(options_from_args(parsed), options_from_args(forwarded))
        self.assertTrue(options_from_args(parsed)['fine_stop_layered'])
        for bad in (dict(fine_stop_radius=float('nan')), dict(fine_stop_radius=.2),
                dict(fine_stop_projected=.1), dict(fine_align_wz=0), dict(fine_align_wz=1.1),
                dict(position_settle_samples=1), dict(position_settle_samples=4.5),
                dict(fine_goal_behind=1)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                checked_options(**bad)


if __name__ == '__main__':
    unittest.main()
