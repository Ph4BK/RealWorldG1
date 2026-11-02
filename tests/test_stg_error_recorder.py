"""Exercise measurements through the real state machine, without ROS or motion."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import test_g1_navigation_core as navigation_tests
from stg_error_recorder import STGErrorRecorder, STGErrorLog


class RecordedNavigationTest(navigation_tests.NavigationTest):
    # Also rerun all existing motion-transition tests with recording enabled.
    def setUp(self):
        super().setUp()
        self.recorder = STGErrorRecorder()
        self.n.recorder = self.recorder

    def complete_walk(self):
        self.tick(1., (.95, .03, .02))
        self.tick(1.5, (1.03, .04, .05))
        return self.recorder.completed[-1]

    def test_small_turn_records_only_final_settle_for_one_stg(self):
        self.n.pose = (0., 0., -.1)
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        self.tick(.2, (0., 0., .31))
        self.tick(.96, (.01, .02, .4))
        self.assertIsNone(self.recorder.current['turn_settle_pose'])
        self.tick(.98)
        self.tick(1., (.01, .02, .1))
        self.assertIsNone(self.recorder.current['turn_settle_pose'])
        final_turn_pose = (.02, .01, .02)
        self.tick(1.75, final_turn_pose)
        self.tick(2., (.9, .03, .02))
        self.tick(2.5, (1., .04, .02))
        self.assertEqual(len(self.recorder.completed), 1)
        row = self.recorder.completed[0]
        self.assertEqual(row['initial_pose'], (0., 0., -.1))
        self.assertEqual(row['stg'], (1., 0.))
        self.assertEqual(row['turn_settle_pose'], final_turn_pose)
        self.assertEqual(row['anomalies'], [])
        self.assertAlmostEqual(row['error_m'], .04)

    def test_exact_boundary_poses_and_l2_not_projection_or_yaw(self):
        self.n.pose = (0., 0., -1.)
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        self.tick(.02, (0., 0., -.1))
        self.tick(.76, (.01, 0., .1))
        self.assertIsNone(self.recorder.current['turn_settle_pose'])
        self.tick(.77, (0., 0., .2))
        self.tick(1., (.95, .03, .3))
        self.tick(1.49, (1., .01, .4))
        self.assertEqual(len(self.recorder.completed), 0)
        self.tick(1.5, (1.03, .04, .5))
        row = self.recorder.completed[0]
        self.assertEqual(row, dict(initial_pose=(0., 0., -1.), stg=(1., 0.),
                                  turn_settle_pose=(0., 0., .2),
                                  walk_settle_pose=(1.03, .04, .5),
                                  anomalies=[], error_m=row['error_m']))
        self.assertAlmostEqual(row['error_m'], .05)
        self.assertTrue(self.n.active)
        self.assertEqual(self.n.state, 'plan')
        self.target = (2., 0.)
        self.tick(1.52)
        self.assertEqual(self.recorder.current['stg'], (2., 0.))
        self.assertEqual(self.recorder.current['initial_pose'], (1.03, .04, .5))
        self.assertEqual(row['stg'], (1., 0.))

    def test_skipped_turn_records_latest_pre_walk_pose_without_delay(self):
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        self.assertIsNone(self.recorder.current['turn_settle_pose'])
        pose = (.01, .01, .02)
        self.assertEqual(self.tick(.02, pose), (.5, 0.))
        self.assertEqual(self.recorder.current['initial_pose'], (0., 0., 0.))
        self.assertEqual(self.recorder.current['turn_settle_pose'], pose)
        self.assertEqual(self.recorder.current['anomalies'], [])

    def test_resumed_pose_pause_still_excludes_error(self):
        self.start_walk()
        self.n.map_time = 1.1
        self.tick(1.1, fresh=False)
        self.tick(1.2, fresh=False)
        self.tick(1.3, (.95, 0., 0.))
        self.tick(1.8, (1., 0., 0.))
        row = self.recorder.completed[0]
        self.assertEqual(row['anomalies'], ['Waiting for pose'])
        self.assertNotIn('error_m', row)
        self.assertIsNotNone(row['walk_settle_pose'])

    def test_map_pause_at_settle_deadline_samples_only_when_resumed(self):
        self.start_walk()
        self.tick(1., (.95, 0., 0.))
        self.n.pose_time = 2.
        self.tick(2., fresh=False)
        self.assertEqual(len(self.recorder.completed), 0)
        self.tick(2.1, (1.02, .01, .1))
        row = self.recorder.completed[0]
        self.assertEqual(row['walk_settle_pose'], (1.02, .01, .1))
        self.assertEqual(row['anomalies'], ['Waiting for map update'])
        self.assertNotIn('error_m', row)

    def test_obstacle_recovery_keeps_anomaly(self):
        self.start_walk()
        self.blocked = True
        self.tick(.8)
        self.blocked = False
        self.tick(2.8)
        self.tick(3., (.95, 0., 0.))
        self.tick(3.5, (1., 0., 0.))
        row = self.recorder.completed[0]
        self.assertEqual(row['anomalies'], ['Obstacle wait'])
        self.assertNotIn('error_m', row)

    def test_replan_saves_incomplete_segment_and_new_segment_is_clean(self):
        self.start_walk()
        self.blocked = True
        self.tick(1.)
        self.tick(3.)
        row = self.recorder.completed[0]
        self.assertIsNone(row['walk_settle_pose'])
        self.assertEqual(row['anomalies'], ['Obstacle wait', 'Obstacle persisted; replanning'])
        self.assertNotIn('error_m', row)
        self.blocked = False
        self.target = (2., 0.)
        self.tick(3.02)
        self.assertEqual(self.recorder.current['anomalies'], [])

    def test_blocked_selection_is_saved_without_settle_poses(self):
        self.blocked = True
        self.n.start((5., 0.), 0.)
        self.tick(0.)
        row = self.recorder.completed[0]
        self.assertEqual(row['stg'], (1., 0.))
        self.assertIsNone(row['turn_settle_pose'])
        self.assertIsNone(row['walk_settle_pose'])
        self.assertNotIn('error_m', row)
        self.tick(.5)
        self.assertEqual(len(self.recorder.completed), 1)

    def test_final_goal_early_stop_remains_excluded_after_drift_outside(self):
        self.start_walk()
        self.n.goal = (1.3, 0.)
        self.tick(1., (.81, 0., 0.))
        self.tick(1.5, (.79, 0., 0.))
        row = self.recorder.completed[0]
        self.assertEqual(row['anomalies'], ['Final goal radius triggered STG stop'])
        self.assertNotIn('error_m', row)

    def test_final_goal_skip_walk_records_both_settle_poses(self):
        self.n.start((1., 0.), 0.)
        self.tick(0.)
        self.tick(.02, (.6, 0., .1))
        self.tick(.52, (.61, 0., .2))
        row = self.recorder.completed[0]
        self.assertEqual(row['turn_settle_pose'], (.6, 0., .1))
        self.assertEqual(row['walk_settle_pose'], (.61, 0., .2))
        self.assertNotIn('error_m', row)
        self.assertFalse(self.n.active)
        self.assertEqual(len(self.recorder.completed), 1)

    def test_goal_entered_only_during_settle_is_not_early_stop(self):
        self.start_walk()
        self.n.goal = (1.5, 0.)
        row = self.complete_walk()
        self.assertFalse(self.n.active)
        self.assertEqual(row['anomalies'], [])
        self.assertIn('error_m', row)

    def test_cancel_or_error_preserves_partial_data_once(self):
        for reason in ['Stopped by user', 'Goal redrawn', 'Navigation stopped: test failure']:
            with self.subTest(reason=reason):
                self.setUp()
                self.start_walk()
                self.n.stop(reason)
                self.n.stop('Navigation process stopped')
                self.assertEqual(len(self.recorder.completed), 1)
                row = self.recorder.completed[0]
                self.assertEqual(row['anomalies'], [reason])
                self.assertIsNotNone(row['turn_settle_pose'])
                self.assertIsNone(row['walk_settle_pose'])
                self.assertNotIn('error_m', row)

    def test_no_stg_selected_produces_no_experiment(self):
        self.n.start((.4, 0.), 0.)
        self.tick(0.)
        self.n.stop()
        self.assertIsNone(self.recorder.current)
        self.assertEqual(len(self.recorder.completed), 0)

    def test_jsonl_flush_is_incremental_and_sessions_do_not_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            first = STGErrorLog(directory)
            second = STGErrorLog(directory)
            try:
                self.assertNotEqual(first.path, second.path)
                self.start_walk()
                self.complete_walk()
                first.flush(self.recorder)
                self.assertEqual(len(self.recorder.completed), 0)
                rows = first.path.read_text().splitlines()
                self.assertEqual(len(rows), 1)
                self.assertAlmostEqual(json.loads(rows[0])['error_m'], .05)
                self.tick(1.52)
                self.n.stop('Stopped by user')
                first.flush(self.recorder)
                first.flush(self.recorder)
                rows = first.path.read_text().splitlines()
                self.assertEqual(len(rows), 2)
                self.assertNotIn('error_m', json.loads(rows[1]))
                self.assertEqual(second.path.read_text(), '')
            finally:
                first.close()
                second.close()


if __name__ == '__main__':
    unittest.main()
