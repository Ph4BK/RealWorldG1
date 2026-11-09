"""Evidence lifecycle and failure tests without ROS or a robot."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from experiment_recorder import (AsyncWriter, ExperimentSession, ExperimentSTGRecorder,
                                 TrialRecorder, sha256)
from g1_navigation_core import Navigation


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


class ExperimentTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.session = ExperimentSession(ROOT, ROOT/'maps/full_lab_v1',
            ROOT/'fastlio/config/live_bev.yaml', self.tmp.name)
        self.recorder = TrialRecorder(self.session.path, {'control': {'goal_tol': .5}})

    def tearDown(self):
        self.recorder.close()
        self.session.close()
        self.tmp.cleanup()

    def begin(self, goal=(3., 0., 1.)):
        self.recorder.begin(goal, (0., 0., 0.), 123, .5)

    def finish(self, reason='Stopped by user', pose=(1., 0., 0.)):
        self.recorder.finish(reason, pose, .01, '', None)

    def test_complete_bundle_is_self_contained_and_checksums_verify(self):
        self.session.initialization({'x': 0., 'T_map_from_odom': np.eye(4)}, {'path_map': 'map.pcd'})
        self.begin()
        self.recorder.pose((.5, 0., 0.), 124, 'walk')
        self.recorder.pose((2.6, 0., .2), 125, 'walk_settle')
        self.finish('Final approach completed', (2.6, 0., .2))
        self.recorder.close()
        self.session.close()
        base = self.session.path
        info = json.loads((base/'session.json').read_text())
        self.assertTrue(info['recording_complete'])
        self.assertEqual((base/'assets/map/map.pcd').read_bytes(), (ROOT/'maps/full_lab_v1/map.pcd').read_bytes())
        self.assertTrue((base/'assets/config/localization_0001.json').exists())
        task = json.loads((base/'trials/trial_0001/task.json').read_text())
        result = json.loads((base/'trials/trial_0001/result.json').read_text())
        self.assertEqual(task['goal'], [3., 0., 1.])
        self.assertTrue(task['final_yaw_control'])
        self.assertFalse(task['position_tolerance_checked'])
        self.assertTrue(result['success'])
        self.assertAlmostEqual(result['internal_error_m'], .4)
        with (base/'trials/trial_0001/trajectory.csv').open() as stream:
            trajectory = list(csv.DictReader(stream))
        self.assertEqual(len(trajectory), 2)
        self.assertEqual(trajectory[-1]['source_stamp_ns'], '125')
        for line in (base/'checksums.sha256').read_text().splitlines():
            digest, relative = line.split('  ', 1)
            self.assertEqual(sha256(base/relative), digest, relative)

    def test_two_tasks_share_assets_but_have_distinct_goals_and_results(self):
        self.begin()
        self.finish('Goal redrawn; press Start Navigation')
        self.begin((5., 1., -1.))
        self.finish()
        self.recorder.close()
        base = self.session.path/'trials'
        first = json.loads((base/'trial_0001/task.json').read_text())
        second = json.loads((base/'trial_0002/task.json').read_text())
        self.assertNotEqual(first['trial_id'], second['trial_id'])
        self.assertEqual(second['goal'], [5., 1., -1.])
        events = rows(self.session.path/'events.jsonl')
        self.assertEqual(len([e for e in events if e['event']=='trial_finished']), 2)

    def test_map_snapshots_are_immutable_and_deduplicated_per_trial(self):
        self.begin()
        grid = np.zeros((5, 6), dtype=np.int8)
        meta = dict(source_stamp_ns=123, frame_id='map', resolution_m=.05,
                    width=6, height=5, origin_position=[-1., -2., 0.], origin_quaternion=[0., 0., 0., 1.])
        ref = self.recorder.snapshot(grid, meta)
        self.assertEqual(ref, self.recorder.snapshot(grid, meta))
        grid[:] = 100
        self.finish()
        self.recorder.close()
        with np.load(self.session.path/ref) as data:
            np.testing.assert_array_equal(data['data'], np.zeros((5, 6)))
        self.assertEqual(len(list((self.session.path/'trials/trial_0001/maps').glob('*.npz'))), 1)

    def test_planning_failure_is_not_fabricated_as_a_segment(self):
        self.begin()
        self.recorder.planning((0, 0), (3, 0), None, 'retry', 'unreachable')
        self.recorder.event('navigation_status', status={'active': True, 'state': 'plan_wait'})
        self.finish()
        self.recorder.close()
        trial = self.session.path/'trials/trial_0001'
        self.assertEqual(rows(trial/'planning.jsonl')[0]['outcome'], 'retry')
        self.assertEqual((trial/'stg_segments.jsonl').read_text(), '')
        result = json.loads((trial/'result.json').read_text())
        self.assertEqual(result['planning_attempts'], 1)
        self.assertEqual(result['stg_count'], 0)
        self.assertEqual(result['outcome'], 'cancelled')

    def test_real_state_machine_records_endpoint_times_and_legacy_shape(self):
        self.begin()
        measure = ExperimentSTGRecorder(self.recorder)
        n = Navigation(lambda *_: (1., 0.), lambda *_: True, measure)
        n.pose = (0., 0., 0.)
        n.pose_time = n.map_time = 0.
        n.start((3., 0.), 0.)
        for t, pose in [(0., (0., 0., 0.)), (.02, (0., 0., 0.)),
                        (.04, (.9, 0., 0.)), (.55, (1.03, .04, .1))]:
            n.pose_time = n.map_time = t
            n.pose = pose
            n.tick(t)
        self.assertEqual(len(measure.completed), 1)
        legacy = measure.completed[0]
        self.assertNotIn('phase_times', legacy)
        self.finish()
        self.recorder.close()
        segment = rows(self.session.path/'trials/trial_0001/stg_segments.jsonl')[0]
        self.assertEqual(segment['stg_id'], 'stg_0001')
        self.assertEqual(set(segment['phase_times']), {'begin', 'turn_settled', 'walk_settled'})
        self.assertAlmostEqual(segment['error_m'], .05)

    def test_interrupted_trial_remains_unknown_and_incomplete(self):
        self.begin()
        self.recorder.close()  # No terminal event was observed.
        self.session.close()
        result = json.loads((self.session.path/'trials/trial_0001/result.json').read_text())
        self.assertIsNone(result['success'])
        self.assertFalse(result['recording_complete'])
        self.assertFalse(json.loads((self.session.path/'session.json').read_text())['recording_complete'])

    def test_disk_failure_is_reported_and_invalidates_completed_result(self):
        self.begin()
        self.finish()
        self.recorder.writer.queue.join()
        with patch.object(self.recorder.writer, '_write', side_effect=OSError('disk full')):
            self.recorder.event('force_write_failure')
            self.recorder.writer.queue.join()
        self.assertIn('disk full', self.recorder.writer.error)
        self.recorder.close()
        self.session.close()
        result = json.loads((self.session.path/'trials/trial_0001/result.json').read_text())
        self.assertFalse(result['recording_complete'])
        self.assertFalse(json.loads((self.session.path/'session.json').read_text())['recording_complete'])

    def test_queue_overflow_does_not_block_producer(self):
        writer = AsyncWriter(self.session.path, capacity=1)
        entered, release = threading.Event(), threading.Event()
        original = writer._write
        def slow(*args):
            entered.set()
            release.wait(2)
            original(*args)
        try:
            with patch.object(writer, '_write', side_effect=slow):
                writer.line('overflow_test.jsonl', {'n': 1})
                self.assertTrue(entered.wait(2))
                writer.line('overflow_test.jsonl', {'n': 2})
                self.assertFalse(writer.line('overflow_test.jsonl', {'n': 3}))
                self.assertIn('queue is full', writer.error)
                self.assertEqual(writer.dropped, 1)
                release.set()
                self.assertFalse(writer.close())
        finally:
            release.set()
            writer.close()


if __name__ == '__main__':
    unittest.main()
