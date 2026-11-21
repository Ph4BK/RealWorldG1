"""Offline real-package and real-projector regressions; no ROS/network/robot."""
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from build_g1_619_map import (build_package, sha256, validate_package,
                              verify_integrity, consumer_file_loader_check)
from build_static_bev import project_bev, read_pcd_xyz, decompress_lzf


class ProjectionTest(unittest.TestCase):
    def config(self):
        return dict(height_range_m_inclusive=[.05, 1.3], resolution_m=.5,
                    origin_xy_m=[0., 0.], shape_rows_cols=[2, 2], min_points_per_cell=2,
                    projection_method='runtime_floor_half_open_v1', expected_height_points_outside_grid=1)

    def test_closed_height_half_open_xy_and_no_inflation(self):
        points = np.array([[.1, .1, .05], [.1, .1, 1.3], [.6, .6, .4],
                           [.1, .1, .049], [.1, .1, 1.301], [1., .1, .5], [np.nan, 0, .5]])
        occ, counts, report = project_bev(points, self.config())
        np.testing.assert_array_equal(occ, [[True, False], [False, False]])
        np.testing.assert_array_equal(counts, [[2, 0], [0, 1]])
        self.assertEqual(report, dict(height_selected_points=4, points_in_grid=3, points_outside_grid=1))

    def test_unapproved_outside_points_rejected(self):
        config = self.config(); config['expected_height_points_outside_grid'] = 0
        with self.assertRaisesRegex(ValueError, 'outside'):
            project_bev([[1., 0., .5]], config)

    def test_bad_projection_config_rejected(self):
        for key, value in [('resolution_m', 0), ('resolution_m', math.nan),
                           ('min_points_per_cell', 0), ('origin_xy_m', [math.nan, 0]),
                           ('projection_method', 'guess')]:
            with self.subTest(key=key, value=value):
                config = self.config(); config[key] = value
                with self.assertRaises(ValueError):
                    project_bev([[0., 0., .5]], config)

    def test_pcd_ascii_reordered_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.pcd'
            path.write_text('FIELDS intensity z x y\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
                            'WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n9 3 1 2\n8 6 4 5\n')
            np.testing.assert_array_equal(read_pcd_xyz(path), [[1, 2, 3], [4, 5, 6]])

    def test_pcd_binary_truncated_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.pcd'
            header = b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n'
            path.write_bytes(header + struct.pack('<ff', 1., 2.))
            with self.assertRaisesRegex(ValueError, 'payload'):
                read_pcd_xyz(path)

    def test_unknown_pcd_encoding_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.pcd'
            path.write_bytes(b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nWIDTH 1\nHEIGHT 1\n'
                             b'POINTS 1\nDATA unknown\nnot raw floats')
            with self.assertRaisesRegex(ValueError, 'encoding'):
                read_pcd_xyz(path)

    def test_lzf_overlapping_reference_and_invalid_streams(self):
        self.assertEqual(decompress_lzf(bytes([0, ord('a'), 128, 0]), 7), b'aaaaaaa')
        for value, length in [(b'\x02a', 3), (b'\x20\x00', 3), (b'\xe0', 3), (b'\x00a', 2)]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                decompress_lzf(value, length)

    def test_pcd_compressed_structure_of_arrays(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.pcd'
            payload = struct.pack('<ffffff', 1., 4., 2., 5., 3., 6.)
            encoded = bytes([len(payload) - 1]) + payload  # One LZF literal run.
            header = (b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nWIDTH 2\nHEIGHT 1\n'
                      b'POINTS 2\nDATA binary_compressed\n')
            path.write_bytes(header + struct.pack('<II', len(encoded), len(payload)) + encoded)
            np.testing.assert_array_equal(read_pcd_xyz(path), [[1, 2, 3], [4, 5, 6]])


class Real619PackageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='g1_619_map_tests_')
        cls.directory = Path(cls.temp.name)
        # The reviewed map package carries the original inputs byte-for-byte;
        # a clean clone must not require the separate Go2 workspace.
        package = ROOT / 'maps/619_go2base'
        cls.inputs = (package / 'map.pcd',
                      package / 'provenance/go2_grid_reference.npz',
                      package / 'provenance/source_map_README.md')
        cls.hashes = [sha256(p) for p in cls.inputs]
        cls.candidate = cls.directory / 'candidate'
        cls.report = build_package(*cls.inputs, cls.candidate)
        cls.cleaned = cls.directory / 'cleaned'
        cls.cleaned_report = build_package(*cls.inputs, cls.cleaned, bev_source='go2-grid',
            imu_height_m=1.24, imu_height_source='TEST user-provided dome approximation')

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_real_619_projection_and_runtime_agree(self):
        self.assertEqual(self.report['source_points'], 446561)
        self.assertEqual(self.report['height_selected_points'], 168331)
        self.assertEqual(self.report['points_outside_grid'], 2)
        self.assertEqual(self.report['occupied_cells'], 23048)
        self.assertEqual(self.report['shape_rows_cols'], [400, 474])
        self.assertEqual(self.report['production_rolling_bev_occupied_iou'], 1.)
        self.assertEqual(self.report['pcd_projection_occupied_iou'], 1.)

    def test_cleaned_grid_bytes_and_icp_map_are_inherited_without_reprojection(self):
        self.assertEqual(sha256(self.cleaned/'bev_occupancy.npz'), self.hashes[1])
        self.assertEqual(sha256(self.cleaned/'map.pcd'), self.hashes[0])
        report = self.cleaned_report
        self.assertEqual(report['occupied_cells'], 107054)
        self.assertEqual(report['fixed_no_go_cells'], 87894)
        self.assertEqual(report['go2_reference_iou'], 1.)
        self.assertIsNone(report['pcd_projection_occupied_iou'])
        self.assertTrue(report['runtime_initialization_ready'])

    def test_cleaned_policy_metadata_and_real_loader_match(self):
        config = json.loads((self.cleaned/'bev_config.json').read_text())
        metadata = json.loads((self.cleaned/'metadata.yaml').read_text())
        live = json.loads((self.cleaned/'runtime_live_bev.yaml').read_text())
        self.assertEqual(config['parameter_source'], 'Go2 cleaned grid (unexpanded)')
        self.assertEqual(metadata['initial_imu_xyz_map'][2], 1.24)
        self.assertIn('dome', metadata['initial_imu_height']['source'])
        self.assertEqual(metadata['planning_inflation']['value_m'], .25)
        self.assertEqual(live['fmm_inflation_cells'], 5)
        self.assertIs(live['allow_initial_clear'], False)
        loaded, _ = consumer_file_loader_check(self.cleaned,self.cleaned/'runtime_live_bev.yaml')
        self.assertEqual(loaded.no_go_cells, 87894)
        self.assertTrue(loaded.task[loaded.no_go].all())
        verify_integrity(self.cleaned)

    def test_real_cleaned_boundary_fusion_preserves_policy_but_not_all_static(self):
        loaded, _ = consumer_file_loader_check(self.cleaned,self.cleaned/'runtime_live_bev.yaml')
        with np.load(self.inputs[1],allow_pickle=False) as source:
            right=float(source['room_bounds'][1])
        pose=[right-.25,-2.,0.]
        ring=loaded.region(*pose[:2])
        ordinary=ring & loaded.static & ~loaded.no_go
        self.assertTrue(ordinary.any())
        for i in range(loaded.window):
            updated=loaded.add(i*.1,[[0.,0.,0.]],pose)
        self.assertTrue(updated)
        self.assertTrue(loaded.task[loaded.no_go].all())
        self.assertEqual(int(loaded.task[loaded.no_go].sum()),87894)
        self.assertFalse(loaded.task[ordinary].any())
        self.assertGreater(loaded.last_no_go_protected_cells,0)

    def test_cleaned_package_validators_and_standard_rebuilder(self):
        report = validate_package(self.cleaned)
        self.assertTrue(report['runtime_initialization_ready'])
        for suffix in ([], ['--require-runtime-ready']):
            result = subprocess.run([sys.executable,'-B',str(ROOT/'tools/build_g1_619_map.py'),
                '--validate-only',str(self.cleaned)]+suffix,capture_output=True,text=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stderr)
        destination = self.directory/'cleaned_standard_rebuild'
        result = subprocess.run([sys.executable,'-B',str(ROOT/'tools/build_static_bev.py'),
            '--map-dir',str(self.cleaned),'--output-dir',str(destination)],
            capture_output=True,text=True,timeout=30)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(sha256(destination/'bev_occupancy.npz'),self.hashes[1])

    def test_declared_policy_cannot_silently_disappear_or_change(self):
        import shutil
        for name in ('missing','changed','geometry'):
            with self.subTest(name=name):
                package = self.directory/('policy_'+name)
                shutil.copytree(self.cleaned,package)
                if name == 'missing':
                    (package/'no_go.npz').unlink()
                elif name == 'changed':
                    (package/'no_go.npz').write_bytes(b'bad policy')
                else:
                    config = json.loads((package/'bev_config.json').read_text())
                    config['fixed_no_go']['room_bounds_xmin_xmax_ymin_ymax_m'][0] += 1.
                    (package/'bev_config.json').write_text(json.dumps(config))
                with self.assertRaises((ValueError,FileNotFoundError)):
                    validate_package(package)

    def test_input_and_icp_pcd_bytes_unchanged(self):
        self.assertEqual([sha256(p) for p in self.inputs], self.hashes)
        self.assertEqual(sha256(self.candidate / 'map.pcd'), self.hashes[0])

    def test_no_fake_imu_height_or_body_extrinsic(self):
        self.assertFalse(self.report['runtime_initialization_ready'])
        self.assertTrue(self.report['consumer_file_loader']['imu_height_missing_rejected'])
        metadata = json.loads((self.candidate / 'metadata.yaml').read_text())
        self.assertEqual(metadata['initial_imu_xyz_map'], [None, None, None])
        self.assertNotIn('T_base_from_imu', metadata)
        np.testing.assert_array_equal(np.loadtxt(self.candidate / 'T_map_from_scene.txt'), np.eye(4))

    def test_provided_height_with_source_allows_consumer_loading(self):
        # Synthetic plumbing value only; never written into the deliverable map.
        package = self.directory / 'synthetic_height_test'
        report = build_package(*self.inputs, package, imu_height_m=1.234,
                               imu_height_source='SYNTHETIC TEST ONLY, not a 619 measurement')
        self.assertTrue(report['runtime_initialization_ready'])
        self.assertTrue(report['consumer_file_loader']['initialization_height_usable'])
        self.assertFalse(report['motion_or_localization_accuracy_validated'])
        verify_integrity(package)

    def test_invalid_height_or_missing_source_rejected_before_writing(self):
        for index, height in enumerate((0., -1., math.nan, math.inf, True, 1.2)):
            package = self.directory / f'bad_height_{index}'
            with self.subTest(height=height), self.assertRaises(ValueError):
                build_package(*self.inputs, package, imu_height_m=height)
            self.assertFalse(package.exists())

    def test_existing_output_not_overwritten(self):
        digest = sha256(self.candidate / 'metadata.yaml')
        with self.assertRaises(FileExistsError):
            build_package(*self.inputs, self.candidate)
        self.assertEqual(sha256(self.candidate / 'metadata.yaml'), digest)

    def test_manifest_and_checksums_complete(self):
        self.assertGreater(verify_integrity(self.candidate), 8)

    def test_repeated_build_is_byte_reproducible(self):
        other = self.directory / 'repeat'
        build_package(*self.inputs, other)
        first = {p.relative_to(self.candidate).as_posix(): sha256(p)
                 for p in self.candidate.rglob('*') if p.is_file()}
        second = {p.relative_to(other).as_posix(): sha256(p)
                  for p in other.rglob('*') if p.is_file()}
        self.assertEqual(first, second)

    def test_standard_rebuild_tool_cli_matches_package(self):
        destination = self.directory / 'standard_rebuild'
        result = subprocess.run([sys.executable, '-B', str(ROOT / 'tools/build_static_bev.py'),
                                 '--map-dir', str(self.candidate), '--output-dir', str(destination)],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        with np.load(destination / 'bev_occupancy.npz') as actual, np.load(self.candidate / 'bev_occupancy.npz') as expected:
            for key in expected.files:
                np.testing.assert_array_equal(actual[key], expected[key])

    def test_runtime_readiness_cli_rejects_missing_height(self):
        result = subprocess.run([sys.executable, '-B', str(ROOT / 'tools/build_g1_619_map.py'),
                                 '--validate-only', str(self.candidate), '--require-runtime-ready'],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 2)
        self.assertIn('IMU_HEIGHT_REQUIRED', result.stderr)

    def test_real_consumer_assert_rejects_origin_mismatch(self):
        package = self.directory / 'bad_origin'; package.mkdir()
        for name in ('bev_occupancy.npz', 'bev_config.json', 'metadata.yaml', 'runtime_live_bev.yaml'):
            (package / name).write_bytes((self.candidate / name).read_bytes())
        config = json.loads((package / 'bev_config.json').read_text())
        config['origin_xy_m'][0] += .5
        (package / 'bev_config.json').write_text(json.dumps(config))
        with self.assertRaises(AssertionError):
            consumer_file_loader_check(package, package / 'runtime_live_bev.yaml')

    def test_hash_detects_modified_artifact(self):
        package = self.directory / 'tamper'; package.mkdir()
        (package / 'README.md').write_text('tampered')
        (package / 'package_manifest.json').write_text(json.dumps(dict(files=[dict(
            path='README.md', bytes=8, sha256=hashlib.sha256(b'original').hexdigest())])))
        with self.assertRaisesRegex(ValueError, 'Manifest mismatch'):
            verify_integrity(package)

    def test_803_legacy_projection_is_unchanged(self):
        original = ROOT / 'maps/full_lab_v1'
        config = json.loads((original / 'bev_config.json').read_text())
        rebuilt, _, _ = project_bev(read_pcd_xyz(original / 'map.pcd'), config)
        with np.load(original / 'bev_occupancy.npz') as data:
            np.testing.assert_array_equal(rebuilt, data['occ'])


if __name__ == '__main__':
    unittest.main()
