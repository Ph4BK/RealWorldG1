"""Exercise the actual packager and check local Python import closure in its TAR."""
import ast
import contextlib
import io
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import package_g1_posnav


class PackageTest(unittest.TestCase):
    def test_archive_contains_local_runtime_imports_and_regression_sources(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory)/'fixture'
            fixture.mkdir()
            (fixture/'entry').mkdir()
            for name in ('docs', 'posnav', 'tools', 'tests', 'fastlio', 'maps'):
                (fixture/name).symlink_to(ROOT/name, target_is_directory=True)
            source = fixture/'.cache/fastlio_ws/src/FAST_LIO_LOCALIZATION_HUMANOID/open3d_loc'
            (source/'src/open3d_registration').mkdir(parents=True)
            (source/'CMakeLists.txt').write_text('        c++\n        c++abi\n', encoding='utf-8')
            (source/'src/global_localization.cpp').write_text(
                '// packaging fixture: pub_live_registration_\n', encoding='utf-8')
            (source/'src/open3d_registration/open3d_registration.cpp').write_text(
                '#include <open3d/utility/Random.h>\n', encoding='utf-8')
            output = Path(directory)/'bundle.tgz'
            with patch.object(package_g1_posnav, '__file__', str(fixture/'entry/package.py')), \
                    patch.object(sys, 'argv', ['package_g1_posnav.py', str(output)]), \
                    contextlib.redirect_stdout(io.StringIO()):
                package_g1_posnav.main()
            with tarfile.open(output, 'r:gz') as archive:
                names = set(archive.getnames())
                for member in archive.getmembers():
                    if not member.isfile() or not member.name.startswith('RealWorldG1/tools/') or not member.name.endswith('.py'):
                        continue
                    tree = ast.parse(archive.extractfile(member).read().decode('utf-8-sig'))
                    for node in ast.walk(tree):
                        modules = ([item.name for item in node.names] if isinstance(node, ast.Import)
                                   else [node.module] if isinstance(node, ast.ImportFrom) and node.level == 0 and node.module else [])
                        for module in modules:
                            local = ROOT/'tools'/(module.split('.')[0]+'.py')
                            if local.is_file():
                                self.assertIn('RealWorldG1/tools/'+local.name, names, member.name)
                self.assertIn('RealWorldG1/tests/frozen_baselines.py', names)
                self.assertIn('RealWorldG1/tests/baselines/manifest.json', names)
                self.assertIn('RealWorldG1/tests/baselines/round1_posnav_fmm/planner.py', names)
                self.assertIn('RealWorldG1/docs/619_navigation_hardening_20261126.md', names)


if __name__ == '__main__':
    unittest.main()
