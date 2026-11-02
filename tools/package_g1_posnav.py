#!/usr/bin/env python3
"""Package the approved static map, live runtime and patched localization source for ARM."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import tempfile


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    if args.output.exists():raise SystemExit('Output already exists; choose a new archive path')
    with tempfile.TemporaryDirectory(prefix='g1-posnav-bundle-') as directory:
        staged=Path(directory)/'RealWorldG1';staged.mkdir()
        files=['docs/G1_PosNav真机部署.md','posnav/README.md','tools/live_bev_node.py','tools/live_bev_core.py','tools/run_live_bev.sh','tools/g1_live_desktop.py',
               'fastlio/config/live_bev.yaml','fastlio/rviz/live_bev.rviz',
               'tests/test_live_bev_core.py','tests/test_live_bev_node.py','tests/test_posnav_fmm.py',
               'tools/g1_navigation_core.py','tools/g1_navigation_node.py','tools/g1_action_server.py','tools/stg_error_recorder.py',
               'tests/test_g1_action_server.py','docs/G1_Action接口与TCP联调.md',
               'tools/experiment_recorder.py','tests/test_experiment_recorder.py',
               'tests/test_experiment_navigation.py',
               'tools/run_g1_live_bev.sh','tests/test_stg_error_recorder.py',
               'tests/test_g1_navigation_core.py','tests/test_g1_navigation_node.py',
               'tests/check_g1_navigation_loop.py']
        for folder in ['posnav/deploy','posnav/interfaces','posnav/bridge','tools/posnav_fmm','fastlio/rviz/pose_replay_panel']:
            files += [str(p.relative_to(root)) for p in (root/folder).rglob('*') if p.is_file() and "__pycache__" not in p.parts]
        map_dir=root/'maps/full_lab_v1'
        manifest=json.loads((map_dir/'package_manifest.json').read_text())
        for entry in manifest['files']:
            if hashlib.sha256((map_dir/entry['path']).read_bytes()).hexdigest()!=entry['sha256']:
                raise ValueError('Formal map checksum mismatch: '+entry['path'])
        files += ['maps/full_lab_v1/'+entry['path'] for entry in manifest['files']]
        files += ['maps/full_lab_v1/package_manifest.json','maps/full_lab_v1/checksums.sha256']
        for file in files:
            dest=staged/file;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/file,dest)
        source=root/'.cache/fastlio_ws/src/FAST_LIO_LOCALIZATION_HUMANOID/open3d_loc'
        if 'pub_live_registration_' not in (source/'src/global_localization.cpp').read_text():
            raise ValueError('Apply the live localization patch before packaging')
        dest=staged/'posnav/source/open3d_loc';shutil.copytree(source,dest)
        cmake=dest/'CMakeLists.txt'
        cmake.write_text(cmake.read_text().replace('        c++\n        c++abi\n',''))
        localization=dest/'src/global_localization.cpp'
        text=localization.read_text()
        for header in ['transform_broadcaster','transform_listener','buffer','static_transform_broadcaster']:
            text=text.replace('tf2_ros/'+header+'.hpp','tf2_ros/'+header+'.h')
        localization.write_text(text)
        registration=dest/'src/open3d_registration/open3d_registration.cpp'
        text=registration.read_text().replace('#include <open3d/utility/Random.h>',
            '#if __has_include(<open3d/utility/Random.h>)\n#include <open3d/utility/Random.h>\n#define G1_OPEN3D_RANDOM_API 1\n#endif\n#include <stdexcept>')
        text=text.replace('        if (seed_) open3d::utility::random::Seed(static_cast<int>(*seed_));',
            '#ifdef G1_OPEN3D_RANDOM_API\n        if (seed_) open3d::utility::random::Seed(static_cast<int>(*seed_));\n#else\n        if (seed_) throw std::runtime_error("Seeded FPFH requires newer Open3D; live localization uses ICP");\n#endif')
        registration.write_text(text)
        # ARM distro Open3D links libstdc++; do not force the downloaded x86 libc++ ABI.
        hashes={str(p.relative_to(staged)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in staged.rglob('*') if p.is_file()}
        (staged/'posnav/deploy/source_manifest.json').write_text(json.dumps(hashes,indent=2)+'\n')
        with tarfile.open(args.output,'w:gz') as archive:archive.add(staged,arcname='RealWorldG1')
    print(f'{args.output}: {args.output.stat().st_size} bytes')

if __name__=='__main__':main()
