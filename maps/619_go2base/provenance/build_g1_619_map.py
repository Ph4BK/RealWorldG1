"""Build an offline 619 PosNav map package, never connect to ROS or a robot.

No initial IMU height is guessed: without --imu-height-m and its source the
result is a geometry-only candidate which the existing loader cannot initialize.
"""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import shutil
from types import SimpleNamespace

import numpy as np
import yaml

from build_static_bev import project_bev, read_pcd_xyz
from live_bev_core import RollingBEV, load_fixed_no_go, room_no_go

REPO = Path(__file__).resolve().parents[1]
WORKSPACE = REPO.parent
SOURCE_DIR = (WORKSPACE / '场景图/619/D3-619_地图_v1_2026-10-21/'
              'D3-619_地图_v1_2026-10-21/D3-619_v1')
SCENE_FRAME = 'd3_619_slam_world_x_forward_y_left_z_up_m'


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path, value):
    # JSON is also valid YAML; explicit UTF-8/LF makes artifacts portable.
    with Path(path).open('w', encoding='utf-8', newline='\n') as stream:
        stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + '\n')


def write_text(path, value):
    with Path(path).open('w', encoding='utf-8', newline='\n') as stream:
        stream.write(value)


def source_id(path):
    path = Path(path).resolve()
    try:
        return path.relative_to(WORKSPACE.resolve()).as_posix()
    except ValueError:
        return path.name


def consumer_file_loader_check(map_dir, config_path):
    """Execute the real LiveBEV constructor's map-loading statements, not ROS.

    This is deliberately NOT a full ROS-node integration test. Compile the
    retained source's exact file-reading/assertion block, without importing its
    ROS/FMM dependencies, constructing a Node, or spawning navigation children.
    """
    source = REPO / 'tools/live_bev_node.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LiveBEV')
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    statements = []
    started = False
    for statement in init.body:
        target = statement.targets[0] if isinstance(statement, ast.Assign) else None
        name = target.attr if isinstance(target, ast.Attribute) else None
        if name == 'cfg':
            started = True
        if started:
            statements.append(statement)
        if started and name == 'z':
            break
    if not statements or name != 'z':
        raise ValueError('LiveBEV map-loader source changed; review this check')
    node = SimpleNamespace()
    args = SimpleNamespace(config=Path(config_path), map_dir=Path(map_dir))
    scope = dict(self=node, args=args, np=np, json=json, yaml=yaml, RollingBEV=RollingBEV,
                 load_fixed_no_go=load_fixed_no_go)
    metadata = json.loads((Path(map_dir) / 'metadata.yaml').read_text(encoding='utf-8'))
    pending = metadata['initial_imu_xyz_map'][2] is None
    rejected = False
    try:
        exec(compile(ast.Module(body=statements, type_ignores=[]), str(source), 'exec'), scope)
    except TypeError:
        if not pending or not hasattr(node, 'core'):
            raise
        rejected = True  # float(None): existing loader cannot silently use a fake Z.
    if rejected != pending or not hasattr(node, 'core'):
        raise ValueError('Unexpected consumer initialization outcome')
    return node.core, dict(
        kind='Exact LiveBEV.__init__ file-loading block; NOT a full ROS node test',
        source_sha256=sha256(source), origin_resolution_assertions_passed=True,
        imu_height_missing_rejected=rejected, initialization_height_usable=not pending)


def validate_package(map_dir):
    map_dir = Path(map_dir)
    config = json.loads((map_dir / 'bev_config.json').read_text(encoding='utf-8'))
    metadata = json.loads((map_dir / 'metadata.yaml').read_text(encoding='utf-8'))
    if sha256(map_dir / 'map.pcd') != config['source_sha256']:
        raise ValueError('PCD checksum mismatch')
    with np.load(map_dir / 'bev_occupancy.npz', allow_pickle=False) as data:
        occ, origin, res = data['occ'].copy(), data['origin'].copy(), float(data['res'])
    mode = config.get('bev_source', 'pcd')
    if (occ.shape != tuple(config['shape_rows_cols']) or
            (mode == 'pcd' and occ.dtype != np.bool_) or not np.isin(occ, [0, 1]).all()):
        raise ValueError('Wrong occupancy dtype/shape')
    if not np.array_equal(origin, config['origin_xy_m']) or res != config['resolution_m']:
        raise ValueError('BEV/config origin or resolution mismatch')
    points = read_pcd_xyz(map_dir / 'map.pcd')
    if mode == 'pcd':
        rebuilt, counts, projection = project_bev(points, config)
        if not np.array_equal(occ, rebuilt):
            raise ValueError('Stored BEV differs from PCD reprojection')
        # A different implementation: the real production rolling-grid projector.
        runtime = RollingBEV(np.zeros_like(occ), origin, res, window=1,
                             min_points=config['min_points_per_cell'],
                             z_min=config['height_range_m_inclusive'][0],
                             z_max=config['height_range_m_inclusive'][1],
                             inner_radius=0., radius=float(np.hypot(*occ.shape) * res * 2))
        center = origin + np.array([occ.shape[1], occ.shape[0]]) * res / 2
        if not runtime.add(1., points, [*center, 0.]):
            raise ValueError('Production RollingBEV.add rejected source points')
        if not np.array_equal(runtime.local, occ):
            raise ValueError('Production rolling projector disagrees with static BEV')
    elif mode == 'go2-grid':
        reference = map_dir / config['bev_source_file']
        if (sha256(reference) != config['bev_source_sha256'] or
                sha256(map_dir / 'bev_occupancy.npz') != config['bev_source_sha256']):
            raise ValueError('Cleaned grid must be inherited byte-for-byte')
        with np.load(reference, allow_pickle=False) as grid:
            expected_no_go = room_no_go(occ.shape, origin, res, grid['room_bounds'])
        no_go = load_fixed_no_go(map_dir, config, occ.shape, origin, res)
        if no_go is None or not np.array_equal(no_go, expected_no_go) or not occ[no_go].all():
            raise ValueError('Cleaned grid fixed no-go policy is missing or inconsistent')
        if metadata.get('fixed_no_go') != config['fixed_no_go']:
            raise ValueError('Metadata/config fixed no-go mismatch')
        projection = dict(projection_not_applicable=True)
    else:
        raise ValueError('Unknown BEV source')
    loaded, consumer = consumer_file_loader_check(map_dir, map_dir / 'runtime_live_bev.yaml')
    if not np.array_equal(loaded.static, occ):
        raise ValueError('Consumer loaded different static occupancy')
    transform = np.loadtxt(map_dir / 'T_map_from_scene.txt')
    if transform.shape != (4, 4) or not np.array_equal(transform, np.eye(4)):
        raise ValueError('619 package must preserve the documented scene coordinates')
    with np.load(map_dir / 'provenance/go2_grid_reference.npz', allow_pickle=False) as data:
        old = data['occ'].astype(bool)
        if (old.shape != occ.shape or not np.array_equal(data['origin'], origin) or
                float(data['res']) != res):
            raise ValueError('Source grid extent was changed')
    intersection = int((old & occ).sum())
    union = int((old | occ).sum())
    config_live = yaml.safe_load((map_dir / 'runtime_live_bev.yaml').read_text(encoding='utf-8'))
    if (config_live['action']['coordinate_frame'] != SCENE_FRAME or
            config_live['action']['map_from_scene_xy_yaw'] != [0., 0., 0.] or
            [config_live['z_min'], config_live['z_max']] != config['height_range_m_inclusive']):
        raise ValueError('Runtime action frame/height configuration mismatch')
    if mode == 'go2-grid' and (config_live.get('allow_initial_clear') is not False or
            config_live['fmm_inflation_cells'] != 5 or metadata['planning_inflation']['value_m'] != .25):
        raise ValueError('Cleaned variant must keep explicit no-clear / 0.25m inflation policy')
    if 'T_base_from_imu' in metadata:
        raise ValueError('Do not invent a body/IMU extrinsic in a map package')
    return dict(
        offline_geometry_passed=True,
        runtime_initialization_ready=consumer['initialization_height_usable'],
        motion_or_localization_accuracy_validated=False,
        source_points=len(points), **projection,
        occupied_cells=int(occ.sum()), shape_rows_cols=list(occ.shape),
        origin_xy_m=origin.tolist(), resolution_m=res,
        bev_source=mode,
        fixed_no_go_cells=loaded.no_go_cells,
        pcd_projection_occupied_iou=1. if mode == 'pcd' else None,
        production_rolling_bev_occupied_iou=1. if mode == 'pcd' else None,
        projection_metric_note='Source consistency only, NOT localization accuracy; no PCD projection check for inherited grid',
        go2_reference_occupied_cells=int(old.sum()), go2_reference_iou=intersection / union if union else 1.,
        go2_reference_note=('Comparison only' if mode == 'pcd' else 'Cleaned Go2 grid inherited byte-for-byte; unexpanded'),
        consumer_file_loader=consumer,
        initial_imu_height=metadata['initial_imu_height'],
        transform_map_from_scene=transform.tolist(),
        boundary_policy='Only points within the explicit half-open source-grid XY ROI enter BEV; ICP PCD unchanged',
        all_nonoccupied_cells_are_free=True,
        unobserved_space_is_not_certified_free=True)


def verify_integrity(map_dir):
    map_dir = Path(map_dir)
    manifest = json.loads((map_dir / 'package_manifest.json').read_text(encoding='utf-8'))
    expected = set()
    for entry in manifest['files']:
        relative = Path(entry['path'])
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Unsafe manifest path')
        path = map_dir / relative
        if path.stat().st_size != entry['bytes'] or sha256(path) != entry['sha256']:
            raise ValueError('Manifest mismatch: ' + entry['path'])
        expected.add(relative.as_posix())
    actual = {p.relative_to(map_dir).as_posix() for p in map_dir.rglob('*') if p.is_file()}
    if actual != expected | {'package_manifest.json', 'checksums.sha256'}:
        raise ValueError('Unlisted/missing package files')
    checksum_names = set()
    for line in (map_dir / 'checksums.sha256').read_text(encoding='utf-8').splitlines():
        digest, name = line.split('  ', 1)
        if name not in expected | {'package_manifest.json'} or name in checksum_names:
            raise ValueError('Invalid checksum entry')
        if sha256(map_dir / name) != digest:
            raise ValueError('Checksum mismatch: ' + name)
        checksum_names.add(name)
    if checksum_names != expected | {'package_manifest.json'}:
        raise ValueError('Incomplete checksums')
    return len(manifest['files'])


def make_preview(map_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    with np.load(map_dir / 'bev_occupancy.npz', allow_pickle=False) as data:
        occ, origin, res = data['occ'], data['origin'], float(data['res'])
    with np.load(map_dir / 'provenance/go2_grid_reference.npz', allow_pickle=False) as data:
        old = data['occ'].astype(bool)
    height, width = occ.shape
    extent = [origin[0], origin[0] + width * res, origin[1], origin[1] + height * res]
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)
    config = json.loads((map_dir / 'bev_config.json').read_text(encoding='utf-8'))
    inherited = config.get('bev_source') == 'go2-grid'
    for axis, grid, title in zip(axes, [old, occ], [
            'Go2 cleaned source / reference',
            '619 G1: cleaned grid + fixed room policy' if inherited else
            '619 G1 candidate: 0.05 <= Z <= 1.30 m; count >= 2']):
        axis.imshow(grid, origin='lower', extent=extent, cmap='gray_r', vmin=0, vmax=1,
                    interpolation='nearest')
        axis.plot(0, 0, '+', color='tab:orange', markersize=12)
        axis.set(title=title, xlabel='Scene / map X (m)', ylabel='Scene / map Y (m)')
        axis.set_aspect('equal')
    fig.suptitle('OFFLINE MAP CONVERSION ONLY | no inflation | IMU height / live localization not validated')
    fig.savefig(map_dir / 'bev_preview.png', dpi=160)
    plt.close(fig)


def build_package(source_pcd, source_grid, source_readme, output_dir, *,
                  imu_height_m=None, imu_height_source=None, preview=False, bev_source='pcd'):
    source_pcd, source_grid, source_readme, output_dir = map(
        Path, (source_pcd, source_grid, source_readme, output_dir))
    if output_dir.exists():
        raise FileExistsError('Refuse to overwrite map package: ' + str(output_dir))
    if bev_source not in ('pcd', 'go2-grid'):
        raise ValueError('Unknown BEV source')
    if imu_height_m is not None and (
            isinstance(imu_height_m, bool) or not math.isfinite(imu_height_m) or imu_height_m <= 0 or
            not imu_height_source or not imu_height_source.strip()):
        raise ValueError('IMU height needs a finite positive value AND its measured source')
    if imu_height_m is None and imu_height_source is not None:
        raise ValueError('A height source without a height is invalid')
    inputs = [source_pcd, source_grid, source_readme, REPO / 'fastlio/config/live_bev.yaml']
    hashes = {source_id(p): sha256(p) for p in inputs}
    with np.load(source_grid, allow_pickle=False) as data:
        origin, res, shape = data['origin'].astype(float), float(data['res']), data['occ'].shape
        if len(shape) != 2 or origin.shape != (2,) or not np.isfinite(origin).all() or res != .05:
            raise ValueError('Expected finite 2D 5cm source grid')
        if not np.isin(data['occ'], [0, 1]).all():
            raise ValueError('Source grid must use binary occupancy, without unknown cells')
        inherited_occ = data['occ'].astype(bool)
        bounds = data['room_bounds'].astype(float) if bev_source == 'go2-grid' else None
    no_go = room_no_go(shape, origin, res, bounds) if bev_source == 'go2-grid' else None
    if no_go is not None and not inherited_occ[no_go].all():
        raise ValueError('Source grid does not block its declared outside-room policy')
    points = read_pcd_xyz(source_pcd)
    selected = points[np.isfinite(points).all(axis=1) & (points[:, 2] >= .05) & (points[:, 2] <= 1.3)]
    indices = np.floor((selected[:, :2] - origin) / res).astype(np.int64)
    valid = ((indices[:, 0] >= 0) & (indices[:, 0] < shape[1]) &
             (indices[:, 1] >= 0) & (indices[:, 1] < shape[0]))
    if not len(selected) or valid.mean() <= .95:
        raise ValueError('Height-band points outside the inherited grid exceed 5%; inspect coordinate/ROI mismatch')
    config = dict(version=1, frame='map', semantic_coordinate_frame=SCENE_FRAME,
                  source='map.pcd', source_sha256=hashes[source_id(source_pcd)],
                  height_range_m_inclusive=[.05, 1.3], resolution_m=res, min_points_per_cell=2,
                  parameter_source='803 formal BEV policy; NOT yet validated on 619 G1',
                  origin_xy_m=origin.tolist(), shape_rows_cols=list(shape),
                  projection_method='runtime_floor_half_open_v1',
                  expected_height_points_outside_grid=int((~valid).sum()),
                  indexing='occ[row_y, col_x]; rows increase with map Y; columns increase with map X',
                  cell_center_formula='x=origin_x+(col+0.5)*res; y=origin_y+(row+0.5)*res',
                  cell_index_formula='col,row=floor((xy-origin)/res); half-open XY bounds',
                  occupied=True, inflated=False,
                  count_definition='Number of retained 5cm PCD points, NOT observation frames',
                  all_other_cells='free, including cells without observations; not a free-space certificate',
                  runtime_update='Load fresh static map; fuse real-time observations; inflate in planner; do not overwrite')
    if bev_source == 'pcd':
        occ, _, projection = project_bev(points, config)
        config.update(selected_points=projection['height_selected_points'])
    else:
        occ = inherited_occ
        config.update(bev_source='go2-grid', bev_source_file='provenance/go2_grid_reference.npz',
                      bev_source_sha256=hashes[source_id(source_grid)],
                      parameter_source='Go2 cleaned grid (unexpanded)',
                      projection_method='inherited_cleaned_grid_v1', min_points_per_cell=None,
                      count_definition='Not applicable: cleaned occupancy inherited, not reprojected',
                      height_range_note='Height band applies to runtime observations ONLY, not inherited static occupancy')
    if not occ.any():
        raise ValueError('Empty projected map')
    config.update(obstacle_cells=int(occ.sum()))
    output_dir.mkdir(parents=True)
    provenance = output_dir / 'provenance'
    provenance.mkdir()
    shutil.copyfile(source_pcd, output_dir / 'map.pcd')
    shutil.copyfile(source_grid, provenance / 'go2_grid_reference.npz')
    shutil.copyfile(source_readme, provenance / 'source_map_README.md')
    shutil.copyfile(inputs[-1], provenance / 'live_bev_803_reference.yaml')
    for name in ('build_g1_619_map.py', 'build_static_bev.py'):
        shutil.copyfile(REPO / 'tools' / name, provenance / name)
    if bev_source == 'go2-grid':
        shutil.copyfile(source_grid, output_dir / 'bev_occupancy.npz')
        np.savez_compressed(output_dir / 'no_go.npz', no_go=no_go, origin=origin, res=res)
        config['fixed_no_go'] = dict(file='no_go.npz', sha256=sha256(output_dir / 'no_go.npz'),
            source='go2_room_bounds_grid_anchor_inclusive', room_bounds_xmin_xmax_ymin_ymax_m=bounds.tolist(),
            occupied_cells=int(no_go.sum()),
            semantic='Immutable policy, OR after dynamic fusion AND explicit initial clearing')
    else:
        np.savez_compressed(output_dir / 'bev_occupancy.npz', occ=occ, origin=origin, res=res)
    write_json(output_dir / 'bev_config.json', config)
    ready = imu_height_m is not None
    metadata = dict(
        version='619_scene_v1_candidate' if not ready else '619_scene_v1', frame='map',
        semantic_coordinate_frame=SCENE_FRAME, units='metres',
        source_inputs_sha256=hashes, source_pointcloud=source_id(source_pcd),
        map_coordinate_basis='Source map README: start chassis ground projection; X forward, Y left, Z up',
        map_floor_z=0., floor_basis='Source README declaration, NOT a new floor-plane fit/survey',
        transform_map_from_scene=np.eye(4).tolist(),
        transform_source='619 source README + existing scene-map documentation; no numerical refit',
        navigation_reference='G1 MID-360 internal IMU; NOT Go2 base / G1 feet or body center',
        initial_imu_xyz_map=[None, None, imu_height_m],
        initial_xy_source='Not a fixed starting pose; draw current IMU XY/yaw manually at runtime',
        initial_imu_height=dict(value_m=imu_height_m, status='provided' if ready else 'pending_619_measurement',
                                source=imu_height_source, applies_to='619 G1 navigation standing posture'),
        runtime_initialization_ready=ready,
        robot_body_imu_extrinsics_status='Unknown; must be calibrated separately, never inserted in T_map_from_scene',
        map_bounds_xyz=[points.min(axis=0).tolist(), points.max(axis=0).tolist()],
        limitations=[
            'Offline coordinate consistency only; no live G1 localization or navigation accuracy validated',
            'Inherited 803 height/count policy is not a certified 619 G1 body-clearance model',
            'Source room observations date from 2026-10-12/13; scene may have changed',
            'Missing/occluded points are not evidence of free space',
            'BEV uses inherited grid ROI; original all-height ICP PCD is copied byte-for-byte',
            'G1 IMU yaw/position is not a body/feet-center goal without a separate extrinsic contract'])
    if bev_source == 'go2-grid':
        metadata.update(version='619_go2base', bev_source=bev_source,
            parameter_source='Go2 cleaned grid (unexpanded)', fixed_no_go=config['fixed_no_go'],
            planning_inflation=dict(cells=5, value_m=.25,
                source='User decision 2026-11-21: measured G1 shoulder width about 0.38m; keep existing 5 cells',
                status='User-selected margin, NOT full-body swept-volume certification'),
            initial_height_reference_note='User-provided lidar dome centre height approximates IMU height; not a measured lidar-to-IMU offset')
        metadata['limitations'][1] = 'Cleaned grid is inherited from Go2; G1 taller-body and overhang clearance not certified'
    write_json(output_dir / 'metadata.yaml', metadata)
    write_text(output_dir / 'T_map_from_scene.txt',
               '# p_map = T_map_from_scene @ p_scene; world frames only, not sensor extrinsics\n' +
               '\n'.join(' '.join(f'{v:.17g}' for v in row) for row in np.eye(4)) + '\n')
    runtime_config = yaml.safe_load(inputs[-1].read_text(encoding='utf-8'))
    runtime_config['action']['coordinate_frame'] = SCENE_FRAME
    runtime_config['action']['map_from_scene_xy_yaw'] = [0., 0., 0.]
    runtime_config['allow_initial_clear'] = False
    write_json(output_dir / 'runtime_live_bev.yaml', runtime_config)
    write_text(output_dir / 'README.md', CLEANED_README if bev_source == 'go2-grid' else README)
    if preview:
        make_preview(output_dir)
    report = validate_package(output_dir)
    report['source_inputs_unchanged'] = all(sha256(p) == hashes[source_id(p)] for p in inputs)
    if not report['source_inputs_unchanged']:
        raise ValueError('Source input changed while generating map')
    write_json(output_dir / 'validation.json', report)
    entries = [dict(path=p.relative_to(output_dir).as_posix(), bytes=p.stat().st_size, sha256=sha256(p))
               for p in sorted(output_dir.rglob('*')) if p.is_file()]
    write_json(output_dir / 'package_manifest.json', dict(
        version=1, frame='map', static_bev='bev_occupancy.npz', localization_map='map.pcd',
        runtime_config='runtime_live_bev.yaml', runtime_initialization_ready=ready,
        files=entries, integrity_note='Manifest lists artifacts; checksums additionally covers manifest; neither hashes itself'))
    checksum_paths = [entry['path'] for entry in entries] + ['package_manifest.json']
    write_text(output_dir / 'checksums.sha256', ''.join(
        sha256(output_dir / name) + '  ' + name + '\n' for name in checksum_paths))
    verify_integrity(output_dir)
    return report


README = """# 619 scene-coordinate PosNav map package — offline candidate

map.pcd is an unchanged byte-for-byte copy of the source 5cm PCD for all-height
3D localization. Only BEV uses Z in [0.05,1.30]m, count >=2, 5cm cells, no
inflation. Parameters are inherited from 803, NOT validated 619 body clearance.
The grid origin/shape comes from the Go2 reference, NOT its cleaned occupancy.
All nonoccupied cells are free in this binary format, including unobserved cells;
this is a format policy, NOT a physical free-space guarantee.

Map = documented 619 scene coordinates. T_map_from_scene.txt is identity, NOT
T_base_from_imu. Do not add Go2/G1 mounting offsets into this world transform.
Pose reference is G1 MID-360 IMU. Body/feet-center goals require a separate
extrinsic/goal-reference agreement. No such calibration is invented here.

## Initialization gate

Read metadata.yaml and validation.json first. If initial_imu_xyz_map[2] is null,
619 G1 navigation-posture IMU height is missing. Existing LiveBEV rejects null
during construction. Do NOT replace with zero or 803's 1.261m for convenience.
XY/yaw must be manually initialized at the actual IMU position; the metadata's
null XY values are not a request to start at the scene origin.

## Rebuild (offline; no ROS nodes, deployment, or robot commands)

From the workspace root with numpy/scipy/PyYAML installed:

    python RealWorldG1/tools/build_g1_619_map.py --output-dir <NEW_DIRECTORY> --preview

After measuring this 619 G1 in its navigation standing posture, build a NEW
directory with --imu-height-m <MEASURED_METRES> --imu-height-source <MEASUREMENT_RECORD>.
Never overwrite a previous candidate. Use --validate-only <PACKAGE> to recheck;
--require-runtime-ready additionally rejects a package without its IMU height.
tools/build_static_bev.py can also rebuild this BEV from its PCD/config alone.

For any later approved deployment, both --map-dir and --config must explicitly
select this package and its runtime_live_bev.yaml. The default remains the 803
map/config. This document does not authorize starting a robot process or motion.

## What has and has not been checked

validation.json checks the PCD projection, actual RollingBEV.add projection,
and the EXACT file-loading/assertion statements in LiveBEV.__init__ without
constructing a ROS node. This is not a complete ROS integration test or ICP test.
100% projection agreement is same-source geometry consistency, not localization
accuracy. Source map README is retained; floor Z=0 is its declaration, not a new
survey. The 2026-10-12/13 map may be stale, especially for taller G1 obstacles.
checksums.sha256 covers artifacts and package_manifest.json (not itself).
"""

CLEANED_README = """# 619 Go2-cleaned BEV map package

bev_occupancy.npz is the byte-for-byte Go2 cleaned UNEXPANDED grid, not a PCD
projection. The original all-height ICP map.pcd is unchanged. no_go.npz locks
ONLY the explicit outside-room policy; ordinary obstacles remain dynamically
clearable. Runtime inflation stays 5 cells / 0.25m by user decision, not Go2's
0.35m. This is a user-selected margin, not full-body clearance certification.
allow_initial_clear=false; a blocked current cell must reject navigation.
The live observation height band/count rules do NOT describe static-map creation.
An initialization-ready package is NOT motion permission or accuracy acceptance.
Rebuild this variant with --bev-source go2-grid. build_static_bev.py copies the
retained grid reference for this variant; it must not reproject map.pcd instead.
There is no point-count grid for cleaned occupancy; --counts is rejected.

## Coordinates and limits

map=619 scene, X forward / Y left / Z up; T_map_from_scene.txt is identity,
NOT a sensor/body extrinsic. The initial Z value and its exact measurement source
are in metadata.yaml. A lidar-dome tape measurement approximates IMU height;
do not claim this is a separately surveyed IMU or body/feet reference.
XY/yaw are selected at the current position, not fixed to the scene origin.
The old map may be stale and cleaned Go2 occupancy does not certify all taller
G1 obstacles or missing/occluded space. Dynamic perception remains necessary.

## Offline verification and rebuild

python tools/build_g1_619_map.py --bev-source go2-grid --output-dir <NEW_DIR> \
  --imu-height-m <HEIGHT> --imu-height-source <RECORD> --preview
python tools/build_g1_619_map.py --validate-only <PACKAGE> --require-runtime-ready

Never overwrite the original PCD-projection package. Validation checks inherited
grid bytes, all-height PCD bytes, immutable room policy, checksums, and the exact
production LiveBEV constructor file-loading block. It is not a live ROS/ICP or
motion test. Select BOTH --map-dir and --config runtime_live_bev.yaml explicitly.
Deploy the matching updated live_bev_core/node and navigation/action/planner
sources together; old runtime code does not enforce the new no-go declaration.
This README does not authorize deployment, node startup or physical motion.

"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-pcd', type=Path, default=SOURCE_DIR / 'pointcloud/map_5cm.pcd')
    parser.add_argument('--source-grid', type=Path, default=WORKSPACE / 'nav/tools/619_map_v1_clean_5cm.npz')
    parser.add_argument('--source-readme', type=Path, default=SOURCE_DIR / 'README.md')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--imu-height-m', type=float)
    parser.add_argument('--imu-height-source')
    parser.add_argument('--bev-source', choices=['pcd', 'go2-grid'], default='pcd',
                        help='Explicit static BEV provenance; ICP PCD stays unchanged')
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--validate-only', type=Path)
    parser.add_argument('--require-runtime-ready', action='store_true')
    args = parser.parse_args()
    if args.validate_only:
        report = validate_package(args.validate_only)
        report['manifest_files_verified'] = verify_integrity(args.validate_only)
    else:
        if args.output_dir is None:
            parser.error('--output-dir is required when building')
        report = build_package(args.source_pcd, args.source_grid, args.source_readme, args.output_dir,
                               imu_height_m=args.imu_height_m, imu_height_source=args.imu_height_source,
                               preview=args.preview, bev_source=args.bev_source)
    print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
    if args.require_runtime_ready and not report['runtime_initialization_ready']:
        parser.exit(2, 'IMU_HEIGHT_REQUIRED: measure this 619 G1 and rebuild with height + source\n')


if __name__ == '__main__':
    main()
