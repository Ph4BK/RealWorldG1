"""Reduce timestamped PosNav poses around timed-command expiry (no ROS imports)."""
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import tempfile

MAX_SAMPLE_GAP_S = 0.5


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def prepare_output(root):
    # Recording has no plotting dependency. Check output access before motion.
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    prefix = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ_')
    return Path(tempfile.mkdtemp(prefix=prefix, dir=str(root)))


def analyze_calibration(report):
    if not report.get('command_completed') or report.get('zero_velocity_sent'):
        raise ValueError('Only a completed action without zero-velocity fallback is a calibration sample')
    raw = report['calibration']
    expiry = raw['expiry_ros_ns']
    source = raw['poses']
    if len(source) < 2:
        raise ValueError('Not enough PosNav poses')
    previous = None
    for pose in source:
        if pose['frame_id'] != 'map' or not all(math.isfinite(pose[k]) for k in ('x', 'y', 'yaw_rad')):
            raise ValueError('Invalid map pose')
        if previous is not None and pose['stamp_ns'] <= previous:
            raise ValueError('PosNav stamps must strictly increase')
        age = (pose['received_ros_ns'] - pose['stamp_ns']) / 1e9
        if not 0 <= age <= MAX_SAMPLE_GAP_S:
            raise ValueError('PosNav source age exceeds 0.5 s or clock is inconsistent')
        previous = pose['stamp_ns']

    def endpoint(offset):
        stamp = expiry + round(offset * 1e9)
        for a, b in zip(source, source[1:]):
            if a['stamp_ns'] <= stamp <= b['stamp_ns']:
                gap = (b['stamp_ns'] - a['stamp_ns']) / 1e9
                if gap > MAX_SAMPLE_GAP_S:
                    raise ValueError('Endpoint interpolation spans a gap > 0.5 s')
                fraction = (stamp - a['stamp_ns']) / (b['stamp_ns'] - a['stamp_ns'])
                result = dict(t_s=offset, stamp_ns=stamp, frame_id='map',
                              x=a['x'] + fraction * (b['x'] - a['x']),
                              y=a['y'] + fraction * (b['y'] - a['y']),
                              yaw_rad=wrap(a['yaw_rad'] + fraction * wrap(b['yaw_rad'] - a['yaw_rad'])))
                result['interpolation'] = dict(before_stamp_ns=a['stamp_ns'], after_stamp_ns=b['stamp_ns'],
                                               before_offset_s=(a['stamp_ns']-stamp)/1e9,
                                               after_offset_s=(b['stamp_ns']-stamp)/1e9)
                return result
        raise ValueError('PosNav samples do not bracket expiry + %g s' % offset)

    p0, p2 = endpoint(0.0), endpoint(2.0)
    points = [p0] + [dict(p, t_s=(p['stamp_ns'] - expiry)/1e9) for p in source
                     if expiry < p['stamp_ns'] < expiry + 2_000_000_000] + [p2]
    velocities = []
    for a, b in zip(points, points[1:]):
        dt = (b['stamp_ns'] - a['stamp_ns']) / 1e9
        if not 0 < dt <= MAX_SAMPLE_GAP_S:
            raise ValueError('PosNav gap in 0–2 s window exceeds 0.5 s')
        vx, vy = (b['x']-a['x'])/dt, (b['y']-a['y'])/dt
        dyaw = wrap(b['yaw_rad']-a['yaw_rad'])
        heading = a['yaw_rad'] + dyaw/2
        velocities.append(dict(t_s=(a['t_s']+b['t_s'])/2, t_start_s=a['t_s'], t_end_s=b['t_s'],
                               vx_map_m_s=vx, vy_map_m_s=vy, speed_m_s=math.hypot(vx, vy),
                               forward_m_s=vx*math.cos(heading)+vy*math.sin(heading),
                               yaw_rate_rad_s=dyaw/dt))
    dx, dy = p2['x']-p0['x'], p2['y']-p0['y']
    dyaw = wrap(p2['yaw_rad']-p0['yaw_rad'])
    sign = 1 if raw['action'] == 'left_turn' else -1 if raw['action'] == 'right_turn' else 0
    return dict(action=raw['action'], pose_0s=p0, pose_2s=p2, velocities=velocities,
                residual=dict(dx_map_m=dx, dy_map_m=dy, distance_xy_m=math.hypot(dx, dy),
                              forward_m=dx*math.cos(p0['yaw_rad'])+dy*math.sin(p0['yaw_rad']),
                              yaw_rad=dyaw, yaw_deg=math.degrees(dyaw),
                              yaw_along_command_rad=sign*dyaw if sign else None),
                measurement=dict(pose_topic=raw['pose_topic'], frame_id='map',
                                 time_basis=raw['time_basis'],
                                 endpoints='linear XY interpolation, shortest-angle yaw interpolation at source timestamps',
                                 velocity='unsmoothed adjacent-pose differences / source timestamp interval; interval midpoint',
                                 settle_assumption_s=2.0, stationary_verified=False,
                                 max_source_age_s=max((p['received_ros_ns']-p['stamp_ns'])/1e9 for p in source)))


def save_calibration(report, output_dir):
    result = analyze_calibration(report)
    output_dir = Path(output_dir)
    result['command'] = {k: report[k] for k in ('vx', 'vy', 'yaw_rate', 'duration')}
    result['raw_report'] = report
    json_path, csv_path = (output_dir/name for name in ('result.json', 'velocity.csv'))
    rows = result['velocities']
    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    # Publish the discoverable result only after the CSV is complete.
    temporary = output_dir / 'result.json.tmp'
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    temporary.replace(json_path)
    return dict(action=result['action'], residual=result['residual'],
                files=dict(json=str(json_path), csv=str(csv_path)))


def load_calibrations(paths):
    """Discover recorded runs on disk, deduplicating overlapping selections."""
    files = set()
    for item in paths:
        path = Path(item).expanduser().resolve()
        if path.is_dir():
            files.update(p.resolve() for p in path.rglob('result.json'))
        elif path.is_file() and path.name == 'result.json':
            files.add(path)
        else:
            raise ValueError('Expected an existing experiment directory or result.json: %s' % path)
    if not files:
        raise ValueError('No saved experiments (result.json) found')
    runs = []
    for path in sorted(files):
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
            if data['action'] not in ('forward', 'left_turn', 'right_turn'):
                raise ValueError('unknown action')
            raw = data['raw_report']
            if not raw['command_completed'] or raw['zero_velocity_sent']:
                raise ValueError('incomplete action or stop fallback')
            for key in ('vx', 'vy', 'yaw_rate', 'duration'):
                if not math.isfinite(data['command'][key]):
                    raise ValueError('invalid command')
            rows = data['velocities']
            if not rows:
                raise ValueError('missing velocity samples')
            previous_end = 0.0
            for row in rows:
                if not all(math.isfinite(row[k]) for k in ('t_start_s', 't_end_s', 'speed_m_s', 'forward_m_s', 'yaw_rate_rad_s')):
                    raise ValueError('invalid velocity sample')
                if abs(row['t_start_s'] - previous_end) > 1e-6 or not row['t_start_s'] < row['t_end_s'] <= 2.0:
                    raise ValueError('velocity intervals do not cover 0–2 s continuously')
                previous_end = row['t_end_s']
            if abs(previous_end - 2.0) > 1e-6:
                raise ValueError('velocity trace ends before 2 s')
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('Invalid experiment %s: %s' % (path, exc)) from exc
        runs.append(dict(path=str(path), data=data))
    return runs


def plot_calibrations(paths, output):
    """Offline only: one figure of unsmoothed repeated trials, grouped by action."""
    output = Path(output).expanduser().resolve()
    if output.suffix.lower() != '.png':
        raise ValueError('--plot-output must end with .png')
    if output.with_suffix('.json').name == 'result.json':
        raise ValueError('Choose a summary filename other than result.png to preserve experiment data')
    runs = load_calibrations(paths)
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    figure, axes = plt.subplots(3, 2, figsize=(15, 11), sharex=True)
    actions = ('forward', 'left_turn', 'right_turn')
    counts = {action: 0 for action in actions}
    manifest = []
    try:
        for number, run in enumerate(runs, 1):
            data = run['data']; action = data['action']; command = data['command']
            index = actions.index(action)
            color = plt.get_cmap('tab20')(counts[action] % 20)
            counts[action] += 1
            label = '#%d v=%g w=%g T=%g' % (number, command['vx'], command['yaw_rate'], command['duration'])
            rows = data['velocities']
            times = [r['t_start_s'] for r in rows] + [rows[-1]['t_end_s']]
            for column, key in enumerate(('forward_m_s', 'yaw_rate_rad_s')):
                values = [r[key] for r in rows]
                axes[index, column].step(times, values+[values[-1]], where='post',
                                         label=label, color=color, alpha=.8, linewidth=1.2)
            manifest.append(dict(number=number, action=action, command=command, result_json=run['path']))
        for index, action in enumerate(actions):
            for column, title in enumerate(('Forward velocity (m/s, +forward / -backward)', 'Yaw rate (rad/s)')):
                axis = axes[index, column]
                axis.set_title('%s | %s | %d trials' % (action, title, counts[action]))
                axis.set_ylabel('m/s' if column == 0 else 'rad/s')
                axis.set_xlim(0, 2); axis.axhline(0, color='black', linewidth=.5)
                axis.grid(True, alpha=.3)
                if counts[action]:
                    axis.legend(fontsize=7, ncol=2 if counts[action] > 5 else 1)
                else:
                    axis.text(.5, .5, 'No experiments', transform=axis.transAxes, ha='center')
                if index == 2:
                    axis.set_xlabel('Seconds after local command expiry')
        figure.suptitle('G1 repeated trials | PosNav pose differences, no smoothing\nLegend: trial ID, v (m/s), w (rad/s), T (s)')
        figure.tight_layout(rect=(0, 0, 1, .95))
        output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(str(output), dpi=150)
    finally:
        plt.close(figure)
    manifest_path = output.with_suffix('.json')
    manifest_path.write_text(json.dumps(dict(counts=counts, trials=manifest,
        plotted_fields=dict(linear='forward_m_s', angular='yaw_rate_rad_s'),
        linear_convention='Signed velocity projected onto the midpoint body heading of each pose interval; lateral velocity excluded'),
        ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return dict(counts=counts, files=dict(png=str(output), manifest=str(manifest_path)))
