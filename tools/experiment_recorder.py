"""Opt-in PosNav evidence bundle. No ROS dependencies and no motion commands.

Each process owns its files; a bounded worker queue keeps disk I/O out of navigation
callbacks. An unfinished manifest is deliberately retained after an unclean exit.
"""
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import queue
import shutil
import subprocess
import threading
import time

import numpy as np

from stg_error_recorder import STGErrorRecorder


def json_default(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(type(value).__name__)


def dumps(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, default=json_default)


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(dumps(value) + '\n', encoding='utf-8')
    temporary.replace(path)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


class Clock:
    def __init__(self, session_id, origin):
        self.session_id, self.origin = session_id, origin
        self.seq = 0

    def stamp(self, source_stamp_ns=None):
        self.seq += 1
        return dict(session_id=self.session_id, seq=self.seq,
                    timestamp=datetime.now(timezone.utc).isoformat(timespec='microseconds'),
                    elapsed_s=time.monotonic() - self.origin,
                    source_stamp_ns=source_stamp_ns)


class AsyncWriter:
    def __init__(self, root, capacity=2048):
        self.root = Path(root)
        self.queue = queue.Queue(maxsize=capacity)
        self.error = None
        self.dropped = 0
        self.closed = False
        self.thread = threading.Thread(target=self._work, name='posnav-evidence-writer', daemon=True)
        self.thread.start()

    def submit(self, kind, relative, payload):
        if self.closed or self.error:
            self.dropped += 1
            return False
        try:
            self.queue.put_nowait((kind, relative, payload))
            return True
        except queue.Full:
            self.error = 'Experiment writer queue is full'
            self.dropped += 1
            return False

    def json(self, relative, payload):
        return self.submit('json', relative, payload)

    def line(self, relative, payload):
        return self.submit('line', relative, payload)

    def _write(self, kind, path, payload):
        path.parent.mkdir(parents=True, exist_ok=True)
        if kind == 'json':
            write_json(path, payload)
        elif kind == 'line':
            with path.open('a', encoding='utf-8') as stream:
                stream.write(dumps(payload) + '\n')
        elif kind == 'csv':
            new = not path.exists()
            with path.open('a', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(payload))
                if new:
                    writer.writeheader()
                writer.writerow(payload)
        elif kind == 'npz':
            np.savez_compressed(path, **payload)
        elif kind == 'touch':
            path.touch(exist_ok=True)
        else:
            raise ValueError('Unknown writer operation: ' + kind)

    def _work(self):
        while True:
            job = self.queue.get()
            try:
                if job is None:
                    return
                self._write(job[0], self.root / job[1], job[2])
            except Exception as exc:
                self.error = self.error or f'{type(exc).__name__}: {exc}'
                self.dropped += 1
            finally:
                self.queue.task_done()

    def close(self, timeout=10):
        if not self.closed:
            self.closed = True
            try:
                self.queue.put(None, timeout=timeout)
            except queue.Full:
                self.error = self.error or 'Experiment writer could not drain on shutdown'
                return False
            self.thread.join(timeout)
            if self.thread.is_alive():
                self.error = self.error or 'Experiment writer shutdown timed out'
        return not self.error and not self.thread.is_alive()


BUNDLE_README = """G1 PosNav 实验记录（schema_version=1）

events.jsonl: 导航操作、状态、控制指令变化、健康采样、任务起止。
localization_events.jsonl: 初始箭头、定位启动/停止、地图重置及初始化配置。
按 elapsed_s 合并两条时间线；seq 仅在各自进程内递增。
timestamp 为 UTC ISO 8601（+00:00），北京时间为 UTC+08:00；elapsed_s 使用
同一机载单调时钟。source_stamp_ns 是原始 ROS 时间，缺失时 null，不冒充墙上时间。
每次接受 Start Navigation 创建 trials/trial_NNNN；拒绝请求只写事件。
task.json 保存导航开始时的实际定位位姿，与手工初始箭头分开。
trajectory.csv 是导航节点实际收到的位姿样本，不保证包含发布者的每条消息；
result.json 中 pose_age_s 为终态所用位姿的接收年龄。位置为 map 系米，yaw
为弧度。轨迹长度为采样估计。
planning.jsonl 包含所有调用规划器的尝试（含无 STG/异常），以及地图引用和
规划器实际输出的 rollout_rc；它不是机器人实际轨迹，也不是完整全局路径。
stg_segments.jsonl 保存阶段端点、阶段时间、异常和无异常段的 XY L2 误差。
maps/*.npz 的 data 为原始 OccupancyGrid，配套 JSON 保存原点姿态、分辨率、
frame_id、源时间及记录时间；快照覆盖任务起止、规划、通行检查失败，不是全量地图流。
command_change 只记录指令变化，非完整 50 Hz 指令流，也不是实际速度测量。
annotations.jsonl 初始为空，人工可填 timestamp、trial_id、干扰类型、起止、描述、视频索引。
若后补人工说明，需重新生成 checksums.sha256，或将说明作为外部补充交付。

当前判据仅为定位输出下 XY 距目标 <= 0.50 m；不实现末端精调/最终 yaw 对准。
result.success 仅表示上述程序判据，不代表独立物理真值或正式验收结论。
等待障碍/等待输入/规划重试不是任务失败。未知、不完整记录不自动算成功。
recording_complete 指正常收尾且未发现写盘/队列错误，不是 DDS 零丢包保证。
强杀或断电时 session/result 可能保留未完成状态；不得推断成功。
component_logs 是本次组件诊断输出，仍可能按组件策略轮转，不替代结构化记录。
不包含原始雷达/IMU 全量数据，本包用于过程复核，不能完整重放感知流水线。
assets/map 为本次使用的地图副本；assets/config 为配置及每次定位初始化；
assets/source 为关键 Python/配置/启动源码副本，不能等同于所有外部依赖的构建归档。
checksums.sha256 在正常收尾后生成，不包含自身。
"""


class ExperimentSession:
    """Owned by live_bev_node; the navigation child writes separate bundle files."""
    def __init__(self, root, map_dir, config, output_dir=None):
        root, map_dir, config = Path(root), Path(map_dir), Path(config)
        session_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.path = (Path(output_dir) if output_dir else root / 'posnav_experiments') / session_id
        self.path.mkdir(parents=True, exist_ok=False)
        self.clock = Clock(session_id, time.monotonic())
        self.info = dict(schema_version=1, **self.clock.stamp(), platform='G1',
                         monotonic_origin=self.clock.origin, status='initializing',
                         recording_complete=False, original_map_dir=str(map_dir.resolve()),
                         original_config=str(config.resolve()), units=dict(position='m', angle='rad'),
                         frame_id='map', final_yaw_control=False)
        write_json(self.path / 'session.json', self.info)
        shutil.copytree(map_dir, self.path / 'assets/map')
        destination = self.path / 'assets/config/live_bev.yaml'
        destination.parent.mkdir(parents=True)
        shutil.copy2(config, destination)
        files = ['tools/g1_navigation_core.py', 'tools/g1_navigation_node.py', 'tools/g1_action_server.py',
                 'tools/stg_error_recorder.py', 'tools/experiment_recorder.py',
                 'tools/live_bev_core.py', 'tools/live_bev_node.py', 'tools/g1_live_desktop.py',
                 'tools/run_g1_live_bev.sh', 'tools/run_live_bev.sh',
                 'posnav/deploy/start_desktop.sh', 'posnav/deploy/env.sh',
                 'fastlio/rviz/live_bev.rviz']
        files += [str(p.relative_to(root)) for p in (root / 'tools/posnav_fmm').rglob('*.py')]
        for folder in ('posnav/interfaces', 'posnav/bridge'):
            files += [str(p.relative_to(root)) for p in (root / folder).rglob('*')
                      if p.is_file() and '__pycache__' not in p.parts]
        for relative in files:
            dest = self.path / 'assets/source' / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(root / relative, dest)
        try:
            self.info['git_commit'] = subprocess.check_output(
                ['git', '-C', str(root), 'rev-parse', 'HEAD'], stderr=subprocess.DEVNULL,
                text=True, timeout=2).strip()
        except (OSError, subprocess.SubprocessError):
            self.info['git_commit'] = None
        self.info['source_identity'] = 'Bundled source SHA-256 is authoritative; Git may exclude this directory.'
        self.info['status'] = 'recording'
        write_json(self.path / 'session.json', self.info)
        (self.path / 'README.txt').write_text(BUNDLE_README, encoding='utf-8')
        (self.path / 'component_logs').mkdir()
        self.writer = AsyncWriter(self.path)
        self.closed = False
        self.initializations = 0
        self.event('session_started')

    def event(self, event, source_stamp_ns=None, **data):
        self.writer.line('localization_events.jsonl', dict(**self.clock.stamp(source_stamp_ns), event=event, **data))

    def initialization(self, initial, params):
        self.initializations += 1
        relative = f'assets/config/localization_{self.initializations:04d}.json'
        self.writer.json(relative, dict(**self.clock.stamp(), initial=initial, parameters=params))
        self.event('localization_initialized', config_ref=relative)

    def close(self, clean=True, error=None):
        if self.closed:
            return
        self.closed = True
        self.event('session_ending', clean=clean, error=error)
        written = self.writer.close()
        nav_file = self.path / 'navigation_recording.json'
        nav = json.loads(nav_file.read_text()) if nav_file.exists() else {}
        complete = clean and written and nav.get('recording_complete', False)
        self.info.update(end=self.clock.stamp(), status='closed' if complete else 'incomplete',
                         recording_complete=bool(complete), error=error,
                         localization_writer_error=self.writer.error,
                         localization_dropped_records=self.writer.dropped,
                         navigation_recording=nav)
        write_json(self.path / 'session.json', self.info)
        if not self.writer.thread.is_alive():
            paths = sorted(p for p in self.path.rglob('*') if p.is_file() and p != self.path/'checksums.sha256')
            (self.path / 'checksums.sha256').write_text(''.join(
                f'{sha256(p)}  {p.relative_to(self.path).as_posix()}\n' for p in paths), encoding='utf-8')


class TrialRecorder:
    def __init__(self, path, parameters):
        self.path = Path(path)
        info = json.loads((self.path / 'session.json').read_text())
        self.clock = Clock(info['session_id'], info['monotonic_origin'])
        self.writer = AsyncWriter(self.path)
        self.trial_id = None
        self.stg_id = None
        self.count = self.attempt = self.stg_count = 0
        self.obstacles = 0
        self.last_command = None
        self.map_refs = {}
        self.results = {}
        self.latest_pose_stamp = None
        self.closed = False
        self.complete = False
        self.writer.json('assets/config/navigation_parameters.json', parameters)
        self.writer.json('navigation_recording.json', dict(recording_complete=False, status='recording'))
        self.event('navigation_recorder_started')

    def stamp(self, source_stamp_ns=None):
        return dict(**self.clock.stamp(source_stamp_ns), trial_id=self.trial_id, stg_id=self.stg_id)

    def event(self, event, source_stamp_ns=None, **data):
        self.writer.line('events.jsonl', dict(**self.stamp(source_stamp_ns), event=event, **data))

    def relative(self, filename):
        return f'trials/{self.trial_id}/{filename}'

    def begin(self, goal, pose, source_stamp_ns, goal_tol):
        self.count += 1
        self.trial_id = f'trial_{self.count:04d}'
        self.stg_id = None
        self.attempt = self.stg_count = self.obstacles = self.pose_samples = 0
        self.map_refs = {}
        self.last_command = self.previous_pose = None
        self.goal = tuple(goal)
        self.latest_pose_stamp = source_stamp_ns
        self.path_length = 0.
        self.started = self.stamp(source_stamp_ns)
        self.writer.json(self.relative('task.json'), dict(**self.started, goal=goal,
            initial_pose=pose, frame_id='map', goal_tolerance_m=goal_tol,
            final_yaw_control=False, success_basis='localization_xy_distance'))
        self.writer.json(self.relative('result.json'), dict(**self.started, outcome='running',
            success=None, recording_complete=False))
        self.writer.submit('touch', self.relative('annotations.jsonl'), None)
        for filename in ('planning.jsonl', 'stg_segments.jsonl'):
            self.writer.submit('touch', self.relative(filename), None)
        self.event('trial_started', goal=goal, initial_pose=pose)

    def snapshot(self, grid, metadata):
        if not self.trial_id or grid is None:
            return None
        # Navigation callbacks replace grids rather than modifying them in place.
        key = metadata['source_stamp_ns']
        if key not in self.map_refs:
            relative = self.relative(f'maps/task_map_{len(self.map_refs)+1:04d}.npz')
            self.map_refs[key] = relative
            self.writer.submit('npz', relative, dict(data=grid.copy()))
            self.writer.json(relative.replace('.npz', '.json'), dict(
                **self.stamp(key), grid=metadata, data_file=Path(relative).name))
        return self.map_refs[key]

    def planning(self, pose, goal, map_ref, outcome, reason, stg=None, rollout_rc=None, duration_s=None):
        if not self.trial_id:
            return
        self.attempt += 1
        self.writer.line(self.relative('planning.jsonl'), dict(**self.stamp(),
            attempt_id=self.attempt, start_xy=pose, goal_xy=goal, map_ref=map_ref,
            outcome=outcome, reason=reason, selected_stg=stg, rollout_rc=rollout_rc,
            duration_s=duration_s))

    def pose(self, pose, source_stamp_ns, state):
        self.latest_pose_stamp = source_stamp_ns
        if not self.trial_id:
            return
        stamp = self.stamp(source_stamp_ns)
        if self.previous_pose is not None:
            self.path_length += math.hypot(pose[0]-self.previous_pose[0], pose[1]-self.previous_pose[1])
        self.previous_pose = tuple(pose)
        self.pose_samples += 1
        self.writer.submit('csv', self.relative('trajectory.csv'), dict(**stamp,
            x=pose[0], y=pose[1], yaw=pose[2], state=state,
            remaining_dist_m=math.hypot(pose[0]-self.goal[0], pose[1]-self.goal[1])))

    def command(self, velocity, duration):
        if tuple(velocity) != self.last_command:
            self.event('command_change', velocity=[velocity[0], 0., velocity[1]], duration_s=duration)
            self.last_command = tuple(velocity)

    def finish(self, reason, pose, pose_age_s, input_problem, map_ref):
        if not self.trial_id:
            return
        success = reason == 'Arrived within 0.50 m'
        outcome = ('arrived' if success else 'cancelled' if reason in (
            'Stopped by user', 'Goal redrawn; press Start Navigation',
            'Localization stopped or map reset') else 'interrupted' if reason ==
            'Navigation process stopped' else 'error')
        result = dict(**self.stamp(self.latest_pose_stamp), outcome=outcome, reason=reason, success=success,
            final_pose=pose, pose_age_s=pose_age_s, input_problem=input_problem,
            final_map_ref=map_ref, elapsed_task_s=time.monotonic()-self.clock.origin-self.started['elapsed_s'],
            internal_error_m=None if pose is None else math.hypot(pose[0]-self.goal[0], pose[1]-self.goal[1]),
            planning_attempts=self.attempt, stg_count=self.stg_count,
            obstacle_checks_failed=self.obstacles, pose_samples=self.pose_samples,
            sampled_path_length_m=self.path_length, recording_complete=not bool(self.writer.error))
        relative = self.relative('result.json')
        self.results[relative] = result
        self.writer.json(relative, result)
        self.event('trial_finished', result_ref=relative, outcome=outcome, reason=reason)
        self.trial_id = self.stg_id = None

    def close(self):
        if self.closed:
            return self.complete
        self.closed = True
        written = self.writer.close()
        complete = written and self.trial_id is None
        self.complete = complete
        if not self.writer.thread.is_alive():
            if not complete:
                for relative, result in self.results.items():
                    result = dict(result, recording_complete=False)
                    write_json(self.path / relative, result)
            write_json(self.path / 'navigation_recording.json', dict(recording_complete=complete,
                writer_error=self.writer.error, dropped_records=self.writer.dropped,
                trial_count=self.count, unfinished_trial=self.trial_id))
        return complete


class ExperimentSTGRecorder(STGErrorRecorder):
    """Reuse existing measurements while preserving their standalone JSONL schema."""
    def __init__(self, experiment):
        super().__init__()
        self.experiment = experiment
        self.times = {}

    def begin(self, pose, stg):
        super().begin(pose, stg)
        e = self.experiment
        e.stg_count += 1
        e.stg_id = f'stg_{e.stg_count:04d}'
        self.times = dict(begin=e.stamp(e.latest_pose_stamp))
        e.event('stg_started', initial_pose=pose, stg=stg)

    def turn_settled(self, pose):
        if self.current is not None:
            self.times['turn_settled'] = self.experiment.stamp(self.experiment.latest_pose_stamp)
        super().turn_settled(pose)

    def walk_settled(self, pose):
        if self.current is not None:
            self.times['walk_settled'] = self.experiment.stamp(self.experiment.latest_pose_stamp)
        super().walk_settled(pose)

    def anomaly(self, reason):
        if self.current is not None and reason not in self.current['anomalies']:
            self.experiment.event('stg_anomaly', reason=reason)
        super().anomaly(reason)

    def _finish(self):
        had_current = self.current is not None
        super()._finish()
        if had_current:
            e = self.experiment
            e.writer.line(e.relative('stg_segments.jsonl'), dict(
                **e.stamp(), **self.completed[-1], phase_times=dict(self.times)))
            e.stg_id = None
