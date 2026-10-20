"""Offline FAST-LIO replay and trajectory capture. No robot control interfaces."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy


def write_localization_preview(config, output, dataset):
    configs=yaml.safe_load(config.read_text())
    params=next(v['ros__parameters'] for v in configs.values()
                if isinstance(v,dict) and 'path_map' in v.get('ros__parameters',{}))
    map_path=Path(params['path_map'])
    if not map_path.is_absolute():
        raise ValueError('path_map must be absolute for reproducible visualization')
    python=os.environ.get('G1_PLOT_PYTHON',str(Path.home()/'miniconda3/envs/unigoal5060_py39/bin/python'))
    image=output/'localization_bev.png'
    subprocess.run([python,str(Path(__file__).with_name('plot_localization_bev.py')),
                    '--map',str(map_path),'--trajectory',str(output/'localization.csv'),
                    '--output',str(image),'--dataset',dataset],check=True,timeout=120)
    return image.name


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--rate', type=float, default=0.5)
    p.add_argument('--localization-config', type=Path)
    p.add_argument('--mapping-config', type=Path)
    p.add_argument('--save-debug-artifacts', action='store_true',
                   help='Also save clouds, local odometry, internal map and timing during localization')
    args = p.parse_args()
    save_debug = args.localization_config is None or args.save_debug_artifacts
    run_started = time.monotonic()
    if args.rate <= 0:
        p.error('--rate must be positive')
    root = Path(__file__).resolve().parents[1]
    config = args.mapping_config or root/'fastlio/config/g1_mid360.yaml'
    params = yaml.safe_load(config.read_text())['/**']['ros__parameters']
    cube = float(params['cube_side_length'])
    detection = float(params['mapping']['det_range'])
    # Upstream lasermap_fov_segment uses MOV_THRESHOLD=1.5 on each side.
    if not (cube > 3 * detection > 0):
        p.error('cube_side_length must exceed 3 * mapping.det_range; otherwise '
                'the local map can move and delete points while stationary')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('ROS_DOMAIN_ID', '91')
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    os.environ.setdefault('ROS_LOG_DIR', str(args.output/'ros_logs'))
    os.environ.setdefault('OMP_NUM_THREADS', '2')
    processes, logs = [], []

    def launch(name, cmd):
        log = (args.output/(name+'.log')).open('w')
        logs.append(log)
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=args.output,
                                start_new_session=True)
        processes.append(proc)
        return proc

    rclpy.init()
    node = rclpy.create_node('g1_offline_capture')
    rows, loc_rows, confidence, clouds = [], [], [], []
    timing_rows = []
    clock_sample = [None, None]
    odometry_count = 0
    poses_after_player_exit = 0
    playback_ended = None
    player_started = None

    def capture_clock(m):
        clock_sample[:] = [m.clock.sec + m.clock.nanosec/1e9, time.monotonic()]

    node.create_subscription(Clock, '/clock', capture_clock,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def capture_cloud(m):
        fields = {f.name: f for f in m.fields}
        endian = '>' if m.is_bigendian else '<'
        dtype = np.dtype(dict(names=['x','y','z'], formats=[endian+'f4']*3,
                              offsets=[fields[k].offset for k in ['x','y','z']], itemsize=m.point_step))
        a = np.ndarray((m.height,m.width), dtype=dtype, buffer=m.data,
                       strides=(m.row_step,m.point_step))
        xyz = np.column_stack([a[k].reshape(-1) for k in ['x','y','z']])
        clouds.append(xyz[np.isfinite(xyz).all(axis=1)])

    if save_debug:
        node.create_subscription(PointCloud2, '/cloud_registered_1', capture_cloud,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))

    def pose_row(stamp, pose):
        t = stamp.sec + stamp.nanosec/1e9
        a, q = pose.position, pose.orientation
        return [t,a.x,a.y,a.z,q.x,q.y,q.z,q.w]

    def capture_odometry(m):
        nonlocal odometry_count, poses_after_player_exit
        odometry_count += 1
        poses_after_player_exit += int(playback_ended is not None)
        if not save_debug:
            return
        row = pose_row(m.header.stamp,m.pose.pose)
        rows.append(row)
        now = time.monotonic()
        sim, received = clock_sample
        # /clock is sampled; this is capture-side playback lag, not CPU kernel timing.
        timing_rows.append([row[0], now-run_started,
                            sim if sim is not None else float('nan'),
                            now-received if received is not None else float('nan'),
                            sim-row[0] if sim is not None else float('nan'),
                            playback_ended is not None])

    node.create_subscription(Odometry, '/Odometry_loc', capture_odometry, 1000)
    node.create_subscription(PoseStamped, '/localization_3d',
                             lambda m: loc_rows.append(pose_row(m.header.stamp,m.pose)), 1000)
    node.create_subscription(Float32, '/localization_3d_confidence',
                             lambda m: confidence.append(float(m.data)), 1000)
    save = node.create_client(Trigger, '/map_save')
    success = False
    try:
        mapping_cmd = ['ros2','run','fast_lio','fastlio_mapping','--ros-args',
                       '--params-file',str(config.resolve())]
        if save_debug:
            mapping_cmd += ['-p',f'map_file_path:={args.output / "map_raw.pcd"}']
        else:
            # Disable node-side PCD/log accumulation too, not just capture-side files.
            mapping_cmd += ['-p','pcd_save.pcd_save_en:=false',
                            '-p','runtime_pos_log_enable:=false']
        mapping = launch('mapping', mapping_cmd)
        deadline = time.monotonic()+30
        while not save.wait_for_service(timeout_sec=0.2):
            if mapping.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError('FAST-LIO did not start; see mapping.log')
        if args.localization_config:
            # Identity only for sensor-frame offline validation, not a G1 base calibration.
            for parent,child in [('base_link','imu_link'),('base_link','motion_link')]:
                launch('tf_'+child, ['ros2','run','tf2_ros','static_transform_publisher',
                       '--frame-id',parent,'--child-frame-id',child])
            time.sleep(1)
            localization = launch('localization', ['ros2','run','open3d_loc','global_localization_node',
                   '--ros-args','-r','__node:=global_localization_node',
                   '--params-file',str(args.localization_config.resolve()),
                   *([] if save_debug else ['-p','save_scan:=false'])])
            time.sleep(3)
            if localization.poll() is not None:
                raise RuntimeError('Localization failed to start; see localization.log')
        player_started = time.monotonic()
        player = launch('playback', ['ros2','bag','play',str(args.bag.resolve()),
                         '--clock','--delay','3','--rate',str(args.rate),'--disable-keyboard-controls'])
        deadline = time.monotonic()+max(180, 120/args.rate)
        while player.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if mapping.poll() is not None:
                raise RuntimeError('FAST-LIO exited during playback')
            if args.localization_config and localization.poll() is not None:
                raise RuntimeError('Localization exited during playback; see localization.log')
            if time.monotonic()>deadline:
                raise RuntimeError('Playback timeout')
        playback_ended = time.monotonic()
        if player.returncode:
            raise RuntimeError('rosbag play failed; see playback.log')
        until = time.monotonic()+5
        while time.monotonic()<until:
            rclpy.spin_once(node, timeout_sec=0.05)
        if save_debug:
            future = save.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=30)
            if not future.done() or not future.result().success:
                raise RuntimeError('Map save failed')
            with (args.output/'map_raw.pcd').open('rb') as f:
                points = 0
                for line in f:
                    if line.startswith(b'POINTS '):
                        points = int(line.split()[1])
                    if line.startswith(b'DATA '):
                        break
            if points == 0:
                raise RuntimeError('Map service returned an empty PCD')
        if odometry_count<10 or (args.localization_config and len(loc_rows)<10):
            raise RuntimeError('Too few poses; inspect output logs')
        success = True
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=5)
        for file in logs:
            file.close()
        if clouds:
            np.save(args.output/'registered_xyz.npy', np.concatenate(clouds))
        trajectories = [('localization', loc_rows)]
        if save_debug:
            trajectories.insert(0, ('odometry', rows))
        for name, values in trajectories:
            with (args.output/(name+'.csv')).open('w') as f:
                writer = csv.writer(f)
                writer.writerow(['stamp','x','y','z','qx','qy','qz','qw'])
                writer.writerows(values)
        if save_debug:
            with (args.output/'output_timing.csv').open('w') as f:
                w = csv.writer(f)
                w.writerow(['pose_stamp','elapsed_wall_s','latest_clock_stamp','clock_sample_age_wall_s','clock_minus_pose_s','after_player_exit'])
                w.writerows(timing_rows)
        summary = dict(success=success, bag=str(args.bag.resolve()), rate=args.rate,
                       wall_seconds=time.monotonic()-run_started,
                       playback_process_seconds=(playback_ended-player_started if playback_ended is not None else None),
                       poses_after_player_exit=poses_after_player_exit,
                       debug_artifacts_saved=save_debug,
                       trajectory_frame=('map' if args.localization_config else 'camera_init'),
                       quality_validated=False,
                       odometry_poses=odometry_count,localization_poses=len(loc_rows),
                       registered_cloud_frames=(len(clouds) if save_debug else None),
                       confidence_min=min(confidence,default=None),
                       confidence_max=max(confidence,default=None),
                       pose_reference='IMU/body; identity base transforms are placeholders')
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()
    if args.localization_config and success:
        try:
            summary['trajectory_preview']=write_localization_preview(
                args.localization_config,args.output,args.bag.resolve().parent.name)
        except Exception as exc:
            summary['trajectory_preview_error']=str(exc)
            raise RuntimeError('Replay succeeded but trajectory visualization failed') from exc
        finally:
            (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary),flush=True)


if __name__ == '__main__':
    main()
