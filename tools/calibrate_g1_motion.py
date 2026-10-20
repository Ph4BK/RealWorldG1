#!/usr/bin/env python3
"""Execute one timed G1 action; opt in to PosNav expiry calibration."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from RealWorldG1.src.g1_motion_executor import G1MotionExecutor, G1MotionError, validate_pulse


def main(argv=None):
    parser = argparse.ArgumentParser(description='直接执行一次 G1 FSM 501 动作。先站稳、留净空并准备遥控接管。')
    parser.add_argument('--speed', '--vx', type=float, help='前向速度 m/s：正前进、负后退')
    parser.add_argument('--lateral-speed', '--vy', type=float, help='横向速度 m/s：正向左、负向右；可与 --speed 同用，待本机标定')
    parser.add_argument('--yaw-rate', '--omega', type=float, help='角速度 rad/s：正左转、负右转；不能与非零平移速度同用')
    parser.add_argument('--duration', type=float, help='动作必须指定持续时间 s：大于 0，无应用层时长上限')
    parser.add_argument('--calibrate', action='store_true', help='记录 PosNav 到期后 0–2 s 位姿及差分速度，仅保存 JSON/CSV；仅支持前进、左转、右转')
    parser.add_argument('--odom', action='store_true', help='仅查询当前 odom，不发送任何运动或 FSM 请求；不能混用动作参数')
    parser.add_argument('--plot', nargs='*', type=Path, metavar='PATH', help='仅汇总已有实验绘图；省略路径时扫描默认结果目录，可指定多个实验/批次目录或 result.json')
    parser.add_argument('--plot-output', type=Path, help='仅搭配 --plot，指定汇总 PNG 路径')
    args = parser.parse_args(argv)
    results_root = ROOT / 'RealWorldG1' / 'calibration_results'
    # The plot branch returns before motion validation or backend construction.
    if args.plot is not None:
        if args.calibrate or args.odom or any(v is not None for v in (args.speed, args.lateral_speed, args.yaw_rate, args.duration)):
            parser.error('--plot 只允许绘图，不能与运控参数、--calibrate 或 --odom 混用')
        try:
            from RealWorldG1.src.motion_calibration import plot_calibrations
            report = plot_calibrations(args.plot or [results_root], args.plot_output or results_root / 'velocity_summary.png')
        except (OSError, ValueError, RuntimeError, ImportError) as exc:
            print('绘图失败：' + str(exc), file=sys.stderr)
            return 1
        print(json.dumps(report, ensure_ascii=False), flush=True)
        return 0
    if args.plot_output is not None:
        parser.error('--plot-output 只能与 --plot 同用')
    if args.odom:
        if args.calibrate or any(v is not None for v in (args.speed,args.lateral_speed,args.yaw_rate,args.duration)):
            parser.error('--odom 不能与动作参数混用')
    elif args.duration is None:
        parser.error('动作必须指定 --duration')
    vx = args.speed if args.speed is not None else 0.0
    vy = args.lateral_speed if args.lateral_speed is not None else 0.0
    yaw = args.yaw_rate if args.yaw_rate is not None else 0.0
    try:
        if not args.odom:
            validate_pulse(vx, yaw, args.duration, vy)
        if not args.odom and vx == vy == yaw == 0:
            raise ValueError('标定请选择非零速度；此入口不是急停工具')
        if args.calibrate and (vx < 0 or vy != 0):
            raise ValueError('--calibrate 仅支持前进、左转、右转')
    except ValueError as exc:
        parser.error(str(exc))
    output_dir = None
    try:
        if args.calibrate:
            from RealWorldG1.src.motion_calibration import prepare_output, save_calibration
            output_dir = prepare_output(results_root)
        backend = G1MotionExecutor()
        options = {'calibrate': True} if args.calibrate else {}
        report = backend.read_odom() if args.odom else backend.pulse(vx=vx, vy=vy, yaw_rate=yaw, duration=args.duration, **options)
        if args.calibrate:
            report = save_calibration(report, output_dir)
    except G1MotionError as exc:
        detail = (exc.report.get('error') or exc.report.get('stderr') or
                  exc.report.get('stdout') or json.dumps(exc.report, ensure_ascii=False))
        print('执行失败：' + str(detail).strip(), file=sys.stderr)
        return 1
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print('执行失败：' + str(exc), file=sys.stderr)
        return 1
    if args.calibrate or args.odom:
        print(json.dumps(report, ensure_ascii=False), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
