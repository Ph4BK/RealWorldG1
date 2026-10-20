#!/usr/bin/env python3
"""Default: print a plan without importing ROS. Hardware requires local confirmation."""
import argparse
import json
import signal
import sys
import threading
import time
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from RealWorldG1.src.motion_probe import MotionProbe, OdomSample, validate_pulse


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["stop", "forward", "turn"], nargs="?", default="stop")
    parser.add_argument("--speed", type=float, default=None)
    parser.add_argument("--duration", type=float, default=0.3)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    speed = args.speed if args.speed is not None else (0.1 if args.action == "turn" else 0.05)
    vx = speed if args.action == "forward" else 0.0
    omega = speed if args.action == "turn" else 0.0
    validate_pulse(vx, omega, args.duration)
    if args.action == "stop" and args.speed is not None:
        parser.error("--speed is not applicable to stop")
    plan = dict(api_id=7105, velocity=[vx, 0.0, omega], duration=args.duration,
                execute=args.execute)
    print(json.dumps(plan), flush=True)
    if not args.execute:
        print("DRY RUN: no ROS connection or commands sent")
        return 0
    if not sys.stdin.isatty():
        parser.error("hardware execution requires an interactive terminal")
    if input("现场确认机器人稳定、遥控器可接管。输入 RUN 执行以上单次测试: ") != "RUN":
        return 1

    from RealWorldG1.src.g1_api_client import _ConcurrentRequestManager, _decode_data_field
    from RealWorldG1.src.ros2_sport_transport import Ros2SportTransport
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    cancel = threading.Event()
    old_handlers = {s: signal.signal(s, lambda *_: cancel.set()) for s in (signal.SIGINT, signal.SIGTERM)}
    transport = Ros2SportTransport(node_name="unigoal_g1_motion_probe")
    manager = None
    latest = [None]
    last_stamp = [None]
    sample_lock = threading.Lock()

    def receive(msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        v = msg.twist.twist
        with sample_lock:
            if last_stamp[0] is not None and stamp <= last_stamp[0]:
                return  # repeated/backward source timestamps cannot refresh freshness
            last_stamp[0] = stamp
            latest[0] = OdomSample(p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                                  math.hypot(v.linear.x, v.linear.y), v.angular.z, time.monotonic())

    def odom():
        with sample_lock:
            return latest[0]

    def rpc(vx, omega, duration):
        validate_pulse(vx, omega, duration)
        return manager.submit(7105, timeout=0.8, decoder=lambda raw: raw,
                              parameter=json.dumps({"velocity": [vx, 0.0, omega], "duration": duration}))

    try:
        manager = _ConcurrentRequestManager(transport, max_pending=4)
        # Keep our cancellation path even if ROS initialization installed handlers.
        for sig in old_handlers:
            signal.signal(sig, lambda *_: cancel.set())
        transport._node.create_subscription(Odometry, "/dog_odom", receive, qos_profile_sensor_data)
        fsm = manager.submit(7001, timeout=2.0, decoder=_decode_data_field).result()
        print(json.dumps({"fsm_id": fsm}), flush=True)
        if fsm != 802:
            raise RuntimeError("this test profile expects the observed walk/run FSM 802")
        deadline = time.monotonic() + 3
        while odom() is None and time.monotonic() < deadline and not cancel.is_set():
            time.sleep(0.02)
        report = MotionProbe(rpc, odom).run(vx, omega, args.duration, cancel)
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        print("ACK does not prove physical motion or emergency-stop capability.", flush=True)
        return 1 if report["errors"] else 0
    finally:
        if manager is not None:
            manager.close()
        else:
            transport.close()
        for s, handler in old_handlers.items():
            signal.signal(s, handler)


if __name__ == "__main__":
    raise SystemExit(main())
