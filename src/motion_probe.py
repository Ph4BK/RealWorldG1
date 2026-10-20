"""Bounded, single-pulse experiment; RPC acceptance is not motion success."""
import math
import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class OdomSample:
    x: float
    y: float
    yaw: float
    speed: float
    yaw_rate: float
    received: float


def validate_pulse(vx, omega, duration):
    if not all(math.isfinite(v) for v in (vx, omega, duration)):
        raise ValueError("pulse values must be finite")
    if abs(vx) > 0.1 or abs(omega) > 0.15 or not 0 < duration <= 1.0:
        raise ValueError("limits: |vx| <= 0.1 m/s, |omega| <= 0.15 rad/s, duration <= 1.0 s")
    if vx and omega:
        raise ValueError("translation and rotation must be tested separately")


class MotionProbe:
    """rpc(vx, omega, duration) returns a Future for API 7105.

    A local monitor keeps checking odometry while awaiting the motion ACK.
    Stop bypasses the in-flight motion ACK; all experiments share one lock.
    This is a bounded test, not a certified emergency-stop controller.
    """

    def __init__(self, rpc, odom, *, clock=time.monotonic, sleep=time.sleep):
        self.rpc, self.odom = rpc, odom
        self.clock, self.sleep = clock, sleep
        self._lock = threading.Lock()

    def sample(self):
        s = self.odom()
        if s is None or not all(math.isfinite(v) for v in vars(s).values()):
            raise RuntimeError("missing or invalid odometry")
        if not 0 <= self.clock() - s.received <= 0.25:
            raise RuntimeError("odometry stale (> 0.25 s)")
        return s

    def run(self, vx=0.0, omega=0.0, duration=0.3, cancel=None):
        validate_pulse(vx, omega, duration)
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("another motion experiment is active")
        report = {"motion_ack": False, "stop_ack": False,
                  "stationary_observed": False, "errors": []}
        started = False
        ack = None
        try:
            first = self.sample()
            if first.speed > 0.03 or abs(first.yaw_rate) > 0.08:
                raise RuntimeError("robot must be stationary before test")
            if cancel is not None and cancel.is_set():
                raise RuntimeError("cancelled before publication")
            started = True  # publish may throw after handing data to DDS
            deadline = self.clock() + duration
            ack = self.rpc(vx, omega, duration)
            while True:
                if cancel is not None and cancel.is_set():
                    raise RuntimeError("experiment interrupted")
                current = self.sample()
                dyaw = math.atan2(math.sin(current.yaw-first.yaw), math.cos(current.yaw-first.yaw))
                if math.hypot(current.x-first.x, current.y-first.y) > 0.15 or abs(dyaw) > 0.2:
                    raise RuntimeError("observed displacement exceeded test bound")
                if ack.done():
                    ack.result()
                    report["motion_ack"] = True
                if self.clock() >= deadline:
                    break
                self.sleep(0.01)
        except BaseException as exc:
            report["errors"].append(f"{type(exc).__name__}: {exc}")
        finally:
            try:
                if started:
                    # Always attempt a zero-velocity command, even if the
                    # motion response is still pending or the odometry failed.
                    stop = self.rpc(0.0, 0.0, 0.3)
                    stop.result(timeout=1.0)
                    report["stop_ack"] = True
                    deadline = self.clock() + 1.5
                    stable_since = None
                    while self.clock() < deadline:
                        s = self.sample()
                        if s.speed <= 0.03 and abs(s.yaw_rate) <= 0.08:
                            if stable_since is None:
                                stable_since = self.clock()
                            if self.clock() - stable_since >= 0.3:
                                report["stationary_observed"] = True
                                break
                        else:
                            stable_since = None
                        self.sleep(0.01)
                    last = self.sample()
                    report.update(dx=last.x-first.x, dy=last.y-first.y,
                                  dyaw=math.atan2(math.sin(last.yaw-first.yaw), math.cos(last.yaw-first.yaw)))
                    if ack is not None and ack.done():
                        ack.result()
                        report["motion_ack"] = True
                    if not report["motion_ack"]:
                        report["errors"].append("motion ACK not received by end of observation")
                    if not report["stationary_observed"]:
                        report["errors"].append("stationary state not observed after stop")
            except BaseException as exc:
                report["errors"].append(f"stop/observation: {type(exc).__name__}: {exc}")
            finally:
                self._lock.release()
        return report
