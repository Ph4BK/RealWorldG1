"""Formal G1 pulse interface. Calls execute motion; official C++ handles DDS.

Calibration is opt-in and uses the PosNav planar pose, not external ground truth.
Do not queue the next pulse until the preceding physical action has settled.
"""
import json
import math
from pathlib import Path
import subprocess
import threading


class G1MotionError(RuntimeError):
    def __init__(self, report):
        self.report = report
        super().__init__(str(report))


def validate_pulse(vx, yaw_rate, duration, vy=0.0):
    if not all(math.isfinite(v) for v in (vx, vy, yaw_rate, duration)):
        raise ValueError('finite parameters required')
    if duration <= 0 or ((vx or vy) and yaw_rate):
        raise ValueError('translation and rotation must be separate; duration must be positive')


class G1MotionExecutor:
    def __init__(self, *, runner=None):
        self.runner = Path(runner) if runner else Path(__file__).resolve().parents[1] / 'tools/run_g1_motion.sh'
        self._lock = threading.Lock()

    def move(self, vx=0.2, duration=1.0, *, vy=0.0):
        return self.pulse(vx=vx, vy=vy, duration=duration)

    def turn(self, yaw_rate=0.15, duration=1.0):
        """Positive yaw is left turn; actual rotation remains to be calibrated."""
        return self.pulse(yaw_rate=yaw_rate, duration=duration)

    def pulse(self, *, vx=0.0, vy=0.0, yaw_rate=0.0, duration=1.0, calibrate=False):
        validate_pulse(vx, yaw_rate, duration, vy)
        if calibrate and (vy != 0 or vx < 0 or vx == yaw_rate == 0):
            raise ValueError('calibration requires forward, left turn or right turn')
        args = [str(vx), str(vy), str(yaw_rate), str(duration)]
        if calibrate:
            args.append('--calibrate')
        return self._run(args, 'command_completed')

    def read_odom(self):
        """Read one fresh odometry sample; never sends sport or motion requests."""
        return self._run(['--odom'], 'odom_received')

    def _run(self, args, success_key):
        if not self._lock.acquire(blocking=False):
            raise RuntimeError('another pulse is active')
        try:
            result = subprocess.run(
                ['bash', str(self.runner), *args],
                capture_output=True, text=True,
            )
            lines = [line[7:] for line in result.stdout.splitlines() if line.startswith('RESULT ')]
            if not lines:
                report = {'returncode': result.returncode, 'stderr': result.stderr, 'stdout': result.stdout}
                if "The command 'docker' could not be found in this WSL" in result.stdout + result.stderr:
                    report['error'] = ('本机 WSL 的 Docker 不可用，运控程序未启动。'
                                       '此入口只在本机执行，不会自动 SSH 到 G1；'
                                       '请登录 G1 PC2（unitree@192.168.123.164），在机载部署目录运行。')
                raise G1MotionError(report)
            report = json.loads(lines[-1])
            if result.returncode != 0 or not report.get(success_key):
                raise G1MotionError(report)
            return report
        finally:
            self._lock.release()
