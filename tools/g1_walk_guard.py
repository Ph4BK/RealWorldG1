"""Opt-in STG walk guards. Pure state/diagnostics; never publishes commands.

15/5 degrees and 0.15 m are Go2-scale starting points, NOT G1 calibration.
0.3 s persistence, 8 s progress, 5 cm progress and retry bounds are candidates.
"""
from collections import deque
from dataclasses import dataclass, asdict
import math


@dataclass(frozen=True)
class WalkGuardConfig:
    reenter_deg: float = 15.       # Go2 initial value; unvalidated on G1.
    align_deg: float = 5.          # Go2 initial value; unvalidated on G1.
    far_m: float = .15             # Go2 distance gate; unvalidated on G1.
    persistence_s: float = .30     # Proposed 11-24 sensor-time persistence, unvalidated.
    progress_s: float = 8.         # Proposed 11-24 progress deadline, unvalidated.
    progress_m: float = .05        # Candidate noise floor, not a G1 accuracy claim.
    align_timeout_s: float = 6.    # Candidate bounded recovery, unvalidated.
    align_attempts: int = 3        # Candidate per-STG attempt bound, unvalidated.
    progress_replans: int = 3      # Candidate per-goal stall bound, unvalidated.
    align_kp: float = 1.5          # Existing PosNav proportional gain, reused for trial.
    align_min_wz: float = .15      # Existing PosNav minimum, not a measured motor limit.
    align_max_wz: float = .5       # Candidate cap below legacy 1 rad/s, unvalidated.
    stop_sample_gap_s: float = .15 # Candidate stop-evidence continuity; recorded source is ~10Hz.

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid walk guard parameter: '+name)
        if not self.align_deg < self.reenter_deg < 180:
            raise ValueError('Walk guard requires 0 < align < reenter < 180 degrees')
        if self.align_min_wz > self.align_max_wz:
            raise ValueError('Invalid walk guard yaw-rate bounds')
        for name in ('align_attempts', 'progress_replans'):
            if not isinstance(getattr(self, name), int):
                raise ValueError('Walk guard counts must be integers')


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class FreshSampleWindow:
    """Shared source-time continuity and settle budget for yaw and XY evidence."""
    def __init__(self):
        self.samples = deque(maxlen=128)
        self.last_identity = None
        self.reset_settle_clock()

    def pause_samples(self):
        self.samples.clear()
        self.settle_last_sample = None

    def observe_value(self, sample_time, value, now, max_age, identity=None):
        if (not math.isfinite(sample_time) or sample_time > now+1e-6 or
                now-sample_time >= max_age):
            return False
        if identity is not None and identity == self.last_identity:
            return False
        if self.samples and sample_time < self.samples[-1][0]:
            self.pause_samples()
        if not self.samples or sample_time > self.samples[-1][0]:
            self.samples.append((sample_time, value))
            self.last_identity = identity
            return True
        return False

    def reset_settle_clock(self):
        self.settle_clock_start = self.settle_last_sample = None
        self.settle_fresh_s = 0.

    def settle_elapsed(self, start, now, max_age, max_gap=None):
        """Count distinct bounded source intervals; pauses and repeated ticks freeze."""
        max_gap = max_age if max_gap is None else max_gap
        if self.settle_clock_start != start:
            self.reset_settle_clock()
            self.settle_clock_start = start
        if not self.samples:
            return self.settle_fresh_s
        latest = self.samples[-1][0]
        if latest > now+1e-6 or now-latest >= max_age:
            self.settle_last_sample = None
            return self.settle_fresh_s
        previous = self.settle_last_sample
        if previous is not None and latest > previous:
            if latest-previous < max_gap:
                self.settle_fresh_s += max(0., latest-max(previous, start))
        self.settle_last_sample = latest
        return self.settle_fresh_s

    def stable_suffix(self, start, now, max_age, max_gap, pair_ok):
        samples = [(t, value) for t, value in self.samples if t >= start-1e-9]
        if not samples or samples[-1][0] > now+1e-6 or now-samples[-1][0] >= max_age:
            return []
        first = len(samples)-1
        for i in range(len(samples)-2, -1, -1):
            a, va = samples[i]; b, vb = samples[i+1]
            dt = b-a
            if dt <= 0 or dt >= max_gap or not pair_ok(va, vb, dt):
                break
            first = i
        return samples[first:]

    def stopped_yaw_since(self, start, now, hold_s, max_rate, max_age, max_gap=None):
        max_gap = max_age if max_gap is None else max_gap
        samples = self.stable_suffix(start, now, max_age, max_gap,
            lambda a, b, dt: abs(wrap(b-a)/dt) <= max_rate)
        return len(samples) >= 2 and samples[-1][0]-samples[0][0] >= hold_s-1e-9


class WalkGuard(FreshSampleWindow):
    def __init__(self, config=None):
        self.config = config or WalkGuardConfig()
        self.events = deque()
        self.reset()

    def reset(self):
        self.samples = deque(maxlen=128)
        self.last_identity = None
        self.align_count = self.replan_count = 0
        self.align_started = None
        self.realign_total_count = 0
        self.realign_total_s = 0.
        self.realign_started = self.last_now = None
        self.reset_settle_clock()
        self.pause()
        self.diagnostics = {}

    def pause(self):
        self.bad_since = self.progress_time = self.best_remaining = None
        # Freeze the settle budget across input pauses; never bridge the gap.
        self.settle_last_sample = None

    def new_segment(self):
        self.align_count = 0
        self.reset_settle_clock()
        self.pause()

    def begin_alignment(self, now):
        self.finish_alignment(now)
        self.last_now = self.realign_started = now
        self.realign_total_count += 1

    def finish_alignment(self, now):
        if self.realign_started is not None and now is not None:
            self.realign_total_s += max(0., now-self.realign_started)
            self.realign_started = None

    def status(self, now):
        # Whole recovery attempt wall time, including before/after settle and pauses.
        active = 0. if self.realign_started is None else max(0., now-self.realign_started)
        return dict(self.diagnostics, align_count=self.align_count,
            realign_total_count=self.realign_total_count,
            realign_total_s=self.realign_total_s+active,
            realign_active_s=active, settle_fresh_s=self.settle_fresh_s)

    def observe(self, sample_time, pose, now, max_age, identity=None):
        if identity is not None:
            if identity==self.last_identity and self.samples and self.samples[-1][1]!=pose[2]:
                self.pause_samples()  # Same raw odometry with a new reference cannot prove rest.
                return
            self.observe_value(sample_time,pose[2],now,max_age,identity=identity)
            return
        if (not math.isfinite(sample_time) or not all(map(math.isfinite, pose)) or
                sample_time > now+1e-6 or now-sample_time >= max_age):
            return
        if self.samples and sample_time < self.samples[-1][0]:
            self.samples.clear()
            self.pause()
        # A repeated control tick/pose is not an independent sensor sample.
        if not self.samples or sample_time > self.samples[-1][0]:
            self.samples.append((sample_time, pose[2]))

    def stopped_since(self, start, now, hold_s, max_rate, max_age, max_gap=None):
        return self.stopped_yaw_since(start, now, hold_s, max_rate, max_age, max_gap)

    def error(self, pose, stg):
        dx, dy = stg[0]-pose[0], stg[1]-pose[1]
        return math.hypot(dx, dy), wrap(math.atan2(dy, dx)-pose[2])

    def needs_alignment(self, pose, stg, ignore_distance=False):
        distance, error = self.error(pose, stg)
        return (ignore_distance or distance > self.config.far_m) and abs(math.degrees(error)) > self.config.align_deg

    def event(self, kind, now, **data):
        self.events.append((kind, dict(time_monotonic=now, **data)))

    def check_walk(self, pose, stg, remaining, sample_time, allow_heading=True, ignore_distance=False):
        distance, error = self.error(pose, stg)
        bad = allow_heading and (ignore_distance or distance > self.config.far_m) and abs(math.degrees(error)) > self.config.reenter_deg
        if bad:
            if self.bad_since is None:
                self.bad_since = sample_time
        else:
            self.bad_since = None
        if self.progress_time is None:
            self.progress_time, self.best_remaining = sample_time, remaining
        elif remaining <= self.best_remaining-self.config.progress_m:
            self.progress_time, self.best_remaining = sample_time, remaining
        self.diagnostics = dict(distance_to_stg_m=distance, heading_error_deg=math.degrees(error),
            bad_heading_s=0. if self.bad_since is None else sample_time-self.bad_since,
            no_progress_s=sample_time-self.progress_time, best_remaining_m=self.best_remaining)
        if self.bad_since is not None and sample_time-self.bad_since >= self.config.persistence_s-1e-9:
            return 'heading'
        if sample_time-self.progress_time >= self.config.progress_s-1e-9:
            return 'progress'
        return ''
