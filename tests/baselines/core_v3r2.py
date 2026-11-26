"""G1 turn/walk state machine; no ROS, command acknowledgements or watchdog."""
import math
from collections import deque
from posnav_fmm.errors import PlanningRetry, GoalUnreachable
from g1_walk_guard import WalkGuard


def velocity_xyz(velocity):
    """Accept legacy (vx, wz) commands and full (vx, vy, wz) commands."""
    if len(velocity) == 2:
        return (velocity[0], 0., velocity[1])
    if len(velocity) == 3:
        return tuple(velocity)
    raise ValueError('Expected (vx, wz) or (vx, vy, wz)')


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Navigation:
    control_hz = 50.
    duration = .1
    pose_timeout = .3
    map_timeout = 1.
    speed = .5
    omega = 1.
    left_tol = .12
    right_tol = -.18
    epsilon = .150
    small_turn_angle = .4
    min_turn_angle = .03
    turn_settle = .75
    # Closed-loop turn stop (2026-11-21): the fixed settle is a lower bound; walk
    # only starts once the measured body yaw rate is small (or the extra bound
    # expires). Measured on the 619 run: the body still turned 27-55 deg/s for
    # ~1.2 s after the zero command while walk began at 0.76 s.
    turn_stop_yaw_rate = .15          # rad/s, approx 8.6 deg/s
    turn_stop_window = .30            # s of pose history used for the rate
    turn_stop_extra = 1.5             # s beyond turn_settle before giving up
    walk_settle = .50
    goal_tol = .50
    plan_retry_s = 1.
    recovery_speed = .5
    recovery_duration = 1.
    recovery_radius = .75
    recovery_near_radius = .30
    recovery_limit = 3
    recovery_failure_limit = 3

    def __init__(self, plan, clear, recorder=None, recovery=None, walk_guard=None):
        self.plan = plan
        self.clear = clear
        self.recorder = recorder
        self.recovery = recovery
        self.walk_guard = None if walk_guard is None else WalkGuard(walk_guard)
        self.map_revision = None
        self.recovery_events = deque()
        self.reset_recovery()

        self.pose = None
        self.pose_time = self.map_time = -math.inf
        self.yaw_samples = deque(maxlen=128)
        self.active = False
        self.state = 'idle'
        self.reason = 'Ready'
        self.paused = ''
        self.stg = None
        self.origin = self.direction = None

    def reset_recovery(self):
        self.recovery_count = 0
        self.planning_failures = 0
        self.failure_map_revision = None
        self.recovery_direction = None
        self.recovery_counts = None
        self.recovery_stop_map = None
        self.segment_walked = False
        self.segment_normal = False

    def feedback_stage(self):
        stage = self.state
        if stage.startswith('recovery_'):
            attempt = self.recovery_count + (stage == 'recovery_prepare')
            stage += ' (%d/%d)' % (attempt, self.recovery_limit)
        return stage + (': '+self.paused if self.paused else '')

    def planning_failed(self, reason, now, unreachable=False):
        """Count one failure per distinct valid task-map snapshot."""
        self.stg = None
        self.origin = self.direction = None
        if self.map_revision is not None and self.map_revision != self.failure_map_revision:
            self.planning_failures += 1
            self.failure_map_revision = self.map_revision
        self.reason = reason
        if self.recovery is not None and (unreachable or self.planning_failures >= self.recovery_failure_limit):
            info = self.recovery(self.pose)
            if info['near_count'] > 0:
                if self.recovery_count >= self.recovery_limit:
                    self.recovery_events.append(('recovery_exhausted', dict(attempts=self.recovery_count,
                        near_count=info['near_count'], sector_counts=info['sector_counts'], trigger=reason)))
                    self.stop('recovery_exhausted')
                    return (0., 0.)
                # Start on the next control cycle, after callbacks/freshness checks.
                self.recovery_trigger = reason
                self.state = 'recovery_prepare'
                self.reason = 'Preparing recovery (%d/%d)' % (self.recovery_count+1, self.recovery_limit)
                return (0., 0.)
        return self.wait('plan_wait', now, self.plan_retry_s)

    def finish_recovery_motion(self, now, reason):
        self.recovery_events.append(('recovery_motion_stopped', dict(attempt=self.recovery_count,
            direction=self.recovery_direction, reason=reason, pose=tuple(self.pose))))
        self.recovery_stop_map = self.map_revision
        self.reason = 'Recovery settling (%d/%d): %s' % (self.recovery_count, self.recovery_limit, reason)
        return self.wait('recovery_settle', now, self.walk_settle)

    def input_problem(self, now):
        if self.pose is None or now - self.pose_time >= self.pose_timeout:
            return 'Waiting for pose'
        if now - self.map_time >= self.map_timeout:
            return 'Waiting for map update'
        return ''

    def start(self, goal, now):
        if self.active:
            raise ValueError('Navigation is already running')
        problem = self.input_problem(now)
        if problem:
            raise ValueError(problem)
        self.goal = tuple(goal)
        self.active = True
        self.state = 'plan'
        self.reason = 'Navigation started'
        self.paused = ''
        self.stg = None
        self.origin = self.direction = None
        self.reset_recovery()
        if self.walk_guard is not None:
            self.walk_guard.reset()

    def stop(self, reason='Stopped'):
        if self.walk_guard is not None:
            self.walk_guard.finish_alignment(self.walk_guard.last_now)
        if self.state.startswith('recovery_'):
            self.recovery_events.append(('recovery_cancelled', dict(attempt=self.recovery_count,
                direction=self.recovery_direction, reason=reason)))
        if self.recorder is not None:
            self.recorder.abort(reason)
        self.active = False
        self.state = 'idle'
        self.reason = reason
        self.paused = ''

    def goal_reached(self):
        return math.hypot(self.goal[0]-self.pose[0], self.goal[1]-self.pose[1]) <= self.goal_tol

    def remaining(self):
        return sum((g-p)*u for g, p, u in zip(self.stg, self.pose, self.direction))

    def wait(self, state, now, seconds):
        self.state = state
        self.until = now + seconds
        return (0., 0.)

    def yaw_rate(self, window=None):
        """Measured yaw rate (rad/s) from recent poses; None when unknown."""
        window = self.turn_stop_window if window is None else window
        if len(self.yaw_samples) < 2:
            return None
        t1, y1 = self.yaw_samples[-1]
        for t0, y0 in reversed(self.yaw_samples):
            span = t1 - t0
            if span >= window:
                return wrap(y1-y0)/span if span > 1e-3 else None
        return None

    def turn_settled(self, now):
        """Fixed settle elapsed AND the body has effectively stopped turning."""
        if now < self.until:
            return False
        if self.walk_guard is not None:
            sample_gap = min(self.pose_timeout, self.walk_guard.config.stop_sample_gap_s)
            fresh_s = self.walk_guard.settle_elapsed(self.until, now, self.pose_timeout,
                                                     max_gap=sample_gap)
            if self.walk_guard.stopped_since(self.until-self.turn_settle, now,
                    self.turn_stop_window, self.turn_stop_yaw_rate, self.pose_timeout,
                    max_gap=sample_gap):
                return True
            if fresh_s >= self.turn_stop_extra-1e-9:
                self.walk_guard.event('walk_guard_stop_timeout', now, state=self.state,
                                      settle_fresh_s=fresh_s)
                self.stop('walk_guard_stop_timeout')
            return False
        rate = self.yaw_rate()
        if rate is None:
            return True                      # no measurement: keep original timing
        if abs(rate) <= self.turn_stop_yaw_rate:
            return True
        if now - self.until >= self.turn_stop_extra:
            if self.recorder is not None:
                self.recorder.anomaly('Turn settle bound reached at %.1f deg/s'
                                      % math.degrees(rate))
            return True
        return False

    def begin_walk(self, now):
        if self.walk_guard is not None:
            self.walk_guard.finish_alignment(now)
        if self.goal_reached():
            self.segment_normal = False
            if self.recorder is not None:
                self.recorder.anomaly('Final goal radius triggered STG stop')
            return self.wait('walk_settle', now, self.walk_settle)
        self.origin = self.pose[:2]
        delta = tuple(g-p for g, p in zip(self.stg, self.origin))
        distance = math.hypot(*delta)
        if distance <= self.epsilon:
            return self.wait('walk_settle', now, self.walk_settle)
        if self.walk_guard is not None and self.walk_guard.needs_alignment(self.pose, self.stg):
            return self.guard_align(now, 'turn_heading_recheck')
        self.direction = tuple(d/distance for d in delta)
        self.state = 'walk'
        if self.walk_guard is not None:
            self.walk_guard.pause()
        return self.walk(now)

    def guard_align(self, now, reason):
        guard = self.walk_guard
        guard.pause()
        self.segment_normal = False
        if guard.align_count >= guard.config.align_attempts:
            guard.event('walk_guard_align_limit', now, reason=reason)
            self.stop('walk_guard_align_limit')
            return (0., 0.)
        guard.align_count += 1
        guard.begin_alignment(now)
        guard.align_started = now
        guard.event('walk_guard_realign', now, reason=reason, attempt=guard.align_count,
                    pose=tuple(self.pose), stg=self.stg)
        if self.recorder is not None:
            self.recorder.anomaly('Walk guard realignment: '+reason)
        self.reason = 'Walk guard: stopping before STG realignment'
        # Stop first; do not reverse a moving body's angular velocity immediately.
        return self.wait('guard_before_align', now, self.turn_settle)

    def guard_tick(self, now):
        guard = self.walk_guard
        if self.state in ('guard_before_align', 'guard_settle'):
            if not self.turn_settled(now):
                return (0., 0.)
            if self.state == 'guard_settle':
                if not guard.needs_alignment(self.pose, self.stg):
                    return self.begin_walk(now)
                return self.guard_align(now, 'post_align_heading_recheck')
            self.state = 'guard_align'
            guard.align_started = now
            self.reason = 'Walk guard: aligning to current STG'
            return (0., 0.)
        if self.state == 'guard_align':
            if now-guard.align_started >= guard.config.align_timeout_s:
                guard.event('walk_guard_align_timeout', now)
                self.stop('walk_guard_align_timeout')
                return (0., 0.)
            distance, error = guard.error(self.pose, self.stg)
            if distance <= self.epsilon or abs(math.degrees(error)) <= guard.config.align_deg:
                return self.wait('guard_settle', now, self.turn_settle)
            rate = min(max(guard.config.align_kp*abs(error), guard.config.align_min_wz),
                       guard.config.align_max_wz)
            return (0., math.copysign(rate, error))
        if self.state == 'guard_replan_settle':
            if now >= self.until:
                self.state = 'plan'
            return (0., 0.)
        raise RuntimeError('Unknown walk guard state: '+self.state)

    def walk(self, now):
        if self.goal_reached() or self.remaining() <= self.epsilon:
            if self.goal_reached():
                self.segment_normal = False
            if self.recorder is not None and self.goal_reached():
                self.recorder.anomaly('Final goal radius triggered STG stop')
            return self.wait('walk_settle', now, self.walk_settle)
        if not self.clear(self.pose[:2], self.stg, 1.):
            if self.walk_guard is not None:
                self.walk_guard.pause()
            self.segment_normal = False
            if self.recorder is not None:
                self.recorder.anomaly('Obstacle wait')
            return self.wait('obstacle', now, 2.)
        if self.walk_guard is not None:
            reason = self.walk_guard.check_walk(self.pose, self.stg, self.remaining(), self.pose_time)
            if reason == 'heading':
                return self.guard_align(now, 'persistent_stg_heading')
            if reason == 'progress':
                guard = self.walk_guard
                guard.replan_count += 1
                guard.event('walk_guard_no_progress', now, attempt=guard.replan_count,
                            **guard.diagnostics)
                if self.recorder is not None:
                    self.recorder.abort('Walk guard: no progress')
                guard.pause()
                self.segment_normal = False
                if guard.replan_count >= guard.config.progress_replans:
                    self.stop('walk_guard_no_progress_limit')
                    return (0., 0.)
                self.reason = 'Walk guard: no progress; settling before replanning'
                return self.wait('guard_replan_settle', now, self.walk_settle)
        self.segment_walked = True
        return (self.speed, 0.)

    def tick(self, now):
        if not self.active:
            return None
        if self.walk_guard is not None:
            self.walk_guard.last_now = now
        if self.walk_guard is not None and self.pose is not None:
            self.walk_guard.observe(self.pose_time, self.pose, now, self.pose_timeout)
        if self.pose is not None:
            if not self.yaw_samples or self.pose[2] != self.yaw_samples[-1][1]:
                self.yaw_samples.append((now, self.pose[2]))
        self.paused = self.input_problem(now)
        if self.paused:
            if self.walk_guard is not None:
                self.walk_guard.pause()
            self.segment_normal = False
            if self.recorder is not None:
                self.recorder.anomaly(self.paused)
            if self.state in ('recovery_forward', 'recovery_left', 'recovery_backward', 'recovery_right'):
                return self.finish_recovery_motion(now, self.paused)
            if self.state == 'recovery_prepare':
                self.reason = 'Recovery preparation interrupted: '+self.paused
                return self.wait('plan_wait', now, self.plan_retry_s)
            return (0., 0.)
        if self.walk_guard is not None and self.state.startswith('guard_'):
            return self.guard_tick(now)
        if self.state == 'recovery_prepare':
            info = self.recovery(self.pose)
            if info['near_count'] == 0:
                self.state = 'plan'
                self.reason = 'Near-field occupancy cleared; replanning'
                return (0., 0.)
            directions = ('forward', 'left', 'backward', 'right')
            self.recovery_counts = list(info['sector_counts'])
            index = min(range(4), key=lambda i: self.recovery_counts[i])
            self.recovery_direction = directions[index]
            v = self.recovery_speed
            self.recovery_velocity = ((v, 0., 0.), (0., v, 0.), (-v, 0., 0.), (0., -v, 0.))[index]
            self.recovery_count += 1
            failures = self.planning_failures
            self.planning_failures = 0
            self.failure_map_revision = None
            self.state = 'recovery_'+self.recovery_direction
            self.reason = 'Recovery %s (%d/%d)' % (self.recovery_direction, self.recovery_count, self.recovery_limit)
            self.until = now + self.recovery_duration
            self.recovery_events.append(('recovery_started', dict(attempt=self.recovery_count,
                direction=self.recovery_direction, sector_counts=self.recovery_counts,
                trigger=self.recovery_trigger, planning_failures=failures,
                near_count=info['near_count'], radius_m=self.recovery_radius,
                velocity=self.recovery_velocity, duration_s=self.recovery_duration, pose=tuple(self.pose))))
            return self.recovery_velocity
        if self.state in ('recovery_forward', 'recovery_left', 'recovery_backward', 'recovery_right'):
            if now >= self.until:
                return self.finish_recovery_motion(now, 'Timed motion complete')
            return self.recovery_velocity
        if self.state == 'recovery_settle':
            if now < self.until:
                return (0., 0.)
            if self.map_revision == self.recovery_stop_map:
                self.reason = 'Waiting for new task map after recovery (%d/%d)' % (self.recovery_count, self.recovery_limit)
                return (0., 0.)
            self.state = 'plan'
            self.reason = 'Recovery complete; replanning'
        if self.state == 'plan_wait':
            if now < self.until:
                return (0., 0.)
            self.state = 'plan'
        if self.state == 'plan':
            if self.goal_reached():
                self.stop('Arrived within 0.50 m')
                return (0., 0.)
            try:
                stg = self.plan(self.pose[:2], self.goal)
            except PlanningRetry as exc:
                return self.planning_failed(str(exc), now, isinstance(exc, GoalUnreachable))
            if stg is None:
                self.stop('No valid STG; navigation stopped')
                return (0., 0.)
            self.stg = tuple(stg)
            if self.walk_guard is not None:
                self.walk_guard.new_segment()
            self.segment_walked = False
            self.segment_normal = True
            if self.recorder is not None:
                self.recorder.begin(self.pose, self.stg)
            self.reason = 'Navigation running'
            if not self.clear(self.pose[:2], self.stg, math.inf):
                self.reason = 'STG segment is blocked'
                if self.recorder is not None:
                    self.recorder.abort('STG segment is blocked; replanning')
                return self.planning_failed(self.reason, now)
            self.planning_failures = 0
            self.failure_map_revision = None
            error = wrap(math.atan2(self.stg[1]-self.pose[1], self.stg[0]-self.pose[0])-self.pose[2])
            self.previous_error = self.turn_error = error
            self.turn_sign = 1 if error > self.left_tol else (-1 if error < self.right_tol else 0)
            if not self.turn_sign:
                if abs(error) <= self.min_turn_angle:
                    # No settling delay without a turn. Let callbacks and input
                    # freshness checks run after potentially slow planning.
                    self.state = 'walk_start'
                    return (0., 0.)
                self.turn_sign = 1 if error > 0. else -1
                self.previous_yaw = self.pose[2]
                self.turn_progress = 0.
                self.state = 'turn_out'
                return (0., 0.)
            self.state = 'turn'
            # Planning may take longer than one control cycle. Start moving on
            # the next cycle, after callbacks and freshness checks have run.
            return (0., 0.)
        if self.state == 'turn_out':
            # Measure the deliberate excursion, including feedback across +/-pi.
            # Unlike final alignment, this leg stops after 0.4 rad of measured
            # rotation; braking drift is handled by recomputing the return leg.
            self.turn_progress += self.turn_sign * wrap(self.pose[2]-self.previous_yaw)
            self.previous_yaw = self.pose[2]
            if self.turn_progress >= self.small_turn_angle:
                return self.wait('turn_out_settle', now, self.turn_settle)
            return (0., self.turn_sign*self.omega)
        if self.state == 'turn_out_settle':
            ready = self.turn_settled(now) if self.walk_guard is not None else now >= self.until
            if not ready:
                return (0., 0.)
            error = wrap(math.atan2(self.stg[1]-self.pose[1], self.stg[0]-self.pose[0])-self.pose[2])
            self.previous_error = self.turn_error = error
            self.turn_sign = 1 if error > 0. else -1
            # Use normal signed stop advances for the final alignment. Do not
            # recursively start another excursion if the latest error is small.
            self.state = 'turn'
            return (0., 0.)
        if self.state == 'turn':
            error = wrap(math.atan2(self.stg[1]-self.pose[1], self.stg[0]-self.pose[0])-self.pose[2])
            self.turn_error += wrap(error-self.previous_error)
            self.previous_error = error
            done = (self.turn_error <= self.left_tol if self.turn_sign > 0
                    else self.turn_error >= self.right_tol)
            if done:
                return self.wait('turn_settle', now, self.turn_settle)
            return (0., self.turn_sign*self.omega)
        if self.state == 'walk_start' or (self.state == 'turn_settle' and self.turn_settled(now)):
            if self.recorder is not None:
                self.recorder.turn_settled(self.pose)
            return self.begin_walk(now)
        if self.state == 'walk':
            return self.walk(now)
        if self.state == 'obstacle':
            self.segment_normal = False
            if self.goal_reached() or self.remaining() <= self.epsilon:
                if self.recorder is not None and self.goal_reached():
                    self.recorder.anomaly('Final goal radius triggered STG stop')
                return self.wait('walk_settle', now, self.walk_settle)
            if now < self.until:
                return (0., 0.)
            if self.clear(self.pose[:2], self.stg, 1.):
                if self.walk_guard is not None:
                    return self.begin_walk(now)
                self.state = 'walk'
                self.segment_walked = True
                return (self.speed, 0.)
            if self.recorder is not None:
                self.recorder.abort('Obstacle persisted; replanning')
            self.state = 'plan'
            return (0., 0.)
        if self.state == 'walk_settle' and now >= self.until:
            if self.recorder is not None:
                self.recorder.walk_settled(self.pose)
            if self.segment_normal and self.segment_walked:
                if self.recovery_count:
                    self.recovery_events.append(('recovery_counter_reset', dict(previous_attempts=self.recovery_count)))
                self.recovery_count = 0
            if self.goal_reached():
                self.stop('Arrived within 0.50 m')
            else:
                self.state = 'plan'
        return (0., 0.)
