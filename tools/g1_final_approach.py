"""PosNav orchestration: STGs to pre, point approach, then one yaw alignment.

The underlying Navigation class remains the separately tested STG executor.
Requested pos_tol remains record-only. Optional stop gates are controller parameters.
"""
from collections import deque
from dataclasses import dataclass, asdict
import math

from g1_navigation_core import Navigation, wrap
from g1_walk_guard import FreshSampleWindow


@dataclass(frozen=True)
class FineApproachConfig:
    fine_stop_layered: bool = False
    fine_stop_radius: float = .08       # 11-25 review initial value; NOT G1 calibration.
    fine_stop_projected: float = .04    # Same source; along actual entry->goal, not goal yaw.
    fine_goal_behind: bool = False
    fine_pre_fallback: bool = False
    fine_align_wz: float = 1.           # Legacy cap; .2 is an uncalibrated field candidate.
    position_settle_fresh: bool = False
    position_settle_step: float = .010  # Stationary 11-24 control_pose p99 7.6..8.0mm.
    position_settle_span: float = .020  # Four-sample diameter max 18.8mm at that station.
    position_settle_samples: int = 4    # Candidate ~10Hz/.3s window, not braking calibration.
    fine_entry_align: bool = False
    fine_entry_align_deg: float = 30.   # v3 review candidate, not measured activation threshold.
    fine_entry_done_deg: float = 10.    # v3 review candidate; stop evidence also required.
    fine_entry_align_timeout: float = 6.  # Effective source time, shared across a goal's entries.
    fine_entry_attempts: int = 3        # Bounded entry/re-alignment attempts per goal, candidate.
    fine_lateral_guard: bool = False
    fine_lateral_max: float = .10      # Review initial value; historical blind-window samples sparse.
    fine_passage_waits: int = 3        # Review candidate: 3 x legacy 2s waits, not field-calibrated.

    def __post_init__(self):
        for name, value in asdict(self).items():
            if name in ('fine_stop_layered', 'fine_goal_behind', 'fine_pre_fallback',
                        'position_settle_fresh', 'fine_entry_align', 'fine_lateral_guard'):
                if not isinstance(value, bool):
                    raise ValueError('Expected boolean: '+name)
            elif isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid fine parameter: '+name)
        if (not 0 < self.fine_stop_projected <= self.fine_stop_radius <= .15 or
                self.fine_align_wz > 1. or self.position_settle_step > self.position_settle_span or
                not isinstance(self.position_settle_samples, int) or
                not 2 <= self.position_settle_samples <= 32):
            raise ValueError('Invalid fine stopping/settling bounds')
        if (not 0 < self.fine_entry_done_deg < self.fine_entry_align_deg < 180 or
                not isinstance(self.fine_entry_attempts, int) or not 1 <= self.fine_entry_attempts <= 32 or
                not isinstance(self.fine_passage_waits, int) or not 1 <= self.fine_passage_waits <= 32 or
                self.fine_lateral_max >= .15):
            raise ValueError('Invalid fine entry/lateral bounds')


def fine_metrics(goal, direction, pose):
    """All four error measures from ONE map-frame control sample."""
    dx, dy = goal[0]-pose[0], goal[1]-pose[1]
    remaining = dx*direction[0]+dy*direction[1] if direction is not None else None
    lateral = -dx*direction[1]+dy*direction[0] if direction is not None else None
    return dict(distance_m=math.hypot(dx, dy), projected_remaining_m=remaining,
        goal_yaw_remaining_m=dx*math.cos(goal[2])+dy*math.sin(goal[2]),
        lateral_m=lateral, lateral_abs_m=None if lateral is None else abs(lateral))


class PosNav(Navigation):
    pre_distance = .60
    direct_stg_range = .50
    fine_speed = .25
    coast = 0.
    fine_no_turn_distance = .15
    drive_align = .10
    yaw_stop_tol = .10
    k_ang = 1.5
    wz_min = .15
    wz_max = 1.
    fine_obstacle_wait = 2.
    completion_reason = 'Final approach completed'
    projected_stop_range = .15    # 11-25 review distance gate; uncalibrated G1 candidate.
    passed_goal_distance = .05    # Review overshoot bound; do not reverse to chase the goal.
    behind_min_distance = .05     # Avoid unstable target bearing very close to goal.
    behind_max_distance = .30     # Review near-end gate, NOT obstacle clearance.
    behind_angle = math.radians(120.)  # Review candidate; not a measured G1 body envelope.
    position_stop_hold = .30      # Existing turn window reused; needs post-walk field trials.
    position_stop_gap = .15       # Existing ~10Hz source continuity candidate.
    position_stop_extra = 1.5     # Valid source-time budget beyond the legacy .5s wait.

    def __init__(self, plan, clear, route, recorder=None, recovery=None, walk_guard=None, fine=None):
        self.fine = fine or FineApproachConfig()
        self.position_samples = FreshSampleWindow()
        self.position_stop_time = None
        self.pose_source_stamp_ns = None
        self.fine_now = None
        self._reset_entry()
        self.plan_callback = plan
        self.route_callback = route
        self.final_events = deque()
        self.final_goal = None
        self.pre = None
        self.terminal_stg = False
        self.position_origin = self.position_direction = None
        self.position_settle_pose = None
        self.final_pose = None
        self.final_error_m = self.final_yaw_error = None
        super().__init__(self._plan_stg, clear, recorder, recovery, walk_guard)

    def start(self, goal, now):
        if self.active:
            raise ValueError('Navigation is already running')
        if len(goal) != 3 or not all(math.isfinite(v) for v in goal):
            raise ValueError('Expected finite final (x, y, yaw)')
        problem = self.input_problem(now)
        if problem:
            raise ValueError(problem)
        route = self.route_callback(tuple(goal))
        target = route['pre'] if route['pre'] is not None else goal[:2]
        super().start(target, now)
        self.final_goal = tuple(goal)
        self.pre = tuple(route['pre']) if route['pre'] is not None else None
        self.terminal_stg = False
        self.position_origin = self.position_direction = None
        self.position_settle_pose = self.final_pose = None
        self.final_error_m = self.final_yaw_error = None
        self.position_samples = FreshSampleWindow()
        self.position_stop_time = None
        self.fine_now = now
        self._reset_entry()
        self.final_events.append(('navigation_route_selected', dict(final_goal=self.final_goal,
            pre=self.pre, coarse_target=self.goal, reason=route['reason'])))
        self._route_events(route)
        # No need to leave an already reached goal to visit its pre point.
        if (self.final_distance() <= self.coast or
                self.fine.fine_stop_layered and self.final_distance() <= self.fine.fine_stop_radius or
                self.fine.fine_goal_behind and self.final_distance() <= self.behind_min_distance):
            self._enter_fine('Already within fine stopping distance')

    def goal_reached(self):
        # The old 0.50 m coarse success gate is never used by full PosNav.
        return False

    def final_distance(self):
        return math.hypot(self.final_goal[0]-self.pose[0], self.final_goal[1]-self.pose[1])

    def position_remaining(self):
        """Signed distance to the goal plane along the fixed approach direction."""
        return sum((self.final_goal[k]-self.pose[k])*self.position_direction[k] for k in (0, 1))

    def evidence(self, **data):
        return dict(pose=tuple(self.pose), source_stamp_ns=self.pose_source_stamp_ns,
            pose_source='latest_fastlio_control_pose', sample_time_monotonic=self.pose_time,
            control_time_monotonic=self.fine_now, final_goal=self.final_goal,
            position_direction=self.position_direction,
            **fine_metrics(self.final_goal, self.position_direction, self.pose), **data)

    def _route_events(self, route):
        for attempt in route.get('attempts', ()):
            self.final_events.append(('fine_pre_candidate', dict(attempt)))

    def _position_stop(self, now, stop_basis):
        self.position_stop_time = now
        self.position_samples = FreshSampleWindow()
        self.final_events.append(('fine_position_stop', self.evidence(stop_basis=stop_basis)))
        self.reason = 'Final position stop; settling'
        return self.wait('fine_position_settle', now, self.walk_settle)

    def _position_settled(self, now):
        samples = self.position_samples
        cfg = self.fine
        samples.observe_value(self.pose_time, self.pose[:2], now, self.pose_timeout,
                              identity=self.pose_source_stamp_ns)
        if samples.samples and samples.samples[-1][1] != self.pose[:2]:
            # A map-reference revision can move the same raw odometry sample.
            # It is not an independent observation of a stationary body.
            samples.pause_samples()
            return False
        elapsed = samples.settle_elapsed(self.until, now, self.pose_timeout, self.position_stop_gap)
        suffix = samples.stable_suffix(self.position_stop_time, now, self.pose_timeout,
            self.position_stop_gap, lambda a, b, dt: math.dist(a, b) <= cfg.position_settle_step)
        # Bound the whole recent window too: small increments alone accept a slow drift.
        recent = []
        for i in range(len(suffix)-cfg.position_settle_samples, -1, -1):
            if suffix[-1][0]-suffix[i][0] >= self.position_stop_hold-1e-9:
                recent = suffix[i:]
                break
        stable = (bool(recent) and
            max(math.dist(a[1], b[1]) for a in recent for b in recent) <= cfg.position_settle_span)
        if now >= self.until and stable:
            return True
        if now >= self.until and elapsed >= self.position_stop_extra-1e-9:
            self.final_events.append(('fine_position_settle_timeout', self.evidence(
                settle_fresh_s=elapsed, independent_samples=len(suffix))))
            self.stop('fine_position_settle_timeout')
        return False

    def turn_rate(self, error):
        if error == 0.:
            return 0.
        return math.copysign(min(max(self.k_ang*abs(error), self.wz_min), self.wz_max), error)

    def _reset_entry(self):
        self.entry_samples = FreshSampleWindow()
        self.entry_attempts = 0
        self.entry_clock_start = self.entry_stop_time = None
        self.entry_origin = self.entry_direction = None
        self.entry_reason = ''
        self.entry_started = None
        self.entry_total_wall_s = 0.
        self.passage_waits = 0

    def entry_status(self):
        active_s = (max(0., self.fine_now-self.entry_started)
                    if self.entry_started is not None and self.fine_now is not None else 0.)
        return dict(attempts=self.entry_attempts, fresh_s=self.entry_samples.settle_fresh_s,
                    total_wall_s=self.entry_total_wall_s+active_s,
                    aligning=self.state.startswith('fine_entry_'))

    def target_alpha(self):
        return wrap(math.atan2(self.final_goal[1]-self.pose[1], self.final_goal[0]-self.pose[0])-self.pose[2])

    def _entry_evidence(self, **data):
        row = self.evidence()
        row['previous_position_direction'] = row['position_direction']
        row.update(fine_metrics(self.final_goal, self.entry_direction, self.pose))
        row.update(entry_origin=self.entry_origin, entry_direction=self.entry_direction,
                   position_direction=self.entry_direction, error_reference='provisional_entry_ray',
                   alpha_deg=math.degrees(self.target_alpha()), **self.entry_status())
        row.update(data)
        return row

    def _close_entry(self):
        if self.entry_started is not None:
            self.entry_total_wall_s += max(0., self.fine_now-self.entry_started)
            self.entry_started = None
        self.entry_samples.pause_samples()

    def _entry_failed(self, detail):
        self.final_events.append(('fine_entry_align_timeout', self._entry_evidence(detail=detail)))
        self.stop('fine_entry_align_timeout')
        return (0., 0., 0.)

    def _entry_attempt(self):
        if self.entry_attempts >= self.fine.fine_entry_attempts:
            return self._entry_failed('attempt_limit')
        self.entry_attempts += 1
        self.state = 'fine_entry_align'
        self.reason = 'Aligning toward final position before fine approach'
        self.entry_stop_time = None
        self.final_events.append(('fine_entry_align_started', self._entry_evidence(
            minimum_turn_s=max(0., abs(self.target_alpha())-math.radians(self.fine.fine_entry_done_deg)) /
                           self.wz_max)))
        return (0., 0., 0.)

    def _entry_tick(self, now):
        # Every rotation/wait tick uses the original inflated passage check.
        # A blocked passage never triggers a new pre point or a coarse replan.
        if not self._goal_clear():
            if self.state != 'fine_entry_obstacle':
                self.final_events.append(('fine_entry_obstacle', self._entry_evidence()))
            self.entry_samples.pause_samples()
            return self._passage_blocked(now, 'fine_entry_obstacle')
        if self.state == 'fine_entry_obstacle':
            self.passage_waits = 0
            self.state = 'fine_entry_align'
            self.reason = 'Final entry passage clear; alignment recheck'
            self.entry_samples.pause_samples()
            return (0., 0., 0.)
        samples = self.entry_samples
        samples.observe_value(self.pose_time, self.pose[2], now, self.pose_timeout,
                              identity=self.pose_source_stamp_ns)
        # A reference update of the same raw frame is not fresh yaw evidence.
        if samples.samples and samples.samples[-1][1] != self.pose[2]:
            samples.pause_samples()
            return (0., 0., 0.)
        # Active alignment budget follows valid control input (.3s). The stricter
        # .15s continuity belongs to STOP evidence, not active-motion timing:
        # otherwise a valid 5Hz source would never spend the 6s alignment budget.
        elapsed = samples.settle_elapsed(self.entry_clock_start, now, self.pose_timeout, self.pose_timeout)
        if elapsed >= self.fine.fine_entry_align_timeout-1e-9:
            return self._entry_failed('effective_time_limit')
        alpha = self.target_alpha()
        if self.state == 'fine_entry_align':
            if (self._entry_stop_already_due() or
                    abs(math.degrees(alpha)) <= self.fine.fine_entry_done_deg+1e-9):
                self.state = 'fine_entry_settle'
                self.reason = 'Final entry alignment stop; confirming rest'
                self.entry_stop_time = now
                self.final_events.append(('fine_entry_align_stop', self._entry_evidence()))
                return (0., 0., 0.)
            rate = self.turn_rate(alpha)
            # Entry alignment is a large-angle coarse turn: it uses the legacy
            # 1.0 rad/s cap only. --fine-align-wz is a heading_align candidate and
            # must not silently make the 6 s entry budget impossible (11-25 review).
            return (0., 0., rate)
        if self.state == 'fine_entry_settle':
            if (now < self.entry_stop_time+self.turn_settle or
                    not samples.stopped_yaw_since(self.entry_stop_time, now,
                        self.turn_stop_window, self.turn_stop_yaw_rate, self.pose_timeout, self.position_stop_gap)):
                return (0., 0., 0.)
            if (not self._entry_stop_already_due() and
                    abs(math.degrees(alpha)) > self.fine.fine_entry_done_deg+1e-9):
                return self._entry_attempt()
            self.final_events.append(('fine_entry_align_completed', self._entry_evidence()))
            self._close_entry()
            self._start_fine(self.entry_reason)
            return (0., 0., 0.)
        raise RuntimeError('Unknown entry state: '+self.state)

    def _entry_stop_already_due(self):
        distance = self.final_distance()
        if distance <= self.coast:
            return True
        if self.fine.fine_stop_layered:
            if distance <= self.fine.fine_stop_radius:
                return True
            if self.position_direction is not None:
                remaining = self.position_remaining()
                if (remaining < -self.passed_goal_distance or
                        remaining <= self.fine.fine_stop_projected and distance <= self.projected_stop_range):
                    return True
        return self.fine.fine_goal_behind and distance <= self.behind_min_distance

    def _plan_stg(self, pose, target):
        self._refresh_pre()
        target = self.goal
        result = self.plan_callback(pose, target)
        if hasattr(result, 'stg_xy'):
            self.terminal_stg = getattr(result, 'final_stg', False)
            return result.stg_xy
        # Allow small deterministic STG executor fixtures without a planner object.
        self.terminal_stg = result is not None and tuple(result) == tuple(target)
        return result

    def _goal_clear(self):
        """Goal-side clearance: stop-side check when the goal band is allowed."""
        callback=getattr(self,'goal_clear',None)
        if callback is not None:return callback(self.pose[:2],self.final_goal[:2])
        if getattr(self, 'allow_goal_in_inflation', False):
            return self.passage_clear(self.pose[:2], self.final_goal[:2])
        return self.clear(self.pose[:2], self.final_goal[:2], math.inf)

    def _enter_fine(self, reason, resume=False):
        align = False
        if self.fine.fine_entry_align:
            distance = self.final_distance()
            alpha_deg = math.degrees(self.target_alpha()) if distance else 0.
            stop_due = self._entry_stop_already_due()
            align = not stop_due and abs(alpha_deg) > self.fine.fine_entry_align_deg+1e-9
            direction = tuple((self.final_goal[k]-self.pose[k])/distance for k in (0, 1)) if distance else (0., 0.)
            row = self.evidence()
            row.update(fine_metrics(self.final_goal, direction, self.pose))
            row.update(previous_position_direction=self.position_direction, position_direction=direction,
                error_reference='prospective_entry_ray', alpha_deg=alpha_deg,
                triggered=align, existing_stop_due=stop_due, resume=resume)
            self.final_events.append(('fine_entry_gate_checked', row))
        if align:
            if self.recorder is not None:
                self.recorder.abort('Handoff to final entry alignment')
            self.stg = None
            self.entry_origin = tuple(self.pose)
            dx, dy = self.final_goal[0]-self.pose[0], self.final_goal[1]-self.pose[1]
            distance = math.hypot(dx, dy)
            self.entry_direction = (dx/distance, dy/distance)
            self.entry_reason = reason
            self.entry_started = self.fine_now
            if self.entry_clock_start is None:
                self.entry_clock_start = self.fine_now
            self.entry_samples.pause_samples()
            self._entry_attempt()
        elif resume:
            # No alignment took place: keep the existing lateral reference ray.
            self.state = 'fine_approach'
            self.reason = 'Final position approach'
        else:
            self._start_fine(reason)

    def _start_fine(self, reason):
        if self.recorder is not None:
            self.recorder.abort('Handoff to final approach')
        self.stg = None
        self.position_origin = tuple(self.pose)
        offset = tuple(self.final_goal[k]-self.position_origin[k] for k in (0, 1))
        length = math.hypot(*offset)
        self.position_direction = tuple(v/length for v in offset) if length else (0., 0.)
        self.state = 'fine_approach'
        self.reason = 'Final position approach'
        self.final_events.append(('fine_approach_started', dict(reason=reason,
            pose=self.position_origin, pre=self.pre, final_goal=self.final_goal,
            position_direction=self.position_direction)))

    def _refresh_pre(self):
        if self.pre is None:
            return
        route = self.route_callback(self.final_goal)
        self._route_events(route)
        if route['pre'] is None:
            self.pre = None
            self.goal = self.final_goal[:2]
            self.terminal_stg = False
            self.final_events.append(('pre_fallback', dict(reason=route['reason'], coarse_target=self.goal)))
        elif self.fine.fine_pre_fallback and tuple(route['pre']) != self.pre:
            self.pre = tuple(route['pre'])
            self.goal = self.pre
            self.terminal_stg = False
            self.final_events.append(('pre_reselected', dict(reason=route['reason'], coarse_target=self.goal)))

    def _passage_blocked(self, now, state):
        """Bound valid blocked-passage rechecks in either v3 phase; never replan."""
        if self.state != state:
            self.passage_waits = 0
            self.reason = 'Final passage blocked; waiting without replanning'
            return self.wait(state, now, self.fine_obstacle_wait)
        if now < self.until:
            return (0., 0., 0.)
        self.passage_waits += 1
        if self.passage_waits >= self.fine.fine_passage_waits:
            evidence = self._entry_evidence if state == 'fine_entry_obstacle' else self.evidence
            self.final_events.append(('fine_passage_blocked', evidence(
                blocked_state=state, waits=self.passage_waits, wait_limit=self.fine.fine_passage_waits)))
            self.stop('fine_passage_blocked')
            return (0., 0., 0.)
        self.reason = ('Final passage remains blocked; waiting without replanning '
                       '(%d/%d)' % (self.passage_waits, self.fine.fine_passage_waits))
        return self.wait(state, now, self.fine_obstacle_wait)

    def _fine_tick(self, now):
        if self.state.startswith('fine_entry_'):
            return self._entry_tick(now)
        if self.state == 'fine_obstacle':
            if now < self.until:
                return (0., 0., 0.)
            if self._goal_clear():
                self.passage_waits = 0     # a resolved blockage starts a fresh budget
                self._enter_fine('Final passage clear; entry recheck', resume=True)
                return (0., 0., 0.)
            if self.fine.fine_entry_align or self.fine.fine_lateral_guard:
                return self._passage_blocked(now, 'fine_obstacle')
            self.final_events.append(('fine_approach_replan', dict(reason='Passage remains blocked', pose=tuple(self.pose))))
            self._refresh_pre()
            self.terminal_stg = False
            self.state = 'plan'
            self.reason = 'Final passage blocked; replanning'
            return (0., 0., 0.)
        if self.state == 'fine_approach':
            distance = self.final_distance()
            remaining = self.position_remaining()
            passed = self.fine.fine_stop_layered and remaining < -self.passed_goal_distance
            if passed:
                # Diagnostic even when the 8cm circle or body-behind guard wins.
                self.final_events.append(('fine_passed_goal', self.evidence()))
            if self.fine.fine_stop_layered and distance <= self.fine.fine_stop_radius:
                return self._position_stop(now, 'radius')
            # Outside the stop circle, a body-behind abort must precede normal
            # position settling; otherwise the next stage would still rotate.
            if self.fine.fine_goal_behind:
                if distance <= self.behind_min_distance:
                    return self._position_stop(now, 'behind_guard_near_goal')
                bearing = math.atan2(self.final_goal[1]-self.pose[1], self.final_goal[0]-self.pose[0])
                rel = wrap(bearing-self.pose[2])
                if distance <= self.behind_max_distance and abs(rel) > self.behind_angle:
                    self.final_events.append(('final_goal_behind', self.evidence(
                        bearing_deg=math.degrees(bearing), rel_deg=math.degrees(rel))))
                    self.stop('final_goal_behind')
                    return (0., 0., 0.)
            if self.fine.fine_stop_layered:
                if passed:
                    return self._position_stop(now, 'passed_goal')
                if remaining <= self.fine.fine_stop_projected and distance <= self.projected_stop_range:
                    return self._position_stop(now, 'projected_near_goal')
                # This strategy REPLACES the old unbounded coast gate. Applying
                # it afterwards would silently defeat the lateral distance cap.
            elif remaining <= self.coast:
                return self._position_stop(now, 'legacy_coast')
            if not self._goal_clear():
                self.reason = 'Final approach obstacle; waiting'
                return self.wait('fine_obstacle', now, self.fine_obstacle_wait)
            if (remaining <= self.fine_no_turn_distance and
                    (not self.fine.fine_stop_layered or distance <= self.projected_stop_range)):
                # v3 guard is local to a real blind-forward command, not a
                # full-approach cross-track limit. Legacy stop precedence stays.
                lateral = fine_metrics(self.final_goal, self.position_direction, self.pose)['lateral_m']
                if (self.fine.fine_lateral_guard and distance <= self.fine_no_turn_distance and
                        abs(lateral) > self.fine.fine_lateral_max+1e-12):
                    self.final_events.append(('fine_lateral_exceeded', self.evidence(
                        alpha_deg=math.degrees(self.target_alpha()), lateral_limit_m=self.fine.fine_lateral_max)))
                    self.stop('fine_lateral_exceeded')
                    return (0., 0., 0.)
                return (self.fine_speed, 0., 0.)
            alpha = wrap(math.atan2(self.final_goal[1]-self.pose[1], self.final_goal[0]-self.pose[0])-self.pose[2])
            if abs(alpha) > self.drive_align:
                return (0., 0., self.turn_rate(alpha))
            return (self.fine_speed, 0., min(max(self.k_ang*alpha, -self.wz_max), self.wz_max))
        if self.state == 'fine_position_settle':
            if self.fine.position_settle_fresh:
                if not self._position_settled(now):
                    return (0., 0., 0.)
            if now < self.until:
                return (0., 0., 0.)
            self.position_settle_pose = tuple(self.pose)
            self.final_events.append(('fine_position_settled', self.evidence(
                internal_error_m=self.final_distance(), fresh_confirmation=self.fine.position_settle_fresh,
                settle_fresh_s=self.position_samples.settle_fresh_s)))
            self.state = 'heading_align'
            self.reason = 'Final yaw alignment'
            return (0., 0., 0.)
        if self.state == 'heading_align':
            error = wrap(self.final_goal[2]-self.pose[2])
            if abs(error) <= self.yaw_stop_tol:
                self.final_events.append(('fine_yaw_stop', self.evidence(yaw_error_rad=error)))
                self.reason = 'Final yaw stop; settling'
                return self.wait('fine_yaw_settle', now, self.turn_settle)
            # A speed cap is NOT a body/arm swept-volume collision guarantee.
            rate = self.turn_rate(error)
            return (0., 0., math.copysign(min(abs(rate), self.fine.fine_align_wz), rate))
        if self.state == 'fine_yaw_settle':
            if now < self.until:
                return (0., 0., 0.)
            self.final_pose = tuple(self.pose)
            self.final_error_m = self.final_distance()
            self.final_yaw_error = wrap(self.final_goal[2]-self.pose[2])
            self.final_events.append(('fine_yaw_settled', self.evidence(
                yaw_error_rad=self.final_yaw_error, fresh_confirmation=False)))
            self.final_events.append(('fine_approach_completed', self.evidence(final_pose=self.final_pose,
                internal_error_m=self.final_error_m, final_yaw_error_rad=self.final_yaw_error,
                position_tolerance_checked=False, yaw_rechecked_for_acceptance=False)))
            self.stop(self.completion_reason)
            return (0., 0., 0.)
        raise RuntimeError('Unknown final approach state: '+self.state)

    def tick(self, now):
        if not self.active:
            return None
        self.fine_now = now
        if self.state.startswith('fine_') or self.state == 'heading_align':
            self.paused = self.input_problem(now)
            if self.paused:
                if self.fine.fine_entry_align:
                    self.entry_samples.pause_samples()
                if self.fine.position_settle_fresh:
                    self.position_samples.pause_samples()
                return (0., 0., 0.)
            return self._fine_tick(now)
        if not self.input_problem(now) and self.state == 'plan_wait' and now >= self.until:
            self.state = 'plan'
        if (not self.input_problem(now) and self.state == 'recovery_settle' and
                now >= self.until and self.map_revision != self.recovery_stop_map):
            self.state = 'plan'
            self.reason = 'Recovery complete; replanning'
        if not self.input_problem(now) and self.state == 'plan':
            self._refresh_pre()
            if (self.pre is None and self.final_distance() <= self.direct_stg_range and
                    self._goal_clear()):
                self._enter_fine('Direct-goal fallback handoff')
                return (0., 0., 0.)
        was_settling = self.state == 'walk_settle'
        terminal = self.terminal_stg
        command = super().tick(now)
        if was_settling and self.state == 'plan' and terminal:
            self._enter_fine('Final pre STG completed' if self.pre is not None else 'Final goal STG completed')
        return command

    def stop(self, reason='Stopped'):
        if self.entry_started is not None:
            self._close_entry()
        if (self.state.startswith('fine_') or self.state == 'heading_align') and reason != self.completion_reason:
            self.final_events.append(('fine_approach_interrupted', dict(reason=reason, pose=self.pose)))
        super().stop(reason)
