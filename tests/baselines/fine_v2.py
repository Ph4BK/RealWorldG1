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

    def __post_init__(self):
        for name, value in asdict(self).items():
            if name in ('fine_stop_layered', 'fine_goal_behind', 'fine_pre_fallback',
                        'position_settle_fresh'):
                if not isinstance(value, bool):
                    raise ValueError('Expected boolean: '+name)
            elif isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid fine parameter: '+name)
        if (not 0 < self.fine_stop_projected <= self.fine_stop_radius <= .15 or
                self.fine_align_wz > 1. or self.position_settle_step > self.position_settle_span or
                not isinstance(self.position_settle_samples, int) or
                not 2 <= self.position_settle_samples <= 32):
            raise ValueError('Invalid fine stopping/settling bounds')


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

    def _enter_fine(self, reason):
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

    def _fine_tick(self, now):
        if self.state == 'fine_obstacle':
            if now < self.until:
                return (0., 0., 0.)
            if self.clear(self.pose[:2], self.final_goal[:2], math.inf):
                self.state = 'fine_approach'
                self.reason = 'Final position approach'
                return (0., 0., 0.)
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
            if not self.clear(self.pose[:2], self.final_goal[:2], math.inf):
                self.reason = 'Final approach obstacle; waiting'
                return self.wait('fine_obstacle', now, self.fine_obstacle_wait)
            if (remaining <= self.fine_no_turn_distance and
                    (not self.fine.fine_stop_layered or distance <= self.projected_stop_range)):
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
                    self.clear(self.pose[:2], self.final_goal[:2], math.inf)):
                self._enter_fine('Direct-goal fallback handoff')
                return (0., 0., 0.)
        was_settling = self.state == 'walk_settle'
        terminal = self.terminal_stg
        command = super().tick(now)
        if was_settling and self.state == 'plan' and terminal:
            self._enter_fine('Final pre STG completed' if self.pre is not None else 'Final goal STG completed')
        return command

    def stop(self, reason='Stopped'):
        if (self.state.startswith('fine_') or self.state == 'heading_align') and reason != self.completion_reason:
            self.final_events.append(('fine_approach_interrupted', dict(reason=reason, pose=self.pose)))
        super().stop(reason)
