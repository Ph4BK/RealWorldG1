"""PosNav orchestration: STGs to pre, point approach, then one yaw alignment.

The underlying Navigation class remains the separately tested STG executor.
No position tolerance is used to stop or accept this experimental fine controller.
"""
from collections import deque
import math

from g1_navigation_core import Navigation, wrap


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

    def __init__(self, plan, clear, route, recorder=None, recovery=None):
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
        super().__init__(self._plan_stg, clear, recorder, recovery)

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
        self.final_events.append(('navigation_route_selected', dict(final_goal=self.final_goal,
            pre=self.pre, coarse_target=self.goal, reason=route['reason'])))
        # No need to leave an already reached goal to visit its pre point.
        if self.final_distance() <= self.coast:
            self._enter_fine('Already within fine stopping distance')

    def goal_reached(self):
        # The old 0.50 m coarse success gate is never used by full PosNav.
        return False

    def final_distance(self):
        return math.hypot(self.final_goal[0]-self.pose[0], self.final_goal[1]-self.pose[1])

    def position_remaining(self):
        """Signed distance to the goal plane along the fixed approach direction."""
        return sum((self.final_goal[k]-self.pose[k])*self.position_direction[k] for k in (0, 1))

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
        if route['pre'] is None:
            self.pre = None
            self.goal = self.final_goal[:2]
            self.terminal_stg = False
            self.final_events.append(('pre_fallback', dict(reason=route['reason'], coarse_target=self.goal)))

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
            if remaining <= self.coast:
                self.final_events.append(('fine_position_stop', dict(pose=tuple(self.pose),
                    distance_m=distance, projected_remaining_m=remaining)))
                self.reason = 'Final position stop; settling'
                return self.wait('fine_position_settle', now, self.walk_settle)
            if not self.clear(self.pose[:2], self.final_goal[:2], math.inf):
                self.reason = 'Final approach obstacle; waiting'
                return self.wait('fine_obstacle', now, self.fine_obstacle_wait)
            if remaining <= self.fine_no_turn_distance:
                return (self.fine_speed, 0., 0.)
            alpha = wrap(math.atan2(self.final_goal[1]-self.pose[1], self.final_goal[0]-self.pose[0])-self.pose[2])
            if abs(alpha) > self.drive_align:
                return (0., 0., self.turn_rate(alpha))
            return (self.fine_speed, 0., min(max(self.k_ang*alpha, -self.wz_max), self.wz_max))
        if self.state == 'fine_position_settle':
            if now < self.until:
                return (0., 0., 0.)
            self.position_settle_pose = tuple(self.pose)
            self.final_events.append(('fine_position_settled', dict(pose=self.position_settle_pose,
                internal_error_m=self.final_distance(), projected_remaining_m=self.position_remaining())))
            self.state = 'heading_align'
            self.reason = 'Final yaw alignment'
            return (0., 0., 0.)
        if self.state == 'heading_align':
            error = wrap(self.final_goal[2]-self.pose[2])
            if abs(error) <= self.yaw_stop_tol:
                self.final_events.append(('fine_yaw_stop', dict(pose=tuple(self.pose), yaw_error_rad=error)))
                self.reason = 'Final yaw stop; settling'
                return self.wait('fine_yaw_settle', now, self.turn_settle)
            return (0., 0., self.turn_rate(error))
        if self.state == 'fine_yaw_settle':
            if now < self.until:
                return (0., 0., 0.)
            self.final_pose = tuple(self.pose)
            self.final_error_m = self.final_distance()
            self.final_yaw_error = wrap(self.final_goal[2]-self.pose[2])
            self.final_events.append(('fine_approach_completed', dict(final_pose=self.final_pose,
                internal_error_m=self.final_error_m, final_yaw_error_rad=self.final_yaw_error,
                position_tolerance_checked=False, yaw_rechecked_for_acceptance=False)))
            self.stop(self.completion_reason)
            return (0., 0., 0.)
        raise RuntimeError('Unknown final approach state: '+self.state)

    def tick(self, now):
        if not self.active:
            return None
        if self.state.startswith('fine_') or self.state == 'heading_align':
            self.paused = self.input_problem(now)
            if self.paused:
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
