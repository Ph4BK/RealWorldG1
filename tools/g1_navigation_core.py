"""G1 turn/walk state machine; no ROS, command acknowledgements or watchdog."""
import math
from posnav_fmm.errors import GoalUnreachable


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
    epsilon = .080
    turn_settle = .75
    walk_settle = .50
    goal_tol = .50
    plan_retry_s = 1.

    def __init__(self, plan, clear):
        self.plan = plan
        self.clear = clear
        self.pose = None
        self.pose_time = self.map_time = -math.inf
        self.active = False
        self.state = 'idle'
        self.reason = 'Ready'
        self.paused = ''
        self.stg = None
        self.origin = self.direction = None

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

    def stop(self, reason='Stopped'):
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

    def begin_walk(self, now):
        if self.goal_reached():
            return self.wait('walk_settle', now, self.walk_settle)
        self.origin = self.pose[:2]
        delta = tuple(g-p for g, p in zip(self.stg, self.origin))
        distance = math.hypot(*delta)
        if distance <= self.epsilon:
            return self.wait('walk_settle', now, self.walk_settle)
        self.direction = tuple(d/distance for d in delta)
        self.state = 'walk'
        return self.walk(now)

    def walk(self, now):
        if self.goal_reached() or self.remaining() <= self.epsilon:
            return self.wait('walk_settle', now, self.walk_settle)
        if not self.clear(self.pose[:2], self.stg, 1.):
            return self.wait('obstacle', now, 2.)
        return (self.speed, 0.)

    def tick(self, now):
        if not self.active:
            return None
        self.paused = self.input_problem(now)
        if self.paused:
            return (0., 0.)
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
            except GoalUnreachable as exc:
                self.stg = None
                self.origin = self.direction = None
                self.reason = str(exc)
                return self.wait('plan_wait', now, self.plan_retry_s)
            if stg is None:
                self.stop('No valid STG; navigation stopped')
                return (0., 0.)
            self.stg = tuple(stg)
            self.reason = 'Navigation running'
            if not self.clear(self.pose[:2], self.stg, math.inf):
                self.reason = 'STG segment is blocked'
                return self.wait('plan_wait', now, self.plan_retry_s)
            error = wrap(math.atan2(self.stg[1]-self.pose[1], self.stg[0]-self.pose[0])-self.pose[2])
            self.previous_error = self.turn_error = error
            self.turn_sign = 1 if error > self.left_tol else (-1 if error < self.right_tol else 0)
            if not self.turn_sign:
                return self.wait('turn_settle', now, self.turn_settle)
            self.state = 'turn'
            # Planning may take longer than one control cycle. Start moving on
            # the next cycle, after callbacks and freshness checks have run.
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
        if self.state == 'turn_settle':
            return self.begin_walk(now) if now >= self.until else (0., 0.)
        if self.state == 'walk':
            return self.walk(now)
        if self.state == 'obstacle':
            if self.goal_reached() or self.remaining() <= self.epsilon:
                return self.wait('walk_settle', now, self.walk_settle)
            if now < self.until:
                return (0., 0.)
            if self.clear(self.pose[:2], self.stg, 1.):
                self.state = 'walk'
                return (self.speed, 0.)
            self.state = 'plan'
            return (0., 0.)
        if self.state == 'walk_settle' and now >= self.until:
            if self.goal_reached():
                self.stop('Arrived within 0.50 m')
            else:
                self.state = 'plan'
        return (0., 0.)
