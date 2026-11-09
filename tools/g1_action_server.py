"""NavigateTo adapter for the existing, single-owner G1 controller.

All callbacks share the navigation node's mutually exclusive callback group.
Execution awaits a Future; it never sleeps or blocks sensor/control callbacks.
"""
from dataclasses import dataclass
import math
import time

from geometry_msgs.msg import PoseStamped
from nav_interfaces.action import NavigateTo
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.task import Future

from g1_navigation_core import wrap


@dataclass
class Run:
    handle: object
    future: object
    started: float
    goal_scene: tuple
    reason: str = ''
    last_feedback: float = -math.inf
    trial_id: object = None


class NavigationAction:
    def __init__(self, node):
        self.node = node
        cfg = node.cfg.get('action', {})
        self.scene_frame = cfg.get('coordinate_frame', 'g1_d1_803_map')
        self.transform = tuple(cfg.get('map_from_scene_xy_yaw', [0., 0., 0.]))
        if (not isinstance(self.scene_frame, str) or not self.scene_frame or
                len(self.transform) != 3 or not all(math.isfinite(v) for v in self.transform)):
            raise ValueError('Invalid Action coordinate configuration')
        self.reserved = False
        self.current = None
        self.runs = {}
        self.server = ActionServer(node, NavigateTo, '/nav/navigate_to',
            execute_callback=self.execute, goal_callback=self.goal,
            cancel_callback=self.cancel, handle_accepted_callback=self.accepted)

    @property
    def busy(self):
        return self.reserved or self.current is not None

    def map_pose(self, pose):
        x, y, yaw = pose
        tx, ty, angle = self.transform
        c, s = math.cos(angle), math.sin(angle)
        return (c*x-s*y+tx, s*x+c*y+ty, wrap(yaw+angle))

    def scene_pose(self, pose):
        x, y, yaw = pose
        tx, ty, angle = self.transform
        c, s = math.cos(angle), math.sin(angle)
        x, y = x-tx, y-ty
        return (c*x+s*y, -s*x+c*y, wrap(yaw-angle))

    def event(self, name, **data):
        if self.node.experiment:
            run = self.current
            self.node.experiment.event(name, action_goal_uuid=self.key(run.handle) if run else None,
                                       action_trial_id=run.trial_id if run else None,
                                       coordinate_frame=self.scene_frame, **data)

    @staticmethod
    def key(handle):
        return bytes(handle.goal_id.uuid).hex()

    def goal(self, request):
        n = self.node
        try:
            if self.busy or n.control.active:
                raise ValueError('Navigation is busy')
            values = [getattr(request, p+k) for p in ('s_', 'g_')
                      for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')]
            values += [request.pos_tol, request.yaw_tol, request.timeout, request.max_speed]
            if not all(math.isfinite(v) for v in values):
                raise ValueError('Goal fields must be finite')
            if any(getattr(request, p+k) != 0. for p in ('s_', 'g_') for k in ('z', 'roll', 'pitch')):
                raise ValueError('Only planar goals are supported (z/roll/pitch=0)')
            if any(abs(getattr(request, p+'yaw')) > math.pi for p in ('s_', 'g_')):
                raise ValueError('Yaw must be in [-pi, pi]')
            if min(request.timeout, request.pos_tol, request.yaw_tol, request.max_speed) <= 0.:
                raise ValueError('Tolerances, timeout and max_speed must be positive')
            if request.max_speed+1e-6 < n.control.speed:
                raise ValueError('Requested speed limit is below the fixed G1 speed')
            problem = n.control.input_problem(time.monotonic())
            if problem or n.recording_error or (n.experiment and n.experiment.writer.error):
                raise ValueError(problem or n.recording_error or 'Experiment recorder failed')
        except ValueError as exc:
            n.get_logger().warning('Action goal rejected: '+str(exc))
            self.event('action_goal_rejected', reason=str(exc))
            return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    def accepted(self, handle):
        g = handle.request
        run = Run(handle, Future(), time.monotonic(), (g.g_x, g.g_y, g.g_yaw))
        self.runs[self.key(handle)] = run
        self.current = run
        self.reserved = False
        # Schedule execution before completing the Future, including start failures.
        handle.execute()
        try:
            self.node.begin_navigation(self.map_pose(run.goal_scene))
            if self.node.experiment:
                run.trial_id = self.node.experiment.trial_id
            self.event('action_goal_started', requested_goal_scene=run.goal_scene,
                requested_start_scene=[g.s_x, g.s_y, g.s_yaw], start_policy='record_only_no_localization_reset',
                requested_pos_tol=g.pos_tol, requested_yaw_tol=g.yaw_tol,
                effective_pos_tol=None, position_tolerance_checked=False, final_yaw_control=True,
                yaw_stop_tol_rad=self.node.control.yaw_stop_tol,
                timeout_s=g.timeout, max_speed=g.max_speed,
                map_from_scene_xy_yaw=self.transform)
        except Exception as exc:
            run.reason = 'start_failed'
            self.node.cancel('Action start failed: '+str(exc))

    async def execute(self, handle):
        key = self.key(handle)
        try:
            return await self.runs[key].future
        finally:
            self.runs.pop(key, None)

    def cancel(self, handle):
        if self.current is not None and self.key(handle) == self.key(self.current.handle):
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def stamped_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = 'scene'
        stamp = self.node.pose_source_stamp
        if stamp is not None:
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1000000000)
        p = self.node.control.pose
        if p is not None:
            x, y, yaw = self.scene_pose(p)
            msg.pose.position.x, msg.pose.position.y = float(x), float(y)
            msg.pose.orientation.z = math.sin(yaw/2.)
            msg.pose.orientation.w = math.cos(yaw/2.)
        else:
            msg.pose.orientation.w = 1.
        return msg

    def update(self):
        run = self.current
        if run is None or run.future.done():
            return
        n, now = self.node, time.monotonic()
        if n.control.active:
            if run.handle.is_cancel_requested:
                run.reason = 'cancelled'
                n.cancel('Action cancelled')
                return
            if now-run.started >= run.handle.request.timeout:
                run.reason = 'timeout'
                n.cancel('Action timeout')
                return
            if now-run.last_feedback >= .1:
                fb = NavigateTo.Feedback()
                fb.stage = n.control.feedback_stage()
                fb.current_pose = self.stamped_pose()
                x, y, yaw = self.scene_pose(n.control.pose)
                gx, gy, gyaw = run.goal_scene
                fb.remaining_dist = float(math.hypot(gx-x, gy-y))
                fb.remaining_yaw = float(wrap(gyaw-yaw))
                run.handle.publish_feedback(fb)
                run.last_feedback = now
            return
        result = NavigateTo.Result()
        fresh = n.control.pose is not None and now-n.control.pose_time < n.control.pose_timeout
        arrived = n.control.reason == n.control.completion_reason and fresh
        reasons = {'Stopped by user': 'stopped_by_user',
                   'Goal redrawn; press Start Navigation': 'goal_redrawn',
                   'Localization stopped or map reset': 'localization_stopped',
                   'Navigation process stopped': 'shutdown',
                   'recovery_exhausted': 'recovery_exhausted',
                   'No valid STG; navigation stopped': 'no_stg'}
        reason = run.reason or ('arrived' if arrived else reasons.get(n.control.reason, 'navigation_error'))
        if not fresh:
            reason = 'no_pose' if n.control.pose is None else 'pose_stale'
        result.success = arrived and not run.reason
        result.reason = reason
        result.final_pose = self.stamped_pose()
        goal_map = self.map_pose(run.goal_scene)
        result.internal_error = float(math.hypot(n.control.pose[0]-goal_map[0],
                                                n.control.pose[1]-goal_map[1])) if fresh else -1.
        # Unknown metrics use sentinels, never fabricated zero uncertainty.
        result.uncertainty = -1.
        result.blocked_distance, result.blocked_bearing = -1., 0.
        result.elapsed = float(now-run.started)
        self.event('action_result', reason=reason, success=result.success,
                   controller_reason=n.control.reason, action_elapsed_s=result.elapsed,
                   internal_error_m=result.internal_error,
                   final_yaw_error_rad=n.control.final_yaw_error,
                   position_tolerance_checked=False)
        if run.handle.is_cancel_requested:
            result.success = False
            run.handle.canceled()
        elif result.success:
            run.handle.succeed()
        else:
            run.handle.abort()
        run.future.set_result(result)
        self.current = None

    def close(self):
        if self.current is not None:
            self.node.cancel('Navigation process stopped')
        self.server.destroy()
