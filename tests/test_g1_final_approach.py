"""Deterministic full PosNav stage tests, with no ROS or robot commands."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from g1_final_approach import PosNav
from g1_navigation_core import velocity_xyz
from posnav_fmm.errors import GoalUnreachable
from stg_error_recorder import STGErrorRecorder


class FinalApproachTest(unittest.TestCase):
    def setUp(self):
        self.use_pre = True
        self.blocked = False
        def route(g):
            pre = (g[0]-.6*math.cos(g[2]),g[1]-.6*math.sin(g[2])) if self.use_pre else None
            return dict(pre=pre,reason='test route')
        def plan(p,g):
            final=math.hypot(p[0]-g[0],p[1]-g[1])<=.5
            return SimpleNamespace(stg_xy=tuple(g) if final else (1.,0.),final_stg=final)
        self.plan=Mock(side_effect=plan)
        self.n=PosNav(self.plan,lambda *_:not self.blocked,route,STGErrorRecorder())
        self.n.pose=(0.,0.,0.)
        self.n.pose_time=self.n.map_time=0.
        self.n.map_revision=1
        self.n.start((2.,0.,0.),0.)

    def tick(self,t,pose=None,fresh=True):
        if pose is not None:self.n.pose=pose
        if fresh:self.n.pose_time=self.n.map_time=t
        command=self.n.tick(t)
        return None if command is None else velocity_xyz(command)

    def fine(self):
        self.n._enter_fine('test handoff')

    def test_route_targets_pre_and_keeps_original_final_goal(self):
        self.tick(0.)
        self.plan.assert_called_with((0.,0.),(1.4,0.))
        self.assertEqual(self.n.final_goal,(2.,0.,0.))
        self.assertEqual(self.n.pre,(1.4,0.))
        self.assertEqual(self.n.state,'walk_start')

    def test_final_pre_stg_settle_hands_off_without_extra_fmm_segment(self):
        self.tick(0.,(1.1,0.,0.))
        self.assertTrue(self.n.terminal_stg)
        self.assertEqual(self.tick(.02),(.5,0.,0.))
        self.tick(.1,(1.3,0.,0.))
        self.assertEqual(self.n.state,'walk_settle')
        self.tick(.59)
        self.assertEqual(self.n.state,'walk_settle')
        self.tick(.61)
        self.assertEqual(self.n.state,'fine_approach')
        self.assertEqual(self.plan.call_count,1)
        self.assertEqual(self.tick(.63),(.25,0.,0.))
        self.assertIsNone(self.n.stg)
        self.assertIsNone(self.n.recorder.current)

    def test_last_pre_stg_inside_epsilon_skips_walk_but_still_hands_off(self):
        self.tick(0.,(1.35,0.,0.))
        self.tick(.02)
        self.assertEqual(self.n.state,'walk_settle')
        self.tick(.53)
        self.assertEqual(self.n.state,'fine_approach')

    def test_no_pre_handoff_requires_range_and_clear_line(self):
        self.use_pre=False
        self.n.stop();self.n.start((2.,0.,0.),0.)
        self.tick(0.,(1.49,0.,0.))
        self.assertNotEqual(self.n.state,'fine_approach')
        self.n.state='plan'
        self.tick(.02,(1.5,0.,0.))
        self.assertEqual(self.n.state,'fine_approach')
        self.n.state='plan';self.blocked=True
        self.tick(.04)
        self.assertEqual(self.n.state,'plan_wait')

    def test_pre_becoming_blocked_falls_back_even_after_retry_wait(self):
        self.n.state='plan_wait';self.n.until=.1
        self.use_pre=False
        self.tick(.1,(1.6,0.,0.))
        self.assertIsNone(self.n.pre)
        self.assertEqual(self.n.goal,(2.,0.))
        self.assertEqual(self.n.state,'fine_approach')

    def test_half_metre_is_never_a_task_success_condition(self):
        self.fine()
        self.assertEqual(self.tick(0.,(1.6,0.,0.)),(.25,0.,0.))
        self.assertTrue(self.n.active)
        self.assertFalse(self.n.goal_reached())

    def test_small_angle_moves_and_turns_without_minimum_rate_floor(self):
        self.fine()
        vx,vy,w=self.tick(0.,(1.5,0.,-.05))
        self.assertEqual((vx,vy),(.25,0.))
        self.assertAlmostEqual(w,.075)
        self.assertLess(abs(w),self.n.wz_min)

    def test_signed_rotation_floor_ceiling_and_forward_boundary(self):
        self.fine()
        for alpha,expected in ((.1,(.25,.15)),(.1001,(0.,.15015)),
                               (-.1001,(0.,-.15015)),(2.,(0.,1.)),(-2.,(0.,-1.))):
            with self.subTest(alpha=alpha):
                vx,vy,w=self.tick(0.,(1.5,0.,-alpha))
                self.assertAlmostEqual(vx,expected[0]);self.assertEqual(vy,0.)
                self.assertAlmostEqual(w,expected[1])
        self.assertEqual(self.n.turn_rate(.01),.15)
        self.assertEqual(self.n.turn_rate(-.01),-.15)
        self.assertEqual(self.n.turn_rate(0.),0.)

    def test_coast_stop_has_priority_over_large_alpha(self):
        self.n.coast=.15
        self.fine()
        self.assertEqual(self.tick(0.,(1.9,0.,-1.)),(0.,0.,0.))
        self.assertEqual(self.n.state,'fine_position_settle')

    def test_position_wait_uses_latest_pose_then_final_yaw_not_target_bearing(self):
        self.n.final_goal=(2.,0.,1.)
        self.fine()
        self.tick(0.,(2.01,0.,0.))
        self.tick(.49,(1.95,0.,.2))
        self.assertEqual(self.n.state,'fine_position_settle')
        self.tick(.5)
        self.assertEqual(self.n.state,'heading_align')
        self.assertEqual(self.n.position_settle_pose,(1.95,0.,.2))
        self.assertEqual(self.tick(.52),(0.,0.,1.))

    def test_yaw_stops_at_threshold_and_waits_then_reports_actual_residuals(self):
        self.n.final_goal=(2.,0.,1.)
        self.fine();self.tick(0.,(2.01,0.,0.))
        self.tick(.5)
        self.assertEqual(self.tick(.52,(1.95,0.,.95)),(0.,0.,0.))
        self.assertEqual(self.n.state,'fine_yaw_settle')
        self.tick(1.26,(1.93,.02,1.2))
        self.assertTrue(self.n.active)
        self.tick(1.28)
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.reason,'Final approach completed')
        self.assertAlmostEqual(self.n.final_yaw_error,-.2)
        self.assertAlmostEqual(self.n.final_error_m,math.hypot(.07,.02))
        self.assertEqual(self.n.final_pose,(1.93,.02,1.2))

    def test_wrap_across_pi_and_yaw_sign(self):
        self.n.final_goal=(2.,0.,-math.pi+.02)
        self.n.state='heading_align'
        self.tick(0.,(1.9,0.,math.pi-.02))
        self.assertEqual(self.n.state,'fine_yaw_settle')
        self.n.state='heading_align'
        self.n.final_goal=(2.,0.,-.3)
        self.assertAlmostEqual(self.tick(.02,(1.9,0.,0.))[2],-.45)

    def test_pause_in_all_fine_phases_sends_zero_and_cancel_never_resumes(self):
        for state in ('fine_approach','fine_position_settle','heading_align','fine_yaw_settle','fine_obstacle'):
            with self.subTest(state=state):
                self.setUp();self.n.state=state;self.n.until=3.
                self.assertEqual(self.tick(1.,fresh=False),(0.,0.,0.))
                self.assertEqual(self.n.state,state)
                self.n.stop('Stopped by user')
                self.assertIsNone(self.tick(2.))
                self.assertTrue(any(e=='fine_approach_interrupted' for e,_ in self.n.final_events))

    def test_dynamic_obstacle_wait_resumes_or_returns_to_coarse(self):
        self.fine();self.blocked=True
        self.assertEqual(self.tick(0.,(1.5,0.,0.)),(0.,0.,0.))
        self.assertEqual(self.n.state,'fine_obstacle')
        self.tick(1.99)
        self.assertEqual(self.n.state,'fine_obstacle')
        self.blocked=False;self.tick(2.)
        self.assertEqual(self.n.state,'fine_approach')
        self.blocked=True;self.tick(2.02);self.tick(4.03)
        self.assertEqual(self.n.state,'plan')
        self.assertEqual(self.n.final_goal,(2.,0.,0.))

    def test_recovery_retains_final_goal_and_uses_new_route_afterwards(self):
        self.n.recovery=lambda _:dict(near_count=1,sector_counts=[2,0,1,1])
        self.plan.side_effect=GoalUnreachable('near-field blocked')
        self.tick(0.);self.tick(.02)
        self.assertEqual(self.n.state,'recovery_left')
        self.tick(1.02)
        self.n.map_revision=2
        self.tick(1.6)
        self.assertEqual(self.n.final_goal,(2.,0.,0.))
        self.assertEqual(self.n.goal,(1.4,0.))
        self.assertEqual(self.n.recovery_count,1)

    def test_already_within_coast_does_not_leave_goal_to_visit_pre(self):
        self.n.coast=.15
        self.n.stop();self.n.pose=(1.9,0.,.3)
        self.n.start((2.,0.,.3),0.)
        self.assertEqual(self.n.state,'fine_approach')
        self.tick(0.)
        self.assertEqual(self.n.state,'fine_position_settle')
        self.plan.assert_not_called()

    def test_zero_coast_stops_after_crossing_with_lateral_error_and_large_alpha(self):
        self.fine()
        self.assertEqual(self.tick(0.,(1.99,.04,0.)),(.25,0.,0.))
        self.assertEqual(self.n.state,'fine_approach')
        self.assertEqual(self.tick(.1,(2.01,.04,0.)),(0.,0.,0.))
        self.assertEqual(self.n.state,'fine_position_settle')
        event=[data for event,data in self.n.final_events if event=='fine_position_stop'][-1]
        self.assertAlmostEqual(event['projected_remaining_m'],-.01)
        self.assertAlmostEqual(event['distance_m'],math.hypot(.01,.04))

    def test_oblique_projection_uses_entry_position_not_yaw_or_pre(self):
        self.n.pose=(1.,-1.,2.)
        self.fine()
        self.assertEqual(self.n.position_origin,(1.,-1.,2.))
        for v in self.n.position_direction:self.assertAlmostEqual(v,1/math.sqrt(2))
        self.tick(0.,(2.1,-.3,0.))  # x alone has passed, fixed projection has not.
        self.assertEqual(self.n.state,'fine_approach')
        self.assertGreater(self.n.position_remaining(),0.)
        self.assertEqual(self.tick(.1,(1.9,.3,-2.)),(0.,0.,0.))
        self.assertEqual(self.n.state,'fine_position_settle')
        self.assertLess(self.n.position_remaining(),0.)

    def test_projection_is_retained_across_obstacle_wait_and_input_pause(self):
        self.fine();direction=self.n.position_direction
        self.blocked=True;self.tick(0.,(1.5,.2,.3))
        self.assertEqual(self.n.state,'fine_obstacle')
        self.blocked=False;self.tick(2.,(1.6,.1,.4))
        self.assertEqual(self.n.position_direction,direction)
        self.tick(3.,fresh=False)
        self.assertEqual(self.n.position_direction,direction)
        self.assertEqual(self.tick(3.1,(2.01,.1,0.)),(0.,0.,0.))
        self.assertEqual(self.n.state,'fine_position_settle')

    def test_reentering_position_stage_refreshes_projection(self):
        self.fine();self.blocked=True;self.tick(0.)
        self.tick(2.01)
        self.assertEqual(self.n.state,'plan')
        self.n.pose=(1.,-1.,0.)
        self.fine()
        self.assertEqual(self.n.position_origin,(1.,-1.,0.))
        self.assertAlmostEqual(self.n.position_direction[0],1/math.sqrt(2))
        self.n.stop();self.n.pose_time=self.n.map_time=3.
        self.n.start((3.,0.,0.),3.)
        self.assertIsNone(self.n.position_direction)

    def test_exact_goal_at_entry_handles_zero_length_direction(self):
        self.n.stop();self.n.pose=(2.,0.,.3)
        self.n.start((2.,0.,.3),0.)
        self.assertEqual(self.n.position_direction,(0.,0.))
        self.assertEqual(self.tick(0.),(0.,0.,0.))
        self.assertEqual(self.n.state,'fine_position_settle')

    def test_sampling_jump_crosses_goal_plane_without_exact_xy(self):
        self.n.final_goal=(5.,5.,.8)
        self.n.pose=(2.,1.,0.)
        self.fine()
        self.tick(0.,(5.2,4.8,-.3))
        self.assertEqual(self.n.state,'fine_approach')
        self.assertEqual(self.tick(.1,(5.3,4.9,-.2)),(0.,0.,0.))
        self.assertEqual(self.n.state,'fine_position_settle')
        self.assertAlmostEqual(self.n.position_remaining(),-.1)
        self.assertGreater(self.n.final_distance(),.09)

    def test_near_goal_disables_both_small_and_large_angle_corrections(self):
        self.fine()
        bearing=math.atan2(-.4,.1)
        for alpha in (.05,-.05,.5,-.5,math.pi):
            with self.subTest(alpha=alpha):
                command=self.tick(0.,(1.9,.4,bearing-alpha))
                self.assertEqual(command,(.25,0.,0.))
                self.assertEqual(self.n.state,'fine_approach')
                self.assertGreater(self.n.final_distance(),self.n.fine_no_turn_distance)

    def test_no_turn_threshold_boundary_uses_projection(self):
        self.n.final_goal=(.15,0.,1.)
        self.fine()
        self.assertEqual(self.tick(0.,(-.000001,0.,-.5)),(0.,0.,.75))
        self.assertEqual(self.tick(.02,(0.,0.,-.5)),(.25,0.,0.))
        self.assertEqual(self.tick(.04,(.05,0.,-.5)),(.25,0.,0.))

    def test_near_goal_obstacle_still_stops_and_projection_stop_has_priority(self):
        self.fine();self.blocked=True
        self.assertEqual(self.tick(0.,(1.9,0.,0.)),(0.,0.,0.))
        self.assertEqual(self.n.state,'fine_obstacle')
        self.blocked=False;self.tick(2.)
        self.assertEqual(self.tick(2.02),(self.n.fine_speed,0.,0.))
        self.blocked=True
        self.assertEqual(self.tick(2.04,(2.01,.1,1.)),(0.,0.,0.))
        self.assertEqual(self.n.state,'fine_position_settle')


if __name__=='__main__':unittest.main()
