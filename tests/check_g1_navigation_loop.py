#!/usr/bin/env python3
"""Real ROS timer/services plus a simulated planar robot, on loopback only."""
import json
import math
import os
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger
from unitree_api.msg import Request
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from g1_navigation_node import NavigationNode


def main():
    assert os.environ.get('ROS_DOMAIN_ID','0') != '0'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    nav=NavigationNode(ROOT/'fastlio/config/live_bev.yaml')
    plant=rclpy.create_node('g1_simulated_navigation_test')
    executor=SingleThreadedExecutor();executor.add_node(nav);executor.add_node(plant)
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps=plant.create_publisher(OccupancyGrid,'/g1/bev/task',latched)
    poses=plant.create_publisher(PoseStamped,'/g1/live_bev/robot',1)
    goals=plant.create_publisher(PoseStamped,'/g1/live_bev/goal_2d',10)
    start=plant.create_client(Trigger,'/g1/navigation/start')
    xy=[0.,0.];yaw=[-.8];command=[0.,0.,0.];received=[];stgs=set();states=set()
    def receive(msg):
        data=json.loads(msg.parameter)
        assert msg.header.identity.api_id==7105 and msg.header.policy.noreply
        assert data['duration']==.1
        now=time.monotonic();received.append(now)
        command[:]=[data['velocity'][0],data['velocity'][2],now+data['duration']]
    plant.create_subscription(Request,'/api/sport/request',receive,1)
    def status(msg):
        data=json.loads(msg.data);states.add(data['state'])
        if data['stg'] is not None:stgs.add(tuple(data['stg']))
    plant.create_subscription(String,'/g1/navigation/status',status,latched)
    grid=OccupancyGrid();grid.header.frame_id='map'
    grid.info.resolution=.05;grid.info.width=grid.info.height=160
    grid.info.origin.position.x=grid.info.origin.position.y=-2.
    grid.data=[0]*(160*160)
    goal=PoseStamped();goal.header.frame_id='map';goal.pose.orientation.w=1.;goal.pose.position.x=2.8
    begun=time.monotonic();previous=begun;last_frame=0.;requested=None;goal_sent=False
    try:
        while time.monotonic()-begun<25.:
            now=time.monotonic();dt=now-previous;previous=now
            if now<command[2]:
                xy[0]+=command[0]*math.cos(yaw[0])*dt
                xy[1]+=command[0]*math.sin(yaw[0])*dt
                yaw[0]+=command[1]*dt
            if now-last_frame>=.1:
                p=PoseStamped();p.header.frame_id='map';p.header.stamp=plant.get_clock().now().to_msg()
                p.pose.position.x,p.pose.position.y=xy
                p.pose.orientation.z=math.sin(yaw[0]/2);p.pose.orientation.w=math.cos(yaw[0]/2)
                poses.publish(p)
                grid.header.stamp=plant.get_clock().now().to_msg();maps.publish(grid)
                last_frame=now
            if not goal_sent and goals.get_subscription_count()>0:
                goals.publish(goal);goal_sent=True
            if requested is None and nav.goal is not None and not nav.control.input_problem(now) and start.service_is_ready():
                requested=start.call_async(Trigger.Request())
            executor.spin_once(timeout_sec=.002)
            if requested is not None and requested.done():
                assert requested.result().success,requested.result().message
                if not nav.control.active:
                    break
        assert requested is not None and requested.done()
        assert not nav.control.active,nav.control.state
        assert nav.control.reason.startswith('Arrived'),nav.control.reason
        assert math.hypot(xy[0]-2.8,xy[1])<=.5
        assert len(stgs)>=2,stgs
        assert {'turn','turn_settle','walk','walk_settle'}.issubset(states),states
        periods=np.diff(received)
        median=float(np.median(periods))
        assert .015<median<.025,median
        # Final zero has to reach the simulated robot too.
        for _ in range(20):executor.spin_once(timeout_sec=.005)
        assert command[:2]==[0.,0.],command
        print(json.dumps(dict(result='passed',commands=len(received),stgs=len(stgs),
                              median_command_period_s=median,final_xy=xy,elapsed_s=time.monotonic()-begun)))
    finally:
        nav.cancel('Test finished');executor.shutdown();nav.destroy_node();plant.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
