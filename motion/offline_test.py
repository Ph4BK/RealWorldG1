"""Integration tests with a fake service; never run on a robot network."""
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from RealWorldG1.src.motion_calibration import analyze_calibration

assert set(os.listdir('/sys/class/net')) == {'lo'}, 'requires --network none'
os.environ['ROS_DOMAIN_ID']='78'
os.environ['CYCLONEDDS_URI']='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" /></Interfaces></General></Domain></CycloneDDS>'
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from unitree_api.msg import Request, Response

binary='/tmp/g1_motion_build/g1_motion'
rclpy.init()
node=rclpy.create_node('fake_g1_formal')
pub=node.create_publisher(Response,'/api/sport/response',10)
odom=node.create_publisher(Odometry,'/dog_odom',10)
pose_pub=node.create_publisher(PoseStamped,'/g1/live_bev/robot',10)
calls=[]
queue=[]
scenario=''

def receive(m):
    calls.append((time.monotonic(),m.header.identity.api_id,m.parameter))
    motion=m.header.identity.api_id==7105
    moving=motion and json.loads(m.parameter)['velocity']!=[0,0,0]
    if (scenario=='missing_motion_ack' and moving) or (scenario=='missing_stop_ack' and motion and not moving): return
    response=Response()
    response.header.identity=m.header.identity
    response.header.status.code=7301 if scenario in ('rejected','missing_stop_ack') and moving else 0
    response.data=json.dumps({'data':802 if scenario=='wrong_fsm' else 501}) if not motion else '{}'
    queue.append((time.monotonic()+.1,response))

sub=node.create_subscription(Request,'/api/sport/request',receive,10)
try:
    for mode in ('--dry-run','--execute','--duration-only-test','--duration-only-dry-run'):
        assert subprocess.run([binary,mode,'.2','0','2'],capture_output=True).returncode==2
    for args in ([],['.2','0','0'],['.2','.1','1'],['nan','0','1'],['.2','0','-1'],['.2x','0','1'], ['0','.1','.1','1'], ['.2','nan','0','1']):
        assert subprocess.run([binary,*args],capture_output=True).returncode==2
    print('PASS removed modes and invalid arguments',flush=True)
    for scenario in ('odom_stationary','odom_moving','odom_missing'):
        calls.clear(); queue.clear()
        process=subprocess.Popen([binary,'--odom'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        deadline=time.monotonic()+9
        try:
            while process.poll() is None and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.005)
                m=Odometry(); m.header.stamp=node.get_clock().now().to_msg()
                m.header.frame_id='odom'; m.child_frame_id='base_link'
                m.pose.pose.position.x=1.25; m.pose.pose.position.y=-.75
                m.pose.pose.orientation.z=math.sin(.3); m.pose.pose.orientation.w=math.cos(.3)
                m.twist.twist.linear.x=.2 if scenario=='odom_moving' else 0.
                if scenario!='odom_missing': odom.publish(m)
            assert process.poll() is not None,'read-only timeout'
            out,err=process.communicate()
            result=json.loads([x[7:] for x in out.splitlines() if x.startswith('RESULT ')][-1])
            assert not calls, calls
            assert process.returncode==(1 if scenario=='odom_missing' else 0),(out,err)
            if scenario!='odom_missing':
                s=result['odom']
                assert s['x']==1.25 and s['y']==-.75 and abs(s['yaw_rad']-.6)<1e-8,s
                assert s['frame_id']=='odom' and s['stamp_ns']>0,s
                assert s['stationary_by_twist']==(scenario=='odom_stationary'),s
            print('PASS',scenario,flush=True)
        finally:
            if process.poll() is None: process.kill(); process.wait()
    successful=('success','turn','right_turn','backward','large_displacement','above_old_caps',
                'lateral_left','lateral_right','forward_lateral_left','forward_lateral_right',
                'calibrate_forward','calibrate_left','calibrate_right')
    for scenario in (*successful,'wrong_fsm','stale','stale_during','rejected','missing_motion_ack','missing_stop_ack','interrupt',
                     'calibrate_missing','calibrate_old','calibrate_tail_stale'):
        calls.clear(); queue.clear()
        args=['.2','0','.4']
        if scenario=='turn': args=['0','.1','.4']
        if scenario=='right_turn': args=['0','-.1','.4']
        if scenario=='backward': args=['-.2','0','.4']
        if scenario=='above_old_caps': args=['0','-.3','3']
        if scenario=='lateral_left': args=['0','.03','0','.4']
        if scenario=='lateral_right': args=['0','-.03','0','.4']
        if scenario=='forward_lateral_left': args=['.2','.03','0','.4']
        if scenario=='forward_lateral_right': args=['.2','-.03','0','.4']
        if scenario not in successful: args=['.2','.03','0','.4']
        if scenario.startswith('calibrate_'):
            args=['.2','0','.4']
            if scenario=='calibrate_left': args=['0','.1','.4']
            if scenario=='calibrate_right': args=['0','-.1','.4']
        numeric_args=list(args)
        if scenario.startswith('calibrate_'): args.append('--calibrate')
        # A 2 s pose trace can exceed a pipe buffer. Drain via temporary files
        # while the parent keeps publishing the fake feedback.
        out_file=tempfile.TemporaryFile(mode='w+t')
        err_file=tempfile.TemporaryFile(mode='w+t')
        process=subprocess.Popen([binary,*args],stdout=out_file,stderr=err_file,text=True)
        deadline=time.monotonic()+16
        signalled=False
        try:
            while process.poll() is None and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.005)
                m=Odometry(); m.header.stamp=node.get_clock().now().to_msg(); m.pose.pose.orientation.w=1.
                motion=[c for c in calls if c[1]==7105]
                if scenario=='large_displacement' and motion:
                    m.pose.pose.position.x=.3
                    m.pose.pose.orientation.z=math.sin(.4/2)
                    m.pose.pose.orientation.w=math.cos(.4/2)
                if scenario!='stale' and not (scenario=='stale_during' and motion): odom.publish(m)
                pose=PoseStamped(); pose.header.stamp=m.header.stamp; pose.header.frame_id='map'
                pose.pose.orientation.w=1.
                moving_time=time.monotonic()-motion[0][0] if motion else 0.
                if motion:
                    # Movement and post-expiry residual, then a stable pose.
                    progress=min(moving_time,.9)
                    if scenario=='calibrate_forward': pose.pose.position.x=.1*progress
                    if scenario in ('calibrate_left','calibrate_right'):
                        yaw=(1 if scenario=='calibrate_left' else -1)*.2*progress
                        pose.pose.orientation.z=math.sin(yaw/2); pose.pose.orientation.w=math.cos(yaw/2)
                if scenario=='calibrate_old': pose.header.stamp.sec-=2
                if scenario!='calibrate_missing' and not (scenario=='calibrate_tail_stale' and moving_time>.45):
                    pose_pub.publish(pose)
                if scenario=='interrupt' and motion and not signalled:
                    process.send_signal(signal.SIGINT); signalled=True
                for item in queue[:]:
                    if time.monotonic()>=item[0]: pub.publish(item[1]); queue.remove(item)
            assert process.poll() is not None, 'test timeout'
            process.wait()
            out_file.seek(0); err_file.seek(0)
            out,err=out_file.read(),err_file.read()
            report=json.loads([x[7:] for x in out.splitlines() if x.startswith('RESULT ')][-1])
            assert not any(k in report for k in ('forward_at_return_m','lateral_at_return_m','yaw_at_return_rad')),report
            assert process.returncode==(0 if scenario in successful else 1),(scenario,out,err)
            assert all(c[1] in (7001,7105) for c in calls),calls
            motion=[c for c in calls if c[1]==7105]
            if scenario in ('wrong_fsm','stale','calibrate_missing','calibrate_old'):
                assert not motion,calls
            else:
                assert 'odom_before' not in report,report
                expected=list(map(float,numeric_args[:3])) if len(numeric_args)==4 else [float(numeric_args[0]),0,float(numeric_args[1])]
                assert all(abs(a-b)<1e-6 for a,b in zip(json.loads(motion[0][2])['velocity'],expected)),calls
                assert abs(json.loads(motion[0][2])['duration']-float(numeric_args[-1]))<1e-6,calls
                assert abs(report['vy']-expected[1])<1e-6,report
                ok=scenario in successful
                assert len(motion)==(1 if ok else 2),calls
                assert report['zero_velocity_sent']==(not ok),report
                if ok:
                    assert report['command_elapsed_s']>=float(numeric_args[-1])-.05,report
                    if scenario.startswith('calibrate_'):
                        data=report['calibration']; poses=data['poses']
                        assert poses[0]['stamp_ns']<=data['expiry_ros_ns'],data
                        assert poses[-1]['stamp_ns']>=data['expiry_ros_ns']+2_000_000_000,data
                        assert time.monotonic()-motion[0][0]>=2.4,calls
                        assert data['action']=={'calibrate_forward':'forward','calibrate_left':'left_turn','calibrate_right':'right_turn'}[scenario]
                        reduced=analyze_calibration(report)
                        if scenario=='calibrate_forward': assert reduced['residual']['forward_m']>0,reduced
                        else: assert reduced['residual']['yaw_along_command_rad']>0,reduced
                    else: assert 'calibration' not in report,report
                else:
                    assert json.loads(motion[-1][2])['velocity']==[0,0,0]
                    assert motion[-1][0]-motion[0][0]<(1.3 if scenario=='calibrate_tail_stale' else .65),calls
            print('PASS',scenario,flush=True)
        finally:
            if process.poll() is None: process.kill(); process.wait()
            out_file.close(); err_file.close()
finally:
    node.destroy_node(); rclpy.shutdown()
