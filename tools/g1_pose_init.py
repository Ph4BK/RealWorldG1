#!/usr/bin/env python3
"""Offline bag initial-pose UI adapter. IMU +X defines navigation forward."""
import argparse,json,sqlite3,subprocess,signal,os,time,shutil
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import Imu
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped,TransformStamped
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

def bag_level(bag,seconds):
    samples=[];first=None
    for db in sorted(bag.glob('*.db3')):
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro',uri=True) as c:
            topic=c.execute('SELECT id FROM topics WHERE name=?',('/utlidar/imu_livox_mid360',)).fetchone()
            if topic is None: continue
            for data, in c.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp',(topic[0],)):
                m=deserialize_message(data,Imu);t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                if first is None:first=t
                if t-first>seconds:break
                a=m.linear_acceleration;g=m.angular_velocity;samples.append([t,a.x,a.y,a.z,g.x,g.y,g.z])
    v=np.asarray(samples)
    if len(v)<100 or v[-1,0]-v[0,0]<seconds*.9:raise ValueError('Insufficient startup IMU samples')
    up=v[:,1:4].mean(axis=0);norm=np.linalg.norm(up)
    if norm<.1:raise ValueError('Invalid accelerometer mean')
    up/=norm
    # Relative acceleration variation and absolute gyro guards, not proof of zero velocity.
    variation=float(np.linalg.norm(v[:,1:4].std(axis=0))/norm)
    gyro=float(np.quantile(np.linalg.norm(v[:,4:7],axis=1),.95))
    if variation>.15 or gyro>.15:raise ValueError(f'Startup IMU not quiet: acc variation={variation}, gyro95={gyro}')
    x=np.array([1.,0,0]);x-=up*np.dot(x,up)
    if np.linalg.norm(x)<.1:raise ValueError('IMU +X too close to vertical')
    x/=np.linalg.norm(x);L=np.stack([x,np.cross(up,x),up])
    return L,dict(samples=len(v),seconds=seconds,mean_acceleration=v[:,1:4].mean(axis=0).tolist(),relative_acc_std=variation,gyro95_rad_s=gyro,forward_axis='IMU +X projected horizontally',assumption='initial IMU/body orientation is FAST-LIO camera_init; click denotes startup pose')

def compose(x,y,yaw,z,L):
    T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()@L;T[:3,3]=[x,y,z]
    return T

def replay_error(pose, running, output):
    if not isinstance(pose, dict) or any(
            not isinstance(pose.get(k), (int, float)) or not np.isfinite(pose[k])
            for k in ('x', 'y', 'yaw_deg')):
        return 'Missing valid x, y, or yaw. Please draw an arrow using 2D Pose Estimate first.'
    if running:
        return 'Replay is already running. Do not start it again.'
    return None


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bag',type=Path,required=True);p.add_argument('--map-dir',type=Path,required=True);p.add_argument('--map-xyz',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--imu-seconds',type=float,default=2.0);p.add_argument('--replay-output',type=Path);a=p.parse_args()
    if not 0<a.imu_seconds<=5:p.error('IMU window must be >0 and <=5 seconds')
    a.output.mkdir(parents=True,exist_ok=True);L,stats=bag_level(a.bag,a.imu_seconds)
    if a.replay_output is None:
        a.replay_output=Path(__file__).resolve().parents[1]/'diagnostics'/a.bag.resolve().parent.name/'replay'
    meta=yaml.safe_load((a.map_dir/'metadata.yaml').read_text());z=float(meta['initial_imu_xyz_map'][2]);np.savetxt(a.output/'T_level_rotation.txt',L);(a.output/'imu_startup.json').write_text(json.dumps(stats,indent=2))
    rclpy.init();node=rclpy.create_node('g1_pose_init')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    pubmap=node.create_publisher(__import__('sensor_msgs.msg',fromlist=['PointCloud2']).PointCloud2,'/reference_map',qos)
    pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    marker=node.create_publisher(PoseStamped,'/initialpose_preview',qos)
    xyz=np.load(a.map_xyz).astype(np.float32);cloud=create_cloud_xyz32(Header(frame_id='map'),xyz)
    pubmap.publish(cloud);node.create_timer(2.,lambda:pubmap.publish(cloud))
    # Mapping publishes camera_init -> body. Localization publishes map -> odom.
    tf=StaticTransformBroadcaster(node);identity=TransformStamped();identity.header.frame_id='odom';identity.child_frame_id='camera_init';identity.transform.rotation.w=1.;tf.sendTransform(identity)
    child=[None];log=[None];selected=[None];reported=[False]
    status=node.create_publisher(String,'/g1/pose_init_status',qos)
    def running():
        return child[0] is not None and child[0].poll() is None
    def announce(message):
        status.publish(String(data=json.dumps(dict(dataset=a.bag.resolve().parent.name,
                       pose=selected[0],running=running(),message=message),ensure_ascii=False)))
    def callback(m):
        if m.header.frame_id!='map':node.get_logger().error('Initial arrow must use frame map');return
        if running():announce('Cannot change the initial pose during replay. Wait for it to finish.');return
        q=m.pose.pose.orientation;arr=np.array([q.x,q.y,q.z,q.w]);xy=m.pose.pose.position
        if not np.isfinite([xy.x,xy.y,*arr]).all() or np.linalg.norm(arr)<1e-8:node.get_logger().error('Invalid pose');return
        yaw=Rotation.from_quat(arr).as_euler('xyz')[2];T=compose(xy.x,xy.y,yaw,z,L)
        out=PoseWithCovarianceStamped();out.header.frame_id='map';out.header.stamp=node.get_clock().now().to_msg();out.pose.pose.position.x=xy.x;out.pose.pose.position.y=xy.y;out.pose.pose.position.z=z
        quat=Rotation.from_matrix(T[:3,:3]).as_quat();out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,quat)
        # Planar display arrow intentionally stays on ground and excludes sensor roll/pitch.
        preview=PoseStamped();preview.header=out.header;preview.pose=m.pose.pose;preview.pose.position.z=.02;marker.publish(preview)
        np.savetxt(a.output/'T_map_from_camera_init.txt',T)
        euler=Rotation.from_matrix(T[:3,:3]).as_euler('XYZ',degrees=True).tolist()
        record=dict(x=xy.x,y=xy.y,yaw_deg=float(np.degrees(yaw)),z=z,initial_transform_euler_XYZ_deg=euler,initial_transform_euler_convention='R = Rx(roll) @ Ry(pitch) @ Rz(yaw); differs from planar heading',T_map_from_camera_init=T.tolist(),quaternion_xyzw=quat.tolist(),imu=stats,bag=str(a.bag.resolve()),map=str(a.map_dir.resolve()))
        (a.output/'initial_pose.json').write_text(json.dumps(record,indent=2))
        cfg={'global_localization_node':{'ros__parameters':dict(use_sim_time=True,path_map=str((a.map_dir/'map.pcd').resolve()),initialpose=[xy.x,xy.y,z,*Rotation.from_matrix(T[:3,:3]).as_euler('XYZ',degrees=True).tolist()],pcd_queue_maxsize=10,voxelsize_coarse=.1,voxelsize_fine=.1,threshold_fitness=.5,threshold_fitness_init=.5,loc_frequence=2.5,maxpoints_source=80000,maxpoints_target=400000,filter_odom2map=False,confidence_loc_th=.7,dis_updatemap=3.5,**{'kf_baselink2map/x':[.001,.002],'kf_baselink2map/y':[.001,.005],'kf_baselink2map/z':[.00001,.04]})}}
        config=a.output/'localization.yaml';config.write_text(yaml.safe_dump(cfg,sort_keys=False));pub.publish(out)
        node.get_logger().info(f'Saved startup pose: x={xy.x:.3f}, y={xy.y:.3f}, yaw={np.degrees(yaw):.1f} deg, z={z:.3f}; {config}')
        selected[0]=record
        announce('Initial pose saved. Press the button to start replay at 1x.')
    node.create_subscription(PoseWithCovarianceStamped,'/initialpose_2d',callback,10)

    # Reuse manual planar input only; rebuild z/rotation/config from this bag and map.
    saved=a.output/'initial_pose.json'
    if saved.exists():
        try:
            old=json.loads(saved.read_text())
            if not isinstance(old,dict) or any(
                    not isinstance(old.get(k),(int,float)) or not np.isfinite(old[k])
                    for k in ('x','y','yaw_deg')):
                raise ValueError('x/y/yaw missing or nonfinite')
            if Path(old['bag']).resolve()!=a.bag.resolve() or Path(old['map']).resolve()!=a.map_dir.resolve():
                raise ValueError('Saved pose belongs to another bag or map')
            m=PoseWithCovarianceStamped();m.header.frame_id='map'
            m.pose.pose.position.x=float(old['x']);m.pose.pose.position.y=float(old['y'])
            q=Rotation.from_euler('z',float(old['yaw_deg']),degrees=True).as_quat()
            m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=map(float,q)
            callback(m)
            announce('Saved pose loaded. Draw a new arrow or press the button to start replay.')
        except (ValueError,KeyError,TypeError,OSError) as exc:
            node.get_logger().warning(f'Cannot reuse saved pose: {exc}')
            selected[0]=None;announce('Saved pose is invalid. Please draw an arrow first.')
    else:
        announce('No initial pose. Please draw an arrow first.')

    def start_replay(request,response):
        error=replay_error(selected[0],running(),a.replay_output)
        if error:
            response.success=False;response.message=error;return response
        try:
            # Only replace replay results; never the source bag, initial pose, or reference map.
            target=a.replay_output.resolve()
            for protected in (a.bag.resolve(),a.output.resolve(),a.map_dir.resolve()):
                if target == protected or target in protected.parents or protected in target.parents:
                    raise OSError('Replay output overlaps protected input data')
            if a.replay_output.is_symlink():
                raise OSError('Replay output must not be a symlink')
            if a.replay_output.exists():
                shutil.rmtree(a.replay_output)
            a.replay_output.parent.mkdir(parents=True,exist_ok=True)
            if log[0]:log[0].close()
            log[0]=(a.replay_output.parent/'replay.log').open('w')
            child[0]=subprocess.Popen(['/usr/bin/python3',str(Path(__file__).with_name('replay_fastlio_bag.py')),
                      str(a.bag),str(a.replay_output),'--rate','1','--localization-config',
                      str(a.output/'localization.yaml')],stdout=log[0],stderr=subprocess.STDOUT,start_new_session=True)
            reported[0]=False;response.success=True;response.message='Localization replay started at 1x.'
            announce(response.message)
        except OSError as exc:
            response.success=False;response.message=f'Failed to start replay: {exc}';announce(response.message)
        return response
    node.create_service(Trigger,'/g1/start_replay',start_replay)
    def check():
        if child[0] and child[0].poll() is not None and not reported[0]:
            code=child[0].returncode
            announce(('Replay complete. ' if code==0 else f'Replay failed (exit code {code}). Check the logs: ')+str(a.replay_output))
            node.get_logger().info(f'Replay finished: exit={code}; inspect {a.replay_output}');reported[0]=True
    node.create_timer(1.,check)
    node.get_logger().info(f'READY: RViz 2D Pose Estimate -> /initialpose_2d; z={z:.3f}; startup IMU {stats["samples"]} samples')
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if child[0] and child[0].poll() is None:
            os.killpg(child[0].pid,signal.SIGINT)
            try:child[0].wait(timeout=45)
            except subprocess.TimeoutExpired:os.killpg(child[0].pid,signal.SIGTERM)
        if log[0]:log[0].close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
