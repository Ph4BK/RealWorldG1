"""Offline rosbag2 SQLite extraction; no ROS node, replay, or coordinate transform."""
import argparse
import json
import math
from pathlib import Path
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    clouds=[]; poses=[]; frames=set(); stamps=[]
    stats=dict(frames=0,input_points=0,nonfinite_points=0,zero_points=0,kept_points=0)
    files=sorted(args.bag.glob('*.db3'))
    if not files: raise RuntimeError('No SQLite bag files')
    for path in files:
        con=sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True)
        try:
            for topic,data in con.execute('SELECT topics.name, messages.data FROM messages JOIN topics ON topics.id=messages.topic_id ORDER BY messages.timestamp'):
                if topic=='/dog_odom':
                    m=deserialize_message(data,Odometry); p=m.pose.pose.position; q=m.pose.pose.orientation
                    poses.append([p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
                elif topic=='/utlidar/cloud_livox_mid360':
                    m=deserialize_message(data,PointCloud2)
                    fields={f.name:f for f in m.fields}
                    endian='>' if m.is_bigendian else '<'
                    dtype=np.dtype({'names':['x','y','z'],'formats':[endian+{7:'f4',8:'f8'}[fields[k].datatype] for k in ('x','y','z')],
                                    'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':m.point_step})
                    cloud=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))
                    xyz=np.column_stack([cloud[k].reshape(-1) for k in ('x','y','z')]).astype(np.float32)
                    finite=np.isfinite(xyz).all(axis=1); zero=(xyz==0).all(axis=1)
                    kept=xyz[finite & ~zero]; clouds.append(kept)
                    stats['frames']+=1; stats['input_points']+=len(xyz)
                    stats['nonfinite_points']+=int((~finite).sum()); stats['zero_points']+=int(zero.sum()); stats['kept_points']+=len(kept)
                    frames.add(m.header.frame_id); stamps.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
        finally: con.close()
    if not clouds or not stats['kept_points']: raise RuntimeError('No nonzero finite cloud points')
    points=np.concatenate(clouds)
    np.save(args.output/'points_xyz.npy',points)
    stats.update(frame_ids=sorted(frames),cloud_stamp_range=[min(stamps),max(stamps)],xyz_min=points.min(axis=0).tolist(),xyz_max=points.max(axis=0).tolist(),transform='none',height_filter='none')
    if poses:
        p=np.asarray(poses); yaw=np.unwrap(p[:,2])
        stats['odom']={'samples':len(p),'xy_min':p[:,:2].min(axis=0).tolist(),'xy_max':p[:,:2].max(axis=0).tolist(),'yaw_unwrapped_range_rad':[float(yaw.min()),float(yaw.max())]}
    (args.output/'summary.json').write_text(json.dumps(stats,indent=2)+'\n')
    print(json.dumps(stats),flush=True)

if __name__=='__main__': main()
