"""Read-only live timing monitor; saves individual samples and aggregate statistics."""
import argparse,csv,json,time
from pathlib import Path
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,Imu
from rclpy.qos import QoSProfile,ReliabilityPolicy
p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--seconds',type=float,default=60);args=p.parse_args()
args.output.mkdir(parents=True,exist_ok=False)
rclpy.init();node=rclpy.create_node('g1_fastlio_live_monitor');data={k:[] for k in ['cloud','imu','odom']}
start=time.monotonic()
def callback(key):
 def receive(m):
  stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
  row=[time.monotonic()-start,time.time(),stamp]
  if key=='odom':row += [m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]
  data[key].append(row)
 return receive
qos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT)
for key,typ,topic in [('cloud',PointCloud2,'/utlidar/cloud_livox_mid360'),('imu',Imu,'/utlidar/imu_livox_mid360'),('odom',Odometry,'/Odometry_loc')]:
 node.create_subscription(typ,topic,callback(key),qos)
try:
 while time.monotonic()-start<args.seconds:rclpy.spin_once(node,timeout_sec=.1)
finally:
 result={}
 for key,rows in data.items():
  with (args.output/(key+'.csv')).open('w') as f:
   w=csv.writer(f);w.writerow(['elapsed_monotonic_s','receive_wall_epoch_s','header_epoch_s']+(['x','y','z'] if key=='odom' else []));w.writerows(rows)
  if len(rows)<2:result[key]={'count':len(rows)};continue
  a=np.array(rows);delay=(a[:,1]-a[:,2])*1000;dt=np.diff(a[:,0]);hd=np.diff(a[:,2]);late=a[:,0]>args.seconds/2
  result[key]=dict(count=len(a),receive_hz=(len(a)-1)/(a[-1,0]-a[0,0]),header_hz=(len(a)-1)/(a[-1,2]-a[0,2]),
   header_nonincreasing=int((hd<=0).sum()),max_receive_gap_s=float(dt.max()),max_header_gap_s=float(hd.max()),
   wall_minus_header_ms_p50_p95_max=np.percentile(delay,[50,95,100]).tolist(),
   first_half_delay_median_ms=float(np.median(delay[~late])) if (~late).any() else None,
   last_half_delay_median_ms=float(np.median(delay[late])) if late.any() else None)
 result['note']='Wall minus sensor header assumes synchronized robot clocks; includes transport and is not pure computation time. Startup discovery can reduce counts.'
 (args.output/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
 node.destroy_node();rclpy.shutdown()
