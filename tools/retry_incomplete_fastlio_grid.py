"""Reject incomplete captures and repeat after rosbag discovery delay was added."""
import concurrent.futures,json,os,shutil,subprocess,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];out=root/'diagnostics/voxel_grid_20260927'
m=json.loads((out/'manifest.json').read_text());jobs=[]
t0=json.loads((out/'bag_audit.json').read_text())['/utlidar/cloud_livox_mid360']['stamp_start_ns']/1e9
for j in m['jobs']:
 r=out/j['name'];s=json.loads((r/'summary.json').read_text());a=np.loadtxt(r/'odometry.csv',delimiter=',',skiprows=1)
 if s['odometry_poses']==298 and s['registered_cloud_frames']==298 and abs(a[0,0]-t0-.4021259)<.001:continue
 rejected=out/'rejected_startup'/j['name'];rejected.mkdir(parents=True,exist_ok=False)
 for name in ['summary.json','odometry.csv','mapping.log','playback.log']:
  shutil.copy2(r/name,rejected/name)
 shutil.rmtree(r);jobs.append(j)
print('Retry:',[j['name'] for j in jobs],flush=True)
def run(j):
 env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(j['domain']),OMP_NUM_THREADS='2')
 env.pop('ROS_LOG_DIR',None)
 with (out/(j['name']+'.log')).open('w') as f:
  subprocess.run([sys.executable,str(root/'tools/replay_fastlio_bag.py'),m['bag'],str(out/j['name']),'--rate',str(m['rate']),'--mapping-config',j['config']],env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
 print('Retried '+j['name'],flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as p:list(p.map(run,jobs))
(out/'retries.json').write_text(json.dumps(dict(reason='missing initial messages before DDS discovery; retry with rosbag --delay 3',runs=[j['name'] for j in jobs]),indent=2))
