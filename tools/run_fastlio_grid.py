"""Reproducible 3x3 offline voxel sweep; one ROS domain per concurrent run."""
import concurrent.futures
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml

root=Path(__file__).resolve().parents[1]
bag=root/'datasets/small_area_walk/bag'
out=root/'diagnostics/voxel_grid_20260927'
out.mkdir(exist_ok=False)
(out/'configs').mkdir()
subprocess.run([sys.executable,str(root/'tools/audit_lidar_imu_bag.py'),str(bag),str(out/'bag_audit.json')],check=True)
base=yaml.safe_load((root/'fastlio/config/g1_mid360.yaml').read_text())
jobs=[]
for i,surf in enumerate([.10,.15,.20]):
 for j,map_size in enumerate([.10,.15,.20]):
  name=f'surf_{surf:.2f}_map_{map_size:.2f}'
  config=json.loads(json.dumps(base));p=config['/**']['ros__parameters']
  p.update(filter_size_surf=surf,filter_size_map=map_size,cube_side_length=1000.)
  path=out/'configs'/f'{name}.yaml';path.write_text(yaml.safe_dump(config,sort_keys=False))
  jobs.append(dict(name=name,surf=surf,map=map_size,domain=101+i*3+j,config=str(path)))
manifest=dict(bag=str(bag),rate=.25,stationary_interval_seconds=[0,3],stationary_time_reference='first cloud header; output pose timestamps are scan end',jobs=jobs)
(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
def run(job):
 env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(job['domain']),OMP_NUM_THREADS='2',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOCALHOST_ONLY='1')
 for k in ['G1_DIAG_DIR','G1_PREDICT_ONLY','ROS_LOG_DIR']:env.pop(k,None)
 start=time.monotonic()
 with (out/(job['name']+'.log')).open('w') as f:
  proc=subprocess.run([sys.executable,str(root/'tools/replay_fastlio_bag.py'),str(bag),str(out/job['name']),'--rate','.25','--mapping-config',job['config']],env=env,stdout=f,stderr=subprocess.STDOUT)
 result=dict(name=job['name'],returncode=proc.returncode,wall_seconds=time.monotonic()-start)
 print(json.dumps(result),flush=True)
 return result
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
 results=list(pool.map(run,jobs))
(out/'execution.json').write_text(json.dumps(results,indent=2))
if any(r['returncode'] for r in results):sys.exit(1)

subprocess.run([sys.executable,str(root/"tools/retry_incomplete_fastlio_grid.py")],check=True)
