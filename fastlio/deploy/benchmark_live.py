"""Bounded live FAST-LIO benchmark. Starts/stops only its own mapping process."""
import argparse,json,os,shutil,signal,subprocess,time
import yaml
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--seconds',type=float,default=90);args=p.parse_args()
out=Path('/work/runs')/args.run;out.mkdir(exist_ok=False)
config=yaml.safe_load(Path('/work/deploy/g1_live.yaml').read_text())
config['/**']['ros__parameters']['runtime_pos_log_enable']=True
(out/'config.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
env=os.environ.copy();env['ROS_LOG_DIR']=str(out/'ros_logs')
log=(out/'mapping.log').open('w')
proc=subprocess.Popen(['/work/ws/install/fast_lio/lib/fast_lio/fastlio_mapping','--ros-args','--params-file',str(out/'config.yaml')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
t0=time.monotonic();metrics={'mapping_pid':proc.pid}
try:
 time.sleep(3)
 if proc.poll() is not None:raise RuntimeError('Mapping failed to start')
 subprocess.run(['python3','/work/deploy/monitor_live.py',str(out/'monitor'),'--seconds',str(args.seconds)],env=env,check=True,stdout=(out/'monitor.log').open('w'))
 if proc.poll() is not None:raise RuntimeError('Mapping exited during monitoring')
 stat=Path(f'/proc/{proc.pid}/stat').read_text().split();metrics['cpu_seconds']=(int(stat[13])+int(stat[14]))/os.sysconf('SC_CLK_TCK')
 metrics['elapsed_seconds']=time.monotonic()-t0
 metrics['cpu_percent_one_core']=100*metrics['cpu_seconds']/metrics['elapsed_seconds']
 for line in Path(f'/proc/{proc.pid}/status').read_text().splitlines():
  if line.startswith(('VmRSS:','VmHWM:','Threads:')):metrics[line.split(':')[0]]=line.split(':')[1].strip()
finally:
 if proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
 metrics['exit_code']=proc.returncode;log.close()
 path=Path('/work/ws/src/fast_lio/Log/fast_lio_time_log.csv')
 if path.exists():shutil.copy2(path,out/'compute_timing.csv')
 (out/'process.json').write_text(json.dumps(metrics,indent=2))
 print(json.dumps(metrics,indent=2))
print((out/'monitor/summary.json').read_text())
