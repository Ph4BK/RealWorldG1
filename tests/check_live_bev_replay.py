"""Opt-in ROS end-to-end check using the retained relocalization_1 bag.
Run with sourced ROS/workspace, isolated ROS_DOMAIN_ID and localhost discovery.
Creates only /tmp diagnostics; never sends motion commands or overwrites datasets.
"""
import json,os,sys,time,tempfile,subprocess,signal
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import rclpy
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'tools'))
from live_bev_node import LiveBEV
from std_msgs.msg import String
if os.environ.get('ROS_DOMAIN_ID') in (None, '0') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
    raise SystemExit('Set a nonzero isolated ROS_DOMAIN_ID and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
run=Path(tempfile.mkdtemp(prefix='g1-live-bev-replay-'))
rclpy.init();node=LiveBEV(SimpleNamespace(config=root/'fastlio/config/live_bev.yaml',map_dir=root/'maps/full_lab_v1',run_dir=run))
node.active=True
registrations=[];changes=[];messages=set()
node.create_subscription(String,'/g1/localization_update',lambda msg:registrations.append(json.loads(msg.data)),10)
log=(run/'replay.log').open('w')
child=subprocess.Popen(['/usr/bin/python3',str(root/'tools/replay_fastlio_bag.py'),str(root/'datasets/relocalization_1/bag'),str(run/'replay'),'--rate','1','--localization-config',str(root/'datasets/relocalization_1/initial_pose/localization.yaml')],stdout=log,stderr=log,start_new_session=True)
try:
    deadline=time.monotonic()+110
    while child.poll() is None and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.02)
        messages.add(node.message)
        changes.append(int(np.count_nonzero(node.core.task!=node.core.static)))
    if child.poll() is None: raise RuntimeError('Replay timeout')
    print(json.dumps(dict(run=str(run),exit=child.returncode,updates=node.updates,registrations=len(registrations),max_changed_cells=max(changes),last_message=node.message,fitness=[r['fitness'] for r in registrations]),indent=2),flush=True)
    assert child.returncode==0,(run/'replay.log').read_text()[-3000:]
    assert registrations,'Missing actual ICP notifications'
    assert node.updates>20,repr(messages)
    assert max(changes)>0
finally:
    if child.poll() is None:
        os.killpg(child.pid,signal.SIGINT)
        try:child.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM)
    log.close();node.destroy_node();rclpy.shutdown()
