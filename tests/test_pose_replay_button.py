"""ROS service integration test; substitutes playback process, never replays a bag."""
import json,os,signal,subprocess,sys,tempfile,time,unittest
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from g1_pose_init import replay_error

class ButtonTest(unittest.TestCase):
    def test_missing_or_invalid_planar_pose(self):
        for pose in [None,{},dict(x=None,y=0,yaw_deg=0),dict(x=0,y=0,yaw_deg=float('nan'))]:
            self.assertIn('draw an arrow',replay_error(pose,False,Path('/unused')))
        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(replay_error(dict(x=0.,y=0.,yaw_deg=0.),False,Path(d)/'new'))
            self.assertIsNone(replay_error(dict(x=0.,y=0.,yaw_deg=0.),False,Path(d)))
            self.assertIn('already running',replay_error(dict(x=0.,y=0.,yaw_deg=0.),True,Path(d)/'new'))
    def test_button_only_start_and_reload(self):
        with tempfile.TemporaryDirectory() as d:
            d=Path(d);np.save(d/'map.npy',np.zeros((1,3),dtype=np.float32))
            harness=d/'harness.py'
            harness.write_text('''import sys,subprocess,json
from pathlib import Path
sys.path.insert(0,sys.argv.pop(1))
import g1_pose_init
real=subprocess.Popen
calls=Path(sys.argv.pop(1))
def fake(command,**kwargs):
 calls.write_text(json.dumps(command))
 return real(['/usr/bin/python3','-c','import time; time.sleep(30)'],**kwargs)
g1_pose_init.subprocess.Popen=fake
g1_pose_init.main()
''')
            rclpy.init();node=rclpy.create_node('button_integration_check');client=node.create_client(Trigger,'/g1/start_replay');pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose_2d',10)
            def request():
                f=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=5);self.assertTrue(f.done());return f.result()
            try:
                for attempt in range(2):
                    log=(d/f'run{attempt}.log').open('w')
                    child=subprocess.Popen(['/usr/bin/python3',str(harness),str(ROOT/'tools'),str(d/'calls.json'),'--bag',str(ROOT/'datasets/relocalization_1/bag'),'--map-dir',str(ROOT/'maps/full_lab_v1'),'--map-xyz',str(d/'map.npy'),'--output',str(d/'selection'),'--replay-output',str(d/'replay')],stdout=log,stderr=log)
                    try:
                        deadline=time.monotonic()+15
                        while time.monotonic()<deadline:
                            rclpy.spin_once(node,timeout_sec=.1)
                            if 'READY' in (d/f'run{attempt}.log').read_text() and client.service_is_ready():break
                        self.assertIsNone(child.poll())
                        if attempt==0:
                            result=request();self.assertFalse(result.success);self.assertIn('draw an arrow',result.message);self.assertFalse((d/'calls.json').exists())
                            m=PoseWithCovarianceStamped();m.header.frame_id='map';m.pose.pose.position.x=3.;m.pose.pose.position.y=-2.;m.pose.pose.orientation.w=1.;pub.publish(m)
                            deadline=time.monotonic()+5
                            while not (d/'selection/initial_pose.json').exists() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
                            self.assertTrue((d/'selection/initial_pose.json').exists());self.assertFalse((d/'calls.json').exists(),'Drawing arrow must not start playback')
                        (d/'replay').mkdir(exist_ok=True)
                        (d/'replay/stale.csv').write_text('old output')
                        result=request();self.assertTrue(result.success,result.message)
                        self.assertFalse((d/'replay/stale.csv').exists())
                        self.assertTrue((d/'selection/initial_pose.json').exists())
                        command=json.loads((d/'calls.json').read_text());self.assertEqual(command[command.index('--rate')+1],'1')
                        result=request();self.assertFalse(result.success);self.assertIn('already running',result.message)
                    finally:
                        child.send_signal(signal.SIGTERM);child.wait(timeout=15);log.close()
                        self.assertEqual(child.returncode,0,(d/f'run{attempt}.log').read_text())
                    (d/'calls.json').unlink()
                    for _ in range(10):rclpy.spin_once(node,timeout_sec=.1)
            finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':unittest.main()
