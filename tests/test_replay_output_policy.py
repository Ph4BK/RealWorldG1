"""Exercise replay output modes with synthetic ROS callbacks and fake child processes."""
import contextlib,io,json,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch,MagicMock
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import replay_fastlio_bag as replay

class OutputPolicy(unittest.TestCase):
    def run_mode(self,localization,debug=False):
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'result';callbacks={};commands=[];clock=[0.];save_calls=[]
            node=MagicMock()
            node.create_subscription.side_effect=lambda typ,topic,callback,*a:callbacks.update({topic:callback})
            client=MagicMock();node.create_client.return_value=client;client.wait_for_service.return_value=True
            def save(_):
                save_calls.append(True);(out/'map_raw.pcd').write_bytes(b'POINTS 1\nDATA ascii\n0 0 0\n')
                return SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(success=True))
            client.call_async.side_effect=save
            def popen(cmd,**kwargs):
                commands.append(cmd)
                is_player='play' in cmd
                return SimpleNamespace(poll=lambda:0 if is_player else None,returncode=0,pid=12345,wait=lambda timeout:0)
            def monotonic():clock[0]+=.05;return clock[0]
            def spin(*a,**k):
                callbacks['/Odometry_loc'](Odometry())
                callbacks['/localization_3d'](PoseStamped())
                if '/cloud_registered_1' in callbacks:
                    callbacks['/cloud_registered_1'](create_cloud_xyz32(Header(),np.array([[1.,2.,3.]],dtype=np.float32)))
            argv=['replay',str(Path(tmp)/'bag'),str(out)]
            if localization:argv+=['--localization-config',str(Path(tmp)/'loc.yaml')]
            if debug:argv+=['--save-debug-artifacts']
            with contextlib.ExitStack() as stack:
                preview=stack.enter_context(patch.object(replay,'write_localization_preview',return_value='localization_bev.png'))
                stack.enter_context(patch.object(sys,'argv',argv))
                stack.enter_context(patch.dict(replay.os.environ,{},clear=False))
                stack.enter_context(patch.object(replay.subprocess,'Popen',side_effect=popen))
                stack.enter_context(patch.object(replay.os,'killpg'))
                stack.enter_context(patch.object(replay.time,'monotonic',side_effect=monotonic))
                stack.enter_context(patch.object(replay.time,'sleep'))
                for name in ['init','shutdown','spin_until_future_complete']:stack.enter_context(patch.object(replay.rclpy,name))
                stack.enter_context(patch.object(replay.rclpy,'create_node',return_value=node))
                stack.enter_context(patch.object(replay.rclpy,'spin_once',side_effect=spin))
                stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                replay.main()
            self.assertEqual(preview.call_count,int(localization))
            detailed=not localization or debug
            for name in ['registered_xyz.npy','map_raw.pcd','odometry.csv','output_timing.csv']:
                self.assertEqual((out/name).exists(),detailed,name)
            self.assertTrue((out/'localization.csv').exists())
            self.assertEqual(bool(save_calls),detailed)
            self.assertEqual('/cloud_registered_1' in callbacks,detailed)
            summary=json.loads((out/'summary.json').read_text());self.assertTrue(summary['success']);self.assertGreaterEqual(summary['odometry_poses'],10)
            mapping=commands[0]
            self.assertEqual('pcd_save.pcd_save_en:=false' in mapping,not detailed)
            if localization:
                loc_command=next(c for c in commands if 'global_localization_node' in c)
                self.assertIn('__node:=global_localization_node',loc_command)
                self.assertEqual('save_scan:=false' in loc_command,not detailed)
    def test_localization_slim(self):self.run_mode(True)
    def test_localization_debug(self):self.run_mode(True,True)
    def test_mapping_unchanged(self):self.run_mode(False)
if __name__=='__main__':unittest.main()
