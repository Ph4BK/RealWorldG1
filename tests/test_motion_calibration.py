import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from RealWorldG1.src.motion_calibration import analyze_calibration, prepare_output, save_calibration, wrap, load_calibrations, plot_calibrations


def fixture(action='forward', heading=0.0):
    expiry=10_000_000_000
    poses=[]
    sign=1 if action=='left_turn' else -1 if action=='right_turn' else 0
    # Intentionally no source samples exactly on either endpoint.
    for index in range(-1,22):
        t=index*.1+.03
        yaw=wrap(heading+sign*.2*t)
        travel=.1*t if action=='forward' else 0.
        stamp=expiry+round(t*1e9)
        poses.append(dict(stamp_ns=stamp,received_ros_ns=stamp+100_000_000,
                          frame_id='map',x=travel*math.cos(heading),y=travel*math.sin(heading),yaw_rad=yaw))
    return dict(command_completed=True,zero_velocity_sent=False,vx=.2 if not sign else 0.,vy=0.,
                yaw_rate=sign*.3,duration=2.,
                calibration=dict(action=action,expiry_ros_ns=expiry,poses=poses,
                                 pose_topic='/g1/live_bev/robot',time_basis='fixture'))


class CalibrationAnalysisTests(unittest.TestCase):
    def test_forward_projection_and_source_time_velocity(self):
        result=analyze_calibration(fixture(heading=math.pi/2))
        self.assertAlmostEqual(result['residual']['forward_m'],.2)
        self.assertAlmostEqual(result['pose_0s']['y'],0)
        self.assertAlmostEqual(result['pose_2s']['y'],.2)
        self.assertEqual(result['pose_0s']['interpolation']['before_offset_s'],-.07)
        for row in result['velocities']:
            self.assertAlmostEqual(row['speed_m_s'],.1)
            self.assertAlmostEqual(row['forward_m_s'],.1)
            self.assertAlmostEqual(row['yaw_rate_rad_s'],0)
        self.assertEqual(result['velocities'][0]['t_start_s'],0)
        self.assertEqual(result['velocities'][-1]['t_end_s'],2)

    def test_both_turns_and_angle_wrap(self):
        for action,sign in [('left_turn',1),('right_turn',-1)]:
            result=analyze_calibration(fixture(action,heading=sign*3.1))
            self.assertAlmostEqual(result['residual']['yaw_rad'],sign*.4)
            self.assertAlmostEqual(result['residual']['yaw_along_command_rad'],.4)
            for row in result['velocities']:
                self.assertAlmostEqual(row['yaw_rate_rad_s'],sign*.2)

    def test_reject_incomplete_stale_reordered_gapped_and_fallback(self):
        base=fixture()
        cases=[]
        for key,value in [('command_completed',False),('zero_velocity_sent',True)]:
            case=copy.deepcopy(base); case[key]=value; cases.append(case)
        case=copy.deepcopy(base); case['calibration']['poses']=case['calibration']['poses'][1:]; cases.append(case)
        case=copy.deepcopy(base); case['calibration']['poses']=case['calibration']['poses'][:-2]; cases.append(case)
        case=copy.deepcopy(base); del case['calibration']['poses'][5:12]; cases.append(case)
        for field,value in [('stamp_ns',1),('received_ros_ns',999_000_000_000),('frame_id','odom'),('x',float('nan'))]:
            case=copy.deepcopy(base); case['calibration']['poses'][4][field]=value; cases.append(case)
        for case in cases:
            with self.subTest(case=case):
                with self.assertRaises(ValueError): analyze_calibration(case)

    def test_record_without_matplotlib_or_png(self):
        import builtins
        original_import=builtins.__import__
        def no_plotting(name,*args,**kwargs):
            if name.startswith('matplotlib'):
                raise ImportError('matplotlib deliberately unavailable')
            return original_import(name,*args,**kwargs)
        with tempfile.TemporaryDirectory() as temp:
            with patch('builtins.__import__',side_effect=no_plotting):
                directory=prepare_output(Path(temp))
                result=save_calibration(fixture('right_turn'),directory)
            data=json.loads(Path(result['files']['json']).read_text())
            self.assertEqual(data['action'],'right_turn')
            self.assertIn('poses',data['raw_report']['calibration'])
            self.assertNotIn('png',result['files'])
            self.assertEqual({p.name for p in directory.iterdir()},{'result.json','velocity.csv'})
            self.assertIn('yaw_rate_rad_s',Path(result['files']['csv']).read_text())

    def test_discover_deduplicate_and_validate(self):
        with tempfile.TemporaryDirectory() as temp:
            directory=prepare_output(Path(temp))
            result=save_calibration(fixture(),directory)
            path=Path(result['files']['json'])
            self.assertEqual(len(load_calibrations([temp,directory,path])),1)
            data=json.loads(path.read_text()); data['velocities'][0]['speed_m_s']=float('nan')
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError,'Invalid experiment'): load_calibrations([temp])
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(ValueError,'No saved experiments'): load_calibrations([temp])
            with self.assertRaises(ValueError): load_calibrations([Path(temp)/'missing'])

    def test_grouped_plot_and_selected_trials(self):
        try:
            import matplotlib
        except ImportError:
            self.skipTest('matplotlib not installed in this interpreter')
        with tempfile.TemporaryDirectory() as temp:
            directories=[]
            for action in ('forward','forward','left_turn','right_turn'):
                directory=prepare_output(Path(temp))
                save_calibration(fixture(action),directory); directories.append(directory)
            original={str(p):p.read_bytes() for p in Path(temp).rglob('result.json')}
            result=plot_calibrations([temp],Path(temp)/'summary.png')
            self.assertEqual(result['counts'],dict(forward=2,left_turn=1,right_turn=1))
            self.assertTrue(Path(result['files']['png']).read_bytes().startswith(b'\x89PNG\r\n\x1a\n'))
            manifest=json.loads(Path(result['files']['manifest']).read_text())
            self.assertEqual(len(manifest['trials']),4)
            self.assertEqual({r['result_json'] for r in manifest['trials']},set(original))
            selected=plot_calibrations([directories[0]],Path(temp)/'selected.png')
            self.assertEqual(selected['counts'],dict(forward=1,left_turn=0,right_turn=0))
            for path,content in original.items(): self.assertEqual(Path(path).read_bytes(),content)
            with self.assertRaises(ValueError): plot_calibrations([temp],directories[0]/'result.png')

    def test_signed_forward_plot_and_residual_exclude_lateral_motion(self):
        try:
            import matplotlib
        except ImportError:
            self.skipTest('matplotlib not installed in this interpreter')
        data=fixture()
        for pose in data['calibration']['poses']:
            t=(pose['stamp_ns']-data['calibration']['expiry_ros_ns'])/1e9
            pose['x']=-.1*t
            pose['y']=.8*t
        reduced=analyze_calibration(data)
        self.assertAlmostEqual(reduced['residual']['forward_m'],-.2)
        for row in reduced['velocities']:
            self.assertAlmostEqual(row['forward_m_s'],-.1)
            self.assertGreater(row['speed_m_s'],.8)
        with tempfile.TemporaryDirectory() as temp:
            save_calibration(data,prepare_output(Path(temp)))
            def inspect_figure(figure,*args,**kwargs):
                self.assertIn('Forward velocity',figure.axes[0].get_title())
                for value in figure.axes[0].lines[0].get_ydata():
                    self.assertAlmostEqual(value,-.1)
            with patch('matplotlib.figure.Figure.savefig',autospec=True,side_effect=inspect_figure) as save:
                result=plot_calibrations([temp],Path(temp)/'summary.png')
                save.assert_called_once()
            manifest=json.loads(Path(result['files']['manifest']).read_text())
            self.assertEqual(manifest['plotted_fields']['linear'],'forward_m_s')


if __name__=='__main__': unittest.main()
