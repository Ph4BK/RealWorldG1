from contextlib import redirect_stdout, redirect_stderr
import io
import json
import unittest
from unittest.mock import patch
from RealWorldG1.tools.calibrate_g1_motion import main
from RealWorldG1.src.g1_motion_executor import G1MotionError


class CalibrationCliTests(unittest.TestCase):
    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_read_only(self, backend):
        backend.return_value.read_odom.return_value={'odom_received':True,'odom':{'x':1}}
        code, rows=self.call(['--odom'])
        self.assertEqual(code,0)
        self.assertEqual(rows[0]['odom']['x'],1)
        backend.return_value.read_odom.assert_called_once_with()
        backend.return_value.pulse.assert_not_called()

    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_read_only_rejects_motion_arguments(self, backend):
        for flag in ('--speed','--vy','--yaw-rate','--duration'):
            with self.assertRaises(SystemExit): self.call(['--odom',flag,'0'])
        backend.assert_not_called()

    def call(self, args):
        output=io.StringIO()
        self.errors=io.StringIO()
        with redirect_stdout(output), redirect_stderr(self.errors):
            code=main(args)
        return code, [json.loads(line) for line in output.getvalue().splitlines()]

    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_four_directions_execute_once(self, backend):
        backend.return_value.pulse.return_value={'command_completed': True}
        for flag,value,vx,yaw in [('--speed','.2',.2,0.),('--vx','-.2',-.2,0.),('--yaw-rate','.15',0.,.15),('--omega','-.15',0.,-.15)]:
            backend.reset_mock()
            code,rows=self.call([flag,value,'--duration','2'])
            self.assertEqual(code,0)
            self.assertEqual(rows, [])
            self.assertEqual(self.errors.getvalue(), '')
            backend.assert_called_once_with()
            backend.return_value.pulse.assert_called_once_with(vx=vx,vy=0.,yaw_rate=yaw,duration=2.)

    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_lateral_and_combined_translation(self, backend):
        backend.return_value.pulse.return_value={'command_completed': True}
        for flag in ('--lateral-speed','--vy'):
            for vy in (-.03,.03):
                for vx in (0.,.2):
                    backend.reset_mock()
                    code,_=self.call([flag,str(vy),'--speed',str(vx),'--duration','2'])
                    self.assertEqual(code,0)
                    backend.return_value.pulse.assert_called_once_with(vx=vx,vy=vy,yaw_rate=0.,duration=2.)

    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_removed_options_rejected_before_backend(self, backend):
        for option in ('--execute','--log','--dry-run','--duration-only-test'):
            with self.assertRaises(SystemExit):
                self.call(['--speed','.2','--duration','2',option])
        backend.assert_not_called()

    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_invalid_input(self, backend):
        for args in ([],['--speed','.2'],['--speed','nan','--duration','2'],['--speed','0','--duration','2'],
                     ['--yaw-rate','inf','--duration','2'],['--speed','.1','--yaw-rate','.1','--duration','2'],
                     ['--speed','.2','--duration','0'], ['--vy','nan','--duration','2'],
                     ['--vy','.1','--yaw-rate','.1','--duration','2'], ['--duration','2']):
            with self.assertRaises(SystemExit): self.call(args)
        backend.assert_not_called()

    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_no_application_caps_mock_only(self, backend):
        backend.return_value.pulse.return_value={'command_completed': True}
        for flag in ('--speed','--yaw-rate'):
            code,_=self.call([flag,'1.5','--duration','10'])
            self.assertEqual(code,0)

    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_failure_no_retry(self, backend):
        backend.return_value.pulse.side_effect=G1MotionError({'error':'wrong FSM'})
        code,_=self.call(['--speed','.2','--duration','2'])
        self.assertEqual(code,1)
        self.assertIn('wrong FSM',self.errors.getvalue())
        backend.return_value.pulse.assert_called_once()

    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_launch_error_visible_without_calibration(self, backend):
        backend.return_value.pulse.side_effect=OSError('docker missing')
        code,rows=self.call(['--speed','.3','--duration','2'])
        self.assertEqual(code,1)
        self.assertEqual(rows,[])
        self.assertIn('docker missing',self.errors.getvalue())

    @patch('RealWorldG1.src.g1_motion_executor.subprocess.run')
    def test_wsl_docker_failure_end_to_end(self, run):
        import subprocess
        run.return_value=subprocess.CompletedProcess([],1,"The command 'docker' could not be found in this WSL 2 distro.\n",'')
        code,rows=self.call(['--speed','.3','--duration','2'])
        self.assertEqual((code,rows),(1,[]))
        self.assertIn('不会自动 SSH',self.errors.getvalue())
        self.assertIn('运控程序未启动',self.errors.getvalue())
        run.assert_called_once()

    @patch('RealWorldG1.src.motion_calibration.save_calibration')
    @patch('RealWorldG1.src.motion_calibration.prepare_output')
    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_opt_in_and_save(self, backend, prepare, save):
        prepare.return_value='/tmp/example'
        save.return_value={'files': {'json':'/tmp/example/result.json'}}
        for flag, value in [('--speed','.2'),('--yaw-rate','.15'),('--yaw-rate','-.15')]:
            backend.reset_mock()
            code,rows=self.call([flag,value,'--duration','2','--calibrate'])
            self.assertEqual(code,0)
            self.assertTrue(backend.return_value.pulse.call_args.kwargs['calibrate'])
            self.assertIn('files',rows[0])
            save.assert_called_with(backend.return_value.pulse.return_value,prepare.return_value)

    @patch('RealWorldG1.src.motion_calibration.prepare_output')
    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_normal_never_prepares_recording(self, backend, prepare):
        code,rows=self.call(['--speed','.2','--duration','2'])
        self.assertEqual((code,rows),(0,[]))
        prepare.assert_not_called()

    @patch('RealWorldG1.src.motion_calibration.prepare_output', side_effect=OSError('output not writable'))
    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_unwritable_output_prevents_motion(self, backend, prepare):
        code,_=self.call(['--speed','.2','--duration','2','--calibrate'])
        self.assertEqual(code,1)
        backend.assert_not_called()

    @patch('RealWorldG1.src.motion_calibration.plot_calibrations')
    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_plot_only_never_constructs_backend(self, backend, plot):
        from pathlib import Path
        plot.return_value={'counts':{'forward':3},'files':{'png':'summary.png'}}
        for args in (['--plot'], ['--plot','/tmp/run1','/tmp/run2','--plot-output','/tmp/summary.png']):
            code,rows=self.call(args)
            self.assertEqual(code,0)
            self.assertEqual(rows[0]['counts']['forward'],3)
        plot.assert_called_with([Path('/tmp/run1'),Path('/tmp/run2')],Path('/tmp/summary.png'))
        backend.assert_not_called()

    @patch('RealWorldG1.src.motion_calibration.plot_calibrations')
    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_plot_rejects_all_motion_options_even_zero(self, backend, plot):
        for flag in ('--speed','--vx','--vy','--lateral-speed','--yaw-rate','--omega','--duration'):
            with self.assertRaises(SystemExit): self.call(['--plot',flag,'0'])
        for flag in ('--calibrate','--odom'):
            with self.assertRaises(SystemExit): self.call(['--plot',flag])
        with self.assertRaises(SystemExit): self.call(['--plot-output','/tmp/a.png','--speed','.3','--duration','2'])
        backend.assert_not_called(); plot.assert_not_called()

    @patch('RealWorldG1.src.motion_calibration.plot_calibrations', side_effect=ValueError('No saved experiments'))
    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_plot_failure_never_falls_through_to_motion(self, backend, plot):
        code,rows=self.call(['--plot'])
        self.assertEqual((code,rows),(1,[]))
        self.assertIn('No saved experiments',self.errors.getvalue())
        backend.assert_not_called()

    @patch('RealWorldG1.tools.calibrate_g1_motion.G1MotionExecutor')
    def test_calibration_rejects_unsupported_actions(self, backend):
        for args in (['--odom'],['--speed','-.2','--duration','2'],['--vy','.03','--duration','2']):
            with self.assertRaises(SystemExit): self.call(args+['--calibrate'])
        backend.assert_not_called()
