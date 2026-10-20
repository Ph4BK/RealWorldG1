import subprocess
import unittest
from unittest.mock import patch
from RealWorldG1.src.g1_motion_executor import G1MotionExecutor, G1MotionError


class ExecutorTests(unittest.TestCase):
    @patch('RealWorldG1.src.g1_motion_executor.subprocess.run')
    def test_read_odom(self, run):
        run.return_value = subprocess.CompletedProcess([], 0, 'RESULT {"odom_received":true,"odom":{"x":1}}\n', '')
        self.assertEqual(G1MotionExecutor().read_odom()['odom']['x'],1)
        self.assertEqual(run.call_args.args[0][-1], '--odom')

    @patch('RealWorldG1.src.g1_motion_executor.subprocess.run')
    def test_direct_execution(self, run):
        run.return_value = subprocess.CompletedProcess([], 0, 'setup\nRESULT {"command_completed":true}\n', '')
        self.assertTrue(G1MotionExecutor().move()['command_completed'])
        self.assertEqual(run.call_args.args[0][-4:], ['0.2', '0.0', '0.0', '1.0'])

    @patch('RealWorldG1.src.g1_motion_executor.subprocess.run')
    def test_explicit_execution_and_turn(self, run):
        run.return_value = subprocess.CompletedProcess([], 0, 'RESULT {"command_completed":true}\n', '')
        self.assertTrue(G1MotionExecutor().turn(-.1, .5)['command_completed'])
        self.assertEqual(run.call_args.args[0][-4:], ['0.0', '0.0', '-0.1', '0.5'])

    @patch('RealWorldG1.src.g1_motion_executor.subprocess.run')
    def test_combined_translation(self, run):
        run.return_value = subprocess.CompletedProcess([], 0, 'RESULT {"command_completed":true}\n', '')
        G1MotionExecutor().move(.2, 2., vy=-.03)
        self.assertEqual(run.call_args.args[0][-4:], ['0.2', '-0.03', '0.0', '2.0'])

    def test_limits(self):
        for args in ({'duration':-1}, {'duration':0}, {'vx':float('nan')}, {'vy':float('nan')}, {'vy':.1,'yaw_rate':.1}, {'yaw_rate':float('inf')}, {'vx':.1,'yaw_rate':.1}):
            with self.assertRaises(ValueError): G1MotionExecutor().pulse(**args)

    @patch('RealWorldG1.src.g1_motion_executor.subprocess.run')
    def test_guard_is_failure_and_lock_released(self, run):
        run.return_value = subprocess.CompletedProcess([], 1, 'RESULT {"error":"guard"}\n', '')
        executor=G1MotionExecutor()
        with self.assertRaises(G1MotionError): executor.move()
        self.assertFalse(executor._lock.locked())

    @patch('RealWorldG1.src.g1_motion_executor.subprocess.run')
    def test_missing_report(self, run):
        run.return_value = subprocess.CompletedProcess([], 1, '', 'docker unavailable')
        with self.assertRaises(G1MotionError): G1MotionExecutor().move()

    def test_no_overlap(self):
        executor=G1MotionExecutor()
        executor._lock.acquire()
        with self.assertRaises(RuntimeError): executor.move()

    @patch('RealWorldG1.src.g1_motion_executor.subprocess.run')
    def test_calibration_flag(self, run):
        run.return_value=subprocess.CompletedProcess([],0,'RESULT {"command_completed":true}\n','')
        G1MotionExecutor().pulse(vx=.2,duration=2.,calibrate=True)
        self.assertEqual(run.call_args.args[0][-5:],['0.2','0.0','0.0','2.0','--calibrate'])

    def test_calibration_action_validation(self):
        for args in ({'vx':-.2},{'vy':.1},{'vx':0}):
            with self.assertRaises(ValueError): G1MotionExecutor().pulse(**args,calibrate=True)


if __name__=='__main__': unittest.main()
