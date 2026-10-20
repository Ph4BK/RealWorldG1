import subprocess
import sys
import threading
import unittest
from concurrent.futures import Future
from pathlib import Path

from RealWorldG1.src.motion_probe import MotionProbe, OdomSample, validate_pulse


class Fixture:
    def __init__(self):
        self.now = 0.0
        self.x = 0.0
        self.velocity = 0.0
        self.calls = []
        self.hook = lambda: None
        self.probe = MotionProbe(self.rpc, self.odom, clock=lambda: self.now, sleep=self.sleep)

    def sleep(self, seconds):
        self.x += self.velocity * seconds
        self.now += seconds
        self.hook()

    def odom(self):
        return OdomSample(self.x, 0, 0, abs(self.velocity), 0, self.now)

    def rpc(self, vx, omega, duration):
        self.calls.append((vx, omega, duration, self.now))
        self.velocity = vx
        future = Future()
        future.set_result("")
        return future


class MotionTests(unittest.TestCase):
    def test_success(self):
        f = Fixture()
        r = f.probe.run(0.05)
        self.assertEqual(r['errors'], [])
        self.assertTrue(r['motion_ack'] and r['stop_ack'] and r['stationary_observed'])
        self.assertAlmostEqual(r['dx'], 0.015)
        self.assertEqual(len(f.calls), 2)

    def test_limits(self):
        for values in [(float('nan'), 0, .3), (0, float('inf'), .3), (.11, 0, .3),
                       (0, .16, .3), (0, 0, 0), (0, 0, 1.01), (.05, .05, .3)]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                validate_pulse(*values)

    def test_one_second_pulse_stops_at_deadline(self):
        f = Fixture()
        r = f.probe.run(.1, duration=1.0)
        self.assertEqual(r['errors'], [])
        self.assertEqual(f.calls[0][:3], (.1, 0, 1.0))
        self.assertEqual(f.calls[1][:2], (0, 0))
        self.assertGreaterEqual(f.calls[1][3], 1.0)
        self.assertLess(f.calls[1][3], 1.011)

    def test_one_second_missing_ack_stops_early_on_error(self):
        f = Fixture()
        pending = Future()
        def rpc(*args):
            result = f.rpc(*args)
            return pending if len(f.calls) == 1 else result
        def hook():
            if f.now >= .8 and not pending.done():
                pending.set_exception(TimeoutError('response timeout'))
        f.probe.rpc = rpc
        f.hook = hook
        r = f.probe.run(.1, duration=1.0)
        self.assertTrue(r['errors'])
        self.assertTrue(r['stop_ack'])
        self.assertLess(f.calls[1][3], .82)

    def test_preflight_no_publication(self):
        for sample in [None, OdomSample(0, 0, 0, 0, 0, -1), OdomSample(0, 0, 0, .1, 0, 0)]:
            f = Fixture()
            f.probe.odom = lambda: sample
            self.assertTrue(f.probe.run(.05)['errors'])
            self.assertEqual(f.calls, [])

    def test_missing_ack_does_not_delay_stop(self):
        f = Fixture()
        def rpc(*args):
            result = f.rpc(*args)
            return Future() if len(f.calls) == 1 else result
        f.probe.rpc = rpc
        r = f.probe.run(.05)
        self.assertFalse(r['motion_ack'])
        self.assertTrue(r['stop_ack'])
        self.assertLessEqual(f.calls[1][3], .311)
        self.assertTrue(r['errors'])

    def test_rejected_or_failed_publication_still_stops(self):
        for publication_failure in [False, True]:
            f = Fixture()
            def rpc(*args):
                result = f.rpc(*args)
                if len(f.calls) == 1:
                    if publication_failure:
                        raise RuntimeError('publication failed')
                    result = Future()
                    result.set_exception(RuntimeError('7301'))
                return result
            f.probe.rpc = rpc
            r = f.probe.run(.05)
            self.assertTrue(r['errors'])
            self.assertTrue(r['stop_ack'])
            self.assertEqual(f.calls[-1][:2], (0, 0))
            self.assertNotIn('UnboundLocalError', str(r))

    def test_stale_during_motion(self):
        f = Fixture()
        f.probe.odom = lambda: OdomSample(0, 0, 0, 0, 0, 0)
        r = f.probe.run(.05)
        self.assertIn('stale', str(r['errors']))
        self.assertTrue(r['stop_ack'])
        self.assertFalse(r['stationary_observed'])

    def test_interrupt(self):
        f = Fixture()
        cancel = threading.Event()
        f.hook = cancel.set
        r = f.probe.run(.05, cancel=cancel)
        self.assertTrue(r['stop_ack'])
        self.assertIn('interrupted', str(r['errors']))

    def test_displacement_guard(self):
        f = Fixture()
        f.hook = lambda: setattr(f, 'x', .2)
        r = f.probe.run(.05)
        self.assertTrue(r['stop_ack'])
        self.assertIn('exceeded', str(r['errors']))

    def test_stop_failure(self):
        f = Fixture()
        def rpc(*args):
            result = f.rpc(*args)
            if len(f.calls) == 2:
                result = Future()
                result.set_exception(TimeoutError('stop timed out'))
            return result
        f.probe.rpc = rpc
        r = f.probe.run(.05)
        self.assertFalse(r['stop_ack'])
        self.assertTrue(r['errors'])

    def test_experiments_serialized(self):
        f = Fixture()
        def hook():
            with self.assertRaisesRegex(RuntimeError, 'active'):
                f.probe.run(.05)
        f.hook = hook
        self.assertEqual(f.probe.run(.05)['errors'], [])

    def test_cli_dry_run_and_noninteractive_refusal(self):
        script = Path(__file__).resolve().parents[1] / 'tools/test_g1_motion.py'
        for args, code in [([], 0), (['forward'], 0), (['turn'], 0), (['--execute'], 2)]:
            result = subprocess.run([sys.executable, str(script), *args], input='',
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, code, result.stderr)
            self.assertNotIn('ModuleNotFoundError', result.stderr)
            if code == 0:
                self.assertIn('DRY RUN', result.stdout)


if __name__ == '__main__':
    unittest.main()
