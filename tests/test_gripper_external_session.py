"""Existing body test fixtures plus a fake loop; no end-effector wire traffic."""
import signal
import unittest
from unittest.mock import patch

from examples import external_control as body
from test_examples import FakeClock, RecordingController
from locomotion_aorta import BODY_CMD_TOPIC

from examples.gripper_external import run_external


class ExternalLifecycleTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.events = []

    def controller(self, **options):
        c = RecordingController(self.clock, **options)
        self.addCleanup(c.close)
        return c

    def run_loop(self, controller, loop, **options):
        return run_external(controller, loop, body=body, timeout=1,
                            sleep=self.clock.sleep, monotonic=self.clock.monotonic,
                            emit=self.events.append, **options)

    def test_held_body_pose_and_zero_gain_option_leave_no_new_target(self):
        for gains in ('hold', 'zero'):
            with self.subTest(gains=gains):
                c = self.controller()
                callbacks = []
                def loop(frame):
                    callbacks.append(frame)
                    for _ in range(4):
                        frame()
                self.run_loop(c, loop, body_gains=gains)
                commands = [value for topic, value in c.transport.published if topic == BODY_CMD_TOPIC]
                self.assertEqual(len(commands), 4)
                self.assertEqual({topic for topic, _ in c.transport.published},
                                 {BODY_CMD_TOPIC, '/sdk/foot_humanoid/control/request'})
                for command in commands:
                    self.assertEqual(len(command.joints), 31)
                    self.assertEqual([j.q for j in command.joints],
                                     [0.0 if i == 14 else .1 for i in range(31)])
                    if gains == 'zero':
                        self.assertTrue(all((j.kp, j.kd, j.dq, j.tau) == (0., 0., 0., 0.)
                                            for j in command.joints))
                self.assertIn('exit_external', c.calls)
                with self.assertRaisesRegex(RuntimeError, 'no longer active'):
                    callbacks[0]()

    def test_loop_exception_stops_frames_and_cancels(self):
        c = self.controller()
        def loop(frame):
            frame()
            raise ValueError('target loop failed')
        with self.assertRaisesRegex(ValueError, 'target loop failed'):
            self.run_loop(c, loop)
        self.assertEqual(c.command_count, 1)
        self.assertIn('exit_external', c.calls)
        self.assertTrue(any('confirmed PASSIVE' in e for e in self.events))

    def test_interrupt_after_entry_also_cancels(self):
        c = self.controller()
        enter = c.enter_external
        def interrupted(timeout):
            enter(timeout)
            raise KeyboardInterrupt('SIGINT')
        with patch.object(c, 'enter_external', side_effect=interrupted):
            with self.assertRaises(KeyboardInterrupt):
                self.run_loop(c, lambda frame: frame())
        self.assertEqual(c.command_count, 0)
        self.assertIn('exit_external', c.calls)

    def test_invalid_gains_fails_before_entry_or_loop(self):
        c = self.controller()
        with self.assertRaisesRegex(ValueError, 'body_gains'):
            self.run_loop(c, lambda _frame: self.fail('loop ran'), body_gains='invalid')
        self.assertNotIn('enter_external', c.calls)
        self.assertEqual(c.command_count, 0)

    def test_body_drift_is_observed_but_reject_changes_abort_then_cancel(self):
        for options in ({'drift_command_range': (1, 3)}, {'reject_after_commands': 1}):
            with self.subTest(options=options):
                c = self.controller(**options)
                def loop(frame):
                    for _ in range(10):
                        frame()
                if 'reject_after_commands' in options:
                    with self.assertRaisesRegex(RuntimeError, 'reject_count'):
                        self.run_loop(c, loop)
                    self.assertLess(c.command_count, 10)
                else:
                    self.run_loop(c, loop)
                    self.assertEqual(c.command_count, 10)
                    self.assertTrue(any('BODY_HOLD summary' in e and '0.200000' in e
                                        for e in self.events))
                self.assertEqual(c.calls.count('exit_external'), 1)

    def test_stale_body_state_aborts_then_cancels(self):
        c = self.controller(stale_after_commands=2)
        def loop(frame):
            for _ in range(200):
                frame()
        with self.assertRaisesRegex(RuntimeError, 'body lowstate became stale'):
            self.run_loop(c, loop)
        self.assertLess(c.command_count, 200)
        self.assertIn('exit_external', c.calls)

    def test_cancel_failure_is_reported_and_not_claimed_passive(self):
        c = self.controller()
        with patch.object(c, 'exit_external', side_effect=RuntimeError('CANCEL transport failed')):
            with self.assertRaisesRegex(RuntimeError, 'CANCEL transport failed'):
                self.run_loop(c, lambda frame: frame())
        self.assertTrue(any('CANCEL UNCONFIRMED' in e for e in self.events))
        self.assertFalse(any('confirmed PASSIVE' in e for e in self.events))

    def test_signal_handler_does_not_interrupt_cancel_and_is_restored(self):
        c = self.controller()
        previous = signal.getsignal(signal.SIGINT)
        exit_external = c.exit_external
        seen = []
        def cancel(timeout):
            seen.append(signal.getsignal(signal.SIGINT))
            return exit_external(timeout)
        def loop(frame):
            frame()
            signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
        with patch.object(c, 'exit_external', side_effect=cancel):
            with self.assertRaises(KeyboardInterrupt):
                self.run_loop(c, loop)
        self.assertEqual(seen, [signal.SIG_IGN])
        self.assertIs(signal.getsignal(signal.SIGINT), previous)


if __name__ == '__main__':
    unittest.main(verbosity=2)
