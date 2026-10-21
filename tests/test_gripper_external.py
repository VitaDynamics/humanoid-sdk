"""Gripper example logic only: fake feedback/control; no generated gripper bytes."""
from dataclasses import replace
import io
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from locomotion_aorta import BODY_CMD_TOPIC
from test_examples import FakeClock, RecordingController
from locomotion_aorta.gripper_types import GripperCmd, GripperHostTarget, GripperSample, GripperState
from test_gripper_client import info
from examples.gripper import gripper_external as demo


class FakeGripper:
    def __init__(self, *, echo=True, reject=False, inhibited=False, stale=False):
        self.side = 'right'
        self.session_id = 77
        self.info = info(2)
        self.commands = []
        self.echo, self.reject, self.inhibited, self.stale = echo, reject, inhibited, stale
        self._state = GripperState(10, 1, 2, ts_state_real_us=5000,
            sample=GripperSample(.05, -.25, 0x40), sample_valid=True,
            output_allowed=True, inhibit_reason=0)

    @property
    def state(self):
        if not self.stale:
            self._state = replace(self._state, state_id=self._state.state_id + 1)
        if self.commands and self.inhibited:
            self._state = replace(self._state, output_allowed=False, inhibit_reason=3)
        return self._state

    def wait_info(self, timeout):
        return self.info

    def wait_state(self, timeout):
        return self.state

    def publish_target(self, q, dq=0., *, torque_cmd_nm=None, input_state=None):
        command = GripperCmd(self.session_id, len(self.commands) + 1, 1, 2, q, dq, torque_cmd_nm,
                             input_state_id=input_state.state_id,
                             input_state_ts_real_us=input_state.ts_state_real_us)
        self.commands.append(command)
        if self.echo:
            self._state = replace(self._state,
                trace_session_id=self.session_id, trace_sequence=command.sequence,
                trace_cmd_id=command.cmd_id, ts_cmd_received_us=5100 + command.sequence,
                ts_cmd_accepted_us=0 if self.reject else 5150 + command.sequence)
        return command


class GripperExampleTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.events = []

    def controller(self):
        c = RecordingController(self.clock)
        self.addCleanup(c.close)
        return c

    def run_demo(self, c, client, **options):
        return demo.run(c, client, q=options.pop('q', .075), duration=options.pop('duration', .08),
                        timeout=1, sleep=self.clock.sleep, monotonic=self.clock.monotonic,
                        emit=self.events.append, **options)

    def test_trajectory_is_robot_intent_and_body_frames_keep_measured_pose(self):
        c, gripper = self.controller(), FakeGripper()
        self.run_demo(c, gripper, dq=0., torque_cmd_nm=None, body_gains='zero')
        self.assertTrue(gripper.commands)
        self.assertTrue(all((cmd.q, cmd.dq, cmd.torque_cmd_nm) == (.075, 0., None)
                            for cmd in gripper.commands))
        body_commands = [v for topic, v in c.transport.published if topic == BODY_CMD_TOPIC]
        self.assertTrue(body_commands)
        self.assertTrue(all(len(cmd.joints) == 31 for cmd in body_commands))
        self.assertTrue(all(j.q == (0. if i == 14 else .1) and
                            (j.kp, j.kd, j.dq, j.tau) == (0., 0., 0., 0.)
                            for cmd in body_commands for i, j in enumerate(cmd.joints)))
        self.assertIn('exit_external', c.calls)
        self.assertTrue(any('confirmed PASSIVE' in e for e in self.events))
        self.assertTrue(any('not inferred' in e for e in self.events))

    def test_transition_duration_formula_and_minimum_are_exact(self):
        self.assertAlmostEqual(demo._transition_duration(.01, .08, .02), 3.5)
        self.assertAlmostEqual(demo._transition_duration(.08, .01, .01), 7.)
        self.assertEqual(demo._transition_duration(.01, .02, .02, 1.), 1.)
        self.assertEqual(demo._transition_duration(.02, .02, 0., 1.), 1.)
        with self.assertRaisesRegex(ValueError, 'no finite transition'):
            demo._transition_duration(.01, .02, 0.)

    def test_each_target_leg_uses_host_target_rate_and_minimum_duration(self):
        for dq, minimum, expected in [(0., .08, (3.25, 2.)),
                                      (.01, .08, (6.5, 4.)),
                                      (0., 4., (4., 4.))]:
            with self.subTest(dq=dq, minimum=minimum):
                self.events.clear()
                c, gripper = self.controller(), FakeGripper()
                gripper._state = replace(gripper._state, host_target_valid=True,
                                        host_target=GripperHostTarget(.01, .5))
                self.run_demo(c, gripper, dq=dq, duration=minimum, return_to_start=True)
                legs = [line for line in self.events if line.startswith('gripper leg target=')]
                self.assertEqual(len(legs), 2)
                for line, seconds in zip(legs, expected):
                    actual = float(line.split('streaming_duration_s=')[1].split(';')[0])
                    self.assertAlmostEqual(actual, seconds)
                    self.assertIn('start=0.01 m', line)  # Measured sample remains0.05.
                self.assertEqual(gripper.commands[-1].q, .05)
                self.assertEqual(c.calls.count('exit_external'), 1)

    def test_unknown_sample_or_excessive_command_fails_before_lease(self):
        for changes, options in (({'sample_valid': False}, {}), ({}, {'q': .2}),
                                 ({}, {'dq': .03}), ({}, {'torque_cmd_nm': -2.})):
            with self.subTest(changes=changes, options=options):
                c, g = self.controller(), FakeGripper()
                g._state = replace(g._state, **changes)
                with self.assertRaises(ValueError):
                    self.run_demo(c, g, **options)
                self.assertNotIn('enter_external', c.calls)
                self.assertFalse(g.commands)

    def test_rejected_or_inhibited_command_aborts_and_cancels(self):
        for settings, message in (({'reject': True}, 'not accepted'),
                                  ({'inhibited': True}, 'inhibited')):
            with self.subTest(settings=settings):
                c, g = self.controller(), FakeGripper(**settings)
                with self.assertRaisesRegex(RuntimeError, message):
                    self.run_demo(c, g)
                self.assertEqual(len(g.commands), 1)
                self.assertIn('exit_external', c.calls)

    def test_missing_trace_or_stale_feedback_aborts_and_cancels_without_retry(self):
        for settings, error in (({'echo': False}, TimeoutError), ({'stale': True}, RuntimeError)):
            with self.subTest(settings=settings):
                c, g = self.controller(), FakeGripper(**settings)
                with self.assertRaises(error):
                    self.run_demo(c, g, duration=.4)
                self.assertEqual(c.calls.count('enter_external'), 1)
                self.assertEqual(c.calls.count('exit_external'), 1)

    def test_feedback_keeps_host_and_trace_keys_separate_and_unknown_zero(self):
        g = FakeGripper()
        g._state = replace(g._state, sample_valid=False, sample=GripperSample(math.nan, math.nan, 0),
                           host_target=GripperHostTarget(.06, .5), host_target_valid=True,
                           host_target_session_id=99, host_target_sequence=8,
                           trace_session_id=77, trace_sequence=10, ts_cmd_received_us=200)
        demo.print_feedback(g, self.events.append)
        text = self.events[-1]
        self.assertIn('measured_opening_m=None', text)
        self.assertIn('host_key=99/8/', text)
        self.assertIn('trace_received_key=77/10/', text)
        self.assertIn('NOT_MEASURED', text)
        self.assertEqual(g.state.ts_cmd_write_ack_us, 0)

    def test_default_and_dryrun_are_readonly_without_controller(self):
        for flags in ([], ['--dry-run']):
            with self.subTest(flags=flags):
                g = FakeGripper()
                context = Mock()
                context.__enter__ = Mock(return_value=g)
                context.__exit__ = Mock(return_value=False)
                factory, controller = Mock(return_value=context), Mock()
                with patch.object(demo, 'observe') as observe:
                    demo.main(['--side', 'right'] + flags, _client_factory=factory,
                              _controller_factory=controller)
                    observe.assert_called_once()
                factory.assert_called_once_with('right', group='default', read_only=True)
                controller.assert_not_called()
                self.assertFalse(g.commands)

    def test_cli_rejects_missing_target_or_unapproved_acceleration_before_factory(self):
        for flags in (['--confirm-motion'], ['--dry-run', '--confirm-motion'],
                      ['--acceleration-limit-m-s2', '1'], ['--speed-m-s', '-1']):
            with self.subTest(flags=flags), patch('sys.stderr', new_callable=io.StringIO):
                factory = Mock()
                with self.assertRaises(SystemExit):
                    demo.main(['--side', 'right'] + flags, _client_factory=factory)
                factory.assert_not_called()

    def test_cli_preserves_none_zero_and_signed_torque(self):
        for flags, expected in (([], None), (['--torque-cmd-nm', '0'], 0.),
                                (['--torque-cmd-nm', '-.25'], -.25)):
            with self.subTest(flags=flags):
                gc, bc = Mock(), Mock()
                bc.__enter__ = Mock(return_value=SimpleNamespace(transport=SimpleNamespace(node=object())))
                bc.__exit__ = Mock(return_value=False)
                gc.__enter__ = Mock(return_value=FakeGripper())
                gc.__exit__ = Mock(return_value=False)
                with patch.object(demo, 'run') as run:
                    demo.main(['--side', 'right', '--confirm-motion', '--target-m', '.075'] + flags,
                              _client_factory=Mock(return_value=gc), _controller_factory=Mock(return_value=bc))
                self.assertEqual(run.call_args.kwargs['torque_cmd_nm'], expected)
                self.assertEqual(run.call_args.kwargs['dq'], 0.)


if __name__ == '__main__':
    unittest.main(verbosity=2)
