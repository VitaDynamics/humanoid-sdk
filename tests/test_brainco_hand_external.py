from dataclasses import replace
import io
import math
import unittest
from unittest.mock import patch

from examples import brainco_hand_external as demo
from locomotion_aorta import EXTERNAL_COMMAND_TOPIC, HandState, HandJointState, HandCmd, HandJointCmd
from test_examples import FakeClock, RecordingController
from test_hand import codec, info_obj
from lowlevel.HandInfo import HandInfo
from test_hand import pack


class FakeHand:
    def __init__(self, side='left', *, stale=False, reject=False, inhibited=False, echo=True):
        self.side = side
        self.session_id = 101 if side == 'left' else 202
        self.echo = echo
        self.info = codec.decode_info(HandInfo.GetRootAs(pack(info_obj(1 if side == 'left' else 2))))
        self.commands = []
        self.stale, self.reject, self.inhibited = stale, reject, inhibited
        self._state = HandState(state_id=1, hand_type=1, side=self.info.side, dof=6,
                                joints=(HandJointState(q=.01),) * 6, output_allowed=True, inhibit_reason=0)
    @property
    def state(self):
        if not self.stale:
            self._state = replace(self._state, state_id=self._state.state_id + 1)
        if self.commands and self.reject:
            self._state = replace(self._state, reject_count=1)
        if self.commands and self.inhibited:
            self._state = replace(self._state, output_allowed=False, inhibit_reason=3)
        return self._state
    def get_info(self, timeout): return self.info
    def wait_state(self, timeout): return self.state
    def publish(self, q, dq):
        self.commands.append((tuple(q), tuple(dq)))
        self._state = replace(self._state, joints=tuple(HandJointState(q=v, q_cmd=v) for v in q),
                              last_cmd_session_id=self.session_id if self.echo else 0,
                              last_cmd_sequence=len(self.commands) if self.echo else 0)
        return HandCmd(self.session_id, len(self.commands), 1, self.info.side,
                       tuple(HandJointCmd(a,b) for a,b in zip(q,dq)))


class ExternalHandTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.events = []
    def controller(self, **kw):
        c = RecordingController(self.clock, **kw); self.addCleanup(c.close)
        return c
    def run_demo(self, c, hands, **kw):
        demo.run(c, hands, timeout=1, sleep=kw.pop('sleep', self.clock.sleep),
                 monotonic=self.clock.monotonic, emit=self.events.append, **kw)

    def test_body_holds_original_pose_and_hands_use_only_independent_publish(self):
        c = self.controller(); hands = [FakeHand('left'), FakeHand('right')]
        self.run_demo(c, hands)
        body_commands = [command for topic, command in c.transport.published if topic == EXTERNAL_COMMAND_TOPIC]
        self.assertGreater(len(body_commands), 100)
        baseline = demo.body._position_command((.1,)*42).motor_cmds
        self.assertTrue(all(cmd.motor_cmds == baseline for cmd in body_commands))
        for hand in hands:
            self.assertGreater(len(hand.commands), 50)
            self.assertAlmostEqual(hand.commands[-1][0][0], .005, places=6)
            previous = (.01,)*6
            for q, dq in hand.commands:
                self.assertTrue(all(0 <= v <= .02 for v in q))
                self.assertTrue(all(v == demo.HAND_SPEED_RAD_S for v in dq))
                self.assertLessEqual(max(abs(a-b) for a,b in zip(q,previous)),
                                     demo.HAND_SPEED_RAD_S * demo.body.PERIOD_S + 1e-9)
                previous = q
        self.assertIn('exit_external', c.calls)
        self.assertTrue(any('CANCEL confirmed PASSIVE' in s for s in self.events))
        self.assertTrue(any('NOT_MEASURED' in s and 'robot_chain_us' in s for s in self.events))

    def test_dryrun_and_default_use_readonly_clients_without_body_controller(self):
        for args in (['--dry-run', '--side', 'left'], ['--side', 'right']):
            with patch.object(demo, 'HandClient') as hand, patch.object(demo, 'ExternalController') as body, \
                 patch.object(demo, 'observe') as observe:
                demo.main(args)
                body.assert_not_called()
                self.assertTrue(hand.call_args.kwargs['read_only'])
                observe.assert_called_once()
        with patch.object(demo, 'HandClient') as hand, patch('sys.stderr', new_callable=io.StringIO):
            with self.assertRaises(SystemExit): demo.main(['--dry-run', '--touch', 'on'])
            hand.assert_not_called()

    def test_keyboardinterrupt_after_entry_cancels_and_sends_no_exit_hand_target(self):
        c = self.controller(); hand = FakeHand()
        def interrupt(t):
            if hand.commands:
                raise KeyboardInterrupt()
            self.clock.sleep(t)
        with self.assertRaises(KeyboardInterrupt): self.run_demo(c, [hand], sleep=interrupt)
        self.assertIn('exit_external', c.calls)
        self.assertEqual(len(hand.commands), 1)

    def test_entry_failure_still_attempts_cancel(self):
        c = self.controller(); hand = FakeHand()
        with patch.object(c, 'enter_external', side_effect=RuntimeError('mock entry')), \
             patch.object(c, 'exit_external', return_value=type('S', (), {'current_action':'PASSIVE'})()) as cancel:
            with self.assertRaisesRegex(RuntimeError, 'mock entry'): self.run_demo(c, [hand])
            cancel.assert_called_once()
        self.assertFalse(hand.commands)

    def test_hand_fault_reject_stale_and_body_fault_cancel(self):
        for hand, kwargs in ((FakeHand(stale=True), {}), (FakeHand(reject=True), {}),
                              (FakeHand(inhibited=True), {}), (FakeHand(), {'fail_phase_after_commands':3}),
                              (FakeHand(), {'reject_after_commands':3}),
                              (FakeHand(), {'stale_after_commands':3}),
                              (FakeHand(), {'drift_command_range':(3,20)})):
            with self.subTest(hand=hand, kwargs=kwargs):
                c = self.controller(**kwargs)
                with self.assertRaises(RuntimeError): self.run_demo(c, [hand])
                self.assertIn('exit_external', c.calls)

    def test_wrong_model_nonfinite_and_limits_fail_before_enter(self):
        for change in ('model','nan','limits'):
            c = self.controller(); hand = FakeHand()
            if change == 'model': hand.info = replace(hand.info, model='LINKER_L6')
            elif change == 'limits': hand.info = replace(hand.info, dq_max=(0.,)*6)
            else: hand._state = replace(hand._state, joints=(HandJointState(q=float('nan')),)*6)
            with self.assertRaises(ValueError): self.run_demo(c, [hand])
            self.assertNotIn('enter_external', c.calls)
            self.assertFalse(hand.commands)

    def test_unconfirmed_cancel_is_reported_and_not_retried(self):
        c = self.controller(cancel_action='EMERGENCY'); hand = FakeHand()
        with self.assertRaisesRegex(RuntimeError,'expected PASSIVE'): self.run_demo(c,[hand])
        self.assertEqual(c.calls.count('exit_external'),1)
        self.assertTrue(any('CANCEL UNCONFIRMED' in s for s in self.events))

    def test_missing_command_echo_times_out_and_releases_lease(self):
        c = self.controller()
        with self.assertRaisesRegex(TimeoutError, 'echo did not advance'):
            self.run_demo(c, [FakeHand(echo=False)])
        self.assertIn('exit_external', c.calls)
