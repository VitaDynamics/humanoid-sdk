from body_fixtures import sample_motor, robot_info_reply
from dataclasses import replace
import io
import math
import signal
import unittest
from unittest.mock import patch

from examples import brainco_hand_external as demo
from locomotion_aorta import HUM_CMD_TOPIC, HandState, HandJointState, HandCmd, HandJointCmd
from test_examples import FakeClock, RecordingController
from test_hand import codec, info_obj
from lowlevel.HandInfo import HandInfo
from test_hand import pack


class FakeHand:
    def __init__(self, side='left', *, stale=False, reject=False, inhibited=False, echo=True,
                 speed_limit=None):
        self.side = side
        self.session_id = 101 if side == 'left' else 202
        self.echo = echo
        self.info = replace(codec.decode_info(HandInfo.GetRootAs(pack(info_obj(1 if side == 'left' else 2)))),
                            dq_max=(demo.HAND_SPEED_RAD_S,) * 6)
        if speed_limit is not None:
            self.info = replace(self.info, dq_max=(speed_limit,) * 6)
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
        self.run_demo(c, hands, body_gains='hold')
        body_commands = [command for topic, command in c.transport.published if topic == HUM_CMD_TOPIC]
        self.assertGreater(len(body_commands), 100)
        baseline = demo.body._position_command(tuple(0.0 if i == 14 else .1 for i in range(31))).motor_cmds
        self.assertTrue(all(cmd.motor_cmds == baseline for cmd in body_commands))
        for hand in hands:
            self.assertGreater(len(hand.commands), 0)
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

    def test_zero_gains_preserve_measured_q_and_independent_hand_motion(self):
        c = self.controller(); hands = [FakeHand('left'), FakeHand('right')]
        self.run_demo(c, hands, body_gains='zero')
        commands = [command for topic, command in c.transport.published if topic == HUM_CMD_TOPIC]
        self.assertGreater(len(commands), 100)
        for command in commands:
            self.assertEqual(len(command.motor_cmds), 31)
            for i, motor in enumerate(command.motor_cmds):
                self.assertEqual(motor.q, 0.0 if i == 14 else .1)
                self.assertEqual((motor.kp, motor.kd, motor.dq, motor.tau), (0., 0., 0., 0.))
        self.assertTrue(all(len(hand.commands) > 0 for hand in hands))
        self.assertIn('exit_external', c.calls)
        self.assertTrue(any('--body-gains zero' in s for s in self.events))

    def test_body_gains_cli_defaults_to_hold_and_forwards_zero(self):
        for args, expected in (([], 'hold'), (['--body-gains', 'hold'], 'hold'),
                               (['--body-gains', 'zero'], 'zero')):
            with self.subTest(expected=expected), patch.object(demo, 'HandClient'), \
                 patch.object(demo, 'ExternalController'), patch.object(demo, 'run') as run:
                demo.main(['--confirm-motion', '--side', 'left'] + args)
                self.assertEqual(run.call_args.kwargs['body_gains'], expected)
        with patch.object(demo, 'HandClient') as hand, patch('sys.stderr', new_callable=io.StringIO):
            with self.assertRaises(SystemExit): demo.main(['--body-gains', 'invalid'])
            hand.assert_not_called()

    def test_invalid_body_gains_fail_before_entry(self):
        c = self.controller(); hand = FakeHand()
        with self.assertRaisesRegex(ValueError, 'body_gains'):
            self.run_demo(c, [hand], body_gains='invalid')
        self.assertNotIn('enter_external', c.calls)
        self.assertFalse(hand.commands)

    def test_dryrun_and_default_use_readonly_clients_without_body_controller(self):
        for args in (['--dry-run', '--side', 'left'], ['--side', 'right'],
                     ['--dry-run', '--body-gains', 'zero'], ['--body-gains', 'zero']):
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

    def test_default_trajectory_matches_original_three_fraction_commands(self):
        c = self.controller(); hand = FakeHand()
        self.run_demo(c, [hand])
        expected = []
        previous = (.01,) * 6
        for fraction in (.25, .75, .25):
            target = tuple(a + fraction * (b - a) for a, b in zip(hand.info.q_min, hand.info.q_max))
            steps = max(1, math.ceil(max(abs(a - b) for a, b in zip(target, previous))
                                    / demo.HAND_SPEED_RAD_S / demo.body.PERIOD_S))
            expected.extend((tuple(a + (b - a) * step / steps for a, b in zip(previous, target)),
                             (demo.HAND_SPEED_RAD_S,) * 6) for step in range(1, steps + 1))
            previous = target
        self.assertEqual(hand.commands, expected)
        self.assertEqual(sum(' joint=' in s for s in self.events), 6)
        self.assertEqual(sum(' complete pattern=' in s for s in self.events), 1)

    def test_sequential_order_and_all_other_joints_remain_low_across_hands(self):
        c = self.controller(); hands = [FakeHand('left', speed_limit=math.radians(160)),
                                      FakeHand('right', speed_limit=math.radians(160))]
        self.run_demo(c, hands, pattern='sequential', speed_deg_s=20, low_frac=.1, high_frac=.9)
        low = tuple(a + .1 * (b - a) for a, b in zip(hands[0].info.q_min, hands[0].info.q_max))
        high = tuple(a + .9 * (b - a) for a, b in zip(hands[0].info.q_min, hands[0].info.q_max))
        initialized = False
        active_order = []
        previous = [(.01,) * 6, (.01,) * 6]
        for frame in zip(*(h.commands for h in hands)):
            qs = [cmd[0] for cmd in frame]
            if initialized:
                active = [(side, j) for side, q in enumerate(qs) for j, v in enumerate(q)
                          if abs(v - low[j]) > 1e-9]
                self.assertLessEqual(len(active), 1)
                if active and (not active_order or active_order[-1] != active[0]):
                    active_order.append(active[0])
            elif all(all(abs(a - b) < 1e-9 for a, b in zip(q, low)) for q in qs):
                initialized = True
            for side, (q, dq) in enumerate(frame):
                self.assertTrue(all(abs(a - b) <= dq[j] * demo.body.PERIOD_S + 1e-9
                                    for j, (a, b) in enumerate(zip(q, previous[side]))))
            previous = qs
        self.assertTrue(initialized)
        self.assertEqual(active_order, [(side, j) for side in range(2) for j in range(6)])
        for hand in hands:
            for actual, target in zip(hand.commands[-1][0], low):
                self.assertAlmostEqual(actual, target)
            for j in range(6):
                self.assertAlmostEqual(max(q[j] for q, _ in hand.commands), high[j])
        self.assertEqual(sum(' joint=' in s for s in self.events), 12)
        self.assertEqual(sum(' complete pattern=' in s for s in self.events), 1)

    def test_loop_signals_cancel_after_multiple_rounds_and_restore_handlers(self):
        for pattern in ('simultaneous', 'sequential'):
            for signum in (signal.SIGINT, signal.SIGTERM):
                with self.subTest(pattern=pattern, signal=signum):
                    self.events.clear()
                    c = self.controller(); hand = FakeHand(speed_limit=math.radians(160))
                    handlers = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
                    at_stop = []
                    def stop_after_rounds(t):
                        if sum(' complete pattern=' in s for s in self.events) >= 2:
                            at_stop.append(len(hand.commands))
                            signal.raise_signal(signum)
                        self.clock.sleep(t)
                    original_exit = c.exit_external
                    def cancel(timeout):
                        # Repeated signals must not prevent PASSIVE confirmation.
                        signal.raise_signal(signal.SIGINT)
                        signal.raise_signal(signal.SIGTERM)
                        return original_exit(timeout)
                    with patch.object(c, 'exit_external', side_effect=cancel):
                        with self.assertRaises(KeyboardInterrupt):
                            self.run_demo(c, [hand], pattern=pattern, loop=True, speed_deg_s=20,
                                          sleep=stop_after_rounds)
                    self.assertEqual(c.calls.count('exit_external'), 1)
                    self.assertEqual(c.status.current_action, 'PASSIVE')
                    self.assertEqual(len(hand.commands), at_stop[0])
                    self.assertEqual(sum(' complete pattern=' in s for s in self.events), 2)
                    self.assertEqual({s: signal.getsignal(s) for s in handlers}, handlers)

    def test_cli_motion_options_defaults_and_forwarding(self):
        for args, expected in (([], dict(speed_deg_s=20., pattern='simultaneous', loop=False,
                                        low_frac=.25, high_frac=.75, feedback_interval=1.)),
                               (['--speed-deg-s', '20', '--pattern', 'sequential', '--loop',
                                 '--low-frac', '.1', '--high-frac', '.9', '--feedback-interval', '3'],
                                dict(speed_deg_s=20., pattern='sequential', loop=True,
                                     low_frac=.1, high_frac=.9, feedback_interval=3.))):
            with patch.object(demo, 'HandClient'), patch.object(demo, 'ExternalController'), \
                 patch.object(demo, 'run') as run:
                demo.main(['--confirm-motion'] + args)
                for key, value in expected.items():
                    self.assertEqual(run.call_args.kwargs[key], value)

    def test_invalid_motion_options_before_connect_or_enter(self):
        cases = [dict(speed_deg_s=v) for v in (0, -1, float('nan'), float('inf'), 1e300, 1e-300)]
        cases += [dict(low_frac=a, high_frac=b) for a, b in
                  ((-.1, .75), (.25, 1.1), (.5, .5), (.8, .2), (float('nan'), .75), (.25, float('inf')))]
        cases += [dict(pattern='invalid'), dict(feedback_interval=.5), dict(feedback_interval=float('nan'))]
        for options in cases:
            with self.subTest(options=options):
                c = self.controller(); hand = FakeHand()
                with self.assertRaises(ValueError): self.run_demo(c, [hand], **options)
                self.assertNotIn('enter_external', c.calls)
                argv = [part for key, value in options.items()
                        for part in ('--' + key.replace('_', '-'), str(value))]
                with patch.object(demo, 'HandClient') as client, \
                     patch.object(demo, 'ExternalController') as controller, \
                     patch('sys.stderr', new_callable=io.StringIO):
                    with self.assertRaises(SystemExit): demo.main(argv)
                    client.assert_not_called(); controller.assert_not_called()

    def test_default_twenty_degrees_rejects_insufficient_advertised_limit(self):
        c = self.controller(); hand = FakeHand(speed_limit=math.radians(19))
        with self.assertRaisesRegex(ValueError, 'speed limit'):
            self.run_demo(c, [hand])
        self.assertNotIn('enter_external', c.calls)
        self.assertFalse(hand.commands)

    def test_requested_speed_limit_is_checked_before_entry_and_each_frame(self):
        c = self.controller(); hand = FakeHand()
        with self.assertRaisesRegex(ValueError, 'speed limit'):
            self.run_demo(c, [hand], speed_deg_s=math.degrees(min(hand.info.dq_max)) + 1)
        self.assertNotIn('enter_external', c.calls)
        hand.info = replace(hand.info, dq_max=(demo.HAND_SPEED_RAD_S,) * 6)
        self.run_demo(c, [hand])
        with self.assertRaisesRegex(ValueError, 'speed limit'):
            demo.validate_hand(hand.info, hand.state, require_output=True, speed=math.radians(21))

    def test_loop_keeps_all_existing_safety_failures_and_cancel(self):
        for hand, kwargs in ((FakeHand(stale=True), {}), (FakeHand(reject=True), {}),
                              (FakeHand(inhibited=True), {}), (FakeHand(echo=False), {}),
                              (FakeHand(), {'fail_phase_after_commands':3}),
                              (FakeHand(), {'reject_after_commands':3}),
                              (FakeHand(), {'stale_after_commands':3}),
                              (FakeHand(), {'drift_command_range':(3,20)})):
            with self.subTest(kwargs=kwargs):
                c = self.controller(**kwargs)
                with self.assertRaises((RuntimeError, TimeoutError)):
                    self.run_demo(c, [hand], pattern='sequential', loop=True)
                self.assertEqual(c.calls.count('exit_external'), 1)

    def test_periodic_feedback_interval_is_respected(self):
        c = self.controller(); hand = FakeHand(); times = []
        original = demo.print_feedback
        def record(*args, **kwargs):
            times.append(self.clock.monotonic())
            return original(*args, **kwargs)
        with patch.object(demo, 'print_feedback', side_effect=record):
            self.run_demo(c, [hand], pattern='sequential', feedback_interval=2.)
        self.assertGreater(len(times), 2)
        self.assertTrue(all(b - a >= 2. for a, b in zip(times, times[1:])))

    def test_fraction_endpoints_and_different_joint_limits_remain_in_range(self):
        c = self.controller(); hand = FakeHand(speed_limit=math.radians(160))
        hand.info = replace(hand.info, q_min=(-.1, -.05, 0., .005, -.2, -.01),
                            q_max=(.2, .1, .02, .04, .2, .03))
        self.run_demo(c, [hand], pattern='sequential', speed_deg_s=20, low_frac=0., high_frac=1.)
        for q, _ in hand.commands:
            self.assertTrue(all(a <= v <= b for a, v, b in zip(hand.info.q_min, q, hand.info.q_max)))
        for j, (low, high) in enumerate(zip(hand.info.q_min, hand.info.q_max)):
            self.assertAlmostEqual(hand.commands[-1][0][j], low)
            self.assertAlmostEqual(max(q[j] for q, _ in hand.commands), high)

    def test_stop_during_normal_cancel_does_not_interrupt_passive_confirmation(self):
        c = self.controller(); hand = FakeHand()
        original_exit = c.exit_external
        handlers = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
        def cancel(timeout):
            signal.raise_signal(signal.SIGINT)
            signal.raise_signal(signal.SIGTERM)
            return original_exit(timeout)
        with patch.object(c, 'exit_external', side_effect=cancel):
            self.run_demo(c, [hand])
        self.assertEqual(c.status.current_action, 'PASSIVE')
        self.assertEqual({s: signal.getsignal(s) for s in handlers}, handlers)
