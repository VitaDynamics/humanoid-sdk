"""T054 synthetic dual-gripper examples; no robot connection or motion."""
from dataclasses import replace
import io
import math
import signal
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from examples import gripper_external as demo
from locomotion_aorta.gripper_types import GripperHostTarget
from test_examples import FakeClock, RecordingController
from test_fixed_stand_pose import body_info
from body_fixtures import robot_info_reply
from test_gripper_external import FakeGripper


class SineTest(unittest.TestCase):
    def setup_case(self, *, configs=None, clock=None, controller_options=None):
        clock = clock or FakeClock()
        controller = RecordingController(clock, **(controller_options or {}))
        self.addCleanup(controller.close)
        clients, writes, events = [], [], []
        for side, wire, q in [('left', 1, .02), ('right', 2, .07)]:
            g = FakeGripper(**((configs or {}).get(side, {})))
            g.side = side
            g.session_id += wire
            g.info = replace(g.info, side=wire)
            g._state = replace(g._state, side=wire, sample=replace(g._state.sample, q=q))
            publish = g.publish_target
            def record(*args, g=g, publish=publish, **kwargs):
                cmd = publish(*args, **kwargs)
                # FakeGripper uses a right-only envelope; change only the fake result.
                cmd = replace(cmd, side=g.info.side)
                g.commands[-1] = cmd
                writes.append((g.side, clock.monotonic(), cmd))
                return cmd
            g.publish_target = record
            clients.append(g)
        reply = robot_info_reply()
        controller.get_robot_info = lambda timeout: replace(reply, info=replace(
            reply.info, body=body_info(), end_effectors=tuple(SimpleNamespace(
                configured=True, side=g.info.side, gripper=g.info) for g in clients)))
        return controller, clients, clock, writes, events

    def run_case(self, case, **kwargs):
        c, clients, clock, writes, events = case
        return demo.run_sine(c, clients, timeout=1, duration=kwargs.pop('duration', .4),
                             period=kwargs.pop('period', .2), sleep=clock.sleep,
                             monotonic=clock.monotonic, emit=events.append, **kwargs)

    def test_sine_quarters_full_period_and_no_range_clip(self):
        for elapsed, expected in [(0, .045), (2.5, .08), (5, .045), (7.5, .01), (10, .045)]:
            self.assertAlmostEqual(demo.sine_opening(elapsed), expected)
        for elapsed in [i / 100 for i in range(3001)]:
            q = demo.sine_opening(elapsed)
            self.assertGreaterEqual(q, .01)
            self.assertLessEqual(q, .08)

    def test_both_same_phase_single_session_real_time_and_return_saved_openings(self):
        case = self.setup_case()
        self.run_case(case)
        c, clients, clock, writes, events = case
        self.assertTrue(writes)
        for left, right in zip(writes[::2], writes[1::2]):
            self.assertEqual((left[0], right[0]), ('left', 'right'))
            self.assertEqual(left[1], right[1])
            self.assertEqual((left[2].dq, right[2].dq), (0, 0))
            self.assertIsNone(left[2].torque_cmd_nm)
        self.assertEqual([g.commands[-1].q for g in clients], [.02, .07])
        sine_logs = [e for e in events if 'stage=sine side=' in e]
        self.assertTrue(sine_logs)
        for line in sine_logs:
            fields = dict(kv.split('=', 1) for kv in line.split() if '=' in kv)
            self.assertAlmostEqual(float(fields['target_m']), demo.sine_opening(
                float(fields['elapsed_s']), period=.2), places=5)
        for a, b in zip(sine_logs[::2], sine_logs[1::2]):
            self.assertEqual(a.split('target_m=')[1].split()[0], b.split('target_m=')[1].split()[0])
        self.assertEqual(c.calls.count('enter_external'), 1)
        self.assertEqual(c.calls.count('exit_external'), 1)
        self.assertTrue(any('confirmed PASSIVE' in e for e in events))
        self.assertEqual(len([e for e in events if 'GRIPPER_SINE summary' in e]), 2)
        self.assertTrue(any('error_m=' in e and 'reported_torque_nm=' in e and 'status_raw=' in e
                            and 'inhibit=' in e for e in events))
        self.assertTrue(any('BODY_HOLD summary observation_only=true' in e for e in events))

    def test_entry_and_return_duration_formula_uses_latest_robot_host_target(self):
        case = self.setup_case()
        for client, q in zip(case[1], (.08, .01)):
            client._state = replace(client._state, host_target_valid=True,
                                    host_target=GripperHostTarget(q, .5))
        self.run_case(case)
        transitions = [line for line in case[-1] if 'linear_duration_s=' in line]
        self.assertEqual(len(transitions), 2)
        self.assertIn('stage=entry', transitions[0])
        self.assertIn('stage=return', transitions[1])
        self.assertAlmostEqual(float(transitions[0].split('linear_duration_s=')[1].split()[0]), 1.25)
        self.assertAlmostEqual(float(transitions[1].split('linear_duration_s=')[1].split()[0]), 3.)
        for side, start, end in [('left', .08, .02), ('right', .01, .07)]:
            line = next(e for e in case[-1] if f'stage=return side={side}' in e)
            values = dict(word.split('=', 1) for word in line.split() if '=' in word)
            self.assertAlmostEqual(float(values['target_m']),
                                   start+(end-start)*float(values['elapsed_s'])/3., places=7)
        self.assertEqual([g.commands[-1].q for g in case[1]], [.02, .07])

    def test_default_30_seconds_covers_three_sine_cycles(self):
        case = self.setup_case()
        self.run_case(case, duration=30, period=10)
        lines = [e for e in case[-1] if 'stage=sine side=left' in e]
        values = [dict(kv.split('=', 1) for kv in e.split() if '=' in kv) for e in lines]
        self.assertAlmostEqual(float(values[-1]['elapsed_s']), 30)
        self.assertGreaterEqual(len(values), 1499)
        q = [float(v['target_m']) for v in values]
        self.assertAlmostEqual(min(q), .01, places=5)
        self.assertAlmostEqual(max(q), .08, places=5)
        self.assertAlmostEqual(q[-1], .045)
        self.assertEqual([g.commands[-1].q for g in case[1]], [.02, .07])

    def test_reject_all_bad_or_unknown_ranges_before_prepare_without_clipping(self):
        for source in ['topic', 'aggregate']:
            case = self.setup_case()
            c, clients, _, writes, _ = case
            if source == 'topic':
                clients[0].info = replace(clients[0].info, opening_max_m=.06)
                clients[1].info = replace(clients[1].info, opening_min_source=0)
            else:
                original = c.get_robot_info
                def bad_info(timeout):
                    response = original(timeout)
                    caps = list(response.info.end_effectors)
                    caps[0].gripper = replace(caps[0].gripper, opening_max_m=.06)
                    caps[1].gripper = replace(caps[1].gripper, opening_min_source=0)
                    return replace(response, info=replace(response.info, end_effectors=tuple(caps)))
                c.get_robot_info = bad_info
            with self.assertRaises(ValueError) as caught:
                self.run_case(case)
            self.assertIn('left', str(caught.exception))
            self.assertIn('right', str(caught.exception))
            self.assertNotIn('enter_external', c.calls)
            self.assertFalse(writes)

    def test_sigint_during_sine_cancels_once_no_return_motion_after_interrupt(self):
        case = self.setup_case()
        c, clients, clock, writes, events = case
        publish = clients[1].publish_target
        def interrupt(*a, **kw):
            result = publish(*a, **kw)
            if len(clients[1].commands) == 100:
                signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
            return result
        clients[1].publish_target = interrupt
        with self.assertRaises(KeyboardInterrupt):
            self.run_case(case, duration=3, period=1)
        self.assertEqual(c.calls.count('exit_external'), 1)
        self.assertFalse(any('stage=return' in e for e in events))
        self.assertTrue(any('confirmed PASSIVE' in e for e in events))

    def test_each_side_reject_stale_inhibit_and_trace_timeout_still_cancel(self):
        for side in ['left', 'right']:
            for config in [{'reject': True}, {'inhibited': True}, {'echo': False}, {'stale': True}]:
                with self.subTest(side=side, config=config):
                    case = self.setup_case(configs={side: config})
                    with self.assertRaises((RuntimeError, TimeoutError)):
                        self.run_case(case)
                    self.assertEqual(case[0].calls.count('enter_external'), 1)
                    self.assertEqual(case[0].calls.count('exit_external'), 1)

    def test_body_ownership_reject_and_cancel_failure_guards_still_apply(self):
        for options, message in [({'fail_phase_after_commands': 70}, 'ownership/phase'),
                                 ({'reject_after_commands': 70}, 'reject_count'),
                                 ({'cancel_action': 'FIXED_STAND'}, 'expected PASSIVE')]:
            with self.subTest(options=options):
                case = self.setup_case(controller_options=options)
                with self.assertRaisesRegex(RuntimeError, message):
                    self.run_case(case)
                self.assertEqual(case[0].calls.count('exit_external'), 1)

    def test_slow_pc_samples_elapsed_sine_time_not_frame_count(self):
        class SlowClock(FakeClock):
            def sleep(self, duration):
                super().sleep(duration + .03)
        case = self.setup_case(clock=SlowClock())
        self.run_case(case, period=.5, duration=1)
        lines = [e for e in case[-1] if 'stage=sine side=left' in e]
        self.assertLess(len(lines), 50)
        for line in lines:
            fields = dict(kv.split('=', 1) for kv in line.split() if '=' in kv)
            self.assertAlmostEqual(float(fields['target_m']), demo.sine_opening(
                float(fields['elapsed_s']), period=.5), places=5)

    def test_cli_both_share_one_node_and_default_sine_values(self):
        node = object()
        controller = Mock()
        controller.__enter__ = Mock(return_value=SimpleNamespace(transport=SimpleNamespace(node=node)))
        controller.__exit__ = Mock(return_value=False)
        contexts = []
        def factory(side, **kw):
            self.assertIs(kw['node'], node)
            context = Mock()
            context.__enter__ = Mock(return_value=side)
            context.__exit__ = Mock(return_value=False)
            contexts.append(context)
            return context
        with patch.object(demo, 'run_sine') as run:
            demo.main(['--side', 'both', '--pattern', 'sine', '--body-pose', 'zero', '--confirm-motion'],
                      _client_factory=factory, _controller_factory=lambda **kw: controller)
        self.assertEqual(run.call_args.args[1], ['left', 'right'])
        for key, value in [('low', .01), ('high', .08), ('period', 10), ('duration', 30), ('body_pose', 'zero')]:
            self.assertEqual(run.call_args.kwargs[key], value)

    def test_cli_bad_sine_arguments_fail_before_factory_and_dry_run_no_controller(self):
        for flags in [['--period-s', '0'], ['--sine-min-m', '.08'], ['--speed-m-s', '.1'],
                      ['--target-m', '.02']]:
            with self.subTest(flags=flags), patch('sys.stderr', new_callable=io.StringIO):
                factory = Mock()
                with self.assertRaises(SystemExit):
                    demo.main(['--side', 'both', '--pattern', 'sine', '--confirm-motion'] + flags,
                              _client_factory=factory)
                factory.assert_not_called()
        context = Mock()
        context.__enter__ = Mock(return_value=FakeGripper())
        context.__exit__ = Mock(return_value=False)
        factory, controller = Mock(return_value=context), Mock()
        with patch.object(demo, 'observe') as observe:
            demo.main(['--side', 'both', '--pattern', 'sine', '--dry-run'],
                      _client_factory=factory, _controller_factory=controller)
        self.assertEqual(observe.call_count, 2)
        controller.assert_not_called()


if __name__ == '__main__':
    unittest.main()
