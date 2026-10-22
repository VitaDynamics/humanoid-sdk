"""Review regressions with fake clocks/rings: no Node, ownership or hardware IO."""
from dataclasses import replace
import importlib.util
import os
import signal
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch

from locomotion_aorta import ControllerClosedError, ExternalControlTimeoutError
from examples.policy_support import feedback_age_ns
from locomotion_aorta.transport import CONTROL_STATUS_TOPIC
import test_pull as pull_tests
import test_joy as joy_tests
import test_ee_pull as ee_tests

payload = joy_tests.payload


class PullReviewTest(unittest.TestCase):
    def test_real_unknown_phase_and_truncated_status_are_skipped(self):
        import flatbuffers
        from locomotion_sdk.ControlStatus import ControlStatus, ControlStatusT
        from locomotion_aorta.transport import _decode_control_status
        from locomotion_aorta import ExternalPhase
        t = pull_tests.PullTest().transport()
        t.status_decode_errors = 0
        t._bindings.control_status_decoder = ControlStatus.GetRootAs
        t._subscriber_spec = lambda topic: (None, _decode_control_status, None)
        ring = t._pulls[CONTROL_STATUS_TOPIC]
        def wire(phase):
            obj = ControlStatusT()
            obj.currentAction = 'EXTERNAL'
            obj.actionPhase = phase
            builder = flatbuffers.Builder(64)
            builder.Finish(obj.Pack(builder))
            return bytes(builder.Output())
        ring.samples.append(S(payload=wire(4)))
        good = t.poll_control_status()
        self.assertEqual(good.phase, ExternalPhase.RUNNING)
        for data in (wire(255), b'bad'):
            ring.samples.append(S(payload=data))
            self.assertIs(t.poll_control_status(), good)
        self.assertEqual(t.status_decode_errors, 2)
        ring.samples.append(S(payload=wire(8)))
        self.assertEqual(t.poll_control_status().phase, ExternalPhase.EMERGENCY)

    def test_bad_status_keeps_good_snapshot_counts_and_recovers(self):
        t = pull_tests.PullTest().transport()
        t.status_decode_errors = 0
        good = S(phase='RUNNING')
        emergency = S(phase='EMERGENCY')
        def decode(value):
            if value == b'bad':
                raise ValueError('unknown phase')
            return value
        t._bindings.control_status_decoder = decode
        ring = t._pulls[CONTROL_STATUS_TOPIC]
        callback = Mock()
        t.subscribe(CONTROL_STATUS_TOPIC, callback)
        for wire, expected in ((good, good), (b'bad', good), (emergency, emergency)):
            ring.samples.append(S(payload=wire))
            self.assertIs(t.poll_control_status(), expected)
        self.assertEqual(t.status_decode_errors, 1)
        self.assertEqual([c.args[0] for c in callback.call_args_list], [good, emergency])
        ring.try_recv = Mock(side_effect=RuntimeError('transport failed'))
        with self.assertRaisesRegex(RuntimeError, 'transport failed'):
            t.poll_control_status()

    def test_controller_legacy_timeout_closed_and_status_contract(self):
        fixture = pull_tests.PullTest()
        c, t = fixture.fixture()
        try:
            c.enter_external(.1)
            saved = c.status
            t.wait_state = Mock(side_effect=TimeoutError('no state'))
            with self.assertRaises(ExternalControlTimeoutError):
                c.wait_body_state(.01)
            c.close()
            self.assertIs(c.status, saved)
            for call in (lambda: c.wait_state(.01), lambda: c.publish(None)):
                with self.assertRaises(ControllerClosedError):
                    call()
        finally:
            c.close()
            fixture.doCleanups()

    def test_readonly_transport_preserves_logging_environment(self):
        # Exercise the real constructor through the existing fake Node fixture.
        for value in (None, 'true', 'false'):
            env = {} if value is None else {'AORTA_LOG_TOPIC_ENABLED': value}
            with self.subTest(value=value), patch.dict(os.environ, env, clear=True):
                pull_tests.PullTest().test_constructor_no_publishers_two_native_depth1_rings()
                self.assertEqual(dict(os.environ), env)

    def test_native_abi_failure_happens_during_construction_and_closes_node(self):
        from locomotion_aorta.transport import _AortaTransport
        node = S(group=lambda:'fake', create_publisher_typed=Mock(return_value=object()), close=Mock())
        bindings = S(aorta=S(Node=Mock(return_value=node),
                            QoS=S(realtime_control=lambda: None)),
                     control_request_schema_meta=object(), body_cmd_schema_meta=object())
        with patch('locomotion_aorta.transport._load_aorta_bindings', return_value=bindings), \
             patch('locomotion_aorta._native_body_codec.NativeBodyPublisher',
                   side_effect=RuntimeError('unsupported Aorta publisher header ABI')):
            with self.assertRaisesRegex(RuntimeError, 'header ABI'):
                _AortaTransport(node_name='fake', group='fake')
        node.close.assert_called_once_with()


class JoyReviewTest(unittest.TestCase):
    def setUp(self):
        self.fixture = joy_tests.JoyReaderTest()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def test_restart_rebases_state_without_inventing_edges(self):
        f = self.fixture
        first = f.read(payload(10, buttons=(1,), axes=(.5,)))
        f.time_ns.return_value += 100
        late = f.read(payload(8, buttons=(0,), axes=(-.5,)))
        self.assertEqual((late.axes, late.buttons), ((-.5,), (0,)))
        self.assertGreater(late.received_realtime_ns, first.received_realtime_ns)
        resumed = f.read(payload(9, buttons=(0,)))
        self.assertFalse(resumed.pressed_counts or resumed.released_counts)
        self.assertEqual(resumed.missing_samples_total, 0)

    def test_duplicate_does_not_refresh_axes_or_receipt(self):
        f = self.fixture
        first = f.read(payload(10, axes=(.5,)))
        f.time_ns.return_value += 100
        duplicate = f.read(payload(10, axes=(-.5,)))
        self.assertEqual(duplicate.axes, first.axes)
        self.assertEqual(duplicate.received_realtime_ns, first.received_realtime_ns)

    def test_remote_clock_does_not_determine_local_freshness(self):
        f = self.fixture
        f.time_ns.return_value = 4_000_000_000
        for n, stamp in enumerate((0, 1_000_000_000, 4_000_000_001)):
            with self.subTest(stamp=stamp):
                sample = f.read(payload(n, published_ns=stamp))
                self.assertTrue(sample.is_fresh(4_000_000_000))
        self.assertTrue(f.read(payload(3, published_ns=4_000_000_000)).is_fresh(4_000_000_000))

    def test_truncated_and_invalid_utf8_drop_count_without_refresh_then_recover(self):
        f = self.fixture
        first = f.read(payload(0))
        invalid = payload(1).replace(b'gamepad', b'\xffamepad')
        f.time_ns.return_value += 100
        sample = f.read(b'bad', invalid)
        self.assertEqual(sample.decode_errors_total, 2)
        self.assertFalse(sample.event_history_complete)
        self.assertEqual(sample.received_realtime_ns, first.received_realtime_ns)
        self.assertEqual(f.read(payload(1, buttons=(1,))).pressed_counts, {0: 1})


class FeedbackAgeTest(unittest.TestCase):
    def state(self, published):
        return S(received_monotonic_ns=10_000_000_000,
                 ts_state_pub_us=published, pull_context_receive_stamp_ns=123)

    def test_publication_clock_skew_is_diagnostic_and_cache_age_is_local(self):
        from examples.wbc_tracking_mode.wbc_tracking_mode import wait_fresh
        policy = S(state=Mock())
        for published in (0, 99_900_000, 100_000_001):
            state = self.state(published); state.stamp_ns = 2
            self.assertEqual(feedback_age_ns(state, 10_000_000_000), 0)
            self.assertIs(wait_fresh(S(wait_state=lambda timeout: state), policy,
                previous_ns=1, timeout=.1, max_age=.05, stop=threading.Event(),
                monotonic=lambda: 10.), state)
        state.received_monotonic_ns = 0
        with self.assertRaises(ValueError):
            feedback_age_ns(state, 10_000_000_000)
        state.received_monotonic_ns = 10_000_000_001
        with self.assertRaises(TimeoutError):
            feedback_age_ns(state, 10_000_000_000)
        state.received_monotonic_ns = 9_900_000_000
        with self.assertRaisesRegex(TimeoutError, 'stale'):
            wait_fresh(S(wait_state=lambda timeout: state), policy,
                previous_ns=1, timeout=.1, max_age=.05, stop=threading.Event(),
                monotonic=lambda: 10.)


class EEConcurrencyTest(unittest.TestCase):
    def test_cached_info_publish_does_not_wait_for_other_threads_state_recv(self):
        fixture = ee_tests.EEPullTest()
        client, node = fixture.hand()
        try:
            client.wait_info(.1)
            entered, release, published = (threading.Event() for _ in range(3))
            ring = node.rings[client.topic+'/state']
            def recv(timeout):
                entered.set()
                release.wait(1)
                return None
            ring.recv = recv
            waiter = threading.Thread(target=lambda: client._pulls.poll('state', .5))
            failures = []
            def publish():
                try:
                    client.publish([.01]*6)
                except BaseException as error:
                    failures.append(error)
                finally:
                    published.set()
            sender = threading.Thread(target=publish)
            waiter.start()
            try:
                self.assertTrue(entered.wait(1))
                sender.start()
                self.assertTrue(published.wait(.5), 'publish waited for blocked state recv')
                self.assertFalse(release.is_set())
                self.assertEqual(failures, [])
                self.assertEqual(len(node.commands), 1)
            finally:
                release.set()
                waiter.join(1)
                if sender.ident is not None:
                    sender.join(1)
        finally:
            fixture.doCleanups()


@unittest.skipIf(any(importlib.util.find_spec(name) is None for name in ('numpy', 'yaml')),
                 'test_velocity_mode optional NumPy/PyYAML dependencies unavailable')
class VelocityReviewTest(unittest.TestCase):
    def run_loop(self, *, frozen=False, compute_ns=0, duration=.12,
                 change_status=False, wall_step=False):
        import test_velocity_mode as fixtures
        import numpy as np
        example = fixtures.example
        clock = S(now=10_000_000_000)
        origin = clock.now
        policy = fixtures.policy_fixture()
        status = fixtures.running_status()
        controller = S(status=status)
        self.published, self.records, self.inferred = [], [], []
        def latest():
            state_id = 1 if frozen else (clock.now-origin)//2_500_000+1
            return example.StateFrame(fixtures.frame_state(state_id), (0.,)*31,
                (0.,)*31, (0.,)*3, (1., 0., 0., 0.), 1, 2, 3)
        def infer(_):
            self.inferred.append(clock.now)
            clock.now += compute_ns
            if change_status:
                controller.status = replace(status, timeout_hold=True)
            return np.zeros(2, np.float32)
        policy.infer = infer
        def publish(command):
            self.published.append(clock.now)
            return replace(command, session_id=1, sequence=len(self.published))
        controller.publish = publish
        def realtime():
            # Deliberately jump backwards/forwards independently of policy time.
            return 1_700_000_000_000_000_000 + (clock.now if not wall_step
                else (-1 if len(self.records) % 2 else 1)*10**12)
        with patch.object(example.time, 'monotonic_ns', side_effect=lambda: clock.now), \
             patch.object(example.time, 'time_ns', side_effect=realtime), \
             patch.object(example.time, 'sleep', side_effect=lambda dt: setattr(
                 clock, 'now', clock.now+round(dt*1e9))):
            example.run_cycles(policy, latest,
                S(read=__import__("test_velocity_mode").joy_sample),
                S(cycle=self.records.append), controller=controller,
                duration=duration, transition=([], 0.))

    def test_stopped_state_stops_publish_at_existing_200ms_window(self):
        with self.assertRaisesRegex(TimeoutError, 'BodyState did not advance'):
            self.run_loop(frozen=True, duration=1.)
        self.assertEqual(len(self.published), 10)
        self.assertLess(self.published[-1]-self.published[0], 200_000_000)

    def test_long_inference_does_not_publish_now_stale_state(self):
        with self.assertRaisesRegex(TimeoutError, 'BodyState did not advance'):
            self.run_loop(compute_ns=201_000_000)
        self.assertEqual(self.published, [])

    def test_overrun_reanchors_full_period_no_back_to_back_commands(self):
        self.run_loop(compute_ns=35_000_000, duration=.13)
        self.assertGreaterEqual(len(self.published), 3)
        self.assertTrue(all(b-a == 55_000_000 for a, b in
                            zip(self.published, self.published[1:])))

    def test_wallclock_steps_do_not_change_policy_frequency_or_duration(self):
        self.run_loop(wall_step=True, duration=.05)
        self.assertEqual(len(self.published), 4)
        self.assertEqual([b-a for a, b in zip(self.published, self.published[1:])],
                         [20_000_000]*3)

    def test_timeout_hold_during_inference_does_not_publish(self):
        with self.assertRaisesRegex(RuntimeError, 'RUNNING'):
            self.run_loop(change_status=True)
        self.assertEqual(self.published, [])

    def test_sigterm_sets_stop_then_cancels_closes_and_restores_handlers(self):
        import test_velocity_mode as fixtures
        example = fixtures.example
        args = example.parse_args(['--live', '--locomotion-dir', '/unused',
                                   '--log', '/unused.jsonl'])
        controller, transport, directions, log = (Mock() for _ in range(4))
        transport.observed_status = S(current_action='PASSIVE',
            phase=S(value='IDLE'), generation=0)
        transport.publishers = {}
        controller.get_robot_info.return_value = S(info=S(body=S(roster_version=1, joints=[])))
        controller.exit_external.return_value = S(current_action='PASSIVE', phase=S(value='IDLE'))
        directions.read.return_value = __import__("test_velocity_mode").joy_sample()
        before = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
        def run(*args, stop, **kwargs):
            self.assertFalse(stop.is_set())
            signal.raise_signal(signal.SIGTERM)
            self.assertTrue(stop.is_set())
        with patch.object(example, 'version', return_value='0.2.1'), \
             patch.object(example, 'parse_args', return_value=args), \
             patch.object(example, 'VelocityModePolicy', return_value=Mock(inventory={}, order=[])), \
             patch.object(example, 'PullTransport', return_value=transport), \
                 patch.object(example, 'wait_ready', return_value=transport.observed_status), \
                 patch.object(example, 'read_frame', return_value=Mock()), \
             patch.object(example, 'PullController', return_value=controller), \
             patch.object(example, 'JoyReader', return_value=directions), \
             patch.object(example, 'CycleLog', return_value=log), \
             patch.object(example, 'transition_duration', return_value=([], 0.)), \
             patch.object(example, 'run_cycles', side_effect=run):
            example.main()
        controller.exit_external.assert_called_once_with(args.timeout)
        controller.close.assert_called_once_with()
        controller.publish.assert_not_called()
        for sig, handler in before.items():
            self.assertIs(signal.getsignal(sig), handler)


class TeleopReviewTest(unittest.TestCase):
    def run_loop(self, *, hold_after_wait=False):
        import test_teleop_policy as fixtures
        from examples.teleop_policy.teleop_policy import live
        clock, stop = S(now=10.), threading.Event()
        body, state = fixtures.fixture()
        for joint in state.joints:
            joint.q = joint.dq = 0.
        status = S(phase='RUNNING', current_action='EXTERNAL', generation=1,
                   client_id='test', lease_id='lease', reject_count=0, timeout_hold=False)
        self.published, self.events = [], []
        controller = S(status=status)
        reads = 0
        def wait(timeout):
            nonlocal reads
            reads += 1
            state.state_id += 1
            if hold_after_wait and reads == 2:
                clock.now += .07
                controller.status = S(**dict(vars(status), timeout_hold=True))
            return state
        def publish(command):
            self.published.append(clock.now)
            if len(self.published) == 3:
                stop.set()
            return command
        def infer(obs):
            clock.now += .035
            return [0.]*12
        controller.get_robot_info = lambda timeout: S(info=S(body=body),
            status=fixtures.API.ServiceStatus.SUCCESS, error_message='')
        controller.wait_state = wait
        controller.enter_external = lambda timeout: status
        controller.publish = publish
        controller.exit_external = lambda timeout: self.events.append('cancel') or S(current_action='PASSIVE')
        live(controller, fixtures.API, fixtures.Policy(fixtures.METADATA, fixtures.CONTRACT),
             S(infer=infer), lambda: (), stop, 1., monotonic=lambda: clock.now,
             sleep=lambda dt: setattr(clock, 'now', clock.now+dt))

    def test_overrun_waits_full_period_instead_of_bursting(self):
        self.run_loop()
        for a, b in zip(self.published, self.published[1:]):
            self.assertAlmostEqual(b-a, .055)
        self.assertEqual(self.events, ['cancel'])

    def test_timeout_hold_after_state_wait_refuses_publish_and_cancels(self):
        with self.assertRaisesRegex(RuntimeError, 'state changed'):
            self.run_loop(hold_after_wait=True)
        self.assertEqual(self.published, [])
        self.assertEqual(self.events, ['cancel'])
