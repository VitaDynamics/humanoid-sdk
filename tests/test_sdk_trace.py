"""Direct public SDK calls with real native trace files and synthetic transport."""
from contextlib import contextmanager, redirect_stderr
from dataclasses import replace
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from locomotion_aorta import (BodyCmd, BodyJointCmd, BodyState, ExternalController,
    PullController, PullStateSource, InvalidCommandError, ControlOperation)
from locomotion_aorta.end_effector import EndEffectorClient
from locomotion_aorta.end_effector_types import SampleStream
from locomotion_aorta.trace import Kind, Outcome, Trace
from locomotion_aorta.trace.reader import Reader
from test_controller import ControllerHarness, make_command, _BODY_TEST_TARGETS
from test_end_effector_client import Node, confirm_info
from test_end_effector_codec import info_object, state_object, touch_object, target
from test_examples import RecordingController, FakeClock, external_control
from fake_transport import FakeTransport
from body_fixtures import robot_info_reply, sample_motor, body_array_frame


class SdkTraceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'run.ftrace'

    def harness(self):
        h = ControllerHarness(runtime_id=100, trace_output=self.path)
        self.addCleanup(h.controller.close)
        h.accept_enter()
        h.controller.enter_external(.2)
        node = Node()
        h.transport.node = node
        return h, node

    def endpoint(self, h, node, side='left', touch=False):
        number = 1 if side == 'left' else 2
        client = EndEffectorClient(side, connection=h.controller,
            observation_sources=((SampleStream.EE_STATE, number),))
        self.addCleanup(client.close)
        confirm_info(client, node, info_object(side=number, touch=touch))
        node.emit(side, 'state', state_object(side=number))
        client.wait_output_ready(.5)
        return client

    def records(self, complete=True):
        reader = Reader(self.path)
        rows = list(reader)
        self.assertEqual(reader.integrity.complete, complete, reader.integrity.issues)
        return rows, reader.integrity

    def test_direct_body_left_right_publish_share_writer_and_actual_reservation_keys(self):
        h, node = self.harness()
        left, right = self.endpoint(h, node), self.endpoint(h, node, 'right')
        body = h.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=25)
        selected = left.state.sample_ref
        a = left.publish(target(), input_observations=(selected,))
        b = right.publish(target(), input_observations=(right.state.sample_ref,))
        node.emit('left', 'state', state_object(sample=9))
        h.controller.close()
        rows, status = self.records()
        successful = [r for r in rows if r['kind'] in (Kind.BODY_COMMAND, Kind.EFFECTOR_COMMAND)
                      and r['outcome'] == Outcome.OK]
        self.assertEqual([(r['session_id'], r['command_sequence']) for r in successful],
                         [(body.session_id, body.sequence), (a.key.session_id, a.key.sequence),
                          (b.key.session_id, b.key.sequence)])
        for row in successful:
            matching = [r for r in rows if r['producer'] == row['producer'] and r['attempt_id'] == row['attempt_id']]
            start = next(r for r in matching if r['kind'] == row['kind'] and r['outcome'] == Outcome.UNKNOWN)
            reservation = next(r for r in matching if r['kind'] == Kind.STAGE_EVENT)
            self.assertLessEqual(start['timestamp_us'], reservation['timestamp_us'])
            self.assertLessEqual(reservation['timestamp_us'], row['timestamp_us'])
            self.assertEqual(reservation['command_sequence'], row['command_sequence'])
        link = next(r for r in rows if r['kind'] == Kind.TRACE_LINK and r['command_sequence'] == a.key.sequence)
        self.assertEqual(link['state_id'], selected.sample_id)
        self.assertNotEqual(link['state_id'], left._latest['state'].state_id if left._latest['state'] else 9)
        self.assertEqual(len(list(Path(self.temp.name).iterdir())), 1)
        self.assertEqual(h.controller.trace_health['failed'], False)

    def test_invalid_command_is_logged_before_validation_without_reserving_key(self):
        h, _ = self.harness()
        before = h.controller._next_external_sequence
        with self.assertRaises(InvalidCommandError):
            h.controller.publish(BodyCmd((BodyJointCmd(),)), target_names=_BODY_TEST_TARGETS, required_fields=25)
        self.assertEqual(h.controller._next_external_sequence, before)
        h.controller.close()
        rows, _ = self.records()
        commands = [r for r in rows if r['kind'] == Kind.BODY_COMMAND]
        self.assertEqual([r['outcome'] for r in commands], [Outcome.UNKNOWN, Outcome.FAILED])
        self.assertFalse(any(r['kind'] == Kind.STAGE_EVENT for r in rows))

    def test_submit_failure_keeps_reserved_identity_and_original_exception(self):
        h, node = self.harness()
        endpoint = self.endpoint(h, node)
        primary = OSError('original send failure')
        node.publish_error = primary
        with self.assertRaises(OSError) as caught:
            endpoint.publish(target(), input_observations=(endpoint.state.sample_ref,))
        self.assertIs(caught.exception, primary)
        h.controller.close()
        rows, _ = self.records()
        self.assertTrue(any(r['kind'] == Kind.STAGE_EVENT and r['command_sequence'] == 1 for r in rows))
        self.assertTrue(any(r['kind'] == Kind.EFFECTOR_COMMAND and r['outcome'] == Outcome.FAILED for r in rows))
        self.assertFalse(any(r['kind'] == Kind.EFFECTOR_COMMAND and r['outcome'] == Outcome.OK for r in rows))

    def test_existing_state_and_touch_callbacks_record_without_user_observer(self):
        h, node = self.harness()
        endpoint = self.endpoint(h, node, touch=True)
        node.emit('left', 'touch', touch_object())
        h.controller.close()
        rows, _ = self.records()
        self.assertTrue(any(r['kind'] == Kind.EFFECTOR_FEEDBACK and r['state_id'] == 1 for r in rows))
        payload = next(r for r in rows if r['kind'] == Kind.INPUT_EVENT)
        words = payload['integers'][4:]
        self.assertEqual(b''.join(v.to_bytes(8, 'little') for v in words)[:8],
                         bytes(touch_object().payload))
        self.assertEqual(node.touch_requests, [])

    def test_two_existing_publisher_threads_have_separate_preallocated_rings(self):
        h, node = self.harness()
        clients = [self.endpoint(h, node, side) for side in ('left', 'right')]
        gate = threading.Barrier(3)
        failures = []
        def publish(client):
            try:
                gate.wait(timeout=2)
                client.publish(target(), input_observations=(client.state.sample_ref,))
            except BaseException as error:
                failures.append(error)
        workers = [threading.Thread(target=publish, args=(c,)) for c in clients]
        for worker in workers:
            worker.start()
        gate.wait(timeout=2)
        for worker in workers:
            worker.join(2)
            self.assertFalse(worker.is_alive())
        self.assertEqual(failures, [])
        h.controller.close()
        rows, status = self.records()
        successful = [r for r in rows if r['kind'] == Kind.EFFECTOR_COMMAND and r['outcome'] == Outcome.OK]
        self.assertEqual(len({r['producer'] for r in successful}), 2)
        self.assertEqual(len({r['command_sequence'] for r in successful}), 2)
        self.assertFalse(any(p['wrong_thread_events'] for p in status.run_end['producers']))

    def test_recording_failure_does_not_fail_publish_or_add_cancel(self):
        h, node = self.harness()
        endpoint = self.endpoint(h, node)
        with patch.object(h.controller._trace.session._writer.__class__, 'submit', side_effect=OSError('disk')):
            sent = endpoint.publish(target(), input_observations=(endpoint.state.sample_ref,))
        self.assertEqual(sent.key.sequence, 1)
        self.assertEqual(len(node.commands), 1)
        h.controller.close()
        self.records(complete=False)
        cancels = [m for _, m in h.transport.published if getattr(m, 'operation', None) == ControlOperation.CANCEL]
        self.assertEqual(len(cancels), 1)

    def test_readonly_and_pull_controller_direct_reads_keep_cached_ids(self):
        for cls in (PullStateSource, PullController):
            with self.subTest(cls=cls):
                self.path = Path(self.temp.name) / (cls.__name__ + '.ftrace')
                t = FakeTransport()
                t.poll_control_status = lambda: None
                state = body_array_frame(BodyState(44, tuple(sample_motor() for _ in range(31)), runtime_id=11))
                t.wait_state = lambda timeout: state
                with cls(_transport=t, trace_output=self.path) as c:
                    c.get_robot_info(.1)
                    self.assertIs(c.wait_state(0), state)
                    self.assertIs(c.wait_state(0), state)
                rows, _ = self.records()
                reads = [r for r in rows if r['kind'] == Kind.BODY_READ and r['outcome'] == Outcome.OK]
                self.assertEqual([r['state_id'] for r in reads], [44, 44])
                self.assertEqual(t.published, [])

    def test_disabled_controller_does_not_load_trace_or_change_public_calls(self):
        with patch('locomotion_aorta.trace.Trace', side_effect=AssertionError('trace created')):
            h = ControllerHarness(runtime_id=100)
            self.addCleanup(h.controller.close)
            h.accept_enter()
            h.controller.enter_external(.1)
            h.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=25)
            self.assertIsNone(h.controller.trace_health)
        self.assertFalse(self.path.exists())

    def test_trace_initialization_failure_closes_transport_before_control(self):
        self.path.write_text('existing')
        t = FakeTransport()
        with self.assertRaises(Exception):
            ExternalController(_transport=t, trace_output=self.path)
        self.assertEqual(t.close_count, 1)
        self.assertEqual(t.published, [])
        self.assertEqual(self.path.read_text(), 'existing')

    def test_body_demo_uses_unwrapped_calls_and_one_discovery(self):
        clock = FakeClock()
        c = RecordingController(clock, trace_output=self.path)
        self.addCleanup(c.close)
        with patch.object(c, 'get_robot_info', wraps=c.get_robot_info) as discovery:
            external_control.run(c, timeout=1, sleep=clock.sleep, monotonic=clock.monotonic, emit=lambda _: None)
        discovery.assert_called_once_with(1)
        c.close()
        rows, _ = self.records()
        commands = [r for r in rows if r['kind'] == Kind.BODY_COMMAND and r['outcome'] == Outcome.OK]
        self.assertEqual(len(commands), c.command_count)
        self.assertGreater(len(commands), 0)
        self.assertEqual(c.calls.count('exit_external'), 1)
        self.assertTrue(all(r['integers'][4] == sum(1 << i for i in range(31) if i != 14) for r in commands))

    def test_cli_only_passes_trace_configuration_to_controller(self):
        clock = FakeClock()
        options = []
        @contextmanager
        def factory(**kwargs):
            options.append(kwargs)
            c = RecordingController(clock, trace_output=kwargs['trace_output'])
            try:
                yield c
            finally:
                c.close()
        original = external_control.run
        def execute(controller, **kwargs):
            return original(controller, **kwargs, sleep=clock.sleep, monotonic=clock.monotonic, emit=lambda _: None)
        with patch.object(external_control, 'PullController', factory), \
             patch.object(external_control, 'run', side_effect=execute), \
             patch('sys.argv', ['external_control.py', '--confirm-motion', '--trace=fbs', '--trace-output', str(self.path)]):
            external_control.main()
        self.assertEqual(options[0]['trace_output'], self.path)
        self.records()

    def test_publish_entry_is_recorded_before_waiting_for_controller_lock(self):
        h, _ = self.harness()
        observed = threading.Event()
        original = h.controller._trace._channels[1].producer.emit
        errors = []
        def mark(*args, **kwargs):
            result = original(*args, **kwargs)
            if args[0] == Kind.BODY_COMMAND and kwargs['outcome'] == Outcome.UNKNOWN:
                observed.set()
            return result
        def publish():
            try:
                h.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=25)
            except BaseException as error:
                errors.append(error)
        with patch.object(h.controller._trace._channels[1].producer, 'emit', side_effect=mark):
            with h.controller._condition:
                worker = threading.Thread(target=publish)
                worker.start()
                self.assertTrue(observed.wait(2), 'entry was blocked behind the control mutex')
                self.assertEqual(h.controller._next_external_sequence, 1)
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        h.controller.close()
        self.records()

    def test_readonly_unknown_zero_and_partial_fields_keep_original_masks(self):
        h, node = self.harness()
        endpoint = self.endpoint(h, node)
        from test_end_effector_codec import Field
        wire = state_object(sample=2)
        wire.axes[0].validFields = 0
        wire.axes[0].pos = float('nan')
        node.emit('left', 'state', wire)
        wire = state_object(sample=3)
        wire.axes[0].validFields = Field.POS
        wire.axes[0].pos = 0.
        node.emit('left', 'state', wire)
        h.controller.close()
        rows, _ = self.records()
        states = [r for r in rows if r['kind'] == Kind.EFFECTOR_FEEDBACK and r['state_id'] in (2, 3)]
        self.assertEqual([r['validity'][0] for r in states], [0, 1])
        self.assertEqual([r['values'][0] for r in states], [0., 0.])
        self.assertEqual([r['integers'][10] for r in states], [0, Field.POS])

    def test_trace_close_failure_preserves_primary_sdk_error(self):
        h, node = self.harness()
        client = self.endpoint(h, node)
        primary = OSError('SDK send error')
        node.publish_error = primary
        with patch.object(h.controller._trace.session, 'close', side_effect=OSError('trace close')):
            with self.assertRaises(OSError) as caught:
                with h.controller:
                    client.publish(target(), input_observations=(client.state.sample_ref,))
        self.assertIs(caught.exception, primary)
        h.controller._trace.close()

    def test_thread_capacity_exhaustion_marks_incomplete_but_never_blocks_commands(self):
        h, _ = self.harness()
        errors = []
        def publish():
            try:
                h.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=25)
            except BaseException as error:
                errors.append(error)
        # Existing main thread plus nine distinct workers exceed the fixed 8 channels.
        for _ in range(9):
            worker = threading.Thread(target=publish)
            worker.start()
            worker.join(2)
            self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(h.controller._next_external_sequence, 10)
        h.controller.close()
        self.records(complete=False)
        self.assertGreater(sum(p['invalid_events'] for p in h.controller.trace_health['producers']), 0)

    def test_cold_info_keeps_unknown_ranges_without_extra_discovery_or_zero_fabrication(self):
        reply = robot_info_reply()
        joint = replace(reply.info.body.joints[14], q_min=float('nan'), q_max=float('nan'))
        body = replace(reply.info.body, joints=reply.info.body.joints[:14]+(joint,)+reply.info.body.joints[15:])
        reply = replace(reply, info=replace(reply.info, body=body))
        t = FakeTransport()
        with patch.object(t, 'get_robot_info', return_value=reply) as discover:
            with ExternalController(_transport=t, trace_output=self.path) as c:
                self.assertIs(c.get_robot_info(.1), reply)
            discover.assert_called_once_with(.1)
        rows, _ = self.records()
        parts = [r for r in rows if r['kind'] == Kind.STREAM_INFO]
        raw = bytearray(parts[0]['integers'][1])
        for row in parts:
            offset, total, *words = row['integers']
            data = b''.join(word.to_bytes(8, 'little') for word in words)
            raw[offset:offset+min(len(data),total-offset)] = data[:total-offset]
        info = json.loads(raw)
        self.assertIsNone(info['value']['info']['body']['joints'][14]['q_min'])
        self.assertEqual(info['value']['info']['body']['joints'][14]['valid_limits'], 0)
        self.assertEqual(info['nonfinite']['.info.body.joints.14.q_min'], 'nan')

    def test_capture_reentry_drops_observation_without_mutating_outer_publish(self):
        h, _ = self.harness()
        channel = h.controller._trace._channels[0]
        original = channel._body_command
        reentered = False
        nested = []
        def snapshot(command, ids, required, names):
            nonlocal reentered
            if not reentered:
                reentered = True
                nested.append(h.controller.publish(make_command(0, q=.2),
                    target_names=_BODY_TEST_TARGETS, required_fields=25))
            return original(command, ids, required, names)
        with patch.object(channel, '_body_command', side_effect=snapshot):
            outer = h.controller.publish(make_command(0, q=.1), target_names=_BODY_TEST_TARGETS, required_fields=25)
        self.assertEqual((outer.sequence, nested[0].sequence), (1, 2))
        self.assertAlmostEqual(outer.joints[0].q, .1)
        h.controller.close()
        rows, _ = self.records(complete=False)
        recorded = [r for r in rows if r['kind'] == Kind.BODY_COMMAND and r['outcome'] == Outcome.OK]
        self.assertEqual(len(recorded), 1)
        self.assertEqual(recorded[0]['command_sequence'], 1)
        self.assertAlmostEqual(recorded[0]['values'][0], .1)

    def test_close_during_inflight_publish_marks_recording_incomplete_and_does_not_retry(self):
        h, _ = self.harness()
        entered, release = threading.Event(), threading.Event()
        errors = []
        def blocked(message, guard):
            entered.set()
            if not release.wait(2):
                raise TimeoutError('test submit barrier')
            guard(lambda: None)
        def publish():
            try:
                h.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=25)
            except BaseException as error:
                errors.append(error)
        with patch.object(h.transport, '_publish_body', side_effect=blocked):
            worker = threading.Thread(target=publish)
            worker.start()
            try:
                self.assertTrue(entered.wait(2))
                h.controller.close()
                h.controller.close()
            finally:
                release.set()
                worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.records(complete=False)
        self.assertEqual(h.controller._next_external_sequence, 2)
        cancels = [m for _, m in h.transport.published if getattr(m, 'operation', None) == ControlOperation.CANCEL]
        self.assertEqual(len(cancels), 1)

    def test_user_model_span_measures_only_selected_call_and_preserves_exception(self):
        t = FakeTransport()
        primary = RuntimeError('model inference failed')
        with ExternalController(_transport=t, trace_output=self.path,
                                trace_stages=('actor',)) as controller:
            with patch('locomotion_aorta.trace._sdk.monotonic_us', side_effect=(100, 180)):
                with controller.trace_span('actor', iteration_id=7) as first:
                    pass  # Deterministic CPU model boundary, not a performance measurement.
            with patch('locomotion_aorta.trace._sdk.monotonic_us', side_effect=(200, 250)):
                with self.assertRaises(RuntimeError) as caught:
                    with controller.trace_span('actor', iteration_id=8):
                        raise primary
            self.assertIs(caught.exception, primary)
        rows, _ = self.records()
        spans = [r for r in rows if r['kind'] == Kind.USER_SPAN]
        self.assertEqual([r['duration_us'] for r in spans], [81, 51])
        self.assertEqual([r['outcome'] for r in spans], [Outcome.OK, Outcome.FAILED])
        self.assertEqual([r['iteration_id'] for r in spans], [7, 8])
        self.assertEqual(spans[0]['inference_id'], first.inference_id)
        self.assertNotEqual(spans[0]['inference_id'], spans[1]['inference_id'])

    def test_run_owned_trace_covers_worker_before_and_after_connection_close(self):
        entered, finish = threading.Event(), threading.Event()
        errors = []
        with Trace(self.path, stages=('planner',)) as trace:
            def planner():
                try:
                    with trace.span('planner'):
                        entered.set()
                        if not finish.wait(2):
                            raise TimeoutError('test barrier')
                except BaseException as error:
                    errors.append(error)
            worker = threading.Thread(target=planner)
            worker.start()
            try:
                self.assertTrue(entered.wait(2))
                with ExternalController(_transport=FakeTransport(), trace_recorder=trace) as controller:
                    with controller.trace_span('planner'):
                        pass
                self.assertFalse(trace.session._closed)
            finally:
                finish.set()
                worker.join(2)
            self.assertFalse(worker.is_alive())
            self.assertEqual(errors, [])
        rows, _ = self.records()
        spans = [r for r in rows if r['kind'] == Kind.USER_SPAN]
        self.assertEqual(len(spans), 2)
        self.assertEqual(len({r['producer'] for r in spans}), 2)
        self.assertEqual(len({r['inference_id'] for r in spans}), 2)

    def test_borrowed_recorder_survives_subscription_failure_and_readonly_close(self):
        with Trace(self.path, stages=('after_close',)) as trace:
            t = FakeTransport()
            with patch.object(t, 'subscribe', side_effect=RuntimeError('subscribe')):
                with self.assertRaisesRegex(RuntimeError, 'subscribe'):
                    ExternalController(_transport=t, trace_recorder=trace)
            self.assertFalse(trace.session._closed)
            with PullStateSource(_transport=FakeTransport(), trace_recorder=trace):
                pass
            with PullController(_transport=FakeTransport(), trace_recorder=trace):
                pass
            with trace.span('after_close'):
                pass
        rows, _ = self.records()
        self.assertEqual(sum(r['kind'] == Kind.USER_SPAN for r in rows), 1)

    def test_borrowed_recorder_rejects_conflicting_or_closed_options_before_transport(self):
        trace = Trace(self.path)
        with patch('locomotion_aorta.controller.create_aorta_transport',
                   side_effect=AssertionError('transport constructed')):
            with self.assertRaises(ValueError):
                ExternalController(trace_output=self.path, trace_recorder=trace)
            with self.assertRaises(ValueError):
                ExternalController(trace_stages=('actor',), trace_recorder=trace)
            trace.close()
            with self.assertRaises(ValueError):
                ExternalController(trace_recorder=trace)
        with patch('locomotion_aorta.pull.PullTransport',
                   side_effect=AssertionError('transport constructed')):
            for factory in (PullController, PullStateSource):
                with self.assertRaises(ValueError):
                    factory(trace_recorder=trace)

    def test_offline_span_summary_separates_stages_failures_and_incomplete_capture(self):
        from locomotion_aorta.trace.summary import summarize_spans
        with Trace(self.path, stages=('planner', 'actor')) as trace:
            with patch('locomotion_aorta.trace._sdk.monotonic_us', side_effect=(100, 110, 200, 230, 300, 305)):
                with trace.span('planner'):
                    pass
                with trace.span('actor'):
                    pass
                with self.assertRaisesRegex(RuntimeError, 'failure'):
                    with trace.span('planner'):
                        raise RuntimeError('failure')
        result = summarize_spans(self.path)
        self.assertTrue(result['capture_complete'])
        groups = {(r['stage'], r['outcome']): r for r in result['stages']}
        self.assertEqual(groups['planner', 'OK']['p99_us'], 11)
        self.assertEqual(groups['planner', 'FAILED']['p99_us'], 6)
        self.assertEqual(groups['actor', 'OK']['p99_us'], 31)
        self.assertTrue(all(r['observed_spans'] == 1 for r in groups.values()))
        self.path.write_bytes(self.path.read_bytes()[:-10])
        incomplete = summarize_spans(self.path)
        self.assertFalse(incomplete['capture_complete'])
        self.assertIn('missing_run_end', incomplete['issues'])

    def test_disabled_user_span_reads_no_clock_and_starts_no_recording(self):
        with ExternalController(_transport=FakeTransport()) as controller:
            with patch('locomotion_aorta.trace._sdk.monotonic_us', side_effect=AssertionError('clock')):
                with controller.trace_span('actor'):
                    result = 42
            self.assertEqual(result, 42)
        self.assertFalse(self.path.exists())


if __name__ == '__main__':
    unittest.main()
