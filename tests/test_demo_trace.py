"""Real demo execution and native trace files; synthetic I/O, no robot session."""
from contextlib import contextmanager, redirect_stdout, redirect_stderr
from dataclasses import replace
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import test_examples as body_fixture

from locomotion_aorta import BODY_CMD_TOPIC
from locomotion_aorta.trace import Kind, Outcome
from locomotion_aorta.trace.demo import DemoTrace
from locomotion_aorta.trace.reader import Reader
import test_end_effector_external as fixture
from test_end_effector_codec import decode, touch_object

demo = fixture.demo


class DemoTraceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'run.ftrace'
        self.case = fixture.EndEffectorExampleTest(
            'test_actual_entry_shares_one_owner_allocator_and_original_body_hold')
        self.case.setUp()
        self.addCleanup(self.case.doCleanups)

    def capture(self, controller):
        result = DemoTrace(self.path, controller.get_robot_info(.3), compression='raw')
        self.addCleanup(result.session.close)
        return result

    def run_demo(self, controller, endpoints, trace, *, body_enabled=True):
        options = demo._MotionOptions(timeout=.3, duration=.02, body_enabled=body_enabled,
            native_hz={e.side: 250 for e in endpoints},
            native_config='synthetic-test-effective-config')
        demo.run(controller, endpoints, {e.side: self.case.target(e) for e in endpoints},
            options, sleep=self.case.clock.sleep, monotonic=self.case.clock.monotonic,
            emit=self.case.events.append, trace=trace)

    def records(self):
        reader = Reader(self.path)
        rows = list(reader)
        self.assertTrue(reader.integrity.complete, reader.integrity.issues)
        return rows

    def test_body_and_both_endpoints_use_one_file_and_actual_shared_allocator_keys(self):
        controller = self.case.controller()
        endpoints = self.case.endpoints(controller)
        with self.capture(controller) as trace:
            self.run_demo(controller, endpoints, trace)
        rows = self.records()
        commands = [r for r in rows if r['kind'] in (Kind.BODY_COMMAND, Kind.EFFECTOR_COMMAND)
                    and r['outcome'] == Outcome.OK]
        self.assertEqual({r['stream'] for r in commands}, {1, 2, 3})
        actual = [(cmd.session_id, cmd.sequence) for topic, cmd in controller.transport.published
                  if topic == BODY_CMD_TOPIC]
        actual += [(cmd.key.session_id, cmd.key.sequence)
                   for endpoint in endpoints for cmd in endpoint.commands]
        self.assertCountEqual([(r['session_id'], r['command_sequence']) for r in commands], actual)
        self.assertEqual(len(actual), len(set(actual)))
        self.assertEqual(controller.calls.count('enter_external'), 1)
        self.assertEqual(controller.calls.count('exit_external'), 1)
        for endpoint in endpoints:
            self.assertTrue(endpoint.commands)
            self.assertEqual(endpoint.commands[-1].target.pos, (.0625,))
        for row in commands:
            self.assertGreater(row['attempt_id'], 0)
            starts = [r for r in rows if r['kind'] == row['kind']
                      and r['attempt_id'] == row['attempt_id'] and r['outcome'] == Outcome.UNKNOWN]
            self.assertEqual(len(starts), 1)
            self.assertLessEqual(starts[0]['timestamp_us'], row['timestamp_us'])
        self.assertEqual(len(list(Path(self.temp.name).iterdir())), 1)

    def test_ee_only_keeps_zero_body_commands_and_links_selected_reference_not_latest(self):
        controller = self.case.controller()
        endpoint = fixture.Endpoint(controller)
        endpoint.after_publish = lambda _: setattr(endpoint, '_state',
            replace(endpoint._state, state_id=endpoint._state.state_id+10))
        with self.capture(controller) as trace:
            self.run_demo(controller, [endpoint], trace, body_enabled=False)
        rows = self.records()
        self.assertFalse(any(topic == BODY_CMD_TOPIC for topic, _ in controller.transport.published))
        self.assertFalse(any(r['kind'] == Kind.BODY_COMMAND for r in rows))
        links = [r for r in rows if r['kind'] == Kind.TRACE_LINK]
        self.assertEqual([r['state_id'] for r in links],
                         [c.input_observations[0].sample_id for c in endpoint.commands])
        self.assertEqual([r['command_sequence'] for r in links],
                         [c.key.sequence for c in endpoint.commands])
        for row, command in zip(links, endpoint.commands):
            ref = command.input_observations[0]
            self.assertEqual(row['integers'], [ref.runtime_id, ref.stream, ref.side, ref.validity])
            self.assertEqual(row['times_us'], [ref.source_time_us])

    def test_cached_body_sample_ids_and_original_source_times_are_not_refreshed(self):
        controller = self.case.controller(advance_lowstate=False)
        with self.capture(controller) as trace:
            first = trace.read_body(controller, .3)
            second = trace.read_body(controller, .3)
        reads = [r for r in self.records() if r['kind'] == Kind.BODY_READ and r['outcome'] == Outcome.OK]
        self.assertEqual([r['state_id'] for r in reads], [first.state_id, second.state_id])
        self.assertEqual(first.state_id, second.state_id)
        self.assertEqual([r['times_us'][0] for r in reads],
                         [first.ts_state_real_us, second.ts_state_real_us])

    def test_recording_and_close_faults_preserve_primary_publish_fault_and_single_cancel(self):
        controller = self.case.controller()
        endpoint = fixture.Endpoint(controller)
        primary = TimeoutError('original endpoint publish fault')
        trace = self.capture(controller)
        with patch.object(endpoint, 'publish', side_effect=primary), \
             patch.object(trace.producer, 'emit', side_effect=OSError('recorder failed')), \
             patch.object(trace.session, 'close', side_effect=OSError('close failed')):
            with self.assertRaises(TimeoutError) as caught:
                with trace:
                    self.run_demo(controller, [endpoint], trace)
        self.assertIs(caught.exception, primary)
        self.assertEqual(controller.calls.count('exit_external'), 1)
        self.assertFalse(endpoint.commands)
        self.assertGreater(trace.health()['producers'][0]['invalid_events'], 0)

    def test_nonblocking_scratch_reentry_drops_observation_without_blocking_publish(self):
        controller = self.case.controller()
        endpoint = fixture.Endpoint(controller)
        with self.capture(controller) as trace:
            trace._lock.acquire()
            try:
                self.run_demo(controller, [endpoint], trace)
            finally:
                trace._lock.release()
        self.assertTrue(endpoint.commands)
        self.assertEqual(controller.calls.count('exit_external'), 1)
        reader = Reader(self.path)
        list(reader)
        self.assertFalse(reader.integrity.complete)
        self.assertGreater(trace.health()['producers'][0]['invalid_events'], 0)

    def test_touch_confirmation_preserves_original_region_stamps_and_payload_bytes(self):
        controller = self.case.controller()
        endpoint = fixture.Endpoint(controller, touch=True)
        sample = decode('touch', touch_object(side=2))
        with self.capture(controller) as trace:
            trace.touch_confirmation(endpoint._state, sample)
        rows = self.records()
        payload = bytearray(len(sample.payload))
        for row in rows:
            if row['kind'] == Kind.INPUT_EVENT:
                layout, side, offset, total, *words = row['integers']
                self.assertEqual((layout, side, total), (sample.layout_id, sample.side, len(sample.payload)))
                data = b''.join(word.to_bytes(8, 'little') for word in words)
                payload[offset:offset+min(len(data), total-offset)] = data[:total-offset]
        self.assertEqual(bytes(payload), sample.payload)
        regions = [r for r in rows if r['kind'] == Kind.STREAM_INFO]
        self.assertEqual(len(regions), len(sample.region_stamps))
        for row, source in zip(regions, sample.region_stamps):
            self.assertEqual(row['state_id'], sample.touch_state_id)
            self.assertEqual(row['integers'][3:], [source.sample_seq, source.status_flags,
                                                 source.direction_flags, source.validity])
            self.assertEqual(row['times_us'], [source.ts_request_us, source.ts_acquired_hw_us,
                                              source.ts_source_us, source.ts_ingested_us])

    def test_cli_trace_arguments_fail_before_any_connection(self):
        for args in (['--trace=fbs'], ['--trace=fbs', '--trace-output', 'wrong.jsonl'],
                     ['--trace=off', '--trace-output', str(self.path)]):
            with self.subTest(args=args), redirect_stderr(io.StringIO()), \
                 patch.object(demo, 'PullStateSource') as factory, \
                 self.assertRaises(SystemExit) as caught:
                demo.main(args)
            self.assertEqual(caught.exception.code, 2)
            factory.assert_not_called()
        self.assertFalse(self.path.exists())

    def test_actual_readonly_cli_writes_trace_without_control_and_default_off_creates_nothing(self):
        controller = self.case.controller()

        @contextmanager
        def source(**_):
            yield controller

        with redirect_stdout(io.StringIO()), patch(
                'locomotion_aorta.trace.demo.DemoTrace', side_effect=AssertionError('trace opened')):
            demo.main(['--side', 'none'], _state_source_factory=source)
        self.assertFalse(self.path.exists())
        with redirect_stdout(io.StringIO()):
            demo.main(['--side', 'none', '--trace=fbs', '--trace-output', str(self.path),
                       '--trace-compression', 'raw'], _state_source_factory=source)
        self.assertTrue(any(r['kind'] == Kind.BODY_READ for r in self.records()))
        self.assertNotIn('enter_external', controller.calls)
        self.assertFalse(any(topic == BODY_CMD_TOPIC for topic, _ in controller.transport.published))

    def test_actual_motion_cli_reaches_trace_and_keeps_cancel_and_client_cleanup(self):
        controller = self.case.controller()
        endpoint = fixture.Endpoint(controller, side='left', hand=True)
        cleanup = []

        @contextmanager
        def connection(**_):
            try:
                yield controller
            finally:
                cleanup.append(('connection', controller.calls.count('exit_external')))

        @contextmanager
        def client(side, **_):
            self.assertEqual(side, 'left')
            try:
                yield endpoint
            finally:
                cleanup.append(('client', controller.calls.count('exit_external')))

        original_run = demo.run

        def clocked(*args, **kwargs):
            return original_run(*args, sleep=self.case.clock.sleep,
                                monotonic=self.case.clock.monotonic, **kwargs)

        with redirect_stdout(io.StringIO()), patch.object(demo, 'run', side_effect=clocked):
            demo.main(['--side', 'left', '--confirm-motion', '--duration', '.02',
                       '--left-unit', 'rad', '--left-pos', *(['.1'] * 6),
                       '--trace=fbs', '--trace-output', str(self.path),
                       '--trace-compression', 'raw'],
                      _controller_factory=connection, _client_factory=client)
        rows = self.records()
        self.assertTrue(any(r['kind'] == Kind.BODY_COMMAND and r['outcome'] == Outcome.OK for r in rows))
        self.assertTrue(any(r['kind'] == Kind.EFFECTOR_COMMAND and r['outcome'] == Outcome.OK for r in rows))
        self.assertEqual(controller.calls.count('exit_external'), 1)
        self.assertEqual(cleanup, [('client', 1), ('connection', 1)])

    def test_unknown_endpoint_values_stay_distinct_from_valid_zero(self):
        controller = self.case.controller()
        endpoint = fixture.Endpoint(controller)
        with self.capture(controller) as trace:
            trace.endpoint_feedback(replace(endpoint._state, valid_fields=(0,), pos=(float('nan'),)),
                                    None, None, None)
            trace.endpoint_feedback(replace(endpoint._state, valid_fields=(fixture.Field.POS,), pos=(0.,)),
                                    None, None, None)
        rows = [r for r in self.records() if r['kind'] == Kind.EFFECTOR_FEEDBACK]
        self.assertEqual([r['values'][0] for r in rows], [0., 0.])
        self.assertEqual([r['validity'][0] for r in rows], [0, 1])
        self.assertEqual([r['integers'][10] for r in rows], [0, fixture.Field.POS])

    def test_full_body_demo_preserves_trajectory_scope_and_uses_one_discovery(self):
        controller = self.case.controller()
        output = []
        with patch.object(controller, 'get_robot_info', wraps=controller.get_robot_info) as discovery:
            body_fixture.external_control.run(controller, timeout=1.,
                sleep=self.case.clock.sleep, monotonic=self.case.clock.monotonic,
                emit=output.append, trace_output=self.path, trace_compression='raw')
        discovery.assert_called_once_with(1.)
        rows = self.records()
        commands = [message for topic, message in controller.transport.published if topic == BODY_CMD_TOPIC]
        recorded = [r for r in rows if r['kind'] == Kind.BODY_COMMAND and r['outcome'] == Outcome.OK]
        self.assertEqual([(r['session_id'], r['command_sequence']) for r in recorded],
                         [(c.session_id, c.sequence) for c in commands])
        self.assertTrue(commands)
        self.assertAlmostEqual(commands[-1].joints[0].q, .1)
        self.assertEqual(commands[-1].joints[14].q, 0.)
        self.assertEqual(commands[-1].joints[18].q, 0.)
        self.assertEqual(commands[-1].joints[25].q, 0.)
        target_mask = sum(1 << i for i in range(31) if i != 14)
        self.assertTrue(all(r['integers'][2:] == [0x19, 1, target_mask, 1] for r in recorded))
        self.assertEqual(controller.calls.count('enter_external'), 1)
        self.assertEqual(controller.calls.count('exit_external'), 1)
        self.assertEqual(sum(line.startswith('Body Info:') for line in output), 1)
        self.assertEqual(output[-1], 'CANCEL confirmed PASSIVE')
        self.assertTrue(all(b-a >= .02-1e-9
                            for a, b in zip(controller.command_times, controller.command_times[1:])))
        self.assertFalse(any(r['kind'] == Kind.EFFECTOR_COMMAND for r in rows))

    def test_body_cli_uses_real_trace_path_and_requires_existing_motion_confirmation(self):
        controller = self.case.controller()
        original = body_fixture.external_control.run

        @contextmanager
        def connection(**_):
            yield controller

        def clocked(*args, **kwargs):
            return original(*args, sleep=self.case.clock.sleep, monotonic=self.case.clock.monotonic,
                            emit=self.case.events.append, **kwargs)

        with patch.object(body_fixture.external_control, 'PullController', side_effect=connection), \
             patch.object(body_fixture.external_control, 'run', side_effect=clocked), \
             patch('sys.argv', ['external_control.py', '--confirm-motion', '--trace=fbs',
                               '--trace-output', str(self.path), '--trace-compression', 'raw']):
            body_fixture.external_control.main()
        self.assertTrue(any(r['kind'] == Kind.BODY_COMMAND and r['outcome'] == Outcome.OK
                            for r in self.records()))
        self.assertEqual(controller.calls.count('exit_external'), 1)
        with redirect_stderr(io.StringIO()), \
             patch.object(body_fixture.external_control, 'PullController') as factory, \
             patch('sys.argv', ['external_control.py', '--trace=fbs',
                               '--trace-output', str(Path(self.temp.name) / 'not-created.ftrace')]), \
             self.assertRaises(SystemExit) as caught:
            body_fixture.external_control.main()
        self.assertEqual(caught.exception.code, 2)
        factory.assert_not_called()
        self.assertFalse((Path(self.temp.name) / 'not-created.ftrace').exists())

    def test_body_stale_preflight_still_blocks_entry_and_preserves_cached_ids(self):
        controller = self.case.controller(advance_lowstate=False)
        with self.assertRaisesRegex(RuntimeError, 'stale during the preflight'):
            body_fixture.external_control.run(controller, timeout=1.,
                sleep=self.case.clock.sleep, monotonic=self.case.clock.monotonic,
                emit=self.case.events.append, trace_output=self.path, trace_compression='raw')
        rows = self.records()
        self.assertNotIn('enter_external', controller.calls)
        self.assertFalse(any(topic == BODY_CMD_TOPIC for topic, _ in controller.transport.published))
        reads = [r for r in rows if r['kind'] == Kind.BODY_READ and r['outcome'] == Outcome.OK]
        self.assertGreater(len(reads), 1)
        self.assertEqual(len({r['state_id'] for r in reads}), 1)
        self.assertTrue(any(r['kind'] == Kind.ERROR and r['outcome'] == Outcome.FAILED for r in rows))

    def test_all_vendor_motion_entries_reach_native_trace_without_body_commands(self):
        for model in (fixture.DeviceModel.BRAINCO_REVO2, fixture.DeviceModel.AGX,
                      fixture.DeviceModel.LINKER_L6, fixture.DeviceModel.AGILINK_OP3):
            with self.subTest(model=model):
                self.path = Path(self.temp.name) / (str(int(model)) + '.ftrace')
                case = fixture.VendorExamplesTest('test_vendor_models_reach_shared_main_and_original_run')
                case.setup_case()
                self.addCleanup(case.doCleanups)
                case.invoke(model, case.motion_flags(model) + [
                    '--trace=fbs', '--trace-output', str(self.path), '--trace-compression', 'raw'])
                rows = self.records()
                commands = case.endpoints['right'].commands
                emitted = [r for r in rows if r['kind'] == Kind.EFFECTOR_COMMAND and r['outcome'] == Outcome.OK]
                self.assertEqual([r['command_sequence'] for r in emitted], [c.key.sequence for c in commands])
                self.assertTrue(commands)
                self.assertEqual(case.control.command_count, 0)
                self.assertFalse(any(r['kind'] == Kind.BODY_COMMAND for r in rows))
                self.assertEqual(case.control.calls.count('enter_external'), 1)
                self.assertEqual(case.control.calls.count('exit_external'), 1)

    def test_masked_nonfinite_info_is_recorded_as_unknown_without_new_admission_gate(self):
        controller = self.case.controller()
        reply = replace(body_fixture.example_robot_info_reply(), runtime_id=100)
        joint = replace(reply.info.body.joints[14], q_min=float('nan'), q_max=float('nan'))
        info = replace(reply.info.body, joints=reply.info.body.joints[:14] + (joint,) + reply.info.body.joints[15:])
        controller.transport.get_robot_info = lambda timeout: replace(reply, info=replace(reply.info, body=info))
        body_fixture.external_control.run(controller, timeout=1.,
            sleep=self.case.clock.sleep, monotonic=self.case.clock.monotonic,
            emit=self.case.events.append, trace_output=self.path, trace_compression='raw')
        reader = Reader(self.path)
        list(reader)
        self.assertTrue(reader.integrity.complete, reader.integrity.issues)
        header = reader.integrity.run_header
        self.assertIsNone(header['robot_info']['body']['joints'][14]['q_min'])
        self.assertEqual(header['info_nonfinite_fields']['.body.joints.14.q_min'], 'nan')
        self.assertEqual(header['robot_info']['body']['joints'][14]['valid_limits'], 0)
        self.assertEqual(controller.calls.count('exit_external'), 1)


if __name__ == '__main__':
    unittest.main()
