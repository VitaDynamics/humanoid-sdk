"""Software timing and safe-exit evidence, using synthetic endpoint feedback."""
from dataclasses import asdict, replace
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from examples import policy_support
from examples.end_effector import end_effector_external as demo
from examples.end_effector.cycle_log import CycleRecorder
from locomotion_aorta.end_effector_types import DeviceModel, Field
import test_end_effector_external as fixtures


class EndEffectorCycleLogTest(unittest.TestCase):
    def setUp(self):
        self.case = fixtures.VendorExamplesTest()
        self.case.setup_case()
        self.addCleanup(self.case.doCleanups)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'cycles.jsonl'
        self.clock, self.control = self.case.clock, self.case.control
        self.clients = [self.case.endpoint(side, DeviceModel.AGILINK_OP3) for side in ('left', 'right')]

    def recorder(self):
        return CycleRecorder(self.path, {'test': 'synthetic'}, support=policy_support,
                             monotonic=self.clock.monotonic)

    def run_case(self, recorder, **kwargs):
        targets = {client.side: demo.position_target(client.info, (.055,), unit='m',
                    vel_limit=(.04,), current=(0.,)) for client in self.clients}
        body_enabled=kwargs.pop('body_enabled',False)
        options = demo._MotionOptions(timeout=.4 if body_enabled else .2, duration=.12, body_enabled=body_enabled,
            rates={'left': 25., 'right': 50.}, recorder=recorder, **kwargs)
        demo.run(self.control, self.clients, targets, options, sleep=self.clock.sleep,
                 monotonic=self.clock.monotonic, emit=lambda *_: None)

    def rows(self):
        return [json.loads(line) for line in self.path.read_text().splitlines()]

    def test_each_publication_records_actual_keys_references_masks_and_separate_cadence(self):
        with self.recorder() as recorder:
            self.run_case(recorder)
        rows = self.rows()
        self.assertEqual('vita.end_effector_latency.v1', rows[0]['schema'])
        info = next(row for row in rows if row['record_type'] == 'end_effector_info')
        self.assertEqual(list(self.clients[0].info.axis_names), info['endpoints']['left']['axis_names'])
        cycles = [row for row in rows if row['record_type'] == 'cycle']
        commands = {(client.side, sent.key.sequence): sent
                    for client in self.clients for sent in client.commands}
        self.assertEqual(len(commands), len(cycles))
        self.assertEqual(list(range(len(cycles))), [row['cycle'] for row in cycles])
        self.assertEqual(len(cycles), len([row for row in rows if row['record_type'] == 'logging_overhead']))
        for row in cycles:
            sent = commands[(row['side'], row['sequence'])]
            self.assertEqual(sent.key.session_id, row['session_id'])
            self.assertEqual([asdict(ref) for ref in sent.input_observations], row['input_observations'])
            self.assertEqual(row['position_sample_ref'], row['input_observations'][0])
            self.assertEqual([Field.POS | Field.VEL_LIMIT | Field.CURRENT], row['submitted_target']['command_fields'])
            self.assertEqual([0.], row['submitted_target']['current'])
            self.assertEqual('returned', row['publish_result'])
            for name in ('S1', 'S2', 'S3', 'S4', 'L1', 'L2', 'publish_timing'):
                self.assertEqual('NOT_MEASURED', row[name])
        summary = next(row for row in rows if row['record_type'] == 'end_effector_summary')
        self.assertLess(summary['sides']['left']['attempts'], summary['sides']['right']['attempts'])
        for side, hz in (('left', 25.), ('right', 50.)):
            side_rows = [row for row in cycles if row['side'] == side]
            self.assertEqual('NOT_MEASURED', side_rows[0]['control_interval_ns'])
            self.assertAlmostEqual(hz, summary['sides'][side]['observed_attempt_hz'], places=4)
            self.assertEqual(len(side_rows), summary['sides'][side]['returned'])
        self.assertEqual('logging_summary', rows[-1]['record_type'])
        self.assertEqual(len(cycles), rows[-1]['N'])
        self.assertEqual(1, self.control.calls.count('exit_external'))

    def test_cached_position_keeps_its_original_receipt_and_latest_status_identity(self):
        for client in self.clients:
            original, received = client._state, client._position_received_at
            client.motion_feedback = lambda c=client, s=original, r=received: (c.state, s, None, r)
        with self.recorder() as recorder:
            self.run_case(recorder)
        cycles = [row for row in self.rows() if row['record_type'] == 'cycle']
        for row in cycles:
            self.assertNotEqual(row['latest_status']['state_id'], row['position_state']['state_id'])
            self.assertEqual(0, row['position_received_monotonic_ns'])
            self.assertEqual(row['position_state']['state_id'], row['position_sample_ref']['sample_id'])
        self.assertGreater(cycles[-1]['feedback_age_ns'], 0)

    def test_publisher_exception_is_recorded_without_a_guessed_command_key(self):
        self.clients[0].publish = Mock(side_effect=RuntimeError('synthetic publish failure'))
        with self.assertRaisesRegex(RuntimeError, 'synthetic publish failure'):
            with self.recorder() as recorder:
                self.run_case(recorder)
        row = next(row for row in self.rows() if row['record_type'] == 'cycle')
        self.assertEqual('raised', row['publish_result'])
        self.assertEqual('NOT_MEASURED', row['sequence'])
        self.assertEqual('RuntimeError', row['error_type'])
        self.assertEqual(1, self.control.calls.count('exit_external'))

    def test_write_failure_cancels_after_returned_publication(self):
        with self.assertRaisesRegex(OSError, 'synthetic disk full'):
            with self.recorder() as recorder:
                recorder.log.cycle = Mock(side_effect=OSError('synthetic disk full'))
                self.run_case(recorder)
        self.assertEqual(1, len(self.clients[0].commands))
        self.assertEqual(1, self.control.calls.count('exit_external'))

    def test_cleanup_and_write_failure_do_not_mask_primary_publisher_fault(self):
        self.clients[0].publish = Mock(side_effect=RuntimeError('primary failure'))
        with self.assertRaisesRegex(RuntimeError, 'primary failure'):
            with self.recorder() as recorder:
                recorder.log.cycle = Mock(side_effect=OSError('secondary disk failure'))
                recorder.close = Mock(side_effect=OSError('secondary close failure'))
                self.run_case(recorder)
        self.assertTrue(recorder.log.stream.closed)
        self.assertEqual(1, self.control.calls.count('exit_external'))

    def test_call_duration_excludes_jsonl_recording_and_uses_monotonic_clock(self):
        original = self.clients[0].publish
        def slow_publish(*args, **kwargs):
            self.clock.sleep(.003)
            return original(*args, **kwargs)
        self.clients[0].publish = slow_publish
        with self.recorder() as recorder:
            original_write = recorder.log.cycle
            def slow_write(row):
                self.clock.sleep(.007)
                original_write(row)
            recorder.log.cycle = slow_write
            # A wall-clock step does not become a negative call duration.
            with patch.object(policy_support, 'realtime_ns', side_effect=lambda: -int(self.clock.monotonic()*1e9)):
                self.run_case(recorder)
        cycles = [row for row in self.rows() if row['record_type'] == 'cycle']
        left = [row for row in cycles if row['side'] == 'left']
        self.assertTrue(all(abs(row['publish_call_ns'] - 3_000_000) <= 1 for row in left))
        self.assertTrue(all(row['S6'] < row['S5'] for row in left))
        self.assertTrue(all(row['publish_call_ns'] < 7_000_000 for row in left))

    def test_optional_nonfinite_measurements_become_null_with_masks_preserved(self):
        for client in self.clients:
            client._state = replace(client._state, current=(float('nan'),))
        with self.recorder() as recorder:
            self.run_case(recorder)
        row = next(row for row in self.rows() if row['record_type'] == 'cycle')
        self.assertEqual([None], row['position_state']['current'])
        self.assertEqual([Field.POS], row['position_state']['valid_fields'])

    def test_existing_log_path_fails_before_constructing_a_connection(self):
        self.path.write_text('keep existing evidence\n')
        controller, client = Mock(), Mock()
        flags = self.case.motion_flags(DeviceModel.AGILINK_OP3) + ['--log', str(self.path)]
        with self.assertRaises(FileExistsError):
            demo.main(flags, _expected_device_model=DeviceModel.AGILINK_OP3,
                      _controller_factory=controller, _client_factory=client)
        controller.assert_not_called()
        client.assert_not_called()
        self.assertEqual('keep existing evidence\n', self.path.read_text())

    def test_cli_log_option_reaches_the_real_shared_publisher_and_finalizes(self):
        flags = self.case.motion_flags(DeviceModel.AGILINK_OP3, side='both') + ['--log', str(self.path)]
        self.case.invoke(DeviceModel.AGILINK_OP3, flags)
        rows = self.rows()
        commands = [sent for endpoint in self.case.endpoints.values() for sent in endpoint.commands]
        cycles = [row for row in rows if row['record_type'] == 'cycle']
        self.assertEqual(len(commands), len(cycles))
        self.assertEqual({sent.key.sequence for sent in commands}, {row['sequence'] for row in cycles})
        self.assertEqual('logging_summary', rows[-1]['record_type'])
        self.assertEqual(1, self.control.calls.count('enter_external'))
        self.assertEqual(1, self.control.calls.count('exit_external'))

    def test_six_axis_hand_records_all_axes_and_original_masks(self):
        client = self.case.endpoint('right', DeviceModel.BRAINCO_REVO2)
        target = demo.position_target(client.info, (.055,)*6, unit='rad', vel_limit=(.04,)*6)
        with self.recorder() as recorder:
            options = demo._MotionOptions(timeout=.2, duration=.04, body_enabled=False, recorder=recorder)
            demo.run(self.control, [client], {'right':target}, options, sleep=self.clock.sleep,
                     monotonic=self.clock.monotonic, emit=lambda *_: None)
        row = next(row for row in self.rows() if row['record_type'] == 'cycle')
        self.assertEqual(6, len(row['position_state']['pos']))
        self.assertEqual([Field.POS | Field.VEL | Field.CURRENT]*6, row['position_state']['valid_fields'])
        self.assertEqual([Field.POS | Field.VEL_LIMIT]*6, row['submitted_target']['command_fields'])

    def test_body_and_both_endpoints_have_separate_cycles_and_one_shared_allocator(self):
        self.clock.sleep(1.)
        original = self.control.wait_state
        def stamped_state(timeout):
            state = original(timeout)
            state.received_monotonic_ns = int(self.clock.monotonic()*1e9)-1_000_000
            state.received_realtime_ns = 123_000_000
            return state
        self.control.wait_state = stamped_state
        with self.recorder() as recorder:
            self.run_case(recorder, body_enabled=True)
        rows = self.rows()
        body_cycles = [r for r in rows if r['record_type']=='body_cycle']
        reads = {r['cycle']:r for r in rows if r['record_type']=='body_state_read'}
        ee_cycles = [r for r in rows if r['record_type']=='cycle']
        self.assertTrue(body_cycles and reads and ee_cycles)
        for row in body_cycles:
            read = reads[row['input_read_cycle']]
            self.assertEqual(row['input_state_id'],read['state_id'])
            self.assertEqual(row['state_id'],read['state_id'])
            self.assertEqual(row['S2'],read['S2'])
            self.assertEqual(123_000_000,row['S1'])
            self.assertGreaterEqual(row['feedback_age_ns'],1_000_000)
            self.assertEqual(row['publish_started_monotonic_ns']+row['publish_call_ns']-row['state_received_monotonic_ns'],
                             row['receipt_to_publish_return_ns'])
            self.assertEqual(31,len(row['measured_q']))
            self.assertEqual(31,len(row['target_q']))
        publications=body_cycles+ee_cycles
        self.assertEqual(set(range(1,len(publications)+1)),{r['sequence'] for r in publications})
        self.assertEqual({17},{r['session_id'] for r in publications})
        summaries=[r for r in rows if r['record_type']=='body_summary']
        self.assertEqual(len(body_cycles),summaries[0]['publications']['returned'])
        self.assertIsInstance(summaries[0]['publications']['receipt_to_publish_return_ms']['p99'],float)
        self.assertEqual(len(reads),summaries[0]['reads']['returned'])
        self.assertEqual(1,self.control.calls.count('enter_external'))
        self.assertEqual(1,self.control.calls.count('exit_external'))

    def test_body_read_timeout_is_logged_and_cancels_before_any_ee_command(self):
        original=self.control.wait_state
        def fail_after_entry(timeout):
            if 'enter_external' in self.control.calls:
                self.clock.sleep(.003)
                raise TimeoutError('synthetic body read timeout')
            return original(timeout)
        self.control.wait_state=fail_after_entry
        with self.assertRaisesRegex(TimeoutError,'synthetic body read timeout'):
            with self.recorder() as recorder:
                self.run_case(recorder,body_enabled=True)
        read=next(r for r in self.rows() if r['record_type']=='body_state_read')
        self.assertEqual('raised',read['read_result'])
        self.assertAlmostEqual(3_000_000,read['state_read_call_ns'],delta=1)
        self.assertTrue(all(not c.commands for c in self.clients))
        self.assertEqual(1,self.control.calls.count('exit_external'))

    def test_body_publish_failure_preserves_primary_error_and_cancels_once(self):
        self.control.publish=Mock(side_effect=RuntimeError('synthetic body publish failure'))
        with self.assertRaisesRegex(RuntimeError,'synthetic body publish failure'):
            with self.recorder() as recorder:
                self.run_case(recorder,body_enabled=True)
        row=next(r for r in self.rows() if r['record_type']=='body_cycle')
        self.assertEqual('raised',row['publish_result'])
        self.assertEqual('NOT_MEASURED',row['sequence'])
        self.assertTrue(all(not c.commands for c in self.clients))
        self.assertEqual(1,self.control.calls.count('exit_external'))

    def test_log_requires_motion_and_a_selected_endpoint(self):
        for flags in (['--dry-run'], ['--side', 'none', '--confirm-motion']):
            with self.subTest(flags=flags), patch('sys.stderr', new_callable=io.StringIO):
                controller, source = Mock(), Mock()
                with self.assertRaises(SystemExit):
                    demo.main(flags + ['--log', str(self.path)],
                              _controller_factory=controller, _state_source_factory=source)
                controller.assert_not_called()
                source.assert_not_called()
                self.assertFalse(self.path.exists())


if __name__ == '__main__':
    unittest.main()
