"""WBC logger backpressure and failure paths; no transport or robot commands."""
import hashlib
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

try:
    import numpy as np
except ImportError:
    raise unittest.SkipTest('install .[wbc-tracking-demo] for WBC log tests')

from examples.wbc_tracking_mode import wbc_tracking_mode as example
from test_wbc_tracking_policy import FakeClock, FakeController, FakeModel, policy


class BlockingStream(io.StringIO):
    def __init__(self, *, fail=False):
        super().__init__()
        self.entered, self.release = threading.Event(), threading.Event()
        self.fail = fail
        self.writer_threads = []
        self.flushed = False
        self.saved = ''

    def write(self, text):
        self.writer_threads.append(threading.get_ident())
        self.entered.set()
        if not self.release.wait(5):
            raise TimeoutError('test did not release blocked writer')
        if self.fail:
            raise OSError('disk unavailable')
        return super().write(text)

    def flush(self):
        self.flushed = True
        super().flush()

    def close(self):
        self.saved = self.getvalue()
        super().close()


class WbcLoggingTest(unittest.TestCase):
    def rows(self, *, full=False):
        observation = np.array([0., -0., 1.25, -2.5], dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'run.jsonl'
            recorder = example.Recorder(path, {}, log_observation=full)
            recorder.write(dict(kind='step', observation=observation,
                                state=dict(q=np.array([.2]), dq=np.array([.3])),
                                decoded=dict(q=[.4], tau_ff=[.5])))
            recorder.write(dict(kind='summary', steps=1))
            recorder.close()
            records = [json.loads(line) for line in path.read_text().splitlines()]
        return observation, records

    def test_default_observation_is_length_and_little_endian_float32_digest(self):
        observation, records = self.rows()
        row = records[1]
        self.assertNotIn('observation', row)
        self.assertEqual(row['observation_length'], 4)
        self.assertEqual(row['observation_sha256'], hashlib.sha256(observation.astype('<f4').tobytes()).hexdigest())
        self.assertEqual(row['state'], dict(q=[.2], dq=[.3]))
        self.assertEqual(row['decoded'], dict(q=[.4], tau_ff=[.5]))
        self.assertEqual(records[-1]['kind'], 'summary')
        self.assertEqual(records[-1]['logging']['records_written'], 3)

    def test_full_option_preserves_observation_and_same_summary_and_feedback(self):
        observation, full = self.rows(full=True)
        _, compact = self.rows()
        self.assertEqual(full[1]['observation'], observation.tolist())
        for key in ('observation_length', 'observation_sha256', 'decoded', 'state'):
            self.assertEqual(full[1][key], compact[1][key])

    def test_full_queue_drops_without_waiting_for_blocked_writer_and_keeps_summary(self):
        stream = BlockingStream()
        with patch.object(example.Path, 'open', return_value=stream):
            recorder = example.Recorder('unused', {}, capacity=1)
        try:
            self.assertTrue(stream.entered.wait(2))
            self.assertTrue(recorder.write(dict(kind='step', cycle=1)))
            self.assertFalse(recorder.write(dict(kind='step', cycle=2)))
            recorder.write(dict(kind='summary', steps=2))
            self.assertFalse(stream.release.is_set())
            self.assertEqual(recorder.statistics()['records_dropped'], 1)
        finally:
            stream.release.set()
            recorder.close()
        rows = [json.loads(line) for line in stream.saved.splitlines()]
        self.assertEqual([row['kind'] for row in rows], ['header', 'step', 'summary'])
        self.assertEqual(rows[-1]['steps'], 2)
        self.assertEqual(rows[-1]['logging']['queue_max_depth'], 1)
        self.assertEqual(rows[-1]['logging']['records_dropped'], 1)
        self.assertEqual(rows[-1]['logging']['records_written'], 3)
        self.assertTrue(stream.flushed)
        self.assertTrue(all(t != threading.get_ident() for t in stream.writer_threads))

    def test_close_drains_all_accepted_rows_flushes_and_joins(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'run.jsonl'
            recorder = example.Recorder(path, {}, capacity=128)
            for cycle in range(40):
                self.assertTrue(recorder.write(dict(kind='step', cycle=cycle)))
            recorder.close()
            recorder.close()
            rows = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual([row['cycle'] for row in rows if row['kind']=='step'], list(range(40)))
        self.assertEqual(rows[-1]['kind'], 'logging_summary')
        self.assertEqual(rows[-1]['logging']['records_written'], 42)
        self.assertEqual(rows[-1]['logging']['records_dropped'], 0)
        self.assertFalse(recorder._thread.is_alive())
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            recorder.write({})

    def test_writer_failure_does_not_prevent_live_cancel_then_reports_error(self):
        stream = BlockingStream(fail=True)
        stream.release.set()
        with patch.object(example.Path, 'open', return_value=stream):
            recorder = example.Recorder('unused', {})
        clock, p = FakeClock(), policy()
        controller = FakeController(p, clock)
        args = type('Args', (), dict(motion=None, timeout=1., max_state_age_ms=50.,
                                    max_compute_ms=50., stand_seconds=3., duration=.041))()
        try:
            result = example.live(FakeModel(), p, args, recorder, controller,
                                  source=controller, monotonic=clock.monotonic, sleep=clock.sleep)
            self.assertEqual(result['steps'], 3)
            self.assertEqual(controller.exits, 1)
        finally:
            with self.assertRaisesRegex(OSError, 'disk unavailable'):
                recorder.close()
        self.assertGreater(recorder.statistics()['records_dropped'], 0)
        self.assertFalse(recorder._thread.is_alive())

    def test_serialization_failure_is_reported_on_close(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = example.Recorder(Path(directory)/'run.jsonl', {})
            recorder.write(dict(kind='step', value=float('nan')))
            with self.assertRaisesRegex(OSError, 'WBC log writer failed'):
                recorder.close()
            self.assertEqual(recorder.statistics()['records_dropped'], 1)

    def test_close_failure_does_not_replace_primary_control_error(self):
        class FailedRecorder:
            def close(self):
                raise OSError('disk unavailable')
        with patch.object(example.sys, 'stderr'), self.assertRaisesRegex(RuntimeError, 'primary control'):
            try:
                raise RuntimeError('primary control')
            finally:
                example.finish_recording(FailedRecorder())

    def test_close_failure_without_primary_error_fails_the_run(self):
        class FailedRecorder:
            def close(self):
                raise OSError('disk unavailable')
        with patch.object(example.sys, 'stderr'), self.assertRaisesRegex(OSError, 'disk unavailable'):
            example.finish_recording(FailedRecorder())

    def test_invalid_capacity_fails_before_starting_writer(self):
        for capacity in (0, -1, True, 1.5):
            with self.subTest(capacity=capacity), self.assertRaises(ValueError):
                example.Recorder('unused', {}, capacity=capacity)


if __name__ == '__main__':
    unittest.main()
