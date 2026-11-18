"""Trace host tests require the real built extension; no fake writer or device."""
from array import array
from pathlib import Path
import struct
import tempfile
import threading
import unittest

from locomotion_aorta.trace import Kind, Outcome, Session, disabled
from locomotion_aorta.trace.reader import Reader


class TraceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'capture.ftrace'

    def capture(self, *, compression='raw'):
        session = Session(self.path, stages=('infer',), compression=compression)
        self.addCleanup(session.close)
        return session

    def test_raw_and_zstd_roundtrip_preserve_zero_and_extended_microseconds(self):
        for codec in ('raw', 'zstd'):
            with self.subTest(codec=codec):
                self.path = Path(self.temp.name) / (codec + '.ftrace')
                with self.capture(compression=codec) as session:
                    producer = session.producer('control')
                    for stamp in (0, 2**32 - 1, 2**32, 2**64 - 1):
                        self.assertTrue(producer.emit(Kind.BODY_COMMAND, timestamp_us=stamp,
                            duration_us=stamp, outcome=Outcome.OK, stage=1,
                            ids=(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12),
                            values=array('d', (0., .25)), integers=array('Q', (0, 91)),
                            times_us=array('Q', (0, 2**64 - 1)), validity=array('B', (1, 0))))
                reader = Reader(self.path)
                rows = list(reader)
                self.assertTrue(reader.integrity.complete, reader.integrity.issues)
                self.assertEqual([r['timestamp_us'] for r in rows], [0, 2**32 - 1, 2**32, 2**64 - 1])
                self.assertEqual([r['duration_us'] for r in rows], [0, 2**32 - 1, 2**32, 2**64 - 1])
                self.assertEqual(rows[0]['runtime_id'], 7)
                self.assertEqual(rows[0]['state_id'], 11)
                self.assertEqual(rows[0]['times_us'], [0, 2**64 - 1])
                self.assertEqual(rows[0]['validity'], [1, 0])

    def test_missing_duration_is_distinct_from_valid_zero(self):
        with self.capture() as session:
            p = session.producer('control')
            self.assertTrue(p.emit(Kind.USER_MARK, timestamp_us=0))
            self.assertTrue(p.emit(Kind.USER_MARK, timestamp_us=0, duration_us=0))
        self.assertEqual([r['duration_us'] for r in Reader(self.path)], [None, 0])

    def test_existing_file_is_not_overwritten(self):
        self.path.write_bytes(b'original')
        with self.assertRaises(Exception):
            Session(self.path)
        self.assertEqual(self.path.read_bytes(), b'original')

    def test_invalid_buffer_is_counted_and_does_not_raise_into_control(self):
        with self.capture() as session:
            p = session.producer('control')
            self.assertFalse(p.emit(Kind.USER_MARK, values=array('d', [0.] * 257)))
            self.assertTrue(p.emit(Kind.USER_MARK))
        counts = session.health()['producers'][0]
        self.assertEqual(counts['attempted_events'], 2)
        self.assertEqual(counts['dropped_events'], 1)
        self.assertEqual(counts['invalid_events'], 1)
        reader = Reader(self.path)
        self.assertEqual(len(list(reader)), 1)
        self.assertFalse(reader.integrity.complete)

    def test_another_thread_cannot_write_the_same_spsc_ring(self):
        with self.capture() as session:
            p = session.producer('control')
            self.assertTrue(p.emit(Kind.USER_MARK))
            results = []
            worker = threading.Thread(target=lambda: results.append(p.emit(Kind.USER_MARK)))
            worker.start()
            worker.join(2.)
            self.assertFalse(worker.is_alive())
            self.assertEqual(results, [False])
        counts = session.health()['producers'][0]
        self.assertEqual(counts['wrong_thread_events'], 1)
        self.assertEqual(counts['attempted_events'], counts['enqueued_events'] + counts['dropped_events'])

    def test_corrupt_block_yields_no_records_from_that_block(self):
        with self.capture() as session:
            session.producer('control').emit(Kind.USER_MARK)
        raw = bytearray(self.path.read_bytes())
        first_size = struct.unpack_from('<I', raw, 8)[0]
        raw[8 + 32 + first_size + 32] ^= 1
        self.path.write_bytes(raw)
        reader = Reader(self.path)
        self.assertEqual(list(reader), [])
        self.assertFalse(reader.integrity.complete)
        self.assertTrue(any('checksum' in s for s in reader.integrity.issues))

    def test_truncated_footer_reports_incomplete_and_keeps_verified_events(self):
        with self.capture() as session:
            session.producer('control').emit(Kind.USER_MARK)
        self.path.write_bytes(self.path.read_bytes()[:-5])
        reader = Reader(self.path)
        self.assertEqual(len(list(reader)), 1)
        self.assertFalse(reader.integrity.complete)
        self.assertIn('missing_run_end', reader.integrity.issues)

    def test_span_does_not_replace_a_user_exception(self):
        primary = RuntimeError('model fault')
        with self.capture() as session:
            with self.assertRaises(RuntimeError) as caught:
                with session.producer('control').span(1, iteration_id=44):
                    raise primary
            self.assertIs(caught.exception, primary)
        row, = Reader(self.path)
        self.assertEqual(row['outcome'], Outcome.FAILED)
        self.assertEqual(row['iteration_id'], 44)

    def test_disabled_capture_creates_no_resource(self):
        self.assertFalse(disabled.emit(Kind.USER_MARK))
        with disabled.span('not-registered'):
            pass
        self.assertFalse(self.path.exists())

    def test_queue_budget_is_checked_before_creating_file(self):
        with self.assertRaises(ValueError):
            Session(self.path, memory_budget_bytes=1)
        self.assertFalse(self.path.exists())


if __name__ == '__main__':
    unittest.main()
