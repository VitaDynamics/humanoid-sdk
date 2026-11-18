"""Offline timing semantics over the real FBS writer, without robot I/O."""
from pathlib import Path
import tempfile
import unittest

from locomotion_aorta.trace import Kind, Outcome, Session
from locomotion_aorta.trace.summary import summarize_spans


class TraceSummaryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'timings.ftrace'

    def test_sdk_calls_exclude_entries_and_feedback_and_keep_sides_and_outcomes(self):
        with Session(self.path, stages=('endpoint_publish', 'endpoint_encode', 'actor'),
                     streams=({'id': 2, 'name': 'left'}, {'id': 3, 'name': 'right'})) as session:
            producer = session.producer('control')
            for duration in (0, 10, 100):
                producer.emit(Kind.EFFECTOR_COMMAND, timestamp_us=1, stage=1,
                              stream=2, outcome=Outcome.UNKNOWN)
                producer.emit(Kind.STAGE_EVENT, timestamp_us=1, duration_us=2,
                              stage=2, stream=2, outcome=Outcome.OK)
                producer.emit(Kind.EFFECTOR_COMMAND, timestamp_us=101,
                              duration_us=duration, stage=1, stream=2, outcome=Outcome.OK)
            producer.emit(Kind.EFFECTOR_COMMAND, timestamp_us=200, duration_us=7,
                          stage=1, stream=3, outcome=Outcome.FAILED)
            # A feedback record is not a publish completion, even with the same
            # stage ID. No generic all-duration aggregation may count it.
            producer.emit(Kind.EFFECTOR_FEEDBACK, timestamp_us=201, duration_us=999,
                          stage=1, stream=2, outcome=Outcome.OK)
            producer.emit(Kind.USER_SPAN, timestamp_us=1, duration_us=30,
                          stage=3, outcome=Outcome.OK)
        result = summarize_spans(self.path)
        self.assertTrue(result['capture_complete'])
        self.assertEqual([(x['stage'], x['observed_spans']) for x in result['stages']], [('actor', 1)])
        groups = {(x['stage'], x['stream'], x['outcome']): x for x in result['sdk_stages']}
        left = groups['endpoint_publish', 2, 'OK']
        self.assertEqual((left['stream_name'], left['observed_calls'], left['duration_samples']), ('left', 3, 3))
        self.assertEqual((left['min_us'], left['p50_us'], left['p95_us'], left['max_us']), (0, 10, 100, 100))
        self.assertEqual(groups['endpoint_encode', 2, 'OK']['p99_us'], 2)
        self.assertEqual(groups['endpoint_publish', 3, 'FAILED']['p99_us'], 7)

    def test_unknown_duration_and_unassigned_decode_are_not_zero_or_nearest_side(self):
        with Session(self.path, stages=('endpoint_decode',),
                     streams=({'id': 2, 'name': 'left'},)) as session:
            producer = session.producer('control')
            producer.emit(Kind.STAGE_EVENT, timestamp_us=10, stage=1, outcome=Outcome.UNKNOWN)
            producer.emit(Kind.EFFECTOR_FEEDBACK, timestamp_us=11, stage=1, stream=2)
            producer.emit(Kind.STAGE_EVENT, timestamp_us=12, stage=1, outcome=Outcome.FAILED)
            producer.note_invalid()
        result = summarize_spans(self.path)
        self.assertFalse(result['capture_complete'])
        self.assertEqual(result['stages'], [])
        call, = result['sdk_stages']
        self.assertEqual((call['stage'], call['stream'], call['stream_name']), ('endpoint_decode', 0, None))
        self.assertEqual((call['observed_calls'], call['duration_samples'], call['invalid_duration_calls']), (1, 0, 1))
        self.assertIsNone(call['p99_us'])
        self.assertGreater(result['producer_issues'][0]['invalid_events'], 0)


if __name__ == '__main__':
    unittest.main()
