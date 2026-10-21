"""Diagnostic joins must not fabricate session matches or motor latency."""
import unittest
from types import SimpleNamespace as NS

from locomotion_aorta._demo_feedback import NOT_MEASURED, publish_log_fields


class DiagnosticJoinTest(unittest.TestCase):
    def test_delta_requires_same_nonzero_session_and_ordered_sequences(self):
        for session, applied_session, applied, expected in (
            (37, 37, 11, 1), (37, 38, 11, NOT_MEASURED),
            (0, 0, 11, NOT_MEASURED), (37, 37, 13, NOT_MEASURED),
        ):
            with self.subTest(session=session, applied_session=applied_session, applied=applied):
                row = dict(applied_session_id=applied_session, applied_sequence=applied)
                publish_log_fields(row, NS(session_id=session, sequence=12))
                self.assertEqual(row['applied_sequence_lag'], expected)

    def test_absent_returned_command_never_creates_a_session_or_lag(self):
        row = dict(applied_session_id=37, applied_sequence=11)
        publish_log_fields(row, None)
        self.assertEqual(row['session_id'], NOT_MEASURED)
        self.assertEqual(row['sequence'], NOT_MEASURED)
        self.assertEqual(row['applied_sequence_lag'], NOT_MEASURED)
