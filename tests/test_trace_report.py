"""Offline report over real native files; no transport or robot dependency."""
from array import array
import csv
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from locomotion_aorta.trace import Session, Kind, Outcome
from locomotion_aorta.trace.report import generate_report


class TraceReportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.trace = self.root / 'test.ftrace'

    def test_report_preserves_attempts_masks_identity_pagination_and_input(self):
        huge = 2**64 - 2
        with Session(self.trace, stages=('endpoint_publish', '<script>bad()</script>'),
                     streams=({'id': 2, 'name': 'left'},)) as session:
            p = session.producer('control')
            raw = json.dumps(dict(type='endpoint_info', value=dict(runtime_id=huge, side=1,
                axis_names=['opening'], position_domain=[2], is_mock=[False]))).encode()
            for offset in range(0, len(raw), 216):
                chunk = raw[offset:offset+216]
                chunk += b'\0' * (-len(chunk) % 8)
                words = [int.from_bytes(chunk[i:i+8], 'little') for i in range(0, len(chunk), 8)]
                p.emit(Kind.STREAM_INFO, ids=(0,0,0,0,10000,0,0,0,0,0,0,0),
                       integers=array('Q', [offset, len(raw), *words]))
            for i in range(129):
                ids = (0, 0, 0, 0, i+1, 0, huge, huge-1, huge-2, i+1, 0, 0)
                p.emit(Kind.EFFECTOR_COMMAND, timestamp_us=i*20, ids=ids, stage=1, stream=2)
                p.emit(Kind.EFFECTOR_COMMAND, timestamp_us=i*20+5, duration_us=6,
                       ids=ids, stage=1, stream=2, outcome=Outcome.OK)
            p.emit(Kind.EFFECTOR_FEEDBACK, timestamp_us=1, stream=2, integers=array('Q', [2]),
                   ids=(0,0,0,0,0,0,huge,0,0,0,0,0),
                   values=array('d', [0., 999.]), validity=array('B', [1, 0]))
            p.emit(Kind.USER_SPAN, timestamp_us=1, duration_us=0, stage=2, outcome=Outcome.OK)
        before = hashlib.sha256(self.trace.read_bytes()).hexdigest()
        output = self.root / 'report'
        page = generate_report(self.trace, output)
        self.assertEqual(hashlib.sha256(self.trace.read_bytes()).hexdigest(), before)
        self.assertIn('&lt;script&gt;bad()', page.read_text())
        self.assertNotIn('<script>bad()', page.read_text())
        self.assertTrue((output / 'report/details/commands-1.html').exists())
        rows = list(csv.DictReader((output / 'commands.csv').read_text().splitlines()))
        self.assertEqual(len(rows), 129)
        self.assertEqual(rows[0]['runtime_id'], str(huge))
        self.assertEqual(rows[0]['sdk_duration_us'], '6')
        self.assertTrue(all(r['hardware_latency_us'] == '' for r in rows))
        feedback = (output / 'report/feedback.html').read_text()
        self.assertIn('axis 0', feedback)
        self.assertIn('opening', feedback)
        self.assertIn('单位：m', feedback)
        self.assertNotIn('axis 1', feedback)
        detail = (output / 'report/details/commands-0.html').read_text()
        self.assertIn('&quot;runtime_id&quot;: &quot;' + str(huge) + '&quot;', detail)
        with self.assertRaises(FileExistsError):
            generate_report(self.trace, output)

    def test_pre_key_failure_and_incomplete_recording_do_not_disappear(self):
        with Session(self.trace, stages=('body_publish',)) as session:
            p = session.producer('control')
            ids = (0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0)
            p.emit(Kind.BODY_COMMAND, timestamp_us=10, ids=ids, stage=1, stream=1)
            p.emit(Kind.BODY_COMMAND, timestamp_us=10, duration_us=1, ids=ids,
                   stage=1, stream=1, outcome=Outcome.FAILED)
            p.note_invalid()
        output = self.root / 'report'
        generate_report(self.trace, output)
        summary = json.loads((output / 'summary.json').read_text())
        self.assertFalse(summary['capture_complete'])
        rows = list(csv.DictReader((output / 'commands.csv').read_text().splitlines()))
        self.assertEqual((rows[0]['outcome'], rows[0]['runtime_id']), ('FAILED', '0'))
        self.assertEqual(rows[0]['hardware_association'], 'NOT_AVAILABLE')


if __name__ == '__main__':
    unittest.main()
