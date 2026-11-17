"""Offline report over real native files; no transport or robot dependency."""
from array import array
import csv
import hashlib
import json
from pathlib import Path
import re
import tempfile
import unittest
from unittest.mock import patch

from locomotion_aorta.trace import Session, Kind, Outcome
from locomotion_aorta.trace.index import TraceIndex, uint_text
from locomotion_aorta.trace.report import PAGE_SIZE, _application_status, _commands, generate_report


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

    def test_application_status_uses_exact_key_and_clock_without_claiming_hardware_sample(self):
        for source, clock, expected in ((1, 0, 'APPLICATION_KEY_INTERVAL_ONLY'),
                                        (3, 0, 'NOT_AVAILABLE'),
                                        (1, 1, 'CLOCK_MAPPING_MISSING')):
            with self.subTest(source=source, clock=clock):
                trace = self.root / f'app-{source}-{clock}.ftrace'
                with Session(trace, stages=('body_publish', 'body_callback')) as session:
                    p = session.producer('control')
                    ids = (0,0,0,0,1,0,11,13,42,7,0,0)
                    p.emit(Kind.BODY_COMMAND, timestamp_us=100, stream=1, stage=1, ids=ids)
                    p.emit(Kind.BODY_COMMAND, timestamp_us=110, duration_us=11,
                           stream=1, stage=1, ids=ids, outcome=Outcome.OK)
                    p.emit(Kind.BODY_FEEDBACK, timestamp_us=150, clock=clock, stream=1, stage=2,
                           ids=(0,0,0,0,0,0,11,0,42,7,91,0), outcome=Outcome.OK,
                           integers=array('Q',[31,1,source]), times_us=array('Q',[0,150,110,120,125,0,0,0]))
                output = self.root / f'app-report-{source}-{clock}'
                generate_report(trace, output)
                row, = csv.DictReader((output/'commands.csv').read_text().splitlines())
                self.assertEqual(row['application_status_association'], expected)
                self.assertEqual(row['application_key_interval_us'], '51' if source == 1 and clock == 0 else '')
                self.assertEqual(row['hardware_latency_us'], '')

    def test_timeline_and_commands_traverse_ordered_results_once_across_pages(self):
        count = PAGE_SIZE * 2 + 7
        with Session(self.trace, stages=('body_publish', 'model')) as session:
            producer = session.producer('control')
            for i in range(count):
                ids = (0, 0, 0, 0, i + 1, 0, 11, 13, 42, i + 1, 0, 0)
                producer.emit(Kind.BODY_COMMAND, timestamp_us=i * 20000, stage=1, stream=1, ids=ids)
                producer.emit(Kind.BODY_COMMAND, timestamp_us=i * 20000 + 1, duration_us=2,
                              stage=1, stream=1, ids=ids, outcome=Outcome.OK)
                producer.emit(Kind.USER_SPAN, timestamp_us=i * 20000 + 2, duration_us=1,
                              stage=2, outcome=Outcome.OK)
        queries = []

        def index(*args, **kwargs):
            result = TraceIndex(*args, **kwargs)
            result.db.set_trace_callback(queries.append)
            return result

        output = self.root / 'pages'
        with patch('locomotion_aorta.trace.report.TraceIndex', side_effect=index):
            generate_report(self.trace, output)
        summary = json.loads((output / 'summary.json').read_text())
        self.assertTrue(summary['capture_complete'], summary['issues'])
        stages = list(csv.DictReader((output / 'stages.csv').read_text().splitlines()))
        expected = [int(row['event_id']) for row in stages]
        self.assertEqual(len(expected), count * 2)
        observed = []
        for page in range((len(expected) + PAGE_SIZE - 1) // PAGE_SIZE):
            rows = re.findall(r'<tr><td>(\d+)</td>',
                              (output / f'report/details/timeline-{page}.html').read_text())
            self.assertLessEqual(len(rows), PAGE_SIZE)
            observed.extend(map(int, rows))
        self.assertEqual(observed, expected)
        self.assertEqual(len(set(observed)), len(expected))
        commands = []
        for page in range((count + PAGE_SIZE - 1) // PAGE_SIZE):
            rows = re.findall(r'<article data-filter="[^"]*" id="c(\d+)"',
                              (output / f'report/details/commands-{page}.html').read_text())
            self.assertLessEqual(len(rows), PAGE_SIZE)
            commands.extend(map(int, rows))
        self.assertEqual(commands, list(range(1, count + 1)))
        self.assertEqual(len(list(csv.DictReader((output / 'commands.csv').read_text().splitlines()))), count)
        events = [json.loads(line) for line in (output / 'events.jsonl').read_text().splitlines()]
        self.assertEqual([row['event_id'] for row in events], list(range(1, len(events) + 1)))
        self.assertEqual(queries.count('SELECT * FROM timings ORDER BY event_id'), 2)  # CSV and HTML
        self.assertEqual(queries.count('SELECT * FROM commands ORDER BY id'), 1)
        self.assertFalse(any('OFFSET' in query for query in queries if 'SELECT * FROM' in query))

    def test_reference_and_application_key_queries_use_full_indexes_and_reject_ambiguity(self):
        for ambiguity in (None, 'reference', 'command'):
            with self.subTest(ambiguity=ambiguity):
                path = self.root / f'lookup-{ambiguity}.ftrace'
                with Session(path, stages=('body_publish', 'body_callback', 'body_read')) as session:
                    producer = session.producer('control')
                    commands = 2 if ambiguity == 'command' else 1
                    for i in range(commands):
                        ids = (0, 0, 0, 0, i + 1, 0, 11, 13 + i, 42, 7, 0, 0)
                        producer.emit(Kind.BODY_COMMAND, timestamp_us=100, stage=1, stream=1, ids=ids)
                        producer.emit(Kind.BODY_COMMAND, timestamp_us=110, duration_us=11,
                                      stage=1, stream=1, ids=ids, outcome=Outcome.OK)
                    for _ in range(2 if ambiguity == 'reference' else 1):
                        producer.emit(Kind.BODY_FEEDBACK, timestamp_us=150, stage=2, stream=1,
                                      ids=(0, 0, 0, 0, 303, 0, 11, 0, 42, 7, 91, 303),
                                      integers=array('Q', [31, 1, 1]), outcome=Outcome.OK)
                    producer.emit(Kind.BODY_READ, timestamp_us=160, stage=3, stream=1,
                                  ids=(0, 0, 0, 0, 304, 0, 11, 0, 42, 7, 91, 303), outcome=Outcome.OK)
                with TraceIndex(path) as index:
                    _commands(index)
                    reference_plan = str(list(index.db.execute(
                        'EXPLAIN QUERY PLAN SELECT row_json FROM events WHERE attempt=? AND kind=? LIMIT 2',
                        (uint_text(303), Kind.BODY_FEEDBACK))))
                    self.assertIn('SEARCH events', reference_plan)
                    self.assertIn('attempt=? AND kind=?', reference_plan)
                    key_plan = str(list(index.db.execute(
                        'EXPLAIN QUERY PLAN SELECT id,producer,attempt,begin_us,status FROM commands '
                        'WHERE runtime=? AND session=? AND sequence=? AND stream=1 LIMIT 2',
                        tuple(map(uint_text, (11, 42, 7))))))
                    self.assertIn('SEARCH commands', key_plan)
                    self.assertIn('runtime=? AND session=? AND sequence=? AND stream=?', key_plan)
                    _application_status(index)
                    rows = list(index.db.execute('SELECT status,latency FROM application_status'))
                    self.assertEqual(rows, [('APPLICATION_KEY_INTERVAL_ONLY', uint_text(51))]
                                     if ambiguity is None else [])


if __name__ == '__main__':
    unittest.main()
