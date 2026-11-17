"""Native IDs, asynchronous span lifetime and explicit result-consumption links."""
from array import array
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import tempfile
import unittest

from locomotion_aorta.trace import Trace, Kind, Outcome
from locomotion_aorta.trace.reader import Reader


class TraceLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'lifecycle.ftrace'

    def read(self):
        reader = Reader(self.path)
        return list(reader), reader.integrity

    def test_worker_inference_is_counted_once_with_multiple_explicit_consumers(self):
        with Trace(self.path, stages=('task', 'model', 'ready', 'consume')) as trace:
            task = trace.start_span('task')
            source = trace.new_id()
            trace.define('model', {'backend': 'synthetic-test', 'weights_sha256': None}, model=True)
            def worker():
                with trace.span('model') as model:
                    pass
                trace.mark('ready', kind=Kind.INFERENCE_LIFECYCLE,
                           inference_id=model.inference_id, reference_id=source)
                task.finish()
                return model.inference_id
            with ThreadPoolExecutor(max_workers=1) as pool:
                model = pool.submit(worker).result(timeout=2)
            for iteration in (1, 2):
                trace.mark('consume', kind=Kind.TRACE_LINK, iteration_id=iteration,
                           inference_id=model, reference_id=source,
                           integers=array('Q', [0, 10]), times_us=array('Q', [0]), validity=array('B', [0]))
        rows, integrity = self.read()
        self.assertTrue(integrity.complete, integrity.issues)
        actual = [r for r in rows if r['kind'] == Kind.USER_SPAN and r['inference_id'] == model]
        self.assertEqual(len(actual), 1)
        links = [r for r in rows if r['kind'] == Kind.TRACE_LINK]
        self.assertEqual([r['iteration_id'] for r in links], [1, 2])
        self.assertTrue(all(r['inference_id'] == model and r['reference_id'] == source for r in links))
        spans = [r for r in rows if r['kind'] == Kind.USER_SPAN and r['span_id'] == task.span_id]
        self.assertEqual(len(spans), 1)
        self.assertGreaterEqual(spans[0]['duration_us'], actual[0]['duration_us'])
        self.assertNotEqual(links[0]['producer'], actual[0]['producer'])

    def test_double_finish_and_unfinished_span_are_visible_integrity_failures(self):
        with Trace(self.path, stages=('task',)) as trace:
            task = trace.start_span('task')
            task.finish(failed=True)
            task.finish()
            trace.start_span('task')
        rows, integrity = self.read()
        self.assertFalse(integrity.complete)
        spans = [r for r in rows if r['kind'] == Kind.USER_SPAN]
        self.assertEqual(len(spans), 1)
        self.assertEqual(spans[0]['outcome'], Outcome.FAILED)
        self.assertGreaterEqual(integrity.run_end['producers'][0]['invalid_events'], 2)

    def test_invalid_optional_event_never_replaces_application_error(self):
        primary = RuntimeError('business failure')
        with self.assertRaises(RuntimeError) as caught:
            with Trace(self.path, stages=('mark',)) as trace:
                self.assertEqual(trace.mark('missing'), 0)
                self.assertEqual(trace.mark('mark', values=[1, 2]), 0)
                raise primary
        self.assertIs(caught.exception, primary)
        _, integrity = self.read()
        self.assertFalse(integrity.complete)

    def test_model_buffer_is_owned_chunked_and_keeps_full_numeric_values(self):
        import numpy as np
        values = np.arange(1520, dtype=np.float32)
        with Trace(self.path, stages=('observation',)) as trace:
            identity = trace.vector('observation', values, iteration_id=17, inference_id=19)
            values.fill(-1.)
        rows, integrity = self.read()
        self.assertTrue(integrity.complete, integrity.issues)
        chunks = sorted((r for r in rows if r['kind'] == Kind.USER_MARK), key=lambda r: r['integers'][0])
        self.assertEqual(len(chunks), 6)
        self.assertTrue(all(r['attempt_id'] == identity and r['inference_id'] == 19 and
                            r['iteration_id'] == 17 and r['integers'][1] == 1520 for r in chunks))
        self.assertEqual([v for r in chunks for v in r['values']], list(range(1520)))
        self.assertTrue(all(v == 1 for r in chunks for v in r['validity']))


if __name__ == '__main__':
    unittest.main()
