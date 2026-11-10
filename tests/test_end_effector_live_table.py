"""Display callbacks must not parse blindly or render on a receiver thread."""
import importlib.util
import io
from pathlib import Path
import threading
import unittest


_PATH = Path(__file__).resolve().parents[1] / 'examples/end_effector/live_table.py'
_SPEC = importlib.util.spec_from_file_location('sdk_live_table_test', _PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


class Terminal(io.StringIO):
    def isatty(self):
        return True


class LiveTableTest(unittest.TestCase):
    def test_incomplete_body_and_leg_logs_show_unknown_fields(self):
        stream = io.StringIO()
        table = _MODULE.LiveTable(stream=stream, monotonic=lambda: 0.0)
        table('BODY stage=hold sequence=7')
        table('BODY stage=hold')
        table('EE leg=')
        table.refresh(force=True)
        self.assertIn('Body hold | sequence --', stream.getvalue())
        self.assertIn('phase --', stream.getvalue())

    def test_callback_cannot_render_but_owner_thread_still_can(self):
        stream = Terminal()
        table = _MODULE.LiveTable(stream=stream, monotonic=lambda: 0.0)
        failures = []

        def receive():
            try:
                table('BODY stage=hold')
                table('EE leg=low')
                table.refresh(force=True)
            except BaseException as error:
                failures.append(error)

        receiver = threading.Thread(target=receive)
        receiver.start()
        receiver.join(timeout=1.0)
        self.assertFalse(receiver.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(stream.getvalue(), '')
        table.refresh(force=True)
        self.assertIn('Body hold | sequence --', stream.getvalue())
        self.assertIn('phase low', stream.getvalue())


if __name__ == '__main__':
    unittest.main()
