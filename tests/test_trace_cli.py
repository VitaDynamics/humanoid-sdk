"""CLI defaults and real FBS output, without robot connections."""
import argparse
from contextlib import redirect_stderr
from datetime import datetime, timezone
import io
from pathlib import Path
import unittest
from unittest.mock import patch

from locomotion_aorta.trace import Trace
from locomotion_aorta.trace.cli import add_trace_arguments, resolve_trace_output
from locomotion_aorta.trace.reader import Reader


class TraceCliTests(unittest.TestCase):
    def parser(self):
        parser = argparse.ArgumentParser()
        add_trace_arguments(parser)
        return parser

    def test_default_and_explicit_off_are_false_without_clock_or_path(self):
        for flags in ([], ['--trace', 'off']):
            parser = self.parser()
            args = parser.parse_args(flags)
            with patch('locomotion_aorta.trace.cli.datetime') as clock, \
                 patch('locomotion_aorta.trace.cli.uuid.uuid4') as identity:
                self.assertIs(args.trace, False)
                self.assertIsNone(resolve_trace_output(parser, args))
                self.assertIsNone(args.trace_output)
                clock.now.assert_not_called()
                identity.assert_not_called()

    def test_on_default_has_utc_timestamp_and_unique_tmp_name(self):
        parser = self.parser()
        with patch('locomotion_aorta.trace.cli.datetime') as clock:
            clock.now.return_value = datetime(2026, 11, 17, 12, 30, 15, 123456, tzinfo=timezone.utc)
            paths = [resolve_trace_output(parser, parser.parse_args(['--trace', 'on'])) for _ in range(2)]
        self.assertNotEqual(*paths)
        for path in paths:
            self.assertEqual(path.parent, Path('/tmp'))
            self.assertRegex(path.name, r'^sdk-trace-20261117T123015123456Z-\d+-[0-9a-f]{8}\.ftrace$')
            self.assertFalse(path.exists())

    def test_default_path_is_usable_by_the_real_native_recorder(self):
        parser = self.parser()
        args = parser.parse_args(['--trace=on'])
        path = resolve_trace_output(parser, args)
        with Trace(path, stages=('cli',)) as trace:
            self.addCleanup(path.unlink, missing_ok=True)
            with trace.span('cli'):
                pass
        reader = Reader(path)
        self.assertTrue(list(reader))
        self.assertTrue(reader.integrity.complete, reader.integrity.issues)

    def test_explicit_path_is_preserved_and_invalid_options_are_rejected(self):
        parser = self.parser()
        args = parser.parse_args(['--trace', 'on', '--trace-output', 'chosen.ftrace'])
        self.assertEqual(resolve_trace_output(parser, args), Path('chosen.ftrace'))
        for flags in (['--trace', 'fbs'], ['--trace', 'false'],
                      ['--trace', 'off', '--trace-output', 'chosen.ftrace'],
                      ['--trace', 'on', '--trace-output', 'chosen.txt']):
            with self.subTest(flags=flags), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as stopped:
                resolve_trace_output(parser, parser.parse_args(flags))
            self.assertEqual(stopped.exception.code, 2)
