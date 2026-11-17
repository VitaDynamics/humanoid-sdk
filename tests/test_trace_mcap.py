"""Real MCAP container and installed generated BFBS; synthetic messages labelled as such."""
import csv
import importlib
import json
from pathlib import Path
import tempfile
import unittest

from mcap.writer import Writer, CompressionType
from locomotion_aorta.trace import Session, Kind, Outcome
from locomotion_aorta.trace.report import generate_report
from test_body_codec import make_command, encode


class TraceMcapTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.trace = self.root / 'synthetic.ftrace'
        with Session(self.trace, stages=('body_publish',)) as session:
            p = session.producer('control')
            ids = (0, 0, 0, 0, 1, 0, 11, 13, 42, 7, 0, 0)
            p.emit(Kind.BODY_COMMAND, timestamp_us=100, ids=ids, stage=1, stream=1)
            p.emit(Kind.BODY_COMMAND, timestamp_us=110, duration_us=11, ids=ids,
                   stage=1, stream=1, outcome=Outcome.OK)

    def mcap(self, *, bad_schema=False, bad_runtime=False, truncated=False):
        path = self.root / 'synthetic.mcap'
        meta = importlib.import_module('body_cmd_schema_meta')
        with path.open('wb') as stream:
            writer = Writer(stream, compression=CompressionType.ZSTD, enable_crcs=True)
            writer.start()
            schema = writer.register_schema(meta.ROOT_TYPE_NAME, 'flatbuffer',
                meta.SCHEMA_BFBS + (b'bad' if bad_schema else b''))
            channel = writer.register_channel('/sdk/foot_humanoid/body_cmd', 'flatbuffer', schema)
            command = make_command()
            if bad_runtime:
                from dataclasses import replace
                command = replace(command, runtime_id=12)
            writer.add_message(channel, 9_000_123, encode(command), 8_000_456)
            writer.finish()
        if truncated:
            path.write_bytes(path.read_bytes()[:-40])
        return path

    def analyze(self, **kwargs):
        output = self.root / 'report'
        generate_report(self.trace, output, mcap_paths=[self.mcap(**kwargs)])
        rows = list(csv.DictReader((output / 'mcap.csv').read_text().splitlines()))
        return rows, json.loads((output / 'mcap-integrity.json').read_text())

    def test_exact_key_is_only_a_sighting_and_mcap_time_converts_to_us(self):
        rows, integrity = self.analyze()
        self.assertEqual(rows[0]['association'], 'COMMAND_KEY_OBSERVED_IN_MCAP')
        self.assertEqual(rows[0]['command_row'], '1')
        self.assertEqual((rows[0]['log_time_us'], rows[0]['publish_time_us']), ('9000', '8000'))
        self.assertTrue(integrity['sources'][0]['complete'])

    def test_mismatched_bfbs_is_not_decoded(self):
        rows, integrity = self.analyze(bad_schema=True)
        self.assertEqual(rows[0]['decode_status'], 'NOT_AVAILABLE')
        self.assertEqual(rows[0]['command_row'], '')
        self.assertIn('mismatch', rows[0]['reason'])

    def test_runtime_mismatch_cannot_associate_and_missing_footer_is_incomplete(self):
        rows, integrity = self.analyze(bad_runtime=True, truncated=True)
        self.assertEqual(rows[0]['association'], 'UNMATCHED')
        self.assertFalse(integrity['sources'][0]['complete'])
        self.assertIn('missing_footer', integrity['sources'][0]['issues'])


if __name__ == '__main__':
    unittest.main()
