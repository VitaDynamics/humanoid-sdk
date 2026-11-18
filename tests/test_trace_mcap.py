"""Real MCAP container and installed generated BFBS; synthetic messages labelled as such."""
import csv
from dataclasses import replace
import importlib
import json
from pathlib import Path
import tempfile
import unittest

import flatbuffers
from mcap.writer import Writer, CompressionType
from locomotion_aorta import BodyCmdAction, BodyJointCmd
from locomotion_aorta.trace import Session, Kind, Outcome
from locomotion_aorta.trace.report import generate_report
from test_body_codec import make_command, make_state, encode, module


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

    def mcap(self, *, bad_schema=False, bad_runtime=False, truncated=False,
             payload=None, stem='body_cmd'):
        path = self.root / 'synthetic.mcap'
        meta = importlib.import_module(stem + '_schema_meta')
        with path.open('wb') as stream:
            writer = Writer(stream, compression=CompressionType.ZSTD, enable_crcs=True)
            writer.start()
            schema = writer.register_schema(meta.ROOT_TYPE_NAME, 'flatbuffer',
                meta.SCHEMA_BFBS + (b'bad' if bad_schema else b''))
            channel = writer.register_channel('/sdk/foot_humanoid/' + stem, 'flatbuffer', schema)
            command = make_command()
            if bad_runtime:
                command = replace(command, runtime_id=12)
            writer.add_message(channel, 9_000_123,
                               encode(command) if payload is None else payload, 8_000_456)
            writer.finish()
        if truncated:
            path.write_bytes(path.read_bytes()[:-40])
        return path

    def analyze(self, *, report_name='report', **kwargs):
        output = self.root / report_name
        generate_report(self.trace, output, mcap_paths=[self.mcap(**kwargs)])
        rows = list(csv.DictReader((output / 'mcap.csv').read_text().splitlines()))
        return rows, json.loads((output / 'mcap-integrity.json').read_text())

    def test_exact_key_is_only_a_sighting_and_mcap_time_converts_to_us(self):
        rows, integrity = self.analyze()
        self.assertEqual(rows[0]['association'], 'COMMAND_KEY_OBSERVED_IN_MCAP')
        self.assertEqual(rows[0]['command_row'], '1')
        self.assertEqual((rows[0]['log_time_us'], rows[0]['publish_time_us']), ('9000', '8000'))
        self.assertTrue(integrity['sources'][0]['complete'])

    def test_body_cmd_versions_preserve_raw_coordinates_and_key_sighting(self):
        ankle_values = {
            4: (0., -.5, 1.25, 8., .25),
            5: (-.125, .75, -1.5, 4., .5),
            10: (.25, -.125, 0., 2., .75),
            11: (-.5, 0., .5, 1., 1.),
        }
        original = make_command()
        joints = list(original.joints)
        for axis, values in ankle_values.items():
            joints[axis] = BodyJointCmd(*values)
        for version in (3, 4):
            with self.subTest(version=version):
                command = replace(original, joints=tuple(joints), protocol_version=version)
                payload = encode(command)
                view = module('BodyCmd').BodyCmd.GetRootAs(payload)
                self.assertEqual(view.ProtocolVersion(), version)
                self.assertEqual(view.Action(), BodyCmdAction.EXTERNAL)
                name = f'body-cmd-v{version}'
                rows, integrity = self.analyze(report_name=name, payload=payload)
                self.assertEqual(rows[0]['decode_status'], 'DECODED')
                self.assertEqual(rows[0]['association'], 'COMMAND_KEY_OBSERVED_IN_MCAP')
                self.assertEqual(rows[0]['command_row'], '1')
                self.assertTrue(integrity['sources'][0]['complete'])
                event = json.loads((self.root / name / 'mcap-events.jsonl').read_text())
                data = event['payload']
                self.assertEqual(data['protocol_version'], version)
                self.assertEqual(data['action'], BodyCmdAction.EXTERNAL)
                self.assertEqual((data['runtime_id'], data['control_id'], data['session_id'],
                                  data['command_sequence']), (11, 13, 42, 7))
                self.assertEqual(len(data['joints']), 31)
                for actual, expected in zip(data['joints'], command.joints):
                    self.assertEqual(tuple(actual[key] for key in ('q', 'dq', 'tau', 'kp', 'kd')),
                                     (expected.q, expected.dq, expected.tau, expected.kp, expected.kd))

    def test_unknown_body_cmd_version_and_non_external_v4_are_invalid(self):
        original = make_command()
        cases = [(f'version-{version}', replace(original, protocol_version=version),
                  'protocol version mismatch') for version in (0, 2, 5, 0xffffffff)]
        cases += [(f'action-{action}', replace(original, protocol_version=4, action=action),
                   'requires EXTERNAL')
                  for action in (BodyCmdAction.HUMANOID_TRACKING, BodyCmdAction.UNKNOWN)]
        for name, command, reason in cases:
            with self.subTest(name=name):
                # Official generated builders encode untrusted input rejected by the SDK sender.
                payload = encode(command, generated=True)
                view = module('BodyCmd').BodyCmd.GetRootAs(payload)
                self.assertEqual(view.ProtocolVersion(), command.protocol_version)
                self.assertEqual(view.Action(), command.action)
                rows, integrity = self.analyze(report_name=name, payload=payload)
                self.assertEqual(rows[0]['decode_status'], 'INVALID')
                self.assertIn(reason, rows[0]['reason'])
                self.assertEqual(rows[0]['association'], 'NOT_AVAILABLE')
                self.assertEqual(rows[0]['command_row'], '')
                self.assertTrue(integrity['sources'][0]['complete'])
                event = json.loads((self.root / name / 'mcap-events.jsonl').read_text())
                self.assertIsNone(event['payload'])

    def test_body_state_remains_version3_only(self):
        state, _ = make_state()
        rows, _ = self.analyze(report_name='body-state-v3', payload=state, stem='body_state')
        self.assertEqual(rows[0]['decode_status'], 'DECODED')
        self.assertEqual(rows[0]['command_row'], '')
        builder = flatbuffers.Builder(1)
        wire = module('BodyState')
        wire.BodyStateStart(builder)
        wire.BodyStateAddProtocolVersion(builder, 4)
        builder.Finish(wire.BodyStateEnd(builder))
        rows, _ = self.analyze(report_name='body-state-v4', payload=bytes(builder.Output()),
                               stem='body_state')
        self.assertEqual(rows[0]['decode_status'], 'INVALID')
        self.assertIn('protocol version mismatch', rows[0]['reason'])
        self.assertEqual(rows[0]['command_row'], '')

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
