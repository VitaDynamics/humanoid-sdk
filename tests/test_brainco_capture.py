"""Host-only capture tests; injected Node never opens a network transport."""
import hashlib
import json
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import flatbuffers

from locomotion_aorta import brainco_capture as capture


def metadata():
    return dict(format_version=1, device="synthetic-only", boot_id="synthetic",
                source_commit="0" * 40, config_sha256="0" * 64,
                schema_sha256="0" * 64, expected_sides=[1, 2])


def touch_payload(side=1, length=5):
    # Real generated binding, not a handwritten decoder or mocked schema.
    from lowlevel.HandTouch import HandTouchT
    from lowlevel.FingerTouchMeta import FingerTouchMetaT
    touch = HandTouchT()
    touch.side = side
    touch.handType = touch.sensorType = 1
    touch.stateId = 1
    touch.tsStatePubUs = 1000
    touch.fingers = [FingerTouchMetaT(tsRequestUs=990+i, tsAcquiredHwUs=50,
                     tsIngestedUs=995, sampleSeq=1) for i in range(length)]
    touch.rawWords = [100, 200, 65535, 400, 500, 256] * 5
    builder = flatbuffers.Builder(512)
    builder.Finish(touch.Pack(builder))
    return bytes(builder.Output())


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


class Clock:
    def __init__(self):
        self.now = 0

    def __call__(self):
        self.now += 0.02
        return self.now


class BrainCoCaptureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            capture.load_bindings()
            import mcap  # noqa: F401 -- optional HIL extra
        except (ImportError, RuntimeError) as error:
            raise unittest.SkipTest("public EE bindings and hil extra required: " + str(error))
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name)

    def test_metadata_and_limits_reject_before_node_creation(self):
        for key, value in (("format_version", True), ("device", ""), ("source_commit", "x"),
                           ("expected_sides", [True]), ("schema_sha256", "absent")):
            item = metadata()
            item[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                capture.metadata_checked(item)
        for kwargs in (dict(duration_s=True), dict(duration_s=float("nan")),
                       dict(duration_s=0), dict(duration_s=1, capacity=257)):
            with patch.object(capture, "load_bindings") as load, self.assertRaises(ValueError):
                capture.capture_live(self.path / "out", metadata(), **kwargs)
            load.assert_not_called()

    def test_missing_bindings_fails_explicitly_before_opening_node_or_file(self):
        with patch.object(capture.importlib, "import_module", side_effect=ImportError("missing")):
            with self.assertRaisesRegex(RuntimeError, "no handwritten"):
                capture.capture_live(self.path / "out", metadata(), 1)
        self.assertFalse((self.path / "out").exists())

    def test_real_generated_touch_both_sides(self):
        bindings = capture.load_bindings()
        for side in (1, 2):
            row = capture.decode(("left" if side == 1 else "right") + "_hand_touch", touch_payload(side), bindings)
            self.assertEqual(row["side"], side)
            self.assertEqual(row["raw_words"], [100, 200, 65535, 400, 500, 256] * 5)
            self.assertEqual([f["ts_request_us"] for f in row["fingers"]], [990, 991, 992, 993, 994])
            self.assertEqual([f["direction_flags"] for f in row["fingers"]], [0] * 5)

    def test_bad_vectors_side_truncation_and_oversized_payload(self):
        bindings = capture.load_bindings()
        for data in (touch_payload(side=2), touch_payload(side=0), b"", touch_payload()[:32],
                     bytes(65537), b"\xff" * 16):
            with self.subTest(length=len(data)), self.assertRaises((ValueError, struct.error, TypeError)):
                capture.decode("left_hand_touch", data, bindings)

    def test_lowcmd_real_binding_preserves_all_fields(self):
        # Historical archive fixture: use the preserved generated42 encoder,
        # never call the current31 demo/transport to manufacture an old command.
        import lowlevel.LowCmd as module
        from locomotion_aorta.types import LowCmd, MotorCommand
        from test_motor_codec import reference_fill_low_cmd
        command = LowCmd(1,tuple(MotorCommand(mode=10,q=.1,kp=30.,kd=1.5) for _ in range(42)))
        builder = flatbuffers.Builder(4096)
        builder.Finish(reference_fill_low_cmd(SimpleNamespace(low_cmd_module=module),command,builder,0))
        row = capture.decode("external_command", bytes(builder.Output()), capture.load_bindings())
        self.assertEqual(row["cmd_id"], 1)
        self.assertEqual(len(row["motor_cmds"]), 42)
        for before, after in zip(command.motor_cmds, row["motor_cmds"]):
            for name in ("mode", "q", "dq", "tau", "kp", "kd"):
                self.assertAlmostEqual(after[name], getattr(before, name), places=6)

    def test_lowstate_real_binding_and_invalid_length_nonfinite(self):
        from test_motor_codec import build_state
        bindings = capture.load_bindings()
        payload = bytes(build_state([{"Q": 0.125, "Dq": 0.25, "TauEst": 0.5}] * 44))
        row = capture.decode("lowstate", payload, bindings)
        self.assertEqual(len(row["motor_states"]), 44)
        self.assertEqual(row["motor_states"][25]["q"], 0.125)
        self.assertEqual(row["stamp_ns"], 1789988776655443000)
        self.assertEqual(row["per_motor_freshness"], "not_available")
        for motors in ([{}] * 42, [{"Q": float("nan")}] * 44):
            with self.assertRaises(ValueError):
                capture.decode("lowstate", bytes(build_state(motors)), bindings)

    def test_control_status_real_generated_binding(self):
        import locomotion_sdk.ControlStatus as module
        builder = flatbuffers.Builder(256)
        action = builder.CreateString("PASSIVE")
        module.ControlStatusStart(builder)
        module.ControlStatusAddCurrentAction(builder, action)
        module.ControlStatusAddStatusSeq(builder, 17)
        module.ControlStatusAddIngressDropCount(builder, 2)
        builder.Finish(module.ControlStatusEnd(builder))
        row = capture.decode("control_status", bytes(builder.Output()), capture.load_bindings())
        self.assertEqual(row["current_action"], "PASSIVE")
        self.assertEqual(row["status_seq"], 17)
        self.assertEqual(row["ingress_drop_count"], 2)
        self.assertIsInstance(row["phase"], str)

    def fake_node(self, *, burst=1, invalid=False, fail=None):
        node = SimpleNamespace(closed=False, subscriptions=[], group=lambda: "default")
        def subscribe(message_type, topic, handler, qos):
            self.assertIs(message_type, bytes)
            node.subscriptions.append(topic)
            if topic == capture.TOPICS["left_hand_touch"]:
                for _ in range(burst):
                    handler(bytes(16) if invalid else touch_payload(), SimpleNamespace(sequence=1, publisher_node="fake"))
            if fail:
                raise fail
        def close():
            node.closed = True
        node.create_subscriber_typed, node.close = subscribe, close
        return node

    def test_readonly_live_drains_bounded_queue_and_marks_drop(self):
        node = self.fake_node(burst=3)
        output = self.path / "trace"
        result = capture.capture_live(output, metadata(), 0.04, capacity=1,
                                      node_factory=lambda *a: node, monotonic=Clock())
        self.assertEqual(result, {"left_hand_touch": 1})
        self.assertEqual(set(node.subscriptions), set(capture.TOPICS.values()))
        self.assertTrue(node.closed)
        rows = read_rows(output)
        self.assertEqual(rows[0]["capture_clock"], "observer_monotonic_ns")
        self.assertEqual(rows[-1]["dropped_messages"], 2)
        self.assertEqual(rows[-1]["transport_loss"], "not_measured")

    def test_pull_all_topics_have_no_python_rx_callbacks_and_keep_capture_scope(self):
        from test_body_codec import make_state
        payload, _ = make_state()
        node = self.fake_node()
        pulls = {}
        context = SimpleNamespace(sequence=77, publisher_node='native-ring')
        def subscribe(topic, *, depth, qos):
            self.assertEqual(depth, 1)
            samples = ([SimpleNamespace(payload=payload, context=context)]
                       if topic == capture.TOPICS['body_state'] else [])
            ring = SimpleNamespace(try_recv=lambda: samples.pop() if samples else None)
            pulls[topic] = ring
            return ring
        node.create_subscriber_pull = subscribe
        output = self.path / 'pull-trace'
        capture.capture_live(output, metadata(), .04, node_factory=lambda *a: node,
                             monotonic=Clock(), pull_feedback=True)
        self.assertTrue(node.closed)
        self.assertEqual(set(pulls), set(capture.TOPICS.values()))
        self.assertFalse(set(pulls).intersection(node.subscriptions))
        self.assertEqual(set(pulls) | set(node.subscriptions), set(capture.TOPICS.values()))
        rows = read_rows(output)
        self.assertEqual(rows[0]['feedback_retention'], 'keep-last-depth-1-all-topics')
        self.assertEqual(rows[0]['pull_overwrites'], 'not_measured')
        body = next(row for row in rows if row['type'] == 'body_state')
        expected = capture.decode('body_state', payload, capture.load_bindings())
        self.assertTrue(all(body[k] == v for k, v in expected.items()))
        self.assertEqual((body['transport_sequence'], body['publisher']), (77, 'native-ring'))
        self.assertTrue(rows[-1]['complete'])

    def test_keyboard_interrupt_and_decode_error_close_node_and_leave_incomplete_file(self):
        for name, kwargs, error in (("interrupt", dict(fail=KeyboardInterrupt()), KeyboardInterrupt),
                                    ("bad", dict(invalid=True), ValueError)):
            node = self.fake_node(**kwargs)
            output = self.path / name
            with self.assertRaises(error):
                capture.capture_live(output, metadata(), 0.04, node_factory=lambda *a: node, monotonic=Clock())
            self.assertTrue(node.closed)
            self.assertFalse(read_rows(output)[-1]["complete"])

    def test_existing_capture_never_overwritten_or_connected(self):
        output = self.path / "trace"
        output.write_text("original")
        with patch("aorta.Node") as node, self.assertRaises(FileExistsError):
            capture.capture_live(output, metadata(), 0.04)
        node.assert_not_called()
        self.assertEqual(output.read_text(), "original")

    def test_effective_group_mismatch_closes_without_subscribing(self):
        node = self.fake_node()
        output = self.path / "trace"
        with self.assertRaisesRegex(ValueError, "effective Aorta group"):
            capture.capture_live(output, metadata(), 1, group="other", node_factory=lambda *a: node)
        self.assertTrue(node.closed)
        self.assertEqual(node.subscriptions, [])
        self.assertFalse(read_rows(output)[-1]["complete"])

    def test_operator_marker_stays_in_observer_clock(self):
        node = self.fake_node()
        output = self.path / "trace"
        capture.capture_live(output, metadata(), 0.04, node_factory=lambda *a: node,
                             monotonic=Clock(), marker_reader=lambda: "right-index-pressed")
        marker = next(row for row in read_rows(output) if row["type"] == "marker")
        self.assertIn("capture_timestamp_ns", marker)
        self.assertNotIn("timestamp_us", marker)

    def mcap_file(self, *, root="lowlevel.HandTouch", encoding="flatbuffer", schema_data=None):
        from mcap.writer import Writer, CompressionType
        from hand_touch_schema_meta import SCHEMA_BFBS
        source = self.path / "input.mcap"
        with source.open("wb") as stream:
            writer = Writer(stream, compression=CompressionType.NONE, enable_data_crcs=True)
            writer.start()
            schema = writer.register_schema(root, encoding, SCHEMA_BFBS if schema_data is None else schema_data)
            channel = writer.register_channel("aorta/default/pub/sdk/foot_humanoid/hand/left/touch", "flatbuffer", schema)
            for side in (1, 2):
                writer.add_message(channel, side * 1000, touch_payload(1), side * 1000, side)
            writer.finish()
        return source

    def export(self, source, **kwargs):
        meta = metadata()
        meta["schema_sha256"] = capture.binding_schema_hash('left_hand_touch')
        return capture.export_mcap(source, self.path / "export", meta,
            {"aorta/default/pub/sdk/foot_humanoid/hand/left/touch": "left_hand_touch"},
            {"left_hand_touch": meta["schema_sha256"]}, **kwargs)

    def test_mcap_export_uses_exact_topic_hash_and_retains_source_hash(self):
        source = self.mcap_file()
        self.assertEqual(self.export(source), {"left_hand_touch": 2})
        rows = read_rows(self.path / "export")
        self.assertEqual(rows[0]["source_mcap_sha256"], hashlib.sha256(source.read_bytes()).hexdigest())
        self.assertEqual([r["side"] for r in rows[1:-1]], [1, 1])
        self.assertTrue(rows[-1]["complete"])

    def test_mcap_truncated_final_magic_is_not_complete(self):
        source = self.mcap_file()
        source.write_bytes(source.read_bytes()[:-4])
        with self.assertRaises(Exception):
            self.export(source)
        self.assertFalse(read_rows(self.path / "export")[-1]["complete"])

    def test_mcap_corrupt_crc_is_not_complete(self):
        source = self.mcap_file()
        data = bytearray(source.read_bytes())
        start = data.find(touch_payload(1))
        self.assertGreater(start, 0)
        data[start + len(touch_payload(1)) - 1] ^= 1
        source.write_bytes(data)
        with self.assertRaises(Exception):
            self.export(source)
        self.assertFalse(read_rows(self.path / "export")[-1]["complete"])

    def test_mcap_wrong_root_encoding_hash_and_missing_topic_fail(self):
        for kwargs in (dict(root="wrong.Root"), dict(encoding="cdr")):
            source = self.mcap_file(**kwargs)
            with self.assertRaisesRegex(ValueError, "schema/encoding"):
                self.export(source)
            (self.path / "export").unlink()
        source = self.mcap_file()
        meta = metadata(); meta['schema_sha256'] = capture.binding_schema_hash('left_hand_touch')
        with self.assertRaises(ValueError):
            capture.export_mcap(source, self.path / 'export', meta,
                                {'wrong/topic': 'left_hand_touch'},
                                {'left_hand_touch': meta['schema_sha256']})
        self.assertFalse(read_rows(self.path / 'export')[-1]['complete'])
        (self.path / 'export').unlink()
        stale = b'synthetic-other-layout'
        source = self.mcap_file(schema_data=stale)
        meta['schema_sha256'] = hashlib.sha256(stale).hexdigest()
        with patch.object(capture, 'decode') as decode, self.assertRaisesRegex(ValueError, 'loaded binding'):
            capture.export_mcap(source, self.path / 'export', meta,
                {'aorta/default/pub/sdk/foot_humanoid/hand/left/touch': 'left_hand_touch'},
                {'left_hand_touch': meta['schema_sha256']})
        decode.assert_not_called()
        self.assertFalse((self.path / 'export').exists())

    def test_new_hand_state_capture_preserves_missing_feedback_as_null(self):
        from test_hand import state_obj, pack
        obj = state_obj(); obj.joints[0].q = float('nan'); obj.inhibitReason = 4
        row = capture.decode('left_hand_state', pack(obj), capture.load_bindings())
        self.assertIsNone(row['joints'][0]['q'])
        self.assertTrue(row['nonfinite_feedback'])
        self.assertEqual(row['inhibit_reason'], 4)
        self.assertEqual(row['ts_acquired_hw_us'], 17)
        self.assertNotIn('ts_acquired_hw_us', row['joints'][0])


class PublicBodyCaptureTest(unittest.TestCase):
    def test_current_body_state_keeps_slot_order_errors_and_nonfinite_null(self):
        from test_body_codec import make_state
        data, _ = make_state()
        row = capture.decode('body_state',data,capture.load_bindings())
        self.assertEqual(len(row['joints']),31)
        self.assertNotIn('status',row['joints'][14])
        self.assertNotIn('body_index',row['joints'][14])
        self.assertIsNone(row['joints'][14]['q'])
        self.assertEqual(row['joints'][14]['error_code'],0)
        self.assertNotIn('valid_fields',row['joints'][14])
        self.assertNotIn('populated',row['joints'][14])
        self.assertTrue(row['nonfinite_feedback'])
        self.assertEqual(row['ts_state_real_us'],1_234_567)

    def test_current_command_keeps_action_and_body_causality_without_mode(self):
        from test_body_codec import make_command, encode
        from locomotion_aorta import BodyCmdAction
        row=capture.decode('body_cmd',encode(make_command()),capture.load_bindings())
        self.assertNotIn('cmd_id', row)
        self.assertEqual((row['session_id'], row['sequence']), (42, 7))
        self.assertEqual(row['action'],BodyCmdAction.EXTERNAL)
        self.assertEqual(len(row['joints']),31)
        self.assertEqual(row['input_state_id'],123)
        self.assertNotIn('mode',row['joints'][0])
        self.assertNotIn('lowstate',capture.TOPICS)
        self.assertNotIn('external_command',capture.TOPICS)
        self.assertIn('lowstate',capture.MODULES)  # explicit archive parsing retained
