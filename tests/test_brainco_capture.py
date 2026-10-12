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
    from aorta.topic.lowlevel.HandTouch import HandTouchT
    touch = HandTouchT()
    touch.handSide = side
    touch.tactileType = touch.timestampDomain = 1
    touch.sampleSequence = 1
    touch.receivedTimestampUs = 1000
    touch.sampledTimestampUs = list(range(990, 990 + length))
    touch.hardwareSampledTimestampUs = [50] * 5
    touch.fingerValid = [True] * 5
    touch.fingerSequence = [1] * 5
    touch.fingerStatusFlags = [0] * 5
    touch.fingerDirectionValid = [False] * 5
    touch.rawRegisters = [100, 200, 65535, 400, 500, 256] * 5
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
            raise unittest.SkipTest("HIL candidate bindings and hil extra required: " + str(error))
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
            row = capture.decode("hand_touch", touch_payload(side), bindings)
            self.assertEqual(row["hand_side"], side)
            self.assertEqual(row["raw_registers"], [100, 200, 65535, 400, 500, 256] * 5)
            self.assertEqual(row["sampled_timestamp_us"], [990, 991, 992, 993, 994])
            self.assertEqual(row["finger_direction_valid"], [False] * 5)

    def test_bad_vectors_side_truncation_and_oversized_payload(self):
        bindings = capture.load_bindings()
        for data in (touch_payload(length=4), touch_payload(side=0), b"", touch_payload()[:32],
                     bytes(65537), b"\xff" * 16):
            with self.subTest(length=len(data)), self.assertRaises((ValueError, struct.error, TypeError)):
                capture.decode("hand_touch", data, bindings)

    def test_lowcmd_real_binding_preserves_all_fields(self):
        from examples import brainco_hand_control as demo
        from test_brainco_hand_control import synthetic_spec
        from locomotion_aorta import transport
        path = self.path / "spec"
        path.write_text(json.dumps(synthetic_spec()))
        command = demo.command_at(demo.load_plan(path), 1, 7)
        builder = flatbuffers.Builder(4096)
        builder.Finish(transport._fill_low_cmd(transport._load_aorta_bindings(), command, builder, 0))
        row = capture.decode("external_command", bytes(builder.Output()), capture.load_bindings())
        self.assertEqual(row["cmd_id"], 7)
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
            if topic == capture.TOPICS["hand_touch"]:
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
        self.assertEqual(result, {"hand_touch": 1})
        self.assertEqual(set(node.subscriptions), set(capture.TOPICS.values()))
        self.assertTrue(node.closed)
        rows = read_rows(output)
        self.assertEqual(rows[0]["capture_clock"], "observer_monotonic_ns")
        self.assertEqual(rows[-1]["dropped_messages"], 2)
        self.assertEqual(rows[-1]["transport_loss"], "not_measured")

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

    def mcap_file(self, *, root="aorta.topic.lowlevel.HandTouch", encoding="flatbuffer"):
        from mcap.writer import Writer, CompressionType
        source = self.path / "input.mcap"
        with source.open("wb") as stream:
            writer = Writer(stream, compression=CompressionType.NONE, enable_data_crcs=True)
            writer.start()
            schema = writer.register_schema(root, encoding, b"synthetic-bfbs-for-hash-guard-only")
            channel = writer.register_channel("aorta/default/pub/locomotion/hand_touch", "flatbuffer", schema)
            for side in (1, 2):
                writer.add_message(channel, side * 1000, touch_payload(side), side * 1000, side)
            writer.finish()
        return source

    def export(self, source, **kwargs):
        meta = metadata()
        meta["schema_sha256"] = hashlib.sha256(b"synthetic-bfbs-for-hash-guard-only").hexdigest()
        return capture.export_mcap(source, self.path / "export", meta,
            {"aorta/default/pub/locomotion/hand_touch": "hand_touch"},
            {"hand_touch": hashlib.sha256(b"synthetic-bfbs-for-hash-guard-only").hexdigest()}, **kwargs)

    def test_mcap_export_uses_exact_topic_hash_and_retains_source_hash(self):
        source = self.mcap_file()
        self.assertEqual(self.export(source), {"hand_touch": 2})
        rows = read_rows(self.path / "export")
        self.assertEqual(rows[0]["source_mcap_sha256"], hashlib.sha256(source.read_bytes()).hexdigest())
        self.assertEqual([r["hand_side"] for r in rows[1:-1]], [1, 2])
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
        for topics, hashes in (({"wrong/topic": "hand_touch"}, {"hand_touch": "0" * 64}),
                                ({"aorta/default/pub/locomotion/hand_touch": "hand_touch"}, {"hand_touch": "0" * 64})):
            with self.assertRaises(ValueError):
                capture.export_mcap(source, self.path / "export", metadata(), topics, hashes)
            self.assertFalse(read_rows(self.path / "export")[-1]["complete"])
            (self.path / "export").unlink()
