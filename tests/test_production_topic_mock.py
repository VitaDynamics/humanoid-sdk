"""Pure-Python invariants; these tests do not compile/run C++ or open Aorta."""
import argparse
import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from tools.production_topic_mock import report, run


class IsolationTests(unittest.TestCase):
    def test_mock_profile_has_no_router_or_discovery(self):
        c = json.loads(run.session_config(24001))
        self.assertEqual(c["mode"], "peer")
        self.assertEqual(c["listen"]["endpoints"], ["tcp/127.0.0.1:24001"])
        self.assertEqual(c["connect"]["endpoints"], [])
        self.assertFalse(c["scouting"]["multicast"]["enabled"])
        self.assertFalse(c["scouting"]["gossip"]["enabled"])
        self.assertFalse(c["transport"]["shared_memory"]["enabled"])

    def test_client_only_connects_to_assigned_loopback_peer(self):
        c = json.loads(run.session_config(24002, 24001))
        self.assertEqual(c["connect"]["endpoints"], ["tcp/127.0.0.1:24001"])

    def test_invalid_ports_rejected(self):
        for port in [True, 0, 1023, 65536, "192.168.1.1", 12.5]:
            with self.subTest(port=port), self.assertRaises(ValueError):
                run.session_config(port)

    def test_default_and_unsafe_groups_rejected(self):
        for group in ["default", "robot", "mock-sdkmain-a/b", "mock-sdkmain-\n", "mock-sdkmain-机器", "mock-sdkmain-" + "a" * 80]:
            with self.subTest(group=group), self.assertRaises(ValueError):
                run.validate_group(group)

    def test_group_alias_and_uri_only_change_child(self):
        parent = {"SDK_GROUP": "mock-sdkmain-test", "AORTA_GROUP": "default", "AORTA_NAMESPACE": "robot",
                  "ZENOH_SESSION_CONFIG_URI": "/app_param/real.json5", "ZENOH_CONFIG": "unsafe", "PATH": "/bin"}
        before = dict(parent)
        child = run.child_environment(parent, "mock-sdkmain-test", "/tmp/mock-client.json")
        self.assertEqual(parent, before)
        self.assertEqual(child["AORTA_GROUP"], "mock-sdkmain-test")
        self.assertEqual(child["SDK_GROUP"], "mock-sdkmain-test")
        self.assertEqual(child["ZENOH_SESSION_CONFIG_URI"], "/tmp/mock-client.json")
        self.assertNotIn("AORTA_NAMESPACE", child)
        self.assertNotIn("ZENOH_CONFIG", child)

    def test_conflicting_sdk_group_is_rejected(self):
        with self.assertRaises(ValueError):
            run.child_environment({"SDK_GROUP": "default"}, "mock-sdkmain-private", "/tmp/session.json")

    def options(self, **values):
        base = dict(mode="production-topics", seconds=60, state_hz=400, apply_hz=500,
                    command_timeout_ms=1000, prepare_timeout_ms=5000,
                    client_command=None, allow_client=False)
        return argparse.Namespace(**{**base, **values})

    def test_production_mode_must_be_explicit(self):
        with self.assertRaises(ValueError):
            run.validate_options(self.options(mode=None))

    def test_nonfinite_and_unbounded_rates_rejected(self):
        for name in ("seconds", "state_hz", "apply_hz"):
            for value in (float("nan"), float("inf"), 0, 5000):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    run.validate_options(self.options(**{name: value}))

    def test_capture_capacity_includes_cancel_grace(self):
        with self.assertRaises(ValueError):
            run.validate_options(self.options(seconds=500, state_hz=2000, client_command=["python", "client.py"], allow_client=True))

    def test_client_requires_explicit_option(self):
        with self.assertRaises(ValueError):
            run.validate_options(self.options(client_command=["python", "client.py"]))

    def test_customer_shell_is_not_a_safe_client_entry(self):
        for command in (["bash", "entry.sh"], ["/userdata/copy/scripts/start_onboard_vita.sh"]):
            with self.subTest(command=command), self.assertRaises(ValueError):
                run.validate_options(self.options(client_command=command, allow_client=True))

    def test_direct_client_option_does_not_start_any_process(self):
        with mock.patch.object(run.subprocess, "Popen", side_effect=AssertionError("must not execute")):
            run.validate_options(self.options(client_command=["python", "reviewed.py"], allow_client=True))

    def test_help_has_no_sdk_import_or_process_execution(self):
        with mock.patch.object(run.subprocess, "Popen", side_effect=AssertionError("must not execute")):
            self.assertIn("--mode", run.parser().format_help())

    def test_initial_imu_preserves_explicit_normalized_fixture(self):
        q = [[1, 0, 0, 0], [0, 0, 0, 1]]
        self.assertEqual(run.validate_imu(q), q)

    def test_bad_imu_is_not_silently_normalized(self):
        for q in ([[1, 0, 0, 0]], [[2, 0, 0, 0], [1, 0, 0, 0]],
                  [[True, 0, 0, 0], [1, 0, 0, 0]], [[float("nan"), 0, 0, 0], [1, 0, 0, 0]]):
            with self.subTest(q=q), self.assertRaises(ValueError):
                run.validate_imu(q)

    def test_protocol_fixture_client_is_explicit(self):
        with self.assertRaises(ValueError):
            run.validate_options(self.options(self_test_client=True))

    def test_protocol_fixture_cannot_replace_custom_client(self):
        with self.assertRaises(ValueError):
            run.validate_options(self.options(self_test_client=True, allow_client=True, client_command=["python", "custom.py"]))


class BuildIdentityTests(unittest.TestCase):
    def test_generated_schema_tampering_is_rejected_before_launch(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)
            (p / "generated").mkdir()
            core = p / "core.so"
            core.write_bytes(b"offline fixture only")
            (p / "libaorta_core.so.13").symlink_to(core)
            binary = p / "production-topic-mock"
            binary.write_bytes(b"never executed")
            generated = {}
            for key in ("body_state", "body_cmd", "get_robot_info", "control_request", "control_ack", "control_status"):
                f = p / "generated" / (key + ".bfbs")
                f.write_bytes(key.encode())
                generated[str(f.relative_to(p))] = run.digest(f)
            manifest = dict(mode="production-topics", target_arch=run.platform.machine(), binary_sha256=run.digest(binary),
                            core_path=str(core), core_sha256=run.digest(core), aorta_abi=13, generated_sha256=generated)
            (p / "build.json").write_text(json.dumps(manifest))
            self.assertEqual(run.verify_build(p)["aorta_abi"], 13)
            (p / "generated/body_state.bfbs").write_bytes(b"different")
            with self.assertRaisesRegex(ValueError, "checksum"):
                run.verify_build(p)

    def test_core_and_binary_identity_are_required(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)
            (p / "production-topic-mock").write_bytes(b"changed")
            (p / "build.json").write_text(json.dumps(dict(mode="production-topics", target_arch=run.platform.machine(), binary_sha256="0"*64)))
            with self.assertRaisesRegex(ValueError, "binary checksum"):
                run.verify_build(p)


class ReportTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.sent = [dict(state_id=100 + i, publish_ns=1000000000 + i * 20000000,
                         publish_return_ns=1000000100 + i * 20000000, state_real_us=1000000 + i * 20000,
                         applied_session_id=0, applied_sequence=0, ts_first_output_us=0) for i in range(3)]
        self.commands = [dict(session_id=42, sequence=i + 1, input_state_id=100 + i,
                              input_state_ts_real_us=1000000 + i * 20000, client_created_us=0, client_send_us=0,
                              received_ns=1005000000 + i * 20000000, accepted_ns=1005100000 + i * 20000000,
                              valid=1) for i in range(2)]
        self.applied = [dict(session_id=42, sequence=i + 1, first_output_ns=1006000000 + i * 20000000) for i in range(2)]
        for i in range(1, 3):
            self.sent[i].update(applied_session_id=42, applied_sequence=i, ts_first_output_us=(1006000000 + (i - 1) * 20000000) // 1000)

    def write(self):
        for name, records in [("states", self.sent), ("received", self.commands), ("applied", self.applied)]:
            keys = list(records[0]) if records else ({"states": list(self.sent[0]), "received": list(self.commands[0]),
                                                     "applied": ["session_id", "sequence", "first_output_ns"]}[name])
            with (self.root / (name + ".csv")).open("w", newline="") as f:
                writer = csv.DictWriter(f, keys)
                writer.writeheader()
                writer.writerows(records)
        (self.root / "mock-result.json").write_text(json.dumps({"synthetic": True, "capture_overflow": False, "info_errors": 0}))

    def analyze(self):
        self.write()
        return report.analyze(self.root)

    def test_exact_input_state_and_apply_join(self):
        r = self.analyze()
        self.assertEqual(r["status"], "MEASURED_SYNTHETIC_ONLY")
        self.assertEqual(r["segments"]["S0_to_G"]["n"], 2)
        self.assertEqual(r["segments"]["S0_to_G"]["mean_ms"], 6)
        self.assertEqual(r["accepted_command_hz"], 50)

    def test_missing_client_stages_are_not_zero(self):
        r = self.analyze()
        self.assertEqual(r["segments"]["S3_to_S4"], {"status": "NOT_MEASURED", "n": 0})

    def test_nonmatching_input_is_not_joined_by_time(self):
        self.commands[0]["input_state_id"] = 999
        r = self.analyze()
        self.assertEqual(len(r["unmatched_input_states"]), 1)
        self.assertEqual(r["segments"]["S0_to_G"]["n"], 1)
        self.assertEqual(r["status"], "INCOMPLETE_OR_CLOCK_ANOMALY")

    def test_same_state_id_with_wrong_source_stamp_not_joined(self):
        self.commands[0]["input_state_ts_real_us"] += 1
        self.assertEqual(len(self.analyze()["unmatched_input_states"]), 1)

    def test_uint64_identities_not_truncated(self):
        for records in (self.commands, self.applied):
            for row in records:
                row["session_id"] = 2**63 + 8
                row["sequence"] += 2**32
        for row in self.sent[1:]:
            row["applied_session_id"] = 2**63 + 8
            row["applied_sequence"] += 2**32
        r = self.analyze()
        self.assertEqual(r["joined"][0]["session_id"], 2**63 + 8)
        self.assertEqual(r["joined"][0]["sequence"], 2**32 + 1)

    def test_duplicate_command_identity_rejected(self):
        self.commands.append(dict(self.commands[0]))
        with self.assertRaisesRegex(ValueError, "duplicate"):
            self.analyze()

    def test_orphan_apply_rejected(self):
        self.applied[0]["sequence"] = 99
        with self.assertRaisesRegex(ValueError, "orphan"):
            self.analyze()

    def test_unapplied_accepted_command_not_fabricated(self):
        self.applied.pop()
        self.sent[2].update(applied_session_id=42, applied_sequence=1, ts_first_output_us=1006000)
        r = self.analyze()
        self.assertEqual(r["counts"]["accepted_not_applied"], 1)
        self.assertEqual(r["segments"]["S0_to_G"]["n"], 1)

    def test_invalid_body_frame_kept_separate(self):
        self.commands[0]["valid"] = 0
        self.applied.pop(0)
        self.sent[1].update(applied_session_id=0, applied_sequence=0, ts_first_output_us=0)
        r = self.analyze()
        self.assertEqual(r["counts"]["commands_rejected"], 1)

    def test_applied_echo_identity_is_verified(self):
        self.sent[1]["applied_sequence"] = 99
        with self.assertRaisesRegex(ValueError, "unapplied"):
            self.analyze()

    def test_applied_echo_timestamp_is_verified(self):
        self.sent[1]["ts_first_output_us"] += 1
        with self.assertRaisesRegex(ValueError, "echo differs"):
            self.analyze()

    def test_echo_delay_and_sequence_lag_have_different_units(self):
        r = self.analyze()
        self.assertEqual(r["applied_echo_delay"]["mean_ms"], 14)
        self.assertEqual(r["applied_sequence_lag"]["unit"], "commands")

    def test_clock_reversal_is_reported_without_clamping(self):
        self.commands[0]["received_ns"] = 999999999
        r = self.analyze()
        self.assertEqual(r["status"], "INCOMPLETE_OR_CLOCK_ANOMALY")
        self.assertEqual(r["clock_anomalies"][0]["segment"], "S0_to_L1")

    def test_zero_or_negative_stats_not_fabricated(self):
        self.assertEqual(report.stats([])["status"], "NOT_MEASURED")
        with self.assertRaises(ValueError):
            report.stats([-1])
        with self.assertRaises(ValueError):
            report.stats([float("nan")])

    def test_client_s3_must_be_inference_start(self):
        self.write()
        log = self.root / "client.jsonl"
        cmd = self.commands[0]
        log.write_text(json.dumps({**cmd, "timing_wall_ns": {"policy_started_wall_ns": 1001000000,
                         "inference_finished_wall_ns": 1002000000}}) + "\n")
        r = report.analyze(self.root, log)
        self.assertEqual(r["segments"]["S3_to_S4"]["n"], 0)

    def test_report_artifacts_do_not_claim_hardware_pass(self):
        self.write()
        report.write_report(self.root)
        text = (self.root / "report.md").read_text()
        self.assertIn("NOT_MEASURED", text)
        self.assertIn("synthetic", text)

    def test_client_cycle_clock_is_not_inferred_from_arrivals(self):
        self.write()
        log = self.root / "client.jsonl"
        log.write_text("".join(json.dumps({**r, "cycle_start_monotonic_ns": 9000000000+i*25000000})+"\n" for i, r in enumerate(self.commands)))
        result = report.analyze(self.root, log)
        self.assertEqual(result["accepted_command_hz"], 50)
        self.assertEqual(result["client_cycle_hz"], 40)


if __name__ == "__main__":
    unittest.main()
