"""Review regressions on isolated fixtures; no transport or customer execution."""
import json
import unittest

from tools.production_topic_mock import CaptureLog, auxiliary_capture_budget, report


class ReportReviewTest(unittest.TestCase):
    def setUp(self):
        from tests.test_production_topic_mock import ReportTests
        self.fixture = ReportTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.root = self.fixture.root

    def write(self, **metadata):
        self.fixture.write()
        path = self.root / "mock-result.json"
        current = json.loads(path.read_text())
        current.update(metadata)
        path.write_text(json.dumps(current))

    def client_trace(self, count):
        path = self.root / "client.jsonl"
        path.write_text("".join(json.dumps({"session_id": 42, "sequence": i + 1,
            "input_state_id": 100 + i, "input_state_ts_real_us": 1000000 + i * 20000,
            "cycle_start_monotonic_ns": 2000000000 + i * 20000000}) + "\n"
            for i in range(count)))
        return path

    def test_parse_failures_make_measurement_incomplete(self):
        self.write(parse_errors=1)
        result = report.write_report(self.root)
        self.assertEqual(result["status"], "INCOMPLETE_OR_CLOCK_ANOMALY")
        self.assertEqual(result["counts"]["parse_errors"], 1)

    def test_client_only_identity_is_counted_as_unreceived(self):
        self.write()
        result = report.write_report(self.root, self.client_trace(3))
        self.assertEqual(result["counts"]["client_sent"], 3)
        self.assertEqual(result["client_not_received"], [[42, 3]])
        self.assertEqual(result["status"], "INCOMPLETE_OR_CLOCK_ANOMALY")

    def test_received_rejection_is_separate_from_client_loss(self):
        f = self.fixture
        f.commands[0]["valid"] = 0
        f.applied.pop(0)
        f.sent[1].update(applied_session_id=0, applied_sequence=0, ts_first_output_us=0)
        self.write()
        result = report.write_report(self.root, self.client_trace(2))
        self.assertEqual(result["client_not_received"], [])
        self.assertEqual(result["client_received_rejected"], [[42, 1]])
        self.assertEqual(result["status"], "INCOMPLETE_OR_CLOCK_ANOMALY")

    def test_reordered_callback_stamps_preserve_report_and_anomaly(self):
        self.fixture.commands.reverse()
        self.write()
        result = report.write_report(self.root)
        self.assertIsNone(result["accepted_command_hz"])
        self.assertTrue(result["clock_anomalies"])
        self.assertEqual(result["status"], "INCOMPLETE_OR_CLOCK_ANOMALY")
        self.assertTrue((self.root / "report.json").is_file())
        self.assertEqual(result["command_arrival_interval"]["n"], 0)

    def test_backward_publication_clock_retains_raw_stamps(self):
        self.fixture.sent[-1]["publish_ns"] = 1
        self.write()
        result = report.write_report(self.root)
        self.assertIsNone(result["mock_state_hz"])
        self.assertTrue(any("raw_timestamps_ns" in r for r in result["clock_anomalies"]))

    def test_synthetic_tick_and_actual_dispatch_are_separate(self):
        for r in self.fixture.applied:
            r["actual_dispatch_ns"] = r["first_output_ns"] + 1000000
        self.write()
        result = report.write_report(self.root)
        self.assertEqual(result["synthetic_tick_dispatch_delay"]["mean_ms"], 1)
        self.assertEqual(result["status"], "MEASURED_SYNTHETIC_ONLY")

    def test_missing_decode_finish_is_not_fabricated(self):
        self.write()
        result = report.write_report(self.root, self.client_trace(2))
        self.assertEqual(result["segments"]["S1_to_S2"]["n"], 0)

    def test_signal_with_active_or_unknown_ownership_is_incomplete(self):
        for phase in (None, 4, 5):
            self.write(stopped_by_signal=True, control_phase_at_stop=phase)
            self.assertEqual(report.write_report(self.root)["status"], "INCOMPLETE_OR_CLOCK_ANOMALY")
        self.write(stopped_by_signal=True, control_phase_at_stop=0)
        self.assertEqual(report.write_report(self.root)["status"], "MEASURED_SYNTHETIC_ONLY")

    def test_rejected_retransmission_does_not_erase_report(self):
        self.fixture.commands.append(dict(self.fixture.commands[0], valid=0))
        self.write()
        result = report.write_report(self.root)
        self.assertEqual(result["counts"]["commands_rejected"], 1)
        self.assertEqual(result["status"], "INCOMPLETE_OR_CLOCK_ANOMALY")

    def test_auxiliary_capture_error_marks_incomplete(self):
        self.write()
        (self.root / "auxiliary.json").write_text(json.dumps({
            "records": [], "capture_error": "command budget exhausted", "limits": []}))
        self.assertEqual(report.write_report(self.root)["status"], "INCOMPLETE_OR_CLOCK_ANOMALY")


class AuxiliaryBudgetTest(unittest.TestCase):
    def test_publications_joy_and_commands_share_explicit_capacity(self):
        spec = {"left": {"kind": "gripper", "state_hz": 50, "info_hz": 5},
                "joy": [{"at_ms": 1}]}
        budget = auxiliary_capture_budget(spec, 10)
        self.assertEqual(budget["publication_bound"], 553)
        self.assertEqual(budget["publication_bound"] + budget["command_bound"], budget["max_records"])

    def test_command_budget_is_checked_before_start(self):
        with self.assertRaisesRegex(ValueError, "capacity"):
            auxiliary_capture_budget({"joy": [{}], "command_record_budget": 1000000}, 1)

    def test_rejected_commands_also_use_budget_without_overwrite(self):
        log = CaptureLog({"max_records": 5, "command_bound": 1})
        log.append({"kind": "command", "accepted": False})
        with self.assertRaisesRegex(RuntimeError, "command budget"):
            log.append({"kind": "command", "accepted": True})
        self.assertEqual(len(log), 1)
        self.assertEqual(log.command_records, 1)
        self.assertIsNotNone(log.error)


if __name__ == "__main__":
    unittest.main()
