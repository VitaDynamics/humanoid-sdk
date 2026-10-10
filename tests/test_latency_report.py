"""Positive controls and fail-closed report checks; no network or Aorta needed."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parent / "latency"))
from report import STAGES, analyze, render


class LatencyReportTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.sent, self.client, self.received = [], [], []
        for seq in range(1, 4):
            begin = seq * 1_000_000_000
            self.sent.append(dict(sequence=seq, publish_ns=begin, return_ns=begin + 50, bytes=4096))
            self.client.append(dict(session=123, sequence=seq, decode_ns=begin + 100,
                decoded_ns=begin + 200, built_ns=begin + 300, fill_ns=begin + 400,
                filled_ns=begin + 500, bytes_ns=begin + 600, returned_ns=begin + 650))
            self.received.append(dict(session=123, sequence=seq, cmd_id=seq,
                callback_ns=begin + 700, copied_ns=begin + 800, valid=1))

    def analyze(self):
        for name, rows in (("sent", self.sent), ("client", self.client), ("received", self.received)):
            with (self.directory / f"{name}.csv").open("w", newline="") as out:
                writer = csv.DictWriter(out, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
        return analyze(self.directory, 123, 2, 50)

    def test_positive_control_exact_stages(self):
        result = self.analyze()
        self.assertEqual(result["rtt"]["n"], 3)
        self.assertEqual(len(result["stages"]), len(STAGES))
        self.assertAlmostEqual(sum(s["mean_ms"] for s in result["stages"]), result["rtt"]["mean_ms"])
        self.assertEqual(result["mock_send_hz"], 1)
        self.assertEqual(result["missing_reply"], 0)

    def test_loss_is_visible_without_nearest_timestamp_pairing(self):
        self.received.pop(1)
        result = self.analyze()
        self.assertEqual(result["missing_reply"], 1)
        self.assertEqual(result["rtt"]["n"], 2)

    def test_duplicates_rejected(self):
        self.received.append(self.received[0])
        with self.assertRaisesRegex(ValueError, "duplicate"):
            self.analyze()

    def test_wrong_session_rejected(self):
        self.received[0]["session"] = 124
        with self.assertRaisesRegex(ValueError, "session"):
            self.analyze()

    def test_orphan_rejected(self):
        self.received[0]["sequence"] = 4
        with self.assertRaisesRegex(ValueError, "orphan"):
            self.analyze()

    def test_zero_or_one_sample_rejected(self):
        self.received = self.received[:1]
        with self.assertRaisesRegex(ValueError, "fewer than two"):
            self.analyze()

    def test_short_window_rejected(self):
        self.sent[-1]["publish_ns"] -= 1
        with self.assertRaisesRegex(ValueError, "shorter"):
            self.analyze()

    def test_negative_stage_rejected(self):
        self.received[0]["callback_ns"] = 1
        with self.assertRaisesRegex(ValueError, "non-monotonic"):
            self.analyze()

    def test_bad_content_rejected(self):
        self.received[0]["valid"] = 0
        with self.assertRaisesRegex(ValueError, "invalid command"):
            self.analyze()

    def test_partial_failure_is_not_pass_and_html_escaped(self):
        render(self.directory, {"status": "FAIL", "results": [self.analyze()], "error": "<script>"})
        self.assertEqual(json.loads((self.directory / "results.json").read_text())["status"], "FAIL")
        self.assertIn("&lt;script&gt;", (self.directory / "report.html").read_text())
        self.assertNotIn("<script>", (self.directory / "report.html").read_text())


if __name__ == "__main__":
    unittest.main()
