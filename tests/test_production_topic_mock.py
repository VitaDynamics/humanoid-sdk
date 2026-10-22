"""Pure-Python invariants; these tests do not compile/run C++ or open Aorta."""
import argparse
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from tools.production_topic_mock import run




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
        c = json.loads(run.session_config(0, 24001))
        self.assertEqual(c["listen"]["endpoints"], ["tcp/127.0.0.1:0"])
        self.assertEqual(c["connect"]["endpoints"], ["tcp/127.0.0.1:24001"])

    def test_client_auxiliary_peer_stays_explicit_loopback(self):
        c = json.loads(run.session_config(0, [24001, 24003]))
        self.assertEqual(c["listen"]["endpoints"], ["tcp/127.0.0.1:0"])
        self.assertEqual(c["connect"]["endpoints"], ["tcp/127.0.0.1:24001", "tcp/127.0.0.1:24003"])

    def test_ephemeral_port_is_only_valid_for_listening(self):
        with self.assertRaises(ValueError):
            run.session_config(24001, 0)
        with self.assertRaises(ValueError):
            run.session_config(0, [24001, 0])

    def test_invalid_ports_rejected(self):
        for port in [True, -1, 1023, 65536, "192.168.1.1", 12.5]:
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

    def test_client_requires_explicit_option(self):
        with self.assertRaises(ValueError):
            run.validate_options(self.options(client_command=["python", "client.py"]))


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








if __name__ == "__main__":
    unittest.main()
