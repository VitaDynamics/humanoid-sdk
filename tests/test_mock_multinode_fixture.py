"""Fixture assertions and cleanup with fake SDK handles; no sockets or Nodes."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

from tools.production_topic_mock import multinode_fixture, run


class MultiNodeFixtureTests(unittest.TestCase):
    def fixture(self, command_factory, *, negative=False):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        output = Path(directory.name)
        (output / "client-session.json").write_text(run.session_config(0, 24001))
        closed = []
        node = lambda: S(group=lambda: "mock-sdkmain-fixture")
        source = S(transport=S(node=node()), wait_state=lambda _: S(state_id=42),
                   close=lambda: closed.append("state"))
        with patch.object(multinode_fixture, "validate_client_environment", return_value="mock-sdkmain-fixture"), \
             patch.object(multinode_fixture, "port", return_value=24002), \
             patch("locomotion_aorta.PullStateSource", return_value=source), \
             patch("locomotion_aorta.PullController", side_effect=lambda **kw: command_factory(node, closed)):
            multinode_fixture.run(output, legacy_fixed_listener=negative)
        return output, closed, json.loads((output / "multinode-fixture.json").read_text())

    def controller(self, node, closed):
        return S(transport=S(node=node()), wait_state=lambda _: S(state_id=43),
                 get_robot_info=lambda **kw: S(info=S(body=S(slot_count=31))),
                 close=lambda: closed.append("command"))

    def test_both_independent_nodes_read_mock_and_close(self):
        _, closed, result = self.fixture(self.controller)
        self.assertEqual(result["status"], "PASS_SAME_PROCESS_TWO_NODES_WITH_MOCK")
        self.assertEqual([result["state_node_state_id"], result["command_node_state_id"]], [42, 43])
        self.assertEqual(closed, ["command", "state"])

    def test_old_profile_failure_is_only_expected_internal_address_in_use(self):
        from aorta.errors import InternalError
        def old(node, closed):
            raise InternalError("Address already in use")
        output, closed, result = self.fixture(old, negative=True)
        self.assertEqual(result["status"], "PASS_EXPECTED_OLD_FIXED_LISTENER_FAILURE")
        self.assertEqual(result["expected_listener_failure"]["code"], -5)
        self.assertEqual(closed, ["state"])
        self.assertEqual(json.loads((output / "client-session.generated.json").read_text())["listen"]["endpoints"], ["tcp/127.0.0.1:0"])
        self.assertEqual(json.loads((output / "client-session.json").read_text())["listen"]["endpoints"], ["tcp/127.0.0.1:24002"])

    def test_unrelated_internal_failure_is_not_accepted_as_negative(self):
        from aorta.errors import InternalError
        def unrelated(node, closed):
            raise InternalError("schema failure")
        with self.assertRaisesRegex(InternalError, "schema failure"):
            self.fixture(unrelated, negative=True)

    def test_positive_does_not_hide_address_conflict(self):
        from aorta.errors import InternalError
        def old(node, closed):
            raise InternalError("Address already in use")
        with self.assertRaises(InternalError):
            self.fixture(old)

    def test_negative_fails_if_both_nodes_succeed(self):
        with self.assertRaisesRegex(AssertionError, "unexpectedly allowed"):
            self.fixture(self.controller, negative=True)


if __name__ == "__main__":
    unittest.main()
