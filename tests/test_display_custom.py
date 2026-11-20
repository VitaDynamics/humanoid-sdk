"""Group selection tests; the injected Aorta module never opens a transport."""

import contextlib
import io
import os
import sys
from types import ModuleType
import unittest
from unittest.mock import Mock, patch

from examples.display import display_custom as example


class DisplayGroupTest(unittest.TestCase):
    def check_group(self, *, environment, expected, argv=None, explicit=None):
        aorta = ModuleType("aorta")
        node = Mock()
        observed = []

        def create_node(name, *, group):
            observed.append((name, group, os.environ["AORTA_GROUP"]))
            return node

        aorta.Node = Mock(side_effect=create_node)
        with patch.dict(os.environ, environment, clear=True):
            with patch.dict(sys.modules, {"aorta": aorta}):
                if argv is None:
                    kwargs = {} if explicit is None else {"group": explicit}
                    with example.Screen(**kwargs):
                        pass
                else:
                    with patch.object(
                        example.Screen, "external_control", return_value=False
                    ):
                        with contextlib.redirect_stdout(io.StringIO()):
                            self.assertEqual(example.main(argv), 0)
                self.assertEqual(os.environ["AORTA_GROUP"], expected)
        self.assertEqual(observed, [("external_display_python", expected, expected)])
        node.close.assert_called_once_with()

    def test_cli_inherits_deployment_group(self):
        self.check_group(
            environment={"AORTA_GROUP": "deployment_group"},
            expected="deployment_group",
            argv=["state"],
        )

    def test_cli_explicit_group_overrides_environment(self):
        self.check_group(
            environment={"AORTA_GROUP": "deployment_group"},
            expected="cli_group",
            argv=["--group", "cli_group", "state"],
        )

    def test_cli_falls_back_to_default_without_environment(self):
        self.check_group(environment={}, expected="default", argv=["state"])

    def test_python_api_uses_same_priority(self):
        for environment, explicit, expected in [
            ({"AORTA_GROUP": "deployment_group"}, None, "deployment_group"),
            ({"AORTA_GROUP": "deployment_group"}, "api_group", "api_group"),
            ({}, None, "default"),
        ]:
            with self.subTest(environment=environment, explicit=explicit):
                self.check_group(
                    environment=environment, explicit=explicit, expected=expected
                )


if __name__ == "__main__":
    unittest.main()
