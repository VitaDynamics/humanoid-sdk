"""Action17 Joy input is the same pull reader used by velocity, never local HID."""
from contextlib import redirect_stderr
import io
import json
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import unittest
from unittest.mock import Mock, patch

from examples.teleop_policy import teleop_policy as teleop
from examples.velocity_mode import velocity_mode as velocity
from test_joy import PullRing, payload
from test_teleop_policy import CONTRACT


class TeleopJoyTest(unittest.TestCase):
    def test_old_input_options_are_rejected_before_model_or_transport(self):
        for option in ('--joystick', '--neutral-input'):
            with patch.object(sys, 'argv', ['teleop_policy', '--model', '/unused', option]), \
                 patch.object(teleop, 'Model') as model, redirect_stderr(io.StringIO()), \
                 self.assertRaises(SystemExit) as error:
                teleop.main()
            self.assertEqual(error.exception.code, 2)
            model.assert_not_called()

    def test_shared_reader_axes_edges_freshness_and_profile_mapping(self):
        ring = PullRing(256)
        node = NS(create_subscriber_pull=Mock(return_value=ring))
        with patch.dict(sys.modules, {'pygame': None}), patch('threading.Thread') as thread:
            directions = velocity.DirectionInput(NS(node=node))
            thread.assert_not_called()
        self.addCleanup(directions.close)
        origin = 1_000_000_000
        with patch.object(velocity.time, 'time_ns', return_value=origin):
            ring.put(payload(0, axes=(0., 0., 0., 0.)))
            directions.snapshot()
            ring.put(payload(1, axes=(-1., 1., 0., -1.), buttons=(1,)),
                     payload(2, axes=(-1., 1., 0., -1.), buttons=(0,)))
            axes = directions.snapshot()
        command = teleop.JoyCommand(CONTRACT)
        for _ in range(200):
            values = command.step(axes)
        self.assertAlmostEqual(values[0], .6)
        self.assertAlmostEqual(values[1], -.5)
        self.assertAlmostEqual(values[2], -.8)
        self.assertEqual(values[3], .855)
        self.assertEqual(directions.statistics()['joy_pressed_counts'], {0: 1})
        self.assertEqual(directions.statistics()['joy_released_counts'], {0: 1})
        for now, expected in ((origin+1_000_000_000, axes), (origin+1_000_000_001, ()), (origin-1, ())):
            with patch.object(velocity.time, 'time_ns', return_value=now):
                self.assertEqual(directions.snapshot(), expected)
        self.assertEqual(directions.statistics()['last_joy_ns'], origin)

    def test_contract_and_packaging_keep_deployed_command_and_no_hid_dependency(self):
        root = Path(__file__).resolve().parents[1]
        contract = json.loads((root/'examples/teleop_policy/action17_contract.json').read_text())
        self.assertEqual(contract['command'], CONTRACT['command'])
        text = (root/'pyproject.toml').read_text()
        self.assertNotIn('pygame', text)
        self.assertEqual(contract['command']['joy_axis_height'], -1)
        self.assertEqual(contract['inference_frequency_hz'], 50)
