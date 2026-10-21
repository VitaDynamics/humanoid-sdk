"""RobotInfo admission precedes control ownership; no real transport or model."""
from contextlib import nullcontext
from functools import partial
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

try:
    import numpy as np
except ImportError:
    raise unittest.SkipTest('install the policy demo extras to run preflight tests')

from examples.velocity_mode import velocity_mode
from examples.velocity_mode.velocity_mode_policy import VelocityModePolicy
from locomotion_aorta._policy_state import ASAMPLE_BODY_JOINT_NAMES
from locomotion_aorta.wbc_tracking_policy import CONTRACT_KEY
from test_wbc_tracking_policy import FakeClock, FakeController, FakeModel, example, policy


ROOT = Path(__file__).resolve().parents[1]


def body_info(names=ASAMPLE_BODY_JOINT_NAMES, unpopulated=()):
    return SimpleNamespace(joint_names=names, slot_count=len(names), roster_version=1,
        joints=[SimpleNamespace(populated=name != 'waist_roll_joint' and name not in unpopulated,
                                is_mock=False) for name in names])


class PolicyPreflightTest(unittest.TestCase):
    def velocity(self, body, expected_missing=()):
        metadata = json.loads((ROOT/'tests/fixtures/velocity_model_metadata.json').read_text())
        p = VelocityModePolicy.__new__(VelocityModePolicy)
        p.names = metadata['joint_names'].split(',')
        p.config = {'fixed_joints': {n: {'q': 0., 'kp': 20., 'kd': 1.}
                                  for n in ('neck_pitch_joint', 'neck_yaw_joint')}}
        p.inventory = {}
        controller, transport, directions = Mock(), Mock(), Mock()
        controller.get_robot_info.return_value = SimpleNamespace(info=SimpleNamespace(body=body))
        controller.exit_external.return_value = SimpleNamespace(current_action='PASSIVE',
                                                               phase=SimpleNamespace(value='IDLE'))
        transport.observed_status = SimpleNamespace(current_action='PASSIVE',
            phase=SimpleNamespace(value='IDLE'), generation=0)
        transport.publishers, directions.node.publishers = {}, []
        directions.statistics.return_value = {}
        with tempfile.TemporaryDirectory() as temp:
            args = velocity_mode.parse_args(['--live', '--locomotion-dir', '/unused',
                                             '--log', str(Path(temp)/'velocity.jsonl')])
            with patch.object(velocity_mode, 'parse_args', return_value=args), \
                 patch.object(velocity_mode, 'version', return_value='0.2.0'), \
                 patch.object(velocity_mode, 'VelocityModePolicy', return_value=p), \
                 patch.object(velocity_mode, 'PolicyTransport', return_value=transport), \
                 patch.object(velocity_mode, 'ExternalController', return_value=controller), \
                 patch.object(velocity_mode, 'DirectionInput', return_value=directions), \
                 patch.object(velocity_mode, 'transition_duration', return_value=([], 0.)), \
                 patch.object(velocity_mode, 'run_cycles') as run:
                if expected_missing:
                    with self.assertRaises(ValueError) as error:
                        velocity_mode.main()
                    for name in expected_missing:
                        self.assertIn(name, str(error.exception))
                    controller.enter_external.assert_not_called()
                    controller.exit_external.assert_not_called()
                    run.assert_not_called()
                else:
                    velocity_mode.main()
                    controller.enter_external.assert_called_once_with(args.timeout)
                    controller.exit_external.assert_called_once_with(args.timeout)
                    run.assert_called_once()
        controller.close.assert_called_once()

    def wbc(self, body, expected_missing=()):
        p, clock = policy(), FakeClock()
        client = FakeController(p, clock)
        client.get_robot_info = Mock(return_value=SimpleNamespace(info=SimpleNamespace(body=body)))
        model = FakeModel()
        model.contract = p.contract
        model.session = SimpleNamespace(metadata={CONTRACT_KEY:
            (ROOT/'tests/fixtures/actor_g1_contract.json').read_text()})
        source = Mock()
        source.wait_body_state = client.wait_body_state
        with tempfile.TemporaryDirectory() as temp:
            model_path, log_path = Path(temp)/'model.onnx', Path(temp)/'wbc.jsonl'
            model_path.write_bytes(b'explicit unit model; inference mocked')
            argv = ['wbc_tracking_mode', '--live', '--confirm-motion', '--model', str(model_path),
                    '--profile', str(ROOT/'examples/wbc_tracking_mode/wbc_tracking_asample.json'),
                    '--duration', '.041', '--log', str(log_path)]
            with patch.object(sys, 'argv', argv), \
                 patch.dict('os.environ', {'ZENOH_SESSION_CONFIG_URI': 'unit-only-no-transport'}), \
                 patch('locomotion_aorta.wbc_tracking_policy.Model', return_value=model), \
                 patch('locomotion_aorta.PullController', return_value=nullcontext(client)), \
                 patch('locomotion_aorta.PullStateSource', return_value=source), \
                 patch.object(example, 'live', partial(example.live,
                    monotonic=clock.monotonic, sleep=clock.sleep)):
                if expected_missing:
                    with self.assertRaises(ValueError) as error:
                        example.main()
                    for name in expected_missing:
                        self.assertIn(name, str(error.exception))
                    self.assertEqual((client.enters, client.exits), (0, 0))
                    self.assertFalse(client.commands)
                else:
                    example.main()
                    self.assertEqual((client.enters, client.exits), (1, 1))
                    self.assertTrue(client.commands)
        source.close.assert_called_once()

    def test_velocity_unpopulated_policy_joint_never_enters(self):
        name = ASAMPLE_BODY_JOINT_NAMES[0]
        self.velocity(body_info(unpopulated=[name]), [name])

    def test_velocity_missing_policy_name_never_enters(self):
        name = ASAMPLE_BODY_JOINT_NAMES[0]
        self.velocity(body_info(ASAMPLE_BODY_JOINT_NAMES[1:]), [name])

    def test_velocity_unpopulated_fixed_neck_never_enters(self):
        self.velocity(body_info(unpopulated=['neck_pitch_joint']), ['neck_pitch_joint'])

    def test_velocity_complete_permuted_roster_enters(self):
        self.velocity(body_info(tuple(reversed(ASAMPLE_BODY_JOINT_NAMES))))

    def test_wbc_unpopulated_policy_joint_never_enters(self):
        name = ASAMPLE_BODY_JOINT_NAMES[0]
        self.wbc(body_info(unpopulated=[name]), [name])

    def test_wbc_missing_policy_name_never_enters(self):
        name = ASAMPLE_BODY_JOINT_NAMES[0]
        self.wbc(body_info(ASAMPLE_BODY_JOINT_NAMES[1:]), [name])

    def test_wbc_missing_neck_names_never_enters_and_lists_every_name(self):
        for missing in [('neck_pitch_joint',), ('neck_yaw_joint',),
                        ('neck_pitch_joint', 'neck_yaw_joint')]:
            with self.subTest(missing=missing):
                self.wbc(body_info(tuple(n for n in ASAMPLE_BODY_JOINT_NAMES if n not in missing)), missing)

    def test_wbc_unpopulated_fixed_neck_never_enters(self):
        self.wbc(body_info(unpopulated=['neck_yaw_joint']), ['neck_yaw_joint'])

    def test_wbc_complete_permuted_roster_enters(self):
        self.wbc(body_info(tuple(reversed(ASAMPLE_BODY_JOINT_NAMES))))
