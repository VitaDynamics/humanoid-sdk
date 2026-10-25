"""New-contract successors to the six retired T059 online scenarios.

The historical T059 bytes stay archived, not normalized into new wire goldens.
These exercise the actual CLI and shared Body allocator on synthetic feedback.
"""
from dataclasses import replace
import importlib.util
from pathlib import Path
import unittest

import locomotion_aorta as sdk
from locomotion_aorta import BODY_CMD_TOPIC
from locomotion_aorta.end_effector_types import CommandKey, ControlRef, Field, SampleStream
from examples.end_effector import end_effector_external as demo
from test_examples import FakeClock, RecordingController
from test_end_effector_external import Endpoint
from test_fixed_stand_pose import body_info
from body_fixtures import robot_info_reply


class EndEffectorSequenceTest(unittest.TestCase):
    def test_six_legacy_use_cases_keep_shared_keys_body_fields_returns_and_cancel(self):
        cases = {
            'single_target': (False, False, False, False),
            'single_round_trip': (False, False, True, False),
            'zero_single_round_trip': (True, False, True, False),
            'dual_sine_measured_hold': (False, True, False, False),
            'dual_sine_zero_hold': (True, True, False, False),
            'single_rejected': (False, False, False, True),
        }
        for name, (zero, sine, back, reject) in cases.items():
            with self.subTest(case=name):
                clock = FakeClock()
                c = RecordingController(clock)
                self.addCleanup(c.close)
                reply = robot_info_reply()
                c.get_robot_info = lambda timeout: replace(reply, info=replace(reply.info, body=body_info()))
                endpoints = [Endpoint(c, side, reject=reject) for side in
                             (('left', 'right') if sine else ('right',))]
                for endpoint, opening in zip(endpoints, (.03125, .0625) if sine else (.0625,)):
                    endpoint._state = replace(endpoint._state, pos=(opening,))
                originals = {e.side: e._state.pos for e in endpoints}
                observed_cancel = []
                original_exit = c.exit_external
                def cancel(timeout):
                    observed_cancel.append((c.command_count, tuple(len(e.commands) for e in endpoints)))
                    return original_exit(timeout)
                c.exit_external = cancel
                def sleep(seconds):
                    clock.sleep(seconds)
                    self.assertLess(clock.monotonic(), 30., 'bounded synthetic scenario did not finish')
                options = dict(timeout=1, duration=.2, sleep=sleep, monotonic=clock.monotonic,
                    emit=lambda _: None, body_pose='zero' if zero else 'measured',
                    return_to_start=back, pattern='sine' if sine else 'target',
                    period=1., sine_min=.015625, sine_max=.078125)
                targets = {e.side: demo.position_target(e.info, (.09375,), unit='m', torque=(0.,))
                           for e in endpoints}
                if reject:
                    with self.assertRaisesRegex(RuntimeError, 'reject'):
                        demo.run(c, endpoints, targets, **options)
                else:
                    demo.run(c, endpoints, targets, **options)
                body = [value for topic, value in c.transport.published if topic == BODY_CMD_TOPIC]
                self.assertTrue(body)
                keys = [CommandKey(value.session_id, value.sequence) for value in body]
                for command in body:
                    self.assertEqual(command.protocol_version, 2)
                    self.assertEqual(len(command.joints), 31)
                    self.assertFalse(hasattr(command, 'cmd_id'))
                for e in endpoints:
                    self.assertTrue(e.commands)
                    keys.extend(command.key for command in e.commands)
                    for command in e.commands:
                        self.assertEqual((command.descriptor_ref, command.layout_id, command.side),
                                         (e.info.descriptor_ref, e.info.layout_id, e.info.side))
                        self.assertEqual(command.control_ref, ControlRef('example-client', 'lease-example'))
                        self.assertEqual(command.target.tau_cmd, (0.,))
                        self.assertTrue(command.target.command_fields[0] & Field.TAU)
                        self.assertEqual(command.target.vel_limit, (0.,))
                        self.assertGreaterEqual(command.target.pos[0], e.info.pos_min[0])
                        self.assertLessEqual(command.target.pos[0], e.info.pos_max[0])
                        self.assertEqual(len(command.input_observations), 1)
                        sample = command.input_observations[0]
                        self.assertEqual((sample.descriptor_ref, sample.stream, sample.side),
                                         (e.info.descriptor_ref, SampleStream.EE_STATE, e.info.side))
                        self.assertEqual((command.ts_client_created_us, command.ts_client_send_us,
                                          command.sdk_time_valid_mask), (0, 0, 0))
                    self.assertTrue(all(b-a >= .02-1e-9 for a, b in zip(e.times, e.times[1:])))
                    if not reject:
                        self.assertEqual(e.commands[-1].target.pos, originals[e.side] if (back or sine) else (.09375,))
                self.assertEqual({key.session_id for key in keys}, {17})
                self.assertEqual(sorted(key.sequence for key in keys), list(range(1, len(keys)+1)))
                self.assertEqual(c.calls.count('enter_external'), 1)
                self.assertEqual(c.calls.count('exit_external'), 1)
                self.assertEqual(observed_cancel, [(c.command_count, tuple(len(e.commands) for e in endpoints))])
                if sine:
                    self.assertEqual(endpoints[0].times, endpoints[1].times)
                if zero:
                    self.assertTrue(all(joint.q == 0 for joint in body[-1].joints))

    def test_public_upgrade_has_no_legacy_online_fallback_but_keeps_nested_descriptions(self):
        for name in ('HandClient', 'HandCmd', 'HandState', 'HandTouch', 'HandJointCmd',
                     'HandJointState', 'FingerTouchMeta', 'HAND_TOPIC_PREFIX', 'GripperClient',
                     'GripperCmd', 'GripperState', 'GripperSample', 'GripperHostTarget', 'GRIPPER_TOPIC_PREFIX'):
            with self.subTest(symbol=name):
                self.assertFalse(hasattr(sdk, name))
                self.assertNotIn(name, sdk.__all__)
        for name in ('hand', '_hand_codec', 'gripper', '_gripper_codec', '_gripper_lifecycle'):
            with self.subTest(module=name):
                self.assertIsNone(importlib.util.find_spec('locomotion_aorta.' + name))
        for name in ('HandInfo', 'HandSide', 'HandType', 'InhibitReason', 'GripperInfo',
                     'GripperType', 'GripperLimitSource', 'EndEffectorClient'):
            self.assertIn(name, sdk.__all__)
            self.assertIsNotNone(getattr(sdk, name))
        root = Path(__file__).resolve().parents[1]
        for path in ('brainco_hand/brainco_hand_external.py', 'gripper/gripper_external.py'):
            self.assertFalse((root/'examples'/path).exists())
