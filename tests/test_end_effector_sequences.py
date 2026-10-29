"""New-contract successors to the six retired T059 online scenarios.

The historical T059 bytes stay archived, not normalized into new wire goldens.
These exercise the actual CLI and shared Body allocator on synthetic feedback.
"""
from dataclasses import replace
import importlib.util
import importlib
import os
from pathlib import Path
import unittest
from unittest.mock import patch

import locomotion_aorta as sdk
from locomotion_aorta import BODY_CMD_TOPIC
from locomotion_aorta.end_effector_types import CommandKey, Field, SampleStream
from examples.end_effector import end_effector_external as demo
from test_examples import FakeClock, RecordingController
from test_end_effector_external import Endpoint
from test_fixed_stand_pose import body_info
from body_fixtures import robot_info_reply


class RobotProducedEndEffectorSequenceTest(unittest.TestCase):
    """Pkg supplies actual C++ capture/typed-serializer outputs; never synthesize.

    Missing inputs are a failure, including in source discovery. This contract
    is deliberately limited to the C2i named sequences, not a fixture runner.
    """
    def setUp(self):
        from test_controller import ControllerHarness
        from test_end_effector_client import Node
        self.root = Path(os.environ['PR2_C2I_PRODUCER_FIXTURES'])
        self.assertTrue(self.root.is_dir(), 'actual robot producer input directory is required')
        self.clock = FakeClock()
        name = self._testMethodName
        group = ('static' if name == 'test_actual_robot_static_info_republish_and_runtime_restart_boundaries'
                 else 'brainco' if name == 'test_actual_robot_touch_disable_replay_and_true_next_sample'
                 else 'linker')
        self.harness = ControllerHarness(info_reply=self.service_reply(group + '/robot_info.fb'))
        self.addCleanup(self.harness.controller.close)
        self.node = Node(); self.harness.transport.node = self.node
        self.client = sdk.EndEffectorClient('left', connection=self.harness.controller,
            observation_sources=((SampleStream.EE_STATE, 1),))
        self.addCleanup(self.client.close)
        clock_patch = patch('locomotion_aorta.end_effector.time.monotonic', self.clock.monotonic)
        clock_patch.start(); self.addCleanup(clock_patch.stop)

    def service_reply(self, name):
        from locomotion_aorta._body_codec import decode_robot_info_response
        payload = (self.root / name).read_bytes()
        self.assertTrue(payload, 'actual service reply is required')
        module = importlib.import_module('aorta.services.humanoid_locomotion.GetRobotInfoResponse')
        return decode_robot_info_response(module.GetRobotInfoResponse.GetRootAs(payload))

    def feed(self, kind, name, *, client=None):
        client = self.client if client is None else client
        payload = (self.root/name).read_bytes()  # No fallback or Python re-encoding.
        self.assertTrue(payload)
        decoder, callback = self.node.subscriptions[
            '/sdk/foot_humanoid/end_effector/' + client.side + '/' + kind]
        callback(decoder(payload), None)
        return getattr(client, kind)

    def test_actual_robot_static_info_republish_and_runtime_restart_boundaries(self):
        right = sdk.EndEffectorClient('right', connection=self.harness.controller,
            observation_sources=((SampleStream.EE_STATE, 2),))
        self.addCleanup(right.close)
        for client in (self.client, right):
            self.feed('info', 'static/' + client.side + '_info.fb', client=client)
            self.feed('state', 'static/' + client.side + '_state.fb', client=client)
        info, state, feedback = self.client.info, self.client.state, self.client.motion_feedback()
        old_ref = info.runtime_id
        old_sample = state.sample_ref
        republished = self.feed('info', 'static/left_info_republished.fb')
        self.assertEqual(republished.runtime_id, old_ref)
        self.assertIs(self.client.state, state)
        self.assertEqual(self.client.motion_feedback(), feedback)
        continued = self.feed('state', 'static/left_next_state.fb')
        self.assertEqual(continued.runtime_id, old_ref)
        self.assertGreater(continued.state_id, state.state_id)
        self.feed('state', 'static/right_inhibited.fb', client=right)
        with self.assertRaisesRegex(RuntimeError, 'inhibited|permission|ready'):
            demo._EndpointGuard(right, right.info, 1., self.clock.monotonic).feedback(True)
        self.harness.accept_enter(); self.harness.controller.enter_external(timeout=.1)
        guard = demo._EndpointGuard(self.client, info, 1., self.clock.monotonic)
        target = demo.position_target(info, continued.pos, unit='rad')
        command = guard.publish(target, continued)
        self.assertEqual(command.runtime_id, old_ref)
        sent = len(self.node.commands)
        self.assertIs(self.feed('info', 'static/left_new_runtime_info.fb'), republished)
        self.assertEqual(right.info.runtime_id, old_ref)
        self.assertEqual(len(self.node.commands), sent)
        self.assertEqual(info.runtime_id, old_ref)
        new_reply = self.service_reply('static/new_runtime_robot_info.fb')
        self.assertNotEqual(new_reply.runtime_id, old_ref)
        self.harness.transport.get_robot_info = lambda timeout: new_reply
        with self.assertRaisesRegex(ValueError, 'new connection'):
            self.harness.controller.get_robot_info(.1)
        with self.assertRaises(ValueError):
            self.client.publish(target, input_observations=(old_sample,))
        with self.assertRaises(ValueError):
            right.wait_info(.01)
        self.assertEqual(len(self.node.commands), sent)
        from test_controller import ControllerHarness
        fresh = ControllerHarness(info_reply=new_reply)
        self.addCleanup(fresh.controller.close)
        fresh.transport.node = self.node
        new_client = sdk.EndEffectorClient('left', connection=fresh.controller,
            observation_sources=((SampleStream.EE_STATE, 1),))
        self.addCleanup(new_client.close)
        new_info = self.feed('info', 'static/left_new_runtime_info.fb', client=new_client)
        self.assertEqual(new_info.runtime_id, new_reply.runtime_id)
        fresh.accept_enter(); fresh.controller.enter_external(timeout=.1)
        with self.assertRaises(ValueError):
            new_client.publish(target, input_observations=(old_sample,))
        self.assertEqual(len(self.node.commands), sent)

    def linker(self):
        self.feed('info', 'linker/info.fb')
        position = self.feed('state', 'linker/position.fb')
        self.assertTrue(all(mask & Field.POS for mask in position.valid_fields))
        return position, demo._EndpointGuard(self.client, self.client.info,
                                             1., self.clock.monotonic)

    def test_actual_robot_linker_position_diagnostics_and_fault_reach_guard(self):
        position, guard = self.linker()
        zero = self.feed('state', 'linker/zero_fault.fb')
        temp = self.feed('state', 'linker/temperature.fb')
        self.assertTrue(all(mask & Field.ERROR_CODE and not mask & Field.POS for mask in zero.valid_fields))
        self.assertTrue(all(mask & Field.TEMPERATURE and not mask & Field.POS for mask in temp.valid_fields))
        self.assertIs(guard.feedback(True), position)
        self.harness.accept_enter(); self.harness.controller.enter_external(timeout=.1)
        target = demo.position_target(self.client.info, position.pos, unit='rad')
        command = guard.publish(target, position)
        self.assertEqual(command.input_observations, (position.sample_ref,))
        newer = self.feed('state', 'linker/next_position.fb')
        self.assertGreater(newer.state_id, temp.state_id)
        self.assertIs(guard.feedback(True), newer)
        self.feed('state', 'linker/nonzero_fault.fb')
        self.feed('state', 'linker/temperature_after_fault.fb')
        with self.assertRaisesRegex(RuntimeError, 'nonzero|inhibited'):
            guard.feedback(True)

    def test_actual_robot_diagnostics_cannot_extend_position_receipt_deadline(self):
        position, guard = self.linker()
        self.clock.sleep(demo.body.LOWSTATE_WINDOW_S / 2)
        self.feed('state', 'linker/zero_fault.fb')
        self.feed('state', 'linker/temperature.fb')
        self.clock.sleep(demo.body.LOWSTATE_WINDOW_S / 2)
        with self.assertRaisesRegex(RuntimeError, 'position became stale'):
            guard.feedback(True)
        self.assertEqual(self.client.motion_feedback()[1].sample_ref, position.sample_ref)

    def test_actual_robot_same_sample_permission_and_stale_are_consumed(self):
        position, guard = self.linker()
        revoked = self.feed('state', 'linker/permission_revoked.fb')
        self.assertEqual(revoked.state_id, position.state_id)
        self.assertEqual(revoked.ts_ingested_us, position.ts_ingested_us)
        with self.assertRaisesRegex(RuntimeError, 'inhibited|permission'):
            guard.feedback(True)
        self.feed('state', 'linker/position.fb')
        self.assertIs(self.client.state, revoked)
        stale = self.feed('state', 'linker/position_stale.fb')
        self.assertEqual(stale.state_id, position.state_id)
        self.assertFalse(stale.sample_ref.validity & 4)
        self.assertIs(self.client.motion_feedback()[1], stale)

    def test_actual_robot_touch_disable_replay_and_true_next_sample(self):
        self.feed('info', 'brainco/info.fb')
        old = self.feed('touch', 'brainco/touch.fb')
        disabled = self.feed('touch', 'brainco/touch_disabled.fb')
        self.assertEqual(disabled.touch_state_id, old.touch_state_id)
        self.assertEqual(disabled.payload, old.payload)
        self.assertFalse(disabled.sample_ref.validity & 4)
        self.feed('touch', 'brainco/touch.fb')
        self.assertIs(self.client.touch, disabled)
        # A deterministic condition seam advances only the wait deadline. The
        # consumer and actual serialized samples remain production code/data.
        with patch.object(self.client._condition, 'wait', side_effect=self.clock.sleep):
            with self.assertRaises(TimeoutError):
                self.client.wait_touch(.01, after_sample=old.sample_ref)
        newer = self.feed('touch', 'brainco/touch_new.fb')
        self.assertGreater(newer.touch_state_id, old.touch_state_id)
        self.assertTrue(newer.sample_ref.validity & 4)
        self.assertIs(self.client.wait_touch(.01, after_sample=old.sample_ref), newer)


class EndEffectorSequenceTest(unittest.TestCase):
    def test_six_legacy_use_cases_keep_shared_keys_body_fields_returns_and_cancel(self):
        cases = {
            'single_target': (False, False, False, False),
            'single_round_trip': (False, False, True, False),
            'zero_single_round_trip': (True, False, True, False),
            'dual_sine_measured_hold': (False, True, False, False),
            'dual_sine_zero_hold': (True, True, False, False),
            'single_inhibited': (False, False, False, True),
        }
        for name, (zero, sine, back, reject) in cases.items():
            with self.subTest(case=name):
                clock = FakeClock()
                c = RecordingController(clock)
                self.addCleanup(c.close)
                reply = robot_info_reply()
                c.transport.get_robot_info = lambda timeout: replace(
                    reply, runtime_id=100, info=replace(reply.info, body=body_info()))
                endpoints = [Endpoint(c, side, inhibited=reject) for side in
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
                    native_hz={e.side:250 for e in endpoints}, native_config='synthetic-test-effective-config',
                    return_to_start=back, pattern='sine' if sine else 'target',
                    period=1., sine_min=.015625, sine_max=.078125)
                targets = {e.side: demo.position_target(e.info, (.09375,), unit='m', torque=(0.,))
                           for e in endpoints}
                if reject:
                    with self.assertRaisesRegex(RuntimeError, 'inhibited|permission'):
                        demo.run(c, endpoints, targets, **options)
                else:
                    demo.run(c, endpoints, targets, **options)
                body = [value for topic, value in c.transport.published if topic == BODY_CMD_TOPIC]
                self.assertTrue(body)
                keys = [CommandKey(value.session_id, value.sequence) for value in body]
                for command in body:
                    self.assertEqual(command.protocol_version, 3)
                    self.assertEqual(len(command.joints), 31)
                    self.assertFalse(hasattr(command, 'cmd_id'))
                for e in endpoints:
                    self.assertTrue(e.commands)
                    keys.extend(command.key for command in e.commands)
                    for command in e.commands:
                        self.assertEqual((command.runtime_id, command.layout_id, command.side),
                                         (e.info.runtime_id, e.info.layout_id, e.info.side))
                        self.assertEqual(command.control_id, 13)
                        self.assertEqual(command.target.tau, (0.,))
                        self.assertTrue(command.target.command_fields[0] & Field.TAU)
                        self.assertEqual(command.target.vel_limit, (0.,))
                        self.assertGreaterEqual(command.target.pos[0], e.info.pos_min[0])
                        self.assertLessEqual(command.target.pos[0], e.info.pos_max[0])
                        self.assertEqual(len(command.input_observations), 1)
                        sample = command.input_observations[0]
                        self.assertEqual((sample.runtime_id, sample.stream, sample.side),
                                         (e.info.runtime_id, SampleStream.EE_STATE, e.info.side))
                        self.assertEqual((command.ts_client_created_us, command.ts_client_send_us), (0, 0))
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

    def test_public_upgrade_has_no_legacy_online_or_info_types(self):
        for name in ('HandClient', 'HandCmd', 'HandState', 'HandTouch', 'HandJointCmd',
                     'HandJointState', 'FingerTouchMeta', 'HAND_TOPIC_PREFIX', 'GripperClient',
                     'GripperCmd', 'GripperState', 'GripperSample', 'GripperHostTarget', 'GRIPPER_TOPIC_PREFIX'):
            with self.subTest(symbol=name):
                self.assertFalse(hasattr(sdk, name))
                self.assertNotIn(name, sdk.__all__)
        for name in ('hand', '_hand_codec', 'gripper', '_gripper_codec', '_gripper_lifecycle'):
            with self.subTest(module=name):
                self.assertIsNone(importlib.util.find_spec('locomotion_aorta.' + name))
        for name in ('HandInfo', 'GripperInfo', 'EndEffectorKind', 'GripperLimitSource'):
            self.assertFalse(hasattr(sdk, name))
            self.assertNotIn(name, sdk.__all__)
        for name in ('HandSide', 'HandType', 'InhibitReason', 'EndEffectorClient'):
            self.assertIn(name, sdk.__all__)
            self.assertIsNotNone(getattr(sdk, name))
        # GripperType is retired from the final public schema closure.
        self.assertNotIn('GripperType', sdk.__all__)
        self.assertFalse(hasattr(sdk, 'GripperType'))
        root = Path(__file__).resolve().parents[1]
        for path in ('brainco_hand/brainco_hand_external.py', 'gripper/gripper_external.py'):
            self.assertFalse((root/'examples'/path).exists())
