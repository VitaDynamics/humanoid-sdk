from body_fixtures import sample_motor, robot_info_reply
import math
from enum import IntEnum
import threading
import unittest
import uuid
from dataclasses import replace
from types import SimpleNamespace
from unittest import mock

import locomotion_aorta.transport as transport_module
import locomotion_aorta.controller as controller_module
from fake_transport import FakeTransport

from locomotion_aorta import (
    AortaBindingsUnavailableError,
    CONTROL_ACK_TOPIC,
    CONTROL_REQUEST_TOPIC,
    CONTROL_STATUS_TOPIC,
    BODY_CMD_TOPIC,
    BODY_STATE_TOPIC,
    ControlAck,
    ExternalControlError,
    ControlOperation,
    ControlRejectedError,
    ControlRequest,
    ControlStatus,
    ControllerClosedError,
    ExternalControlTimeoutError,
    ExternalController,
    ExternalPhase,
    BodyState,
    InvalidCommandError,
    BodyCmd,
    BodyJointCmd,
    NotRunningError,
)
from locomotion_aorta._policy_state import ASAMPLE_BODY_JOINT_NAMES


_BODY_TEST_TARGETS = tuple('fake_joint_' + str(i) for i in range(31))

class ControllerHarness:
    def __init__(self, runtime_id: int = 11, *, info_reply=None) -> None:
        self.transport = FakeTransport()
        if info_reply is None:
            # Synthetic all-targets-proven fixture for generic BodyCmd positive tests.
            reply = robot_info_reply()
            body = reply.info.body
            body = replace(body,
                joints=tuple(replace(j, is_mock=False, supported_fields=31, valid_limits=31,
                                     q_min=-100., q_max=100., dq_min=-100., dq_max=100.,
                                     tau_min=-100., tau_max=100., kp_min=-100., kp_max=100.,
                                     kd_min=-100., kd_max=100.)
                             for j in body.joints),
                motor_identities=tuple(replace(m, present=True) for m in body.motor_identities))
            reply = replace(reply, runtime_id=runtime_id, info=replace(reply.info, body=body))
        else:
            reply = info_reply
        if not reply.runtime_id:
            raise ValueError('test service response has no runtime identity')
        self.runtime_id = reply.runtime_id
        self.transport.get_robot_info = lambda timeout: reply
        self.controller = ExternalController(
            client_id=7, _transport=self.transport
        )
        self.controller.get_robot_info(.1)
        self.control_id = 13
        self.status_sequence = 0
        self.generation = 42

    def ack(self, request, *, accepted: bool = True, reason: str = "") -> None:
        self.transport.emit(
            CONTROL_ACK_TOPIC,
            ControlAck(
                request_id=request.request_id,
                operation=request.operation,
                accepted=accepted,
                control_id=self.control_id,
                client_id=9,
                runtime_id=self.runtime_id,
                reason=reason,
            ),
        )

    def status(
        self,
        *,
        action: str,
        phase: ExternalPhase,
        generation: int | None = None,
    ) -> ControlStatus:
        self.status_sequence += 1
        status = ControlStatus(
            status_seq=self.status_sequence,
            current_action=action,
            phase=phase,
            client_id=9,
            control_id=self.control_id,
            runtime_id=self.runtime_id,
            generation=self.generation if generation is None else generation,
        )
        self.transport.emit(CONTROL_STATUS_TOPIC, status)
        return status

    def accept_enter(self, *, publish_running: bool = True, action: str = "EXTERNAL") -> None:
        def on_publish(topic, request) -> None:
            if topic != CONTROL_REQUEST_TOPIC:
                return
            if request.operation == ControlOperation.PREPARE:
                self.ack(request)
            elif request.operation == ControlOperation.COMMIT:
                self.ack(request)
                if publish_running:
                    self.status(action=action, phase=ExternalPhase.RUNNING)

        self.transport.on_publish = on_publish


def make_command(sequence_hint: int, *, q: float = 0.0) -> BodyCmd:
    return BodyCmd(
        sequence=sequence_hint, joints=tuple(BodyJointCmd(q=q) for _ in range(31)),
    )


class ExternalControllerTest(unittest.TestCase):
    def test_generation_accepts_integer_subclasses_but_wire_identity_is_exact_int(self):
        class IntegerSubclass(int):
            pass

        class NumericEnum(IntEnum):
            VALUE = 1

        for value in (IntegerSubclass(1), NumericEnum.VALUE):
            with self.subTest(value=value):
                self.assertTrue(controller_module._valid_generation(value))
                with self.assertRaisesRegex(ValueError, 'robot_runtime_id must be a nonzero uint64'):
                    controller_module._uint64_nonzero(value, 'robot_runtime_id')
        for value in (False, True, 0, -1, 2**64):
            self.assertFalse(controller_module._valid_generation(value))

    def test_body3_preserves_selected_callback_state_after_newer_sample(self):
        import flatbuffers
        from test_body_codec import module, make_state, encode
        from locomotion_aorta._body_codec import decode_body_state
        h = ControllerHarness()
        self.addCleanup(h.controller.close)
        h.accept_enter()
        h.controller.enter_external(.1)
        def sample(state_id, stamp):
            obj = module('BodyState').BodyStateT.InitFromObj(make_state(runtime_id=11)[1])
            obj.stateId, obj.tsStateRealUs = state_id, stamp
            builder = flatbuffers.Builder(1)
            builder.Finish(obj.Pack(builder))
            return decode_body_state(module('BodyState').BodyState.GetRootAs(builder.Output()))
        selected = sample(100, 1000000)
        h.controller._on_body_state(selected)
        command = replace(make_command(0), input_state_id=selected.state_id,
                          input_state_ts_real_us=selected.ts_state_real_us,
                          runtime_id=selected.runtime_id)
        h.controller._on_body_state(sample(102, 1002000))
        sent = h.controller.publish(command, target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        wire = module('BodyCmd').BodyCmd.GetRootAs(encode(sent))
        self.assertEqual((wire.InputStateId(), wire.InputStateTsRealUs()), (100, 1000000))
        self.assertEqual((wire.RuntimeId(), wire.ControlId()), (11, 13))
        self.assertEqual(wire.JointsLength(), 31)

    def test_body3_uses_confirmed_runtime_control_and_preserves_selected_input(self):
        h = ControllerHarness(runtime_id=2**64 - 1)
        self.addCleanup(h.controller.close)
        h.accept_enter()
        h.controller.enter_external(.1)
        command = replace(make_command(99), input_state_id=100,
                          input_state_ts_real_us=123456,
                          control_id=77)
        sent = h.controller.publish(command, target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        self.assertEqual((sent.protocol_version, sent.runtime_id,
                          sent.control_id, sent.session_id, sent.sequence),
                         (3, 2**64 - 1, 13, 42, 1))
        self.assertEqual((sent.input_state_id, sent.input_state_ts_real_us), (100, 123456))
        self.assertEqual(h.transport.published[-1], (BODY_CMD_TOPIC, sent))

    def test_body_rejects_explicit_old_runtime_without_allocating(self):
        h = ControllerHarness(runtime_id=22)
        self.addCleanup(h.controller.close)
        h.accept_enter()
        h.controller.enter_external(.1)
        for ref in (11, 0, True):
            with self.subTest(ref=ref), self.assertRaises(InvalidCommandError):
                h.controller.publish(replace(make_command(0), runtime_id=ref,
                    input_state_id=100, input_state_ts_real_us=123456), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        self.assertEqual(h.controller._next_external_sequence, 1)
        self.assertFalse(any(topic == BODY_CMD_TOPIC for topic, _ in h.transport.published))
        sent = h.controller.publish(replace(make_command(0), runtime_id=22), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        self.assertEqual((sent.runtime_id, sent.sequence), (22, 1))

    def test_body_submission_after_close_or_owner_change_is_rejected(self):
        for transition in ('close', 'owner', 'runtime', 'body_info'):
            with self.subTest(transition=transition):
                h = ControllerHarness()
                self.addCleanup(h.controller.close)
                h.accept_enter()
                h.controller.enter_external(.1)
                entered, release = threading.Event(), threading.Event()
                original = h.transport._publish_body
                errors = []
                def delayed(message, guard):
                    entered.set()
                    if not release.wait(2):
                        raise TimeoutError('test gate not released')
                    return original(message, guard)
                h.transport._publish_body = delayed
                def publish():
                    try:
                        h.controller.publish(make_command(0),
                            target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
                    except Exception as error:
                        errors.append(error)
                worker = threading.Thread(target=publish)
                worker.start()
                try:
                    self.assertTrue(entered.wait(2))
                    if transition == 'close':
                        h.controller.close()
                    elif transition == 'owner':
                        h.status(action='EXTERNAL', phase=ExternalPhase.IDLE)
                    else:
                        reply = h.transport.get_robot_info(.1)
                        if transition == 'runtime':
                            reply = replace(reply, runtime_id=12)
                        else:
                            reply = replace(reply, info=replace(reply.info, body=replace(reply.info.body, roster_version=reply.info.body.roster_version + 1)))
                        h.transport.get_robot_info = lambda _: reply
                        with self.assertRaisesRegex(ValueError, 'new connection'):
                            h.controller.get_robot_info(.1)
                finally:
                    release.set()
                    worker.join(2)
                self.assertFalse(worker.is_alive())
                self.assertEqual(len(errors), 1)
                self.assertIsInstance(errors[0], (ControllerClosedError, NotRunningError, ValueError))
                self.assertFalse(any(topic == BODY_CMD_TOPIC for topic, _ in h.transport.published))

    def test_bound_action_accepts_both_ack_status_orders_without_early_permission(self):
        for action in ("EXTERNAL", "HUMANOID_TELEOP"):
            for status_first in (False, True):
                with self.subTest(action=action, status_first=status_first):
                    h = ControllerHarness()
                    self.addCleanup(h.controller.close)

                    def publish(topic, request):
                        if topic != CONTROL_REQUEST_TOPIC:
                            return
                        self.assertEqual(request.action_name, action)
                        if request.operation == ControlOperation.COMMIT:
                            if not status_first:
                                h.ack(request)
                            h.status(action=action, phase=ExternalPhase.RUNNING)
                            for operation in (h.controller._reserve_command,
                                              h.controller._control_context):
                                with self.assertRaises(NotRunningError):
                                    operation()
                            if status_first:
                                h.ack(request)
                        else:
                            h.ack(request)
                            if request.operation == ControlOperation.CANCEL:
                                h.status(action="PASSIVE", phase=ExternalPhase.IDLE)

                    h.transport.on_publish = publish
                    active = h.controller.enter_control(action, .1)
                    self.assertEqual((active.current_action, active.generation), (action, 42))
                    self.assertEqual(h.controller._reserve_command().sequence, 1)
                    self.assertEqual(h.controller.exit_control(.1).phase, ExternalPhase.IDLE)
                    requests = [r for t, r in h.transport.published if t == CONTROL_REQUEST_TOPIC]
                    self.assertEqual([r.operation for r in requests], list(ControlOperation))
                    self.assertEqual(len({r.request_id for r in requests}), 1)
                    self.assertTrue(all(r.action_name == action for r in requests))
                    self.assertIsNone(h.controller._request_action)

    def test_action_validation_and_active_request_do_not_rewrite_session(self):
        h = ControllerHarness()
        self.addCleanup(h.controller.close)
        for action in (None, True, "", "TELEOP", "PASSIVE", " HUMANOID_TELEOP", []):
            with self.subTest(action=action), self.assertRaises(ValueError):
                h.controller.enter_control(action, .1)
        self.assertEqual(h.transport.published, [])
        self.assertIsNone(h.controller._request_id)
        h.accept_enter(action="HUMANOID_TELEOP")
        h.controller.enter_control("HUMANOID_TELEOP", .1)
        identity = (h.controller._request_id, h.controller._control_id)
        before = list(h.transport.published)
        for action in ("EXTERNAL", "HUMANOID_TELEOP"):
            with self.assertRaises(ExternalControlError):
                h.controller.enter_control(action, .1)
        self.assertEqual(h.transport.published, before)
        self.assertEqual((h.controller._request_id, h.controller._control_id), identity)
        self.assertEqual(h.controller._request_action, "HUMANOID_TELEOP")

    def test_teleop_entry_requires_fresh_matching_running_not_only_commit_ack(self):
        for change in (None, {"current_action": "EXTERNAL"}, {"client_id": 17},
                       {"control_id": 19}, {"generation": 0},
                       {"phase": ExternalPhase.TIMEOUT_HOLD}, {"status_seq": 1}):
            with self.subTest(change=change):
                h = ControllerHarness()
                self.addCleanup(h.controller.close)
                previous = h.status(action="HUMANOID_TELEOP", phase=ExternalPhase.RUNNING)

                def publish(topic, request):
                    if topic == CONTROL_REQUEST_TOPIC:
                        h.ack(request)
                        if request.operation == ControlOperation.COMMIT and change is not None:
                            h.transport.emit(CONTROL_STATUS_TOPIC,
                                             replace(previous, **({"status_seq": 2} | change)))

                h.transport.on_publish = publish
                with self.assertRaises(ExternalControlTimeoutError):
                    h.controller.enter_control("HUMANOID_TELEOP", .01)
                self.assertEqual(h.controller._external_session_id, 0)
                self.assertEqual(h.controller._next_external_sequence, 1)
                self.assertEqual([r.operation for t, r in h.transport.published],
                                 [ControlOperation.PREPARE, ControlOperation.COMMIT, ControlOperation.CANCEL])

    def test_running_before_rejected_commit_does_not_activate(self):
        h = ControllerHarness()
        self.addCleanup(h.controller.close)

        def publish(topic, request):
            if topic != CONTROL_REQUEST_TOPIC:
                return
            if request.operation == ControlOperation.COMMIT:
                h.status(action="HUMANOID_TELEOP", phase=ExternalPhase.RUNNING)
                h.ack(request, accepted=False, reason="activation refused")
            else:
                h.ack(request)

        h.transport.on_publish = publish
        with self.assertRaisesRegex(ControlRejectedError, "activation refused"):
            h.controller.enter_control("HUMANOID_TELEOP", .1)
        self.assertEqual(h.controller._external_session_id, 0)
        with self.assertRaises(NotRunningError):
            h.controller._reserve_command()

    def test_cancel_or_close_before_activation_cannot_reopen_permission(self):
        for close in (False, True):
            with self.subTest(close=close):
                h = ControllerHarness()
                self.addCleanup(h.controller.close)
                h.accept_enter(action="HUMANOID_TELEOP")
                original_wait = h.controller._wait_for

                def wait(value, deadline, message):
                    if message.endswith("/RUNNING status"):
                        if close:
                            h.controller.close()
                        else:
                            h.controller._best_effort_cancel()
                    return original_wait(value, deadline, message)

                with mock.patch.object(h.controller, "_wait_for", side_effect=wait):
                    with self.assertRaises(ControllerClosedError if close else NotRunningError):
                        h.controller.enter_control("HUMANOID_TELEOP", .1)
                self.assertEqual(h.controller._external_session_id, 0)
                self.assertEqual(h.controller._next_external_sequence, 1)
                self.assertEqual(sum(r.operation == ControlOperation.CANCEL
                                     for _, r in h.transport.published), 1)

    def test_cancelled_enter_cannot_cancel_a_replacement_request(self):
        h = ControllerHarness()
        self.addCleanup(h.controller.close)
        waiting, resume = threading.Event(), threading.Event()
        failures = []
        original_wait = h.controller._wait_for

        def wait(value, deadline, message):
            if message == "timed out waiting for owned HUMANOID_TELEOP/RUNNING status":
                waiting.set()
                if not resume.wait(1):
                    raise AssertionError("test did not release old entry")
            return original_wait(value, deadline, message)

        def publish(topic, request):
            if topic == CONTROL_REQUEST_TOPIC:
                h.ack(request)
                if request.operation in (ControlOperation.COMMIT, ControlOperation.CANCEL):
                    cancel = request.operation == ControlOperation.CANCEL
                    h.status(action="PASSIVE" if cancel else request.action_name,
                             phase=ExternalPhase.IDLE if cancel else ExternalPhase.RUNNING)

        def enter():
            try:
                h.controller.enter_control("HUMANOID_TELEOP", 1)
            except BaseException as error:
                failures.append(error)

        h.transport.on_publish = publish
        with mock.patch.object(h.controller, "_wait_for", side_effect=wait):
            thread = threading.Thread(target=enter)
            thread.start()
            try:
                self.assertTrue(waiting.wait(.5))
                h.controller.exit_control(.1)
                active = h.controller.enter_external(.1)
                new_request = h.controller._request_id
            finally:
                resume.set()
                thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(failures), 1)
        self.assertIsInstance(failures[0], NotRunningError)
        self.assertEqual(h.controller._request_id, new_request)
        self.assertFalse(h.controller._cancel_sent)
        self.assertEqual(h.controller._control_context().generation, active.generation)
        self.assertEqual(sum(r.operation == ControlOperation.CANCEL
                             for _, r in h.transport.published), 1)

    def test_teleop_body_rejection_precedes_generation_learning_and_key_allocation(self):
        h = ControllerHarness()
        self.addCleanup(h.controller.close)
        h.accept_enter(action="HUMANOID_TELEOP")
        h.controller.enter_control("HUMANOID_TELEOP", .1)
        first = h.controller._reserve_command()
        h.status(action="HUMANOID_TELEOP", phase=ExternalPhase.RUNNING, generation=84)
        before = list(h.transport.published)
        with self.assertRaisesRegex(NotRunningError, "BodyCmd"):
            h.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        self.assertEqual(h.transport.published, before)
        self.assertEqual((h.controller._external_session_id, h.controller._next_external_sequence), (42, 2))
        second = h.controller._reserve_command()
        self.assertEqual((first.session_id, first.sequence, second.session_id, second.sequence), (42, 1, 84, 1))
        h.controller._best_effort_cancel()
        for operation in (h.controller._reserve_command, h.controller._control_context):
            with self.assertRaises(NotRunningError):
                operation()
        self.assertEqual(h.controller._next_external_sequence, 2)

    def test_teleop_exit_requires_leaving_bound_action_and_completed_cleanup(self):
        h = ControllerHarness()
        self.addCleanup(h.controller.close)
        h.accept_enter(action="HUMANOID_TELEOP")
        h.controller.enter_control("HUMANOID_TELEOP", .1)
        original_wait = h.controller._wait_for
        checked = []

        def publish(topic, request):
            if topic == CONTROL_REQUEST_TOPIC:
                h.ack(request)

        def wait(value, deadline, message):
            if message == "timed out waiting for HUMANOID_TELEOP to exit":
                with h.controller._condition:
                    for action, phase in (("HUMANOID_TELEOP", ExternalPhase.IDLE),
                                          ("PASSIVE", ExternalPhase.CANCELLED)):
                        h.status(action=action, phase=phase)
                        self.assertIsNone(value())
                        checked.append((action, phase))
                h.status(action="PASSIVE", phase=ExternalPhase.IDLE)
            return original_wait(value, deadline, message)

        h.transport.on_publish = publish
        with mock.patch.object(h.controller, "_wait_for", side_effect=wait):
            exited = h.controller.exit_control(.1)
        self.assertEqual(len(checked), 2)
        self.assertEqual(exited.phase, ExternalPhase.IDLE)
        self.assertIsNone(h.controller._request_action)
        self.assertEqual(h.controller._external_session_id, 0)
        self.assertEqual(h.transport.published[-1][1].action_name, "HUMANOID_TELEOP")

    def test_rpc_control_context_is_frozen_nonconsuming_and_does_not_learn_generation(self):
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter(); harness.controller.enter_external(.1)
        first = harness.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        before = list(harness.transport.published)
        context = harness.controller._control_context()
        self.assertEqual((context.client_id, context.control_id, context.generation, context.request_id),
                         (9, harness.control_id, 42, harness.controller._request_id))
        with self.assertRaises(AttributeError): context.control_id = 19
        self.assertEqual(harness.transport.published, before)
        harness.status(action='EXTERNAL', phase=ExternalPhase.RUNNING, generation=84)
        self.assertEqual(harness.controller._control_context().generation, 84)
        self.assertEqual((harness.controller._external_session_id, harness.controller._next_external_sequence), (42, 2))
        self.assertEqual(harness.transport.published, before)
        second = harness.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        self.assertEqual((first.session_id, first.sequence, second.session_id, second.sequence), (42, 1, 84, 1))
        harness.controller._next_external_sequence = 1 << 64
        self.assertEqual(harness.controller._control_context().generation, 84)
        with self.assertRaises(ExternalControlError): harness.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)

    def test_rpc_control_context_reuses_owned_status_guards_and_rejects_local_cancel(self):
        for phase, changed in ((ExternalPhase.RUNNING, {'client_id': 17}),
                               (ExternalPhase.TIMEOUT_HOLD, {'control_id': 19}),
                               (ExternalPhase.RUNNING, {'generation': 0}),
                               (ExternalPhase.TIMEOUT_HOLD, {'generation': 84})):
            with self.subTest(phase=phase, changed=changed):
                harness = ControllerHarness(); self.addCleanup(harness.controller.close)
                harness.accept_enter(); owned = harness.controller.enter_external(.1)
                harness.transport.emit(CONTROL_STATUS_TOPIC, replace(owned, status_seq=2, phase=phase, **changed))
                before = list(harness.transport.published)
                with self.assertRaises(NotRunningError): harness.controller._control_context()
                with self.assertRaises(NotRunningError): harness.controller._reserve_command()
                self.assertEqual(harness.controller._next_external_sequence, 1)
                self.assertEqual(harness.transport.published, before)
        harness = ControllerHarness(); self.addCleanup(harness.controller.close)
        with self.assertRaises(NotRunningError): harness.controller._control_context()
        harness.accept_enter(); harness.controller.enter_external(.1)
        harness.controller._best_effort_cancel()
        before = list(harness.transport.published)
        with self.assertRaises(NotRunningError): harness.controller._control_context()
        self.assertEqual(harness.transport.published, before)
        harness.controller.close()
        with self.assertRaises(ControllerClosedError): harness.controller._control_context()

    def test_foreign_owner_or_lease_cannot_publish_or_consume_key(self):
        for phase in (ExternalPhase.RUNNING, ExternalPhase.TIMEOUT_HOLD):
            for foreign in ({"client_id": 17},
                            {"control_id": 19}):
                with self.subTest(phase=phase, foreign=foreign):
                    harness = ControllerHarness()
                    self.addCleanup(harness.controller.close)
                    harness.accept_enter()
                    owned = harness.controller.enter_external(timeout=0.1)
                    first = harness.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
                    requests = [message for topic, message in harness.transport.published
                                if topic == CONTROL_REQUEST_TOPIC]

                    harness.transport.emit(CONTROL_STATUS_TOPIC, replace(
                        owned, status_seq=2, phase=phase,
                        generation=84 if phase == ExternalPhase.RUNNING else 42,
                        **foreign))
                    with self.assertRaises(NotRunningError):
                        harness.controller._reserve_command()
                    with self.assertRaises(NotRunningError):
                        harness.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
                    self.assertEqual(
                        [message for topic, message in harness.transport.published
                         if topic == BODY_CMD_TOPIC], [first])
                    self.assertEqual(
                        [message for topic, message in harness.transport.published
                         if topic == CONTROL_REQUEST_TOPIC], requests)

                    harness.transport.emit(
                        CONTROL_STATUS_TOPIC, replace(owned, status_seq=3))
                    resumed = harness.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
                    self.assertEqual((resumed.session_id, resumed.sequence), (42, 2))

    def test_invalid_running_generation_does_not_reset_or_consume_key(self):
        for generation in (0, -1, True, False, 42.0, "42", 1 << 64):
            with self.subTest(generation=generation):
                harness = ControllerHarness()
                self.addCleanup(harness.controller.close)
                harness.accept_enter()
                owned = harness.controller.enter_external(timeout=0.1)
                first = harness.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
                harness.transport.emit(CONTROL_STATUS_TOPIC, replace(
                    owned, status_seq=2, generation=generation))

                with self.assertRaises(NotRunningError):
                    harness.controller._reserve_command()
                with self.assertRaises(NotRunningError):
                    harness.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
                self.assertEqual(
                    [message for topic, message in harness.transport.published
                     if topic == BODY_CMD_TOPIC], [first])

                harness.transport.emit(
                    CONTROL_STATUS_TOPIC, replace(owned, status_seq=3))
                resumed = harness.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
                self.assertEqual((resumed.session_id, resumed.sequence), (42, 2))

    def test_closed_controller_cannot_reserve_publish_or_accept_late_status(self):
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        owned = harness.controller.enter_external(timeout=0.1)
        harness.controller.close()
        sent_at_close = list(harness.transport.published)

        harness.transport.emit(CONTROL_STATUS_TOPIC, replace(
            owned, status_seq=2, generation=84))
        with self.assertRaises(ControllerClosedError):
            harness.controller._reserve_command()
        with self.assertRaises(ControllerClosedError):
            harness.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)

        self.assertIs(harness.controller.status, owned)
        self.assertEqual(harness.transport.published, sent_at_close)
        self.assertEqual(harness.transport.close_count, 1)

    def test_concurrent_reservations_and_body_publish_assign_unique_keys(self):
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)
        start = threading.Barrier(2, timeout=1.0)
        results = [[], []]
        errors = []

        def allocate(index):
            try:
                for _ in range(8):
                    start.wait()
                    command = (harness.controller.publish(make_command(999), target_names=_BODY_TEST_TARGETS, required_fields=0x1f) if index == 0
                               else harness.controller._reserve_command())
                    results[index].append((command.session_id, command.sequence))
            except BaseException as error:
                errors.append(error)
                start.abort()

        threads = [threading.Thread(target=allocate, args=(i,), daemon=True) for i in range(2)]
        try:
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join(2.0)
        finally:
            start.abort()
        self.assertFalse(any(thread.is_alive() for thread in threads))
        self.assertEqual(errors, [])
        self.assertEqual(sorted(results[0] + results[1]), [(42, i) for i in range(1, 17)])
        for keys in results:
            self.assertEqual(keys, sorted(keys))
        self.assertEqual(
            [(m.session_id, m.sequence) for topic, m in harness.transport.published
             if topic == BODY_CMD_TOPIC], results[0])

    def test_connection_reservations_share_the_body_command_stream(self):
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)

        first = harness.controller._reserve_command()
        body = harness.controller.publish(make_command(999), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        last = harness.controller._reserve_command()

        self.assertEqual(
            [(value.session_id, value.sequence) for value in (first, body, last)],
            [(42, 1), (42, 2), (42, 3)],
        )
        self.assertEqual((first.client_id, first.control_id),
                         (harness.controller.client_id, harness.control_id))
        self.assertEqual([message.operation for topic, message in harness.transport.published
                          if topic == CONTROL_REQUEST_TOPIC],
                         [ControlOperation.PREPARE, ControlOperation.COMMIT])

    def test_failed_body_send_does_not_reuse_a_connection_key(self):
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)
        attempted = []

        def fail_body(topic, message):
            if topic == BODY_CMD_TOPIC:
                attempted.append(message)
                raise OSError('transport send failed')

        harness.transport.on_publish = fail_body
        with self.assertRaisesRegex(OSError, 'transport send failed'):
            harness.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        reserved = harness.controller._reserve_command()
        self.assertEqual((attempted[0].session_id, attempted[0].sequence), (42, 1))
        self.assertEqual((reserved.session_id, reserved.sequence), (42, 2))

    def test_connection_reservation_rechecks_owner_and_generation(self):
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)
        first = harness.controller._reserve_command()
        harness.status(action='EXTERNAL', phase=ExternalPhase.TIMEOUT_HOLD, generation=84)
        with self.assertRaises(NotRunningError):
            harness.controller._reserve_command()
        harness.status(action='EXTERNAL', phase=ExternalPhase.RUNNING, generation=42)
        unchanged = harness.controller._reserve_command()
        self.assertEqual((unchanged.session_id, unchanged.sequence), (42, 2))
        harness.status(action='EXTERNAL', phase=ExternalPhase.RUNNING, generation=84)
        second = harness.controller._reserve_command()
        self.assertEqual((first.session_id, first.sequence), (42, 1))
        self.assertEqual((second.session_id, second.sequence), (84, 1))
        # Previously returned context cannot establish ownership after revocation.
        harness.status(action='PASSIVE', phase=ExternalPhase.IDLE)
        with self.assertRaises(NotRunningError):
            harness.controller._reserve_command()
        harness.status(action='EXTERNAL', phase=ExternalPhase.RUNNING, generation=84)
        resumed = harness.controller._reserve_command()
        self.assertEqual((resumed.session_id, resumed.sequence), (84, 2))

    def test_default_transport_fails_closed_until_aorta_bindings_exist(
        self,
    ) -> None:
        with mock.patch.object(
            transport_module.importlib, "import_module", side_effect=ImportError("missing")
        ), self.assertRaisesRegex(
            AortaBindingsUnavailableError, "M0 binding gate"
        ):
            ExternalController()

    def test_aorta_transport_uses_typed_endpoints_and_matching_qos(self) -> None:
        class FakeQoS:
            realtime_control = staticmethod(lambda: "realtime")
            sensor_data = staticmethod(lambda: "sensor")
            state_update = staticmethod(lambda: "state")

        class FakePublisher:
            def __init__(self) -> None:
                self.fills = []
                self.closed = False

            def publish_typed(self, fill) -> None:
                self.fills.append(fill)

            def close(self):
                self.closed = True

        class FakeNode:
            def __init__(self, name, group) -> None:
                self.name = name
                self._group = group
                self.publisher_calls = []
                self.subscriber_calls = []
                self.close_count = 0

            def group(self):
                return self._group

            def create_publisher_typed(self, schema_meta, topic, *, qos):
                publisher = FakePublisher()
                self.publisher_calls.append((schema_meta, topic, qos, publisher))
                return publisher

            def create_subscriber_typed(
                self, decoder, topic, handler, *, qos
            ):
                subscription = SimpleNamespace(close=mock.Mock())
                self.subscriber_calls.append(
                    (decoder, topic, handler, qos, subscription)
                )
                return subscription

            def close(self) -> None:
                self.close_count += 1

        fake_aorta = SimpleNamespace(Node=FakeNode, QoS=FakeQoS)
        fake_bindings = SimpleNamespace(
            aorta=fake_aorta,
            control_request_schema_meta=object(),
            body_cmd_schema_meta=object(),
            body_state_decoder=object(),
            control_ack_decoder=object(),
            control_status_decoder=object(),
        )

        with mock.patch.object(
            transport_module,
            "_load_aorta_bindings",
            return_value=fake_bindings,
        ), mock.patch('locomotion_aorta._native_body_codec.NativeBodyPublisher') as native:
            transport = transport_module.create_aorta_transport(
                node_name="sdk-node", group="robot-domain"
            )
            native.assert_not_called()
            self.assertEqual(list(transport.publishers), [CONTROL_REQUEST_TOPIC])
            transport._prepare_body_output()
            native.assert_called_once()

        self.assertEqual((transport.node.name, transport.node.group()), ("sdk-node", "robot-domain"))
        self.assertEqual(
            [
                (schema, topic, qos)
                for schema, topic, qos, _ in transport.node.publisher_calls
            ],
            [
                (
                    fake_bindings.control_request_schema_meta,
                    CONTROL_REQUEST_TOPIC,
                    "realtime",
                ),
                (
                    fake_bindings.body_cmd_schema_meta,
                    BODY_CMD_TOPIC,
                    "realtime",
                ),
            ],
        )

        callback = mock.Mock()
        subscription = transport.subscribe(CONTROL_STATUS_TOPIC, callback)
        decoder, topic, _, qos, raw_subscription = transport.node.subscriber_calls[0]
        self.assertIs(decoder, fake_bindings.control_status_decoder)
        self.assertEqual((topic, qos), (CONTROL_STATUS_TOPIC, "state"))
        self.assertIs(subscription, raw_subscription)
        self.assertEqual(transport.subscriptions, [raw_subscription])

        request = ControlRequest(
            request_id="request-1",
            client_id=7,
            operation=ControlOperation.PREPARE,
            runtime_id=11,
        )
        transport._publish_control(request, lambda fill: fill())
        self.assertEqual(
            len(transport.publishers[CONTROL_REQUEST_TOPIC].fills), 1
        )
        transport.close()
        transport.close()
        self.assertEqual(transport.node.close_count, 1)

    def test_generated_control_views_decode_to_public_values(self) -> None:
        ack = transport_module._decode_control_ack(
            SimpleNamespace(
                ProtocolVersion=lambda: 4,
                Operation=lambda: 2,
                RequestId=lambda: b"request-1",
                Accepted=lambda: True,
                ErrorCode=lambda: 0,
                Reason=lambda: b"",
                ClientId=lambda: 9,
                ControlId=lambda: 13,
                RuntimeId=lambda: 11,
            )
        )
        status = transport_module._decode_control_status(
            SimpleNamespace(
                ProtocolVersion=lambda: 4,
                StatusSeq=lambda: 9,
                CurrentAction=lambda: b"EXTERNAL",
                ActionPhase=lambda: 4,
                ClientId=lambda: 9,
                ControlId=lambda: 13,
                RuntimeId=lambda: 11,
                Generation=lambda: 42,
                LastAcceptedCmdId=lambda: 8,
                LastValidAgeMs=lambda: 10,
                EstimatedInputPeriodUs=lambda: 2000,
                TimeoutHold=lambda: False,
                LastRejectCode=lambda: 0,
                RejectCount=lambda: 1,
                IngressDropCount=lambda: 2,
                StampMonotonicNs=lambda: 456,
            )
        )

        self.assertEqual(ack.operation, ControlOperation.COMMIT)
        self.assertEqual((ack.client_id, ack.control_id, ack.runtime_id), (9, 13, 11))
        self.assertEqual(status.phase, ExternalPhase.RUNNING)
        self.assertEqual((status.client_id, status.control_id, status.runtime_id), (9, 13, 11))
        self.assertEqual((status.generation, status.last_accepted_cmd_id), (42, 8))

        with self.assertRaisesRegex(ValueError, "ControlActionPhase"):
            transport_module._decode_phase(-1)

    def test_subscribes_to_public_state_topics(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)

        self.assertEqual(
            set(harness.transport.callbacks),
            {BODY_STATE_TOPIC, CONTROL_ACK_TOPIC, CONTROL_STATUS_TOPIC},
        )

    def test_wait_body_state_times_out_then_returns_the_latest_state(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)

        with self.assertRaises(ExternalControlTimeoutError):
            harness.controller.wait_body_state(timeout=0.001)

        first = BodyState(
            state_id=10,
            ts_state_real_us=100,
            joints=(sample_motor(q=0.1, dq=0.2),) * 31,
            runtime_id=11,
        )
        latest = BodyState(
            state_id=11,
            ts_state_real_us=200,
            joints=(sample_motor(q=0.3, dq=0.4),) * 31,
            runtime_id=11,
        )
        harness.transport.emit(BODY_STATE_TOPIC, first)
        harness.transport.emit(BODY_STATE_TOPIC, latest)
        for runtime_id in (0, True, 12):
            harness.transport.emit(BODY_STATE_TOPIC, replace(latest, state_id=12, runtime_id=runtime_id))

        self.assertIs(harness.controller.wait_body_state(timeout=0.01), latest)

    def test_body_state3_keeps_all_source_association_and_cmd_stays2(self):

        harness = ControllerHarness(runtime_id=2**64-1)
        self.addCleanup(harness.controller.close)
        state = BodyState(2**64-1, (sample_motor(),) * 31, runtime_id=2**64-1,
                          ts_control_us=2**64-2, applied_key_valid=True, association_source_type=2)
        harness.transport.emit(BODY_STATE_TOPIC, state)
        self.assertIs(harness.controller.wait_body_state(.01), state)
        self.assertEqual((state.applied_session_id, state.applied_sequence, state.ts_first_output_us), (0, 0, 0))
        for protocol in (0, 1, 2, 4, 0xffffffff):
            harness.transport.emit(BODY_STATE_TOPIC, replace(state, protocol_version=protocol))
            self.assertIs(harness.controller.wait_body_state(.01), state)
        for joints in ((), state.joints[:30], state.joints + state.joints[:1]):
            harness.transport.emit(BODY_STATE_TOPIC, replace(state, joints=joints))
            self.assertIs(harness.controller.wait_body_state(.01), state)
        harness.accept_enter()
        harness.controller.enter_external(timeout=.1)
        self.assertEqual(harness.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f).protocol_version, 3)

    def test_status_cache_does_not_regress_or_replace_a_sequence(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        current = ControlStatus(runtime_id=11,
            status_seq=2,
            current_action="EXTERNAL",
            phase=ExternalPhase.RUNNING,
        )
        duplicate = ControlStatus(runtime_id=11,
            status_seq=2,
            current_action="FIXED_STAND",
            phase=ExternalPhase.IDLE,
        )
        stale = ControlStatus(runtime_id=11,
            status_seq=1,
            current_action="FIXED_STAND",
            phase=ExternalPhase.IDLE,
        )

        harness.transport.emit(CONTROL_STATUS_TOPIC, current)
        harness.transport.emit(CONTROL_STATUS_TOPIC, duplicate)
        harness.transport.emit(CONTROL_STATUS_TOPIC, stale)

        self.assertIs(harness.controller.status, current)

    def test_status_cache_accepts_sequence_reset_after_server_restart(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        previous_process = ControlStatus(runtime_id=11,
            status_seq=100,
            stamp_monotonic_ns=1_000,
            current_action="EXTERNAL",
            phase=ExternalPhase.RUNNING,
        )
        restarted_process = ControlStatus(runtime_id=11,
            status_seq=0,
            stamp_monotonic_ns=2_000,
            current_action="FIXED_STAND",
            phase=ExternalPhase.IDLE,
        )
        delayed_previous_status = ControlStatus(runtime_id=11,
            status_seq=99,
            stamp_monotonic_ns=900,
            current_action="EXTERNAL",
            phase=ExternalPhase.RUNNING,
        )

        harness.transport.emit(CONTROL_STATUS_TOPIC, previous_process)
        harness.transport.emit(CONTROL_STATUS_TOPIC, restarted_process)
        harness.transport.emit(CONTROL_STATUS_TOPIC, delayed_previous_status)

        self.assertIs(harness.controller.status, restarted_process)

    def test_enter_uses_one_exact_request_id_and_waits_for_running(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        fixed_request_id = uuid.UUID("00000000-0000-0000-0000-000000000042")

        with mock.patch(
            "locomotion_aorta.controller.uuid.uuid4",
            return_value=fixed_request_id,
        ):
            status = harness.controller.enter_external(timeout=0.1)

        requests = [
            message
            for topic, message in harness.transport.published
            if topic == CONTROL_REQUEST_TOPIC
        ]
        self.assertEqual(
            [request.operation for request in requests],
            [ControlOperation.PREPARE, ControlOperation.COMMIT],
        )
        self.assertEqual(
            {request.request_id for request in requests}, {str(fixed_request_id)}
        )
        self.assertEqual([request.client_id for request in requests], [7, 9])
        self.assertEqual({request.protocol_version for request in requests}, {4})
        self.assertEqual([request.runtime_id for request in requests], [11, 11])
        self.assertEqual(requests[0].control_id, 0)
        self.assertFalse(hasattr(requests[0], "requested_lease_ms"))
        self.assertEqual(requests[1].control_id, harness.control_id)
        self.assertFalse(hasattr(requests[1], "requested_lease_ms"))
        self.assertEqual((status.phase, status.runtime_id), (ExternalPhase.RUNNING, 11))
        self.assertFalse(hasattr(status, "lease_deadline_monotonic_ns"))

    def test_control_requests_always_send_zero_duration(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)
        harness.controller.close()
        requests = [
            message
            for topic, message in harness.transport.published
            if topic == CONTROL_REQUEST_TOPIC
        ]
        self.assertEqual(
            [request.operation for request in requests],
            [ControlOperation.PREPARE, ControlOperation.COMMIT, ControlOperation.CANCEL],
        )
        self.assertTrue(all(not hasattr(request, "requested_lease_ms") for request in requests))

    def test_removed_duration_argument_fails_before_sending(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        for duration in (0, 30):
            with self.subTest(duration=duration):
                with self.assertRaisesRegex(TypeError, "lease_duration_s"):
                    harness.controller.enter_external(
                        timeout=0.1, lease_duration_s=duration
                    )
                self.assertEqual(harness.transport.published, [])
        harness.controller.enter_external(timeout=0.1)
        prepare = harness.transport.published[0][1]
        self.assertFalse(hasattr(prepare, "requested_lease_ms"))

    def test_unlimited_request_rejection_does_not_fall_back_to_finite(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)

        def on_publish(topic, request) -> None:
            if topic == CONTROL_REQUEST_TOPIC:
                harness.ack(
                    request, accepted=False, reason="request denied"
                )

        harness.transport.on_publish = on_publish
        with self.assertRaisesRegex(ControlRejectedError, "request denied"):
            harness.controller.enter_external(timeout=0.1)
        requests = [
            message for topic, message in harness.transport.published
            if topic == CONTROL_REQUEST_TOPIC
        ]
        self.assertEqual(
            [request.operation for request in requests],
            [ControlOperation.PREPARE, ControlOperation.CANCEL],
        )
        self.assertTrue(all(not hasattr(request, "requested_lease_ms") for request in requests))
        with self.assertRaises(NotRunningError):
            harness.controller.publish(make_command(1), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)

    def test_entry_wait_timeout_still_validated_before_sending(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        for timeout in (0, -1, True, None, math.nan, math.inf):
            with self.subTest(timeout=timeout):
                with mock.patch.object(controller_module.time, 'monotonic',
                                       side_effect=AssertionError('invalid timeout read a clock')):
                    with self.assertRaises(ValueError):
                        harness.controller.enter_external(timeout=timeout)
                    with self.assertRaises(ValueError):
                        harness.controller.wait_body_state(timeout)
                    with self.assertRaises(ValueError):
                        harness.controller.get_robot_info(timeout)
                self.assertEqual(harness.transport.published, [])

    def test_enter_accepts_running_status_after_server_sequence_reset(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.transport.emit(
            CONTROL_STATUS_TOPIC,
            ControlStatus(runtime_id=11,
                status_seq=100,
                stamp_monotonic_ns=1_000,
                current_action="FIXED_STAND",
                phase=ExternalPhase.IDLE,
            ),
        )

        def on_publish(topic, request) -> None:
            if topic != CONTROL_REQUEST_TOPIC:
                return
            harness.ack(request)
            if request.operation == ControlOperation.COMMIT:
                harness.transport.emit(
                    CONTROL_STATUS_TOPIC,
                    ControlStatus(runtime_id=11,
                        status_seq=0,
                        stamp_monotonic_ns=2_000,
                        current_action="EXTERNAL",
                        phase=ExternalPhase.RUNNING,
                        client_id=9,
                        control_id=harness.control_id,
                        generation=harness.generation,
                    ),
                )

        harness.transport.on_publish = on_publish

        status = harness.controller.enter_external(timeout=0.1)

        self.assertEqual(status.generation, harness.generation)

    def test_unsupported_ack_version_cannot_advance_prepare(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)

        def on_publish(topic, request) -> None:
            if topic != CONTROL_REQUEST_TOPIC:
                return
            if request.operation == ControlOperation.PREPARE:
                harness.transport.emit(
                    CONTROL_ACK_TOPIC,
                    ControlAck(runtime_id=11, client_id=9,
                        request_id=request.request_id,
                        operation=request.operation,
                        accepted=True,
                        control_id=harness.control_id,
                        protocol_version=2,
                    ),
                )
                harness.transport.emit(
                    CONTROL_ACK_TOPIC,
                    ControlAck(runtime_id=11, client_id=9,
                        request_id=request.request_id,
                        operation=request.operation,
                        accepted=True, control_id=harness.control_id,
                        protocol_version=3,
                    ),
                )
            elif request.operation == ControlOperation.COMMIT:
                harness.ack(request)
                harness.status(
                    action="EXTERNAL", phase=ExternalPhase.RUNNING
                )

        harness.transport.on_publish = on_publish

        with self.assertRaises(ExternalControlTimeoutError):
            harness.controller.enter_external(timeout=0.01)

        operations = [
            message.operation
            for topic, message in harness.transport.published
            if topic == CONTROL_REQUEST_TOPIC
        ]
        self.assertEqual(
            operations, [ControlOperation.PREPARE, ControlOperation.CANCEL]
        )

    def test_unsupported_status_version_cannot_complete_enter(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)

        def on_publish(topic, request) -> None:
            if topic != CONTROL_REQUEST_TOPIC:
                return
            if request.operation in (
                ControlOperation.PREPARE,
                ControlOperation.COMMIT,
            ):
                harness.ack(request)
            if request.operation == ControlOperation.COMMIT:
                harness.transport.emit(
                    CONTROL_STATUS_TOPIC,
                    ControlStatus(runtime_id=11,
                        status_seq=1,
                        current_action="EXTERNAL",
                        phase=ExternalPhase.RUNNING,
                        client_id=9,
                        control_id=harness.control_id,
                        generation=harness.generation,
                        protocol_version=2,
                    ),
                )
                harness.transport.emit(
                    CONTROL_STATUS_TOPIC,
                    ControlStatus(runtime_id=11, status_seq=2,
                        current_action="EXTERNAL", phase=ExternalPhase.RUNNING,
                        client_id=9, control_id=harness.control_id,
                        generation=harness.generation,
                        protocol_version=3),
                )

        harness.transport.on_publish = on_publish

        with self.assertRaises(ExternalControlTimeoutError):
            harness.controller.enter_external(timeout=0.01)

    def test_commit_rejection_raises_and_sends_cancel_in_cleanup(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)

        def on_publish(topic, request) -> None:
            if topic != CONTROL_REQUEST_TOPIC:
                return
            if request.operation == ControlOperation.PREPARE:
                harness.ack(request)
            elif request.operation == ControlOperation.COMMIT:
                harness.ack(request, accepted=False, reason="commit denied")

        harness.transport.on_publish = on_publish

        with self.assertRaisesRegex(ControlRejectedError, "commit denied"):
            harness.controller.enter_external(timeout=0.1)

        operations = [
            message.operation
            for topic, message in harness.transport.published
            if topic == CONTROL_REQUEST_TOPIC
        ]
        self.assertEqual(
            operations,
            [
                ControlOperation.PREPARE,
                ControlOperation.COMMIT,
                ControlOperation.CANCEL,
            ],
        )

    def test_keyboard_interrupt_during_enter_still_sends_cancel(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)

        def on_publish(topic, request) -> None:
            if topic != CONTROL_REQUEST_TOPIC:
                return
            if request.operation == ControlOperation.PREPARE:
                harness.ack(request)
            elif request.operation == ControlOperation.COMMIT:
                raise KeyboardInterrupt()

        harness.transport.on_publish = on_publish

        with self.assertRaises(KeyboardInterrupt):
            harness.controller.enter_external(timeout=0.1)

        operations = [
            message.operation
            for topic, message in harness.transport.published
            if topic == CONTROL_REQUEST_TOPIC
        ]
        self.assertEqual(
            operations,
            [
                ControlOperation.PREPARE,
                ControlOperation.COMMIT,
                ControlOperation.CANCEL,
            ],
        )

    def test_accepted_commit_ack_without_running_status_times_out_and_cancels(
        self,
    ) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter(publish_running=False)

        with self.assertRaises(ExternalControlTimeoutError):
            harness.controller.enter_external(timeout=0.01)

        operations = [
            message.operation
            for topic, message in harness.transport.published
            if topic == CONTROL_REQUEST_TOPIC
        ]
        self.assertEqual(operations[-1], ControlOperation.CANCEL)
        self.assertIsNone(harness.controller.status)

    def test_enter_requires_running_status_newer_than_commit(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.status(action="EXTERNAL", phase=ExternalPhase.RUNNING)
        harness.accept_enter(publish_running=False)

        with self.assertRaises(ExternalControlTimeoutError):
            harness.controller.enter_external(timeout=0.01)

    def test_timeout_hold_status_does_not_complete_initial_enter(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)

        def on_publish(topic, request) -> None:
            if topic != CONTROL_REQUEST_TOPIC:
                return
            if request.operation == ControlOperation.PREPARE:
                harness.ack(request)
            elif request.operation == ControlOperation.COMMIT:
                harness.ack(request)
                harness.status(
                    action="EXTERNAL", phase=ExternalPhase.TIMEOUT_HOLD
                )

        harness.transport.on_publish = on_publish

        with self.assertRaises(ExternalControlTimeoutError):
            harness.controller.enter_external(timeout=0.01)

    def test_publish_requires_owned_running_or_timeout_hold_status(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        first_command = make_command(1, q=0.1)

        with self.assertRaises(NotRunningError):
            harness.controller.publish(first_command, target_names=_BODY_TEST_TARGETS, required_fields=0x1f)

        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)
        harness.controller.publish(first_command, target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        harness.status(action="EXTERNAL", phase=ExternalPhase.TIMEOUT_HOLD)
        harness.controller.publish(make_command(2, q=0.2), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)

        published_commands = [
            message
            for topic, message in harness.transport.published
            if topic == BODY_CMD_TOPIC
        ]
        self.assertEqual([message.sequence for message in published_commands], [1, 2])
        self.assertEqual(
            [message.protocol_version for message in published_commands], [2, 2]
        )
        self.assertEqual(
            [message.session_id for message in published_commands],
            [harness.generation, harness.generation],
        )
        self.assertEqual(
            [message.sequence for message in published_commands], [1, 2]
        )

        harness.status(action="FIXED_STAND", phase=ExternalPhase.IDLE)
        with self.assertRaises(NotRunningError):
            harness.controller.publish(first_command, target_names=_BODY_TEST_TARGETS, required_fields=0x1f)

    def test_publish_requires_a_locally_started_request_even_with_running_status(
        self,
    ) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.transport.emit(
            CONTROL_STATUS_TOPIC,
            ControlStatus(runtime_id=11,
                status_seq=1,
                current_action="EXTERNAL",
                phase=ExternalPhase.RUNNING,
                client_id=9,
                control_id=0,
                generation=42,
            ),
        )

        with self.assertRaises(NotRunningError):
            harness.controller.publish(make_command(1), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)

    def test_publish_assigns_sequence_independently_of_caller_hint(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)

        harness.controller.publish(make_command(60000), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        harness.controller.publish(make_command(7), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)

        published_commands = [
            message
            for topic, message in harness.transport.published
            if topic == BODY_CMD_TOPIC
        ]
        self.assertTrue(all(not hasattr(message, 'cmd_id') for message in published_commands))
        self.assertEqual(
            [message.sequence for message in published_commands], [1, 2]
        )

    def test_publish_returns_actual_key_and_sequence_does_not_wrap_at_uint16(self):
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=.1)
        harness.controller._next_external_sequence = 65535
        published = [harness.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f) for _ in range(3)]
        self.assertEqual([(m.session_id, m.sequence) for m in published],
                         [(42, 65535), (42, 65536), (42, 65537)])
        actual = [m for topic, m in harness.transport.published if topic == BODY_CMD_TOPIC]
        self.assertEqual(published, actual)

    def test_new_running_generation_restarts_external_sequence(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)
        harness.controller.publish(make_command(1), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)

        harness.status(
            action="EXTERNAL",
            phase=ExternalPhase.RUNNING,
            generation=84,
        )
        harness.controller.publish(make_command(2), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)

        published_commands = [
            message
            for topic, message in harness.transport.published
            if topic == BODY_CMD_TOPIC
        ]
        self.assertEqual(
            [message.session_id for message in published_commands],
            [42, 84],
        )
        self.assertEqual(
            [message.sequence for message in published_commands], [1, 1]
        )

    def test_publish_refuses_external_sequence_overflow(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)
        harness.controller._next_external_sequence = (1 << 64) - 1
        last = harness.controller.publish(make_command(1), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        self.assertEqual((last.session_id, last.sequence), (42, (1 << 64) - 1))

        with self.assertRaisesRegex(ExternalControlError, "sequence exhausted"):
            harness.controller.publish(make_command(1), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        with self.assertRaisesRegex(ExternalControlError, "sequence exhausted"):
            harness.controller._reserve_command()

        self.assertEqual([m for topic, m in harness.transport.published if topic == BODY_CMD_TOPIC],
                         [last])

    def test_publish_rejects_invalid_structure_and_declared_target_ranges(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)

        invalid_commands = (
            BodyCmd(joints=()),
            BodyCmd(
                joints=(BodyJointCmd(q=math.nan),)
                + tuple(BodyJointCmd() for _ in range(30)),
            ),
            BodyCmd(
                joints=(BodyJointCmd(q=True),)
                + tuple(BodyJointCmd() for _ in range(30)),
            ),
        )
        for invalid_command in invalid_commands:
            with self.subTest(command=invalid_command):
                with self.assertRaises(InvalidCommandError):
                    harness.controller.publish(invalid_command, target_names=_BODY_TEST_TARGETS, required_fields=0x1f)

        with self.assertRaises(InvalidCommandError):
            harness.controller.publish(
                BodyCmd(joints=tuple(BodyJointCmd(q=1e9, tau=-1e9) for _ in range(31))),
                target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        published = harness.controller.publish(
            BodyCmd(joints=tuple(BodyJointCmd(q=.5, tau=-.5) for _ in range(31))),
            target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        self.assertEqual((published.session_id, published.sequence), (42, 1))

    def test_body_payload_is_validated_once_before_reservation(self):
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(.1)
        command = make_command(0)
        with mock.patch.object(controller_module, '_validate_command',
                               wraps=controller_module._validate_command) as validate:
            published = harness.controller.publish(command, target_names=_BODY_TEST_TARGETS,
                                                   required_fields=0x1f)
        validate.assert_called_once_with(command)
        self.assertEqual((published.session_id, published.sequence), (42, 1))

    def test_body_target_reentry_rechecks_live_context_before_key(self):
        for change in ('generation', 'body_info', 'owner', 'cancel', 'close'):
            with self.subTest(change=change):
                h = ControllerHarness()
                self.addCleanup(h.controller.close)
                h.accept_enter()
                h.controller.enter_external(.1)
                reply = h.transport.get_robot_info(.1)
                fired = []
                def change_context():
                    if change == 'generation':
                        h.status(action='EXTERNAL', phase=ExternalPhase.RUNNING, generation=84)
                    elif change == 'body_info':
                        h.transport.get_robot_info = lambda _: replace(
                            reply, info=replace(reply.info, body=replace(reply.info.body, roster_version=reply.info.body.roster_version + 1)))
                        h.controller.get_robot_info(.1)
                    elif change == 'owner':
                        status = h.controller.status
                        h.transport.emit(CONTROL_STATUS_TOPIC,
                            replace(status, status_seq=status.status_seq + 1, client_id=17))
                    elif change == 'cancel':
                        h.controller._best_effort_cancel()
                    else:
                        h.controller.close()
                class TargetName(str):
                    def __hash__(self):
                        if not fired:
                            fired.append(True)
                            change_context()
                        return super().__hash__()
                names = (TargetName(_BODY_TEST_TARGETS[0]),) + _BODY_TEST_TARGETS[1:]
                with self.assertRaises((NotRunningError, ControllerClosedError, ValueError)):
                    h.controller.publish(make_command(0), target_names=names, required_fields=0x1f)
                self.assertEqual(fired, [True])
                self.assertEqual(h.controller._next_external_sequence, 1)
                self.assertEqual(h.controller._external_session_id, 42)
                self.assertFalse(any(topic == BODY_CMD_TOPIC for topic, _ in h.transport.published))

    def test_equal_body_rediscovery_during_publish_keeps_canonical_and_key(self):
        h = ControllerHarness()
        self.addCleanup(h.controller.close)
        h.accept_enter(); h.controller.enter_external(.1)
        canonical = h.controller._body_info
        reply = h.controller.get_robot_info(.1)
        rebuilt = replace(canonical, joints=tuple(replace(j) for j in canonical.joints))
        h.transport.get_robot_info = lambda _: replace(reply, info=replace(reply.info, body=rebuilt))
        original = h.transport._publish_body
        reserved = []
        def rediscover_then_submit(command, guard):
            reserved.append((command.session_id, command.sequence, h.controller._next_external_sequence))
            h.controller.get_robot_info(.1)
            return original(command, guard)
        h.transport._publish_body = rediscover_then_submit
        sent = h.controller.publish(make_command(0), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        self.assertIs(h.controller._body_info, canonical)
        self.assertIsNot(rebuilt, canonical)
        self.assertEqual(reserved, [(42, 1, 2)])
        self.assertEqual((sent.session_id, sent.sequence), (42, 1))
        self.assertEqual(sum(t == BODY_CMD_TOPIC for t, _ in h.transport.published), 1)

    def test_equal_unknown_body_ranges_do_not_retire_but_real_change_does(self):
        h = ControllerHarness()
        self.addCleanup(h.controller.close)
        reply = h.controller.get_robot_info(.1)
        # Establish a separate connection with an unknown (invalid) q range.
        joint = replace(reply.info.body.joints[0], valid_limits=0, q_min=float('nan'))
        body = replace(reply.info.body, joints=(joint,) + reply.info.body.joints[1:])
        h = ControllerHarness(info_reply=replace(reply, info=replace(reply.info, body=body)))
        self.addCleanup(h.controller.close)
        rebuilt = replace(body, joints=(replace(joint, q_min=float('nan')),) + body.joints[1:])
        h.transport.get_robot_info = lambda _: replace(reply, info=replace(reply.info, body=rebuilt))
        h.controller.get_robot_info(.1)
        self.assertIs(h.controller._body_info, body)
        changed = replace(rebuilt, joints=(replace(rebuilt.joints[0], is_mock=True),) + rebuilt.joints[1:])
        h.transport.get_robot_info = lambda _: replace(reply, info=replace(reply.info, body=changed))
        with self.assertRaises(ValueError):
            h.controller.get_robot_info(.1)
        self.assertTrue(h.controller._runtime_binding.poisoned)

    def test_v4_request_uses_v4_ack_and_status_without_fallback(self):
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        acks = []
        subscription = harness.transport.subscribe(CONTROL_ACK_TOPIC, acks.append)
        self.addCleanup(subscription.close)
        harness.accept_enter()
        status = harness.controller.enter_external(timeout=0.1)
        requests = [m for topic, m in harness.transport.published if topic == CONTROL_REQUEST_TOPIC]
        self.assertEqual([r.protocol_version for r in requests], [4, 4])
        self.assertEqual([ack.protocol_version for ack in acks], [4, 4])
        self.assertEqual(status.protocol_version, 4)

    def test_publish_requires_the_public_fixed31_span(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)
        for count in (0, 1, 30, 32, 42, 44, 45):
            with self.subTest(count=count), self.assertRaises(InvalidCommandError):
                harness.controller.publish(BodyCmd(tuple(BodyJointCmd() for _ in range(count))), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        harness.controller.publish(make_command(1), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)

    def test_unknown_body_targets_and_nonfinite_slot_are_rejected_before_key(self):
        harness = ControllerHarness(info_reply=robot_info_reply())
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)
        harness.controller.get_robot_info()
        motors = list(make_command(1).joints)
        motors[14] = BodyJointCmd(q=math.nan, dq=math.inf)
        with self.assertRaises(InvalidCommandError):
            harness.controller.publish(BodyCmd(tuple(motors)))
        motors[14] = BodyJointCmd()
        motors[13] = BodyJointCmd(q=math.nan)
        with self.assertRaises(InvalidCommandError):
            harness.controller.publish(BodyCmd(tuple(motors)))
        self.assertEqual(harness.controller._next_external_sequence, 1)
        self.assertFalse(any(topic == BODY_CMD_TOPIC for topic, _ in harness.transport.published))
        with self.assertRaisesRegex(InvalidCommandError, 'target scope must be explicitly declared'):
            harness.controller.publish(make_command(1))
        self.assertEqual(harness.controller._next_external_sequence, 1)
        self.assertTrue(harness.controller.get_robot_info().info.body.joints[14].is_mock)

    def test_declared_thirty_targets_publish_and_cancel_with_mock_padding(self):
        reply = robot_info_reply()
        body = reply.info.body
        joints = list(body.joints)
        # Slot 0 is a synthetic non-ankle q-only shape: this is local SDK
        # validation only, not proof of any physical producer or adapter.
        for i in (0, 4, 5, 10, 11):
            joints[i] = replace(joints[i], supported_fields=31, valid_limits=1,
                q_min=-.751 if i in (4, 10) else -.43625,
                q_max=.414 if i in (4, 10) else .43625,
                dq_min=0., dq_max=0., tau_min=0., tau_max=0.,
                kp_min=0., kp_max=0., kd_min=0., kd_max=0.)
        body = replace(body, joint_names=ASAMPLE_BODY_JOINT_NAMES, joints=tuple(joints))
        harness = ControllerHarness(info_reply=replace(reply, info=replace(reply.info, body=body)))
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(.1)
        info = harness.controller.get_robot_info(.1).info.body
        targets = tuple(name for i, name in enumerate(info.joint_names) if i != 14)
        self.assertEqual((info.joints[4].supported_fields, info.joints[4].valid_limits,
                          info.joints[4].kp_min, info.joints[4].kp_max), (31, 1, 0., 0.))
        command = BodyCmd(tuple(BodyJointCmd(q=.1, kp=1., kd=.5) if i != 14
                                else BodyJointCmd() for i in range(31)))
        sent = harness.controller.publish(command, target_names=targets, required_fields=0x19)
        self.assertEqual((sent.session_id, sent.sequence), (42, 1))
        all_fields = BodyCmd(tuple(BodyJointCmd(q=.1, dq=.1, tau=.1, kp=1., kd=.5)
                                   if i != 14 else BodyJointCmd() for i in range(31)))
        self.assertEqual(harness.controller.publish(all_fields, target_names=targets,
                                                    required_fields=0x1f).sequence, 2)
        def cancel(topic, request):
            if topic == CONTROL_REQUEST_TOPIC and request.operation == ControlOperation.CANCEL:
                harness.ack(request)
                harness.status(action='PASSIVE', phase=ExternalPhase.IDLE)
        harness.transport.on_publish = cancel
        self.assertEqual(harness.controller.exit_external(.1).phase, ExternalPhase.IDLE)

    def test_declared_mock_target_and_non_target_padding_reject_before_key(self):
        reply = robot_info_reply()
        body = reply.info.body
        targets = tuple(name for i, name in enumerate(body.joint_names) if i != 14)
        for failure in ('mock_target', 'padding', 'undeclared_field', 'empty', 'duplicate', 'unknown',
                        'missing_q', 'missing_capability',
                        'known_gain_range', 'known_undeclared_range', 'outside_q', 'kp_only'):
            with self.subTest(failure=failure):
                joints = list(body.joints)
                if failure == 'mock_target':
                    joints[0] = replace(joints[0], is_mock=True)
                    self.assertNotEqual(joints[0].supported_fields, 0)
                elif failure == 'missing_q':
                    joints[0] = replace(joints[0], valid_limits=0x18)
                elif failure == 'missing_capability':
                    joints[0] = replace(joints[0], supported_fields=0x11)
                elif failure == 'known_gain_range':
                    joints[0] = replace(joints[0], kp_max=.5)
                elif failure == 'known_undeclared_range':
                    joints[0] = replace(joints[0], dq_min=.2, dq_max=1.2)
                elif failure == 'outside_q':
                    joints[0] = replace(joints[0], q_min=.2, q_max=1.2)
                source = replace(reply, info=replace(reply.info, body=replace(body, joints=tuple(joints))))
                harness = ControllerHarness(info_reply=source)
                self.addCleanup(harness.controller.close)
                harness.accept_enter(); harness.controller.enter_external(.1)
                values = [BodyJointCmd() for _ in range(31)]
                if failure == 'padding':
                    values[14] = BodyJointCmd(q=.1)
                if failure == 'undeclared_field':
                    values[0] = BodyJointCmd(kp=1.)
                if failure in ('known_gain_range', 'outside_q', 'kp_only'):
                    values[0] = BodyJointCmd(q=0., kp=1.)
                if failure == 'kp_only':
                    self.assertLessEqual(joints[0].q_min, 0.)
                    self.assertGreaterEqual(joints[0].q_max, 0.)
                declared = (() if failure == 'empty' else targets + (targets[0],) if failure == 'duplicate'
                            else targets + ('not-in-roster',) if failure == 'unknown' else targets)
                with self.assertRaises(InvalidCommandError):
                    harness.controller.publish(BodyCmd(tuple(values)), target_names=declared,
                                               required_fields=(1 if failure == 'undeclared_field'
                                                                else 0x08 if failure == 'kp_only' else 0x19))
                self.assertEqual(harness.controller._next_external_sequence, 1)
                self.assertFalse(any(topic == BODY_CMD_TOPIC for topic, _ in harness.transport.published))

    def test_exit_waits_until_status_no_longer_reports_external(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)

        def on_publish(topic, request) -> None:
            if topic != CONTROL_REQUEST_TOPIC:
                return
            if request.operation == ControlOperation.CANCEL:
                harness.ack(request)
                harness.status(action="FIXED_STAND", phase=ExternalPhase.IDLE)

        harness.transport.on_publish = on_publish

        status = harness.controller.exit_external(timeout=0.1)

        self.assertEqual(status.current_action, "FIXED_STAND")
        self.assertEqual(
            [
                message.operation
                for topic, message in harness.transport.published
                if topic == CONTROL_REQUEST_TOPIC
            ][-1],
            ControlOperation.CANCEL,
        )
        cancel = next(
            message
            for topic, message in reversed(harness.transport.published)
            if topic == CONTROL_REQUEST_TOPIC
            and message.operation == ControlOperation.CANCEL
        )
        self.assertFalse(hasattr(cancel, "requested_lease_ms"))

    def test_exit_waits_for_cleanup_completed_idle_status(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)
        cancel_seen = threading.Event()
        result: list[ControlStatus] = []
        failure: list[BaseException] = []

        def on_publish(topic, request) -> None:
            if (
                topic == CONTROL_REQUEST_TOPIC
                and request.operation == ControlOperation.CANCEL
            ):
                harness.ack(request)
                harness.status(
                    action="PASSIVE", phase=ExternalPhase.CANCELLED
                )
                cancel_seen.set()

        def exit_controller() -> None:
            try:
                result.append(harness.controller.exit_external(timeout=1.0))
            except BaseException as error:
                failure.append(error)

        harness.transport.on_publish = on_publish
        thread = threading.Thread(target=exit_controller)
        thread.start()
        self.assertTrue(cancel_seen.wait(0.5))
        thread.join(0.05)
        self.assertTrue(thread.is_alive())

        harness.status(action="PASSIVE", phase=ExternalPhase.IDLE)
        thread.join(0.5)

        self.assertFalse(thread.is_alive())
        self.assertEqual(failure, [])
        self.assertEqual(result[0].current_action, "PASSIVE")

    def test_context_manager_cancels_and_close_is_idempotent(self) -> None:
        harness = ControllerHarness()
        harness.accept_enter()

        with self.assertRaisesRegex(RuntimeError, "boom"):
            with harness.controller as controller:
                controller.enter_external(timeout=0.1)
                raise RuntimeError("boom")

        harness.controller.close()
        self.assertEqual(harness.transport.close_count, 1)
        self.assertTrue(
            all(
                subscription.close_count == 1
                for subscription in harness.transport.subscriptions
            )
        )
        operations = [
            message.operation
            for topic, message in harness.transport.published
            if topic == CONTROL_REQUEST_TOPIC
        ]
        self.assertEqual(operations[-1], ControlOperation.CANCEL)


if __name__ == "__main__":
    unittest.main()

class RuntimeIdentityR3Test(unittest.TestCase):
    """Control tests use the original controller/transport entry, without a wire shim."""

    def make_controller(self, runtime=11):

        transport = FakeTransport()
        transport.get_robot_info = lambda timeout: replace(
            robot_info_reply(), runtime_id=runtime)
        controller = ExternalController(client_id=7, _transport=transport)
        controller.get_robot_info(.1)
        self.addCleanup(controller.close)
        return controller, transport

    def test_prepare_canonical_client_differs_from_proposal_and_running_confirms(self):

        controller, transport = self.make_controller()
        seen = []

        def publish(topic, request):
            if topic != CONTROL_REQUEST_TOPIC:
                return
            seen.append(request)
            transport.emit(CONTROL_ACK_TOPIC, ControlAck(
                request.request_id, request.operation, True, client_id=9,
                control_id=13, runtime_id=11))
            if request.operation == ControlOperation.COMMIT:
                transport.emit(CONTROL_STATUS_TOPIC, ControlStatus(
                    1, 'EXTERNAL', ExternalPhase.RUNNING, client_id=9,
                    control_id=13, generation=17, runtime_id=11))

        transport.on_publish = publish
        self.assertEqual(controller.enter_external(.1).generation, 17)
        self.assertEqual((seen[0].client_id, seen[0].control_id), (7, 0))
        self.assertEqual((seen[1].client_id, seen[1].control_id), (9, 13))
        key = controller._reserve_command()
        self.assertEqual((key.client_id, key.control_id, key.session_id, key.sequence), (9, 13, 17, 1))

    def test_lost_prepare_ack_cancels_with_original_proposal_and_zero_control(self):
        controller, transport = self.make_controller()
        with self.assertRaises(ExternalControlTimeoutError):
            controller.enter_external(.001)
        requests = [value for topic, value in transport.published if topic == CONTROL_REQUEST_TOPIC]
        self.assertEqual([value.operation for value in requests], [ControlOperation.PREPARE, ControlOperation.CANCEL])
        self.assertEqual((requests[-1].client_id, requests[-1].control_id), (7, 0))
        self.assertEqual(requests[-1].request_id, requests[0].request_id)

    def test_unrelated_status_does_not_change_runtime_and_service_restart_retires(self):

        controller, transport = self.make_controller()
        creating_runtime = controller._runtime_binding.runtime_id
        transport.emit(CONTROL_STATUS_TOPIC, ControlStatus(
            1, 'EXTERNAL', ExternalPhase.RUNNING, client_id=2,
            control_id=3, generation=1, runtime_id=99))
        self.assertIsNone(controller.status)
        transport.get_robot_info = lambda timeout: replace(
            robot_info_reply(), runtime_id=12)
        with self.assertRaisesRegex(ValueError, 'robot runtime changed'):
            controller.get_robot_info(.1)
        self.assertEqual(controller._runtime_binding.runtime_id, creating_runtime)
        self.assertTrue(controller._runtime_binding.poisoned)
        with self.assertRaisesRegex(ValueError, 'robot runtime changed'):
            controller.enter_external(.1)

    def test_failed_or_zero_discovery_never_authorizes_entry(self):
        from locomotion_aorta.body_types import ServiceStatus
        for reply in (replace(robot_info_reply(), runtime_id=0),
                      replace(robot_info_reply(), status=ServiceStatus.INTERNAL_ERROR,
                              runtime_id=11)):
            transport = FakeTransport()
            transport.get_robot_info = lambda timeout, result=reply: result
            controller = ExternalController(client_id=7, _transport=transport)
            self.addCleanup(controller.close)
            self.assertEqual(controller.get_robot_info(.1), reply)
            with self.assertRaises(NotRunningError):
                controller.enter_external(.1)
            self.assertEqual(transport.published, [])

    def test_cancel_between_prepare_and_typed_fill_blocks_late_prepare(self):
        controller, transport = self.make_controller()
        reached = threading.Event()
        proceed = threading.Event()
        errors = []
        original = transport._publish_control

        def blocked(request, guard):
            if request.operation == ControlOperation.PREPARE:
                reached.set()
                self.assertTrue(proceed.wait(1))
            original(request, guard)

        transport._publish_control = blocked
        def enter():
            try:
                controller.enter_external(.5)
            except BaseException as error:
                errors.append(error)
        worker = threading.Thread(target=enter)
        worker.start()
        self.assertTrue(reached.wait(1))
        controller._best_effort_cancel()
        proceed.set()
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertTrue(errors)
        operations = [value.operation for topic, value in transport.published
                      if topic == CONTROL_REQUEST_TOPIC]
        self.assertEqual(operations, [ControlOperation.CANCEL])

    def test_late_prepare_ack_cannot_reopen_cancelled_context(self):

        controller, transport = self.make_controller()
        with self.assertRaises(ExternalControlTimeoutError):
            controller.enter_external(.001)
        request = transport.published[0][1]
        transport.emit(CONTROL_ACK_TOPIC, ControlAck(
            request.request_id, ControlOperation.PREPARE, True,
            client_id=9, control_id=13, runtime_id=11))
        self.assertNotIn((request.request_id, ControlOperation.PREPARE), controller._acks)
        with self.assertRaises(NotRunningError):
            controller._reserve_command()

    def test_invalid_client_ids_never_create_control_transport(self):
        for value in (True, -1, 0, 1 << 64, '7'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                ExternalController(client_id=value, _transport=FakeTransport())

    def test_correlated_management_reply_with_new_runtime_retires_connection(self):
        controller, transport = self.make_controller()
        def publish(topic, request):
            if topic == CONTROL_REQUEST_TOPIC and request.operation == ControlOperation.PREPARE:
                transport.emit(CONTROL_ACK_TOPIC, ControlAck(
                    request.request_id, request.operation, True, client_id=9,
                    control_id=13, runtime_id=12))
        transport.on_publish = publish
        with self.assertRaisesRegex(ValueError, 'runtime changed'):
            controller.enter_external(.1)
        with self.assertRaisesRegex(ValueError, 'runtime changed'):
            controller._control_context()

    def test_close_while_discovery_waits_cannot_bind_late_response(self):
        entered, release = threading.Event(), threading.Event()
        transport = FakeTransport()
        reply = robot_info_reply()
        def discover(timeout):
            entered.set()
            if not release.wait(1):
                raise RuntimeError('test did not release discovery')
            return reply
        transport.get_robot_info = discover
        controller = ExternalController(client_id=7, _transport=transport)
        failures = []
        def read():
            try:
                controller.get_robot_info(.5)
            except BaseException as error:
                failures.append(error)
        worker = threading.Thread(target=read)
        worker.start()
        self.assertTrue(entered.wait(1))
        controller.close()
        release.set()
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(failures), 1)
        self.assertIsInstance(failures[0], ControllerClosedError)
        self.assertIsNone(controller._runtime_binding.runtime_id)

    def test_body_prepare_failure_precedes_prepare_and_key_allocation(self):
        controller, transport = self.make_controller()
        def fail():
            self.assertFalse(controller._condition._is_owned())
            raise RuntimeError('actual Body ABI mismatch')
        transport._prepare_body_output = fail
        with self.assertRaisesRegex(RuntimeError, 'actual Body ABI mismatch'):
            controller.enter_external(.1)
        self.assertEqual(transport.published, [])
        self.assertIsNone(controller._request_id)
        self.assertEqual(controller._next_external_sequence, 1)

    def test_body_prepare_is_not_used_for_discovery_or_teleop(self):
        controller, transport = self.make_controller()
        transport._prepare_body_output = mock.Mock(side_effect=AssertionError('unexpected Body preparation'))
        controller.get_robot_info(.1)
        def reply(topic, request):
            if topic != CONTROL_REQUEST_TOPIC:
                return
            transport.emit(CONTROL_ACK_TOPIC, ControlAck(
                request.request_id, request.operation, True, client_id=9,
                control_id=13, runtime_id=11))
            if request.operation == ControlOperation.COMMIT:
                transport.emit(CONTROL_STATUS_TOPIC, ControlStatus(
                    1, 'HUMANOID_TELEOP', ExternalPhase.RUNNING, client_id=9,
                    control_id=13, generation=17, runtime_id=11))
        transport.on_publish = reply
        self.assertEqual(controller.enter_control('HUMANOID_TELEOP', .1).generation, 17)
        transport._prepare_body_output.assert_not_called()

    def test_close_during_body_prepare_never_sends_prepare(self):
        controller, transport = self.make_controller()
        reached, release = threading.Event(), threading.Event()
        failures = []
        def prepare():
            self.assertFalse(controller._condition._is_owned())
            reached.set()
            if not release.wait(1):
                raise RuntimeError('test did not release preparation')
        transport._prepare_body_output = prepare
        def enter():
            try:
                controller.enter_external(.5)
            except BaseException as error:
                failures.append(error)
        worker = threading.Thread(target=enter)
        worker.start()
        self.assertTrue(reached.wait(1))
        controller.close()
        release.set()
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(failures), 1)
        self.assertIsInstance(failures[0], ControllerClosedError)
        self.assertEqual(transport.published, [])

    def test_runtime_change_during_body_prepare_never_sends_prepare(self):
        controller, transport = self.make_controller()
        def prepare():
            self.assertFalse(controller._condition._is_owned())
            transport.get_robot_info = lambda timeout: replace(
                robot_info_reply(), runtime_id=12)
            with self.assertRaisesRegex(ValueError, 'runtime changed'):
                controller.get_robot_info(.1)
        transport._prepare_body_output = prepare
        with self.assertRaisesRegex(ValueError, 'runtime changed'):
            controller.enter_external(.1)
        self.assertEqual(transport.published, [])

    def test_transport_body_prepare_failure_closes_only_new_publisher(self):
        class Publisher:
            def __init__(self):
                self.close_count = 0
            def close(self):
                self.close_count += 1
        class Node:
            def __init__(self, name, group):
                self._group = group
                self.created = []
                self.close_count = 0
            def group(self):
                return self._group
            def create_publisher_typed(self, meta, topic, *, qos):
                pub = Publisher()
                self.created.append((topic, pub))
                return pub
            def close(self):
                self.close_count += 1
        qos = SimpleNamespace(realtime_control=lambda: 'control')
        bindings = SimpleNamespace(aorta=SimpleNamespace(Node=Node, QoS=qos),
                                   control_request_schema_meta=object(),
                                   body_cmd_schema_meta=object())
        with mock.patch.object(transport_module, '_load_aorta_bindings', return_value=bindings), \
             mock.patch('locomotion_aorta._native_body_codec.NativeBodyPublisher',
                        side_effect=RuntimeError('native schema mismatch')):
            transport = transport_module._AortaTransport(node_name='node', group='group')
            self.addCleanup(transport.close)
            self.assertEqual([topic for topic, _ in transport.node.created], [CONTROL_REQUEST_TOPIC])
            with self.assertRaisesRegex(RuntimeError, 'native schema mismatch'):
                transport._prepare_body_output()
            self.assertEqual([topic for topic, _ in transport.node.created],
                             [CONTROL_REQUEST_TOPIC, BODY_CMD_TOPIC])
            self.assertEqual(transport.node.created[-1][1].close_count, 1)
            self.assertNotIn(BODY_CMD_TOPIC, transport.publishers)
            self.assertIsNone(transport._body_publisher)

    def test_old_discovery_reply_cannot_retire_newly_bound_runtime(self):
        entered, release = threading.Event(), threading.Event()
        transport = FakeTransport()
        controller = ExternalController(client_id=7, _transport=transport)
        self.addCleanup(controller.close)
        errors = []
        def discover(timeout):
            if threading.current_thread().name == 'old-discovery':
                entered.set()
                if not release.wait(1):
                    raise RuntimeError('test did not release old discovery')
                return replace(robot_info_reply(), runtime_id=11)
            return replace(robot_info_reply(), runtime_id=12)
        transport.get_robot_info = discover
        def old():
            try:
                controller.get_robot_info(.5)
            except BaseException as error:
                errors.append(error)
        worker = threading.Thread(target=old, name='old-discovery')
        worker.start()
        self.assertTrue(entered.wait(1))
        self.assertEqual(controller.get_robot_info(.1).runtime_id, 12)
        release.set()
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertRegex(str(errors[0]), 'discovery context changed')
        self.assertEqual(controller.get_robot_info(.1).runtime_id, 12)
