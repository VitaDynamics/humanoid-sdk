import math
import threading
import unittest
import uuid
from types import SimpleNamespace
from unittest import mock

import locomotion_aorta.transport as transport_module
from fake_transport import FakeTransport
from locomotion_aorta import (
    AortaBindingsUnavailableError,
    CONTROL_ACK_TOPIC,
    CONTROL_REQUEST_TOPIC,
    CONTROL_STATUS_TOPIC,
    EXTERNAL_COMMAND_TOPIC,
    LOWSTATE_TOPIC,
    ControlAck,
    ExternalControlError,
    ControlOperation,
    ControlRejectedError,
    ControlRequest,
    ControlStatus,
    ExternalControlTimeoutError,
    ExternalController,
    ExternalPhase,
    HumanoidLowState,
    InvalidCommandError,
    LowCmd,
    MotorCommand,
    MotorState,
    NotRunningError,
)


class ControllerHarness:
    def __init__(self) -> None:
        self.transport = FakeTransport()
        self.controller = ExternalController(
            client_id="test-client", _transport=self.transport
        )
        self.lease_id = "lease-123"
        self.status_sequence = 0
        self.generation = 42

    def ack(self, request, *, accepted: bool = True, reason: str = "") -> None:
        self.transport.emit(
            CONTROL_ACK_TOPIC,
            ControlAck(
                request_id=request.request_id,
                operation=request.operation,
                accepted=accepted,
                lease_id=self.lease_id,
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
            client_id="test-client",
            lease_id=self.lease_id,
            generation=self.generation if generation is None else generation,
        )
        self.transport.emit(CONTROL_STATUS_TOPIC, status)
        return status

    def accept_enter(self, *, publish_running: bool = True) -> None:
        def on_publish(topic, request) -> None:
            if topic != CONTROL_REQUEST_TOPIC:
                return
            if request.operation == ControlOperation.PREPARE:
                self.ack(request)
            elif request.operation == ControlOperation.COMMIT:
                self.ack(request)
                if publish_running:
                    self.status(action="EXTERNAL", phase=ExternalPhase.RUNNING)

        self.transport.on_publish = on_publish


def make_command(cmd_id: int, *, q: float = 0.0) -> LowCmd:
    return LowCmd(
        cmd_id=cmd_id,
        motor_cmds=tuple(MotorCommand(mode=10, q=q) for _ in range(44)),
    )


class ExternalControllerTest(unittest.TestCase):
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

            def publish_typed(self, fill) -> None:
                self.fills.append(fill)

        class FakeNode:
            def __init__(self, name, group) -> None:
                self.name = name
                self.group = group
                self.publisher_calls = []
                self.subscriber_calls = []
                self.close_count = 0

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
            low_cmd_schema_meta=object(),
            humanoid_low_state_decoder=object(),
            control_ack_decoder=object(),
            control_status_decoder=object(),
        )

        with mock.patch.object(
            transport_module,
            "_load_aorta_bindings",
            return_value=fake_bindings,
        ):
            transport = transport_module.create_aorta_transport(
                node_name="sdk-node", group="robot-domain"
            )

        self.assertEqual((transport.node.name, transport.node.group), ("sdk-node", "robot-domain"))
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
                    fake_bindings.low_cmd_schema_meta,
                    EXTERNAL_COMMAND_TOPIC,
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
            client_id="client-1",
            operation=ControlOperation.PREPARE,
        )
        transport.publish(CONTROL_REQUEST_TOPIC, request)
        self.assertEqual(
            len(transport.publishers[CONTROL_REQUEST_TOPIC].fills), 1
        )
        transport.close()
        transport.close()
        self.assertEqual(transport.node.close_count, 1)

    def test_generated_control_views_decode_to_public_values(self) -> None:
        ack = transport_module._decode_control_ack(
            SimpleNamespace(
                ProtocolVersion=lambda: 1,
                Operation=lambda: 2,
                RequestId=lambda: b"request-1",
                Accepted=lambda: True,
                ErrorCode=lambda: 0,
                Reason=lambda: b"",
                LeaseId=lambda: b"lease-1",
                LeaseDeadlineMonotonicNs=lambda: 123,
            )
        )
        status = transport_module._decode_control_status(
            SimpleNamespace(
                ProtocolVersion=lambda: 1,
                StatusSeq=lambda: 9,
                CurrentAction=lambda: b"EXTERNAL",
                ActionPhase=lambda: 4,
                ClientId=lambda: b"client-1",
                LeaseId=lambda: b"lease-1",
                LeaseDeadlineMonotonicNs=lambda: 123,
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
        self.assertEqual(ack.lease_id, "lease-1")
        self.assertEqual(status.phase, ExternalPhase.RUNNING)
        self.assertEqual((status.generation, status.last_accepted_cmd_id), (42, 8))

        with self.assertRaisesRegex(ValueError, "ControlActionPhase"):
            transport_module._decode_phase(-1)

    def test_subscribes_to_public_state_topics(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)

        self.assertEqual(
            set(harness.transport.callbacks),
            {LOWSTATE_TOPIC, CONTROL_ACK_TOPIC, CONTROL_STATUS_TOPIC},
        )

    def test_wait_lowstate_times_out_then_returns_the_latest_state(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)

        with self.assertRaises(ExternalControlTimeoutError):
            harness.controller.wait_lowstate(timeout=0.001)

        first = HumanoidLowState(
            state_id=10,
            stamp_ns=100,
            motor_states=(MotorState(q=0.1, dq=0.2),),
        )
        latest = HumanoidLowState(
            state_id=11,
            stamp_ns=200,
            motor_states=(MotorState(q=0.3, dq=0.4),),
        )
        harness.transport.emit(LOWSTATE_TOPIC, first)
        harness.transport.emit(LOWSTATE_TOPIC, latest)

        self.assertIs(harness.controller.wait_lowstate(timeout=0.01), latest)

    def test_status_cache_does_not_regress_or_replace_a_sequence(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        current = ControlStatus(
            status_seq=2,
            current_action="EXTERNAL",
            phase=ExternalPhase.RUNNING,
        )
        duplicate = ControlStatus(
            status_seq=2,
            current_action="FIXED_STAND",
            phase=ExternalPhase.IDLE,
        )
        stale = ControlStatus(
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
        previous_process = ControlStatus(
            status_seq=100,
            stamp_monotonic_ns=1_000,
            current_action="EXTERNAL",
            phase=ExternalPhase.RUNNING,
        )
        restarted_process = ControlStatus(
            status_seq=0,
            stamp_monotonic_ns=2_000,
            current_action="FIXED_STAND",
            phase=ExternalPhase.IDLE,
        )
        delayed_previous_status = ControlStatus(
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
        self.assertEqual({request.client_id for request in requests}, {"test-client"})
        self.assertEqual({request.protocol_version for request in requests}, {1})
        self.assertEqual(requests[0].lease_id, "")
        self.assertEqual(requests[0].requested_lease_ms, 30_000)
        self.assertEqual(requests[1].lease_id, harness.lease_id)
        self.assertEqual(requests[1].requested_lease_ms, 0)
        self.assertEqual(status.phase, ExternalPhase.RUNNING)

    def test_enter_sends_requested_lease_only_in_prepare(self) -> None:
        for seconds, milliseconds in (
            (90.0, 90_000), (0.001, 1), (1.234, 1234),
            (4_294_967.295, 4_294_967_295),
        ):
            with self.subTest(seconds=seconds):
                harness = ControllerHarness()
                self.addCleanup(harness.controller.close)
                harness.accept_enter()
                harness.controller.enter_external(
                    timeout=0.1, lease_duration_s=seconds
                )
                requests = [
                    message
                    for topic, message in harness.transport.published
                    if topic == CONTROL_REQUEST_TOPIC
                ]
                self.assertEqual(requests[0].requested_lease_ms, milliseconds)
                self.assertEqual(requests[1].requested_lease_ms, 0)
                harness.controller.close()
                cancel = harness.transport.published[-1][1]
                self.assertEqual(cancel.operation, ControlOperation.CANCEL)
                self.assertEqual(cancel.requested_lease_ms, 0)

    def test_invalid_lease_is_rejected_before_request_and_allows_retry(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        for value in (
            0, -1, 0.0009, True, "90", None, math.nan,
            math.inf, -math.inf, 4_294_967.296, 10**400,
        ):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    harness.controller.enter_external(
                        timeout=0.1, lease_duration_s=value
                    )
                self.assertEqual(harness.transport.published, [])
        harness.controller.enter_external(timeout=0.1)
        prepare = harness.transport.published[0][1]
        self.assertEqual(prepare.requested_lease_ms, 30_000)

    def test_enter_accepts_running_status_after_server_sequence_reset(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.transport.emit(
            CONTROL_STATUS_TOPIC,
            ControlStatus(
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
                    ControlStatus(
                        status_seq=0,
                        stamp_monotonic_ns=2_000,
                        current_action="EXTERNAL",
                        phase=ExternalPhase.RUNNING,
                        client_id="test-client",
                        lease_id=harness.lease_id,
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
                    ControlAck(
                        request_id=request.request_id,
                        operation=request.operation,
                        accepted=True,
                        lease_id=harness.lease_id,
                        protocol_version=2,
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
                    ControlStatus(
                        status_seq=1,
                        current_action="EXTERNAL",
                        phase=ExternalPhase.RUNNING,
                        client_id="test-client",
                        lease_id=harness.lease_id,
                        protocol_version=2,
                    ),
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
            harness.controller.publish(first_command)

        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)
        harness.controller.publish(first_command)
        harness.status(action="EXTERNAL", phase=ExternalPhase.TIMEOUT_HOLD)
        harness.controller.publish(make_command(2, q=0.2))

        published_commands = [
            message
            for topic, message in harness.transport.published
            if topic == EXTERNAL_COMMAND_TOPIC
        ]
        self.assertEqual([message.cmd_id for message in published_commands], [1, 2])
        self.assertEqual(
            [message.external_version for message in published_commands], [1, 1]
        )
        self.assertEqual(
            [message.external_session_id for message in published_commands],
            [harness.generation, harness.generation],
        )
        self.assertEqual(
            [message.external_sequence for message in published_commands], [1, 2]
        )

        harness.status(action="FIXED_STAND", phase=ExternalPhase.IDLE)
        with self.assertRaises(NotRunningError):
            harness.controller.publish(first_command)

    def test_publish_requires_a_locally_started_request_even_with_running_status(
        self,
    ) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.transport.emit(
            CONTROL_STATUS_TOPIC,
            ControlStatus(
                status_seq=1,
                current_action="EXTERNAL",
                phase=ExternalPhase.RUNNING,
                client_id="test-client",
                lease_id="",
                generation=42,
            ),
        )

        with self.assertRaises(NotRunningError):
            harness.controller.publish(make_command(1))

    def test_publish_does_not_use_legacy_cmd_id_for_external_ordering(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)

        harness.controller.publish(make_command(60000))
        harness.controller.publish(make_command(7))

        published_commands = [
            message
            for topic, message in harness.transport.published
            if topic == EXTERNAL_COMMAND_TOPIC
        ]
        self.assertEqual([message.cmd_id for message in published_commands], [60000, 7])
        self.assertEqual(
            [message.external_sequence for message in published_commands], [1, 2]
        )

    def test_new_running_generation_restarts_external_sequence(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)
        harness.controller.publish(make_command(1))

        harness.status(
            action="EXTERNAL",
            phase=ExternalPhase.RUNNING,
            generation=84,
        )
        harness.controller.publish(make_command(2))

        published_commands = [
            message
            for topic, message in harness.transport.published
            if topic == EXTERNAL_COMMAND_TOPIC
        ]
        self.assertEqual(
            [message.external_session_id for message in published_commands],
            [42, 84],
        )
        self.assertEqual(
            [message.external_sequence for message in published_commands], [1, 1]
        )

    def test_publish_refuses_external_sequence_overflow(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)
        harness.controller._next_external_sequence = 1 << 64

        with self.assertRaisesRegex(ExternalControlError, "sequence exhausted"):
            harness.controller.publish(make_command(1))

        self.assertFalse(
            any(
                topic == EXTERNAL_COMMAND_TOPIC
                for topic, _ in harness.transport.published
            )
        )

    def test_publish_rejects_invalid_structure_but_not_robot_side_ranges(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)

        invalid_commands = (
            make_command(0),
            make_command(1 << 16),
            LowCmd(cmd_id=1, motor_cmds=()),
            LowCmd(
                cmd_id=1,
                motor_cmds=(MotorCommand(mode=10, q=math.nan),)
                + tuple(MotorCommand(mode=10) for _ in range(43)),
            ),
            LowCmd(
                cmd_id=1,
                motor_cmds=(MotorCommand(mode=256),)
                + tuple(MotorCommand(mode=10) for _ in range(43)),
            ),
        )
        for invalid_command in invalid_commands:
            with self.subTest(command=invalid_command):
                with self.assertRaises(InvalidCommandError):
                    harness.controller.publish(invalid_command)

        harness.controller.publish(
            LowCmd(
                cmd_id=1,
                motor_cmds=(MotorCommand(mode=10, q=1e9, tau=-1e9),),
            )
        )

    def test_publish_does_not_hardcode_the_robot_command_span(self) -> None:
        harness = ControllerHarness()
        self.addCleanup(harness.controller.close)
        harness.accept_enter()
        harness.controller.enter_external(timeout=0.1)

        harness.controller.publish(
            LowCmd(cmd_id=1, motor_cmds=(MotorCommand(mode=10),))
        )
        harness.controller.publish(
            LowCmd(
                cmd_id=2,
                motor_cmds=tuple(MotorCommand(mode=10) for _ in range(45)),
            )
        )

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
        self.assertEqual(cancel.requested_lease_ms, 0)

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
