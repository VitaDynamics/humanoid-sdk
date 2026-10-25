"""Shared-connection EE feedback callers after retiring the old pull clients.

The EE client uses typed callbacks and a bounded latest-value cache. Body remains
pull based. These synthetic callback tests do not prove native scheduling/GIL.
"""
import threading
import unittest
from unittest.mock import patch

from locomotion_aorta import EndEffectorClient
from locomotion_aorta.end_effector_types import Field, SampleStream
from test_controller import ControllerHarness
from test_end_effector_client import Node, confirm_info
from test_end_effector_codec import info_object, state_object, target


class EEPullTest(unittest.TestCase):
    def client(self, side='left', **options):
        owner = ControllerHarness()
        self.addCleanup(owner.controller.close)
        node = Node()
        owner.transport.node = node
        owner.accept_enter()
        owner.controller.enter_external(timeout=.1)
        # Construction must not add a Python receiver/polling thread.
        with patch('threading.Thread', side_effect=AssertionError('Python worker created')):
            client = EndEffectorClient(side, connection=owner.controller, **options)
        self.addCleanup(client.close)
        confirm_info(client, node, info_object(side=1 if side == 'left' else 2))
        return client, node, owner

    def test_contending_waiters_sleep_and_receive_same_actual_sample_on_both_sides(self):
        for side, wire_side in (('left', 1), ('right', 2)):
            with self.subTest(side=side):
                client, node, _ = self.client(side, read_only=True)
                waiting = threading.Event()
                calls, values, errors = [], [], []
                original_wait = client._condition.wait
                def wait(timeout):
                    calls.append(timeout)
                    if len(calls) == 2:
                        waiting.set()
                    return original_wait(timeout)
                def reader():
                    try:
                        values.append(client.wait_state(2))
                    except BaseException as error:
                        errors.append(error)
                with patch.object(client._condition, 'wait', side_effect=wait):
                    threads = [threading.Thread(target=reader) for _ in range(2)]
                    for thread in threads:
                        thread.start()
                    try:
                        self.assertTrue(waiting.wait(1))
                        node.emit(side, 'state', state_object(side=wire_side))
                        for thread in threads:
                            thread.join(1)
                    finally:
                        client.close()
                        for thread in threads:
                            thread.join(1)
                self.assertFalse(any(thread.is_alive() for thread in threads))
                self.assertEqual(errors, [])
                self.assertEqual(len(values), 2)
                self.assertIs(values[0], values[1])
                self.assertEqual(values[0].side, wire_side)
                self.assertTrue(all(0 < timeout <= 2 for timeout in calls))
                self.assertEqual(node.commands, [])
                self.assertEqual(node.close_count, 0)

    def test_latest_state_and_explicit_sample_ref_survive_feedback_burst(self):
        observed = []
        client, node, owner = self.client(
            observation_sources=((SampleStream.EE_STATE, 1),),
            on_state=lambda value: observed.append(value.state_id))
        for state_id in range(1, 401):
            state = state_object()
            state.stateId = state_id
            node.emit('left', 'state', state)
        self.assertEqual(client.wait_state(.1).state_id, 400)
        self.assertEqual(observed, list(range(1, 401)))
        self.assertEqual(node.commands, [])
        sample = client.state.sample_ref
        sent = client.publish(target(), input_observations=(sample,))
        wire = node.commands[-1][1]
        self.assertEqual((sent.input_observations[0].sample_id,
                          wire.InputObservations(0).SampleId()), (400, 400))
        self.assertEqual(sent.descriptor_ref, sample.descriptor_ref)
        # No independently generated session/clock and no polling-thread burst.
        self.assertEqual(sent.key.session_id, owner.generation)
        self.assertEqual((sent.sdk_time_valid_mask, sent.ts_client_send_us), (0, 0))
        handles = list(node.handles)
        client.close()
        self.assertTrue(all(handle.close_count == 1 for handle in handles))
        self.assertEqual((node.close_count, owner.transport.close_count), (0, 0))

    def test_wrong_side_feedback_and_wait_timeout_never_publish(self):
        client, node, _ = self.client()
        with self.assertRaises(TimeoutError):
            client.wait_state(.002)
        node.emit('left', 'state', state_object(side=2))
        with self.assertRaisesRegex(ValueError, 'invalid EE state'):
            client.wait_state(.1)
        self.assertEqual(node.commands, [])

    def test_nullable_command_intent_and_readonly_guard_use_shared_owner(self):
        client, node, _ = self.client()
        state = state_object(); state.stateId = 400
        node.emit('left', 'state', state)
        for torque in (None, 0., -.25):
            sent = client.publish(target(torque=torque))
            wire = node.commands[-1][1]
            self.assertEqual(bool(wire.CommandFields(0) & Field.TAU), torque is not None)
            self.assertEqual(wire.TauCmd(0), 0. if torque is None else torque)
            self.assertEqual(sent.target.tau_cmd, (0. if torque is None else torque,))
        readonly, readonly_node, owner = self.client(read_only=True)
        node_before = len(readonly_node.handles)
        requests_before = list(owner.transport.published)
        with self.assertRaisesRegex(RuntimeError, 'read-only'):
            readonly.publish(target())
        self.assertEqual(readonly_node.commands, [])
        self.assertEqual(len(readonly_node.handles), node_before)
        self.assertEqual(owner.transport.published, requests_before)
