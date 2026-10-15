"""Fake-node lifecycle tests only: no wire layout, generated code or transport."""
from dataclasses import dataclass, replace
import math
import threading
from types import SimpleNamespace
import unittest

from locomotion_aorta._gripper_lifecycle import GripperClientLifecycle


@dataclass(frozen=True)
class Info:
    side: int = 1
    gripper_type: int = 1
    ts_info_us: int = 1200


@dataclass(frozen=True)
class State:
    side: int = 1
    gripper_type: int = 1
    state_id: int = 10
    ts_state_real_us: int = 4567


class Handle:
    def __init__(self):
        self.closed = False
        self.error = None

    def close(self):
        self.closed = True
        if self.error:
            raise self.error


class FakeNode:
    def __init__(self):
        self.handlers = {}
        self.qos = {}
        self.handles = []
        self.group_name = 'default'
        self.closed = False
        self.stamp = 1_000_000
        self.fail_subscription = None

    def group(self):
        return self.group_name

    def now(self):
        self.stamp += 1000
        return self.stamp

    def create_subscriber_typed(self, decoder, topic, handler, qos):
        if len(self.handles) == self.fail_subscription:
            raise RuntimeError('subscription failed')
        self.handlers[topic] = lambda value: handler(decoder(value), None)
        self.qos[topic] = qos
        handle = Handle()
        self.handles.append(handle)
        return handle

    def create_publisher_typed(self, *args, **kwargs):
        raise AssertionError('lifecycle must not create publishers')

    def create_client_typed(self, *args, **kwargs):
        raise AssertionError('lifecycle must not create RPC clients')

    def close(self):
        self.closed = True

    def emit(self, kind, value, side='left'):
        self.handlers[f'/sdk/foot_humanoid/gripper/{side}/{kind}'](value)


def bindings():
    identity = SimpleNamespace(GetRootAs=lambda value: value)
    qos = SimpleNamespace(sensor_data=lambda: 'sensor', state_update=lambda: 'state_update')
    return SimpleNamespace(group='default', aorta=SimpleNamespace(QoS=qos),
                           GripperState=SimpleNamespace(GripperState=identity),
                           GripperInfo=SimpleNamespace(GripperInfo=identity))


class LifecycleTest(unittest.TestCase):
    def client(self, side='left', node=None, **options):
        node = node or FakeNode()
        client = GripperClientLifecycle(side, node=node, bindings=bindings(),
                                       decode_info=options.pop('decode_info', lambda value: value),
                                       decode_state=options.pop('decode_state', lambda value: value),
                                       **options)
        self.addCleanup(client.close)
        return client, node

    def ready(self, **options):
        c, n = self.client(**options)
        side = c.wire_side
        n.emit('info', Info(side=side), c.side)
        n.emit('state', State(side=side), c.side)
        return c, n

    def test_side_names_route_independently_with_expected_qos(self):
        node = FakeNode()
        left, _ = self.client(node=node)
        right, _ = self.client('right', node=node)
        self.assertNotEqual(left.session_id, right.session_id)
        self.assertEqual((left.wire_side, right.wire_side), (1, 2))
        self.assertEqual(node.qos, {
            '/sdk/foot_humanoid/gripper/left/info': 'state_update',
            '/sdk/foot_humanoid/gripper/right/info': 'state_update',
            '/sdk/foot_humanoid/gripper/left/state': 'state_update',
            '/sdk/foot_humanoid/gripper/right/state': 'state_update'})
        with self.assertRaises(ValueError):
            self.client(1)

    def test_late_info_waits_for_event_without_rpc_or_command(self):
        c, n = self.client(read_only=True)
        with self.assertRaises(TimeoutError):
            c.wait_info(.001)
        info = Info()
        n.emit('info', info)
        self.assertIs(c.wait_info(), info)
        with self.assertRaises(TimeoutError):
            c.wait_info(.001, after_info=info)
        newer = replace(info)
        n.emit('info', newer)
        self.assertIs(c.wait_info(after_info=info), newer)
        self.assertEqual(newer.ts_info_us, info.ts_info_us)

    def test_info_required_and_readonly_cannot_reserve(self):
        c, n = self.client()
        with self.assertRaisesRegex(ValueError, 'GripperInfo'):
            c.reserve_envelope()
        readonly, _ = self.ready(read_only=True)
        with self.assertRaisesRegex(RuntimeError, 'read-only'):
            readonly.reserve_envelope()
        self.assertEqual(readonly._sequence, 0)

    def test_envelope_uses_real_input_and_separate_client_clock(self):
        c, n = self.ready(_session_id=99)
        envelope = c.reserve_envelope(cmd_id=7)
        self.assertEqual((envelope.session_id, envelope.sequence, envelope.cmd_id), (99, 1, 7))
        self.assertEqual((envelope.input_state_id, envelope.input_state_ts_real_us), (10, 4567))
        self.assertEqual((envelope.ts_client_created_us, envelope.ts_client_send_us), (1001, 1002))
        self.assertEqual((envelope.protocol_version, envelope.side, envelope.gripper_type), (1, 1, 1))
        self.assertEqual(c.reserve_envelope().sequence, 2)

    def test_no_state_keeps_unknown_input_zero(self):
        c, n = self.client()
        n.emit('info', Info())
        command = c.reserve_envelope()
        self.assertEqual((command.input_state_id, command.input_state_ts_real_us), (0, 0))

    def test_labels_wrap_without_sequence_reuse_and_no_implicit_new_session(self):
        c, _ = self.ready()
        c._sequence = 65535
        command = c.reserve_envelope()
        self.assertEqual((command.sequence, command.cmd_id), (65536, 0))
        original_session = c.session_id
        c._sequence = (1 << 64) - 1
        with self.assertRaises(OverflowError):
            c.reserve_envelope()
        self.assertEqual(c.session_id, original_session)

    def test_invalid_identity_does_not_reserve_or_change_session(self):
        for bad in (True, 0, -1, 1 << 64, 1.5):
            with self.subTest(session=bad), self.assertRaises(ValueError):
                self.client(_session_id=bad)
        c, _ = self.ready()
        for label in (True, -1, 65536, .5):
            with self.subTest(label=label), self.assertRaises(ValueError):
                c.reserve_envelope(cmd_id=label)
        for state in (State(side=2), State(gripper_type=2), State(state_id=-1),
                      State(ts_state_real_us=True)):
            with self.subTest(state=state), self.assertRaises(ValueError):
                c.reserve_envelope(input_state=state)
        self.assertEqual(c._sequence, 0)

    def test_client_timestamp_range_is_checked_without_burning_sequence(self):
        c, n = self.ready()
        n.stamp = -2000
        with self.assertRaisesRegex(ValueError, 'ts_client_created'):
            c.reserve_envelope()
        self.assertEqual(c._sequence, 0)

    def test_new_state_requires_strictly_later_id(self):
        c, n = self.ready()
        with self.assertRaises(TimeoutError):
            c.wait_state(.001, after_state_id=10)
        n.emit('state', State(state_id=11))
        self.assertEqual(c.wait_state(after_state_id=10).state_id, 11)
        for timeout in (True, 0, -1, math.inf, math.nan):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                c.wait_state(timeout)

    def test_decoder_and_callback_errors_wake_readers(self):
        def broken(_view):
            raise ValueError('sample decode failed')
        c, n = self.client(decode_state=broken)
        n.emit('state', State())
        with self.assertRaisesRegex(RuntimeError, 'invalid gripper feedback') as raised:
            c.wait_state()
        self.assertEqual(str(raised.exception.__cause__), 'sample decode failed')
        d, m = self.client(on_info=broken)
        m.emit('info', Info())
        with self.assertRaisesRegex(RuntimeError, 'invalid gripper feedback'):
            d.wait_info()

    def test_wrong_side_feedback_is_never_used(self):
        c, n = self.client()
        n.emit('info', Info(side=2))
        with self.assertRaisesRegex(RuntimeError, 'invalid gripper feedback'):
            _ = c.info

    def test_close_wakes_waiter_and_shared_node_stays_open(self):
        c, n = self.client()
        errors = []
        started = threading.Event()
        def wait():
            started.set()
            try:
                c.wait_state(5)
            except Exception as error:
                errors.append(error)
        thread = threading.Thread(target=wait)
        thread.start()
        self.assertTrue(started.wait(1))
        c.close()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIn('closed', str(errors[0]))
        self.assertFalse(n.closed)
        self.assertTrue(all(h.closed for h in n.handles))
        n.emit('state', State())
        c.close()

    def test_failed_construction_closes_own_node_and_partial_subscription(self):
        node = FakeNode()
        node.fail_subscription = 1
        with self.assertRaisesRegex(RuntimeError, 'subscription failed'):
            self.client(node=node, owns_node=True)
        self.assertTrue(node.closed)
        self.assertTrue(all(h.closed for h in node.handles))
        shared = FakeNode()
        shared.group_name = 'wrong'
        with self.assertRaisesRegex(ValueError, 'group differs'):
            self.client(node=shared)
        self.assertFalse(shared.closed)

    def test_cleanup_continues_after_handle_error(self):
        c, n = self.client(owns_node=True)
        n.handles[0].error = RuntimeError('close failed')
        with self.assertRaisesRegex(RuntimeError, 'close failed'):
            c.close()
        self.assertTrue(all(h.closed for h in n.handles))
        self.assertTrue(n.closed)


if __name__ == '__main__':
    unittest.main(verbosity=2)
