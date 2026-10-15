"""DTO and client behavior with a fake adapter; no FlatBuffers/codegen proof."""
from dataclasses import replace
import math
from types import SimpleNamespace
import unittest

from locomotion_aorta.gripper import GripperClient, validate_target
from locomotion_aorta.gripper_types import GripperCmd, GripperInfo, GripperLimitSource, GripperSample, GripperState
from test_gripper_lifecycle import FakeNode, Handle, bindings


def info(side=1):
    return GripperInfo(1200, 1, '', '', side, 0., .1, .02, .5,
                       opening_min_source=3, opening_max_source=1, max_opening_rate_source=1,
                       torque_limit_nm=1., torque_limit_source=1, hold_torque_source=1)


class Publisher(Handle):
    def __init__(self, node, topic):
        super().__init__()
        self.node, self.topic = node, topic

    def publish_typed(self, fill):
        command = fill(None, None)  # fake DTO adapter, not FlatBuffers
        self.node.commands.append((self.topic, command))
        if self.node.publish_error:
            raise self.node.publish_error


class CommandNode(FakeNode):
    def __init__(self):
        super().__init__()
        self.commands = []
        self.publish_error = None
        self.publisher_count = 0

    def create_publisher_typed(self, meta, topic, qos):
        if meta is not COMMAND_META:
            raise AssertionError('unexpected metadata')
        self.publisher_count += 1
        handle = Publisher(self, topic)
        self.handles.append(handle)
        return handle


COMMAND_META = object()  # never presented as generated metadata


def wire():
    b = bindings()
    b.aorta.QoS.realtime_control = lambda: 'realtime'
    return SimpleNamespace(bindings=b, decode_info=lambda value: value, decode_state=lambda value: value,
                           command_meta=COMMAND_META,
                           fill_command=lambda command, builder, header: command)


class ClientTest(unittest.TestCase):
    def client(self, side='left', **options):
        n = CommandNode()
        c = GripperClient(side, node=n, _wire=wire(), **options)
        self.addCleanup(c.close)
        n.emit('info', info(c.wire_side), side)
        n.emit('state', GripperState(10, 1, c.wire_side, ts_state_real_us=4567,
                                    sample=GripperSample(.05, -.25, 0x40), sample_valid=True), side)
        return c, n

    def test_dq_zero_and_nullable_torque_are_preserved_not_replaced(self):
        c, n = self.client()
        omitted = c.publish_target(.06)
        zero = c.publish_target(.06, torque_cmd_nm=0.)
        signed = c.publish_target(.06, dq=.01, torque_cmd_nm=-.25)
        self.assertEqual([cmd.dq for _, cmd in n.commands], [0., 0., .01])
        self.assertIsNone(omitted.torque_cmd_nm)
        self.assertEqual(zero.torque_cmd_nm, 0.)
        self.assertEqual(signed.torque_cmd_nm, -.25)
        self.assertEqual(n.publisher_count, 1)
        self.assertEqual([cmd.sequence for _, cmd in n.commands], [1, 2, 3])
        self.assertEqual(omitted.input_state_ts_real_us, 4567)

    def test_each_side_sends_only_its_independent_topic(self):
        for side in ('left', 'right'):
            c, n = self.client(side)
            cmd = c.publish_target(.025)
            self.assertEqual(n.commands, [(f'/sdk/foot_humanoid/gripper/{side}/cmd', cmd)])
            self.assertEqual(cmd.side, c.wire_side)

    def test_rejects_out_of_range_nonfinite_and_boolean_payload_before_publisher(self):
        c, n = self.client()
        for q, dq, torque in ((-.01, 0., None), (.11, 0., None), (.05, -.01, None),
                (.05, .021, None), (math.nan, 0., None), (.05, math.inf, None),
                (True, 0., None), (.05, True, None), (.05, 0., True),
                (.05, 0., math.nan), (.05, 0., 1.001), (.05, 0., -1.001)):
            with self.subTest(q=q, dq=dq, torque=torque), self.assertRaises(ValueError):
                c.publish_target(q, dq, torque_cmd_nm=torque)
        self.assertEqual(n.publisher_count, 0)
        self.assertFalse(n.commands)

    def test_declared_endpoints_and_torque_bounds_are_legal(self):
        c, n = self.client()
        low = c.publish_target(0., 0., torque_cmd_nm=-1.)
        high = c.publish_target(.1, .02, torque_cmd_nm=1.)
        self.assertEqual((low.q, high.q), (0., .1))
        self.assertEqual((low.torque_cmd_nm, high.torque_cmd_nm), (-1., 1.))

    def test_unknown_capabilities_remain_readable_but_cannot_authorize_commands(self):
        c, n = self.client()
        unknown = replace(c.info, opening_max_source=GripperLimitSource.UNKNOWN)
        n.emit('info', unknown)
        self.assertIs(c.wait_info(), unknown)
        with self.assertRaisesRegex(ValueError, 'unknown sources'):
            c.publish_target(.05)
        self.assertFalse(n.commands)

    def test_invalid_config_torque_range_or_source_fails(self):
        for changed in ({'hold_torque_nm': 1.1}, {'torque_limit_nm': -1.},
                        {'torque_limit_source': 0}, {'hold_torque_source': 2}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                validate_target(replace(info(), **changed), .05)

    def test_readonly_creates_no_command_publisher(self):
        c, n = self.client(read_only=True)
        with self.assertRaisesRegex(RuntimeError, 'read-only'):
            c.publish_target(.05)
        self.assertEqual(n.publisher_count, 0)

    def test_transport_error_consumes_sequence_and_is_not_retried(self):
        c, n = self.client()
        n.publish_error = RuntimeError('send failed')
        with self.assertRaisesRegex(RuntimeError, 'send failed'):
            c.publish_target(.05)
        self.assertEqual(len(n.commands), 1)
        n.publish_error = None
        second = c.publish_target(.05)
        self.assertEqual(second.sequence, 2)
        self.assertEqual(len(n.commands), 2)

    def test_publisher_and_subscriptions_closed_without_closing_shared_node(self):
        c, n = self.client()
        c.publish_target(.05)
        c.close()
        self.assertTrue(all(h.closed for h in n.handles))
        self.assertFalse(n.closed)
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            c.publish_target(.05)

    def test_invalid_sample_and_independent_trace_target_keys_are_not_fabricated(self):
        c, n = self.client()
        value = GripperState(11, 1, 1, sample=None, sample_valid=False,
                             trace_session_id=2, trace_sequence=10, ts_cmd_received_us=800,
                             host_target_valid=False, host_target_session_id=7, host_target_sequence=3)
        n.emit('state', value)
        self.assertIs(c.wait_state(after_state_id=10), value)
        self.assertIsNone(c.state.sample)
        self.assertEqual((c.state.trace_session_id, c.state.host_target_session_id), (2, 7))
        self.assertEqual((c.state.ts_cmd_accepted_us, c.state.ts_cmd_first_output_us,
                          c.state.ts_cmd_write_ack_us, c.state.ts_cmd_first_feedback_us), (0, 0, 0, 0))


if __name__ == '__main__':
    unittest.main(verbosity=2)
