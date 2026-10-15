"""Real generated encoding/decoding with mocked Node/RPC; no transport is opened."""
from dataclasses import replace
import importlib
import math
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import flatbuffers
from locomotion_aorta import HandClient, HandSide, InhibitReason
from locomotion_aorta import _hand_codec as codec
from lowlevel.HandInfo import HandInfoT
from lowlevel.HandState import HandStateT
from lowlevel.HandJointState import HandJointStateT
from lowlevel.HandTouch import HandTouchT
from lowlevel.FingerTouchMeta import FingerTouchMetaT
from aorta.services.humanoid_locomotion.HandSetTouchResponse import HandSetTouchResponseT


def pack(obj):
    builder = flatbuffers.Builder(2048)
    builder.Finish(obj.Pack(builder))
    return bytes(builder.Output())


def info_obj(side=1):
    return HandInfoT(tsInfoUs=1200, tsStatePubUs=1300, handType=1, firmware='synthetic',
                     side=side, dof=6, jointNames=['thumb_pitch', 'thumb_roll', 'index', 'middle', 'ring', 'little'],
                     qMin=[0.] * 6, qMax=[.02] * 6, dqMax=[math.radians(1.)] * 6,
                     touchAvailable=True, touchEnabled=True, touchFingers=5, touchWordsPerFinger=6)


def state_obj(side=1):
    return HandStateT(stateId=9, handType=1, side=side,
                      tsStateRealUs=4900, tsStatePubUs=5100,
                      tsAcquiredHwUs=17, tsIngestedUs=4900, tsAssembledUs=5000,
                      outputAllowed=True, inhibitReason=0,
                      lastCmdSessionId=11, lastCmdSequence=7, lastCmdId=3,
                      estimatedInputPeriodUs=20000, rejectCount=0, lastRejectCode=0, ingressDropCount=2,
                      tsCmdReceivedUs=4100, tsCmdAcceptedUs=4150, tsCmdFirstOutputUs=4200,
                      tsCmdWriteAckUs=4300, tsCmdFirstFeedbackUs=4900,
                      timeoutCount=4, crcErrorCount=5,
                      joints=[HandJointStateT(q=.01, dq=.005, currentMa=12.5, qCmd=.011, dqCmd=.017,
                              qRaw=100, dqRaw=200, currentRaw=300, statusRaw=7) for _ in range(6)])


class Handle:
    def __init__(self):
        self.closed = False
    def close(self):
        self.closed = True


class FakePublisher(Handle):
    def __init__(self, node, topic):
        super().__init__()
        self.node, self.topic = node, topic
    def publish_typed(self, fill):
        builder = flatbuffers.Builder(512)
        builder.Finish(fill(builder, 0))
        view = self.node.bindings.HandCmd.HandCmd.GetRootAs(builder.Output())
        self.node.commands.append((self.topic, codec.decode_cmd(view)))
        if self.node.publish_error:
            raise self.node.publish_error


class FakeClient(Handle):
    def __init__(self, node, topic, decoder):
        super().__init__()
        self.node, self.topic, self.decoder = node, topic, decoder
    def raw(self):
        return self
    def wait_for_matching(self, timeout):
        return self.node.matching
    def call(self, fill):
        cls = self.node.bindings.HandSetTouchRequest.HandSetTouchRequest
        builder = flatbuffers.Builder(256)
        builder.Finish(fill(builder, 0))
        request = cls.GetRootAs(builder.Output())
        self.node.touch_requests.append(request.Enable())
        response = HandSetTouchResponseT(status=self.node.service_status, enabled=request.Enable())
        result = self.decoder(pack(response))
        return SimpleNamespace(decode=lambda: result)


class FakeNode:
    def __init__(self):
        self.bindings = codec.load_bindings()
        self.subscriptions = {}
        self.handles = []
        self.commands, self.touch_requests, self.client_options = [], [], []
        self.stamp = 1000000
        self.publish_error = None
        self.matching, self.closed = True, False
        self.service_status = 0
    def group(self):
        return 'default'
    def now(self):
        self.stamp += 1000
        return self.stamp
    def create_subscriber_typed(self, decoder, topic, handler, qos):
        self.subscriptions[topic] = decoder, handler
        handle = Handle(); self.handles.append(handle)
        return handle
    def create_publisher_typed(self, meta, topic, qos):
        assert meta.ROOT_TYPE_NAME == 'lowlevel.HandCmd'
        handle = FakePublisher(self, topic); self.handles.append(handle)
        return handle
    def create_client_typed(self, topic, decoder, request_schema_meta, options):
        assert request_schema_meta.ROOT_TYPE_NAME.endswith('HandSetTouchRequest')
        self.client_options.append(options)
        handle = FakeClient(self, topic, decoder); self.handles.append(handle)
        return handle
    def emit(self, side, kind, obj):
        decoder, handler = self.subscriptions[f'/sdk/foot_humanoid/hand/{side}/{kind}']
        handler(decoder(pack(obj)), None)
    def close(self):
        self.closed = True


class HandTest(unittest.TestCase):
    def client(self, side='left', **kwargs):
        node = FakeNode()
        client = HandClient(side, node=node, **kwargs)
        self.addCleanup(client.close)
        wire_side = 1 if side == 'left' else 2
        node.emit(side, 'info', info_obj(wire_side))
        node.emit(side, 'state', state_obj(wire_side))
        return client, node

    def test_wire_fields_units_vitanow_and_independent_sessions(self):
        node = FakeNode()
        clients = [HandClient(side, node=node) for side in ('left', 'right')]
        for c in clients:
            self.addCleanup(c.close)
            side = int(c.wire_side)
            node.emit(c.side, 'info', info_obj(side)); node.emit(c.side, 'state', state_obj(side))
            first = c.publish([.0125]*6, [math.radians(.5)]*6, cmd_id=123)
            second = c.publish([.01]*6)
            self.assertEqual((first.sequence, second.sequence), (1, 2))
            self.assertEqual((first.protocol_version, first.hand_type, first.side), (1, 1, side))
            self.assertEqual((first.input_state_id, first.input_state_ts_real_us), (9, 4900))
            self.assertEqual(first.cmd_id, 123)
            decoded = node.commands[-2][1]
            self.assertEqual(decoded.session_id, c.session_id)
            self.assertEqual(decoded.ts_client_created_us, first.ts_client_created_us)
            self.assertEqual(decoded.ts_client_send_us - decoded.ts_client_created_us, 1)
            self.assertAlmostEqual(decoded.joints[0].q, .0125)
            self.assertAlmostEqual(decoded.joints[0].dq, math.radians(.5))
        self.assertNotEqual(clients[0].session_id, clients[1].session_id)
        self.assertEqual({topic for topic, _ in node.commands}, {
            '/sdk/foot_humanoid/hand/left/cmd', '/sdk/foot_humanoid/hand/right/cmd'})

    def test_all_state_fields_and_nan_are_preserved(self):
        client, node = self.client()
        obj = state_obj(); obj.joints[0].q = float('nan'); obj.inhibitReason = 4
        node.emit('left', 'state', obj)
        s = client.state
        self.assertTrue(math.isnan(s.joints[0].q))
        self.assertEqual((s.joints[0].current_ma, s.joints[0].q_raw, s.joints[0].status_raw), (12.5, 100, 7))
        self.assertEqual((s.ts_acquired_hw_us, s.ts_ingested_us, s.ts_assembled_us), (17, 4900, 5000))
        self.assertEqual(s.ts_cmd_received_us, 4100)
        self.assertEqual((s.ts_cmd_write_ack_us, s.ts_cmd_first_feedback_us), (4300, 4900))
        self.assertEqual((s.ingress_drop_count, s.timeout_count, s.crc_error_count), (2, 4, 5))
        self.assertEqual(s.last_valid_age_us, (1 << 64) - 1)

    def test_touch_raw_geometry_timestamps_and_callback(self):
        seen = []
        c, n = self.client(on_touch=seen.append)
        obj = HandTouchT(stateId=18, handType=1, side=1, sensorType=1, tsStatePubUs=9000,
                        fingers=[FingerTouchMetaT(tsRequestUs=8000, tsAcquiredHwUs=17, tsIngestedUs=8900,
                                    sampleSeq=2, statusFlags=4, directionFlags=3) for _ in range(5)],
                        rawWords=[0, 256, 65535, 0, 0, 0] * 5)
        n.emit('left', 'touch', obj)
        self.assertEqual(c.wait_touch().raw_words[:3], (0, 256, 65535))
        self.assertEqual(c.touch.fingers[0].ts_acquired_hw_us, 17)
        self.assertEqual(seen, [c.touch])
        obj.rawWords.pop()
        n.emit('left', 'touch', obj)
        with self.assertRaisesRegex(RuntimeError, 'invalid hand feedback'):
            c.wait_touch()

    def test_set_touch_real_requests_and_service_failures(self):
        c, n = self.client()
        info = c.wait_info(.25)
        self.assertEqual(info.hand_type, 1)
        self.assertEqual(info.ts_state_pub_us, 1300)
        self.assertEqual(info.joint_names[0], 'thumb_pitch')
        self.assertTrue(c.set_touch(True, .25)); self.assertFalse(c.set_touch(False, .25))
        self.assertEqual(n.touch_requests, [True, False])
        n.service_status = 4
        with self.assertRaisesRegex(RuntimeError, 'ServiceStatus=4'):
            c.set_touch(True)
        n.matching = False
        with self.assertRaises(TimeoutError):
            c.set_touch(True, .1)

    def test_late_subscription_waits_for_info_without_rpc(self):
        node = FakeNode()
        client = HandClient('left', node=node, read_only=True)
        self.addCleanup(client.close)
        with self.assertRaises(TimeoutError):
            client.wait_info(.001)
        node.emit('left', 'info', info_obj())
        self.assertIs(client.wait_info(), client.info)
        self.assertEqual(node.client_options, [])
        self.assertEqual(node.commands, [])

    def test_later_info_event_works_with_unchanged_timestamp(self):
        client, node = self.client()
        cached = client.info
        with self.assertRaises(TimeoutError):
            client.wait_info(.001, after_info=cached)
        self.assertFalse(client.set_touch(False))
        # The cached capabilities remain available to command publication.
        self.assertIs(client.info, cached)
        obj = info_obj(); obj.touchEnabled = False
        node.emit('left', 'info', obj)
        later = client.wait_info(after_info=cached)
        self.assertIsNot(later, cached)
        self.assertEqual(later.ts_info_us, cached.ts_info_us)
        self.assertFalse(later.touch_enabled)

    def test_invalid_commands_and_sides_never_publish(self):
        c, n = self.client()
        for q, dq in (([0]*5, None), ([float('nan')]*6, None), ([.03]*6, None),
                      ([.01]*6, [-1]*6), ([.01]*6, [1.]*6), ([True]*6, None)):
            with self.subTest(q=q, dq=dq), self.assertRaises(ValueError):
                c.publish(q, dq)
        with self.assertRaises(ValueError):
            c.publish([.01]*6, input_state=replace(c.state, side=2))
        self.assertEqual(n.commands, [])
        for side in (1, 2, 'both', 'LEFT'):
            with self.assertRaises(ValueError):
                HandClient(side, node=n)

    def test_failed_publish_consumes_sequence_and_threads_do_not_duplicate(self):
        c, n = self.client(); n.publish_error = RuntimeError('mock transport')
        with self.assertRaises(RuntimeError):
            c.publish([.01]*6)
        n.publish_error = None
        threads = [threading.Thread(target=lambda: c.publish([.01]*6)) for _ in range(8)]
        for t in threads: t.start()
        for t in threads: t.join()
        self.assertEqual([command.sequence for _, command in n.commands], list(range(1, 10)))

    def test_readonly_close_shared_node_and_wait_timeout(self):
        c, n = self.client(read_only=True)
        c.wait_info()
        with self.assertRaises(RuntimeError): c.publish([.01]*6)
        with self.assertRaises(RuntimeError): c.set_touch(True)
        with self.assertRaises(TimeoutError): c.wait_state(.001, after_state_id=9)
        c.close(); c.close()
        self.assertFalse(n.closed)
        self.assertTrue(all(handle.closed for handle in n.handles))
        self.assertFalse(n.commands)
        self.assertFalse(n.touch_requests)
        with self.assertRaises(RuntimeError): c.wait_info()

    def test_feedback_wrong_side_and_joint_count_fail_explicitly(self):
        for obj in (state_obj(2), HandStateT(stateId=2, side=1, joints=[])):
            c, n = self.client(); n.emit('left', 'state', obj)
            with self.assertRaises(RuntimeError): _ = c.state

    def test_joint_count_is_checked_against_later_info(self):
        n = FakeNode(); c = HandClient('left', node=n)
        self.addCleanup(c.close)
        n.emit('left', 'state', HandStateT(stateId=1, handType=1, side=1, joints=[]))
        self.assertEqual(c.state.joints, ())
        n.emit('left', 'info', info_obj())
        with self.assertRaisesRegex(RuntimeError, 'invalid hand feedback'):
            c.wait_info()

    def test_missing_bindings_do_not_open_node(self):
        with patch.object(codec.importlib, 'import_module', side_effect=ImportError('missing')):
            with self.assertRaisesRegex(RuntimeError, 'vbot-humanoid-msgs'):
                HandClient('left')

    def test_owned_node_cleanup_and_group_mismatch(self):
        node = FakeNode()
        with patch.object(node.bindings.aorta, 'Node', return_value=node):
            client = HandClient('left')
            client.close()
        self.assertTrue(node.closed)
        self.assertTrue(all(handle.closed for handle in node.handles))
        wrong = FakeNode(); wrong.group = lambda: 'other'
        with patch.object(wrong.bindings.aorta, 'Node', return_value=wrong):
            with self.assertRaisesRegex(ValueError, 'group differs'):
                HandClient('left')
        self.assertTrue(wrong.closed)
        self.assertFalse(wrong.subscriptions)
