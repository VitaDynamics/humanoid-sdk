"""Real EE wire bytes over keep-last rings; no Node or Python RX worker."""
from collections import deque
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from locomotion_aorta import GripperClient, HandClient
from test_hand import FakeNode as HandNode, pack, info_obj, state_obj
from test_gripper_codec import RealPacketNode
from lowlevel.GripperInfo import GripperInfoT
from lowlevel.GripperState import GripperStateT
from lowlevel.GripperSample import GripperSampleT


class Ring:
    def __init__(self):
        self.queue = deque(maxlen=1)
        self.closed = False

    def put(self, payload):
        self.queue.append(NS(payload=payload))

    def try_recv(self):
        return self.queue.popleft() if self.queue else None

    def recv(self, timeout):
        sample = self.try_recv()
        if sample is None:
            time.sleep(timeout)
        return sample

    def close(self):
        self.closed = True


class PullNode:
    def create_subscriber_pull(self, topic, *, depth, qos):
        if depth != 1:
            raise AssertionError('feedback ring must keep only latest')
        ring = Ring()
        self.rings[topic] = ring
        self.handles.append(ring)
        return ring

    def create_subscriber_typed(self, *args, **kwargs):
        raise AssertionError('pull feedback must not create a callback subscriber')


class PullHandNode(PullNode, HandNode):
    def __init__(self):
        super().__init__()
        self.rings = {}


class PullGripperNode(PullNode, RealPacketNode):
    def __init__(self):
        super().__init__()
        self.rings = {}


class EEPullTest(unittest.TestCase):
    def test_contending_waiters_sleep_on_condition_for_both_clients(self):
        for factory, node in ((HandClient, PullHandNode()),
                              (GripperClient, PullGripperNode())):
            with self.subTest(client=factory.__name__):
                client = factory('left', node=node, pull_feedback=True, read_only=True)
                self.addCleanup(client.close)
                pulls = client._pulls
                owner = client
                locked, release = threading.Event(), threading.Event()
                def hold():
                    with pulls._lock:
                        locked.set()
                        release.wait(1)
                thread = threading.Thread(target=hold)
                thread.start()
                self.assertTrue(locked.wait(.5))
                expected = NS(state_id=7)
                def received(_timeout):
                    owner._latest['state'] = expected
                try:
                    with patch.object(owner._condition, 'wait', side_effect=received) as wait:
                        self.assertIs(client.wait_state(.1), expected)
                    wait.assert_called_once()
                    self.assertLessEqual(wait.call_args.args[0], .05)
                finally:
                    release.set()
                    thread.join(1)
                self.assertFalse(thread.is_alive())

    def hand(self, **kw):
        node = PullHandNode()
        with patch('threading.Thread', side_effect=AssertionError('Python worker created')):
            client = HandClient('left', node=node, pull_feedback=True, **kw)
        self.addCleanup(client.close)
        node.rings[client.topic+'/info'].put(pack(info_obj()))
        return client, node

    def test_hand_latest_info_state_publish_and_inline_callback(self):
        calls = []
        client, node = self.hand(on_state=lambda value: calls.append((threading.get_ident(), value.state_id)))
        for state_id in range(1, 401):
            obj = state_obj(); obj.stateId = state_id
            node.rings[client.topic+'/state'].put(pack(obj))
        self.assertEqual(client.wait_info(.1).dof, 6)
        self.assertEqual(client.wait_state(.1).state_id, 400)
        self.assertEqual(calls, [(threading.get_ident(), 400)])
        newer = state_obj(); newer.stateId = 401
        node.rings[client.topic+'/state'].put(pack(newer))
        self.assertEqual(client.wait_state(.1).state_id, 401)
        self.assertEqual(len(node.rings), 3)
        cmd = client.publish([.01]*6)
        self.assertEqual(cmd.input_state_id, 401)
        actual = node.commands[-1][1]
        self.assertEqual((actual.session_id, actual.sequence, actual.input_state_id),
                         (cmd.session_id, cmd.sequence, 401))
        for joint in actual.joints:
            self.assertAlmostEqual(joint.q, .01)
        before = client.info
        node.rings[client.topic+'/info'].put(pack(info_obj()))
        self.assertIsNot(client.wait_info(.1, after_info=before), before)
        client.close()
        self.assertTrue(all(r.closed for r in node.rings.values()))
        self.assertFalse(node.closed)  # Shared Node lifetime stays with caller.

    def test_hand_wrong_side_still_poisoned_and_timeout_does_not_publish(self):
        client, node = self.hand()
        with self.assertRaises(TimeoutError):
            client.wait_state(.002)
        self.assertEqual(node.commands, [])
        node.rings[client.topic+'/state'].put(pack(state_obj(2)))
        with self.assertRaisesRegex(RuntimeError, 'invalid hand feedback'):
            client.wait_state(.1)

    def test_gripper_latest_state_nullable_command_and_readonly_guard(self):
        node = PullGripperNode()
        with patch('threading.Thread', side_effect=AssertionError('Python worker created')):
            client = GripperClient('left', node=node, pull_feedback=True)
        self.addCleanup(client.close)
        obj = GripperInfoT(side=1, gripperType=1, openingMinM=0., openingMaxM=.1,
            maxOpeningRateMS=.1, holdTorqueNm=.2, torqueLimitNm=.5,
            openingMinSource=1, openingMaxSource=1, maxOpeningRateSource=1,
            holdTorqueSource=1, torqueLimitSource=1)
        node.rings[client.topic+'/info'].put(pack(obj))
        for state_id in range(1, 401):
            node.rings[client.topic+'/state'].put(pack(GripperStateT(
                stateId=state_id, side=1, gripperType=1, sampleValid=True,
                sample=GripperSampleT(q=.05, reportedTorqueNm=-.25, statusRaw=0x40))))
        self.assertEqual(client.wait_info(.1).side, 1)
        self.assertEqual(client.wait_state(.1).state_id, 400)
        from lowlevel.GripperCmd import GripperCmd
        for torque in (None, 0., -.25):
            client.publish_target(.0625, torque_cmd_nm=torque)
            wire = GripperCmd.GetRootAs(node.payloads[-1])
            self.assertEqual(wire.TorqueCmdNm(), torque)
            self.assertEqual(wire.InputStateId(), 400)
        client.close()
        self.assertTrue(all(r.closed for r in node.rings.values()))
        readonly = GripperClient('left', node=PullGripperNode(), pull_feedback=True, read_only=True)
        self.addCleanup(readonly.close)
        with self.assertRaisesRegex(RuntimeError, 'read-only'):
            readonly.publish_target(.05)
