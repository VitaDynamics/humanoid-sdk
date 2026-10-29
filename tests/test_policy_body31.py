"""Roster, exact snapshot clocks and public-record replay; never opens transport."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
try:
    import numpy as np
except ImportError:
    raise unittest.SkipTest('install the policy demo extras to run body31 policy tests')

from locomotion_aorta import BodyJointState, BodyState
from locomotion_aorta._policy_state import ASAMPLE_BODY_JOINT_NAMES, PolicyFeedback, PolicyStateSource
from locomotion_aorta.wbc_tracking_policy import Model
from test_wbc_tracking_policy import policy, example


class PublicPolicyStateTest(unittest.TestCase):
    def test_named_roster_permutation_preserves_actor_and_neck_targets(self):
        p = policy()
        before, _ = p.decode(np.linspace(-1,1,28))
        old = dict(zip(p.order, before.joints))
        names = tuple(reversed(p.order))
        p.bind_body_info(SimpleNamespace(joint_names=names,
            joints=[SimpleNamespace(body_index=i, is_mock=name == 'waist_roll_joint',
                                    supported_fields=31, valid_limits=1, q_min=-3., q_max=3.)
                    for i, name in enumerate(names)]))
        after, _ = p.decode(np.linspace(-1,1,28))
        self.assertEqual(dict(zip(names, after.joints)), old)
        self.assertEqual(after.joints[names.index('waist_roll_joint')].kp, 0.)

    def test_snapshot_uses_actual_callback_clocks_and_uint64(self):
        state = BodyState(2**64-1, (BodyJointState(),)*31, ts_state_real_us=123,
                          applied_session_id=2**63, applied_sequence=2**64-1, ts_first_output_us=42)
        snapshots = [SimpleNamespace(body_state=lambda: state, received_monotonic_ns=i,
                                     received_realtime_ns=100+i) for i in (8,9)]
        previous = []
        def receive(timeout, *, after):
            previous.append(after)
            return snapshots[len(previous)-1]
        stream = PolicyStateSource(SimpleNamespace(snapshot=receive))
        first, second = stream.wait_body_state(1), stream.wait_body_state(1)
        self.assertEqual(previous, [None, snapshots[0]])
        self.assertEqual((first.state_id, first.stamp_ns, first.received_monotonic_ns,
                          first.received_realtime_ns), (2**64-1,123000,8,108))
        self.assertEqual((second.state_id, second.applied_sequence), (first.state_id, 2**64-1))
        # New reception alone is not a progressing robot source timestamp.
        self.assertEqual(first.stamp_ns, second.stamp_ns)

    def test_wbc_lazy_native_model_uses_metadata_and_explicit_threads(self):
        p = policy()
        metadata = {'groot_wbc.tracking_contract.v1': (Path(__file__).parent/'fixtures/actor_g1_contract.json').read_text()}
        calls = []
        class Session:
            def __init__(self, path, *, intra_threads, inter_threads):
                calls.append((path,intra_threads,inter_threads))
                self.metadata = metadata
                self.inputs = [SimpleNamespace(name='obs',shape=[1,p.contract.input_dim],dtype='tensor(float)')]
                self.outputs = [SimpleNamespace(name='actions',shape=[1,28],dtype='tensor(float)')]
            def run(self, input_name, output_name, values):
                self.call = (input_name,output_name,values.shape,values.dtype)
                return np.zeros((1,28),np.float32)
        with patch.dict(sys.modules, {'locomotion_aorta._onnx': SimpleNamespace(Session=Session)}):
            model = Model('/model.onnx',3)
        self.assertEqual(calls, [('/model.onnx',3,1)])
        self.assertEqual(model.infer(np.zeros(p.contract.input_dim,np.float32)).shape, (28,))
        self.assertEqual(model.session.call[:3], ('obs','actions',(1,p.contract.input_dim)))

    @unittest.skipUnless(importlib.util.find_spec('yaml'), 'install .[velocity-demo] for PyYAML')
    def test_velocity_lazy_native_model_and_public_roster(self):
        from examples.velocity_mode.velocity_mode_policy import VelocityModePolicy
        import yaml
        metadata = json.loads((Path(__file__).parent/'fixtures/velocity_model_metadata.json').read_text())
        calls = []
        class Session:
            def __init__(self, path, *, intra_threads, inter_threads):
                calls.append((intra_threads,inter_threads))
                self.metadata = metadata
                self.inputs = [SimpleNamespace(shape=[1,465])]
            def run(self, input_name, output_name, values):
                self.call = (input_name,output_name,values.dtype,values.shape)
                return np.zeros((1,28),np.float32)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'actions/policy_profiles').mkdir(parents=True)
            cfg = {'humanoid_velocity': {'model_path':'model.onnx',
                    'fixed_joints':{n:{'q':0.,'kp':20.,'kd':1.} for n in ('neck_pitch_joint','neck_yaw_joint')},
                    'command':{'smoothing':{'enabled':False}}}}
            (root/'actions/policy_profiles/vt_human_v2_velocity_asample.yaml').write_text(yaml.safe_dump(cfg))
            (root/'model.onnx').write_bytes(b'mocked model; metadata fixture is from 7978651a')
            with patch.dict(sys.modules, {'locomotion_aorta._onnx': SimpleNamespace(Session=Session)}):
                p = VelocityModePolicy(root)
            before = dict(zip(p.order,p.decode(np.zeros(28,np.float32))))
            names = tuple(reversed(ASAMPLE_BODY_JOINT_NAMES))
            p.bind_body_info(SimpleNamespace(joint_names=names,
                joints=[SimpleNamespace(body_index=i, is_mock=name == 'waist_roll_joint',
                                        supported_fields=31, valid_limits=1, q_min=-3., q_max=3.)
                        for i, name in enumerate(names)]))
            self.assertEqual(dict(zip(p.order,p.decode(np.zeros(28,np.float32)))), before)
            self.assertEqual(p.inventory['body_joint_names'], p.order)
            self.assertEqual(p.inventory['policy_to_body_slot'], p.policy_to_body)
            self.assertEqual(p.inventory['fixed_joints'], cfg['humanoid_velocity']['fixed_joints'])
            self.assertEqual(len(p.infer(np.zeros((1,465),np.float32))),28)
        self.assertEqual(calls,[(1,1)])
        self.assertEqual(p.session.call[2:],(np.dtype('float32'),(1,465)))
        self.assertNotIn(p.order.index('waist_roll_joint'),p.driven_slots)

    def test_public_mcap_replay_uses_real_binding_and_keeps_unknown_receipt_clocks(self):
        import body_state_schema_meta
        import flatbuffers
        from lowlevel.BodyState import BodyStateT
        from lowlevel.BodyJointState import BodyJointStateT
        from locomotion_aorta import BODY_STATE_TOPIC
        from mcap.writer import Writer
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'body.mcap'
            with path.open('wb') as output:
                writer = Writer(output)
                writer.start()
                schema = writer.register_schema('lowlevel.BodyState','flatbuffer',body_state_schema_meta.SCHEMA_BFBS)
                channel = writer.register_channel(BODY_STATE_TOPIC,'flatbuffer',schema)
                for i in range(3):
                    raw = BodyStateT(protocolVersion=3,stateId=2**63+i,tsStateRealUs=1_000_000+20_000*i,
                                    joints=[BodyJointStateT(q=j/4) for j in range(31)])
                    builder = flatbuffers.Builder(2048);builder.Finish(raw.Pack(builder))
                    writer.add_message(channel,100+i,bytes(builder.Output()),100+i)
                writer.finish()
            frames = list(example.replay_states(path,3,.02))
        self.assertEqual([s.state_id for s,_ in frames], [2**63+i for i in range(3)])
        self.assertTrue(all(s.received_monotonic_ns==s.received_realtime_ns==0 for s,_ in frames))
        self.assertEqual(frames[-1][0].joints[30].q, 7.5)
