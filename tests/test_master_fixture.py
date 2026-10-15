"""Directly replay fixed master's existing Vita02A observation fixture.

Source: onnx_feedforward_runner_parity_test.cpp:320-347 at e07e8d8.
This is a source-fixture comparison, not a robot-captured golden.
"""
from pathlib import Path
import json
import struct
from types import SimpleNamespace as NS
import unittest

from examples.teleop_policy import Policy
from test_teleop_policy import fixture


class ExistingMasterFixture(unittest.TestCase):
    def test_vita02a_wrists_neck_fixture_all_82_raw_values(self):
        root=Path(__file__).parent / "fixtures/teleop"
        md=json.loads((root/'model-metadata.json').read_text())['metadata']
        policy=Policy(md,json.loads((root/'action17_contract.json').read_text()))
        body,state=fixture()
        # Inputs come verbatim from the existing robot-side test fixture.
        q=[.02*i for i in range(30)]
        dq=[-.03*i for i in range(30)]
        default=[.005*i for i in range(30)]
        policy.default=tuple(default)
        policy.previous_actions=(.25,)*12
        by_name={name:(q[i],dq[i]) for i,name in enumerate(policy.names)}
        for name,motor in zip(body.joint_names,state.joints):
            if name in by_name:motor.q,motor.dq=by_name[name]
        state.imu_states[1].state=NS(quaternion=(1.,0.,0.,0.),gyroscope=(.1,-.2,.3))
        obs=policy.observation(state,body,(.4,-.2,.6,.855),'HIP')
        values=[.4,-.2,.6,.855,.1,-.2,.3,0.,0.,-1.]
        values += [q[i]-default[i] for i in range(30)] + dq + [.25]*12
        expected=struct.unpack('82f',struct.pack('82f',*values))
        self.assertEqual(obs,expected*6)
        self.assertEqual(obs[38],expected[38])  # existing fixture's neck yaw
        self.assertEqual(obs[39],expected[39])  # neck pitch


if __name__=='__main__':unittest.main()
