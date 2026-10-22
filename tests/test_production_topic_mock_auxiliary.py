"""Fixture-only optional EE/Joy checks; no SDK nodes or worker processes."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from tools.production_topic_mock import validate_auxiliary
from tools.production_topic_mock import run


class AuxiliaryFixtureTests(unittest.TestCase):
    def fixture(self):
        return {"left": {"kind": "brainco", "state_hz": 100, "info_hz": 1,
                         "touch_hz": 20, "info": {"firmware": "SYNTHETIC"}},
                "right": {"kind": "gripper", "state_hz": 200, "info_hz": 1, "info": {}}}

    def test_explicit_rates_preserved(self):
        f = self.fixture()
        self.assertEqual(validate_auxiliary(f), f)

    def test_no_real_frequency_is_invented(self):
        for side, key in [("left", "state_hz"), ("left", "touch_hz"), ("right", "info_hz")]:
            f = self.fixture()
            del f[side][key]
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "rate"):
                validate_auxiliary(f)

    def test_rates_reject_bool_nan_negative_and_unbounded(self):
        for value in (False, float("nan"), float("inf"), 0, -1, 2001):
            f = self.fixture()
            f["left"]["state_hz"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_auxiliary(f)

    def test_unknown_endpoint_or_kind_is_rejected(self):
        for f in ({"robot": {}}, {"left": {"kind": "real-can"}}):
            with self.subTest(f=f), self.assertRaises(ValueError):
                validate_auxiliary(f)

    def test_info_must_be_explicit(self):
        f = self.fixture()
        del f["left"]["info"]
        with self.assertRaises(ValueError):
            validate_auxiliary(f)

    def test_synthetic_output_is_never_enabled_by_truthy_string(self):
        f = self.fixture()
        f["left"]["synthetic_output"] = "false"
        with self.assertRaises(ValueError):
            validate_auxiliary(f)

    def test_virtual_sample_requires_a_finite_number(self):
        f = self.fixture()
        f["right"]["sample_q"] = float("nan")
        with self.assertRaises(ValueError):
            validate_auxiliary(f)

    def test_quick_joy_press_release_are_distinct_levels(self):
        f = {"joy": [{"at_ms": 0, "axes": [0.], "buttons": [0]},
                     {"at_ms": 1, "axes": [0.], "buttons": [1]},
                     {"at_ms": 2, "axes": [0.], "buttons": [0]}]}
        self.assertEqual(len(validate_auxiliary(f)["joy"]), 3)

    def test_duplicate_or_backward_joy_time_rejected(self):
        for times in [(2, 1), (1, 1), (-2, 1), (-.5, 1)]:
            f = {"joy": [{"at_ms": t, "axes": [], "buttons": []} for t in times]}
            with self.subTest(times=times), self.assertRaises(ValueError):
                validate_auxiliary(f)

    def test_joy_script_must_be_a_list(self):
        with self.assertRaises(ValueError):
            validate_auxiliary({"joy": {}})

    def test_bad_joy_axis_or_button_rejected(self):
        for axes, buttons in [([float("inf")], []), ([1.1], []), ([True], []), ([], [True]), ([], [2])]:
            with self.subTest(axes=axes, buttons=buttons), self.assertRaises(ValueError):
                validate_auxiliary({"joy": [{"at_ms": 0, "axes": axes, "buttons": buttons}]})

    def test_third_peer_is_still_only_assigned_loopback(self):
        c = json.loads(run.session_config(24001, [24002, 24003]))
        self.assertEqual(c["connect"]["endpoints"], ["tcp/127.0.0.1:24002", "tcp/127.0.0.1:24003"])




def check_real_binding_packets():
    import importlib
    import importlib.metadata
    import json
    from unittest import mock

    from tools.production_topic_mock import generated_object, pack_object
    from locomotion_aorta.hand_types import HandInfo, HandState, HandJointState, HandTouch, FingerTouchMeta
    from locomotion_aorta.gripper_types import GripperInfo, GripperState, GripperSample, GripperHostTarget
    from locomotion_aorta import _hand_codec, _gripper_codec
    import aorta

    fixtures = [
        HandInfo(100, 1, 'SYNTHETIC', 1, 6, tuple('fixture_'+str(i) for i in range(6)),
                 (0.,)*6, (1.,)*6, (2.,)*6, True, True, 5, 2, 100),
        HandState(2**63+1, 1, 1, tuple(HandJointState(q=.5, q_cmd=.5) for _ in range(6)),
                  last_cmd_session_id=2**63+8, last_cmd_sequence=2**32+1),
        HandTouch(2, 1, 1, 1, 100, tuple(FingerTouchMeta(0,0,100,1,0,0) for _ in range(5)), (0,)*10),
        GripperInfo(100,1,'SYNTHETIC','SYNTHETIC',2,0.,.08,.1,0.,100,1,1,1,1.,1,1),
        GripperState(2**63+2,1,2,sample=GripperSample(.02,0.,0),sample_valid=True,
                     host_target=GripperHostTarget(.02,0.),host_target_valid=True,
                     host_target_session_id=2**63+8,host_target_sequence=2**32+2),
    ]
    records=[]
    with mock.patch.object(aorta,'Node',side_effect=AssertionError('no communication construction')) as forbidden:
        for dto in fixtures:
            payload=pack_object(generated_object(dto),3,1000000000)
            module=importlib.import_module('lowlevel.'+type(dto).__name__)
            view=getattr(module,type(dto).__name__).GetRootAs(payload,0)
            assert view.AortaHeader().Sequence()==3
            assert view.AortaHeader().PublishStampNs()==1000000000
            if isinstance(dto,HandInfo):
                decoded=_hand_codec.decode_info(view);assert decoded.dof==6 and decoded.touch_fingers==5
            elif isinstance(dto,HandState):
                decoded=_hand_codec.decode_state(view);assert decoded.state_id==dto.state_id and decoded.last_cmd_sequence==2**32+1
            elif isinstance(dto,HandTouch):
                decoded=_hand_codec.decode_touch(view);assert len(decoded.raw_words)==10 and len(decoded.fingers)==5
            elif isinstance(dto,GripperInfo):
                decoded=_gripper_codec.decode_info(view);assert decoded.torque_limit_source==1
            else:
                decoded=_gripper_codec.decode_state(view);assert decoded.state_id==dto.state_id and decoded.host_target_sequence==2**32+2
            records.append({'kind':type(dto).__name__,'bytes':len(payload),'passed':True})
        assert forbidden.call_count==0
    return {'scope':'real generated Python EE packing/SDK decode; no CPP compilation/Aorta IPC/device',
                      'schema_version':importlib.metadata.version('vbot-humanoid-msgs'),
                      'fixtures':records,'Node_constructions':0,'status':'PASS_OFFLINE_BINDINGS_ONLY'}

class AuxiliaryWireTest(unittest.TestCase):
    def test_real_generated_ee_roundtrips(self):
        result = check_real_binding_packets()
        self.assertEqual(len(result['fixtures']), 5)
        self.assertTrue(all(item['passed'] for item in result['fixtures']))


if __name__ == '__main__':
    unittest.main()
