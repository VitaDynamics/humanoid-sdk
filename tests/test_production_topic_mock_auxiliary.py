"""Main-owned EE fixture checks and Joy-only auxiliary; no processes or Nodes."""
import json
import unittest
from unittest import mock

from tools.production_topic_mock import validate_auxiliary
from tools.production_topic_mock import run
from tools.production_topic_mock.ee_fixture import validate_ee_spec, prepare_ee_fixture


class AuxiliaryFixtureTests(unittest.TestCase):
    def fixture(self):
        return {"left": {"info": "left-info.bin", "state_hz": 100, "info_hz": 1,
                         "touch_hz": 20, "touch": "left-touch.bin", "initial_pos": [.03]*6,
                         "observation_sources": [[2, 1]]},
                "right": {"info": "right-info.bin", "state_hz": 200, "info_hz": 1,
                          "initial_pos": [.02], "observation_sources": [[2, 2]]}}

    def test_explicit_rates_preserved(self):
        f = self.fixture()
        self.assertEqual(validate_ee_spec(f), f)

    def test_no_real_frequency_is_invented(self):
        for side, key in [("left", "state_hz"), ("left", "touch_hz"), ("right", "info_hz")]:
            f = self.fixture()
            del f[side][key]
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "rate"):
                validate_ee_spec(f)

    def test_rates_reject_bool_nan_negative_and_unbounded(self):
        for value in (False, float("nan"), float("inf"), 0, -1, 2001, 1e-300):
            f = self.fixture()
            f["left"]["state_hz"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_ee_spec(f)

    def test_unknown_endpoint_or_kind_is_rejected(self):
        for f in ({"robot": {}}, {"left": {"kind": "real-can"}}):
            with self.subTest(f=f), self.assertRaises(ValueError):
                validate_ee_spec(f)
        with self.assertRaisesRegex(ValueError, 'main-owned EE'):
            validate_auxiliary(self.fixture())

    def test_info_must_be_explicit(self):
        f = self.fixture()
        del f["left"]["info"]
        with self.assertRaises(ValueError):
            validate_ee_spec(f)

    def test_synthetic_output_is_never_enabled_by_truthy_string(self):
        # Permission is now exclusively the main Protocol owner. Even bool
        # fixture permission flags are rejected, not independent authorities.
        for flag in ("false", True, False):
            f = self.fixture()
            f["left"]["synthetic_output"] = flag
            with self.subTest(flag=flag), self.assertRaises(ValueError):
                validate_ee_spec(f)

    def test_virtual_sample_requires_a_finite_number(self):
        for value in (float("nan"), float("inf"), True):
            f = self.fixture()
            f["right"]["initial_pos"] = [value]
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_ee_spec(f)

    def test_fixed_observations_require_configured_unique_ordered_sources(self):
        for sources in ([[2, 1], [2, 1]], [[3, 2]], [[1, 1]], [[True, 0]], None):
            f = self.fixture()
            f['left']['observation_sources'] = sources
            with self.subTest(sources=sources), self.assertRaises(ValueError):
                validate_ee_spec(f)
        f = self.fixture(); f['left']['observation_sources'] = [[2, 2], [1, 0], [3, 1]]
        self.assertEqual(validate_ee_spec(f)['left']['observation_sources'], [[2, 2], [1, 0], [3, 1]])

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
    from pathlib import Path
    import tempfile
    import importlib.metadata
    import aorta
    from locomotion_aorta import _end_effector_codec as codec
    from locomotion_aorta.end_effector_types import DeviceModel
    from test_end_effector_codec import info_object, state_object, touch_object, pack, module

    records = []
    with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            aorta, 'Node', side_effect=AssertionError('no communication construction')) as forbidden:
        root = Path(directory); output = root/'out'; output.mkdir()
        spec = {}
        for side, number, n, model in (('left',1,6,DeviceModel.BRAINCO_REVO2), ('right',2,1,DeviceModel.AGX)):
            info = info_object(side=number, n=n, touch=side=='left')
            info.deviceModel = model
            payload = pack(info); (root/(side+'-info.bin')).write_bytes(payload)
            spec[side] = dict(info=side+'-info.bin', initial_pos=[.03125]*n,
                              state_hz=100, info_hz=1, observation_sources=[[2,number]])
            if side == 'left':
                (root/'left-touch.bin').write_bytes(pack(touch_object(side=number)))
                spec[side].update(touch='left-touch.bin', touch_hz=20)
        fixture = root/'fixture.json'; fixture.write_text(json.dumps(spec))
        kinds = prepare_ee_fixture(fixture, output)
        assert kinds == {'left': 1, 'right': 2}
        for side, n in (('left',6), ('right',1)):
            payload = (output/('ee-'+side+'-info.bin')).read_bytes()
            assert payload == (root/(side+'-info.bin')).read_bytes()
            decoded = codec.decode_info(module('EndEffectorInfo').EndEffectorInfo.GetRootAs(payload))
            assert len(decoded.axis_names) == n
            records.append(dict(kind=side+' Info', bytes=len(payload), passed=True))
            state = state_object(side=decoded.side, n=n, sample=2**63+1)
            payload = pack(state)
            decoded_state = codec.decode_state(module('EndEffectorState').EndEffectorState.GetRootAs(payload))
            codec.validate_state(decoded, decoded_state)
            assert decoded_state.state_id == 2**63+1 and decoded_state.runtime_id == decoded.runtime_id
            records.append(dict(kind=side+' State', bytes=len(payload), passed=True))
        payload = (output/'ee-left-touch.bin').read_bytes()
        touch = codec.decode_touch(module('EndEffectorTouch').EndEffectorTouch.GetRootAs(payload))
        assert payload == (root/'left-touch.bin').read_bytes()
        assert len(touch.region_stamps) == 2 and len(touch.payload) == 8
        records.append(dict(kind='left Touch', bytes=len(payload), passed=True))
        lines = (output/'ee-config.txt').read_text().splitlines()
        assert lines[0].split()[-3:] == ['1', '2', '1']
        assert lines[1].split()[-3:] == ['1', '2', '2']
        assert forbidden.call_count == 0
    return dict(scope='actual generated EE fixtures/main input; no CPP compilation/Aorta IPC/device',
                schema_version=importlib.metadata.version('vbot-humanoid-msgs'), fixtures=records,
                Node_constructions=0, status='PASS_OFFLINE_BINDINGS_ONLY')


class AuxiliaryWireTest(unittest.TestCase):
    def test_real_generated_ee_roundtrips(self):
        result = check_real_binding_packets()
        self.assertEqual(len(result['fixtures']), 5)
        self.assertTrue(all(item['passed'] for item in result['fixtures']))

    def test_bad_identity_shape_and_capability_leave_no_main_fixture_files(self):
        from pathlib import Path
        import tempfile
        import aorta
        from locomotion_aorta.end_effector_types import DeviceModel
        from test_end_effector_codec import info_object, touch_object, pack
        for failure in ('side', 'position', 'shape', 'touch_ref', 'touch_length', 'op3'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                root = Path(directory); output = root/'out'; output.mkdir()
                info = info_object(side=1, touch=True); info.deviceModel = DeviceModel.BRAINCO_REVO2
                touch = touch_object(side=1)
                spec = {'left': dict(info='info.bin', touch='touch.bin', touch_hz=20,
                    initial_pos=[.03], state_hz=100, info_hz=1, observation_sources=[[2,1]])}
                if failure == 'side': info.side = 2
                elif failure == 'position': spec['left']['initial_pos'] = [.1]
                elif failure == 'shape': info.supportedStateFields = []
                elif failure == 'touch_ref': touch.runtimeId = 101
                elif failure == 'touch_length': touch.payload = [0]
                else: info.deviceModel = DeviceModel.AGILINK_OP3
                (root/'info.bin').write_bytes(pack(info)); (root/'touch.bin').write_bytes(pack(touch))
                path = root/'fixture.json'; path.write_text(json.dumps(spec))
                with mock.patch.object(aorta, 'Node', side_effect=AssertionError('Node created')) as node:
                    with self.assertRaises(ValueError): prepare_ee_fixture(path, output)
                    node.assert_not_called()
                self.assertEqual(list(output.iterdir()), [])


if __name__ == '__main__':
    unittest.main()
