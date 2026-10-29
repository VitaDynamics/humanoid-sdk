"""Actual new generated roots; synthetic capabilities are not device evidence."""
import dataclasses
import hashlib
import math
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import flatbuffers

from locomotion_aorta import _end_effector_codec as codec
from locomotion_aorta.end_effector_types import (
    CommandKey, ControlMode, EndEffectorSubmission,
    EndEffectorTarget, Field, PositionDomain, SampleRef, SampleStream, TorqueDomain,
    EndEffectorSetTouchRequest, ServiceStatus,
)


def module(name):
    import importlib
    return importlib.import_module('lowlevel.ee.' + name)


def pack(value):
    builder = flatbuffers.Builder(1)
    builder.ForceDefaults(True)
    builder.Finish(value.Pack(builder))
    return bytes(builder.Output())


def info_object(*, side=1, n=1, touch=False, runtime=100):
    # Deliberately synthetic, with every generic quantity, so tests can check
    # SI/masks independently. This does not declare real AGX/BrainCo support.
    cmd = Field.POS | Field.VEL | Field.VEL_LIMIT | Field.TAU | Field.CURRENT | Field.MODE
    state = Field.POS | Field.VEL | Field.TAU | Field.CURRENT | Field.TEMPERATURE | Field.ERROR_CODE
    regions = []
    if touch:
        regions = [module('TouchRegionInfo').TouchRegionInfoT(
            regionId=i, name='region'+str(i), frame='test', sensorType=1,
            shape=[2], components=['raw'], dtype=1, unit='1',
            payloadOffset=i*4, payloadBytes=4, encodingVersion=1,
            geometry=[], geometryValid=False) for i in range(2)]
    return module('EndEffectorInfo').EndEffectorInfoT(
        protocolVersion=1, runtimeId=runtime,
        layoutId=7, side=side, deviceModel=0, firmware='synthetic-μ', serial='fixture',
        identityValidMask=0, axisNames=['axis'+str(i) for i in range(n)],
        positionDomain=[PositionDomain.TOTAL_OPENING]*n,
        torqueDomain=[TorqueDomain.VENDOR_UNCONFIRMED]*n,
        supportedModes=[(1 << ControlMode.POSITION) | (1 << ControlMode.POSITION_TORQUE)]*n,
        supportedCommandFields=[cmd]*n, supportedStateFields=[state]*n,
        validLimits=[Field.POS | Field.VEL | Field.VEL_LIMIT | Field.TAU | Field.CURRENT]*n,
        posMin=[0.]*n, posMax=[.0625]*n, velMin=[-.125]*n, velMax=[.125]*n,
        velLimitMax=[.125]*n, tauMin=[-.5]*n, tauMax=[.5]*n,
        currentMin=[-.25]*n, currentMax=[.25]*n,
        touchAvailable=touch, touchRegions=regions, isMock=[False]*n,
        tsDescriptorAcquiredUs=10, tsStatePubUs=20, timeValidMask=3)


def state_object(*, side=1, n=1, sample=1, runtime=100):
    field = Field.POS | Field.VEL | Field.TAU | Field.CURRENT | Field.TEMPERATURE | Field.ERROR_CODE
    return module('EndEffectorState').EndEffectorStateT(
        protocolVersion=2, runtimeId=runtime,
        layoutId=7, side=side, stateId=sample, tsStateRealUs=100, tsStatePubUs=110,
        tsAcquiredHwUs=5, tsIngestedUs=101, tsAssembledUs=102,
        sampleStatus=1, axes=[module('AxisState').AxisStateT(
            validFields=field, pos=.03125, vel=-.015625, tau=-.25,
            current=.125, temperatureC=25., errorCode=0) for _ in range(n)],
        ready=True, outputAllowed=True, inhibitReason=0,
        touchEnabled=False, statusValidMask=15)


def touch_object(*, side=1, sample=1, runtime=100):
    return module('EndEffectorTouch').EndEffectorTouchT(
        protocolVersion=1, runtimeId=runtime,
        layoutId=7, side=side, touchStateId=sample, tsStatePubUs=200, publishTimeValid=True,
        regionStamps=[module('RegionStamp').RegionStampT(
            sampleSeq=i+1, tsRequestUs=100+i, tsAcquiredHwUs=3+i,
            tsSourceUs=120+i, tsIngestedUs=130+i, statusFlags=0x8000+i,
            directionFlags=i, validity=63) for i in range(2)],
        payload=list(struct.pack('<4H', 0, 0x7fff, 0x8000, 0xffff)))


def decode(kind, value):
    wire = getattr(module('EndEffector'+kind.capitalize()), 'EndEffector'+kind.capitalize()).GetRootAs(pack(value))
    return getattr(codec, 'decode_'+kind)(wire)


def target(n=1, *, torque=None):
    mask = Field.POS | Field.VEL_LIMIT
    if torque is not None:
        mask |= Field.TAU
    return EndEffectorTarget((ControlMode.POSITION_TORQUE,)*n, (mask,)*n,
                             pos=(.03125,)*n, vel_limit=(0.,)*n,
                             tau=None if torque is None else (torque,)*n)


class EndEffectorCodecTest(unittest.TestCase):
    def test_real_set_touch_request_uses_full_binding_control_ref_and_no_motion_key(self):
        bindings, service = codec.load_bindings(), codec.load_touch_bindings()
        for side in (1, 2):
            request = EndEffectorSetTouchRequest(2**64-1, 2**32-1,
                                                 side, 2**64-1, True)
            builder = flatbuffers.Builder(1)
            builder.Finish(codec.fill_set_touch(bindings, service, request, builder, 0))
            wire = service.request.EndEffectorSetTouchRequest.GetRootAs(builder.Output())
            self.assertEqual(wire.RuntimeId(), 2**64-1)
            self.assertEqual((wire.ProtocolVersion(), wire.LayoutId(), wire.Side(), wire.Enable()), (2, 2**32-1, side, True))
            self.assertEqual(wire.ControlId(), 2**64-1)
            self.assertFalse(hasattr(wire, 'Key'))

    def test_real_set_touch_response_preserves_default_failure_and_unknown_actual_state(self):
        service = codec.load_touch_bindings()
        obj = service.response.EndEffectorSetTouchResponseT(protocolVersion=2,
            runtimeId=100, side=1)
        wire = service.response.EndEffectorSetTouchResponse.GetRootAs(pack(obj))
        response = codec.decode_set_touch(wire)
        request = EndEffectorSetTouchRequest(100, 7, 1, 13, False)
        codec.validate_set_touch_response(request, response)
        self.assertEqual(response.status, ServiceStatus.INTERNAL_ERROR)
        self.assertFalse(hasattr(response, 'enabled_valid'))
        self.assertFalse(hasattr(response, 'layout_id'))
        self.assertFalse(hasattr(response, 'key'))
        for changed in ({'protocol_version': 0}, {'side': 2}, {'runtime_id': 101},
                        {'reason': 65536}, {'status': 128}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                codec.validate_set_touch_response(request, dataclasses.replace(response, **changed))

    def test_set_touch_metadata_gate_rejects_wrong_service_namespace_or_hash(self):
        original = codec.importlib.import_module
        for root, digest in (('lowlevel.HandCmd', hashlib.sha256(b'wrong').hexdigest()),
                             ('aorta.services.humanoid_locomotion.EndEffectorSetTouchRequest', '0'*64)):
            def wrong(name):
                if name == 'end_effector_set_touch_schema_meta':
                    return SimpleNamespace(ROOT_TYPE_NAME=root, SCHEMA_BFBS=b'wrong', SCHEMA_HASH_HEX=digest)
                return original(name)
            with patch.object(codec.importlib, 'import_module', side_effect=wrong), self.assertRaisesRegex(
                    RuntimeError, 'set_touch binding'):
                codec.load_touch_bindings()

    def test_actual_generated_info_state_touch_preserve_si_and_complete_identity(self):
        info = decode('info', info_object(n=2, touch=True))
        state = decode('state', state_object(n=2, sample=2**64-1))
        touch = decode('touch', touch_object(sample=2**63+1))
        codec.validate_info(info); codec.validate_state(info, state); codec.validate_touch(info, touch)
        self.assertEqual(info.runtime_id, 100)
        self.assertEqual((info.firmware, info.axis_names), ('synthetic-μ', ('axis0', 'axis1')))
        self.assertEqual(info.is_mock, (False, False))
        self.assertFalse(hasattr(state, 'is_mock'))
        self.assertEqual((state.pos, state.current, state.temperature_c), ((.03125,)*2, (.125,)*2, (25.,)*2))
        self.assertEqual((state.sample_ref.stream, state.sample_ref.side, state.sample_ref.sample_id),
                         (SampleStream.EE_STATE, 1, 2**64-1))
        self.assertEqual(state.sample_ref.source_time_us, 100)
        self.assertEqual(touch.sample_ref.source_time_us, 0)
        self.assertFalse(touch.sample_ref.validity & 2)
        self.assertEqual(struct.unpack('<4H', touch.payload), (0, 0x7fff, 0x8000, 0xffff))
        self.assertEqual([s.ts_source_us for s in touch.region_stamps], [120, 121])
        self.assertEqual((info.protocol_version, state.protocol_version, touch.protocol_version), (1, 2, 1))
        old = state_object(); old.protocolVersion = 1
        with self.assertRaisesRegex(ValueError, 'protocol_version'):
            decode('state', old)

    def test_real_command_preserves_full_context_missing_intent_and_explicit_zero(self):
        bindings = codec.load_bindings()
        info = decode('info', info_object())
        for torque in (None, 0., -.25):
            with self.subTest(torque=torque):
                prepared = codec.prepare_target(info, target(torque=torque))
                result = EndEffectorSubmission(info.runtime_id, 7, 1,
                    13, CommandKey(2**64-1, 2**64-2), prepared)
                builder = flatbuffers.Builder(1)
                builder.Finish(codec.fill_command(bindings, result, builder, 0))
                wire = bindings.EndEffectorCmd.EndEffectorCmd.GetRootAs(builder.Output())
                self.assertEqual((wire.Key().SessionId(), wire.Key().Sequence()), (2**64-1, 2**64-2))
                self.assertEqual(wire.RuntimeId(), 100)
                self.assertEqual(wire.ControlId(), 13)
                self.assertEqual(wire.AxesLength(), 1)
                self.assertEqual((wire.Axes(0).Pos(), wire.Axes(0).VelLimit()), (.03125, 0.))
                self.assertEqual(wire.Axes(0).Tau(), torque or 0.)
                self.assertEqual(bool(wire.Axes(0).CommandFields() & Field.TAU), torque is not None)
                self.assertEqual((wire.InputObservationsLength(), wire.TsClientCreatedUs(),
                                  wire.TsClientSendUs()), (0, 0, 0))
                self.assertFalse(hasattr(wire, 'SdkTimeValidMask'))

    def test_new_command_rejects_missing_axes_and_invalid_numeric_control(self):
        bindings = codec.load_bindings()
        info = decode('info', info_object())
        command = EndEffectorSubmission(info.runtime_id, 7, 1, 13,
            CommandKey(42, 1), codec.prepare_target(info, target()))
        for changed in ({'control_id': 0}, {'control_id': True},
                        {'protocol_version': 1},
                        {'target': dataclasses.replace(command.target, mode=())}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                codec.fill_command(bindings, dataclasses.replace(command, **changed),
                                   flatbuffers.Builder(1), 0)

    def test_all_axis_vector_lengths_and_capability_sets_are_checked(self):
        info = decode('info', info_object())
        for changed in ({'position_domain': ()}, {'supported_modes': (1,)},
                        {'is_mock': ()}, {'is_mock': (1,)}, {'pos_max': ()},
                        {'supported_state_fields': (Field.MODE,)},
                        {'supported_command_fields': (Field.TEMPERATURE,)}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                codec.validate_info(dataclasses.replace(info, **changed))

    def test_touch_layout_rejects_overlap_bad_geometry_and_overflow(self):
        info = decode('info', info_object(touch=True))
        region = info.touch_regions[0]
        for bad in (dataclasses.replace(region, payload_bytes=3),
                    dataclasses.replace(region, shape=(0xffffffff, 2)),
                    dataclasses.replace(region, geometry_valid=True, geometry=()),
                    dataclasses.replace(region, geometry=(0.,))):
            with self.subTest(region=bad), self.assertRaises(ValueError):
                codec.validate_info(dataclasses.replace(info, touch_regions=(bad, info.touch_regions[1])))
        with self.assertRaises(ValueError):
            codec.validate_info(dataclasses.replace(info, touch_regions=(region, dataclasses.replace(
                info.touch_regions[1], payload_offset=2))))

    def test_invalid_measurement_placeholder_does_not_become_a_valid_zero(self):
        info = decode('info', info_object())
        obj = state_object(); obj.axes[0].pos = math.nan; obj.axes[0].validFields = Field.ERROR_CODE
        state = decode('state', obj)
        codec.validate_state(info, state)
        self.assertTrue(math.isnan(state.pos[0]))
        self.assertFalse(state.valid_fields[0] & Field.POS)
        with self.assertRaises(ValueError):
            codec.validate_state(info, dataclasses.replace(state, valid_fields=(Field.POS,)))

    def test_error_valid_zero_unknown_and_high_uint32_stay_distinct(self):
        info = decode('info', info_object(n=3))
        obj = state_object(n=3)
        for axis, code, mask in zip(obj.axes, (0, 0, 0xf1234567),
                (Field.POS | Field.ERROR_CODE, Field.POS, Field.POS | Field.ERROR_CODE)):
            axis.errorCode, axis.validFields = code, mask
        state = decode('state', obj)
        codec.validate_state(info, state)
        self.assertEqual(state.error_code, (0, 0, 0xf1234567))
        self.assertEqual([bool(v & Field.ERROR_CODE) for v in state.valid_fields], [True, False, True])

    def test_state_has_no_retired_command_correlation_or_progress(self):
        info = decode('info', info_object(n=2))
        obj = state_object(n=2)
        state = decode('state', obj)
        codec.validate_state(info, state)
        self.assertEqual(state.pos, (.03125, .03125))
        for name in ('received_key', 'host_target_key', 'actual_output_key', 'progress'):
            self.assertFalse(hasattr(state, name))

    def test_corrupt_endpoint_shape_and_permission_are_rejected(self):
        info = decode('info', info_object())
        state = decode('state', state_object())
        for changed in ({'side': 2}, {'layout_id': 8}, {'protocol_version': 0},
                        {'runtime_id': 101}, {'pos': ()},
                        {'valid_fields': ()}, {'output_allowed': False}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                codec.validate_state(info, dataclasses.replace(state, **changed))

    def test_target_validation_checks_units_ranges_modes_and_float32_domain(self):
        info = decode('info', info_object())
        for changed in ({'mode': (0,)}, {'mode': (31,)}, {'mode': (True,)},
                        {'command_fields': (Field.TEMPERATURE,)}, {'command_fields': (0,)},
                        {'pos': ()}, {'pos': (math.nan,)}, {'pos': (math.inf,)},
                        {'pos': (.125,)}, {'vel_limit': (-.1,)}, {'pos': None},
                        {'current': (1e100,), 'command_fields': (Field.CURRENT,)}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                codec.prepare_target(info, dataclasses.replace(target(), **changed))
        valid = codec.prepare_target(info, target())
        self.assertEqual(valid.vel_limit, (0.,))  # preserves configured-default intent
        for value in (0., .0625):
            codec.prepare_target(info, dataclasses.replace(target(), pos=(value,)))

    def test_missing_limits_unknown_domains_and_unsupported_fields_cannot_publish(self):
        info = decode('info', info_object())
        for changed in ({'valid_limits': (0,)}, {'position_domain': (0,)},
                        {'supported_command_fields': (Field.MODE,)}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                codec.prepare_target(dataclasses.replace(info, **changed), target())
        with self.assertRaises(ValueError):
            codec.prepare_target(dataclasses.replace(info, torque_domain=(0,)), target(torque=0.))

    def test_current_is_amperes_and_signed_velocity_is_not_a_speed_limit(self):
        info = decode('info', info_object())
        prepared = codec.prepare_target(info, EndEffectorTarget((ControlMode.POSITION,),
            (Field.CURRENT | Field.VEL,), current=(.125,), vel=(-.03125,)))
        self.assertEqual((prepared.current, prepared.vel), ((.125,), (-.03125,)))
        with self.assertRaises(ValueError):
            codec.prepare_target(info, dataclasses.replace(prepared, current=(125.,)))

    def test_unsupported_columns_stay_empty_and_unknown_read_enums_stay_unknown(self):
        obj = info_object()
        obj.supportedCommandFields = [Field.POS | Field.VEL_LIMIT | Field.MODE]
        obj.supportedStateFields = [Field.POS]
        obj.validLimits = [Field.POS | Field.VEL_LIMIT]
        obj.velMin = obj.velMax = obj.tauMin = obj.tauMax = obj.currentMin = obj.currentMax = []
        obj.deviceModel = 65535
        info = decode('info', obj)
        codec.validate_info(info)
        self.assertEqual(info.device_model, 65535)
        prepared = codec.prepare_target(info, target())
        self.assertEqual((prepared.vel, prepared.tau, prepared.current), ((), (), ()))
        state = state_object()
        state.axes[0].validFields = Field.POS
        state.sampleStatus = 251; state.inhibitReason = 251; state.outputAllowed = False
        decoded = decode('state', state)
        codec.validate_state(info, decoded)
        self.assertEqual((decoded.sample_status, decoded.inhibit_reason), (251, 251))
        self.assertFalse(decoded.sample_ref.validity & 4)
        with self.assertRaises(ValueError):
            codec.validate_state(info, dataclasses.replace(decoded, valid_fields=(Field.CURRENT,)))

    def test_touch_length_validity_and_independent_region_clocks_are_preserved(self):
        info = decode('info', info_object(touch=True))
        obj = touch_object(); obj.regionStamps[1].validity = 64
        touch = decode('touch', obj)
        codec.validate_touch(info, touch)
        self.assertFalse(touch.sample_ref.validity & 4)
        self.assertTrue(touch.sample_ref.validity & 8)
        self.assertEqual([s.status_flags for s in touch.region_stamps], [0x8000, 0x8001])
        for changed in ({'payload': touch.payload[:-1]}, {'region_stamps': touch.region_stamps[:1]},
                        {'layout_id': 8}, {'protocol_version': 0}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                codec.validate_touch(info, dataclasses.replace(touch, **changed))

    def test_real_binding_gate_checks_new_namespace_and_metadata_hash(self):
        bindings = codec.load_bindings()
        self.assertEqual(bindings.metas['cmd'].ROOT_TYPE_NAME, 'lowlevel.ee.EndEffectorCmd')
        original = codec.importlib.import_module
        for root, digest in (('lowlevel.HandCmd', hashlib.sha256(b'wrong').hexdigest()),
                             ('lowlevel.ee.EndEffectorCmd', '0'*64)):
            def wrong(name):
                if name == 'end_effector_cmd_schema_meta':
                    return SimpleNamespace(ROOT_TYPE_NAME=root, SCHEMA_BFBS=b'wrong', SCHEMA_HASH_HEX=digest)
                return original(name)
            with patch.object(codec.importlib, 'import_module', side_effect=wrong), self.assertRaisesRegex(
                    RuntimeError, 'new generated EE'):
                codec.load_bindings()

    def test_fabricated_sdk_time_is_not_encoded(self):
        bindings = codec.load_bindings()
        info = decode('info', info_object())
        result = EndEffectorSubmission(info.runtime_id, 7, 1, 13,
                                      CommandKey(42, 1), codec.prepare_target(info, target()))
        for changed in ({'ts_client_created_us': 1}, {'ts_client_send_us': 1}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                codec.fill_command(bindings, dataclasses.replace(result, **changed), flatbuffers.Builder(1), 0)

    def test_nonempty_observations_use_real_struct_vector_and_roundtrip(self):
        bindings = codec.load_bindings()
        info = decode('info', info_object())
        # Decoded source samples, including genuinely valid zero and wide IDs.
        observations = tuple(decode('state', state_object(side=side, sample=sample)).sample_ref
                             for side, sample in ((1, 0), (2, 2**64-1)))
        observations += (decode('touch', touch_object(sample=2**63+7)).sample_ref,)
        submission = EndEffectorSubmission(info.runtime_id, info.layout_id, 1,
            13, CommandKey(2**64-1, 2**63+9),
            codec.prepare_target(info, target()), observations)
        for size in (1, 7, 4096):
            with self.subTest(builder_size=size):
                builder = flatbuffers.Builder(size)
                builder.Finish(codec.fill_command(bindings, submission, builder, 0))
                wire = bindings.EndEffectorCmd.EndEffectorCmd.GetRootAs(builder.Output())
                self.assertEqual(wire.InputObservationsLength(), 3)
                self.assertEqual(codec.decode_cmd(wire), submission)
                self.assertEqual(wire.InputObservations(0).SampleId(), 0)
                self.assertEqual(wire.InputObservations(1).SampleId(), 2**64-1)
                self.assertEqual(wire.InputObservations(2).SourceTimeUs(), 0)
                self.assertEqual((wire.TsClientCreatedUs(), wire.TsClientSendUs()), (0, 0))

    def test_wide_descriptor_and_sample_ref_preserved_by_command_codec(self):
        bindings = codec.load_bindings()
        info = decode('info', info_object(runtime=2**64-1))
        sample = decode('state', state_object(runtime=2**64-1, sample=2**64-1)).sample_ref
        submission = EndEffectorSubmission(info.runtime_id, 7, 1, 13,
            CommandKey(0, 0), codec.prepare_target(info, target()), (sample,))
        builder = flatbuffers.Builder(1)
        builder.Finish(codec.fill_command(bindings, submission, builder, 0))
        actual = codec.decode_cmd(bindings.EndEffectorCmd.EndEffectorCmd.GetRootAs(builder.Output()))
        self.assertEqual(actual, submission)
        self.assertEqual(actual.input_observations[0].runtime_id, 2**64-1)

    def test_zero_source_time_is_unknown_without_a_time_mask(self):
        wire = state_object(sample=2**64-1)
        wire.tsStateRealUs = 0
        actual = decode('state', wire)
        self.assertEqual(actual.state_id, 2**64-1)
        self.assertFalse(hasattr(actual, 'time_valid_mask'))
        self.assertEqual(actual.ts_state_real_us, 0)
        self.assertEqual(actual.sample_ref.source_time_us, 0)
        self.assertFalse(actual.sample_ref.validity & 2)
