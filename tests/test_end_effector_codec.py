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
    CommandKey, ControlMode, ControlRef, DescriptorRef, EndEffectorSubmission,
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


def info_object(*, side=1, n=1, touch=False, epoch=1, runtime=100):
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
        protocolVersion=1, descriptorRef=module('DescriptorRef').DescriptorRefT(runtime, epoch),
        layoutId=7, side=side, deviceModel=0, firmware='synthetic-μ', serial='fixture',
        identityValidMask=0, axisNames=['axis'+str(i) for i in range(n)],
        positionDomain=[PositionDomain.TOTAL_OPENING]*n,
        torqueDomain=[TorqueDomain.VENDOR_UNCONFIRMED]*n,
        supportedModes=[(1 << ControlMode.POSITION) | (1 << ControlMode.POSITION_TORQUE)]*n,
        supportedCommandFields=[cmd]*n, supportedStateFields=[state]*n,
        validLimits=[Field.POS | Field.VEL | Field.VEL_LIMIT | Field.TAU | Field.CURRENT]*n,
        posMin=[0.]*n, posMax=[.0625]*n, velMin=[-.125]*n, velMax=[.125]*n,
        velLimitMax=[.125]*n, tauMin=[-.5]*n, tauMax=[.5]*n,
        currentMin=[-.25]*n, currentMax=[.25]*n, limitSource=[1]*(n*5),
        touchAvailable=touch, touchRegions=regions, diagnosticEncoding=0,
        tsDescriptorAcquiredUs=10, tsStatePubUs=20, timeValidMask=3)


def state_object(*, side=1, n=1, sample=1, epoch=1, runtime=100):
    field = Field.POS | Field.VEL | Field.TAU | Field.CURRENT | Field.TEMPERATURE | Field.ERROR_CODE
    return module('EndEffectorState').EndEffectorStateT(
        protocolVersion=1, descriptorRef=module('DescriptorRef').DescriptorRefT(runtime, epoch),
        layoutId=7, side=side, stateId=sample, tsStateRealUs=100, tsStatePubUs=110,
        tsAcquiredHwUs=5, tsIngestedUs=101, tsAssembledUs=102, timeValidMask=31,
        sampleStatus=1, validFields=[field]*n, pos=[.03125]*n, vel=[-.015625]*n,
        tau=[-.25]*n, current=[.125]*n, temperatureC=[25.]*n, errorCode=[0]*n,
        isMock=[False]*n, ready=True, outputAllowed=True, inhibitReason=0,
        touchEnabled=False, statusValidMask=15, receivedKey=None, hostTargetKey=None,
        keyValidMask=0, actualOutputKey=[module('CommandKey').CommandKeyT(42, i+1) for i in range(n)],
        actualOutputKeyValid=[False]*n, feedbackAppliedKey=None, feedbackAppliedTarget=None,
        progress=[], progressComplete=False, lastValidAgeUs=0, estimatedInputPeriodUs=0,
        rejectCount=0, lastRejectCode=0, ingressDropCount=0, timeoutCount=0,
        crcErrorCount=0, diagnosticValidMask=0, rawPos=[-32768]*n,
        rawVel=[32767]*n, rawCurrent=[-1]*n, rawStatus=[0xffffffff]*n,
        rawValidFields=[15]*n)


def touch_object(*, side=1, sample=1, epoch=1, runtime=100):
    return module('EndEffectorTouch').EndEffectorTouchT(
        protocolVersion=1, descriptorRef=module('DescriptorRef').DescriptorRefT(runtime, epoch),
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
                             tau_cmd=None if torque is None else (torque,)*n)


class EndEffectorCodecTest(unittest.TestCase):
    def test_real_set_touch_request_uses_full_binding_control_ref_and_no_motion_key(self):
        bindings, service = codec.load_bindings(), codec.load_touch_bindings()
        for side in (1, 2):
            request = EndEffectorSetTouchRequest(DescriptorRef(2**64-1, 2**63+1), 2**32-1,
                                                 side, ControlRef('client-μ', 'lease-λ'), True)
            builder = flatbuffers.Builder(1)
            builder.Finish(codec.fill_set_touch(bindings, service, request, builder, 0))
            wire = service.request.EndEffectorSetTouchRequest.GetRootAs(builder.Output())
            self.assertEqual((wire.DescriptorRef().RobotRuntimeId(), wire.DescriptorRef().DescriptorEpoch()),
                             (2**64-1, 2**63+1))
            self.assertEqual((wire.ProtocolVersion(), wire.LayoutId(), wire.Side(), wire.Enable()), (1, 2**32-1, side, True))
            self.assertEqual((wire.ControlRef().ClientId(), wire.ControlRef().LeaseId()), ('client-μ'.encode(), 'lease-λ'.encode()))
            self.assertFalse(hasattr(wire, 'Key'))

    def test_real_set_touch_response_preserves_default_failure_and_unknown_actual_state(self):
        service = codec.load_touch_bindings()
        obj = service.response.EndEffectorSetTouchResponseT(protocolVersion=1,
            descriptorRef=module('DescriptorRef').DescriptorRefT(100, 1), side=1)
        wire = service.response.EndEffectorSetTouchResponse.GetRootAs(pack(obj))
        response = codec.decode_set_touch(wire)
        request = EndEffectorSetTouchRequest(DescriptorRef(100, 1), 7, 1, ControlRef('C', 'L'), False)
        codec.validate_set_touch_response(request, response)
        self.assertEqual(response.status, ServiceStatus.INTERNAL_ERROR)
        self.assertFalse(response.enabled_valid)
        self.assertFalse(hasattr(response, 'layout_id'))
        self.assertFalse(hasattr(response, 'key'))
        for changed in ({'protocol_version': 0}, {'side': 2}, {'descriptor_ref': DescriptorRef(101, 1)},
                        {'descriptor_ref': DescriptorRef(100, 2)}, {'reason': 65536}, {'status': 128},
                        {'enabled_valid': 0}):
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
        self.assertEqual(info.descriptor_ref, DescriptorRef(100, 1))
        self.assertEqual((info.firmware, info.axis_names), ('synthetic-μ', ('axis0', 'axis1')))
        self.assertEqual((state.pos, state.current, state.temperature_c), ((.03125,)*2, (.125,)*2, (25.,)*2))
        self.assertEqual((state.sample_ref.stream, state.sample_ref.side, state.sample_ref.sample_id),
                         (SampleStream.EE_STATE, 1, 2**64-1))
        self.assertEqual(state.sample_ref.source_time_us, 100)
        self.assertEqual(touch.sample_ref.source_time_us, 0)
        self.assertFalse(touch.sample_ref.validity & 2)
        self.assertEqual(struct.unpack('<4H', touch.payload), (0, 0x7fff, 0x8000, 0xffff))
        self.assertEqual([s.ts_source_us for s in touch.region_stamps], [120, 121])

    def test_real_command_preserves_full_context_missing_intent_and_explicit_zero(self):
        bindings = codec.load_bindings()
        info = decode('info', info_object())
        for torque in (None, 0., -.25):
            with self.subTest(torque=torque):
                prepared = codec.prepare_target(info, target(torque=torque))
                result = EndEffectorSubmission(info.descriptor_ref, 7, 1,
                    ControlRef('client-μ', 'lease'), CommandKey(2**64-1, 2**64-2), prepared)
                builder = flatbuffers.Builder(1)
                builder.Finish(codec.fill_command(bindings, result, builder, 0))
                wire = bindings.EndEffectorCmd.EndEffectorCmd.GetRootAs(builder.Output())
                self.assertEqual((wire.Key().SessionId(), wire.Key().Sequence()), (2**64-1, 2**64-2))
                self.assertEqual((wire.DescriptorRef().RobotRuntimeId(), wire.DescriptorRef().DescriptorEpoch()), (100, 1))
                self.assertEqual((wire.ControlRef().ClientId(), wire.ControlRef().LeaseId()), ('client-μ'.encode(), b'lease'))
                self.assertEqual((wire.Pos(0), wire.VelLimit(0)), (.03125, 0.))
                self.assertEqual(wire.TauCmd(0), torque or 0.)
                self.assertEqual(bool(wire.CommandFields(0) & Field.TAU), torque is not None)
                self.assertEqual((wire.InputObservationsLength(), wire.TsClientCreatedUs(),
                                  wire.TsClientSendUs(), wire.SdkTimeValidMask()), (0, 0, 0, 0))

    def test_all_axis_vector_lengths_and_capability_sets_are_checked(self):
        info = decode('info', info_object())
        for changed in ({'position_domain': ()}, {'supported_modes': (1,)},
                        {'limit_source': (1,)}, {'pos_max': ()},
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
        obj = state_object(); obj.pos = [math.nan]; obj.validFields = [Field.ERROR_CODE]
        state = decode('state', obj)
        codec.validate_state(info, state)
        self.assertTrue(math.isnan(state.pos[0]))
        self.assertFalse(state.valid_fields[0] & Field.POS)
        with self.assertRaises(ValueError):
            codec.validate_state(info, dataclasses.replace(state, valid_fields=(Field.POS,)))

    def test_error_valid_zero_unknown_and_high_uint32_stay_distinct(self):
        info = decode('info', info_object(n=3))
        obj = state_object(n=3); obj.errorCode = [0, 0, 0xf1234567]
        obj.validFields = [Field.POS | Field.ERROR_CODE, Field.POS, Field.POS | Field.ERROR_CODE]
        state = decode('state', obj)
        codec.validate_state(info, state)
        self.assertEqual(state.error_code, (0, 0, 0xf1234567))
        self.assertEqual([bool(v & Field.ERROR_CODE) for v in state.valid_fields], [True, False, True])

    def test_output_progress_and_feedback_keys_are_not_relabelled_as_targets(self):
        info = decode('info', info_object(n=2))
        obj = state_object(n=2)
        obj.receivedKey = module('CommandKey').CommandKeyT(9, 3)
        obj.hostTargetKey = module('CommandKey').CommandKeyT(9, 2)
        obj.feedbackAppliedKey = module('CommandKey').CommandKeyT(9, 1)
        obj.feedbackAppliedTarget = module('FeedbackAppliedTarget').FeedbackAppliedTargetT(
            pos=[.015625, .03125], velLimit=[0., .0625], fields=[Field.POS, Field.POS | Field.VEL_LIMIT])
        obj.keyValidMask = 7
        obj.actualOutputKey = [module('CommandKey').CommandKeyT(9, i) for i in (1, 2)]
        obj.actualOutputKeyValid = [True, True]
        obj.progress = [module('ProgressEntry').ProgressEntryT(
            key=module('CommandKey').CommandKeyT(9, 1), tsReceivedUs=0, stageValidMask=1)]
        state = decode('state', obj)
        codec.validate_state(info, state)
        self.assertEqual((state.received_key.sequence, state.host_target_key.sequence,
                          state.feedback_applied_key.sequence), (3, 2, 1))
        self.assertEqual(tuple(k.sequence for k in state.actual_output_key), (1, 2))
        self.assertEqual(state.feedback_applied_target.pos, (.015625, .03125))
        self.assertEqual(state.feedback_applied_target.fields, (Field.POS, Field.POS | Field.VEL_LIMIT))
        self.assertEqual((state.progress[0].ts_received_us, state.progress[0].stage_valid_mask), (0, 1))
        with self.assertRaises(ValueError):
            codec.validate_state(info, dataclasses.replace(state, progress=state.progress*2))

    def test_corrupt_endpoint_shape_and_permission_are_rejected(self):
        info = decode('info', info_object())
        state = decode('state', state_object())
        for changed in ({'side': 2}, {'layout_id': 8}, {'protocol_version': 0},
                        {'descriptor_ref': DescriptorRef(100, 2)}, {'pos': ()},
                        {'actual_output_key_valid': ()}, {'output_allowed': False}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                codec.validate_state(info, dataclasses.replace(state, **changed))

    def test_target_validation_checks_units_ranges_modes_and_float32_domain(self):
        info = decode('info', info_object())
        for changed in ({'mode': (0,)}, {'mode': (31,)}, {'mode': (True,)},
                        {'command_fields': (Field.TEMPERATURE,)}, {'command_fields': (0,)},
                        {'pos': ()}, {'pos': (math.nan,)}, {'pos': (math.inf,)},
                        {'pos': (.125,)}, {'vel_limit': (-.1,)}, {'pos': None},
                        {'current_cmd': (1e100,), 'command_fields': (Field.CURRENT,)}):
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
            (Field.CURRENT | Field.VEL,), current_cmd=(.125,), vel=(-.03125,)))
        self.assertEqual((prepared.current_cmd, prepared.vel), ((.125,), (-.03125,)))
        with self.assertRaises(ValueError):
            codec.prepare_target(info, dataclasses.replace(prepared, current_cmd=(125.,)))

    def test_unsupported_columns_stay_empty_and_unknown_read_enums_stay_unknown(self):
        obj = info_object()
        obj.supportedCommandFields = [Field.POS | Field.VEL_LIMIT | Field.MODE]
        obj.supportedStateFields = [Field.POS]
        obj.validLimits = [Field.POS | Field.VEL_LIMIT]
        obj.velMin = obj.velMax = obj.tauMin = obj.tauMax = obj.currentMin = obj.currentMax = []
        obj.deviceModel = 65535; obj.diagnosticEncoding = 65535
        info = decode('info', obj)
        codec.validate_info(info)
        self.assertEqual((info.device_model, info.diagnostic_encoding), (65535, 65535))
        prepared = codec.prepare_target(info, target())
        self.assertEqual((prepared.vel, prepared.tau_cmd, prepared.current_cmd), ((), (), ()))
        state = state_object()
        state.validFields = [Field.POS]
        state.vel = state.tau = state.current = state.temperatureC = state.errorCode = []
        state.sampleStatus = 251; state.inhibitReason = 251; state.outputAllowed = False
        decoded = decode('state', state)
        codec.validate_state(info, decoded)
        self.assertEqual((decoded.sample_status, decoded.inhibit_reason), (251, 251))
        self.assertFalse(decoded.sample_ref.validity & 4)
        with self.assertRaises(ValueError):
            codec.validate_state(info, dataclasses.replace(decoded, current=(0.,)))

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
        result = EndEffectorSubmission(info.descriptor_ref, 7, 1, ControlRef('C', 'L'),
                                      CommandKey(42, 1), codec.prepare_target(info, target()))
        for changed in ({'sdk_time_valid_mask': 1},
                        {'ts_client_created_us': 1}, {'ts_client_send_us': 1}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                codec.fill_command(bindings, dataclasses.replace(result, **changed), flatbuffers.Builder(1), 0)

    def test_nonempty_observations_use_real_struct_vector_and_roundtrip(self):
        bindings = codec.load_bindings()
        info = decode('info', info_object())
        # Decoded source samples, including genuinely valid zero and wide IDs.
        observations = tuple(decode('state', state_object(side=side, sample=sample)).sample_ref
                             for side, sample in ((1, 0), (2, 2**64-1)))
        observations += (decode('touch', touch_object(sample=2**63+7)).sample_ref,)
        submission = EndEffectorSubmission(info.descriptor_ref, info.layout_id, 1,
            ControlRef('C', 'L'), CommandKey(2**64-1, 2**63+9),
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
                self.assertEqual((wire.SdkTimeValidMask(), wire.TsClientCreatedUs(), wire.TsClientSendUs()), (0, 0, 0))

    def test_wide_descriptor_and_sample_ref_preserved_by_command_codec(self):
        bindings = codec.load_bindings()
        info = decode('info', info_object(runtime=2**64-1, epoch=2**63+1))
        sample = decode('state', state_object(runtime=2**64-1, epoch=2**63+1, sample=2**64-1)).sample_ref
        submission = EndEffectorSubmission(info.descriptor_ref, 7, 1, ControlRef('C', 'L'),
            CommandKey(0, 0), codec.prepare_target(info, target()), (sample,))
        builder = flatbuffers.Builder(1)
        builder.Finish(codec.fill_command(bindings, submission, builder, 0))
        actual = codec.decode_cmd(bindings.EndEffectorCmd.EndEffectorCmd.GetRootAs(builder.Output()))
        self.assertEqual(actual, submission)
        self.assertEqual(actual.input_observations[0].descriptor_ref, DescriptorRef(2**64-1, 2**63+1))

    def test_diagnostic_counters_unknown_times_and_signed_raw_keep_full_received_values(self):
        wire = state_object(sample=2**64-1)
        wire.lastValidAgeUs, wire.estimatedInputPeriodUs = 2**64-1, 2**63+1
        wire.rejectCount, wire.ingressDropCount, wire.timeoutCount, wire.crcErrorCount = 2, 3, 4, 5
        wire.lastRejectCode = 65535
        wire.timeValidMask = wire.diagnosticValidMask = 0
        actual = decode('state', wire)
        self.assertEqual(actual.state_id, 2**64-1)
        self.assertEqual((actual.last_valid_age_us, actual.estimated_input_period_us), (2**64-1, 2**63+1))
        self.assertEqual((actual.reject_count, actual.ingress_drop_count, actual.timeout_count,
                          actual.crc_error_count, actual.last_reject_code), (2, 3, 4, 5, 65535))
        self.assertEqual((actual.raw_pos, actual.raw_vel, actual.raw_current, actual.raw_status),
                         ((-32768,), (32767,), (-1,), (0xffffffff,)))
        self.assertEqual((actual.time_valid_mask, actual.diagnostic_valid_mask), (0, 0))
        self.assertEqual(actual.ts_state_real_us, 100)
        self.assertEqual(actual.sample_ref.source_time_us, 0)
        self.assertFalse(actual.sample_ref.validity & 2)
