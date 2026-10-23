"""Generated e38 EE accessors/builders; no offsets or old Hand/Gripper bridge."""
import hashlib
import importlib
import math
import struct
from dataclasses import replace
from types import SimpleNamespace

from ._body_codec import _dto, _getter, _text
from .transport import AortaBindingsUnavailableError
from .end_effector_types import (
    CommandKey, ControlMode, DescriptorRef, EE_PROTOCOL_VERSION, EndEffectorInfo,
    EndEffectorState, EndEffectorTarget, EndEffectorTouch, FeedbackAppliedTarget,
    Field, InhibitReason, LimitSource, PositionDomain, ProgressEntry, RegionStamp,
    TorqueDomain, TouchPayloadType, TouchRegionInfo,
)

_COMMAND_FIELDS = Field.POS | Field.VEL | Field.VEL_LIMIT | Field.TAU | Field.CURRENT | Field.MODE
_STATE_FIELDS = Field.POS | Field.VEL | Field.TAU | Field.CURRENT | Field.TEMPERATURE | Field.ERROR_CODE
_LIMIT_FIELDS = Field.POS | Field.VEL | Field.VEL_LIMIT | Field.TAU | Field.CURRENT
_KNOWN_FIELDS = _COMMAND_FIELDS | _STATE_FIELDS
_LIMITS = (
    (Field.POS, 'pos_min', 'pos_max'), (Field.VEL, 'vel_min', 'vel_max'),
    (Field.VEL_LIMIT, None, 'vel_limit_max'), (Field.TAU, 'tau_min', 'tau_max'),
    (Field.CURRENT, 'current_min', 'current_max'),
)
_COMMAND_COLUMNS = (
    ('pos', Field.POS), ('vel', Field.VEL), ('vel_limit', Field.VEL_LIMIT),
    ('tau_cmd', Field.TAU), ('current_cmd', Field.CURRENT),
)
_STATE_COLUMNS = (
    ('pos', Field.POS), ('vel', Field.VEL), ('tau', Field.TAU),
    ('current', Field.CURRENT), ('temperature_c', Field.TEMPERATURE),
    ('error_code', Field.ERROR_CODE),
)
_INFO_VECTORS = (
    'position_domain', 'torque_domain', 'supported_modes', 'supported_command_fields',
    'supported_state_fields', 'valid_limits', 'pos_min', 'pos_max', 'vel_min',
    'vel_max', 'vel_limit_max', 'tau_min', 'tau_max', 'current_min', 'current_max',
    'limit_source',
)
_STATE_VECTORS = (
    'valid_fields', 'pos', 'vel', 'tau', 'current', 'temperature_c', 'error_code',
    'is_mock', 'actual_output_key_valid', 'raw_pos', 'raw_vel', 'raw_current',
    'raw_status', 'raw_valid_fields',
)


def load_bindings():
    try:
        names = ('EndEffectorInfo', 'EndEffectorState', 'EndEffectorTouch',
                 'EndEffectorCmd', 'DescriptorRef', 'CommandKey', 'ControlRef')
        modules = {name: importlib.import_module('lowlevel.ee.' + name) for name in names}
        metas = {}
        for kind in ('info', 'state', 'touch', 'cmd'):
            meta = importlib.import_module('end_effector_' + kind + '_schema_meta')
            root = 'lowlevel.ee.EndEffector' + kind.capitalize()
            if meta.ROOT_TYPE_NAME != root or hashlib.sha256(meta.SCHEMA_BFBS).hexdigest() != meta.SCHEMA_HASH_HEX:
                raise ValueError('new EE schema metadata does not match ' + root)
            metas[kind] = meta
        for name in ('EndEffectorInfo', 'EndEffectorState', 'EndEffectorTouch'):
            if not callable(getattr(modules[name], name).GetRootAs):
                raise ValueError('missing new EE root decoder: ' + name)
        return SimpleNamespace(**modules, metas=metas, aorta=importlib.import_module('aorta'))
    except (ImportError, AttributeError, ValueError, OSError) as error:
        raise AortaBindingsUnavailableError(
            'matching new generated EE roots/schema_meta are required; no public0.1.5 '
            'or legacy Hand/Gripper fallback'
        ) from error


def _vector(view, name):
    getter = _getter(name)
    return tuple(getattr(view, getter)(i) for i in range(getattr(view, getter + 'Length')()))


def _ref(view):
    if view is None:
        raise ValueError('missing DescriptorRef')
    return DescriptorRef(view.RobotRuntimeId(), view.DescriptorEpoch())


def _key(view):
    return None if view is None else CommandKey(view.SessionId(), view.Sequence())


def decode_info(view):
    regions = tuple(_dto(view.TouchRegions(i), TouchRegionInfo,
                         name=_text(view.TouchRegions(i).Name()),
                         frame=_text(view.TouchRegions(i).Frame()),
                         unit=_text(view.TouchRegions(i).Unit()),
                         shape=_vector(view.TouchRegions(i), 'shape'),
                         components=tuple(_text(v) for v in _vector(view.TouchRegions(i), 'components')),
                         geometry=_vector(view.TouchRegions(i), 'geometry'))
                    for i in range(view.TouchRegionsLength()))
    return _dto(view, EndEffectorInfo, descriptor_ref=_ref(view.DescriptorRef()),
                axis_names=tuple(_text(v) for v in _vector(view, 'axis_names')),
                firmware=_text(view.Firmware()), serial=_text(view.Serial()),
                touch_regions=regions, **{name: _vector(view, name) for name in _INFO_VECTORS})


def decode_state(view):
    target = view.FeedbackAppliedTarget()
    target = None if target is None else _dto(
        target, FeedbackAppliedTarget, **{name: _vector(target, name) for name in ('pos', 'vel_limit', 'fields')})
    progress = tuple(_dto(view.Progress(i), ProgressEntry, key=_key(view.Progress(i).Key()))
                     for i in range(view.ProgressLength()))
    return _dto(view, EndEffectorState, descriptor_ref=_ref(view.DescriptorRef()),
                received_key=_key(view.ReceivedKey()), host_target_key=_key(view.HostTargetKey()),
                feedback_applied_key=_key(view.FeedbackAppliedKey()), feedback_applied_target=target,
                actual_output_key=tuple(_key(view.ActualOutputKey(i)) for i in range(view.ActualOutputKeyLength())),
                progress=progress, **{name: _vector(view, name) for name in _STATE_VECTORS})


def decode_touch(view):
    return _dto(view, EndEffectorTouch, descriptor_ref=_ref(view.DescriptorRef()),
                region_stamps=tuple(_dto(view.RegionStamps(i), RegionStamp)
                                    for i in range(view.RegionStampsLength())),
                payload=bytes(_vector(view, 'payload')))


def _uint(value, bits, name):
    if type(value) is not int or not 0 <= value < 1 << bits:
        raise ValueError(name + ' must be uint' + str(bits))


def _tuple(value, count, name):
    if not isinstance(value, tuple) or len(value) != count:
        raise ValueError(name + ' has an incompatible fixed length')


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(name + ' must be finite')
    try:
        if not math.isfinite(value) or not math.isfinite(struct.unpack('<f', struct.pack('<f', value))[0]):
            raise ValueError(name + ' must be finite float32')
    except (OverflowError, struct.error) as error:
        raise ValueError(name + ' must fit float32') from error


def _identity(value):
    _uint(value.protocol_version, 32, 'protocol_version')
    if value.protocol_version != EE_PROTOCOL_VERSION:
        raise ValueError('incompatible new EE protocol_version')
    if not isinstance(value.descriptor_ref, DescriptorRef):
        raise ValueError('missing DescriptorRef')
    _uint(value.descriptor_ref.robot_runtime_id, 64, 'robot_runtime_id')
    _uint(value.descriptor_ref.descriptor_epoch, 64, 'descriptor_epoch')
    _uint(value.layout_id, 32, 'layout_id')
    if value.side not in (1, 2) or type(value.side) is not int:
        raise ValueError('invalid EE side')


def validate_info(info):
    _identity(info)
    n = len(info.axis_names)
    if not n or any(not isinstance(v, str) or not v.strip() for v in info.axis_names) or len(set(info.axis_names)) != n:
        raise ValueError('axis_names must be nonempty and unique')
    for name, bits in (('device_model', 16), ('identity_valid_mask', 8),
                       ('diagnostic_encoding', 16), ('time_valid_mask', 8),
                       ('ts_descriptor_acquired_us', 64), ('ts_state_pub_us', 64)):
        _uint(getattr(info, name), bits, name)
    for name, bits in (('position_domain', 8), ('torque_domain', 8), ('supported_modes', 32),
                       ('supported_command_fields', 32), ('supported_state_fields', 32), ('valid_limits', 32)):
        _tuple(getattr(info, name), n, name)
        for value in getattr(info, name):
            _uint(value, bits, name)
    for commands, states, limits, modes in zip(info.supported_command_fields,
            info.supported_state_fields, info.valid_limits, info.supported_modes):
        if commands & _KNOWN_FIELDS & ~_COMMAND_FIELDS or states & _KNOWN_FIELDS & ~_STATE_FIELDS:
            raise ValueError('field is declared in the wrong capability set')
        if limits & ~_LIMIT_FIELDS or limits & ~(commands | states) or modes & 1:
            raise ValueError('invalid limits or reserved supported_modes bit')
    _tuple(info.limit_source, n * 5, 'limit_source')
    for value in info.limit_source:
        _uint(value, 8, 'limit_source')
    for ordinal, (bit, lower, upper) in enumerate(_LIMITS):
        supported = any((c | s) & bit for c, s in zip(info.supported_command_fields, info.supported_state_fields))
        for name in (lower, upper):
            if name is not None:
                _tuple(getattr(info, name), n if supported else 0, name)
        for i, mask in enumerate(info.valid_limits):
            if mask & bit:
                maximum = getattr(info, upper)[i]
                minimum = 0.0 if lower is None else getattr(info, lower)[i]
                _finite(minimum, upper); _finite(maximum, upper)
                if minimum > maximum or info.limit_source[5 * i + ordinal] not in (
                        LimitSource.CONFIG, LimitSource.DEVICE_READBACK, LimitSource.PROTOCOL_DOCUMENT):
                    raise ValueError('invalid or unproven limit: ' + upper)
    if type(info.touch_available) is not bool or bool(info.touch_regions) != info.touch_available:
        raise ValueError('touch capability and fixed regions disagree')
    seen = set()
    end = 0
    for region in sorted(info.touch_regions, key=lambda r: r.payload_offset):
        for name, bits in (('region_id', 16), ('sensor_type', 16), ('dtype', 8),
                           ('payload_offset', 32), ('payload_bytes', 32), ('encoding_version', 16)):
            _uint(getattr(region, name), bits, name)
        if region.region_id in seen or not region.shape or not region.components:
            raise ValueError('invalid touch region identity/shape')
        seen.add(region.region_id)
        count = 1
        for size in region.shape:
            _uint(size, 32, 'touch shape')
            if not size or count > 0xffffffff // size:
                raise ValueError('touch shape overflow')
            count *= size
        if region.dtype != TouchPayloadType.UINT16_LE or region.unit != '1':
            raise ValueError('unsupported touch payload encoding/unit')
        if count * len(region.components) * 2 != region.payload_bytes:
            raise ValueError('touch region byte count differs from shape/components')
        if region.payload_offset < end or region.payload_offset + region.payload_bytes > 0xffffffff:
            raise ValueError('overlapping or overflowing touch payload')
        end = region.payload_offset + region.payload_bytes
        if type(region.geometry_valid) is not bool:
            raise ValueError('geometry_valid must be boolean')
        _tuple(region.geometry, count * 6 if region.geometry_valid else 0, 'geometry')
        for value in region.geometry:
            _finite(value, 'geometry')


def _same_endpoint(info, value):
    _identity(value)
    if (value.descriptor_ref, value.layout_id, value.side) != (info.descriptor_ref, info.layout_id, info.side):
        raise ValueError('sample DescriptorRef/layout/side does not match Info')


def _validate_key(key, valid):
    if key is None:
        if valid:
            raise ValueError('valid key is missing')
        return
    if not isinstance(key, CommandKey):
        raise ValueError('invalid CommandKey')
    _uint(key.session_id, 64, 'key.session_id'); _uint(key.sequence, 64, 'key.sequence')


def validate_state(info, state):
    _same_endpoint(info, state)
    n = len(info.axis_names)
    for name in ('state_id', 'ts_state_real_us', 'ts_state_pub_us', 'ts_acquired_hw_us',
                 'ts_ingested_us', 'ts_assembled_us', 'last_valid_age_us', 'estimated_input_period_us',
                 'reject_count', 'ingress_drop_count', 'timeout_count', 'crc_error_count'):
        _uint(getattr(state, name), 64, name)
    for name, bits in (('time_valid_mask', 8), ('sample_status', 8), ('inhibit_reason', 8),
                       ('status_valid_mask', 8), ('key_valid_mask', 8), ('last_reject_code', 16),
                       ('diagnostic_valid_mask', 32)):
        _uint(getattr(state, name), bits, name)
    for name in ('valid_fields', 'is_mock', 'actual_output_key', 'actual_output_key_valid', 'raw_valid_fields'):
        _tuple(getattr(state, name), n, name)
    for i, mask in enumerate(state.valid_fields):
        _uint(mask, 32, 'valid_fields')
        if mask & ~info.supported_state_fields[i]:
            raise ValueError('State claims an unsupported measurement')
    for name, bit in _STATE_COLUMNS:
        column = getattr(state, name)
        _tuple(column, n if any(m & bit for m in info.supported_state_fields) else 0, name)
        for i, value in enumerate(column):
            if bit == Field.ERROR_CODE:
                _uint(value, 32, name)
            elif state.valid_fields[i] & bit:
                _finite(value, name)
    for name in ('is_mock', 'actual_output_key_valid'):
        if any(type(v) is not bool for v in getattr(state, name)):
            raise ValueError('invalid boolean vector: ' + name)
    for name in ('ready', 'output_allowed', 'touch_enabled', 'progress_complete'):
        if type(getattr(state, name)) is not bool:
            raise ValueError('invalid boolean: ' + name)
    if state.status_valid_mask & 6 == 6 and state.output_allowed != (state.inhibit_reason == InhibitReason.NONE):
        raise ValueError('inconsistent output permission snapshot')
    for key, bit in ((state.received_key, 1), (state.host_target_key, 2), (state.feedback_applied_key, 4)):
        _validate_key(key, bool(state.key_valid_mask & bit))
    for key, valid in zip(state.actual_output_key, state.actual_output_key_valid):
        _validate_key(key, valid)
    target = state.feedback_applied_target
    if target is not None:
        _tuple(target.fields, n, 'feedback_applied_target.fields')
        for mask in target.fields:
            _uint(mask, 32, 'feedback target fields')
            if mask & ~(Field.POS | Field.VEL_LIMIT):
                raise ValueError('unsupported feedback target field')
        for name, bit in (('pos', Field.POS), ('vel_limit', Field.VEL_LIMIT)):
            values = getattr(target, name)
            if not isinstance(values, tuple) or len(values) not in (0, n) or any(m & bit for m in target.fields) and len(values) != n:
                raise ValueError('feedback target fixed length differs')
            for i, value in enumerate(values):
                if target.fields[i] & bit:
                    _finite(value, 'feedback target ' + name)
    keys = set()
    for entry in state.progress:
        _validate_key(entry.key, True)
        if entry.key in keys:
            raise ValueError('duplicate key in progress')
        keys.add(entry.key)
        _uint(entry.stage_valid_mask, 8, 'stage_valid_mask')
        for name in ('ts_received_us', 'ts_accepted_us', 'ts_first_output_us', 'ts_write_ack_us', 'ts_first_feedback_us'):
            _uint(getattr(entry, name), 64, name)
    for i, mask in enumerate(state.raw_valid_fields):
        _uint(mask, 8, 'raw_valid_fields')
        for ordinal, name in enumerate(('raw_pos', 'raw_vel', 'raw_current', 'raw_status')):
            values = getattr(state, name)
            if not isinstance(values, tuple) or len(values) not in (0, n) or mask & (1 << ordinal) and len(values) != n:
                raise ValueError('raw column fixed length differs')
    for name in ('raw_pos', 'raw_vel', 'raw_current', 'raw_status'):
        for value in getattr(state, name):
            if name == 'raw_status':
                _uint(value, 32, name)
            elif type(value) is not int or not -(1 << 31) <= value < 1 << 31:
                raise ValueError('raw signed value outside int32')


def validate_touch(info, touch):
    _same_endpoint(info, touch)
    if not info.touch_available:
        raise ValueError('endpoint has no touch capability')
    _uint(touch.touch_state_id, 64, 'touch_state_id')
    _uint(touch.ts_state_pub_us, 64, 'ts_state_pub_us')
    if type(touch.publish_time_valid) is not bool or not isinstance(touch.payload, bytes):
        raise ValueError('invalid touch payload/time validity')
    _tuple(touch.region_stamps, len(info.touch_regions), 'region_stamps')
    size = max(r.payload_offset + r.payload_bytes for r in info.touch_regions)
    if len(touch.payload) != size:
        raise ValueError('touch payload has an incompatible fixed length')
    for stamp in touch.region_stamps:
        for name, bits in (('sample_seq', 32), ('ts_request_us', 64), ('ts_acquired_hw_us', 64),
                           ('ts_source_us', 64), ('ts_ingested_us', 64), ('status_flags', 16),
                           ('direction_flags', 16), ('validity', 8)):
            _uint(getattr(stamp, name), bits, name)


def prepare_target(info, target):
    if not isinstance(target, EndEffectorTarget):
        raise ValueError('target must be EndEffectorTarget')
    n = len(info.axis_names)
    _tuple(target.mode, n, 'mode'); _tuple(target.command_fields, n, 'command_fields')
    if not any(target.command_fields):
        raise ValueError('target has no requested command fields')
    for i, (mode, mask) in enumerate(zip(target.mode, target.command_fields)):
        _uint(mode, 16, 'mode'); _uint(mask, 32, 'command_fields')
        if mode not in (ControlMode.POSITION, ControlMode.POSITION_TORQUE) or not info.supported_modes[i] & (1 << mode):
            raise ValueError('unsupported mode')
        if mask & ~_COMMAND_FIELDS or mask & ~info.supported_command_fields[i]:
            raise ValueError('unsupported command field')
    columns = {}
    for name, bit in _COMMAND_COLUMNS:
        values = getattr(target, name)
        supported = any(m & bit for m in info.supported_command_fields)
        if values is None:
            if any(m & bit for m in target.command_fields):
                raise ValueError('selected column is missing: ' + name)
            values = (0.0,) * n if supported else ()
        _tuple(values, n if supported else 0, name)
        for i, value in enumerate(values):
            _finite(value, name)
            if not target.command_fields[i] & bit:
                continue
            if not info.valid_limits[i] & bit:
                raise ValueError('no proven limit for ' + name)
            if bit in (Field.POS, Field.VEL, Field.VEL_LIMIT) and info.position_domain[i] not in (
                    PositionDomain.JOINT_ANGLE, PositionDomain.TOTAL_OPENING, PositionDomain.MOTOR_SHAFT):
                raise ValueError('position quantity has no declared SI domain')
            if bit == Field.TAU and info.torque_domain[i] not in (
                    TorqueDomain.JOINT_AXIS, TorqueDomain.MOTOR_SHAFT, TorqueDomain.VENDOR_UNCONFIRMED):
                raise ValueError('torque quantity has no declared domain')
            _, lower, upper = next(row for row in _LIMITS if row[0] == bit)
            minimum = 0.0 if lower is None else getattr(info, lower)[i]
            if not minimum <= value <= getattr(info, upper)[i]:
                raise ValueError(name + ' is outside declared SI limits')
        columns[name] = values
    return replace(target, **columns)


def fill_command(bindings, command, builder, header):
    if command.input_observations or command.sdk_time_valid_mask or command.ts_client_created_us or command.ts_client_send_us:
        raise ValueError('this candidate supports K=0 and unverified SDK time only')
    module = bindings.EndEffectorCmd
    control = bindings.ControlRef
    client = builder.CreateString(command.control_ref.client_id)
    lease = builder.CreateString(command.control_ref.lease_id)
    control.ControlRefStart(builder)
    control.ControlRefAddClientId(builder, client); control.ControlRefAddLeaseId(builder, lease)
    control_offset = control.ControlRefEnd(builder)
    vectors = {}
    for name, prepend in [('mode', 'PrependUint16'), ('command_fields', 'PrependUint32')] + [(name, 'PrependFloat32') for name, _ in _COMMAND_COLUMNS]:
        values = getattr(command.target, name)
        if values:
            getattr(module, 'EndEffectorCmdStart' + _getter(name) + 'Vector')(builder, len(values))
            for value in reversed(values):
                getattr(builder, prepend)(value)
            vectors[name] = builder.EndVector()
    module.EndEffectorCmdStart(builder)
    module.EndEffectorCmdAddAortaHeader(builder, header)
    module.EndEffectorCmdAddProtocolVersion(builder, command.protocol_version)
    module.EndEffectorCmdAddDescriptorRef(builder, bindings.DescriptorRef.CreateDescriptorRef(
        builder, command.descriptor_ref.robot_runtime_id, command.descriptor_ref.descriptor_epoch))
    module.EndEffectorCmdAddKey(builder, bindings.CommandKey.CreateCommandKey(
        builder, command.key.session_id, command.key.sequence))
    module.EndEffectorCmdAddLayoutId(builder, command.layout_id)
    module.EndEffectorCmdAddSide(builder, command.side)
    module.EndEffectorCmdAddControlRef(builder, control_offset)
    for name, offset in vectors.items():
        getattr(module, 'EndEffectorCmdAdd' + _getter(name))(builder, offset)
    return module.EndEffectorCmdEnd(builder)
