"""Generated R3 EE accessors/builders; no old Hand/Gripper bridge."""
import hashlib
import importlib
import math
import struct
from dataclasses import replace
from types import SimpleNamespace

from ._body_codec import _dto, _getter, _text
from .transport import AortaBindingsUnavailableError
from .end_effector_types import (
    CommandKey, ControlMode, EE_INFO_PROTOCOL_VERSION,
    EE_STATE_PROTOCOL_VERSION, EE_TOUCH_PROTOCOL_VERSION, EE_CMD_PROTOCOL_VERSION,
    EE_SET_TOUCH_REQUEST_VERSION, EE_SET_TOUCH_RESPONSE_VERSION, EndEffectorInfo,
    EndEffectorState, EndEffectorTarget, EndEffectorTouch,
    Field, InhibitReason, PositionDomain, RegionStamp,
    TorqueDomain, TouchPayloadType, TouchRegionInfo, EndEffectorSetTouchResponse,
    EndEffectorSubmission, SampleRef, SampleStream,
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
    ('tau', Field.TAU), ('current', Field.CURRENT),
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
    'is_mock',
)
_STATE_VECTORS = (
    'valid_fields', 'pos', 'vel', 'tau', 'current', 'temperature_c', 'error_code',
)


def _schema_meta(stem, root):
    meta = importlib.import_module(stem + '_schema_meta')
    if meta.ROOT_TYPE_NAME != root or hashlib.sha256(meta.SCHEMA_BFBS).hexdigest() != meta.SCHEMA_HASH_HEX:
        raise ValueError('new EE schema metadata does not match ' + root)
    return meta


def load_bindings():
    try:
        names = ('EndEffectorInfo', 'EndEffectorState', 'EndEffectorTouch',
                 'EndEffectorCmd', 'CommandKey', 'SampleRef', 'AxisCommand')
        modules = {name: importlib.import_module('lowlevel.ee.' + name) for name in names}
        metas = {}
        for kind in ('info', 'state', 'touch', 'cmd'):
            root = 'lowlevel.ee.EndEffector' + kind.capitalize()
            metas[kind] = _schema_meta('end_effector_' + kind, root)
        for name in ('EndEffectorInfo', 'EndEffectorState', 'EndEffectorTouch'):
            if not callable(getattr(modules[name], name).GetRootAs):
                raise ValueError('missing new EE root decoder: ' + name)
        return SimpleNamespace(**modules, metas=metas, aorta=importlib.import_module('aorta'))
    except (ImportError, AttributeError, ValueError, OSError) as error:
        raise AortaBindingsUnavailableError(
            'matching new generated EE roots/schema_meta are required; no public0.1.5 '
            'or legacy Hand/Gripper fallback'
        ) from error


def load_touch_bindings():
    try:
        namespace = 'aorta.services.humanoid_locomotion.'
        return SimpleNamespace(
            request=importlib.import_module(namespace + 'EndEffectorSetTouchRequest'),
            response=importlib.import_module(namespace + 'EndEffectorSetTouchResponse'),
            request_meta=_schema_meta('end_effector_set_touch', namespace + 'EndEffectorSetTouchRequest'))
    except (ImportError, AttributeError, ValueError, OSError) as error:
        raise AortaBindingsUnavailableError('matching new generated EE set_touch binding/meta required') from error


def _vector(view, name):
    getter = _getter(name)
    return tuple(getattr(view, getter)(i) for i in range(getattr(view, getter + 'Length')()))


def _ref(view):
    if view is None:
        raise ValueError('missing runtime_id')
    return view.RuntimeId()


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
    return _dto(view, EndEffectorInfo, runtime_id=_ref(view),
                axis_names=tuple(_text(v) for v in _vector(view, 'axis_names')),
                firmware=_text(view.Firmware()), serial=_text(view.Serial()),
                touch_regions=regions, **{name: _vector(view, name) for name in _INFO_VECTORS})


def decode_state(view):
    if view.ProtocolVersion() != EE_STATE_PROTOCOL_VERSION:
        raise ValueError('incompatible EndEffectorState protocol_version')
    axes = tuple(view.Axes(i) for i in range(view.AxesLength()))
    if not axes:
        raise ValueError('EndEffectorState requires fixed axes')
    return _dto(view, EndEffectorState, runtime_id=_ref(view),
                **{name: tuple(getattr(axis, _getter(name))() for axis in axes)
                   for name in _STATE_VECTORS})


def decode_touch(view):
    return _dto(view, EndEffectorTouch, runtime_id=_ref(view),
                region_stamps=tuple(_dto(view.RegionStamps(i), RegionStamp)
                                    for i in range(view.RegionStampsLength())),
                payload=bytes(_vector(view, 'payload')))


def _uint(value, bits, name):
    if type(value) is not int or not 0 <= value < 1 << bits:
        raise ValueError(name + ' must be uint' + str(bits))


def _uint_fields(value, fields):
    for name, bits in fields:
        _uint(getattr(value, name), bits, name)


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


def _identity(value, version):
    _uint(value.protocol_version, 32, 'protocol_version')
    if value.protocol_version != version:
        raise ValueError('incompatible new EE protocol_version')
    _uint(value.runtime_id, 64, 'runtime_id')
    if not value.runtime_id:
        raise ValueError('runtime_id must be nonzero')
    _uint(value.layout_id, 32, 'layout_id')
    if value.side not in (1, 2) or type(value.side) is not int:
        raise ValueError('invalid EE side')


def validate_info(info):
    _identity(info, EE_INFO_PROTOCOL_VERSION)
    n = len(info.axis_names)
    if not n or any(not isinstance(v, str) or not v.strip() for v in info.axis_names) or len(set(info.axis_names)) != n:
        raise ValueError('axis_names must be nonempty and unique')
    _uint_fields(info, (('device_model', 16), ('identity_valid_mask', 8),
                        ('time_valid_mask', 8),
                        ('ts_descriptor_acquired_us', 64), ('ts_state_pub_us', 64)))
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
    _tuple(info.is_mock, n, 'is_mock')
    if any(type(value) is not bool for value in info.is_mock):
        raise ValueError('invalid boolean vector: is_mock')
    for bit, lower, upper in _LIMITS:
        supported = any((c | s) & bit for c, s in zip(info.supported_command_fields, info.supported_state_fields))
        for name in (lower, upper):
            if name is not None:
                _tuple(getattr(info, name), n if supported else 0, name)
        for i, mask in enumerate(info.valid_limits):
            if mask & bit:
                maximum = getattr(info, upper)[i]
                minimum = 0.0 if lower is None else getattr(info, lower)[i]
                _finite(minimum, upper); _finite(maximum, upper)
                if minimum > maximum:
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
    _identity(value, EE_STATE_PROTOCOL_VERSION if isinstance(value, EndEffectorState)
              else EE_TOUCH_PROTOCOL_VERSION)
    if (value.runtime_id, value.layout_id, value.side) != (info.runtime_id, info.layout_id, info.side):
        raise ValueError('sample runtime_id/layout/side does not match Info')


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
                 'ts_ingested_us', 'ts_assembled_us'):
        _uint(getattr(state, name), 64, name)
    _uint_fields(state, (('sample_status', 8), ('inhibit_reason', 8), ('status_valid_mask', 8)))
    _tuple(state.valid_fields, n, 'valid_fields')
    for i, mask in enumerate(state.valid_fields):
        _uint(mask, 32, 'valid_fields')
        if mask & ~info.supported_state_fields[i]:
            raise ValueError('State claims an unsupported measurement')
    for name, bit in _STATE_COLUMNS:
        column = getattr(state, name)
        _tuple(column, n, name)
        for i, value in enumerate(column):
            if bit == Field.ERROR_CODE:
                _uint(value, 32, name)
            elif state.valid_fields[i] & bit:
                _finite(value, name)
    for name in ('ready', 'output_allowed', 'touch_enabled'):
        if type(getattr(state, name)) is not bool:
            raise ValueError('invalid boolean: ' + name)
    if state.status_valid_mask & 6 == 6 and state.output_allowed != (state.inhibit_reason == InhibitReason.NONE):
        raise ValueError('inconsistent output permission snapshot')


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
        _uint_fields(stamp, (('sample_seq', 32), ('ts_request_us', 64), ('ts_acquired_hw_us', 64),
                             ('ts_source_us', 64), ('ts_ingested_us', 64), ('status_flags', 16),
                             ('direction_flags', 16), ('validity', 8)))


def prepare_target(info, target):
    if not isinstance(target, EndEffectorTarget):
        raise ValueError('target must be EndEffectorTarget')
    n = len(info.axis_names)
    _tuple(target.mode, n, 'mode'); _tuple(target.command_fields, n, 'command_fields')
    if not any(target.command_fields):
        raise ValueError('target has no requested command fields')
    for i, (mode, mask) in enumerate(zip(target.mode, target.command_fields)):
        _uint(mode, 16, 'mode'); _uint(mask, 32, 'command_fields')
        if mask and info.is_mock[i]:
            raise ValueError('target axis has no physical device')
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


def validate_observation_sources(sources):
    """The startup tuple is ordered and immutable; duplicates are errors."""
    if not isinstance(sources, tuple):
        raise ValueError('observation_sources must be a tuple')
    for source in sources:
        if not isinstance(source, tuple) or len(source) != 2:
            raise ValueError('each observation source must be a (stream, side) tuple')
        stream, side = source
        _uint(stream, 8, 'observation stream')
        _uint(side, 8, 'observation side')
        if not ((stream == SampleStream.BODY_STATE and side == 0) or
                (stream in (SampleStream.EE_STATE, SampleStream.EE_TOUCH) and side in (1, 2))):
            raise ValueError('invalid observation stream/side')
    if len(set(sources)) != len(sources):
        raise ValueError('duplicate observation source')


def validate_observations(runtime_id, observations, *, sources=None):
    _uint(runtime_id, 64, 'runtime_id')
    if not runtime_id:
        raise ValueError('runtime_id must be nonzero')
    if not isinstance(observations, tuple):
        raise ValueError('input_observations must be a tuple')
    for sample in observations:
        if not isinstance(sample, SampleRef):
            raise ValueError('input_observations must contain actual SampleRef values')
        _uint(sample.runtime_id, 64, 'runtime_id')
        if not sample.runtime_id:
            raise ValueError('runtime_id must be nonzero')
        for name in ('sample_id', 'source_time_us'):
            _uint(getattr(sample, name), 64, name)
        _uint(sample.validity, 8, 'sample validity')
        if sample.validity & ~15:
            raise ValueError('unknown SampleRef validity bits')
        if sample.runtime_id != runtime_id:
            raise ValueError('observation runtime_id differs from command runtime_id')
    actual = tuple((sample.stream, sample.side) for sample in observations)
    validate_observation_sources(actual)
    if sources is not None and actual != sources:
        raise ValueError('input_observations must match startup K and stream/side order')
    # No replay table, latest-sample replacement or time/content admission gate:
    # consecutive calls may actually use the same independent sensor sample.


def _validate_command_envelope(command):
    _identity(command, EE_CMD_PROTOCOL_VERSION)
    _uint(command.control_id, 64, 'control_id')
    if not command.control_id:
        raise ValueError('control_id must be nonzero')
    _validate_key(command.key, True)
    validate_observations(command.runtime_id, command.input_observations)
    _uint(command.ts_client_created_us, 64, 'ts_client_created_us')
    _uint(command.ts_client_send_us, 64, 'ts_client_send_us')


def decode_cmd(view):
    if view.ProtocolVersion() != EE_CMD_PROTOCOL_VERSION:
        raise ValueError('incompatible EndEffectorCmd protocol_version')
    runtime = _ref(view)
    observations = []
    for i in range(view.InputObservationsLength()):
        sample = view.InputObservations(i)
        observations.append(SampleRef(runtime, sample.Stream(),
            sample.Side(), sample.SampleId(), sample.SourceTimeUs(), sample.Validity()))
    axes = tuple(view.Axes(i) for i in range(view.AxesLength()))
    if not axes:
        raise ValueError('EndEffectorCmd requires fixed axes')
    target = EndEffectorTarget(tuple(axis.Mode() for axis in axes),
                              tuple(axis.CommandFields() for axis in axes),
                              **{name: tuple(getattr(axis, _getter(name))() for axis in axes)
                                 for name, _ in _COMMAND_COLUMNS})
    command = _dto(view, EndEffectorSubmission, runtime_id=runtime,
                key=_key(view.Key()), target=target, input_observations=tuple(observations))
    _validate_command_envelope(command)
    return command


def fill_command(bindings, command, builder, header):
    # Private writer: publish validated the caller payload before reserving,
    # then constructed identity/key/time from the existing owned context.
    module = bindings.EndEffectorCmd
    target = command.target
    n = len(target.mode)
    module.EndEffectorCmdStartInputObservationsVector(builder, len(command.input_observations))
    for sample in reversed(command.input_observations):
        bindings.SampleRef.CreateSampleRef(builder, sample.stream, sample.side,
            sample.sample_id, sample.source_time_us, sample.validity)
    observations = builder.EndVector()
    module.EndEffectorCmdStartAxesVector(builder, n)
    columns = [getattr(target, name) for name, _ in _COMMAND_COLUMNS]
    for i in reversed(range(n)):
        bindings.AxisCommand.CreateAxisCommand(builder, target.command_fields[i], target.mode[i],
            *(values[i] if values else 0.0 for values in columns))
    axes = builder.EndVector()
    previous = builder.forceDefaults
    try:
        module.EndEffectorCmdStart(builder)
        module.EndEffectorCmdAddAortaHeader(builder, header)
        module.EndEffectorCmdAddInputObservations(builder, observations)
        module.EndEffectorCmdAddAxes(builder, axes)
        builder.ForceDefaults(True)
        module.EndEffectorCmdAddProtocolVersion(builder, command.protocol_version)
        module.EndEffectorCmdAddRuntimeId(builder, command.runtime_id)
        module.EndEffectorCmdAddKey(builder, bindings.CommandKey.CreateCommandKey(
            builder, command.key.session_id, command.key.sequence))
        module.EndEffectorCmdAddLayoutId(builder, command.layout_id)
        module.EndEffectorCmdAddSide(builder, command.side)
        module.EndEffectorCmdAddControlId(builder, command.control_id)
        module.EndEffectorCmdAddTsClientCreatedUs(builder, command.ts_client_created_us)
        module.EndEffectorCmdAddTsClientSendUs(builder, command.ts_client_send_us)
        return module.EndEffectorCmdEnd(builder)
    finally:
        builder.ForceDefaults(previous)


def fill_set_touch(bindings, service, request, builder, header):
    _identity(request, EE_SET_TOUCH_REQUEST_VERSION)
    _uint(request.control_id, 64, 'control_id')
    if not request.control_id:
        raise ValueError('control_id must be nonzero')
    if type(request.enable) is not bool:
        raise ValueError('enable must be boolean')
    module = service.request
    module.EndEffectorSetTouchRequestStart(builder)
    module.EndEffectorSetTouchRequestAddAortaHeader(builder, header)
    module.EndEffectorSetTouchRequestAddProtocolVersion(builder, request.protocol_version)
    module.EndEffectorSetTouchRequestAddRuntimeId(builder, request.runtime_id)
    module.EndEffectorSetTouchRequestAddLayoutId(builder, request.layout_id)
    module.EndEffectorSetTouchRequestAddSide(builder, request.side)
    module.EndEffectorSetTouchRequestAddControlId(builder, request.control_id)
    module.EndEffectorSetTouchRequestAddEnable(builder, request.enable)
    return module.EndEffectorSetTouchRequestEnd(builder)


def decode_set_touch(view):
    return _dto(view, EndEffectorSetTouchResponse, runtime_id=_ref(view))


def validate_set_touch_response(request, response):
    _uint(response.protocol_version, 32, 'protocol_version')
    if response.protocol_version != EE_SET_TOUCH_RESPONSE_VERSION:
        raise ValueError('incompatible set_touch response protocol_version')
    _uint(response.runtime_id, 64, 'runtime_id')
    if not response.runtime_id:
        raise ValueError('runtime_id must be nonzero')
    if (response.runtime_id, response.side) != (request.runtime_id, request.side):
        raise ValueError('set_touch response runtime_id/side differs from request')
    _uint(response.side, 8, 'side')
    if type(response.status) is not int or not -128 <= response.status < 128:
        raise ValueError('status must fit shared ServiceStatus int8')
    _uint(response.reason, 16, 'reason')
