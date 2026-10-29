"""Bulk 31-slot vector paths for the pinned public schema, guarded by generated goldens."""
import struct
from dataclasses import fields
from typing import Any
from .body_types import (
    BODY_SLOTS, BODY_STATE_PROTOCOL_VERSION, BodyJointState, BodyImuState, BodyImuSample,
    BodyState, BodyJointCmd, BodyJointCapability, BodyMotorIdentity, BodyInfo,
    EndEffectorCapability, RobotInfo, GetRobotInfoResponse,
)

# BodyCmd3 and BodyState3 retain the same 31-slot joint struct strides.
_COMMAND = struct.Struct("<" + "fffff" * BODY_SLOTS)
_JOINT_STATE = struct.Struct("<QQQfffI")
_MOTOR = struct.Struct("<fffff")
_STATE_JOINTS_SLOT = 22  # BodyState.joints, id9
_COMMAND_JOINTS_SLOT = 20  # BodyCmd.joints, id8 after unpublished ID compaction


def create_body_cmd(builder: Any, joints: tuple[BodyJointCmd, ...]) -> int:
    if len(joints) != BODY_SLOTS:
        raise ValueError("BodyCmd requires exactly 31 joint slots")
    payload = _COMMAND.pack(*(v for joint in joints for v in (joint.q, joint.dq, joint.tau, joint.kp, joint.kd)))
    builder.StartVector(_MOTOR.size, len(joints), 4)
    end = builder.Head()
    builder.head = end - len(payload)
    builder.Bytes[builder.Head():end] = payload
    return builder.EndVector()


def _vector_field_data(view: Any, slot: int, stride: int, alignment: int) -> memoryview | None:
    data = memoryview(view._tab.Bytes).cast("B")
    pos, size = view._tab.Pos, len(data)
    def bounds(start: int, length: int) -> None:
        if start < 0 or start + length > size:
            raise ValueError("body vector outside buffer")
    bounds(pos, 4)
    vtable = pos - struct.unpack_from('<i', data, pos)[0]
    bounds(vtable, 4)
    length, object_size = struct.unpack_from('<HH', data, vtable)
    if length < 4 or length % 2 or object_size < 4:
        raise ValueError("invalid body vtable")
    bounds(vtable, length)
    bounds(pos, object_size)
    offset = struct.unpack_from('<H', data, vtable + slot)[0] if slot < length else 0
    if not offset:
        return None
    field = pos + offset
    if offset < 4 or offset + 4 > object_size or field % 4:
        raise ValueError("body vector field outside table or misaligned")
    relative = struct.unpack_from('<I', data, field)[0]
    if relative < 4:
        raise ValueError("invalid body vector offset")
    vector = field + relative
    bounds(vector, 4)
    count = struct.unpack_from('<I', data, vector)[0]
    if count != BODY_SLOTS:
        raise ValueError("body vector requires exactly 31 joint slots")
    payload = vector + 4
    if payload % alignment:
        raise ValueError("body vector payload misaligned")
    bounds(payload, count * stride)
    return data[payload:payload + count * stride]


def _require_body_joints(view: Any) -> memoryview:
    data = _vector_field_data(view, _STATE_JOINTS_SLOT, _JOINT_STATE.size, 8)
    if data is None:
        raise ValueError('body vector requires exactly 31 joint slots')
    return data


def unpack_body_joints(view: Any) -> tuple[BodyJointState, ...]:
    return tuple(BodyJointState(*v) for v in _JOINT_STATE.iter_unpack(_require_body_joints(view)))


def unpack_body_cmd(view: Any, *, slot: int = _COMMAND_JOINTS_SLOT) -> tuple[BodyJointCmd, ...]:
    if view is None:
        return ()
    data = _vector_field_data(view, slot, _MOTOR.size, 4)
    return () if data is None else tuple(BodyJointCmd(*v) for v in _MOTOR.iter_unpack(data))


def _getter(name: str) -> str:
    return ''.join(part.capitalize() for part in name.split('_'))


def _dto(view: Any, cls: Any, **overrides: Any) -> Any:
    return cls(**{f.name: overrides[f.name] if f.name in overrides
                  else getattr(view, _getter(f.name))() for f in fields(cls)})


def _text(value: bytes | str | None) -> str:
    return value.decode('utf-8') if isinstance(value, bytes) else value or ''


def decode_body_state(view: Any) -> BodyState:
    # Check the existing BODY revision before interpreting a fixed-stride struct.
    if view.ProtocolVersion() != BODY_STATE_PROTOCOL_VERSION:
        raise ValueError("incompatible BodyState protocol_version; expected BodyState 3")
    imus = []
    for i in range(view.ImuStatesLength()):
        sample = view.ImuStates(i)
        state = sample.State()
        imu = None if state is None else BodyImuState(**{
            name: tuple(getattr(state, _getter(name))(j)
                        for j in range(getattr(state, _getter(name) + 'Length')()))
            for name in ('quaternion', 'gyroscope', 'accelerometer', 'rpy')})
        imus.append(_dto(sample, BodyImuSample, state=imu))
    runtime = view.RuntimeId()
    if runtime == 0:
        raise ValueError("BodyState requires nonzero runtime_id")
    return _dto(view, BodyState, joints=unpack_body_joints(view),
                imu_states=tuple(imus), runtime_id=runtime)


_decode_body_state_python = decode_body_state  # Reference for native golden tests.


def decode_robot_info(view: Any) -> RobotInfo:
    body = view.Body()
    body_info = None if body is None else _dto(
        body, BodyInfo,
        joint_names=tuple(_text(body.JointNames(i)) for i in range(body.JointNamesLength())),
        joints=tuple(_dto(body.Joints(i), BodyJointCapability) for i in range(body.JointsLength())),
        motor_identities=tuple(_dto(body.MotorIdentities(i), BodyMotorIdentity,
                                   bus=_text(body.MotorIdentities(i).Bus()))
                               for i in range(body.MotorIdentitiesLength())))
    if view.EndEffectorsLength() != 2:
        raise ValueError('RobotInfo requires LEFT and RIGHT endpoint slots')
    from ._end_effector_codec import decode_info
    endpoints = []
    for i in range(view.EndEffectorsLength()):
        slot = view.EndEffectors(i)
        info_view = slot.Info()
        info = None if info_view is None else decode_info(info_view)
        # Schema73 consistency contract: a valid N>0-axis Info's per-axis
        # is_mock vector must all agree with the slot copy. A conflicting copy
        # is an inconsistent description, not a valid mock identity.
        if info is not None and info.axis_names:
            if len(info.is_mock) != len(info.axis_names):
                raise ValueError('endpoint Info is_mock length differs from axes')
            if any(type(value) is not bool for value in info.is_mock):
                raise ValueError('endpoint Info is_mock must be boolean')
            if slot.IsMock() != all(info.is_mock):
                raise ValueError(
                    'endpoint capability is_mock conflicts with its Info axes')
        endpoints.append(_dto(slot, EndEffectorCapability, info=info,
                              reason=_text(slot.Reason())))
    return _dto(view, RobotInfo, body=body_info, end_effectors=tuple(endpoints))


def decode_robot_info_response(view: Any) -> GetRobotInfoResponse:
    info = view.Info()
    return GetRobotInfoResponse(status=view.Status(), info=None if info is None else decode_robot_info(info),
                                error_message=_text(view.ErrorMessage()),
                                runtime_id=view.RuntimeId())
