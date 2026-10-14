"""Bulk fixed31 paths for the pinned public schema, guarded by generated goldens."""
import struct
from dataclasses import fields
from typing import Any
from .body_types import (
    BODY_SLOTS, BodyJointState, BodyImuState, BodyImuSample, BodyAppliedCommand,
    BodyState, HumMotorCmd, BodyJointCapability, BodyMotorIdentity, BodyInfo,
    GripperInfo, EndEffectorCapability, RobotInfo, GetRobotInfoResponse,
)

# Offsets and sizes from actual S2 generated factory/accessors, not old vectors.
_COMMAND = struct.Struct("<" + "fffff" * BODY_SLOTS)
_JOINT_STATE = struct.Struct("<H??BxHQQQfffffI")
_MOTOR = struct.Struct("<fffff")
_BODY_JOINTS_SLOT = 22  # actual generated BodyState.MotorStates, id9
_APPLIED_MOTORS_SLOT = 14  # actual generated BodyAppliedCommand.MotorCmd, id5


def create_body_cmd(builder: Any, motors: tuple[HumMotorCmd, ...]) -> int:
    if len(motors) != BODY_SLOTS:
        raise ValueError("HumCmd requires exactly31 motor slots")
    payload = _COMMAND.pack(*(v for m in motors for v in (m.q, m.dq, m.tau, m.kp, m.kd)))
    builder.Prep(4, _COMMAND.size)
    end = builder.Head()
    builder.head = end - len(payload)
    builder.Bytes[builder.Head():end] = payload
    return builder.Offset()


def _inline_data(block: Any, width: int) -> memoryview:
    data = memoryview(block._tab.Bytes).cast("B")
    pos = block._tab.Pos
    if pos < 0 or pos + width > len(data):
        raise ValueError("body inline struct outside buffer")
    return data[pos:pos + width]


def _inline_field_data(view: Any, slot: int, width: int, alignment: int) -> memoryview | None:
    data = memoryview(view._tab.Bytes).cast("B")
    pos, size = view._tab.Pos, len(data)
    def bounds(start: int, length: int) -> None:
        if start < 0 or start + length > size:
            raise ValueError("body table outside buffer")
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
    if offset < 4 or offset + width > object_size or (pos + offset) % alignment:
        raise ValueError("body inline field outside table or misaligned")
    return data[pos + offset:pos + offset + width]


def unpack_body_joints(view: Any) -> tuple[BodyJointState, ...]:
    data = _inline_field_data(view, _BODY_JOINTS_SLOT, _JOINT_STATE.size * BODY_SLOTS, 8)
    return () if data is None else tuple(BodyJointState(*v) for v in _JOINT_STATE.iter_unpack(data))


def unpack_body_cmd(block: Any) -> tuple[HumMotorCmd, ...]:
    if block is None:
        return ()
    return tuple(HumMotorCmd(*v) for v in _MOTOR.iter_unpack(_inline_data(block, _COMMAND.size)))


def _getter(name: str) -> str:
    return ''.join(part.capitalize() for part in name.split('_'))


def _dto(view: Any, cls: Any, **overrides: Any) -> Any:
    return cls(**{f.name: overrides[f.name] if f.name in overrides
                  else getattr(view, _getter(f.name))() for f in fields(cls)})


def _text(value: bytes | str | None) -> str:
    return value.decode('utf-8') if isinstance(value, bytes) else value or ''


def decode_body_state(view: Any) -> BodyState:
    imus = []
    for i in range(view.ImuStatesLength()):
        sample = view.ImuStates(i)
        state = sample.State()
        imu = None if state is None else BodyImuState(**{
            name: tuple(getattr(state, _getter(name))(j)
                        for j in range(getattr(state, _getter(name) + 'Length')()))
            for name in ('quaternion', 'gyroscope', 'accelerometer', 'rpy')})
        imus.append(_dto(sample, BodyImuSample, state=imu))
    applied = view.AppliedCmd()
    echo = None
    if applied is not None:
        data = _inline_field_data(applied, _APPLIED_MOTORS_SLOT, _COMMAND.size, 4)
        motors = () if data is None else tuple(HumMotorCmd(*v) for v in _MOTOR.iter_unpack(data))
        echo = _dto(applied, BodyAppliedCommand, motor_cmds=motors)
    return _dto(view, BodyState, motor_states=unpack_body_joints(view),
                imu_states=tuple(imus), applied_cmd=echo)


def decode_robot_info(view: Any) -> RobotInfo:
    from ._hand_codec import decode_info
    body = view.Body()
    body_info = None if body is None else _dto(
        body, BodyInfo,
        joint_names=tuple(_text(body.JointNames(i)) for i in range(body.JointNamesLength())),
        joints=tuple(_dto(body.Joints(i), BodyJointCapability) for i in range(body.JointsLength())),
        motor_identities=tuple(_dto(body.MotorIdentities(i), BodyMotorIdentity,
                                   bus=_text(body.MotorIdentities(i).Bus()))
                               for i in range(body.MotorIdentitiesLength())))
    effectors = []
    for i in range(view.EndEffectorsLength()):
        entry = view.EndEffectors(i)
        hand, gripper = entry.Hand(), entry.Gripper()
        grip = None if gripper is None else _dto(
            gripper, GripperInfo, model=_text(gripper.Model()), firmware=_text(gripper.Firmware()),
            max_opening_rate_m_s=gripper.MaxOpeningRateMS(), hold_torque_nm=gripper.HoldTorqueNm())
        effectors.append(_dto(entry, EndEffectorCapability,
                             get_info_service=_text(entry.GetInfoService()),
                             hand=None if hand is None else decode_info(hand),
                             gripper=grip, reason=_text(entry.Reason())))
    return _dto(view, RobotInfo, body=body_info, end_effectors=tuple(effectors))


def decode_robot_info_response(view: Any) -> GetRobotInfoResponse:
    info = view.Info()
    return GetRobotInfoResponse(status=view.Status(), info=None if info is None else decode_robot_info(info),
                                error_message=_text(view.ErrorMessage()))
