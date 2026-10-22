"""Synthetic typed fixtures; no claim of robot capability discovery."""
from locomotion_aorta import (
    BodyJointState, BodyInfo, BodyJointCapability, BodySlotRole,
    BodyMotorIdentity, RobotInfo, GetRobotInfoResponse, ServiceStatus,
)

def sample_motor(*, q=0.0, dq=0.0, body_index=0, populated=True):
    return BodyJointState(q=q, dq=dq)

def robot_info_reply():
    joints = tuple(BodyJointCapability(i, BodySlotRole.ACTIVE, False, i != 14,
                                      31 if i != 14 else 0, 31 if i != 14 else 0,
                                      -3., 3., -10., 10., -20., 20., 0., 200., 0., 10.) for i in range(31))
    identities = tuple(BodyMotorIdentity('fake-can', i + 1, i != 14) for i in range(31))
    body = BodyInfo(0, 31, tuple('fake_joint_' + str(i) for i in range(31)), joints, identities)
    return GetRobotInfoResponse(ServiceStatus.SUCCESS, RobotInfo(1, 0, body, ()), '')


def body_array_frame(state):
    """Array-shaped feedback for pull-based example tests."""
    from dataclasses import fields
    from types import SimpleNamespace
    return SimpleNamespace(**{field.name: getattr(state, field.name) for field in fields(state)},
                           q=tuple(j.q for j in state.joints),
                           dq=tuple(j.dq for j in state.joints),
                           error_code=tuple(j.error_code for j in state.joints))
