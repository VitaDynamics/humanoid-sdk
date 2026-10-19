"""Shared public-body policy input; no inference or control ownership."""
from dataclasses import dataclass, fields
from .body_types import BodyState

ASAMPLE_BODY_JOINT_NAMES = (
    'left_hip_pitch_joint',
    'left_hip_roll_joint',
    'left_hip_yaw_joint',
    'left_knee_joint',
    'left_ankle_pitch_joint',
    'left_ankle_roll_joint',
    'right_hip_pitch_joint',
    'right_hip_roll_joint',
    'right_hip_yaw_joint',
    'right_knee_joint',
    'right_ankle_pitch_joint',
    'right_ankle_roll_joint',
    'waist_yaw_joint',
    'waist_pitch_joint',
    'waist_roll_joint',
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
    'left_wrist_yaw_joint',
    'left_wrist_pitch_joint',
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
    'right_wrist_yaw_joint',
    'right_wrist_pitch_joint',
    'neck_pitch_joint',
    'neck_yaw_joint',
)


def body_joint_order(body_info=None, *, required_names=()):
    names = tuple(body_info.joint_names) if body_info is not None else ASAMPLE_BODY_JOINT_NAMES
    required_names = tuple(dict.fromkeys(required_names))
    missing = [name for name in required_names if name not in names]
    if missing:
        raise ValueError('get_robot_info roster is missing joints: ' + ', '.join(missing))
    if len(names) != 31 or len(set(names)) != 31:
        raise ValueError("expected 31 unique public body joint names")
    if body_info is not None and required_names:
        if len(body_info.joints) != len(names):
            raise ValueError('get_robot_info capability count differs from roster')
        missing = [name for name in required_names if not body_info.joints[names.index(name)].populated]
        if missing:
            raise ValueError('policy joints are unpopulated: ' + ', '.join(missing))
    return names


@dataclass(frozen=True, slots=True)
class PolicyFeedback(BodyState):
    received_monotonic_ns: int = 0
    received_realtime_ns: int = 0

    @property
    def stamp_ns(self):
        return self.ts_state_real_us * 1000

    @classmethod
    def from_snapshot(cls, snapshot):
        state = snapshot.body_state()
        return cls(**{f.name: getattr(state, f.name) for f in fields(BodyState)},
                   received_monotonic_ns=snapshot.received_monotonic_ns,
                   received_realtime_ns=snapshot.received_realtime_ns)


class PolicyStateSource:
    def __init__(self, source):
        self.source, self.previous = source, None

    def wait_body_state(self, timeout):
        self.previous = self.source.snapshot(timeout, after=self.previous)
        return PolicyFeedback.from_snapshot(self.previous)
