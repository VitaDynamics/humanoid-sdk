"""Shared public-body policy input; no inference or control ownership."""
from dataclasses import dataclass, fields
import math
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
        if (len(body_info.joints) != len(names)
                or any(cap.body_index != i for i, cap in enumerate(body_info.joints))):
            raise ValueError('get_robot_info capability count differs from roster')
        unavailable = [name for name in required_names if body_info.joints[names.index(name)].is_mock]
        if unavailable:
            raise ValueError('policy joints are unavailable: ' + ', '.join(unavailable))
        missing = [name for name in required_names
                   if (body_info.joints[names.index(name)].supported_fields & 0x19) != 0x19
                   or not body_info.joints[names.index(name)].valid_limits & 1
                   or not math.isfinite(body_info.joints[names.index(name)].q_min)
                   or not math.isfinite(body_info.joints[names.index(name)].q_max)
                   or body_info.joints[names.index(name)].q_min > body_info.joints[names.index(name)].q_max]
        if missing:
            raise ValueError('policy joint position capability or limit unavailable: ' + ', '.join(missing))
    return names


def format_robot_info(response):
    """Format the already discovered service reply once; this does not grant control."""
    from .body_types import ServiceStatus

    if (response.status not in (ServiceStatus.SUCCESS, ServiceStatus.PARTIAL_SUCCESS)
            or type(response.runtime_id) is not int or not 0 < response.runtime_id < 2**64
            or response.info is None or response.info.body is None):
        raise ValueError('validated robot Info and runtime_id are required')
    body = response.info.body
    names = body_joint_order(body)
    if (body.slot_count != 31 or len(body.joints) != 31
            or any(cap.body_index != i for i, cap in enumerate(body.joints))):
        raise ValueError('robot Info body capabilities do not match roster')
    lines = [f'Body Info: status={response.status} reason={response.error_message}',
             'index name is_mock limits']
    units = (('q', 'rad'), ('dq', 'rad/s'), ('tau', 'N*m'),
             ('kp', 'N*m/rad'), ('kd', 'N*m/(rad/s)'))
    for i, (name, cap) in enumerate(zip(names, body.joints)):
        if type(cap.is_mock) is not bool:
            raise ValueError('invalid is_mock in robot Info')
        limits = []
        for bit, (field, unit) in enumerate(units):
            if cap.valid_limits & (1 << bit):
                low, high = getattr(cap, field + '_min'), getattr(cap, field + '_max')
                if (isinstance(low, bool) or isinstance(high, bool)
                        or not isinstance(low, (int, float)) or not isinstance(high, (int, float))
                        or not math.isfinite(low) or not math.isfinite(high) or low > high):
                    raise ValueError('invalid public limit: ' + field)
                value = f'[{low}, {high}] {unit}'
            else:
                value = 'unknown' if cap.supported_fields & (1 << bit) else 'unsupported'
            limits.append(f'{field}={value}')
        lines.append(f'{i} {name} is_mock={cap.is_mock} ' + ' '.join(limits))
    endpoints = response.info.end_effectors
    if len(endpoints) != 2 or tuple(endpoint.side for endpoint in endpoints) != (1, 2):
        raise ValueError('robot Info endpoint slots are invalid')
    for label, endpoint in zip(('LEFT', 'RIGHT'), endpoints):
        lines.append(f'{label}: is_mock={endpoint.is_mock} ready={endpoint.ready} '
                     f'status={endpoint.status} reason={endpoint.reason}')
        if endpoint.info is not None:
            lines.append(f'{label} axis limits: pending direct runtime_id Info validation')
    return '\n'.join(lines)


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
