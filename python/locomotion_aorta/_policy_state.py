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
            from ._end_effector_codec import validate_info
            from .end_effector_types import Field, PositionDomain, TorqueDomain

            info = endpoint.info
            validate_info(info)
            if info.runtime_id != response.runtime_id or info.side != endpoint.side:
                raise ValueError('endpoint Info runtime_id/side differs from service reply')
            for i, name in enumerate(info.axis_names):
                position_unit = {PositionDomain.JOINT_ANGLE: 'rad',
                                 PositionDomain.MOTOR_SHAFT: 'rad',
                                 PositionDomain.TOTAL_OPENING: 'm'}.get(info.position_domain[i], 'unknown unit')
                velocity_unit = position_unit + '/s' if position_unit != 'unknown unit' else 'unknown unit'
                torque_unit = ('N*m' if info.torque_domain[i] in
                               (TorqueDomain.JOINT_AXIS, TorqueDomain.MOTOR_SHAFT) else 'unknown unit')
                specs = ((Field.POS, 'pos', 'pos_min', 'pos_max', position_unit),
                         (Field.VEL, 'vel', 'vel_min', 'vel_max', velocity_unit),
                         (Field.VEL_LIMIT, 'vel_limit', None, 'vel_limit_max', velocity_unit),
                         (Field.TAU, 'tau', 'tau_min', 'tau_max', torque_unit),
                         (Field.CURRENT, 'current', 'current_min', 'current_max', 'A'))
                limits = []
                for bit, field, lower, upper, unit in specs:
                    if info.valid_limits[i] & bit:
                        high = getattr(info, upper)[i]
                        value = (f'<={high} {unit}' if lower is None else
                                 f'[{getattr(info, lower)[i]}, {high}] {unit}')
                    else:
                        supported = info.supported_command_fields[i] | info.supported_state_fields[i]
                        value = 'unknown' if supported & bit else 'unsupported'
                    limits.append(f'{field}={value}')
                lines.append(f'{label} {i} {name} is_mock={info.is_mock[i]} ' + ' '.join(limits))
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
