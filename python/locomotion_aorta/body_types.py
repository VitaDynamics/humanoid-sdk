"""Public body DTOs. Angles rad; robot/client clocks are explicitly microseconds."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .end_effector_types import EndEffectorInfo
from lowlevel.BodyCmdAction import BodyCmdAction
from lowlevel.BodySampleStatus import BodySampleStatus
from lowlevel.JointError import JointError
from lowlevel.BodyImuLocation import BodyImuLocation
from lowlevel.BodySlotRole import BodySlotRole
from aorta.services.common.ServiceStatus import ServiceStatus

BODY_SLOTS = 31
BODY_CMD_PROTOCOL_VERSION = 3
BODY_STATE_PROTOCOL_VERSION = 3


@dataclass(frozen=True, slots=True)
class DescriptorRef:
    """Shared Body/EE identity for one immutable robot process."""
    robot_runtime_id: int

@dataclass(frozen=True, slots=True)
class BodyJointCmd:
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0

@dataclass(frozen=True, slots=True)
class BodyCmd:
    joints: tuple[BodyJointCmd, ...]
    action: int = BodyCmdAction.EXTERNAL
    protocol_version: int = BODY_CMD_PROTOCOL_VERSION
    session_id: int = 0
    sequence: int = 0
    input_state_id: int = 0
    input_state_ts_real_us: int = 0
    ts_client_created_us: int = 0
    ts_client_send_us: int = 0
    root_quat: tuple[float, float, float, float] | None = None
    descriptor_ref: DescriptorRef | None = None
    control_id: int = 0

@dataclass(frozen=True, slots=True)
class BodyJointState:
    ts_acquired_hw_us: int = 0
    ts_ingested_us: int = 0
    ts_assembled_us: int = 0
    q: float = 0.0
    dq: float = 0.0
    tau_est: float = 0.0
    error_code: int = JointError.NONE  # uint32, including unlisted high-bit codes

@dataclass(frozen=True, slots=True)
class BodyImuState:
    quaternion: tuple[float, ...]
    gyroscope: tuple[float, ...]
    accelerometer: tuple[float, ...]
    rpy: tuple[float, ...]  # degrees, follows the producer's IMU contract

@dataclass(frozen=True, slots=True)
class BodyImuSample:
    location: int
    state: BodyImuState | None
    status: int
    ts_sampled_us: int = 0
    ts_acquired_hw_us: int = 0
    ts_ingested_us: int = 0
    ts_assembled_us: int = 0

@dataclass(frozen=True, slots=True)
class BodyState:
    """All-source association; valid zero keys and unobserved stamps are retained."""
    state_id: int
    joints: tuple[BodyJointState, ...]
    protocol_version: int = BODY_STATE_PROTOCOL_VERSION
    roster_version: int = 0
    ts_state_real_us: int = 0
    ts_state_pub_us: int = 0
    ts_consumed_us: int = 0
    attitude_valid: bool = False
    imu_states: tuple[BodyImuSample, ...] = ()
    applied_session_id: int = 0
    applied_sequence: int = 0
    ts_first_output_us: int = 0
    descriptor_ref: DescriptorRef | None = None
    ts_control_us: int = 0
    applied_key_valid: bool = False
    association_source_type: int = 0

@dataclass(frozen=True, slots=True)
class BodyJointCapability:
    body_index: int
    slot_role: int
    is_mock: bool
    supported_fields: int
    valid_limits: int
    q_min: float
    q_max: float
    dq_min: float
    dq_max: float
    tau_min: float
    tau_max: float
    kp_min: float
    kp_max: float
    kd_min: float
    kd_max: float

    def __post_init__(self) -> None:
        if type(self.body_index) is not int or not 0 <= self.body_index < BODY_SLOTS:
            raise ValueError('body_index must identify a Body slot')
        if type(self.is_mock) is not bool:
            raise ValueError('is_mock must be boolean')
        for name in ('supported_fields', 'valid_limits'):
            value = getattr(self, name)
            if type(value) is not int or not 0 <= value < 2**16:
                raise ValueError(name + ' must be uint16')
        for bit, name in enumerate(('q', 'dq', 'tau', 'kp', 'kd')):
            if not self.valid_limits & (1 << bit):
                continue
            low, high = getattr(self, name + '_min'), getattr(self, name + '_max')
            if (isinstance(low, bool) or isinstance(high, bool)
                    or not isinstance(low, (int, float)) or not isinstance(high, (int, float))
                    or not math.isfinite(low) or not math.isfinite(high) or low > high):
                raise ValueError('invalid valid limit: ' + name)

@dataclass(frozen=True, slots=True)
class BodyMotorIdentity:
    bus: str
    node_id: int
    present: bool

@dataclass(frozen=True, slots=True)
class BodyInfo:
    roster_version: int
    slot_count: int
    joint_names: tuple[str, ...]
    joints: tuple[BodyJointCapability, ...]
    motor_identities: tuple[BodyMotorIdentity, ...]

@dataclass(frozen=True, slots=True)
class EndEffectorCapability:
    side: int
    is_mock: bool
    info: EndEffectorInfo | None
    ready: bool
    status: int
    reason: str

    def __post_init__(self) -> None:
        if type(self.side) is not int or self.side not in (1, 2):
            raise ValueError('endpoint side must be LEFT or RIGHT')
        if type(self.is_mock) is not bool or type(self.ready) is not bool:
            raise ValueError('endpoint is_mock and ready must be boolean')
        if type(self.status) is not int or not -128 <= self.status <= 127:
            raise ValueError('endpoint status must fit the service status byte')
        if not isinstance(self.reason, str):
            raise ValueError('endpoint reason must be text')
        if self.info is not None:
            from .end_effector_types import EndEffectorInfo
            if not isinstance(self.info, EndEffectorInfo) or self.info.side != self.side:
                raise ValueError('endpoint Info must match its side')

@dataclass(frozen=True, slots=True)
class RobotInfo:
    protocol_version: int
    ts_info_us: int
    body: BodyInfo | None
    end_effectors: tuple[EndEffectorCapability, ...]

    def __post_init__(self) -> None:
        if (not isinstance(self.end_effectors, tuple) or len(self.end_effectors) != 2 or
                tuple(endpoint.side for endpoint in self.end_effectors) != (1, 2)):
            raise ValueError('RobotInfo requires ordered LEFT and RIGHT endpoint slots')

@dataclass(frozen=True, slots=True)
class GetRobotInfoResponse:
    status: int
    info: RobotInfo | None
    error_message: str
    runtime_id: int = 0
