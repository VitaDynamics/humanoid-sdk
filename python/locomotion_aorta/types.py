from dataclasses import dataclass
from enum import Enum
from .body_types import DescriptorRef


PROTOCOL_VERSION = 3
CONTROL_REQUEST_VERSION = 3


class ControlOperation(str, Enum):
    PREPARE = "PREPARE"
    COMMIT = "COMMIT"
    CANCEL = "CANCEL"


class ExternalPhase(str, Enum):
    IDLE = "IDLE"
    PREPARING = "PREPARING"
    ARMED = "ARMED"
    TRANSITIONING = "TRANSITIONING"
    RUNNING = "RUNNING"
    TIMEOUT_HOLD = "TIMEOUT_HOLD"
    CANCELLED = "CANCELLED"
    EXPIRED = "EXPIRED"  # Legacy wire value; retained for decoding older servers.
    EMERGENCY = "EMERGENCY"


@dataclass(frozen=True, slots=True)
class MotorCommand:
    mode: int = 0
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0


@dataclass(frozen=True, slots=True)
class LowCmd:
    cmd_id: int
    motor_cmds: tuple[MotorCommand, ...]
    external_version: int = 0
    external_session_id: int = 0
    external_sequence: int = 0


@dataclass(frozen=True, slots=True)
class MotorState:
    q: float = 0.0
    dq: float = 0.0
    tau_est: float = 0.0
    temperature: float = 0.0


@dataclass(frozen=True, slots=True)
class HumanoidLowState:
    state_id: int
    stamp_ns: int
    motor_states: tuple[MotorState, ...]
    attitude_valid: bool = True


@dataclass(frozen=True, slots=True)
class ControlRequest:
    """Numeric control ownership and expected robot process identity."""

    request_id: str
    client_id: int
    operation: ControlOperation
    action_name: str = "EXTERNAL"
    control_id: int = 0
    requested_lease_ms: int = 0  # Legacy wire field; the controller always sends 0.
    protocol_version: int = CONTROL_REQUEST_VERSION
    descriptor_ref: DescriptorRef | None = None


@dataclass(frozen=True, slots=True)
class ControlAck:
    request_id: str
    operation: ControlOperation
    accepted: bool
    error_code: int = 0
    reason: str = ""
    control_id: int = 0
    lease_deadline_monotonic_ns: int = 0  # Legacy wire field; 0 means no expiry.
    protocol_version: int = PROTOCOL_VERSION
    descriptor_ref: DescriptorRef | None = None
    client_id: int = 0


@dataclass(frozen=True, slots=True)
class ControlStatus:
    status_seq: int
    current_action: str
    phase: ExternalPhase
    client_id: int = 0
    control_id: int = 0
    lease_deadline_monotonic_ns: int = 0  # Legacy wire field; 0 means no expiry.
    generation: int = 0
    last_accepted_cmd_id: int = 0
    last_valid_age_ms: int = 0
    estimated_input_period_us: int = 0
    timeout_hold: bool = False
    last_reject_code: int = 0
    reject_count: int = 0
    ingress_drop_count: int = 0
    stamp_monotonic_ns: int = 0
    protocol_version: int = PROTOCOL_VERSION
    descriptor_ref: DescriptorRef | None = None
