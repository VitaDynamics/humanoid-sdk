"""Development EE views for the fixed e38 contract; SI values, no old-wire adapter.

Requires the actual new generated public bindings. The existing 0.1.5 pin does
not supply them. AortaHeader remains owned by the typed transport.
"""
from dataclasses import dataclass

from .transport import AortaBindingsUnavailableError
from .body_types import DescriptorRef

try:
    from lowlevel.ee.ControlMode import ControlMode
    from lowlevel.ee.DeviceModel import DeviceModel
    from lowlevel.ee.DiagnosticEncoding import DiagnosticEncoding
    from lowlevel.ee.EndEffectorSide import EndEffectorSide
    from lowlevel.ee.Field import Field
    from lowlevel.ee.InhibitReason import InhibitReason
    from lowlevel.ee.LimitSource import LimitSource
    from lowlevel.ee.PositionDomain import PositionDomain
    from lowlevel.ee.SampleStatus import SampleStatus
    from lowlevel.ee.SampleStream import SampleStream
    from lowlevel.ee.TorqueDomain import TorqueDomain
    from lowlevel.ee.TouchPayloadType import TouchPayloadType
    from lowlevel.ee.TouchSetReason import TouchSetReason
    from aorta.services.common.ServiceStatus import ServiceStatus
except ImportError as error:
    raise AortaBindingsUnavailableError(
        "new generated EndEffector bindings are required; public 0.1.5 is not "
        "the new EE pack; no Hand/Gripper fallback"
    ) from error

EE_PROTOCOL_VERSION = 1


@dataclass(frozen=True, slots=True)
class CommandKey:
    session_id: int
    sequence: int


@dataclass(frozen=True, slots=True)
class SampleRef:
    descriptor_ref: DescriptorRef
    stream: int
    side: int
    sample_id: int
    source_time_us: int = 0
    validity: int = 0


@dataclass(frozen=True, slots=True)
class ControlRef:
    client_id: str
    lease_id: str


@dataclass(frozen=True, slots=True)
class TouchRegionInfo:
    region_id: int
    name: str
    frame: str
    sensor_type: int
    shape: tuple[int, ...]
    components: tuple[str, ...]
    dtype: int
    unit: str
    payload_offset: int
    payload_bytes: int
    encoding_version: int
    geometry: tuple[float, ...] = ()
    geometry_valid: bool = False


@dataclass(frozen=True, slots=True)
class EndEffectorInfo:
    descriptor_ref: DescriptorRef
    layout_id: int
    side: int
    axis_names: tuple[str, ...]
    protocol_version: int = EE_PROTOCOL_VERSION
    device_model: int = 0
    firmware: str = ""
    serial: str = ""
    identity_valid_mask: int = 0
    position_domain: tuple[int, ...] = ()
    torque_domain: tuple[int, ...] = ()
    supported_modes: tuple[int, ...] = ()
    supported_command_fields: tuple[int, ...] = ()
    supported_state_fields: tuple[int, ...] = ()
    valid_limits: tuple[int, ...] = ()
    pos_min: tuple[float, ...] = ()
    pos_max: tuple[float, ...] = ()
    vel_min: tuple[float, ...] = ()
    vel_max: tuple[float, ...] = ()
    vel_limit_max: tuple[float, ...] = ()
    tau_min: tuple[float, ...] = ()
    tau_max: tuple[float, ...] = ()
    current_min: tuple[float, ...] = ()
    current_max: tuple[float, ...] = ()
    limit_source: tuple[int, ...] = ()
    touch_available: bool = False
    touch_regions: tuple[TouchRegionInfo, ...] = ()
    diagnostic_encoding: int = 0
    ts_descriptor_acquired_us: int = 0
    ts_state_pub_us: int = 0
    time_valid_mask: int = 0


@dataclass(frozen=True, slots=True)
class ProgressEntry:
    key: CommandKey
    ts_received_us: int = 0
    ts_accepted_us: int = 0
    ts_first_output_us: int = 0
    ts_write_ack_us: int = 0
    ts_first_feedback_us: int = 0
    stage_valid_mask: int = 0


@dataclass(frozen=True, slots=True)
class FeedbackAppliedTarget:
    pos: tuple[float, ...] = ()
    vel_limit: tuple[float, ...] = ()
    fields: tuple[int, ...] = ()


@dataclass(frozen=True, slots=True)
class EndEffectorState:
    descriptor_ref: DescriptorRef
    layout_id: int
    side: int
    state_id: int
    protocol_version: int = EE_PROTOCOL_VERSION
    ts_state_real_us: int = 0
    ts_state_pub_us: int = 0
    ts_acquired_hw_us: int = 0
    ts_ingested_us: int = 0
    ts_assembled_us: int = 0
    time_valid_mask: int = 0
    sample_status: int = 0
    valid_fields: tuple[int, ...] = ()
    pos: tuple[float, ...] = ()
    vel: tuple[float, ...] = ()
    tau: tuple[float, ...] = ()
    current: tuple[float, ...] = ()
    temperature_c: tuple[float, ...] = ()
    error_code: tuple[int, ...] = ()
    is_mock: tuple[bool, ...] = ()
    ready: bool = False
    output_allowed: bool = False
    inhibit_reason: int = 0
    touch_enabled: bool = False
    status_valid_mask: int = 0
    received_key: CommandKey | None = None
    host_target_key: CommandKey | None = None
    key_valid_mask: int = 0
    actual_output_key: tuple[CommandKey | None, ...] = ()
    actual_output_key_valid: tuple[bool, ...] = ()
    feedback_applied_key: CommandKey | None = None
    feedback_applied_target: FeedbackAppliedTarget | None = None
    progress: tuple[ProgressEntry, ...] = ()
    progress_complete: bool = False
    last_valid_age_us: int = 0
    estimated_input_period_us: int = 0
    reject_count: int = 0
    last_reject_code: int = 0
    ingress_drop_count: int = 0
    timeout_count: int = 0
    crc_error_count: int = 0
    diagnostic_valid_mask: int = 0
    raw_pos: tuple[int, ...] = ()
    raw_vel: tuple[int, ...] = ()
    raw_current: tuple[int, ...] = ()
    raw_status: tuple[int, ...] = ()
    raw_valid_fields: tuple[int, ...] = ()

    @property
    def sample_ref(self) -> SampleRef:
        validity = 1 | (2 if self.time_valid_mask & 1 else 0)
        validity |= 4 if self.sample_status == SampleStatus.OK else 0
        validity |= 8 if self.sample_status == SampleStatus.STALE else 0
        return SampleRef(self.descriptor_ref, SampleStream.EE_STATE, self.side,
                         self.state_id, self.ts_state_real_us if validity & 2 else 0,
                         validity)


@dataclass(frozen=True, slots=True)
class RegionStamp:
    sample_seq: int
    ts_request_us: int
    ts_acquired_hw_us: int
    ts_source_us: int
    ts_ingested_us: int
    status_flags: int
    direction_flags: int
    validity: int


@dataclass(frozen=True, slots=True)
class EndEffectorTouch:
    descriptor_ref: DescriptorRef
    layout_id: int
    side: int
    touch_state_id: int
    region_stamps: tuple[RegionStamp, ...]
    payload: bytes
    protocol_version: int = EE_PROTOCOL_VERSION
    ts_state_pub_us: int = 0
    publish_time_valid: bool = False

    @property
    def sample_ref(self) -> SampleRef:
        # Independent regions have no common acquisition time. Publication is
        # not acquisition; never invent a simultaneous Touch source timestamp.
        validity = 1 | (4 if all(s.validity & 1 for s in self.region_stamps) else 0)
        validity |= 8 if any(s.validity & 64 for s in self.region_stamps) else 0
        return SampleRef(self.descriptor_ref, SampleStream.EE_TOUCH, self.side,
                         self.touch_state_id, 0, validity)


@dataclass(frozen=True, slots=True)
class EndEffectorTarget:
    """Fixed-axis SI target; command_fields is the sole per-axis intent mask.

    An omitted numeric column has no intent bits and becomes zero placeholders
    on a supported wire column. A selected explicit zero stays selected.
    """
    mode: tuple[int, ...]
    command_fields: tuple[int, ...]
    pos: tuple[float, ...] | None = None
    vel: tuple[float, ...] | None = None
    vel_limit: tuple[float, ...] | None = None
    tau_cmd: tuple[float, ...] | None = None
    current_cmd: tuple[float, ...] | None = None


@dataclass(frozen=True, slots=True)
class EndEffectorSubmission:
    """Returned after local typed publish returns; not robot/TX/ACK evidence."""
    descriptor_ref: DescriptorRef
    layout_id: int
    side: int
    control_ref: ControlRef
    key: CommandKey
    target: EndEffectorTarget
    input_observations: tuple[SampleRef, ...] = ()
    ts_client_created_us: int = 0
    ts_client_send_us: int = 0
    sdk_time_valid_mask: int = 0
    protocol_version: int = EE_PROTOCOL_VERSION


@dataclass(frozen=True, slots=True)
class EndEffectorSetTouchRequest:
    descriptor_ref: DescriptorRef
    layout_id: int
    side: int
    control_ref: ControlRef
    enable: bool
    protocol_version: int = EE_PROTOCOL_VERSION


@dataclass(frozen=True, slots=True)
class EndEffectorSetTouchResponse:
    descriptor_ref: DescriptorRef
    side: int
    status: int
    reason: int
    enabled: bool
    enabled_valid: bool
    protocol_version: int = EE_PROTOCOL_VERSION
