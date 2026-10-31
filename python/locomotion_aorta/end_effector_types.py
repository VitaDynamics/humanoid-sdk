"""Development EE views for the fixed e38 contract; SI values, no old-wire adapter.

Requires the matching generated public bindings from the dependency manifest.
AortaHeader remains owned by the typed transport.
"""
from dataclasses import dataclass

from .transport import AortaBindingsUnavailableError

try:
    from lowlevel.ee.ControlMode import ControlMode
    from lowlevel.ee.DeviceModel import DeviceModel
    from lowlevel.ee.EndEffectorSide import EndEffectorSide
    from lowlevel.ee.Field import Field
    from lowlevel.ee.InhibitReason import InhibitReason
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

_POSITION_DOMAIN_UNITS = {
    PositionDomain.JOINT_ANGLE: 'rad',
    PositionDomain.MOTOR_SHAFT: 'rad',
    PositionDomain.TOTAL_OPENING: 'm',
}

EE_INFO_PROTOCOL_VERSION = 1
EE_STATE_PROTOCOL_VERSION = 2
EE_TOUCH_PROTOCOL_VERSION = 1
EE_CMD_PROTOCOL_VERSION = 2
EE_SET_TOUCH_REQUEST_VERSION = 2
EE_SET_TOUCH_RESPONSE_VERSION = 2

_INFO_TIME_ACQUIRED_VALID = 1 << 0
_INFO_TIME_PUBLISHED_VALID = 1 << 1

_STATE_READY_VALID = 1 << 0
_STATE_OUTPUT_ALLOWED_VALID = 1 << 1
_STATE_INHIBIT_REASON_VALID = 1 << 2
_STATE_TOUCH_ENABLED_VALID = 1 << 3

_TOUCH_REGION_PAYLOAD_VALID = 1 << 0
_TOUCH_REGION_REQUEST_TIME_VALID = 1 << 1
_TOUCH_REGION_HARDWARE_TIME_VALID = 1 << 2
_TOUCH_REGION_SOURCE_TIME_VALID = 1 << 3
_TOUCH_REGION_INGEST_TIME_VALID = 1 << 4
_TOUCH_REGION_SEQUENCE_VALID = 1 << 5
_TOUCH_REGION_STALE = 1 << 6

_SAMPLE_REF_IDENTITY_VALID = 1 << 0
_SAMPLE_REF_SOURCE_TIME_VALID = 1 << 1
_SAMPLE_REF_CONTENT_VALID = 1 << 2
_SAMPLE_REF_STALE = 1 << 3
_SAMPLE_REF_VALID_MASK = 0x0f


@dataclass(frozen=True, slots=True)
class CommandKey:
    session_id: int
    sequence: int


@dataclass(frozen=True, slots=True)
class SampleRef:
    runtime_id: int
    stream: int
    side: int
    sample_id: int
    source_time_us: int = 0
    validity: int = 0


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
    runtime_id: int
    layout_id: int
    side: int
    axis_names: tuple[str, ...]
    protocol_version: int = EE_INFO_PROTOCOL_VERSION
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
    touch_available: bool = False
    touch_regions: tuple[TouchRegionInfo, ...] = ()
    is_mock: tuple[bool, ...] = ()
    ts_descriptor_acquired_us: int = 0
    ts_state_pub_us: int = 0
    time_valid_mask: int = 0


@dataclass(frozen=True, slots=True)
class EndEffectorState:
    runtime_id: int
    layout_id: int
    side: int
    state_id: int
    protocol_version: int = EE_STATE_PROTOCOL_VERSION
    ts_state_real_us: int = 0
    ts_state_pub_us: int = 0
    ts_acquired_hw_us: int = 0
    ts_ingested_us: int = 0
    ts_assembled_us: int = 0
    sample_status: int = 0
    valid_fields: tuple[int, ...] = ()
    pos: tuple[float, ...] = ()
    vel: tuple[float, ...] = ()
    tau: tuple[float, ...] = ()
    current: tuple[float, ...] = ()
    temperature_c: tuple[float, ...] = ()
    error_code: tuple[int, ...] = ()
    ready: bool = False
    output_allowed: bool = False
    inhibit_reason: int = 0
    touch_enabled: bool = False
    status_valid_mask: int = 0

    @property
    def sample_ref(self) -> SampleRef:
        validity = _SAMPLE_REF_IDENTITY_VALID
        if self.ts_state_real_us:
            validity |= _SAMPLE_REF_SOURCE_TIME_VALID
        if self.sample_status == SampleStatus.OK:
            validity |= _SAMPLE_REF_CONTENT_VALID
        if self.sample_status == SampleStatus.STALE:
            validity |= _SAMPLE_REF_STALE
        return SampleRef(self.runtime_id, SampleStream.EE_STATE, self.side,
                         self.state_id,
                         self.ts_state_real_us if validity & _SAMPLE_REF_SOURCE_TIME_VALID else 0,
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
    runtime_id: int
    layout_id: int
    side: int
    touch_state_id: int
    region_stamps: tuple[RegionStamp, ...]
    payload: bytes
    protocol_version: int = EE_TOUCH_PROTOCOL_VERSION
    ts_state_pub_us: int = 0
    publish_time_valid: bool = False

    @property
    def sample_ref(self) -> SampleRef:
        # Independent regions have no common acquisition time. Publication is
        # not acquisition; never invent a simultaneous Touch source timestamp.
        validity = _SAMPLE_REF_IDENTITY_VALID
        if all(s.validity & _TOUCH_REGION_PAYLOAD_VALID for s in self.region_stamps):
            validity |= _SAMPLE_REF_CONTENT_VALID
        if any(s.validity & _TOUCH_REGION_STALE for s in self.region_stamps):
            validity |= _SAMPLE_REF_STALE
        return SampleRef(self.runtime_id, SampleStream.EE_TOUCH, self.side,
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
    tau: tuple[float, ...] | None = None
    current: tuple[float, ...] | None = None


@dataclass(frozen=True, slots=True)
class EndEffectorSubmission:
    """Returned after local typed publish returns; not robot/TX/ACK evidence."""
    runtime_id: int
    layout_id: int
    side: int
    control_id: int
    key: CommandKey
    target: EndEffectorTarget
    input_observations: tuple[SampleRef, ...] = ()
    ts_client_created_us: int = 0
    ts_client_send_us: int = 0
    protocol_version: int = EE_CMD_PROTOCOL_VERSION


@dataclass(frozen=True, slots=True)
class EndEffectorSetTouchRequest:
    runtime_id: int
    layout_id: int
    side: int
    control_id: int
    enable: bool
    protocol_version: int = EE_SET_TOUCH_REQUEST_VERSION


@dataclass(frozen=True, slots=True)
class EndEffectorSetTouchResponse:
    runtime_id: int
    side: int
    status: int
    reason: int
    protocol_version: int = EE_SET_TOUCH_RESPONSE_VERSION
