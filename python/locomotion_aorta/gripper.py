"""Independent gripper topics; body EXTERNAL ownership stays separate."""
from dataclasses import asdict
import math
import uuid

from lowlevel.GripperType import GripperType

from ._gripper_lifecycle import GRIPPER_TOPIC_PREFIX, GripperClientLifecycle, _uint64
from .gripper_types import GripperCmd, GripperInfo, GripperLimitSource


def _finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def validate_info(info):
    if not isinstance(info, GripperInfo) or info.gripper_type != GripperType.AGX:
        raise ValueError('AGX GripperInfo is required')
    sources = (info.opening_min_source, info.opening_max_source, info.max_opening_rate_source)
    known = (GripperLimitSource.CONFIG, GripperLimitSource.DEVICE_READBACK,
             GripperLimitSource.PROTOCOL_DOCUMENT)
    if any(source not in known for source in sources):
        raise ValueError('GripperInfo opening/speed limits have unknown sources')
    if (info.torque_limit_source != GripperLimitSource.CONFIG
            or info.hold_torque_source != GripperLimitSource.CONFIG):
        raise ValueError('GripperInfo torque limits/default must come from CONFIG')
    if not all(_finite(value) for value in (info.opening_min_m, info.opening_max_m,
            info.max_opening_rate_m_s, info.hold_torque_nm, info.torque_limit_nm)):
        raise ValueError('GripperInfo limits must be finite')
    if (info.opening_min_m > info.opening_max_m or info.max_opening_rate_m_s < 0
            or info.torque_limit_nm < 0 or abs(info.hold_torque_nm) > info.torque_limit_nm):
        raise ValueError('GripperInfo limits are invalid')


def validate_target(info, q, dq=0.0, torque_cmd_nm=None):
    validate_info(info)
    if (not _finite(q) or not _finite(dq)
            or not info.opening_min_m <= q <= info.opening_max_m
            or not 0 <= dq <= info.max_opening_rate_m_s):
        raise ValueError('gripper q/dq is nonfinite or outside Info limits')
    if torque_cmd_nm is not None and (not _finite(torque_cmd_nm)
                                      or abs(torque_cmd_nm) > info.torque_limit_nm):
        raise ValueError('torque_cmd_nm must be finite and within configured absolute limit')


class GripperClient(GripperClientLifecycle):
    """Independent cmd/state/info lifecycle; no body ownership or CAN writer.

    Scalar q/dq/optional torque use the public generated binding. Passing an
    existing node shares its transport without transferring node ownership.
    """

    def __init__(self, side, *, node=None, _wire=None, group='default', read_only=False,
                 on_info=None, on_state=None, _session_id=None, pull_feedback=False):
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        if _session_id is not None:
            _uint64(_session_id, 'session_id', nonzero=True)
        wire = _wire
        if wire is None:
            from ._gripper_codec import load_bindings
            wire = load_bindings()
        owns_node = node is None
        if node is None:
            node = wire.bindings.aorta.Node('gripper-' + side + '-' + uuid.uuid4().hex, group)
        self._wire = wire
        super().__init__(side, node=node, bindings=wire.bindings, group=group,
                         decode_info=wire.decode_info, decode_state=wire.decode_state,
                         read_only=read_only, on_info=on_info, on_state=on_state,
                         owns_node=owns_node, _session_id=_session_id, pull_feedback=pull_feedback)

    def publish_target(self, q, dq=0.0, *, torque_cmd_nm=None, cmd_id=None, input_state=None):
        """Send q in metres and dq host cap in m/s; 0 stays configuration-default.

        None torque uses the robot's configuration; explicit 0 remains 0.
        Signed software motor-side N·m, not force N or a device effort limit.
        Validation does not acquire ownership or establish output permission.
        """
        with self._publish_lock:
            self._ensure_open()
            if self._read_only:
                raise RuntimeError('read-only GripperClient cannot publish')
            info = self.info
            validate_target(info, q, dq, torque_cmd_nm)
            state = self.state if input_state is None else input_state
            if state is not None and (state.side != self.wire_side or state.gripper_type != info.gripper_type):
                raise ValueError('input state belongs to another gripper')
            if cmd_id is not None and (type(cmd_id) is not int or not 0 <= cmd_id <= 0xffff):
                raise ValueError('cmd_id must be uint16')
            if self._publisher is None:
                self._publisher = self.node.create_publisher_typed(
                    self._wire.command_meta, self.topic + '/cmd',
                    qos=self._wire.bindings.aorta.QoS.realtime_control())
            envelope = self.reserve_envelope(cmd_id=cmd_id, input_state=state)
            command = GripperCmd(**asdict(envelope), q=float(q), dq=float(dq),
                                 torque_cmd_nm=None if torque_cmd_nm is None else float(torque_cmd_nm))
            self._publisher.publish_typed(lambda builder, header:
                                          self._wire.fill_command(command, builder, header))
            return command
