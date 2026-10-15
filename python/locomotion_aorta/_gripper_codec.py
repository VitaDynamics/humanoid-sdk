"""Generated single-DOF gripper codecs; no legacy layout fallback."""
import importlib
from types import SimpleNamespace

from locomotion_aorta._hand_codec import _getter, _table
from locomotion_aorta.transport import AortaBindingsUnavailableError, _decode_text
from .gripper_types import GripperCmd, GripperInfo, GripperSample, GripperHostTarget, GripperState


def load_bindings():
    try:
        modules = {name: importlib.import_module('lowlevel.' + name) for name in (
            'GripperCmd', 'GripperState', 'GripperInfo', 'GripperSample', 'GripperHostTarget')}
        modules['aorta'] = importlib.import_module('aorta')
        modules['gripper_cmd_meta'] = importlib.import_module('gripper_cmd_schema_meta')
        bindings = SimpleNamespace(**modules)
        # The older same-version candidate has incompatible gripper payloads.
        for name in ('GripperCmdAddQ', 'GripperCmdAddDq', 'GripperCmdAddTorqueCmdNm'):
            getattr(bindings.GripperCmd, name)
        for name in ('Sample', 'HostTarget', 'TraceSessionId', 'TsCmdReceivedUs'):
            getattr(bindings.GripperState.GripperState, name)
        return SimpleNamespace(bindings=bindings, command_meta=bindings.gripper_cmd_meta,
            decode_info=decode_info, decode_state=decode_state,
            fill_command=lambda command, builder, header: fill_command(bindings, command, builder, header))
    except (ImportError, AttributeError, OSError) as error:
        raise AortaBindingsUnavailableError('matching T027 public gripper bindings are required; '
                                           'no old gripper layout or LowCmd fallback') from error


def decode_info(view):
    return _table(view, GripperInfo, model=_decode_text(view.Model()),
                  firmware=_decode_text(view.Firmware()))


def decode_state(view):
    sample, target = view.Sample(), view.HostTarget()
    return _table(view, GripperState,
                  sample=None if sample is None else _table(sample, GripperSample),
                  host_target=None if target is None else _table(target, GripperHostTarget))


def fill_command(bindings, command, builder, header):
    module = bindings.GripperCmd
    module.GripperCmdStart(builder)
    module.GripperCmdAddAortaHeader(builder, header)
    for name in ('protocol_version', 'session_id', 'sequence', 'cmd_id',
                 'input_state_id', 'input_state_ts_real_us', 'ts_client_created_us',
                 'ts_client_send_us', 'gripper_type', 'side', 'q', 'dq'):
        getattr(module, 'GripperCmdAdd' + _getter(name))(builder, getattr(command, name))
    if command.torque_cmd_nm is not None:
        module.GripperCmdAddTorqueCmdNm(builder, command.torque_cmd_nm)
    return module.GripperCmdEnd(builder)


def decode_cmd(view):
    return _table(view, GripperCmd)
