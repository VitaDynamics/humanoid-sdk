"""Generated FlatBuffers bindings only; no legacy slot or JSON wire protocol."""
from dataclasses import fields
import importlib
from types import SimpleNamespace

from .hand_types import (
    FingerTouchMeta, HandCmd, HandInfo, HandJointCmd, HandJointState, HandState, HandTouch,
)
from .transport import AortaBindingsUnavailableError, _decode_text


def load_bindings():
    try:
        modules = {name: importlib.import_module('lowlevel.' + name) for name in (
            'HandCmd', 'HandJointCmd', 'HandState', 'HandTouch', 'HandInfo')}
        for name in ('HandGetInfoRequest', 'HandGetInfoResponse',
                     'HandSetTouchRequest', 'HandSetTouchResponse'):
            modules[name] = importlib.import_module('aorta.services.humanoid_locomotion.' + name)
        for stem in ('hand_cmd', 'hand_get_info', 'hand_set_touch'):
            modules[stem + '_meta'] = importlib.import_module(stem + '_schema_meta')
        modules['aorta'] = importlib.import_module('aorta')
        return SimpleNamespace(**modules)
    except (ImportError, OSError, AttributeError) as error:
        raise AortaBindingsUnavailableError(
            'HIL only: install the vita-msgs wheel built from schemas 489488e, '
            'with Aorta 2026.10.12; legacy HandTouch bindings cannot be used'
        ) from error


def _getter(name):
    return ''.join(part.capitalize() for part in name.split('_'))


def _table(view, cls, **overrides):
    return cls(**{f.name: overrides[f.name] if f.name in overrides
                  else getattr(view, _getter(f.name))() for f in fields(cls)})


def decode_info(view):
    vectors = {name: tuple(getattr(view, getter)(i) for i in
                          range(getattr(view, getter + 'Length')()))
               for name, getter in (('q_min', 'QMin'), ('q_max', 'QMax'), ('dq_max', 'DqMax'))}
    return _table(view, HandInfo, model=_decode_text(view.Model()),
                  firmware=_decode_text(view.Firmware()),
                  joint_names=tuple(_decode_text(view.JointNames(i))
                                    for i in range(view.JointNamesLength())), **vectors)


def decode_state(view):
    joints = tuple(_table(view.Joints(i), HandJointState) for i in range(view.JointsLength()))
    if len(joints) != view.Dof():
        raise ValueError('HandState joint count differs from dof')
    # Preserve NaN/NOT_READY and all original counters; never replace missing feedback with zero.
    return _table(view, HandState, joints=joints)


def decode_touch(view):
    fingers = tuple(_table(view.Fingers(i), FingerTouchMeta) for i in range(view.FingersLength()))
    raw = tuple(view.Raw(i) for i in range(view.RawLength()))
    if any(f.word_offset + f.word_count > len(raw) for f in fingers):
        raise ValueError('HandTouch finger range exceeds raw words')
    return _table(view, HandTouch, fingers=fingers, raw=raw)


def fill_command(bindings, command, builder, header):
    module = bindings.HandCmd
    module.HandCmdStartJointsVector(builder, len(command.joints))
    for joint in reversed(command.joints):
        bindings.HandJointCmd.CreateHandJointCmd(builder, joint.q, joint.dq)
    joints = builder.EndVector()
    module.HandCmdStart(builder)
    module.HandCmdAddAortaHeader(builder, header)
    module.HandCmdAddJoints(builder, joints)
    for name in ('protocol_version', 'session_id', 'sequence', 'cmd_id',
                 'input_state_id', 'input_state_ts_real_us', 'ts_client_created_us',
                 'ts_client_send_us', 'hand_type', 'side'):
        getattr(module, 'HandCmdAdd' + _getter(name))(builder, getattr(command, name))
    return module.HandCmdEnd(builder)


def fill_request(module, name, builder, header, enable=None):
    getattr(module, name + 'Start')(builder)
    getattr(module, name + 'AddAortaHeader')(builder, header)
    if enable is not None:
        getattr(module, name + 'AddEnable')(builder, enable)
    return getattr(module, name + 'End')(builder)


def decode_cmd(view):
    return _table(view, HandCmd, joints=tuple(
        _table(view.Joints(i), HandJointCmd) for i in range(view.JointsLength())))
