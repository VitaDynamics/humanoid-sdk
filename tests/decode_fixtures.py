"""Public generated body31 fixtures, independent of selected decoding internals."""
import copy
import flatbuffers
from test_body_codec import make_state, module


def state_object(*, applied=False, normal=False):
    obj = module('BodyState').BodyStateT.InitFromObj(make_state()[1])
    obj.rosterVersion = 7
    obj.tsConsumedUs = 991
    if normal:
        obj.joints[14] = copy.copy(obj.joints[13])
        obj.joints[14].bodyIndex = 14
    if applied:
        obj.appliedCmd = module('BodyAppliedCmd').BodyAppliedCmdT(
            action=13, sessionId=781, sequence=992, cmdId=123,
            tsFirstOutputUs=131313, sourceKeyValid=True,
            joints=[module('BodyJointCmd').BodyJointCmdT(
                q=i+.25, dq=-i, tau=2., kp=3., kd=4.) for i in range(31)])
    return obj


def pack_state(obj, *, reverse_root=False, defaults=False, prefix=0):
    b = flatbuffers.Builder(1)
    b.ForceDefaults(defaults)
    if prefix:
        b.CreateByteVector(bytes(prefix))
    m = module('BodyState')
    if not reverse_root:
        b.Finish(obj.Pack(b))
        return bytes(b.Output())
    # Same logical message, another legal root-field write order and extra unknown field.
    imus = None
    if obj.imuStates is not None:
        offsets = [v.Pack(b) for v in obj.imuStates]
        m.BodyStateStartImuStatesVector(b, len(offsets))
        for offset in reversed(offsets): b.PrependUOffsetTRelative(offset)
        imus = b.EndVector()
    joints = None
    if obj.joints is not None:
        m.BodyStateStartJointsVector(b, len(obj.joints))
        for v in reversed(obj.joints): v.Pack(b)
        joints = b.EndVector()
    applied = obj.appliedCmd.Pack(b) if obj.appliedCmd is not None else None
    b.StartObject(12)
    b.PrependUint32Slot(11, 321, 0)
    for name, value in [('Joints', joints), ('ImuStates', imus), ('AppliedCmd', applied),
                        ('StateId', obj.stateId), ('ProtocolVersion', obj.protocolVersion),
                        ('RosterVersion', obj.rosterVersion), ('TsConsumedUs', obj.tsConsumedUs),
                        ('TsStatePubUs', obj.tsStatePubUs), ('TsStateRealUs', obj.tsStateRealUs),
                        ('AttitudeValid', obj.attitudeValid)]:
        if value is not None: getattr(m, 'BodyStateAdd'+name)(b, value)
    b.Finish(b.EndObject())
    return bytes(b.Output())
