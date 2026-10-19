"""Public generated body31 fixtures, independent of selected decoding internals."""
import flatbuffers
from test_body_codec import make_state, module


def state_object(*, applied=False):
    obj = module('BodyState').BodyStateT.InitFromObj(make_state()[1])
    obj.rosterVersion = 7
    obj.tsConsumedUs = 991
    if applied:
        obj.appliedCmd = module('BodyAppliedCmd').BodyAppliedCmdT(
            action=13, sessionId=781, sequence=992, cmdId=123,
            tsFirstOutputUs=131313, sourceKeyValid=True,
            joints=[module('BodyJointCmd').BodyJointCmdT(
                q=i+.25, dq=-i, tau=2., kp=3., kd=4.) for i in range(31)])
    return obj


def pack_state(obj):
    builder = flatbuffers.Builder(1)
    builder.Finish(obj.Pack(builder))
    return bytes(builder.Output())
