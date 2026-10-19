"""Public generated body31 fixtures, independent of selected decoding internals."""
import flatbuffers
from test_body_codec import make_state, module


def state_object(*, applied=False):
    obj = module('BodyState').BodyStateT.InitFromObj(make_state()[1])
    obj.rosterVersion = 7
    obj.tsConsumedUs = 991
    if applied:
        obj.appliedSessionId = 781
        obj.appliedSequence = 992
        obj.tsFirstOutputUs = 131313
    return obj


def pack_state(obj):
    builder = flatbuffers.Builder(1)
    builder.Finish(obj.Pack(builder))
    return bytes(builder.Output())
