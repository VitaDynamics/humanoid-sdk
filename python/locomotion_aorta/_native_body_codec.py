"""Pinned Aorta header adapter and native BODY2 encoder; no ownership logic."""
import ctypes
import hashlib
import time
from functools import lru_cache


@lru_cache(maxsize=1)
def _state_decoder():
    from . import _body_cmd_native
    import body_state_schema_meta as meta
    from .body_types import BODY_STATE_PROTOCOL_VERSION
    if (getattr(_body_cmd_native, "BODY_STATE_PROTOCOL_VERSION", None) != BODY_STATE_PROTOCOL_VERSION
            or meta.SCHEMA_HASH_HEX != _body_cmd_native.BODY_STATE_SCHEMA_SHA256
            or hashlib.sha256(meta.SCHEMA_BFBS).hexdigest()
            != _body_cmd_native.BODY_STATE_SCHEMA_SHA256):
        raise RuntimeError('native BodyState decoder requires the pinned BodyState3 schema')
    return _body_cmd_native.decode_state


def decode_body_state_native(view):
    payload, offset = ((view._tab.Bytes, view._tab.Pos) if hasattr(view, "_tab")
                       else (view, 0))
    return _state_decoder()(payload, offset)


class NativeBodyPublisher:
    def __init__(self, publisher, schema_meta):
        from . import _body_cmd_native
        from .body_types import BODY_CMD_PROTOCOL_VERSION
        if BODY_CMD_PROTOCOL_VERSION != 3:
            raise RuntimeError('native BodyCmd encoder requires the R3 BodyCmd3 contract')
        if (schema_meta.SCHEMA_HASH_HEX != _body_cmd_native.BODY_SCHEMA_SHA256
                or hashlib.sha256(schema_meta.SCHEMA_BFBS).hexdigest()
                != _body_cmd_native.BODY_SCHEMA_SHA256):
            raise RuntimeError('native BodyCmd encoder requires the pinned BODY2 schema')
        # Aorta 2026.10.20 has public publish_bytes but no public header-reserve
        # API. Keep its counter, Core clock and captured node policy exactly.
        for name in ('_seq_lock', '_sequence', '_lib', '_node_name', '_include_node', '_closed'):
            if not hasattr(publisher, name):
                raise RuntimeError('unsupported Aorta publisher header ABI: ' + name)
        self.publisher = publisher
        self.native = _body_cmd_native

    def publish(self, command, trace=None):
        from aorta.errors import AortaError, check_rc
        publisher = self.publisher
        if publisher._closed:
            raise AortaError('publish on a closed Publisher')
        measured = trace is not None
        begin = time.perf_counter_ns() if measured else 0
        with publisher._seq_lock:
            sequence = publisher._sequence
            publisher._sequence += 1
        if measured:
            trace['header_reserve_ns'] = time.perf_counter_ns() - begin
            trace['header_sequence'] = sequence
        stamp = ctypes.c_int64(0)
        begin = time.perf_counter_ns() if measured else 0
        try:
            rc = publisher._lib.aorta_now_ns(ctypes.byref(stamp))
            check_rc(publisher._lib, rc, 'aorta_now_ns')
        finally:
            if measured:
                trace['core_clock_call_wall_ns'] = time.perf_counter_ns() - begin
        begin = time.perf_counter_ns() if measured else 0
        try:
            payload, native_trace = self.native.encode(
                command, stamp.value,
                publisher._node_name if publisher._include_node else None,
                sequence, measured)
            if measured:
                trace.update(native_trace)
        finally:
            if measured:
                trace['encode_call_wall_ns'] = time.perf_counter_ns() - begin
        begin = time.perf_counter_ns() if measured else 0
        try:
            publisher.publish_bytes(payload)
        finally:
            if measured:
                # Includes ctypes marshaling/GIL reacquisition; not pure Core
                # send time, network delivery or hardware output completion.
                trace['send_call_wall_ns'] = time.perf_counter_ns() - begin
