"""Pinned Aorta header adapter and native BodyCmd3 encoder."""
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


def prepare_native_body_pull(subscriber, *, slot_count, max_payload_bytes,
                             max_slices):
    """Cold slots on this receiver/Core; the native owner retains the binding."""
    _state_decoder()  # Preserve the real native protocol/BFBS gate before new.
    from . import _body_cmd_native
    binding = subscriber.native_rx_binding(
        slot_count=slot_count, max_payload_bytes=max_payload_bytes,
        max_slices=max_slices)
    try:
        reader = _body_cmd_native.NativeBodyPullReader(binding)
    except BaseException as primary:
        try:
            binding.close()  # Transferred slots were unwound by the native ctor.
        except BaseException as cleanup:
            raise primary from cleanup
        raise
    try:
        binding.close()  # Refuse more construction, not receiver/slot drain.
    except BaseException as primary:
        from aorta.errors import exception_for_code
        rc = reader.close()
        if rc:
            raise primary from exception_for_code(rc, 'native Body pull cleanup')
        raise
    return reader


def _close_header(context):
    from aorta.errors import exception_for_code
    rc = context.close()
    if rc:
        raise exception_for_code(rc, 'native header context free failed; handle consumed')


def _prepare_header(publisher, factory):
    prepare = getattr(publisher, 'native_header_binding', None)
    if not callable(prepare):
        raise RuntimeError('public native_header_binding from the fixed bee source is required')
    binding = prepare()
    try:
        context = factory(binding)
    except BaseException as primary:
        try:
            binding.close()
        except BaseException as cleanup:
            raise primary from cleanup
        raise
    try:
        binding.close()  # The native clone retains this binding/library independently.
    except BaseException as primary:
        try:
            _close_header(context)
        except BaseException as cleanup:
            raise primary from cleanup
        raise
    return context


class NativeBodyPublisher:
    def __init__(self, publisher, schema_meta):
        from . import _body_cmd_native
        from .body_types import BODY_CMD_PROTOCOL_VERSION
        if (BODY_CMD_PROTOCOL_VERSION != 3 or
                getattr(_body_cmd_native, 'BODY_CMD_PROTOCOL_VERSION', None) != 3):
            raise RuntimeError('native BodyCmd encoder requires the R3 BodyCmd3 contract')
        if (schema_meta.SCHEMA_HASH_HEX != _body_cmd_native.BODY_SCHEMA_SHA256
                or hashlib.sha256(schema_meta.SCHEMA_BFBS).hexdigest()
                != _body_cmd_native.BODY_SCHEMA_SHA256):
            raise RuntimeError('native BodyCmd encoder requires the pinned BodyCmd3 schema')
        self.publisher = publisher
        self.native = _body_cmd_native
        self._header = _prepare_header(publisher, self.native.HeaderContext)

    def close(self):
        _close_header(self._header)

    def publish(self, command, trace=None, guard=None):
        from aorta.errors import exception_for_code
        measured = trace is not None
        begin = time.perf_counter_ns() if measured else 0
        try:
            rc, payload, native_trace = self.native.encode_with_header(command, self._header, measured)
            if rc:
                # This lane returns codes; last_error may belong to an older call.
                raise exception_for_code(rc, 'native header reservation failed')
            if measured:
                trace.update(native_trace)
        finally:
            if measured:
                trace['encode_call_wall_ns'] = time.perf_counter_ns() - begin
        begin = time.perf_counter_ns() if measured else 0
        try:
            if guard is None:
                raise RuntimeError('BodyCmd submission requires the connection gate')
            guard(lambda: self.publisher.publish_bytes(payload))
        finally:
            if measured:
                trace['send_call_wall_ns'] = time.perf_counter_ns() - begin


class NativeEEPublisher:
    """Cold frozen layout; encode outside the connection's final send gate.

    This still returns a copied bytes payload. It does not claim owned TX or
    the upstream bounded RX/aggregate drain contract.
    """
    def __init__(self, publisher, schema_meta, info, sources):
        from . import _body_cmd_native as native, _end_effector_codec as codec
        codec.validate_info(info)
        codec.validate_observation_sources(sources)
        if (getattr(native, 'EE_CMD_PROTOCOL_VERSION', None) != 2 or
                schema_meta.SCHEMA_HASH_HEX != native.EE_CMD_SCHEMA_SHA256 or
                hashlib.sha256(schema_meta.SCHEMA_BFBS).hexdigest() != native.EE_CMD_SCHEMA_SHA256):
            raise RuntimeError('native EE encoder requires the pinned EndEffectorCmd2 schema')
        self.publisher = publisher
        self._closed = False
        self._header = _prepare_header(publisher, lambda binding: native.EECommandContext(
            binding, info.runtime_id, info.layout_id, info.side, len(info.axis_names), sources))

    def encode(self, command):
        from aorta.errors import exception_for_code
        rc, payload, _ = self._header.encode(command)
        if rc:
            raise exception_for_code(rc, 'native EE header reservation failed')
        return payload

    def publish_bytes(self, payload):
        self.publisher.publish_bytes(payload)

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            _close_header(self._header)
        except BaseException as primary:
            try:
                self.publisher.close()
            except BaseException as cleanup:
                raise primary from cleanup
            raise
        self.publisher.close()
