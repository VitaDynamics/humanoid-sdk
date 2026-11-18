"""Private optional native observation context; no imports of the trace backend."""
import threading

_local = threading.local()


def current_sink():
    return getattr(_local, 'sink', None)


def set_sink(sink):
    previous = current_sink()
    _local.sink = sink
    return previous


def copy_reference(source, destination):
    reference = getattr(source, '_trace_ref', None)
    if reference is not None:
        # Observation metadata is outside dataclass fields/public serialization.
        object.__setattr__(destination, '_trace_ref', reference)
    return destination


class _TraceMetadata:
    # Private provenance only: absent from dataclass fields/asdict/equality.
    __slots__ = ("_trace_ref",)
