"""User-thread Joy reception; drain snapshots before counting button edges."""

from dataclasses import dataclass
import struct
import time
from types import MappingProxyType
from typing import Mapping

from .transport import AortaBindingsUnavailableError


JOY_TOPIC = "locomotion/joy"
# At 50 Hz, ten consumer periods and a 1000 Hz capacity budget need 202 slots.
# This is capacity headroom, not a measured input rate or an input rate limit.
JOY_PULL_DEPTH = 256
JOY_FRESHNESS_NS = 1_000_000_000
_SEQUENCE_MASK = (1 << 64) - 1


@dataclass(frozen=True)
class JoySnapshot:
    axes: tuple[float, ...]
    buttons: tuple[int, ...]
    pressed_counts: Mapping[int, int]
    released_counts: Mapping[int, int]
    received_realtime_ns: int | None
    missing_samples_since_read: int
    missing_samples_total: int
    sequence_available: bool
    event_history_complete: bool
    samples_since_read: int
    samples_total: int
    sequence_diagnostics: Mapping[str, int]
    published_realtime_ns: int | None = None
    decode_errors_total: int = 0

    def is_fresh(self, now_ns: int | None = None) -> bool:
        if self.received_realtime_ns is None:
            return False
        now = time.time_ns() if now_ns is None else now_ns
        return 0 <= now - self.received_realtime_ns <= JOY_FRESHNESS_NS


class JoyReader:
    """Single-consumer pull reader sharing a caller-owned Aorta Node.

    read() drains to empty, returns the latest axes/buttons and consumes only
    the returned edge/gap increments. No callback or Python worker is created.
    First samples establish a baseline; missing events cannot be reconstructed.
    Close this reader before its node. See examples/velocity_mode/README.md for the
    finite-ring, source-identity and dequeue-timestamp boundaries.
    """

    def __init__(self, node, *, depth: int = JOY_PULL_DEPTH) -> None:
        if isinstance(depth, bool) or not isinstance(depth, int) or depth < 1:
            raise ValueError("Joy pull depth must be a positive integer")
        try:
            from aorta import QoS
            from locomotion.Joy import Joy
        except ImportError as error:
            raise AortaBindingsUnavailableError(
                "JoyReader requires the matching generated locomotion.Joy "
                "binding (public humanoid messages 0.1.5); install the complete "
                "matching wheelhouse"
            ) from error
        self._decoder = Joy.GetRootAs
        self._sub = node.create_subscriber_pull(JOY_TOPIC, depth=depth,
                                                qos=QoS.realtime_control())
        self._closed = False
        self._axes = ()
        self._buttons = None
        self._source = None
        self._sequences = {}
        self._received_ns = None
        self._published_ns = None
        self._decode_errors_total = 0
        self._samples_total = 0
        self._missing_total = 0
        self._sequence_available = False
        self._history_complete = False
        self._reset_pending()

    def _reset_pending(self) -> None:
        self._pressed = {}
        self._released = {}
        self._missing = 0
        self._samples = 0
        self._incomplete = False
        self._diagnostics = {}

    def _diagnose(self, name: str) -> None:
        self._diagnostics[name] = self._diagnostics.get(name, 0) + 1
        self._incomplete = True

    def _accept(self, payload: bytes, received_ns: int) -> None:
        # Decode the whole frame before changing edge/sequence state. Generated
        # getters own the wire contract; do not add another FlatBuffers decoder.
        view = self._decoder(payload)
        axes = tuple(view.Axes(i) for i in range(view.AxesLength()))
        buttons = tuple(view.Buttons(i) for i in range(view.ButtonsLength()))
        header = view.AortaHeader()
        sequence = header.Sequence() if header is not None else None
        raw_source = header.PublisherNode() if header is not None else None
        source = raw_source.decode("utf-8") if raw_source else None
        published_ns = header.PublishStampNs() if header is not None else None
        previous = self._buttons
        if self._samples_total and source != self._source:
            self._diagnose("source_changed")
            previous = None

        # Without publisher identity, interleaved counters cannot be separated.
        sequence_available = sequence is not None and source is not None
        if not sequence_available:
            self._diagnose("sequence_unavailable")
            self._sequences.clear()
        else:
            prior_sequence = self._sequences.get(source)
            if prior_sequence is not None:
                distance = (sequence - prior_sequence) & _SEQUENCE_MASK
                if distance == 0:
                    self._diagnose("duplicate")
                    return
                elif distance >= 1 << 63:
                    self._diagnose("reset_or_reorder")
                    # A same-name publisher may have restarted. Accept its
                    # new baseline without inventing gaps or button edges.
                    previous = None
                elif distance > 1:
                    self._missing += distance - 1
                    self._missing_total += distance - 1
                    self._incomplete = True
            self._sequences[source] = sequence

        if previous is None:
            self._incomplete = True
        else:
            if len(previous) != len(buttons):
                self._diagnose("button_count_changed")
            for index, (before, after) in enumerate(zip(previous, buttons)):
                if bool(before) != bool(after):
                    counts = self._pressed if after else self._released
                    counts[index] = counts.get(index, 0) + 1
        self._axes, self._buttons, self._source = axes, buttons, source
        self._received_ns = received_ns
        self._published_ns = published_ns
        self._sequence_available = sequence_available
        self._samples += 1
        self._samples_total += 1

    def read(self) -> JoySnapshot:
        """Drain in arrival order; skip and count malformed input frames.

        received_realtime_ns is CLOCK_REALTIME immediately after pull returns,
        not network arrival time. Freshness uses this local receipt clock;
        producer stamps are diagnostic and may use a different machine clock.
        Header gaps report observable missing sequence
        numbers; they neither locate the loss nor recover missed button edges.
        """
        if self._closed:
            raise RuntimeError("JoyReader is closed")
        while True:
            sample = self._sub.try_recv()
            if sample is None:
                break
            received_ns = time.time_ns()
            try:
                self._accept(sample.payload, received_ns)
            except (ValueError, TypeError, IndexError, AttributeError, struct.error):
                self._decode_errors_total += 1
                self._diagnose('decode_error')
        complete = (self._sequence_available and not self._incomplete
                    if self._samples else self._history_complete and not self._incomplete)
        snapshot = JoySnapshot(
            axes=self._axes, buttons=self._buttons if self._buttons is not None else (),
            pressed_counts=MappingProxyType(self._pressed),
            released_counts=MappingProxyType(self._released),
            received_realtime_ns=self._received_ns,
            missing_samples_since_read=self._missing,
            missing_samples_total=self._missing_total,
            sequence_available=self._sequence_available,
            event_history_complete=complete,
            samples_since_read=self._samples, samples_total=self._samples_total,
            sequence_diagnostics=MappingProxyType(self._diagnostics),
            published_realtime_ns=self._published_ns,
            decode_errors_total=self._decode_errors_total,
        )
        self._history_complete = complete
        self._reset_pending()
        return snapshot

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._sub.close()

    def __enter__(self):
        return self

    def __exit__(self, *_exc) -> None:
        self.close()
