"""Explicit, read-only body31 state subscriptions and frame-owned snapshots."""
from __future__ import annotations

import math
import threading
import time
import uuid
from collections.abc import Callable
from dataclasses import dataclass
from typing import Literal

from ._body_codec import _dto, decode_body_state
from .body_types import BodyState
from ._state_stream import create_state_stream


class StateSourceClosedError(RuntimeError):
    """The source is closed; cached snapshots remain independently usable."""


class StateSourceTimeoutError(TimeoutError):
    """No qualifying frame arrived before the caller's deadline."""


@dataclass(frozen=True, slots=True)
class FrameToken:
    source_id: str
    sequence: int


@dataclass(frozen=True, slots=True)
class BodyStateMetadata:
    state_id: int
    protocol_version: int
    roster_version: int
    ts_state_real_us: int
    ts_state_pub_us: int
    ts_consumed_us: int
    attitude_valid: bool


class BodyStateSnapshot:
    """Own one immutable payload; conversions are cached only on this frame."""

    __slots__ = ("_payload", "_decoder", "_token", "_monotonic", "_realtime",
                 "_lock", "_cache")

    def __init__(self, payload: bytes, decoder: Callable, token: FrameToken,
                 received_monotonic_ns: int, received_realtime_ns: int) -> None:
        self._payload = payload if isinstance(payload, bytes) else bytes(payload)
        self._decoder = decoder
        self._token = token
        self._monotonic = received_monotonic_ns
        self._realtime = received_realtime_ns
        self._lock = threading.RLock()
        self._cache: dict = {}

    @property
    def token(self) -> FrameToken:
        return self._token

    @property
    def received_monotonic_ns(self) -> int:
        return self._monotonic

    @property
    def received_realtime_ns(self) -> int:
        return self._realtime

    def _cached(self, key, create):
        with self._lock:
            if key not in self._cache:
                self._cache[key] = create()
            return self._cache[key]

    def _view(self):
        return self._cached("view", lambda: self._decoder(self._payload))

    @property
    def metadata(self) -> BodyStateMetadata:
        return self._cached("metadata", lambda: _dto(self._view(), BodyStateMetadata))

    def body_state(self) -> BodyState:
        return self._cached("body_state", lambda: decode_body_state(self._view()))



class BodyStateSource:
    """One state subscriber, no control publishers, service clients, or lease."""

    def __init__(self, *, group: str, conversion: Literal["eager", "on_demand"],
                 node_name: str | None = None) -> None:
        if not isinstance(group, str) or not group.strip():
            raise ValueError("group must be a non-empty string")
        if conversion not in ("eager", "on_demand"):
            raise ValueError("conversion must be eager or on_demand")
        if node_name is not None and (not isinstance(node_name, str) or not node_name.strip()):
            raise ValueError("node_name must be a non-empty string")
        self._group = group
        self._conversion = conversion
        self._source_id = str(uuid.uuid4())
        self._condition = threading.Condition()
        self._sequence = 0
        self._latest: BodyStateSnapshot | None = None
        self._closed = False
        self._stream = create_state_stream(
            node_name=node_name or f"body-state-{self._source_id}", group=group,
            callback=self._receive)

    @property
    def group(self) -> str:
        return self._group

    def snapshot(self, timeout: float, *, after_token: FrameToken | None = None) -> BodyStateSnapshot:
        if (isinstance(timeout, bool) or not isinstance(timeout, (int, float))
                or math.isnan(timeout) or timeout < 0):
            raise ValueError("timeout must be non-negative (positive infinity is allowed)")
        if after_token is not None and (
                not isinstance(after_token, FrameToken)
                or after_token.source_id != self._source_id
                or isinstance(after_token.sequence, bool)
                or not isinstance(after_token.sequence, int)
                or after_token.sequence < 1):
            raise ValueError("after_token must belong to this source")
        deadline = time.monotonic() + timeout
        with self._condition:
            while True:
                self._ensure_open()
                if self._latest is not None and (
                        after_token is None or self._latest.token.sequence > after_token.sequence):
                    return self._latest
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise StateSourceTimeoutError("timed out waiting for body state")
                self._condition.wait(None if math.isinf(remaining)
                                     else min(remaining, threading.TIMEOUT_MAX))

    def close(self) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._condition.notify_all()
        self._stream.close()

    def _ensure_open(self) -> None:
        if self._closed:
            raise StateSourceClosedError("body state source is closed")

    def _receive(self, payload, decoder) -> None:
        # Both clocks are sampled directly at callback entry, never converted.
        received_monotonic_ns = time.monotonic_ns()
        received_realtime_ns = time.time_ns()
        with self._condition:
            if self._closed:
                return
            self._sequence += 1
            token = FrameToken(self._source_id, self._sequence)
        snapshot = BodyStateSnapshot(payload, decoder, token,
                                     received_monotonic_ns, received_realtime_ns)
        failure = None
        if self._conversion == "eager":
            try:
                snapshot.body_state()
            except Exception as error:
                failure = error
        with self._condition:
            if self._closed or (self._latest is not None
                                and self._latest.token.sequence >= token.sequence):
                return
            self._latest = snapshot
            self._condition.notify_all()
        if failure is not None:
            # Publish the received frame identity even when conversion failed:
            # readers must observe its error rather than an older successful DTO.
            raise failure
