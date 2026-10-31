"""Read-only body31 subscriptions and independently owned, lazy snapshots."""
from __future__ import annotations

import threading
import time
import uuid
import struct
from collections.abc import Callable
from dataclasses import dataclass
from functools import cached_property

from ._body_codec import decode_body_state, _require_body_joints
from .body_types import BODY_STATE_PROTOCOL_VERSION, BodyState
from .controller import _deadline, _discovered_runtime, _nonempty_text
from ._state_stream import create_state_stream


@dataclass(frozen=True, eq=False)
class BodyStateSnapshot:
    """Own a received payload and cache its decode result on first access."""

    _payload: bytes
    _decoder: Callable
    received_monotonic_ns: int
    received_realtime_ns: int

    def __post_init__(self):
        object.__setattr__(self, "_payload", bytes(self._payload))

    @cached_property
    def _decoded(self) -> BodyState | Exception:
        try:
            return decode_body_state(self._decoder(self._payload))
        except Exception as error:
            return error

    def body_state(self) -> BodyState:
        value = self._decoded
        if isinstance(value, Exception):
            raise value
        return value


class BodyStateSource:
    """Read-only state subscriber and discovery service; no control publisher."""

    def __init__(self, *, group: str = "default", node_name: str | None = None) -> None:
        self._group = _nonempty_text(group, "group")
        if node_name is not None:
            _nonempty_text(node_name, "node_name")
        self._condition = threading.Condition()
        self._latest: BodyStateSnapshot | None = None
        self._closed = False
        self._runtime_id = None
        self._runtime_changed = False
        self._stream = create_state_stream(
            node_name=node_name or "body-state-" + uuid.uuid4().hex,
            group=group, callback=self._receive)

    @property
    def group(self) -> str:
        return self._group

    def snapshot(self, timeout: float = 5.0, *,
                 after: BodyStateSnapshot | None = None) -> BodyStateSnapshot:
        """Wait for a received object other than after."""
        deadline = _deadline(timeout)
        with self._condition:
            while True:
                self._ensure_open()
                if self._runtime_id is None:
                    raise ValueError('get_robot_info must establish a valid runtime first')
                if self._latest is not None and self._latest is not after:
                    return self._latest
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("timed out waiting for body state")
                self._condition.wait(remaining)

    def get_robot_info(self, timeout: float = 5.0):
        with self._condition:
            self._ensure_open()
            observed_runtime = self._runtime_id
        reply = self._stream.get_robot_info(timeout)
        with self._condition:
            self._ensure_open()
            runtime = _discovered_runtime(reply, observed_runtime, self._runtime_id)
            if runtime is not None:
                if self._runtime_id is None:
                    self._runtime_id = runtime
                elif self._runtime_id != runtime:
                    self._runtime_changed = True
                    self._latest = None
                    self._condition.notify_all()
                    raise ValueError('robot runtime changed; create a new source')
        return reply

    def close(self) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._condition.notify_all()
        self._stream.close()

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("body state source is closed")
        if self._runtime_changed:
            raise ValueError('robot runtime changed; create a new source')

    def _receive(self, payload, decoder) -> None:
        # Direct host clock reads at callback entry, with no offset conversion.
        monotonic_ns = time.monotonic_ns()
        realtime_ns = time.time_ns()
        try:
            payload = bytes(payload)
            view = decoder(payload)
            if view.ProtocolVersion() != BODY_STATE_PROTOCOL_VERSION:
                return
            runtime = view.RuntimeId()
            _require_body_joints(view)
        except (ValueError, TypeError, AttributeError, IndexError, OverflowError, struct.error):
            return
        with self._condition:
            if (not self._closed and not self._runtime_changed and self._runtime_id is not None
                    and runtime == self._runtime_id):
                self._latest = BodyStateSnapshot(payload, decoder, monotonic_ns, realtime_ns)
                self._condition.notify_all()
