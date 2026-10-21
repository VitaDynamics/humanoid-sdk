"""Read-only HIL capture using generated bindings; never creates publishers.

Capture timestamps are observer-local, not robot or sensor clocks. Sequence
checks by downstream judges do not prove absence of transport loss at edges.
"""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import importlib
import json
import math
from pathlib import Path
import queue
import re
import threading
import time

from . import _hand_codec, _gripper_codec
from .transport import _decode_control_status
from ._body_codec import decode_body_state, unpack_body_cmd
from ._motor_codec import unpack_motor_states
from .types import HumanoidLowState
from .body_types import BodyCmd, BODY_SLOTS


TOPICS = {
    "body_state": "/sdk/foot_humanoid/body_state",
    "body_cmd": "/sdk/foot_humanoid/body_cmd",
    "control_status": "/sdk/foot_humanoid/control/status",
}
# Historical MCAP kinds are explicit offline choices, never live or command fallbacks.
ARCHIVE_TOPICS = {
    "lowstate": "/sdk/foot_humanoid/lowstate",
    "external_command": "/locomotion/external_command",
}
MODULES = {
    "body_state": ("lowlevel.BodyState", "BodyState"),
    "body_cmd": ("lowlevel.BodyCmd", "BodyCmd"),
    "lowstate": ("lowlevel.HumanoidLowState", "HumanoidLowState"),
    "external_command": ("lowlevel.LowCmd", "LowCmd"),
    "control_status": ("locomotion_sdk.ControlStatus", "ControlStatus"),
}
for _side in ("left", "right"):
    for _kind, _root in (("cmd", "HandCmd"), ("state", "HandState"),
                         ("touch", "HandTouch"), ("info", "HandInfo")):
        _key = f"{_side}_hand_{_kind}"
        TOPICS[_key] = f"/sdk/foot_humanoid/hand/{_side}/{_kind}"
        MODULES[_key] = ("lowlevel." + _root, _root)
    for _kind, _root in (("cmd", "GripperCmd"), ("state", "GripperState"), ("info", "GripperInfo")):
        _key = f"{_side}_gripper_{_kind}"
        TOPICS[_key] = f"/sdk/foot_humanoid/gripper/{_side}/{_kind}"
        MODULES[_key] = ("lowlevel." + _root, _root)


def metadata_checked(metadata):
    if (not isinstance(metadata, dict) or type(metadata.get("format_version")) is not int
            or metadata["format_version"] != 1):
        raise ValueError("metadata format_version=1 required")
    for field in ("device", "boot_id"):
        if not isinstance(metadata.get(field), str) or not metadata[field].strip():
            raise ValueError(f"metadata.{field} required")
    for field, size in (("source_commit", 40), ("config_sha256", 64), ("schema_sha256", 64)):
        if not isinstance(metadata.get(field), str) or not re.fullmatch(rf"[0-9a-f]{{{size}}}", metadata[field]):
            raise ValueError(f"metadata.{field} requires {size} hex digits")
    sides = metadata.get("expected_sides")
    if sides not in ([1], [2], [1, 2]) or any(type(side) is not int for side in sides):
        raise ValueError("expected_sides must be [1], [2], or [1,2]")
    return dict(metadata)


def load_bindings():
    try:
        return {kind: getattr(importlib.import_module(module), name).GetRootAs
                for kind, (module, name) in MODULES.items()}
    except (ImportError, AttributeError) as error:
        raise RuntimeError("matching vbot-humanoid-msgs public bindings are required, including "
                           "lowlevel.HandTouch; no handwritten decoder fallback") from error


def _json_feedback(result):
    nonfinite = False
    def value(item):
        nonlocal nonfinite
        if isinstance(item, float) and not math.isfinite(item):
            nonfinite = True
            return None
        if isinstance(item, dict):
            return {key: value(v) for key, v in item.items()}
        if isinstance(item, (tuple, list)):
            return [value(v) for v in item]
        return item
    result = value(result)
    result["nonfinite_feedback"] = nonfinite
    return result


def binding_schema_hash(kind):
    root = MODULES[kind][1]
    stem = re.sub(r'(?<!^)(?=[A-Z])', '_', root).lower()
    module = importlib.import_module(stem + '_schema_meta')
    return hashlib.sha256(module.SCHEMA_BFBS).hexdigest()


def decode(kind, payload, bindings, *, native_body=False):
    if not isinstance(payload, bytes) or not 8 <= len(payload) <= 65536:
        raise ValueError("invalid or oversized FlatBuffer payload")
    view = bindings[kind](payload)
    if "_gripper_" in kind:
        side, gripper_kind = kind.split("_gripper_")
        record = getattr(_gripper_codec, "decode_" + gripper_kind)(view)
        if record.side != (1 if side == "left" else 2):
            raise ValueError("Gripper message side differs from selected topic")
        return _json_feedback(asdict(record))
    if "_hand_" in kind:
        side, hand_kind = kind.split("_hand_")
        record = getattr(_hand_codec, "decode_" + hand_kind)(view)
        if record.side != (1 if side == "left" else 2):
            raise ValueError("Hand message side differs from selected topic")
        # NOT_READY and an unpopulated body slot may carry NaN; JSON null
        # records absence without synthesizing a healthy zero.
        return _json_feedback(asdict(record))
    if kind == "body_state":
        if native_body:
            from .state_arrays import decode_body_state_arrays
            state = decode_body_state_arrays(payload).body_state()
        else:
            state = decode_body_state(view)
        if len(state.joints) != BODY_SLOTS:
            raise ValueError("requires31-slot BodyState")
        return _json_feedback(asdict(state))
    if kind == "body_cmd":
        motors = unpack_body_cmd(view)
        if len(motors) != BODY_SLOTS:
            raise ValueError("requires31-slot BodyCmd")
        quat = view.RootQuat()
        command = BodyCmd(joints=motors,
                         protocol_version=view.ProtocolVersion(), action=view.Action(),
                         session_id=view.SessionId(), sequence=view.Sequence(),
                         input_state_id=view.InputStateId(), input_state_ts_real_us=view.InputStateTsRealUs(),
                         ts_client_created_us=view.TsClientCreatedUs(), ts_client_send_us=view.TsClientSendUs(),
                         root_quat=None if quat is None else (quat.W(),quat.X(),quat.Y(),quat.Z()))
        return _json_feedback(asdict(command))
    if kind == "lowstate":
        if view.MotorStatesLength() != 44:
            raise ValueError("requires 44-slot humanoid LowState")
        state = HumanoidLowState(state_id=view.StateId(), stamp_ns=view.TsStateReal() * 1000,
                                 motor_states=unpack_motor_states(view), attitude_valid=view.AttitudeValid())
        result = asdict(state)
        result["input_cmd_id"] = view.InputCmdId()
        result["per_motor_freshness"] = "not_available"
    elif kind == "external_command":
        if view.MotorCmdLength() != 42:
            raise ValueError("requires complete 42-slot LowCmd")
        result = {"cmd_id": view.CmdId(), "external_version": view.ExternalVersion(),
                  "external_session_id": view.ExternalSessionId(),
                  "external_sequence": view.ExternalSequence(), "motor_cmds": []}
        for slot in range(42):
            motor = view.MotorCmd(slot)
            result["motor_cmds"].append({name: getattr(motor, name.capitalize())()
                                         for name in ("mode", "q", "dq", "tau", "kp", "kd")})
    else:
        result = asdict(_decode_control_status(view))
        result["phase"] = result["phase"].value
    # Reject non-finite feedback instead of silently serializing NaN as JSON.
    json.dumps(result, allow_nan=False)
    return result


class TraceWriter:
    def __init__(self, stream, metadata, clock):
        self.stream = stream
        self.counts = {}
        self.write({**metadata_checked(metadata), "type": "metadata",
                    "capture_format": "brainco_capture_v3", "capture_clock": clock,
                    "transport_loss": "not_measured"})

    def write(self, record):
        self.stream.write(json.dumps(record, allow_nan=False) + "\n")

    def sample(self, kind, payload, timestamp_ns, bindings, *, native_body=False, **context):
        record = decode(kind, payload, bindings, native_body=native_body)
        self.write({**record, "type": kind, "capture_timestamp_ns": timestamp_ns,
                    "payload_sha256": hashlib.sha256(payload).hexdigest(), **context})
        self.counts[kind] = self.counts.get(kind, 0) + 1

    def finish(self, *, complete, dropped, error=None):
        self.write({"type": "end", "complete": complete, "dropped_messages": dropped,
                    "counts": self.counts, "error": error,
                    "transport_loss": "not_measured"})
        self.stream.flush()


def capture_live(output: Path, metadata, duration_s, *, group="default", capacity=256,
                 node_factory=None, bindings=None, monotonic=time.monotonic, marker_reader=None,
                 pull_feedback=False):
    """Read-only capture; opt-in keep-last rings retain one sample per topic.

    The callback API is unchanged. Pull retention is depth one and may overwrite
    intermediate frames; the queue drop count does not measure these overwrites.
    """
    metadata_checked(metadata)
    if type(duration_s) not in (int, float) or not math.isfinite(duration_s) or not 0 < duration_s <= 86400:
        raise ValueError("duration must be in (0,86400] seconds")
    if type(capacity) is not int or not 1 <= capacity <= 256:
        raise ValueError("queue capacity must be in [1,256] (maximum 16 MiB raw payload)")
    if not isinstance(group, str) or not group.strip():
        raise ValueError("explicit nonempty Aorta group required")
    bindings = load_bindings() if bindings is None else bindings
    import aorta
    node_factory = aorta.Node if node_factory is None else node_factory
    pending = queue.Queue(capacity)
    lock = threading.Lock()
    accepting, dropped = True, 0
    node = None
    pulls = {}
    failure = None
    def receive(kind, payload, context):
        nonlocal dropped
        with lock:
            if not accepting:
                return
            if len(payload) > 65536:
                dropped += 1
                return
            try:
                pending.put_nowait((kind, bytes(payload), time.monotonic_ns(),
                                    getattr(context, "sequence", 0),
                                    getattr(context, "publisher_node", "")))
            except queue.Full:
                dropped += 1
    with output.open("x", encoding="utf-8") as stream:
        writer = TraceWriter(stream, {**metadata, "requested_group": group,
            "feedback_retention": "keep-last-depth-1-all-topics" if pull_feedback else "callback-queue",
            "pull_overwrites": "not_measured" if pull_feedback else "not_applicable",
            "capture_stamp_semantics": "host dequeue, not arrival" if pull_feedback else "host callback"},
            "observer_monotonic_ns")
        try:
            node = node_factory("brainco_hil_observer", group)
            from ._group import require_node_group
            require_node_group(node, group)
            for kind, topic in TOPICS.items():
                if pull_feedback:
                    pulls[kind] = node.create_subscriber_pull(topic, depth=1,
                                                              qos=aorta.QoS.sensor_data())
                    continue
                node.create_subscriber_typed(bytes, topic,
                    lambda payload, ctx, kind=kind: receive(kind, payload, ctx),
                    qos=aorta.QoS.sensor_data())
            deadline = monotonic() + duration_s
            while monotonic() < deadline:
                for kind, sub in pulls.items():
                    sample = sub.try_recv()
                    if sample is None:
                        continue
                    stamp = time.monotonic_ns()
                    if len(sample.payload) > 65536:
                        with lock:
                            dropped += 1
                        continue
                    writer.sample(kind, bytes(sample.payload), stamp, bindings,
                        native_body=kind == "body_state", publisher=sample.context.publisher_node,
                        transport_sequence=sample.context.sequence)
                label = marker_reader() if marker_reader else None
                if label is not None:
                    if not isinstance(label, str) or not 1 <= len(label) <= 512:
                        raise ValueError("marker must contain 1..512 characters")
                    writer.write({"type": "marker", "label": label,
                                  "capture_timestamp_ns": time.monotonic_ns()})
                try:
                    item = pending.get(timeout=min(0.001 if pulls else 0.05,
                                                   max(0, deadline - monotonic())))
                except queue.Empty:
                    continue
                kind, payload, stamp, sequence, publisher = item
                writer.sample(kind, payload, stamp, bindings, publisher=publisher,
                              transport_sequence=sequence)
        except BaseException as error:
            failure = error
        finally:
            with lock:
                accepting = False
            try:
                if node is not None:
                    node.close()
                if failure is None:
                    while not pending.empty():
                        kind, payload, stamp, sequence, publisher = pending.get_nowait()
                        writer.sample(kind, payload, stamp, bindings, publisher=publisher,
                                      transport_sequence=sequence)
            except BaseException as error:
                failure = failure or error
            writer.finish(complete=failure is None, dropped=dropped,
                          error=None if failure is None else str(failure))
        if failure is not None:
            raise failure
        return writer.counts


def export_mcap(source: Path, output: Path, metadata, topic_map, schema_hashes, *, bindings=None):
    """Offline export. Exact topic and BFBS hashes are mandatory, no suffix guessing."""
    metadata_checked(metadata)
    if (not topic_map or any(kind not in MODULES for kind in topic_map.values())
            or len(set(topic_map.values())) != len(topic_map)):
        raise ValueError("topic_map must map distinct exact topics to distinct known kinds")
    for kind in topic_map.values():
        if not re.fullmatch(r"[0-9a-f]{64}", schema_hashes.get(kind, "")):
            raise ValueError(f"expected BFBS sha256 required for {kind}")
    for kind in topic_map.values():
        if kind.endswith("_hand_touch") and schema_hashes[kind] != metadata["schema_sha256"]:
            raise ValueError("metadata HandTouch schema_sha256 differs from requested BFBS")
        if schema_hashes[kind] != binding_schema_hash(kind):
            raise ValueError(f"recorded BFBS differs from loaded binding for {kind}; use its matching SDK")
    bindings = load_bindings() if bindings is None else bindings
    from mcap.reader import NonSeekingReader
    with source.open("rb") as incoming, output.open("x", encoding="utf-8") as stream:
        digest = hashlib.sha256()
        for chunk in iter(lambda: incoming.read(1024 * 1024), b""):
            digest.update(chunk)
        incoming.seek(0)
        writer = TraceWriter(stream, {**metadata, "source_mcap_sha256": digest.hexdigest()}, "mcap_log_time_ns")
        try:
            # Seeking readers can skip DataEnd CRC and the final magic. Read the
            # entire container, including unselected channels, without sorting.
            reader = NonSeekingReader(incoming, validate_crcs=True)
            # Storage order exposes reordering instead of silently sorting it away.
            for schema, channel, message in reader.iter_messages(log_time_order=False):
                kind = topic_map.get(channel.topic)
                if kind is None:
                    continue
                if (schema is None or schema.name != MODULES[kind][0] or schema.encoding != "flatbuffer"
                        or channel.message_encoding != "flatbuffer"
                        or hashlib.sha256(schema.data).hexdigest() != schema_hashes[kind]):
                    raise ValueError(f"schema/encoding mismatch for {channel.topic}")
                writer.sample(kind, message.data, message.log_time, bindings,
                              mcap_sequence=message.sequence, topic=channel.topic)
            if set(writer.counts) != set(topic_map.values()):
                raise ValueError("requested MCAP topics contain no messages")
        except BaseException as error:
            writer.finish(complete=False, dropped=0, error=str(error))
            raise
        writer.finish(complete=True, dropped=0)
        return writer.counts
