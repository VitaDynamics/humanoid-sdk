"""Explicit, loopback-only production-topic mock; never started by SDK imports."""

import dataclasses
import importlib
import json
import math
from pathlib import Path
import struct
import sys
import time


def validate_auxiliary(spec):
    """Require explicit rates/Info fixtures; no invented real-device frequency."""
    if not isinstance(spec, dict) or set(spec) - {"left", "right", "joy"}:
        raise ValueError("auxiliary fixture supports only left/right/joy")
    for side in ("left", "right"):
        if side not in spec:
            continue
        value = spec[side]
        if not isinstance(value, dict) or value.get("kind") not in ("brainco", "gripper"):
            raise ValueError("EE kind must be brainco or gripper")
        for key in ("state_hz", "info_hz", *(["touch_hz"] if value["kind"] == "brainco" else [])):
            hz = value.get(key)
            if isinstance(hz, bool) or not isinstance(hz, (int, float)) or not math.isfinite(hz) or not 0 < hz <= 2000:
                raise ValueError("explicit finite EE rate required: " + key)
        if not isinstance(value.get("info"), dict):
            raise ValueError("explicit public Info fields required")
        if type(value.get("synthetic_output", False)) is not bool:
            raise ValueError("synthetic_output must be bool")
        if "sample_q" in value and (isinstance(value["sample_q"], bool) or not isinstance(value["sample_q"], (int, float)) or not math.isfinite(value["sample_q"])):
            raise ValueError("synthetic sample_q must be finite")
    if not isinstance(spec.get("joy", []), list):
        raise ValueError("Joy script must be a frame list")
    previous = -1
    for frame in spec.get("joy", []):
        if not isinstance(frame, dict) or set(frame) != {"at_ms", "axes", "buttons"}:
            raise ValueError("Joy frame requires at_ms/axes/buttons")
        stamp = frame["at_ms"]
        if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or not 0 <= stamp <= 600000 or stamp <= previous:
            raise ValueError("Joy script times must strictly increase")
        previous = stamp
        if not isinstance(frame["axes"], list) or not isinstance(frame["buttons"], list):
            raise ValueError("Joy axes/buttons must be lists")
        if len(frame["axes"]) > 64 or len(frame["buttons"]) > 128:
            raise ValueError("Joy vectors exceed fixture bounds")
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not -1 <= v <= 1 for v in frame["axes"]):
            raise ValueError("invalid Joy axis")
        if any(type(v) is not int or v not in (0, 1) for v in frame["buttons"]):
            raise ValueError("Joy buttons must be integer levels0/1")
    if len(spec.get("joy", [])) > 100000:
        raise ValueError("Joy fixture too large")
    return spec


def _camel(name):
    first, *rest = name.split("_")
    return first + "".join(part.capitalize() for part in rest)


def generated_object(dto):
    """Pack the public generated object API, never a parallel wire layout."""
    module = importlib.import_module("lowlevel." + type(dto).__name__)
    obj = getattr(module, type(dto).__name__ + "T")()
    for f in dataclasses.fields(dto):
        name, value = _camel(f.name), getattr(dto, f.name)
        if not hasattr(obj, name):
            raise ValueError("public fixture/installed generated fields differ: " + name)
        if dataclasses.is_dataclass(value):
            value = generated_object(value)
        elif isinstance(value, tuple):
            value = [generated_object(v) if dataclasses.is_dataclass(v) else v for v in value]
        setattr(obj, name, value)
    return obj


def pack_object(obj, sequence, wall_ns):
    import flatbuffers
    from aorta.sys.AortaHeader import AortaHeaderT
    header = AortaHeaderT()
    header.sequence = sequence
    header.publishStampNs = wall_ns
    header.publisherNode = "production_topic_mock_auxiliary"
    obj.aortaHeader = header
    builder = flatbuffers.Builder(2048)
    builder.Finish(obj.Pack(builder))
    return bytes(builder.Output())


def run_auxiliary(output, fixture, seconds):
    """Optional independent process: main-thread pull cmd, fixture-paced EE/Joy."""
    import aorta
    import hashlib
    import os
    import signal
    from locomotion_aorta import _hand_codec, _gripper_codec
    from locomotion_aorta.hand import validate_info as validate_hand_info
    from locomotion_aorta.gripper import validate_info as validate_gripper_info, validate_target
    from locomotion_aorta.hand_types import HandInfo, HandState, HandJointState, HandTouch, FingerTouchMeta
    from locomotion_aorta.gripper_types import GripperInfo, GripperState, GripperSample, GripperHostTarget

    output, fixture = Path(output), Path(fixture)
    spec = validate_auxiliary(json.loads(fixture.read_text()))
    group = os.environ["AORTA_GROUP"]
    node = aorta.Node("production_topic_mock_auxiliary", group)
    if node.group() != group:
        node.close()
        raise RuntimeError("effective auxiliary group differs")
    streams, endpoints = [], []
    counts = dict(command=0, publish=0, joy=0)
    dropped = 0
    start = time.monotonic()
    stopped = False
    def stop(_number, _frame):
        nonlocal stopped
        stopped = True
    old_handlers = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}

    def publisher(stem, topic):
        meta = importlib.import_module(stem + "_schema_meta")
        # Match the integration checkout's immutable compatibility manifest.
        manifest = json.loads((Path(__file__).resolve().parents[2] / "compatibility.json").read_text())
        actual = hashlib.sha256(meta.SCHEMA_BFBS).hexdigest()
        if actual != meta.SCHEMA_HASH_HEX or actual != manifest["schema_sha256"].get(stem):
            raise ValueError("auxiliary BFBS pin mismatch: " + stem)
        qos = aorta.QoS.state_update() if stem.endswith("info") or stem == "gripper_state" else aorta.QoS.realtime_control()
        pub = node.create_publisher_typed(meta, topic, qos=qos)
        return pub

    try:
        for side in ("left", "right"):
            if side not in spec:
                continue
            cfg = spec[side]
            brainco = cfg["kind"] == "brainco"
            kind = "hand" if brainco else "gripper"
            prefix = f"/sdk/foot_humanoid/{kind}/{side}"
            fields = dict(cfg["info"])
            fields.update(side=1 if side == "left" else 2, ts_info_us=time.time_ns() // 1000,
                          ts_state_pub_us=time.time_ns() // 1000)
            info = HandInfo(**fields) if brainco else GripperInfo(**fields)
            (validate_hand_info if brainco else validate_gripper_info)(info)
            if not brainco and "sample_q" in cfg and not info.opening_min_m <= cfg["sample_q"] <= info.opening_max_m:
                raise ValueError("synthetic gripper sample outside fixture Info")
            if brainco and (info.hand_type != 1 or info.dof != 6):
                raise ValueError("brainco fixture requires BRAINCO_REVO2/six joints")
            if brainco and info.touch_available and (info.touch_fingers < 1 or info.touch_words_per_finger < 1 or info.touch_fingers * info.touch_words_per_finger > 4096):
                raise ValueError("invalid touch geometry")
            cmd_module = importlib.import_module("lowlevel." + ("HandCmd" if brainco else "GripperCmd"))
            sub = node.create_subscriber_pull(prefix + "/cmd", depth=256, qos=aorta.QoS.realtime_control())
            endpoint = {"side": side, "cfg": cfg, "kind": kind, "info": info, "sub": sub,
                        "decoder": getattr(cmd_module, "HandCmd" if brainco else "GripperCmd").GetRootAs,
                        "decode": _hand_codec.decode_cmd if brainco else _gripper_codec.decode_cmd,
                        "command": None, "last_received_us": 0, "last_accepted_us": 0,
                        "last_output_us": 0, "pending": False, "rejected": 0, "last_by_session": {}, "state_id": 0,
                        "feedback_q": tuple(info.q_min) if brainco else None}
            endpoints.append(endpoint)
            for role in ("info", "state", *(["touch"] if brainco and info.touch_available and info.touch_enabled else [])):
                streams.append({"endpoint": endpoint, "role": role, "next": start, "period": 1 / cfg[role + "_hz"],
                                "pub": publisher(kind + "_" + role, prefix + "/" + role), "seq": 0})
        joy = None
        if spec.get("joy"):
            joy_meta = importlib.import_module("joy_schema_meta")
            joy_module = importlib.import_module(joy_meta.ROOT_TYPE_NAME)
            joy_class = getattr(joy_module, joy_meta.ROOT_TYPE_NAME.rsplit(".", 1)[1] + "T")
            joy = (publisher("joy", "locomotion/joy"), joy_class)
            match_deadline = time.monotonic() + 15
            matched = False
            while not stopped and time.monotonic() < match_deadline:
                if joy[0].wait_for_matching(100):
                    matched = True
                    break
            if not matched and not stopped:
                raise TimeoutError("no Joy subscriber matched; scripted press/release would be unobservable")
        start = time.monotonic()
        for stream in streams:
            stream["next"] = start
        joy_index = 0
        while not stopped and time.monotonic() - start < seconds:
            now = time.monotonic()
            for endpoint in endpoints:
                while True:
                    sample = endpoint["sub"].try_recv()
                    if sample is None:
                        break
                    received = time.time_ns() // 1000
                    try:
                        command = endpoint["decode"](endpoint["decoder"](sample.payload, 0))
                        if command.protocol_version != 1 or command.side != endpoint["info"].side or not 0 < command.session_id < 2**64 or not endpoint["last_by_session"].get(command.session_id, 0) < command.sequence < 2**64:
                            raise ValueError("invalid EE command identity/side/sequence")
                        if endpoint["kind"] == "hand":
                            info = endpoint["info"]
                            if command.hand_type != info.hand_type or len(command.joints) != info.dof:
                                raise ValueError("hand command differs from fixture Info")
                            for j, qmin, qmax, dqmax in zip(command.joints, info.q_min, info.q_max, info.dq_max):
                                if not all(math.isfinite(v) for v in (j.q, j.dq)) or not qmin <= j.q <= qmax or not 0 <= j.dq <= dqmax:
                                    raise ValueError("hand target outside fixture Info")
                        else:
                            if command.gripper_type != endpoint["info"].gripper_type:
                                raise ValueError("gripper type differs from fixture Info")
                            validate_target(endpoint["info"], command.q, command.dq, command.torque_cmd_nm)
                        if command.session_id not in endpoint["last_by_session"] and len(endpoint["last_by_session"]) >= 1024:
                            raise ValueError("synthetic EE session capacity exceeded")
                        endpoint["last_by_session"][command.session_id] = command.sequence
                        endpoint.update(command=command, last_received_us=received, last_accepted_us=time.time_ns() // 1000,
                                        pending=True, last_output_us=0)
                        counts["command"] += 1
                    except (ValueError, TypeError, IndexError, AttributeError, struct.error) as exc:
                        print(f"Rejected {endpoint['side']} EE command: {exc}", file=sys.stderr)
                        endpoint["rejected"] += 1
                        counts["command"] += 1
            for stream in streams:
                if now < stream["next"]:
                    continue
                endpoint, role = stream["endpoint"], stream["role"]
                info, command = endpoint["info"], endpoint["command"]
                wall = time.time_ns()
                us = wall // 1000
                stream["seq"] += 1
                if role == "info":
                    dto = dataclasses.replace(info, ts_info_us=us, ts_state_pub_us=us)
                elif role == "touch":
                    fingers = tuple(FingerTouchMeta(0, 0, us, stream["seq"] % 256, 0, 0) for _ in range(info.touch_fingers))
                    dto = HandTouch(stream["seq"], info.hand_type, info.side, 1, us, fingers,
                                    (0,) * (info.touch_fingers * info.touch_words_per_finger))
                else:
                    endpoint["state_id"] += 1
                    allowed = endpoint["cfg"].get("synthetic_output", False)
                    if endpoint["pending"] and allowed:
                        endpoint["last_output_us"] = us
                        if endpoint["kind"] == "hand":
                            endpoint["feedback_q"] = tuple(j.q for j in command.joints)
                        endpoint["pending"] = False
                    common = dict(state_id=endpoint["state_id"], side=info.side, ts_state_real_us=us,
                                  ts_state_pub_us=us, ts_ingested_us=us, ts_assembled_us=us,
                                  output_allowed=allowed, inhibit_reason=0 if allowed else 4,
                                  ts_cmd_received_us=endpoint["last_received_us"], ts_cmd_accepted_us=endpoint["last_accepted_us"],
                                  ts_cmd_first_output_us=endpoint["last_output_us"])
                    identity = dict(session_id=command.session_id, sequence=command.sequence, cmd_id=command.cmd_id) if command else dict(session_id=0, sequence=0, cmd_id=0)
                    if endpoint["kind"] == "hand":
                        joints = tuple(HandJointState(q=q, q_cmd=command.joints[i].q if command else q,
                                                     dq_cmd=command.joints[i].dq if command else 0.)
                                       for i, q in enumerate(endpoint["feedback_q"]))
                        dto = HandState(hand_type=info.hand_type, joints=joints, reject_count=endpoint["rejected"],
                                        last_cmd_session_id=identity["session_id"], last_cmd_sequence=identity["sequence"],
                                        last_cmd_id=identity["cmd_id"], **common)
                    else:
                        target = GripperHostTarget(command.q, info.hold_torque_nm if command.torque_cmd_nm is None else command.torque_cmd_nm) if command else None
                        # A virtual sample exists only when explicitly supplied by the
                        # synthetic fixture. Device ACK/feedback timestamps remain zero.
                        sample_q = command.q if command and allowed and "sample_q" in endpoint["cfg"] else endpoint["cfg"].get("sample_q")
                        sample = GripperSample(sample_q, 0., 0) if sample_q is not None else None
                        dto = GripperState(gripper_type=info.gripper_type, sample=sample, sample_valid=sample is not None,
                                           host_target=target, host_target_valid=target is not None,
                                           host_target_session_id=identity["session_id"], host_target_sequence=identity["sequence"],
                                           host_target_cmd_id=identity["cmd_id"], trace_session_id=identity["session_id"],
                                           trace_sequence=identity["sequence"], trace_cmd_id=identity["cmd_id"], **common)
                stream["pub"].publish_bytes(pack_object(generated_object(dto), stream["seq"], wall))
                counts["publish"] += 1
                stream["next"] += (int((now - stream["next"]) / stream["period"]) + 1) * stream["period"]
            while joy and joy_index < len(spec["joy"]) and (now - start) * 1000 >= spec["joy"][joy_index]["at_ms"]:
                frame = spec["joy"][joy_index]
                obj = joy[1]()
                obj.axes, obj.buttons = frame["axes"], frame["buttons"]
                wall = time.time_ns()
                joy[0].publish_bytes(pack_object(obj, joy_index, wall))
                counts["joy"] += 1
                joy_index += 1
            time.sleep(.0005)
    finally:
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        for endpoint in endpoints:
            dropped += endpoint["sub"].dropped_count()
        node.close()
        (output / "auxiliary.json").write_text(json.dumps({"synthetic": True, "counts": counts,
            "requested": spec, "pull_dropped_count": dropped,
            "limits": ["no CAN/device ACK/physical touch", "no set_touch RPC", "synthetic rates are not measured timing", "ring overflow cannot reconstruct unobserved commands"]}, indent=2) + "\n")
