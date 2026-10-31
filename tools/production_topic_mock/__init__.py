"""Loopback-only Joy support; EE authority belongs to the main Mock Protocol."""
import importlib
import json
import math
from pathlib import Path
import time


def validate_auxiliary(spec):
    """Joy levels only. Generic EE fixtures belong to the main process."""
    if not isinstance(spec, dict) or set(spec) - {"joy"}:
        raise ValueError("auxiliary supports only Joy; use --ee-fixture for main-owned EE")
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
    """Publish explicit Joy levels; no EE subscriber, owner or motion writer."""
    import aorta
    import hashlib
    import os
    import signal

    output, fixture = Path(output), Path(fixture)
    spec = validate_auxiliary(json.loads(fixture.read_text()))
    group = os.environ["AORTA_GROUP"]
    meta = importlib.import_module("joy_schema_meta")
    manifest = json.loads((Path(__file__).resolve().parents[2] / "compatibility.json").read_text())
    actual = hashlib.sha256(meta.SCHEMA_BFBS).hexdigest()
    if actual != meta.SCHEMA_HASH_HEX or actual != manifest["schema_sha256"].get("joy"):
        raise ValueError("auxiliary BFBS pin mismatch: joy")
    module = importlib.import_module(meta.ROOT_TYPE_NAME)
    message = getattr(module, meta.ROOT_TYPE_NAME.rsplit(".", 1)[1] + "T")
    node = aorta.Node("production_topic_mock_auxiliary", group)
    if node.group() != group:
        node.close()
        raise RuntimeError("effective auxiliary group differs")
    stopped = False
    def stop(_number, _frame):
        nonlocal stopped
        stopped = True
    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    count = 0
    try:
        pub = node.create_publisher_typed(meta, "locomotion/joy", qos=aorta.QoS.state_update())
        if spec.get("joy"):
            match_deadline = time.monotonic() + 15
            matched = False
            while not stopped and time.monotonic() < match_deadline:
                if pub.wait_for_matching(100):
                    matched = True
                    break
            if not matched and not stopped:
                raise TimeoutError("no Joy subscriber matched; scripted press/release would be unobservable")
        start = time.monotonic()
        while not stopped and time.monotonic() - start < seconds:
            now = time.monotonic()
            while count < len(spec.get("joy", [])) and (now-start)*1000 >= spec["joy"][count]["at_ms"]:
                frame = spec["joy"][count]
                obj = message()
                obj.axes, obj.buttons = frame["axes"], frame["buttons"]
                pub.publish_bytes(pack_object(obj, count, time.time_ns()))
                count += 1
            time.sleep(.0005)
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        node.close()
        (output / "auxiliary.json").write_text(json.dumps({"synthetic": True,
            "counts": {"joy": count}, "requested": spec,
            "limits": ["Joy fixtures only; no EE permission or command execution", "no device evidence"]}, indent=2) + "\n")
