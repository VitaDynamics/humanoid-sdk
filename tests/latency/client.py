"""Instrument the current SDK, never a copied codec; no robot control requests."""
import argparse
import csv
import json
from pathlib import Path
import sys
import threading
import time

def require_installed_codec(sdk, native):
    """Reject source shadowing: the measured wheel must contain the codec."""
    prefix = Path(sys.prefix).resolve()
    for module in (sdk, native):
        if not Path(module.__file__).resolve().is_relative_to(prefix):
            raise RuntimeError(f"benchmark requires installed SDK wheel: {module.__file__}")


def encode_with_timing(encode, timing, *args, **kwargs):
    # Whole native encoder boundary: owned-input conversion through returned bytes.
    timing.fill_ns = time.monotonic_ns()
    timing.fill_realtime_ns = time.time_ns()
    result = encode(*args, **kwargs)
    timing.filled_ns = time.monotonic_ns()
    timing.filled_realtime_ns = time.time_ns()
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    parser.add_argument("session", type=int)
    parser.add_argument("duration", type=float)
    args = parser.parse_args()
    if not 0 < args.session < 2**64 or not 1 <= args.duration <= 120:
        parser.error("invalid bounded benchmark")
    import locomotion_aorta.transport as sdk
    from locomotion_aorta import BodyCmd, BodyJointCmd, BodyCmdAction, ServiceStatus
    from aorta.publisher import Publisher
    from locomotion_aorta import _body_cmd_native as native
    require_installed_codec(sdk, native)

    prefix = f"/bench/sdk_latency/{args.session}"
    sdk.BODY_STATE_TOPIC = prefix + "/lowstate"
    sdk.BODY_CMD_TOPIC = prefix + "/command"
    sdk.GET_ROBOT_INFO_SERVICE = prefix + "/get_robot_info"
    sdk.CONTROL_REQUEST_TOPIC = prefix + "/unused_request"
    sdk.CONTROL_ACK_TOPIC = prefix + "/unused_ack"
    sdk.CONTROL_STATUS_TOPIC = prefix + "/unused_status"
    decode, encode, publish_bytes = sdk.decode_body_state, native.encode, Publisher.publish_bytes
    timing = threading.local()
    rows, errors = [], []
    lock = threading.Lock()

    def timed_decode(view):
        timing.decode_ns = time.monotonic_ns()
        timing.decode_realtime_ns = time.time_ns()
        try:
            result = decode(view)
        except Exception as error:
            # Aorta logs callback exceptions; make them fail the benchmark too,
            # rather than misreporting an SDK decoding regression as packet loss.
            with lock:
                errors.append(repr(error))
            raise
        timing.decoded_ns = time.monotonic_ns()
        timing.decoded_realtime_ns = time.time_ns()
        return result

    def timed_encode(*argv, **kwargs):
        return encode_with_timing(encode, timing, *argv, **kwargs)

    def timed_publish(publisher, *argv, **kwargs):
        # This is the Python Aorta byte-publish entry, NOT a CAN/TCP send stamp.
        timing.bytes_ns = time.monotonic_ns()
        timing.bytes_realtime_ns = time.time_ns()
        return publish_bytes(publisher, *argv, **kwargs)

    sdk.decode_body_state = timed_decode
    native.encode = timed_encode
    Publisher.publish_bytes = timed_publish
    transport = sdk.create_aorta_transport(node_name="sdk_ci_python", group="default")

    def callback(state):
        with lock:
            try:
                if state.protocol_version != 2 or len(state.joints) != 31 or not 0 < state.state_id <= 65000:
                    raise ValueError("invalid state slots/sequence")
                command = BodyCmd(
                    protocol_version=2, action=BodyCmdAction.EXTERNAL,
                    session_id=args.session, sequence=state.state_id,
                    input_state_id=state.state_id, input_state_ts_real_us=state.ts_state_real_us,
                    joints=tuple(BodyJointCmd(m.q, m.dq, m.tau_est, 1., .5) if cap.populated else BodyJointCmd()
                                    for m, cap in zip(state.joints, body.joints)))
                built_ns = time.monotonic_ns()
                built_realtime_ns = time.time_ns()
                transport.publish(sdk.BODY_CMD_TOPIC, command)
                returned_ns = time.monotonic_ns()
                returned_realtime_ns = time.time_ns()
                rows.append((args.session, state.state_id, timing.decode_ns, timing.decoded_ns,
                             built_ns, timing.fill_ns, timing.filled_ns, timing.bytes_ns, returned_ns,
                             timing.decode_realtime_ns, timing.decoded_realtime_ns, built_realtime_ns,
                             timing.fill_realtime_ns, timing.filled_realtime_ns,
                             timing.bytes_realtime_ns, returned_realtime_ns))
                if len(rows) > 65000:
                    raise ValueError("capture limit exceeded")
            except Exception as error:
                errors.append(repr(error))

    transport.subscribe(sdk.BODY_STATE_TOPIC, callback)
    try:
        if not transport.publishers[sdk.BODY_CMD_TOPIC].wait_for_matching(15000):
            raise RuntimeError("mock command subscriber did not match")
        info = transport.get_robot_info(timeout=15.0)
        body = None if info.info is None else info.info.body
        if (info.status != ServiceStatus.SUCCESS or body is None or body.slot_count != 31
                or len(body.joints) != 31 or len(body.motor_identities) != 31
                or len(info.info.end_effectors) != 2 or body.joints[14].populated):
            raise ValueError("invalid actual Mock RobotInfo reply")
        (args.directory / "client_info.json").write_text(json.dumps({
            "status": info.status, "slot_count": body.slot_count,
            "joint_names": body.joint_names, "populated": [j.populated for j in body.joints],
            "motor_identity_count": len(body.motor_identities), "end_effector_count": len(info.info.end_effectors)}))
        (args.directory / "client.ready").write_text("ready\n")
        deadline = time.monotonic() + args.duration + 30
        while not (args.directory / "stop").exists():
            if time.monotonic() > deadline:
                raise TimeoutError("client watchdog")
            with lock:
                if errors:
                    raise RuntimeError(errors[0])
            time.sleep(.01)
    finally:
        transport.close()
        with lock:
            with (args.directory / "client.csv").open("w") as out:
                writer = csv.writer(out)
                writer.writerow(("session", "sequence", "decode_ns", "decoded_ns", "built_ns",
                                 "fill_ns", "filled_ns", "bytes_ns", "returned_ns",
                                 "decode_realtime_ns", "decoded_realtime_ns", "built_realtime_ns",
                                 "fill_realtime_ns", "filled_realtime_ns", "bytes_realtime_ns",
                                 "returned_realtime_ns"))
                writer.writerows(rows)
            (args.directory / "client.json").write_text(json.dumps({
                "sdk_source": sdk.__file__, "native_codec": native.__file__,
                "encode_boundary": "native.encode including marshal and returned bytes",
                "errors": errors, "callbacks": len(rows)}, indent=2))
    return 0 if rows and not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
