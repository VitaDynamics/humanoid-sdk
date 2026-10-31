"""Instrument the current SDK with an isolated synthetic control owner."""
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
    import locomotion_aorta.controller as controller_module
    from locomotion_aorta import BodyCmd, BodyJointCmd, ExternalController, ServiceStatus
    from aorta.publisher import Publisher
    from locomotion_aorta import _body_cmd_native as native
    require_installed_codec(sdk, native)

    prefix = f"/bench/sdk_latency/{args.session}"
    sdk.BODY_STATE_TOPIC = prefix + "/lowstate"
    sdk.BODY_CMD_TOPIC = prefix + "/command"
    sdk.GET_ROBOT_INFO_SERVICE = prefix + "/get_robot_info"
    sdk.CONTROL_REQUEST_TOPIC = prefix + "/control/request"
    sdk.CONTROL_ACK_TOPIC = prefix + "/control/ack"
    sdk.CONTROL_STATUS_TOPIC = prefix + "/control/status"
    controller_module.BODY_STATE_TOPIC = sdk.BODY_STATE_TOPIC
    controller_module.CONTROL_ACK_TOPIC = sdk.CONTROL_ACK_TOPIC
    controller_module.CONTROL_STATUS_TOPIC = sdk.CONTROL_STATUS_TOPIC
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
    controller = ExternalController(_transport=transport)
    entered = False

    def callback(state):
        with lock:
            try:
                if state.protocol_version != 3 or len(state.joints) != 31 or not 0 < state.state_id <= 65000:
                    raise ValueError("invalid state slots/sequence")
                command = BodyCmd(
                    runtime_id=state.runtime_id,
                    input_state_id=state.state_id, input_state_ts_real_us=state.ts_state_real_us,
                    joints=tuple(BodyJointCmd(m.q, m.dq, m.tau_est, 1., .5) if i in target_slots else BodyJointCmd()
                                    for i, m in enumerate(state.joints)))
                built_ns = time.monotonic_ns()
                built_realtime_ns = time.time_ns()
                sent = controller.publish(command, target_names=target_names, required_fields=0x1f)
                returned_ns = time.monotonic_ns()
                returned_realtime_ns = time.time_ns()
                rows.append((sent.session_id, sent.sequence, state.state_id, timing.decode_ns, timing.decoded_ns,
                             built_ns, timing.fill_ns, timing.filled_ns, timing.bytes_ns, returned_ns,
                             timing.decode_realtime_ns, timing.decoded_realtime_ns, built_realtime_ns,
                             timing.fill_realtime_ns, timing.filled_realtime_ns,
                             timing.bytes_realtime_ns, returned_realtime_ns))
                if len(rows) > 65000:
                    raise ValueError("capture limit exceeded")
            except Exception as error:
                errors.append(repr(error))

    try:
        info = controller.get_robot_info(timeout=15.0)
        body = None if info.info is None else info.info.body
        if (info.status != ServiceStatus.SUCCESS or body is None or body.slot_count != 31
                or len(body.joints) != 31 or len(body.motor_identities) != 31
                or len(info.info.end_effectors) != 2
                or tuple(body.joint_names) != tuple(f'mock_joint_{i}' for i in range(31))):
            raise ValueError("invalid actual Mock RobotInfo reply")
        # This synthetic benchmark fixture explicitly targets all slots except 14.
        target_slots = tuple(i for i in range(31) if i != 14)
        if any(body.joints[i].is_mock for i in target_slots):
            raise ValueError('synthetic benchmark target is unavailable')
        if any(body.joints[i].supported_fields & 0x19 != 0x19
               or not body.joints[i].valid_limits & 1 for i in target_slots):
            raise ValueError('synthetic benchmark target capability is unavailable')
        target_names = tuple(body.joint_names[i] for i in target_slots)
        running = controller.enter_external(timeout=15.0)
        entered = True
        if not transport.publishers[sdk.BODY_CMD_TOPIC].wait_for_matching(15000):
            raise RuntimeError("mock command subscriber did not match")
        transport.subscribe(sdk.BODY_STATE_TOPIC, callback)
        (args.directory / "client_info.json").write_text(json.dumps({
            "status": info.status, "slot_count": body.slot_count,
            "joint_names": body.joint_names, "is_mock": [j.is_mock for j in body.joints],
            "supported_fields": [j.supported_fields for j in body.joints],
            "motor_identity_count": len(body.motor_identities), "end_effector_count": len(info.info.end_effectors),
            "runtime_id": info.runtime_id, "client_id": running.client_id,
            "control_id": running.control_id, "generation": running.generation}))
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
        original_error = sys.exc_info()[1]
        cleanup_error = None
        try:
            if entered:
                controller.exit_external(timeout=5.0)
        except Exception as error:
            cleanup_error = error
            errors.append('exit_external: ' + repr(error))
        try:
            controller.close()
        except Exception as error:
            cleanup_error = cleanup_error or error
            errors.append('controller.close: ' + repr(error))
        with lock:
            with (args.directory / "client.csv").open("w") as out:
                writer = csv.writer(out)
                writer.writerow(("session", "sequence", "input_state_id", "decode_ns", "decoded_ns", "built_ns",
                                 "fill_ns", "filled_ns", "bytes_ns", "returned_ns",
                                 "decode_realtime_ns", "decoded_realtime_ns", "built_realtime_ns",
                                 "fill_realtime_ns", "filled_realtime_ns", "bytes_realtime_ns",
                                 "returned_realtime_ns"))
                writer.writerows(rows)
            (args.directory / "client.json").write_text(json.dumps({
                "sdk_source": sdk.__file__, "native_codec": native.__file__,
                "encode_boundary": "native.encode including marshal and returned bytes",
                "errors": errors, "callbacks": len(rows)}, indent=2))
        if cleanup_error is not None and original_error is None:
            raise cleanup_error
    return 0 if rows and not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
