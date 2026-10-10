"""Instrument the current SDK, never a copied codec; no robot control requests."""
import argparse
import csv
import json
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    parser.add_argument("session", type=int)
    parser.add_argument("duration", type=float)
    args = parser.parse_args()
    if not 0 < args.session < 2**64 or not 1 <= args.duration <= 120:
        parser.error("invalid bounded benchmark")
    import locomotion_aorta.transport as sdk
    from locomotion_aorta.types import LowCmd, MotorCommand
    from aorta.publisher import Publisher

    prefix = f"/bench/sdk_latency/{args.session}"
    sdk.LOWSTATE_TOPIC = prefix + "/lowstate"
    sdk.EXTERNAL_COMMAND_TOPIC = prefix + "/command"
    sdk.CONTROL_REQUEST_TOPIC = prefix + "/unused_request"
    sdk.CONTROL_ACK_TOPIC = prefix + "/unused_ack"
    sdk.CONTROL_STATUS_TOPIC = prefix + "/unused_status"
    decode, fill, publish_bytes = sdk._decode_humanoid_lowstate, sdk._fill_low_cmd, Publisher.publish_bytes
    timing = threading.local()
    rows, errors = [], []
    lock = threading.Lock()

    def timed_decode(view):
        timing.decode_ns = time.monotonic_ns()
        try:
            result = decode(view)
        except Exception as error:
            # Aorta logs callback exceptions; make them fail the benchmark too,
            # rather than misreporting an SDK decoding regression as packet loss.
            with lock:
                errors.append(repr(error))
            raise
        timing.decoded_ns = time.monotonic_ns()
        return result

    def timed_fill(*argv):
        timing.fill_ns = time.monotonic_ns()
        result = fill(*argv)
        timing.filled_ns = time.monotonic_ns()
        return result

    def timed_publish(publisher, *argv, **kwargs):
        # This is the Python Aorta byte-publish entry, NOT a CAN/TCP send stamp.
        timing.bytes_ns = time.monotonic_ns()
        return publish_bytes(publisher, *argv, **kwargs)

    sdk._decode_humanoid_lowstate = timed_decode
    sdk._fill_low_cmd = timed_fill
    Publisher.publish_bytes = timed_publish
    transport = sdk.create_aorta_transport(node_name="sdk_ci_python", group="default")

    def callback(state):
        with lock:
            try:
                if len(state.motor_states) != 44 or not 0 < state.state_id <= 65000:
                    raise ValueError("invalid state slots/sequence")
                command = LowCmd(
                    cmd_id=state.state_id, external_version=1,
                    external_session_id=args.session, external_sequence=state.state_id,
                    motor_cmds=tuple(MotorCommand(1, m.q, m.dq, m.tau_est, 1., .5)
                                    for m in state.motor_states[:42]))
                built_ns = time.monotonic_ns()
                transport.publish(sdk.EXTERNAL_COMMAND_TOPIC, command)
                returned_ns = time.monotonic_ns()
                rows.append((args.session, state.state_id, timing.decode_ns, timing.decoded_ns,
                             built_ns, timing.fill_ns, timing.filled_ns, timing.bytes_ns, returned_ns))
                if len(rows) > 65000:
                    raise ValueError("capture limit exceeded")
            except Exception as error:
                errors.append(repr(error))

    transport.subscribe(sdk.LOWSTATE_TOPIC, callback)
    try:
        if not transport.publishers[sdk.EXTERNAL_COMMAND_TOPIC].wait_for_matching(15000):
            raise RuntimeError("mock command subscriber did not match")
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
                                 "fill_ns", "filled_ns", "bytes_ns", "returned_ns"))
                writer.writerows(rows)
            (args.directory / "client.json").write_text(json.dumps({
                "sdk_source": sdk.__file__, "errors": errors, "callbacks": len(rows)}, indent=2))
    return 0 if rows and not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
