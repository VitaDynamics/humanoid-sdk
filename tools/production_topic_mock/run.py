"""Explicit isolated runner. No robot connection, SDK import or child on --help."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import signal
import socket
import subprocess
import sys
import time

try:
    from .report import write_report
    from . import validate_auxiliary, run_auxiliary, auxiliary_capture_budget
    from .group_contract import new_group, validate_group
except ImportError:
    from report import write_report
    from __init__ import validate_auxiliary, run_auxiliary, auxiliary_capture_budget
    from group_contract import new_group, validate_group


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def session_config(port, connect_port=None):
    connections = connect_port if isinstance(connect_port, list) else ([] if connect_port is None else [connect_port])
    for index, value in enumerate((port, *connections)):
        if value is not None and (isinstance(value, bool) or not isinstance(value, int) or
                                  not (1024 <= value <= 65535 or (index == 0 and value == 0))):
            raise ValueError("loopback port must be in 1024..65535; only listen may use zero")
    config = {"mode": "peer", "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
              "transport": {"shared_memory": {"enabled": False}, "unicast": {"compression": {"enabled": False}}},
              "listen": {"endpoints": [f"tcp/127.0.0.1:{port}"]},
              "connect": {"endpoints": [f"tcp/127.0.0.1:{value}" for value in connections]}}
    return json.dumps(config, separators=(",", ":"))


def child_environment(parent, group, uri):
    validate_group(group)
    if parent.get("SDK_GROUP") and parent["SDK_GROUP"] != group:
        raise ValueError("SDK_GROUP conflicts with selected mock group")
    # Mapping is confined to the two isolated child environments, not this process.
    env = {k: v for k, v in parent.items() if not k.startswith(("AORTA_", "ZENOH_"))}
    env.update(SDK_GROUP=group, AORTA_GROUP=group, AORTA_LOG_TOPIC_ENABLED="false",
               ZENOH_SESSION_CONFIG_URI=str(Path(uri).resolve()))
    return env


def verify_build(build_dir):
    build_dir = Path(build_dir)
    manifest = json.loads((build_dir / "build.json").read_text())
    if manifest.get("mode") != "production-topics" or manifest.get("target_arch") != platform.machine():
        raise ValueError("build is not an architecture-matching production-topic mock")
    if digest(build_dir / "production-topic-mock") != manifest["binary_sha256"]:
        raise ValueError("mock binary checksum mismatch")
    if digest(manifest["core_path"]) != manifest["core_sha256"]:
        raise ValueError("Aorta core checksum mismatch")
    if (build_dir / f"libaorta_core.so.{manifest['aorta_abi']}").resolve() != Path(manifest["core_path"]).resolve():
        raise ValueError("runtime Aorta core link changed")
    for relative, expected in manifest["generated_sha256"].items():
        path = Path(relative)
        if path.is_absolute() or ".." in path.parts or path.parts[0] != "generated":
            raise ValueError("unsafe generated manifest path")
        if digest(build_dir / path) != expected:
            raise ValueError("generated schema/header checksum mismatch: " + relative)
    required = {f"generated/{key}.bfbs" for key in ("body_state", "body_cmd", "get_robot_info", "control_request", "control_ack", "control_status")}
    if not required <= manifest["generated_sha256"].keys():
        raise ValueError("incomplete generated schema manifest")
    return manifest


def port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def terminate(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def validate_options(args):
    if args.mode != "production-topics":
        raise ValueError("explicit --mode production-topics is required")
    for name in ("seconds", "state_hz", "apply_hz"):
        value = getattr(args, name)
        maximum = 590 if name == "seconds" else 2000
        if not math.isfinite(value) or not 1 <= value <= maximum:
            raise ValueError("invalid bounded " + name)
    if (args.seconds + (10 if args.client_command else 0)) * args.state_hz + 1 > 1000000:
        raise ValueError("capture exceeds fixed row capacity")
    for name in ("command_timeout_ms", "prepare_timeout_ms"):
        value = getattr(args, name)
        if not 1 <= value <= 60000:
            raise ValueError("invalid " + name)
    if args.client_command and not args.allow_client:
        raise ValueError("client execution requires explicit --allow-client")
    if args.client_command and Path(args.client_command[0]).name in ("sh", "bash", "dash", "start_onboard_vita.sh"):
        raise ValueError("use a reviewed direct Python client; shells can overwrite the isolated profile")
    if getattr(args, "self_test_client", False):
        if args.client_command or not args.allow_client:
            raise ValueError("self-test client requires --allow-client and no custom client command")


def validate_imu(quaternions):
    if not isinstance(quaternions, list) or len(quaternions) != 2:
        raise ValueError("initial IMU requires HIP and TRUNK WXYZ quaternions")
    for q in quaternions:
        if not isinstance(q, list) or len(q) != 4 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in q):
            raise ValueError("IMU quaternion requires four finite numbers")
        if abs(sum(v * v for v in q) - 1) > .001:
            raise ValueError("IMU quaternion must be normalized; no implicit correction")
    return quaternions


def validate_worker_environment(output):
    output = Path(output).resolve()
    validate_group(os.environ.get("AORTA_GROUP"))
    if os.environ.get("SDK_GROUP") != os.environ["AORTA_GROUP"] or os.environ.get("AORTA_LOG_TOPIC_ENABLED") != "false":
        raise ValueError("invalid auxiliary group mapping")
    if os.environ.get("ZENOH_SESSION_CONFIG_URI") != str(output / "auxiliary-session.json"):
        raise ValueError("auxiliary session URI differs from assigned profile")
    allowed = {"AORTA_GROUP", "AORTA_LOG_TOPIC_ENABLED", "ZENOH_SESSION_CONFIG_URI"}
    if any(k.startswith(("AORTA_", "ZENOH_")) and k not in allowed for k in os.environ):
        raise ValueError("unexpected auxiliary transport override")
    raw = (output / "auxiliary-session.json").read_text()
    c = json.loads(raw)
    endpoint = c["listen"]["endpoints"][0]
    if (not endpoint.startswith("tcp/127.0.0.1:") or
            int(endpoint.rsplit(":", 1)[1]) == 0 or
            raw != session_config(int(endpoint.rsplit(":", 1)[1]))):
        raise ValueError("auxiliary profile is not canonical loopback-only")
    if (output / "auxiliary.json").exists():
        raise ValueError("refusing to overwrite auxiliary capture")


def validate_client_environment(output):
    """Validate the runner-assigned, private client group and peer profile."""
    output = Path(output).resolve()
    group = validate_group(os.environ.get("AORTA_GROUP"))
    if os.environ.get("SDK_GROUP") != group or os.environ.get("ZENOH_SESSION_CONFIG_URI") != str(output / "client-session.json"):
        raise ValueError("self-test client must use its assigned isolated group/profile")
    request = json.loads((output / "request.json").read_text())
    if request["group"] != group:
        raise ValueError("self-test request group differs")
    profile = (output / "client-session.json").read_text()
    ports = [json.loads((output / name).read_text())["listen"]["endpoints"][0].rsplit(":", 1)[1]
             for name in ("mock-session.json", "auxiliary-session.json") if (output / name).exists()]
    if profile != session_config(0, [int(p) for p in ports]):
        raise ValueError("self-test peer profile is not assigned loopback-only")
    allowed = {"AORTA_GROUP", "AORTA_LOG_TOPIC_ENABLED", "ZENOH_SESSION_CONFIG_URI"}
    if any(k.startswith(("AORTA_", "ZENOH_")) and k not in allowed for k in os.environ):
        raise ValueError("unexpected self-test transport override")
    return group


def self_test_client(output, seconds):
    """Protocol/timer fixture, not sdkmain inference or a hardware demo."""
    output = Path(output).resolve()
    group = validate_client_environment(output)
    from locomotion_aorta import PullController, BodyCmd, BodyJointCmd
    stop = False
    def stop_handler(_number, _frame):
        nonlocal stop
        stop = True
    handlers = {sig: signal.signal(sig, stop_handler) for sig in (signal.SIGINT, signal.SIGTERM)}
    controller = None
    capture = []
    entered = False
    try:
        # SDK checks its requested group against Core's effective AORTA_GROUP.
        controller = PullController(group=group)
        if controller.transport.node.group() != group:
            raise ValueError("effective self-test group differs")
        reply = controller.get_robot_info(timeout=5)
        if reply.info is None or reply.info.body is None or reply.info.body.slot_count != 31:
            raise ValueError("mock RobotInfo unavailable")
        controller.wait_state(5)
        controller.enter_external(timeout=5)
        entered = True
        deadline = time.monotonic() + seconds
        next_tick = time.monotonic_ns()
        while not stop and time.monotonic() < deadline:
            now = time.monotonic_ns()
            if now < next_tick:
                time.sleep((next_tick - now) / 1e9)
            cycle_start = time.monotonic_ns()
            next_tick += ((max(cycle_start, next_tick) - next_tick) // 20000000 + 1) * 20000000
            state = controller.wait_state(0)
            command = BodyCmd(joints=tuple(BodyJointCmd() if i == 14 else BodyJointCmd(q=float(state.q[i]), kp=1., kd=.5) for i in range(31)),
                              input_state_id=state.state_id, input_state_ts_real_us=state.ts_state_real_us)
            started = time.time_ns()
            sent = controller.publish(command)
            returned = time.time_ns()
            capture.append({"record_type": "synthetic_protocol_cycle", "session_id": sent.session_id,
                            "sequence": sent.sequence, "input_state_id": sent.input_state_id,
                            "input_state_ts_real_us": sent.input_state_ts_real_us,
                            "cycle_start_monotonic_ns": cycle_start,
                            "timing_wall_ns": {"state_received_wall_ns": state.received_realtime_ns,
                                               "sdk_publish_started_wall_ns": started, "sdk_returned_wall_ns": returned},
                            "scope": "no model/inference/real robot; S3/S4 not measured"})
    finally:
        try:
            if controller is not None:
                try:
                    if entered:
                        controller.exit_external(timeout=5)
                finally:
                    controller.close()
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
            (output / "self-test-client.jsonl").write_text("".join(json.dumps(r) + "\n" for r in capture))


def run(args):
    validate_options(args)
    build = args.build_dir.resolve()
    manifest = verify_build(build)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    group = validate_group(args.group or os.environ.get("SDK_GROUP") or new_group())
    if args.self_test_client:
        args.client_command = [sys.executable, str(Path(__file__).resolve()), "--mode", "production-topics",
                               "--build-dir", str(build), "--output", str(output), "--seconds", str(args.seconds), "--client-worker"]
        args.client_log = output / "self-test-client.jsonl"
    server_port = port()
    profiles = {"mock": output / "mock-session.json", "client": output / "client-session.json"}
    auxiliary = validate_auxiliary(json.loads(args.auxiliary_fixture.read_text())) if args.auxiliary_fixture else None
    if auxiliary:
        auxiliary_capture_budget(auxiliary, args.seconds)
    aux_port = None
    if auxiliary:
        aux_port = port()
        while aux_port == server_port:
            aux_port = port()
        profiles["auxiliary"] = output / "auxiliary-session.json"
        profiles["auxiliary"].write_text(session_config(aux_port))
    profiles["mock"].write_text(session_config(server_port))
    # Each Node opens its own session; a fixed client listener collides in a
    # multi-Node process. Both Nodes connect to the same assigned Mock peer.
    profiles["client"].write_text(session_config(0, [server_port, aux_port] if aux_port else server_port))
    initial = [0.0] * 31 if not args.initial_q else json.loads(args.initial_q.read_text())
    if not isinstance(initial, list) or len(initial) != 31 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or abs(v) > 10 for v in initial):
        raise ValueError("initial-q must be a31 element finite JSON list in synthetic range")
    (output / "initial-q.txt").write_text(" ".join(str(v) for v in initial) + "\n")
    imu = validate_imu(json.loads(args.initial_imu.read_text()) if args.initial_imu else [[1, 0, 0, 0], [1, 0, 0, 0]])
    (output / "initial-imu.txt").write_text(" ".join(str(v) for q in imu for v in q) + "\n")
    envs = {role: child_environment(os.environ, group, uri) for role, uri in profiles.items()}
    envs["mock"]["LD_LIBRARY_PATH"] = str(build) + os.pathsep + str(Path(manifest["core_path"]).parent) + os.pathsep + envs["mock"].get("LD_LIBRARY_PATH", "")
    for side in ("left", "right"):
        cfg = auxiliary.get(side) if auxiliary else None
        envs["mock"]["PRODUCTION_MOCK_EE_" + side.upper() + "_KIND"] = str({"brainco": 1, "gripper": 2}[cfg["kind"]] if cfg else 0)
    exported = {"SDK_GROUP": group, "AORTA_GROUP": group, "AORTA_LOG_TOPIC_ENABLED": "false",
                "ZENOH_SESSION_CONFIG_URI": str(profiles["client"])}
    (output / "client-environment.json").write_text(json.dumps(exported, indent=2) + "\n")
    # Keep the service alive while a cooperative client handles SIGTERM by CANCEL.
    mock_seconds = args.seconds + (10 if args.client_command else 0)
    command = [str(build / "production-topic-mock"), "--production-topics", str(output), group, str(server_port),
               str(args.state_hz), str(args.apply_hz), str(mock_seconds), str(args.command_timeout_ms),
               str(args.prepare_timeout_ms), str(args.reject_operation)]
    request = {"mode": args.mode, "group": group, "state_hz_requested": args.state_hz,
               "apply_hz_requested": args.apply_hz, "source_seconds_limit": args.seconds,
               "client_command": args.client_command, "mock_command": command,
               "build": manifest, "isolation": "fresh peer profiles, loopback endpoints only, no multicast/gossip/router/SHM",
               "initial_q_sha256": digest(output / "initial-q.txt"), "client_execution_is_not_sandboxed": True,
               "initial_imu_sha256": digest(output / "initial-imu.txt"),
               "scope": "synthetic; no real locomotion/CAN/EE hardware evidence"}
    (output / "request.json").write_text(json.dumps(request, indent=2) + "\n")
    if auxiliary:
        (output / "auxiliary-fixture.json").write_text(json.dumps(auxiliary, indent=2) + "\n")
    processes = {}
    result = {"mock_rc": None, "client_rc": None, "report": "NOT_MEASURED", "errors": []}
    logs = []
    try:
        mock_log = (output / "mock.log").open("w")
        logs.append(mock_log)
        processes["mock"] = subprocess.Popen(command, env=envs["mock"], stdout=mock_log, stderr=subprocess.STDOUT, start_new_session=True)
        if auxiliary:
            aux_log = (output / "auxiliary.log").open("w")
            logs.append(aux_log)
            aux_command = [sys.executable, str(Path(__file__).resolve()), "--mode", "production-topics", "--build-dir", str(build),
                           "--output", str(output), "--seconds", str(args.seconds), "--auxiliary-fixture", str(output / "auxiliary-fixture.json"), "--auxiliary-worker"]
            processes["auxiliary"] = subprocess.Popen(aux_command, env=envs["auxiliary"], stdout=aux_log, stderr=subprocess.STDOUT, start_new_session=True)
        if args.client_command:
            client_log = (output / "client.log").open("w")
            logs.append(client_log)
            processes["client"] = subprocess.Popen(args.client_command, env=envs["client"], stdout=client_log, stderr=subprocess.STDOUT, start_new_session=True)
        client_stop = time.monotonic() + args.seconds
        deadline = client_stop + 20
        client_stop_sent = False
        while processes["mock"].poll() is None:
            if "auxiliary" in processes and processes["auxiliary"].poll() not in (None, 0):
                raise RuntimeError("optional EE/Joy process failed; see auxiliary.log")
            if "client" in processes and processes["client"].poll() is not None:
                result["client_rc"] = processes["client"].returncode
                terminate(processes["mock"])
                break
            if "client" in processes and time.monotonic() >= client_stop and not client_stop_sent:
                os.killpg(processes["client"].pid, signal.SIGTERM)
                client_stop_sent = True
                result["client_stop_request"] = "SIGTERM while mock remains available for cooperative CANCEL"
            if time.monotonic() > deadline:
                raise TimeoutError("own mock process exceeded bounded source window")
            time.sleep(.05)
        result["mock_rc"] = processes["mock"].wait(timeout=6)
    except BaseException as exc:
        result["errors"].append(type(exc).__name__ + ": " + str(exc))
        if isinstance(exc, KeyboardInterrupt):
            result["interrupted"] = True
    finally:
        for role in ("client", "auxiliary", "mock"):
            terminate(processes.get(role))
            if role in processes:
                result[role + "_rc"] = processes[role].returncode
        for stream in logs:
            stream.close()
        try:
            report = write_report(output, args.client_log)
            result["report"] = report["status"]
        except (OSError, ValueError, KeyError) as exc:
            result["errors"].append("report: " + repr(exc))
        result["not_hardware_validation"] = True
        (output / "runner-result.json").write_text(json.dumps(result, indent=2) + "\n")
        # Record only declared files under our new output; never a caller's source/venv.
        (output / "SHA256SUMS").write_text("".join(digest(p) + "  " + p.name + "\n" for p in sorted(output.iterdir()) if p.is_file() and p.name != "SHA256SUMS"))
    if result["errors"] or result["mock_rc"] != 0 or (args.client_command and result["client_rc"] != 0):
        return 2
    return 0


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", choices=("production-topics",), required=True)
    p.add_argument("--build-dir", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seconds", type=float, default=60)
    p.add_argument("--state-hz", type=float, default=400)
    p.add_argument("--apply-hz", type=float, default=500)
    p.add_argument("--command-timeout-ms", type=int, default=1000)
    p.add_argument("--prepare-timeout-ms", type=int, default=5000)
    p.add_argument("--reject-operation", type=int, choices=(0, 1, 2, 3), default=0)
    p.add_argument("--group")
    p.add_argument("--initial-q", type=Path)
    p.add_argument("--initial-imu", type=Path, help="Two normalized WXYZ JSON quaternions matching the client's approved alignment")
    p.add_argument("--client-log", type=Path)
    p.add_argument("--allow-client", action="store_true")
    p.add_argument("--self-test-client", action="store_true", help="Explicit50Hz SDK protocol fixture, without sdkmain/models/hardware")
    p.add_argument("--client-command", nargs=argparse.REMAINDER)
    p.add_argument("--auxiliary-fixture", type=Path, help="Explicit EE Info/rates and/or scripted Joy; no hardware defaults")
    p.add_argument("--auxiliary-worker", action="store_true", help=argparse.SUPPRESS)
    p.add_argument("--client-worker", action="store_true", help=argparse.SUPPRESS)
    return p


if __name__ == "__main__":
    args = parser().parse_args()
    if args.client_worker:
        validate_options(args)
        verify_build(args.build_dir)
        self_test_client(args.output, args.seconds)
    elif args.auxiliary_worker:
        validate_options(args)
        verify_build(args.build_dir)
        validate_worker_environment(args.output)
        if not args.auxiliary_fixture:
            raise ValueError("auxiliary fixture is required")
        run_auxiliary(args.output, args.auxiliary_fixture, args.seconds)
    else:
        sys.exit(run(args))
