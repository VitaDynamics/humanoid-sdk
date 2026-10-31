"""Explicit isolated runner. No robot connection, SDK import or child on --help."""
import argparse
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
    from . import validate_auxiliary, run_auxiliary
    from .group_contract import new_group, validate_group
    from .ee_fixture import prepare_ee_fixture
except ImportError:
    from __init__ import validate_auxiliary, run_auxiliary
    from group_contract import new_group, validate_group
    from ee_fixture import prepare_ee_fixture


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
    for name in ("command_timeout_ms", "prepare_timeout_ms"):
        value = getattr(args, name)
        if not 1 <= value <= 60000:
            raise ValueError("invalid " + name)
    if args.client_command and not args.allow_client:
        raise ValueError("client execution requires explicit --allow-client")


def validate_imu(quaternions):
    if not isinstance(quaternions, list) or len(quaternions) != 2:
        raise ValueError("initial IMU requires HIP and TRUNK WXYZ quaternions")
    for q in quaternions:
        if not isinstance(q, list) or len(q) != 4 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in q):
            raise ValueError("IMU quaternion requires four finite numbers")
        if abs(sum(v * v for v in q) - 1) > .001:
            raise ValueError("IMU quaternion must be normalized; no implicit correction")
    return quaternions


def run(args):
    validate_options(args)
    build = args.build_dir.resolve()
    manifest = verify_build(build)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    group = validate_group(args.group or os.environ.get("SDK_GROUP") or new_group())
    server_port = port()
    profiles = {"mock": output / "mock-session.json", "client": output / "client-session.json"}
    auxiliary = validate_auxiliary(json.loads(args.auxiliary_fixture.read_text())) if args.auxiliary_fixture else None
    if args.ee_fixture:
        prepare_ee_fixture(args.ee_fixture, output)
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
    exported = {"SDK_GROUP": group, "AORTA_GROUP": group, "AORTA_LOG_TOPIC_ENABLED": "false",
                "ZENOH_SESSION_CONFIG_URI": str(profiles["client"])}
    (output / "client-environment.json").write_text(json.dumps(exported, indent=2) + "\n")
    # Keep the service alive while a cooperative client handles SIGTERM by CANCEL.
    mock_seconds = args.seconds + (10 if args.client_command else 0)
    command = [str(build / "production-topic-mock"), "--production-topics", str(output), group, str(server_port),
               str(args.state_hz), str(args.apply_hz), str(mock_seconds), str(args.command_timeout_ms),
               str(args.prepare_timeout_ms), str(args.reject_operation)]
    if auxiliary:
        (output / "auxiliary-fixture.json").write_text(json.dumps(auxiliary, indent=2) + "\n")
    processes = {}
    result = {"mock_rc": None, "client_rc": None, "errors": []}
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
                raise RuntimeError("optional Joy process failed; see auxiliary.log")
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
        print(json.dumps(result))
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
    p.add_argument("--allow-client", action="store_true")
    p.add_argument("--client-command", nargs=argparse.REMAINDER)
    p.add_argument("--ee-fixture", type=Path, help="Main-owned EE: actual Info/Touch binary fixtures, initial positions, explicit rates and fixed observation sources")
    p.add_argument("--auxiliary-fixture", type=Path, help="Scripted Joy only; EE belongs to the main Mock")
    p.add_argument("--auxiliary-worker", action="store_true", help=argparse.SUPPRESS)
    return p


if __name__ == "__main__":
    args = parser().parse_args()
    if args.auxiliary_worker:
        if not args.auxiliary_fixture:
            raise ValueError("auxiliary fixture is required")
        run_auxiliary(args.output, args.auxiliary_fixture, args.seconds)
    else:
        sys.exit(run(args))
