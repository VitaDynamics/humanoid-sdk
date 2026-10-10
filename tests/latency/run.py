"""Bounded real IPC sweep. No profiling, models, robot topic, or control session."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import signal
import socket
import subprocess
import sys
import time

from report import analyze, render

ROOT = Path(__file__).resolve().parents[2]
RATES = [50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000]


def config_files(directory, port):
    common = {"mode": "peer", "scouting": {"multicast": {"enabled": False},
              "gossip": {"enabled": False}},
              "transport": {"shared_memory": {"enabled": False},
                            "unicast": {"compression": {"enabled": False}}}}
    for role, listen, connect in (("python", [f"tcp/127.0.0.1:{port}"], []),
                                  ("mock", ["tcp/127.0.0.1:0"], [f"tcp/127.0.0.1:{port}"])):
        (directory / f"{role}.json").write_text(json.dumps({**common,
            "listen": {"endpoints": listen}, "connect": {"endpoints": connect}}, indent=2))


def terminate(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)  # only the session we created
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def window(build, directory, rate, seconds):
    directory.mkdir()
    session = time.time_ns()
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    # A port race is a visible process/matching failure, never an automatic retry.
    config_files(directory, port)
    env = {k: v for k, v in os.environ.items() if not k.startswith(("AORTA_", "ZENOH_"))}
    env["RUST_LOG"] = "error"
    processes, logs = [], []
    commands = [
        ("python", [sys.executable, str(ROOT / "tests/latency/client.py"), str(directory), str(session), str(seconds)]),
        ("mock", [str(build / "mock"), str(directory), str(session), str(rate), str(seconds),
                  str(build / "generated/humanoid_low_state.bfbs")]),
    ]
    (directory / "commands.json").write_text(json.dumps(commands, indent=2))
    try:
        for role, argv in commands:
            log = (directory / f"{role}.log").open("w")
            logs.append(log)
            p = subprocess.Popen(argv, cwd=ROOT, env={**env,
                "ZENOH_SESSION_CONFIG_URI": str(directory / f"{role}.json")},
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(p)
        mock_rc = processes[1].wait(timeout=seconds + 45)
        (directory / "stop").touch()
        client_rc = processes[0].wait(timeout=15)
        (directory / "exit.json").write_text(json.dumps({"mock": mock_rc, "python": client_rc}))
        if mock_rc or client_rc:
            raise RuntimeError(f"{rate} Hz: mock exit {mock_rc}, python exit {client_rc}; see per-process logs")
        return analyze(directory, session, seconds, rate)
    finally:
        for p in processes:
            terminate(p)
        for log in logs:
            log.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--rates", type=int, nargs="+", default=RATES)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 120 or len(set(args.rates)) != len(args.rates) or any(
            hz < 1 or hz > 1000 or args.seconds * hz + 1 > 65000 for hz in args.rates):
        parser.error("invalid duration/rates/capture bound")
    output, build = args.output.resolve(), args.build_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest = {"status": "NOT_MEASURED", "results": [], "rates": args.rates,
                "seconds": args.seconds, "platform": platform.platform(), "python": sys.version,
                "cpu_count": os.cpu_count(), "clock": "CLOCK_MONOTONIC ns", "transport": "TCP loopback; SHM off",
                "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                "pr_head": os.environ.get("PR_HEAD_SHA"), "run_id": os.environ.get("GITHUB_RUN_ID"),
                "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for base in (ROOT / "tests/latency", ROOT / "python/locomotion_aorta")
                                  for p in sorted(base.rglob("*")) if p.suffix in (".py", ".h", ".cpp")}}
    try:
        manifest["build"] = json.loads((build / "build.json").read_text())
        for rate in args.rates:
            print(f"START {rate} Hz, {args.seconds} s", flush=True)
            manifest["results"].append(window(build, output / f"{rate}hz", rate, args.seconds))
            render(output, manifest)
        manifest["status"] = "PASS" if args.rates == RATES and args.seconds >= 60 else "SMOKE_ONLY"
    except Exception as error:
        manifest["status"] = "FAIL"
        manifest["error"] = repr(error)
    finally:
        render(output, manifest)
    print(manifest["status"], output / "report.html", flush=True)
    return 1 if manifest["status"] == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
