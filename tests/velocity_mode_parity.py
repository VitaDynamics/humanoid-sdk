"""Replay one archived wire frame and deterministic perturbations through C++/Python."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))
from velocity_mode import decode_state
from velocity_mode_policy import VelocityModePolicy
from lowlevel.BodyState import BodyState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--locomotion-dir", type=Path, required=True)
    parser.add_argument("--robot-source", type=Path, required=True, help="validation-only fixed robot header root")
    parser.add_argument("--state-bin", type=Path, required=True)
    parser.add_argument("--ort-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    policy = VelocityModePolicy(args.locomotion_dir)
    frame = decode_state(BodyState.GetRootAs(args.state_bin.read_bytes()), 1)
    oracle = args.output_dir / "oracle"
    command = ["g++", "-std=c++17", "-O2", str(Path(__file__).with_suffix(".cpp")),
               "-I", str(args.robot_source), "-I", str(args.ort_dir / "include"),
               "-L", str(args.ort_dir / "lib"), f"-Wl,-rpath,{args.ort_dir / 'lib'}",
               "-lonnxruntime", "-o", str(oracle)]
    subprocess.run(command, check=True)
    n, frames = len(policy.names), 128
    lines = [f"{frames} {policy.history_len} {policy.frame_dim} {n} {len(policy.terms)} "
             f"{policy.metadata['clip_observations']} {policy.metadata['clip_actions']}"]
    lines.extend(f"{term} {policy.metadata.get('obs_scale_' + term, '1')}" for term in policy.terms)
    for values in (policy.default_q, policy.action_scale, policy.lower, policy.upper):
        lines.append(" ".join(map(repr, values.tolist())))
    observations, actions, targets = [], [], []
    for index in range(frames):
        q = np.asarray(frame.q, dtype=np.float64).copy()
        dq = np.asarray(frame.dq, dtype=np.float64).copy()
        if index:
            q[policy.policy_to_body] += .001 * np.sin(np.arange(len(policy.names)) + index)
            dq[policy.policy_to_body] += .01 * np.cos(np.arange(len(policy.names)) + index)
        cmd = policy.command((0., float(np.sin(index * .13)), 0., float(np.cos(index * .09))))
        # The exact standing-zero rule is applied before either implementation.
        cfg = policy.config["command"]
        if abs(cmd[0]) < cfg["stand_vxy_deadband"] and abs(cmd[2]) < cfg["stand_wz_deadband"]:
            cmd[:] = 0.
        for values in (frame.gyro, frame.quat, cmd, q[policy.policy_to_body], dq[policy.policy_to_body]):
            lines.append(" ".join(map(repr, list(values))))
        obs = policy.observe(q, dq, frame.gyro, frame.quat, cmd)
        action = policy.infer(obs)
        motors = policy.decode(action)
        observations.append(obs.ravel())
        actions.append(action)
        targets.append(np.asarray([motors[slot]["q"] for slot in policy.policy_to_body], dtype=np.float32))
    input_data = ("\n".join(lines) + "\n").encode()
    (args.output_dir / "input.txt").write_bytes(input_data)
    output = subprocess.run([str(oracle), str(policy.model_path)], input=input_data,
                            capture_output=True, check=True).stdout
    (args.output_dir / "oracle.bin").write_bytes(output)
    dtypes = np.dtype([("obs", "<f4", policy.history_len * policy.frame_dim),
                       ("action", "<f4", n), ("target", "<f8", n)])
    native = np.frombuffer(output, dtype=dtypes)
    python_values = dict(obs=np.asarray(observations), action=np.asarray(actions), target=np.asarray(targets))
    comparisons = {}
    for key, value in python_values.items():
        native_value = native[key].astype(np.float32) if key == "target" else native[key]
        comparisons[key] = {"max_abs_error": float(np.max(abs(value - native_value))),
                            "exact_equal": bool(np.isfinite(value).all() and np.isfinite(native_value).all() and np.array_equal(value, native_value))}
    report = dict(frames=frames, source_state_id=frame.state.state_id,
                  state_sha256=hashlib.sha256(args.state_bin.read_bytes()).hexdigest(),
                  model=policy.inventory, comparisons=comparisons,
                  method="unmodified C++ vt_human_obs.h plus native ORT vs Python; 1 archived frame and 127 deterministic perturbations")
    (args.output_dir / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not all(row["exact_equal"] for row in comparisons.values()):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
