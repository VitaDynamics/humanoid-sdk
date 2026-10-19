"""Metadata-driven Velocity mode policy; no transport or robot operations."""

from collections import deque
from pathlib import Path
import hashlib

import numpy as np
import yaml


PROFILE = "actions/policy_profiles/vt_human_v2_velocity_asample.yaml"


class VelocityModePolicy:
    def __init__(self, locomotion_dir: Path) -> None:
        profile_path = locomotion_dir / PROFILE
        self.config = yaml.safe_load(profile_path.read_text())["humanoid_velocity"]
        self.model_path = locomotion_dir / self.config["model_path"]
        from locomotion_aorta._onnx import Session
        self.session = Session(str(self.model_path), intra_threads=1, inter_threads=1)
        self.metadata = self.session.metadata
        md = self.metadata
        if md["policy_kind"] != "velocity" or md["include_gait_observation"] != "false":
            raise ValueError("expected the phase-free action 13 velocity contract")
        self.names = md["joint_names"].split(",")
        self.bind_body_info()
        self.default_q = self._joint_values("default_joint_pos", "default_joint_pos")
        self.kp = self._joint_values("joint_stiffness", "joint_kp")
        self.kd = self._joint_values("joint_damping", "joint_kd")
        self.action_scale = self._values("action_scale")
        self.lower = self._values("joint_pos_limit_lower")
        self.upper = self._values("joint_pos_limit_upper")
        for p, name in enumerate(self.names):
            if name in self.config.get("joint_limits", {}):
                lo, hi = self.config["joint_limits"][name]
                self.lower[p] = max(self.lower[p], lo)
                self.upper[p] = min(self.upper[p], hi)
        self.terms = md["observation_names"].split(",")
        self.history_len = int(md["history_length"])
        self.frame_dim = int(md["frame_dim"])
        if md["history_order"] != "oldest_to_newest" or md["history_reset"] != "backfill_first_frame":
            raise ValueError("unsupported history contract")
        self.input_name = md["input_name"]
        self.output_name = md["output_name"]
        if self.session.inputs[0].shape != [1, self.history_len * self.frame_dim]:
            raise ValueError("ONNX input shape differs from its metadata")
        if self.config["command"]["smoothing"]["enabled"]:
            raise ValueError("this velocity mode example requires the source profile's disabled smoothing")
        self.inventory = {
            "profile": PROFILE,
            "profile_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(),
            "model": self.config["model_path"],
            "model_sha256": hashlib.sha256(self.model_path.read_bytes()).hexdigest(),
            "checkpoint": md["checkpoint"],
            "input_shape": [1, self.history_len * self.frame_dim],
            "output_shape": [1, len(self.names)],
            "policy_joint_names": self.names,
            "body_joint_names": self.order,
            "policy_to_body_slot": self.policy_to_body,
            "fixed_joints": {n: v for n, v in self.config["fixed_joints"].items() if n in self.order},
        }
        self.reset()

    def bind_body_info(self, body_info=None):
        from locomotion_aorta._policy_state import body_joint_order
        self.order = body_joint_order(body_info)
        self.policy_to_body = [self.order.index(name) for name in self.names]
        self.driven_slots = set(self.policy_to_body)
        self.driven_slots.update(self.order.index(n) for n in self.config["fixed_joints"] if n in self.order)
        if hasattr(self, "inventory"):
            self.inventory.update(body_joint_names=self.order, policy_to_body_slot=self.policy_to_body)

    def _values(self, key: str) -> np.ndarray:
        result = np.asarray([float(v) for v in self.metadata[key].split(",")], dtype=np.float64)
        if result.shape != (len(self.names),):
            raise ValueError(f"metadata {key} length differs from joint_names")
        return result

    def _joint_values(self, metadata_key: str, override_key: str) -> np.ndarray:
        result = self._values(metadata_key)
        for p, name in enumerate(self.names):
            result[p] = self.config.get(override_key, {}).get(name, result[p])
        return result

    def reset(self) -> None:
        self.history: deque[np.ndarray] = deque(maxlen=self.history_len)
        self.last_action = np.zeros(len(self.names), dtype=np.float32)

    def command(self, axes: tuple[float, ...], twist=(0.0, 0.0, 0.0)) -> np.ndarray:
        cfg = self.config["command"]
        result = []
        for name in ("vx", "vy", "wz"):
            i = cfg[f"joy_axis_{name}"]
            v = float(axes[i]) if 0 <= i < len(axes) else 0.0
            lo, hi = cfg[f"{name}_range"]
            result.append(0.0 if abs(v) < cfg["deadband"] else v * (hi if v >= 0 else -lo))
        # Source profile is twist+joy: active sticks win, otherwise /vel_cmd.
        if not any(result):
            result = list(twist)
        return np.asarray([
            min(max(v, cfg[f"{name}_range"][0]), cfg[f"{name}_range"][1])
            for v, name in zip(result, ("vx", "vy", "wz"))
        ], dtype=np.float64)

    def observe(self, q: tuple[float, ...], dq: tuple[float, ...],
                gyro: tuple[float, ...], quat: tuple[float, ...],
                command: np.ndarray) -> np.ndarray:
        # Same conjugate-quaternion rotation and double arithmetic as
        # VtHumanProjectedGravity; do not renormalize the published quaternion.
        w, x, y, z = map(float, quat)
        gravity = [2.0 * (w * y - x * z),
                   -2.0 * (w * x + y * z),
                   -1.0 + 2.0 * (x * x + y * y)]
        cfg = self.config["command"]
        cmd = command.copy()
        if (cfg["stand_enabled"] and abs(cmd[0]) < cfg["stand_vxy_deadband"]
                and abs(cmd[1]) < cfg["stand_vxy_deadband"]
                and abs(cmd[2]) < cfg["stand_wz_deadband"]):
            cmd[:] = 0.0
        terms = {
            "base_ang_vel": np.asarray(gyro, dtype=np.float64),
            "projected_gravity": np.asarray(gravity, dtype=np.float64),
            "command": cmd,
            "joint_pos": np.asarray(q, dtype=np.float64)[self.policy_to_body] - self.default_q,
            "joint_vel": np.asarray(dq, dtype=np.float64)[self.policy_to_body],
            "actions": self.last_action.astype(np.float64),
        }
        frame = np.concatenate([
            terms[term] * float(self.metadata.get(f"obs_scale_{term}", "1"))
            for term in self.terms
        ]).astype(np.float32)
        frame = np.clip(frame, -np.float32(self.metadata["clip_observations"]),
                        np.float32(self.metadata["clip_observations"]))
        if frame.size != self.frame_dim:
            raise ValueError("observation frame differs from the model contract")
        if not self.history:
            self.history.extend(frame.copy() for _ in range(self.history_len))
        else:
            self.history.append(frame)
        return np.concatenate(tuple(self.history)).reshape(1, -1)

    def infer(self, observation: np.ndarray) -> np.ndarray:
        return self.session.run(self.input_name, self.output_name, observation)[0]

    def decode(self, raw_action: np.ndarray) -> list[dict]:
        self.last_action = np.clip(raw_action, -np.float32(self.metadata["clip_actions"]),
                                   np.float32(self.metadata["clip_actions"])).astype(np.float32)
        raw_q = self.last_action.astype(np.float64) * self.action_scale + self.default_q
        target_q = np.clip(raw_q, self.lower, self.upper)
        result = [dict(q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0)
                  for _ in self.order]
        for p, slot in enumerate(self.policy_to_body):
            tau = (self.kp[p] * (raw_q[p] - target_q[p])
                   if self.config.get("ankle_clamp_feedforward", False) and "ankle" in self.names[p]
                   else 0.0)
            result[slot] = dict(q=float(np.float32(target_q[p])),
                                dq=0.0, tau=float(np.float32(tau)),
                                kp=float(np.float32(self.kp[p])), kd=float(np.float32(self.kd[p])))
        for name, hold in self.config["fixed_joints"].items():
            if name not in self.order:
                self.config["hardware_joint_order"].index(name)  # Preserve unknown-name failure.
                continue  # End effectors are not part of the public body command.
            slot = self.order.index(name)
            if slot in self.policy_to_body:
                raise ValueError(f"fixed joint {name} is already policy-driven")
            result[slot].update(q=float(np.float32(hold.get("q", 0.0))),
                                kp=float(np.float32(hold.get("kp", 0.0))),
                                kd=float(np.float32(hold.get("kd", 0.0))))
        return result
