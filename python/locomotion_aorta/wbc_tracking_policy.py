"""GR00T Vita02A G1 observation/target math; no transport or robot side effects."""
from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np

from .body_types import BodyCmd, BodyJointCmd

CONTRACT_KEY = 'groot_wbc.tracking_contract.v1'
REF_DT = 0.02
HIP = 0
SAMPLE_OK = 1


def finite(value, name):
    value = np.asarray(value, dtype=np.float64)
    if not np.isfinite(value).all():
        raise ValueError(f'{name} contains NaN or infinity')
    return value


def quaternion(value):
    q = finite(value, 'quaternion')
    if q.shape != (4,) or not .5 <= np.linalg.norm(q) <= 1.5:
        raise ValueError('expected a plausible wxyz quaternion')
    return q / np.linalg.norm(q)


def conjugate(q):
    return q * np.array([1., -1., -1., -1.])


def multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw*bw-ax*bx-ay*by-az*bz,
                     aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx,
                     aw*bz+ax*by-ay*bx+az*bw])


def rotation(q):
    w, x, y, z = quaternion(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def yaw(q):
    r = rotation(q)
    return math.atan2(r[1, 0], r[0, 0])


def yaw_quaternion(angle):
    return np.array([math.cos(angle/2), 0., 0., math.sin(angle/2)])


def slerp(a, b, alpha):
    a, b = quaternion(a), quaternion(b)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    dot = min(dot, 1.)
    if dot > .9995:
        return quaternion(a + alpha*(b-a))
    angle = math.acos(dot)
    return quaternion((math.sin((1-alpha)*angle)*a + math.sin(alpha*angle)*b)
                      / math.sin(angle))


@dataclass(frozen=True)
class Contract:
    names: tuple[str, ...]
    default: np.ndarray
    scale: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    action_clip: float
    period: float
    future_frames: int
    future_dt: float
    history_length: int
    input_dim: int
    include_z: bool

    @classmethod
    def from_metadata(cls, metadata):
        if CONTRACT_KEY not in metadata:
            raise ValueError(f'model is missing {CONTRACT_KEY}')
        raw = json.loads(metadata[CONTRACT_KEY])
        if (raw.get('schema_version') != 1 or raw.get('action_dim') != 28
                or raw.get('policy_kind') != 'groot_vita02a_tracking'
                or raw.get('actor_history_order') != 'term_major_oldest_first'
                or raw.get('input_layout') != 'tokenizer_then_proprioception'):
            raise ValueError('unsupported GR00T tracking contract/layout')
        names = tuple(raw['joint_names'])
        if len(names) != 28 or len(set(names)) != 28 or not all(names):
            raise ValueError('model must name 28 distinct policy joints')
        arrays = [finite(raw[k], k) for k in (
            'default_joint_pos', 'action_scale', 'joint_stiffness',
            'joint_damping', 'joint_pos_limit_lower', 'joint_pos_limit_upper')]
        if any(x.shape != (28,) for x in arrays):
            raise ValueError('model arrays must match the 28 named joints')
        default, scale, kp, kd, lower, upper = arrays
        if (np.any(scale <= 0) or np.any(kp < 0) or np.any(kd < 0)
                or np.any(lower >= upper) or np.any(default < lower)
                or np.any(default > upper)):
            raise ValueError('model has invalid scales, gains or limits')
        period, clip = float(raw['policy_dt']), float(raw['action_clip'])
        if not math.isclose(period, REF_DT, abs_tol=1e-9) or not math.isfinite(clip) or clip <= 0:
            raise ValueError('expected a 50 Hz policy and finite positive action_clip')
        future = raw['future_reference']
        count, dt = future['num_frames'], float(future['dt'])
        if (type(count) is not int or count <= 0 or not math.isfinite(dt) or dt < REF_DT
                or not math.isclose(dt/REF_DT, round(dt/REF_DT), abs_tol=1e-7)
                or future['quaternion_order'] != 'wxyz'
                or future['orientation_representation'] != 'rotation_matrix_6d'):
            raise ValueError('unsupported future-reference timing/orientation')
        terms = raw['actor_observation_terms']
        lengths = {x['history_length'] for x in terms}
        if len(lengths) != 1:
            raise ValueError('all proprioception terms must share one history length')
        history = lengths.pop()
        expected = [('base_ang_vel', 3), ('joint_pos', 28), ('joint_vel', 28),
                    ('actions', 28), ('gravity_dir', 3)]
        if ([(x['name'], x['frame_dim']) for x in terms] != expected
                or type(history) is not int or history <= 0):
            raise ValueError('unsupported proprioception term order')
        tokenizer = raw['tokenizer_observations']
        include_z = any(x['name'] == 'command_z_multi_future_nonflat' for x in tokenizer)
        tok_expected = [('command_multi_future_nonflat', [count, 56])]
        if include_z:
            tok_expected.append(('command_z_multi_future_nonflat', [count]))
        tok_expected.append(('motion_anchor_ori_b_mf_nonflat', [count, 6]))
        if [(x['name'], x['shape']) for x in tokenizer] != tok_expected:
            raise ValueError('unsupported tokenizer observation layout')
        dim = count*(56+6+int(include_z)) + history*90
        if raw['input_dim'] != dim or raw['proprioception_dim'] != history*90:
            raise ValueError('metadata observation dimensions disagree')
        if raw.get('ankle', {}).get('control_space') != 'virtual_pitch_roll':
            raise ValueError('tracking example requires virtual pitch/roll ankles')
        return cls(names, default, scale, kp, kd, lower, upper, clip, period,
                   count, dt, history, dim, include_z)


class Motion:
    def __init__(self, positions, base_positions, quaternions):
        self.q = finite(positions, 'reference joint positions')
        self.base = finite(base_positions, 'reference base positions')
        quaternions = finite(quaternions, 'reference quaternions')
        n = len(self.q)
        if n == 0 or self.q.shape != (n, 28) or self.base.shape != (n, 3) or quaternions.shape != (n, 4):
            raise ValueError('expected reference arrays [N,28], [N,3], [N,4]')
        self.quat = np.array([quaternion(q) for q in quaternions])

    @classmethod
    def hold(cls, q, quat=(1., 0., 0., 0.)):
        return cls([q], [[0., 0., 0.]], [quat])

    @classmethod
    def from_npz(cls, path, names):
        with np.load(path, allow_pickle=False) as data:
            src_names = [n.decode() if isinstance(n, bytes) else str(n) for n in data['joint_names']]
            if len(set(src_names)) != len(src_names):
                raise ValueError('motion has duplicate joint names')
            cols = [src_names.index(n) for n in names]
            q = finite(data['joint_pos'], 'joint_pos')
            base = finite(data['base_pos_w'], 'base_pos_w')
            quat = finite(data['base_quat_w'], 'base_quat_w')
            fps = float(np.asarray(data['framerate']).reshape(()))
        if not math.isfinite(fps) or fps <= 0 or q.ndim != 2 or q.shape[1] != len(src_names):
            raise ValueError('invalid motion framerate/joint axis')
        src = cls(q[:, cols], base, quat)
        time = np.arange(len(q))/fps
        target = np.arange(0., time[-1]+.5*REF_DT, REF_DT)
        qp = np.stack([np.interp(target, time, src.q[:, j]) for j in range(28)], axis=1)
        bp = np.stack([np.interp(target, time, base[:, j]) for j in range(3)], axis=1)
        bq = []
        for t in target:
            if len(time) == 1 or t >= time[-1]:
                bq.append(src.quat[-1])
            else:
                i = max(0, int(np.searchsorted(time, t, side='right'))-1)
                bq.append(slerp(src.quat[i], src.quat[i+1], (t-time[i])*fps))
        return cls(qp, bp, bq)

    def normalized_heading(self):
        rot = yaw_quaternion(-yaw(self.quat[0]))
        base = (self.base-self.base[0]) @ rotation(rot).T + self.base[0]
        return Motion(self.q.copy(), base, [multiply(rot, q) for q in self.quat])

    def indices(self, ref_time, count, dt):
        frame = int(round(ref_time/REF_DT))
        stride = max(1, int(round(dt/REF_DT)))
        return np.clip(frame+stride*np.arange(count), 0, len(self.q)-1)

    def velocities(self, indices):
        # C++ G1 endpoint rule reuses the penultimate forward slope.
        start = np.where(indices < len(self.q)-1, indices,
                         np.maximum(indices-(2 if len(self.q) >= 3 else 1), 0))
        end = np.minimum(start+1, len(self.q)-1)
        return (self.q[end]-self.q[start])/REF_DT

    def lead_in(self, q_start, quat_start, config):
        target = self.normalized_heading()
        speed = float(config['speed_q_per_s'])
        low, high = float(config['min_duration_s']), float(config['max_duration_s'])
        if not all(math.isfinite(x) for x in (speed, low, high)) or speed <= 0 or low <= 0 or high < low:
            raise ValueError('invalid lead-in speed/duration')
        q_start = finite(q_start, 'lead-in start pose')
        if q_start.shape != (28,):
            raise ValueError('lead-in requires 28 joint positions')
        n = max(1, int(round(float(np.clip(np.max(abs(target.q[0]-q_start))/speed, low, high))/REF_DT)))
        alpha = .5-.5*np.cos(np.pi*np.arange(n)/n)
        start = multiply(yaw_quaternion(-yaw(quat_start)), quaternion(quat_start))
        return Motion(np.vstack((q_start+alpha[:, None]*(target.q[0]-q_start), target.q)),
                      np.vstack((np.repeat(target.base[:1], n, axis=0), target.base)),
                      np.vstack(([slerp(start, target.quat[0], a) for a in alpha], target.quat)))


class WbcTrackingPolicy:
    def __init__(self, contract, profile):
        self.contract, self.profile = contract, profile
        self.order = tuple(profile['body_joint_order'])
        if (profile.get('schema_version') != 1 or profile.get('command_slots') != 31
                or profile.get('state_slots') != 31 or len(self.order) != 31
                or len(set(self.order)) != 31):
            raise ValueError('expected the explicit 31-command/31-state A-sample profile')
        self.mapping = np.array([self.order.index(n) for n in contract.names])
        limits = finite([profile['joint_limits'][n] for n in contract.names], 'profile limits')
        if limits.shape != (28, 2):
            raise ValueError('profile limits must contain one pair per policy joint')
        self.lower = np.maximum(contract.lower, limits[:, 0])
        self.upper = np.minimum(contract.upper, limits[:, 1])
        if np.any(self.lower >= self.upper) or np.any(contract.default < self.lower) or np.any(contract.default > self.upper):
            raise ValueError('ONNX and hardware limits have an invalid intersection')
        if set(profile['fixed_joints']) & set(contract.names):
            raise ValueError('fixed joints must not overlap policy joints')
        for name, fixed in profile['fixed_joints'].items():
            if (name not in self.order or not np.isfinite([fixed[k] for k in ('q', 'kp', 'kd')]).all()
                    or fixed['kp'] < 0 or fixed['kd'] < 0):
                raise ValueError('invalid fixed joint')
        tilt_limit = profile['fall_tilt_threshold_rad']
        if not math.isfinite(tilt_limit) or not 0 < tilt_limit < math.pi:
            raise ValueError('invalid body tilt limit')
        self.ankles = np.array([i for i, n in enumerate(contract.names) if 'ankle_' in n])
        if len(self.ankles) != 4:
            raise ValueError('expected four virtual ankle joints')
        self.history = deque(maxlen=contract.history_length)
        self.last_action = np.zeros(28, np.float32)

    def bind_body_info(self, body_info):
        from ._policy_state import body_joint_order
        self.order = body_joint_order(body_info,
                                     required_names=(*self.contract.names, *self.profile['fixed_joints']))
        self.mapping = np.array([self.order.index(n) for n in self.contract.names])

    def reset(self):
        self.history.clear()
        self.last_action[:] = 0

    def state(self, state):
        from .state_arrays import BodyStateArrays
        if isinstance(state, BodyStateArrays):
            if len(state.q) != 31 or not state.attitude_valid:
                raise ValueError('expected 31-slot feedback with valid HIP attitude')
            indices = np.flatnonzero(state.imu_location == HIP)
            if len(indices) != 1:
                raise ValueError('one healthy HIP IMU is required; TRUNK is not a fallback')
            i = indices[0]
            if state.imu_status[i] != SAMPLE_OK or not state.imu_state_present[i]:
                raise ValueError('one healthy HIP IMU is required; TRUNK is not a fallback')
            quat = quaternion(state.imu_values[i, :4])
            gyro = finite(state.imu_values[i, 4:7], 'HIP gyroscope')
            q = finite(state.q[self.mapping], 'joint position')
            dq = finite(state.dq[self.mapping], 'joint velocity')
        else:
            if len(state.joints) != 31 or not state.attitude_valid:
                raise ValueError('expected 31-slot feedback with valid HIP attitude')
            hip = [x for x in state.imu_states if x.location == HIP]
            if len(hip) != 1 or hip[0].status != SAMPLE_OK or hip[0].state is None:
                raise ValueError('one healthy HIP IMU is required; TRUNK is not a fallback')
            quat = quaternion(hip[0].state.quaternion)
            gyro = finite(hip[0].state.gyroscope, 'HIP gyroscope')
            q = finite([state.joints[i].q for i in self.mapping], 'joint position')
            dq = finite([state.joints[i].dq for i in self.mapping], 'joint velocity')
        if gyro.shape != (3,):
            raise ValueError('expected three body-frame angular velocities')
        gravity = rotation(quat).T @ [0., 0., -1.]
        tilt = math.acos(float(np.clip(-gravity[2], -1, 1)))
        if tilt > self.profile['fall_tilt_threshold_rad']:
            raise ValueError(f'body tilt {tilt:.3f} rad exceeds tracking profile limit')
        return q, dq, quat, gyro

    def observe(self, motion, ref_time, q, dq, quat, gyro, yaw_offset=0.):
        c = self.contract
        q, dq, gyro = finite(q, 'q'), finite(dq, 'dq'), finite(gyro, 'gyro')
        if q.shape != (28,) or dq.shape != (28,) or gyro.shape != (3,):
            raise ValueError('observation state dimensions disagree')
        quat = quaternion(quat)
        indices = motion.indices(ref_time, c.future_frames, c.future_dt)
        parts = [motion.q[indices].ravel(), motion.velocities(indices).ravel()]
        if c.include_z:
            parts.append(motion.base[indices, 2])
        offset = yaw_quaternion(yaw_offset)
        ori = [rotation(multiply(conjugate(quat), multiply(offset, motion.quat[i])))[:, :2].ravel() for i in indices]
        parts.append(np.asarray(ori).ravel())
        terms = tuple(np.asarray(x, np.float32) for x in (
            gyro, q-c.default, dq, self.last_action.copy(), rotation(quat).T @ [0., 0., -1.]))
        if not self.history:
            self.history.extend([terms]*c.history_length)
        else:
            self.history.append(terms)
        parts.extend(np.concatenate([h[k] for h in self.history]) for k in range(5))
        obs = np.concatenate(parts).astype(np.float32)
        if obs.shape != (c.input_dim,) or not np.isfinite(obs).all():
            raise ValueError('constructed observation is invalid')
        return obs

    def decode(self, raw_action, **timing):
        action = finite(raw_action, 'model action')
        if action.shape != (28,):
            raise ValueError('model output must have 28 actions')
        c = self.contract
        self.last_action = np.clip(action, -c.action_clip, c.action_clip).astype(np.float32)
        raw_q = c.default+c.scale*self.last_action.astype(np.float64)
        target = np.clip(raw_q, self.lower, self.upper)
        tau = np.zeros(28)
        if self.profile['ankle_clamp_feedforward']:
            tau[self.ankles] = c.kp[self.ankles]*(raw_q[self.ankles]-target[self.ankles])
        motors = [BodyJointCmd() for _ in range(31)]
        for p, slot in enumerate(self.mapping):
            motors[slot] = BodyJointCmd(float(np.float32(target[p])), 0.,
                                       float(np.float32(tau[p])),
                                       float(np.float32(c.kp[p])), float(np.float32(c.kd[p])))
        for name, fixed in self.profile['fixed_joints'].items():
            motors[self.order.index(name)] = BodyJointCmd(fixed['q'], 0., 0., fixed['kp'], fixed['kd'])
        return BodyCmd(joints=tuple(motors), **timing), {
            'action': self.last_action.tolist(), 'raw_target': raw_q.tolist(),
            'target': target.tolist(), 'tau_ff': tau.tolist(),
            'clamped': (target != raw_q).tolist(),
        }


class Model:
    def __init__(self, path, threads=1):
        from ._onnx import Session
        if threads < 1:
            raise ValueError('threads must be positive')
        self.session = Session(str(Path(path)), intra_threads=threads, inter_threads=1)
        self.contract = Contract.from_metadata(self.session.metadata)
        inputs, outputs = self.session.inputs, self.session.outputs
        if (len(inputs) != 1 or len(outputs) != 1
                or inputs[0].shape != [1, self.contract.input_dim]
                or outputs[0].shape != [1, 28]
                or inputs[0].dtype != 'tensor(float)' or outputs[0].dtype != 'tensor(float)'):
            raise ValueError('ONNX IO does not match its tracking metadata')
        self.input, self.output = inputs[0].name, outputs[0].name

    def infer(self, obs):
        obs = np.asarray(obs, np.float32)
        if obs.shape != (self.contract.input_dim,) or not np.isfinite(obs).all():
            raise ValueError('invalid model input')
        out = self.session.run(self.input, self.output, obs[None])
        if out.shape != (1, 28) or not np.isfinite(out).all():
            raise ValueError('invalid ONNX output')
        return out[0]
