"""Action 17 observation/ONNX -> D2 EXTERNAL BodyCmd example draft.

No transport is imported in dry-run. Model paths always come from --model.
"""
from __future__ import annotations

import argparse
from collections import deque
from contextlib import closing
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import signal
import struct
import sys
import threading
import time
from types import SimpleNamespace
from typing import Any, Callable, Sequence

from locomotion_aorta._demo_feedback import joint_columns, state_log_fields, publish_log_fields, NOT_MEASURED
from lowlevel.BodySampleStatus import BodySampleStatus
from lowlevel.JointError import JointError


def f32(value: float) -> float:
    return struct.unpack('f', struct.pack('f', value))[0]


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def finite(values: Sequence[float]) -> None:
    if not all(math.isfinite(x) for x in values):
        raise ValueError('policy input/output contains a nonfinite value')


def csv(metadata: dict[str, str], key: str, convert: Callable = float) -> tuple:
    return tuple(convert(x.strip()) for x in metadata[key].split(',') if x.strip())


@dataclass(frozen=True)
class MotorTarget:
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0


class Policy:
    def __init__(self, metadata: dict[str, str], contract: dict):
        terms = csv(metadata, 'observation_names', str)
        if terms != ('command', 'height_command', 'base_ang_vel',
                     'projected_gravity', 'joint_pos', 'joint_vel', 'actions'):
            raise ValueError('model does not implement the action17 raw observation ABI')
        self.legs = csv(metadata, 'joint_names', str)
        self.upper = csv(metadata, 'upper_joint_names', str)
        self.names = self.legs + self.upper
        if len(self.legs) != 12 or len(self.upper) != 18 or len(set(self.names)) != 30:
            raise ValueError('expected 12 leg and 18 distinct held-upper joints')
        if int(metadata['history_length']) != 6 or int(metadata['frame_dim']) != 82:
            raise ValueError('expected six time-major 82-wide raw frames')
        for key, expected in (('history_order', 'oldest_to_newest'),
                              ('history_reset', 'backfill_first_frame')):
            if key in metadata and metadata[key] != expected:
                raise ValueError(f'unsupported {key}')
        self.scale = csv(metadata, 'action_scale')
        self.default = csv(metadata, 'default_joint_pos') + csv(metadata, 'upper_default_pos')
        self.kp = csv(metadata, 'joint_stiffness') + csv(metadata, 'upper_joint_stiffness')
        self.kd = csv(metadata, 'joint_damping') + csv(metadata, 'upper_joint_damping')
        if len(self.scale) != 12 or any(len(x) != 30 for x in (self.default, self.kp, self.kd)):
            raise ValueError('metadata arrays do not match their named joint axis')
        finite(self.scale + self.default + self.kp + self.kd)
        self.contract = contract
        self.period = 1.0 / contract['inference_frequency_hz']
        self.action_clip = float(metadata.get('clip_actions', 'inf'))
        # Match master literally: joint_target_* is NOT consumed as a fallback.
        lower, upper = 'joint_pos_limit_lower', 'joint_pos_limit_upper'
        if (lower in metadata) != (upper in metadata):
            raise ValueError('model position-limit pair is incomplete')
        self.limits = None
        if lower in metadata:
            lo, hi = csv(metadata, lower), csv(metadata, upper)
            if len(lo) != 12 or len(hi) != 12 or any(a > b for a, b in zip(lo, hi)):
                raise ValueError('invalid metadata leg limits')
            finite(lo + hi)
            self.limits = tuple(zip(lo, hi))
        self.previous_actions = (0.0,) * 12
        self.history: deque[tuple[float, ...]] = deque(maxlen=6)

    def roster(self, body: Any) -> dict[str, int]:
        names = tuple(body.joint_names)
        if body.slot_count != 31 or len(names) != 31 or len(set(names)) != 31:
            raise ValueError('get_robot_info must provide 31 unique public joint names')
        if set(names) != set(self.names) | {'waist_roll_joint'}:
            raise ValueError('get_robot_info roster differs from model plus unpopulated roll')
        if len(body.joints) != 31:
            raise ValueError('get_robot_info capability count differs from roster')
        indices = {name: i for i, name in enumerate(names)}
        for name in self.names:
            if not body.joints[indices[name]].populated:
                raise ValueError(f'policy joint is unpopulated: {name}')
        return indices

    def observation(self, state: Any, body: Any, command: Sequence[float], hip: Any) -> tuple[float, ...]:
        indices = self.roster(body)
        positions, velocities, errors = joint_columns(state)
        if len(positions) != 31:
            raise ValueError('BodyState must contain the 31 public joints')
        if hasattr(state, 'imu_values'):
            samples = [i for i, location in enumerate(state.imu_location) if location == hip]
            if len(samples) != 1 or not state.imu_state_present[samples[0]]:
                raise ValueError('BodyState lacks one unambiguous HIP IMU sample')
            i = samples[0]
            if not state.attitude_valid or state.imu_status[i] != BodySampleStatus.OK:
                raise ValueError('HIP attitude/gyro feedback is invalid')
            quat = tuple(float(v) for v in state.imu_values[i, :4])
            gyro = tuple(float(v) for v in state.imu_values[i, 4:7])
        else:
            samples = [sample for sample in state.imu_states if sample.location == hip]
            if len(samples) != 1 or samples[0].state is None:
                raise ValueError('BodyState lacks one unambiguous HIP IMU sample')
            sample = samples[0]
            if not state.attitude_valid or sample.status != BodySampleStatus.OK:
                raise ValueError('HIP attitude/gyro feedback is invalid')
            quat, gyro = tuple(sample.state.quaternion), tuple(sample.state.gyroscope)
        if len(quat) != 4 or len(gyro) != 3 or len(command) != 4:
            raise ValueError('invalid quaternion/gyro/velocity-height dimensions')
        q, dq = [], []
        for name in self.names:
            i = indices[name]
            if errors[i] != JointError.NONE:
                raise ValueError(f'policy joint feedback is invalid: {name}')
            q.append(float(positions[i]))
            dq.append(float(velocities[i]))
        finite(tuple(command) + quat + gyro + tuple(q) + tuple(dq))
        # R^T [0,0,-1], exact MuJoCo-style quaternion rotation from vt_human_obs.h.
        w, x, y, z = quat[0], -quat[1], -quat[2], -quat[3]
        tmp = (-y, x, -w)
        gravity = (2*(y*tmp[2]-z*tmp[1]), 2*(z*tmp[0]-x*tmp[2]),
                   -1 + 2*(x*tmp[1]-y*tmp[0]))
        frame = tuple(f32(x) for x in (*command, *gyro, *gravity,
                                      *(a-b for a, b in zip(q, self.default)),
                                      *dq, *self.previous_actions))
        if not self.history:
            self.history.extend([frame] * 6)
        else:
            self.history.append(frame)
        return tuple(x for frame in self.history for x in frame)

    def targets(self, actions: Sequence[float], body: Any) -> tuple[MotorTarget, ...]:
        indices = self.roster(body)
        if len(actions) != 12:
            raise ValueError('ONNX output must contain twelve leg actions')
        finite(actions)
        # Runner output and clipped feedback are float32, decode then uses double.
        self.previous_actions = tuple(f32(clamp(f32(x), -self.action_clip, self.action_clip)) for x in actions)
        targets = [MotorTarget() for _ in range(31)]
        for p, name in enumerate(self.names):
            q = self.default[p]
            if p < 12:
                q += self.previous_actions[p] * self.scale[p]
                if self.limits is not None:
                    q = clamp(q, *self.limits[p])
            else:
                # No upper stream/replay in this example: hold metadata training pose.
                q = clamp(q, *self.contract['joint_limits'][name])
            kp = self.contract['joint_kp'].get(name, self.kp[p])
            kd = self.contract['joint_kd'].get(name, self.kd[p])
            targets[indices[name]] = MotorTarget(q=q, kp=kp, kd=kd)
        # Unpopulated roll stays placeholder; D2 codec/server populated mask skips dispatch.
        # Do not add a physical range/zero/finite gate for that slot.
        return tuple(targets)


class JoyCommand:
    def __init__(self, contract: dict):
        self.config = contract['command']
        self.dt = 1.0 / contract['inference_frequency_hz']
        self.velocity = [0.0, 0.0, 0.0]
        self.height: float | None = None

    def step(self, axes: Sequence[float]) -> tuple[float, ...]:
        finite(axes)
        cfg = self.config
        for n, key in enumerate(('vx', 'vy', 'wz')):
            i = cfg['joy_axis_'+key]
            v = axes[i] if 0 <= i < len(axes) else 0.0
            lo, hi = cfg[key+'_range']
            target = 0.0 if abs(v) < cfg['deadband'] else v*(hi if v >= 0 else -lo)
            target = clamp(target, lo, hi)
            s = cfg['smoothing']
            old = self.velocity[n]
            if s['enabled']:
                decel = abs(target) < abs(old) or target*old < 0
                axis = 'wz' if key == 'wz' else 'xy'
                limit = s[('decel' if decel else 'accel')+'_limit_'+axis] * self.dt
                self.velocity[n] = old + clamp(s['lpf_alpha']*(target-old), -limit, limit)
            else:
                self.velocity[n] = target
        i = cfg['joy_axis_height']
        lo, hi = cfg['height_range']
        target = cfg['height_default']
        if 0 <= i < len(axes):
            target = lo + (axes[i]*0.5+0.5)*(hi-lo)
        target = clamp(target, lo, hi)
        s = cfg['height_smoothing']
        if self.height is None or not s['enabled']:
            self.height = target
        else:
            rate = s['down_rate_m_s'] if target < self.height else s['up_rate_m_s']
            self.height += clamp(s['lpf_alpha']*(target-self.height), -rate*self.dt, rate*self.dt)
        return (*self.velocity, self.height)


class Model:
    def __init__(self, path: str):
        import platform
        import sys
        if (sys.version_info[:2] != (3, 10) or sys.implementation.name != 'cpython'
                or sys.platform != 'linux' or platform.machine() not in ('x86_64', 'aarch64')):
            raise RuntimeError('teleop-demo requires CPython 3.10 on Linux x86_64/aarch64 '
                               '(Ubuntu 22.04 or newer)')
        from locomotion_aorta._onnx import Session  # optional; no SDK core import
        self.session = Session(path, intra_threads=1, inter_threads=1)
        inputs, outputs = self.session.inputs, self.session.outputs
        if len(inputs) != 1 or len(outputs) != 1 or inputs[0].shape != [1,492] or outputs[0].shape != [1,12]:
            raise ValueError('expected one [1,492] float input and one [1,12] output')
        if inputs[0].dtype != 'tensor(float)' or outputs[0].dtype != 'tensor(float)':
            raise ValueError('expected float32 tensors')
        self.input_name, self.output_name = inputs[0].name, outputs[0].name
        self.metadata = self.session.metadata

    def infer(self, observation: Sequence[float]) -> tuple[float, ...]:
        import numpy as np
        values = np.asarray(observation, dtype=np.float32).reshape(1,492)
        result = self.session.run(self.input_name, self.output_name, values)
        return tuple(float(x) for x in result[0])


def bodycmd(api: Any, targets: Sequence[MotorTarget], state: Any) -> Any:
    return api.BodyCmd(joints=tuple(api.BodyJointCmd(q=m.q,dq=m.dq,tau=m.tau,kp=m.kp,kd=m.kd) for m in targets),
                      action=api.BodyCmdAction.EXTERNAL,
                      input_state_id=state.state_id,
                      input_state_ts_real_us=state.ts_state_real_us)


def live(controller: Any, api: Any, policy: Policy, model: Any,
         axes: Callable[[], Sequence[float]], stop: threading.Event, timeout: float,
         sleep: Callable = time.sleep, monotonic: Callable = time.monotonic,
         duration: float = 0.0, recorder=None, input_statistics=None) -> None:
    reply = controller.get_robot_info(timeout=timeout)
    if reply.status not in (api.ServiceStatus.SUCCESS, api.ServiceStatus.PARTIAL_SUCCESS):
        raise RuntimeError(f'get_robot_info failed: status={reply.status!r}; '
                           f'error_message={reply.error_message!r}')
    if reply.info is None or reply.info.body is None:
        raise RuntimeError('get_robot_info did not return real body capabilities')
    body = reply.info.body
    policy.roster(body)
    initial = controller.wait_state(timeout=timeout)
    command = JoyCommand(policy.contract)
    # Validate the real observation before requesting ownership; don't consume history yet.
    policy.observation(initial, body, command.step(()), api.BodyImuLocation.HIP)
    policy.history.clear()
    last_state_id = initial.state_id
    attempted = False
    try:
        if stop.is_set():
            return
        attempted = True
        entered = controller.enter_external(timeout=timeout)
        if entered.phase != api.ExternalPhase.RUNNING or entered.current_action != 'EXTERNAL':
            raise RuntimeError('PREPARE/ACK/COMMIT did not reach actual EXTERNAL RUNNING')
        started = deadline = monotonic()
        cycles, previous_publish_ns = 0, None
        while not stop.is_set() and (duration == 0 or monotonic()-started < duration):
            status = controller.status
            if status is None or any(getattr(status,key) != getattr(entered,key)
                                     for key in ('phase','generation','client_id','lease_id','current_action','reject_count')):
                raise RuntimeError('EXTERNAL ownership/status changed')
            # wait_state may return its cache. Reuse --timeout to wait for
            # a strictly newer input; never infer/publish again from that cache.
            state_deadline = monotonic() + timeout
            while not stop.is_set():
                remaining = state_deadline - monotonic()
                if remaining <= 0:
                    raise TimeoutError('BodyState did not advance within --timeout')
                state = controller.wait_state(timeout=remaining)
                if state.state_id > last_state_id:
                    break
                remaining = state_deadline - monotonic()
                if remaining <= 0:
                    raise TimeoutError('BodyState did not advance within --timeout')
                sleep(min(policy.period, remaining))
            if stop.is_set():
                break
            last_state_id = state.state_id
            cycles += 1
            row = dict(cycle=cycles, state_id=state.state_id,
                       input_state_id=state.state_id,
                       input_state_ts_real_us=state.ts_state_real_us,
                       S2=time.time_ns(), S3=NOT_MEASURED, S4=NOT_MEASURED,
                       S5=NOT_MEASURED, S6=NOT_MEASURED,
                       session_id=NOT_MEASURED, sequence=NOT_MEASURED,
                       publish_result='not_called', **state_log_fields(state))
            try:
                joy_axes = axes()
                row['joy_axes_used'] = list(joy_axes)
                if input_statistics is not None:
                    row.update(input_statistics())
                velocity_height = command.step(joy_axes)
                row['velocity_height_command'] = list(velocity_height)
                observation = policy.observation(state,body,velocity_height,api.BodyImuLocation.HIP)
                row['S3'] = time.time_ns()
                actions = model.infer(observation)
                row['S4'] = time.time_ns()
                targets = policy.targets(actions,body)
                if stop.is_set():
                    break
                row['target_q'] = [m.q for m in targets]
                # Existing controller guards still validate the session/command.
                msg = bodycmd(api,targets,state)
                row['S5'] = time.time_ns()
                publish_mono = round(monotonic()*1e9)
                row['control_interval_ns'] = (publish_mono-previous_publish_ns
                    if previous_publish_ns is not None else NOT_MEASURED)
                previous_publish_ns = publish_mono
                try:
                    sent = controller.publish(msg)
                    row['publish_result'] = 'returned'
                    publish_log_fields(row,sent)
                except BaseException:
                    row['publish_result'] = 'raised'
                    raise
                finally:
                    row['S6'] = time.time_ns()
                    timing = getattr(controller,'_last_publish_timing',lambda:None)()
                    if timing is not None:
                        row['publish_timing'] = timing
            except BaseException as error:
                row['error'] = f'{type(error).__name__}: {error}'
                raise
            finally:
                if recorder is not None:
                    if sys.exc_info()[0] is not None:
                        try:
                            recorder.cycle(row)
                        except Exception:
                            pass  # Keep the original control failure if logging also fails.
                    else:
                        recorder.cycle(row)
            deadline = max(deadline+policy.period, monotonic())
            sleep(max(0.0,deadline-monotonic()))
    finally:
        if attempted:
            exited = controller.exit_external(timeout=timeout)
            if exited.current_action != 'PASSIVE':
                raise RuntimeError('CANCEL exit has not confirmed actual PASSIVE')


def objects(value: Any) -> Any:
    if isinstance(value,dict):
        return SimpleNamespace(**{k:objects(v) for k,v in value.items()})
    if isinstance(value,list):
        return tuple(objects(v) for v in value)
    return value


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',required=True,help='path to separately delivered action17 model')
    parser.add_argument('--contract',type=Path,default=Path(__file__).with_name('action17_contract.json'))
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--body-state-jsonl',type=Path,help='offline capture: first line body info, then BodyState records with axes')
    parser.add_argument('--group',default='default')
    parser.add_argument('--timeout',type=float,default=5.0,help='existing SDK control/discovery wait timeout in seconds')
    parser.add_argument('--duration',type=float,default=0.,help='bounded live seconds; 0 keeps the original until-interrupted behavior')
    parser.add_argument('--log',type=Path,help='new S0-S6/publish/applied diagnostic JSONL, never overwritten')
    parser.add_argument('--confirm-motion',action='store_true',help='existing SDK demo motion acknowledgement; does not replace on-site authorization')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    if args.dry_run and args.body_state_jsonl is None:
        parser.error('--dry-run requires --body-state-jsonl; no transport is opened')
    if not args.dry_run and not args.confirm_motion:
        parser.error('--confirm-motion is required for live control')
    model = Model(args.model)
    policy = Policy(model.metadata,json.loads(args.contract.read_text()))
    if args.dry_run:
        command = JoyCommand(policy.contract)
        with args.body_state_jsonl.open() as stream:
            body = objects(json.loads(next(stream))['body'])
            for n,line in enumerate(stream):
                record = json.loads(line)
                state = objects(record['state'])
                # Offline capture uses symbolic locations; real D2 uses its enum.
                obs = policy.observation(state,body,command.step(record['axes']),'HIP')
                targets = policy.targets(model.infer(obs),body)
                print(json.dumps({'frame':n,'state_id':state.state_id,'observation_dim':len(obs),
                                  'command_slots':len(targets),'q':[m.q for m in targets],
                                  'physical_command_names':list(policy.names)}))
        return
    import locomotion_aorta as api  # D2 public binding; no legacy LowCmd fallback
    if __package__:
        from .velocity_mode import DirectionInput
    else:
        from velocity_mode import DirectionInput
    stop = threading.Event()
    old_handlers = {sig:signal.signal(sig,lambda signum,frame:stop.set())
                    for sig in (signal.SIGINT,signal.SIGTERM)}
    log = None
    try:
        if args.log:
            if __package__:
                from .velocity_mode_latency import CycleLog
            else:
                from velocity_mode_latency import CycleLog
            log = CycleLog(args.log,dict(
                demo='teleop_policy', input='aorta locomotion/joy via JoyReader',
                period_s=policy.period, model=str(Path(args.model).resolve()),
                model_sha256=hashlib.sha256(Path(args.model).read_bytes()).hexdigest(),
                clock='CLOCK_REALTIME', source_stamp='S1 SDK recv return; dequeue stamp is not arrival'))
        with api.PullController(group=args.group) as controller:
            with closing(DirectionInput(controller.transport)) as directions:
                live(controller,api,policy,model,directions.snapshot,stop,args.timeout,
                     duration=args.duration,recorder=log,input_statistics=directions.statistics)
    finally:
        if log is not None:
            log.close()
        for sig,handler in old_handlers.items():
            signal.signal(sig,handler)


if __name__ == '__main__':
    main()
