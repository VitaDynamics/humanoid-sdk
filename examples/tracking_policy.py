#!/usr/bin/env python3
"""actor_g1 tracking through legacy42 EXTERNAL; defaults to offline inference.

Live mode requests EXTERNAL ownership and moves the body. Exit sends CANCEL
and must confirm PASSIVE, which can let loaded limbs fall. Installation and
--confirm-motion do not replace authorization for a specific robot and trial.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time


class ReferenceSession:
    """Advance one 20 ms reference frame per successful inference, as C++."""
    def __init__(self, policy, measured, motion=None, stand_seconds=3.):
        from locomotion_aorta.tracking_policy import Motion, yaw
        self.policy, self.motion = policy, motion
        self.stand_ticks = math.ceil(stand_seconds/policy.contract.period)
        q, _, quat, _ = policy.state(measured)
        self.reference = Motion.hold(policy.contract.default).lead_in(q, quat, policy.profile['lead_in'])
        self.offset = yaw(quat)
        self.frame, self.tick = 0, 0
        self.playing = False

    def observation(self, state):
        from locomotion_aorta.tracking_policy import REF_DT, yaw
        q, dq, quat, gyro = self.policy.state(state)
        if self.motion is not None and not self.playing and self.tick >= self.stand_ticks:
            i = min(self.frame, len(self.reference.q)-1)
            self.reference = self.motion.lead_in(self.reference.q[i], self.reference.quat[i], self.policy.profile['lead_in'])
            self.offset = yaw(quat)
            self.frame, self.playing = 0, True
        obs = self.policy.observe(self.reference, self.frame*REF_DT, q, dq, quat, gyro, self.offset)
        self.frame = min(self.frame+1, len(self.reference.q)-1)
        self.tick += 1
        return obs


def synthetic_state(policy):
    from locomotion_aorta import HumanoidLowState, MotorState, ImuSample
    positions = [0.]*44
    for p, i in enumerate(policy.mapping):
        positions[i] = float(policy.contract.default[p])
    return HumanoidLowState(1, 1_000_000_000, tuple(MotorState(q=q) for q in positions),
                            imu_states=(ImuSample(0, 1, (1., 0., 0., 0.), (0., 0., 0.)),))


def replay_states(path, steps, period):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    from locomotion_aorta import HumanoidLowState, MotorState, ImuSample
    next_source_ns, previous_ns, emitted = None, 0, 0
    with open(path, 'rb') as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        for _, _, message, ros in reader.iter_decoded_messages(topics=['/rt/foot_humanoid/lowstate']):
            source_ns = int(ros.ts_state_real)*1000
            if source_ns <= previous_ns:
                continue
            previous_ns = source_ns
            if next_source_ns is not None and source_ns < next_source_ns:
                continue
            if next_source_ns is None:
                next_source_ns = source_ns
            # Sample a fixed 50 Hz grid; avoid accumulating feedback quantization.
            while next_source_ns <= source_ns:
                next_source_ns += round(period*1e9)
            motors = tuple(MotorState(x.q, x.dq, x.tau_est, x.temperature) for x in ros.motor_state)
            imus = tuple(ImuSample(x.location, x.status, tuple(x.state.quaternion),
                                  tuple(x.state.gyroscope), tuple(x.state.accelerometer),
                                  tuple(x.state.rpy), x.ts_sampled_us, x.ts_acquired_hw_us,
                                  x.ts_ingested_us, x.ts_assembled_us) for x in ros.imu_states)
            yield HumanoidLowState(ros.state_id, source_ns, motors, ros.attitude_valid,
                                   imus, ros.ts_state_pub, ros.ts_consumed_us), message.log_time
            emitted += 1
            if emitted >= steps:
                return
    if not emitted:
        raise ValueError('MCAP contains no usable /rt/foot_humanoid/lowstate samples')


class Recorder:
    def __init__(self, path, header):
        # Refuse overwrites so a failed trial does not erase the previous evidence.
        self.file = Path(path).open('x', buffering=1024*1024)
        self.write(dict(kind='header', **header))

    def write(self, row):
        self.file.write(json.dumps(row, allow_nan=False, separators=(',', ':'))+'\n')

    def close(self):
        self.file.close()


def infer_step(model, policy, session, state, cmd_id, *, record_time=0):
    acquired_mono = time.monotonic_ns()
    acquired_wall = time.time_ns()
    obs = session.observation(state)
    before_mono, before_wall = time.monotonic_ns(), time.time_ns()
    action = model.infer(obs)
    after_mono, after_wall = time.monotonic_ns(), time.time_ns()
    command, decoded = policy.decode(
        action, cmd_id, input_state_id=state.state_id,
        input_state_ts_real=state.stamp_ns//1000,
        ts_infer_input_state=acquired_wall//1000,
        ts_before_infer=before_wall//1000, ts_after_infer=after_wall//1000,
        max_abs_obs=float(abs(obs).max()))
    row = dict(kind='step', cmd_id=cmd_id, input_state_id=state.state_id,
               input_state_ts_real_us=state.stamp_ns//1000,
               robot_state_pub_us=state.ts_state_pub_us,
               received_monotonic_ns=state.received_monotonic_ns,
               acquired_monotonic_ns=acquired_mono, before_infer_monotonic_ns=before_mono,
               after_infer_monotonic_ns=after_mono,
               acquired_wall_ns=acquired_wall, before_infer_wall_ns=before_wall,
               after_infer_wall_ns=after_wall,
               cache_age_ms=((acquired_mono-state.received_monotonic_ns)/1e6
                             if state.received_monotonic_ns else None),
               infer_ms=(after_mono-before_mono)/1e6,
               observation_ms=(before_mono-acquired_mono)/1e6,
               record_log_time_ns=record_time, reference_frame=session.frame,
               reference_source='clip' if session.playing else 'stand',
               observation=obs.tolist(), decoded=decoded)
    # Capture real input evidence: no inference-input/state-ID placeholders.
    q, dq, quat, gyro = policy.state(state)
    row['state'] = dict(q=q.tolist(), dq=dq.tolist(), quaternion=quat.tolist(), gyro=gyro.tolist())
    return command, row


def offline(model, policy, args, recorder):
    from locomotion_aorta.tracking_policy import Motion
    clip = Motion.from_npz(args.motion, policy.contract.names) if args.motion else None
    if args.replay_mcap:
        states = replay_states(args.replay_mcap, args.steps, policy.contract.period)
    else:
        state = synthetic_state(policy)
        from dataclasses import replace
        states = ((replace(state, state_id=(i+1) % 65536,
                           stamp_ns=state.stamp_ns+round(i*policy.contract.period*1e9)), 0)
                  for i in range(args.steps))
    session, count, costs = None, 0, []
    for state, stamp in states:
        if session is None:
            session = ReferenceSession(policy, state, clip, args.stand_seconds)
        _, row = infer_step(model, policy, session, state, count % 65535+1, record_time=stamp)
        recorder.write(row)
        costs.append(row['infer_ms'])
        count += 1
    return dict(mode='recorded_feedback_replay' if args.replay_mcap else 'synthetic_offline',
                steps=count, infer_ms_mean=sum(costs)/len(costs), infer_ms_max=max(costs),
                stability='not_measured')


def wait_fresh(controller, policy, *, previous_ns, timeout, max_age,
               stop, monotonic=time.monotonic, sleep=time.sleep):
    deadline = monotonic()+timeout
    while not stop.is_set():
        remaining = deadline-monotonic()
        if remaining <= 0:
            raise TimeoutError('feedback did not advance before the freshness deadline')
        state = controller.wait_lowstate(timeout=remaining)
        if state.stamp_ns > previous_ns:
            if not state.received_monotonic_ns:
                raise ValueError('SDK must provide feedback receipt timestamps')
            age_ns = round(monotonic()*1e9)-state.received_monotonic_ns
            if not 0 <= age_ns <= max_age*1e9:
                raise TimeoutError('feedback callback cache is stale or has a future timestamp')
            policy.state(state)
            return state
        sleep(min(.001, remaining))
    return None


def live(model, policy, args, recorder, controller, *,
         stop=None, monotonic=time.monotonic, sleep=time.sleep):
    from locomotion_aorta import ExternalPhase
    from locomotion_aorta.tracking_policy import Motion
    stop = stop if stop is not None else threading.Event()
    clip = Motion.from_npz(args.motion, policy.contract.names) if args.motion else None
    initial = wait_fresh(controller, policy, previous_ns=0, timeout=args.timeout,
                         max_age=args.max_state_age_ms/1000, stop=stop,
                         monotonic=monotonic, sleep=sleep)
    if initial is None:
        return dict(mode='live', steps=0)
    # Warm ONNX before taking ownership, then erase synthetic inference history.
    warm = ReferenceSession(policy, initial)
    model.infer(warm.observation(initial))
    policy.reset()
    attempted, steps, misses = False, 0, 0
    try:
        if stop.is_set():
            return dict(mode='live', steps=0)
        attempted = True
        entered = controller.enter_external(timeout=args.timeout)
        if entered.current_action != 'EXTERNAL' or entered.phase != ExternalPhase.RUNNING:
            raise RuntimeError('entry did not confirm EXTERNAL/RUNNING')
        initial = wait_fresh(controller, policy, previous_ns=initial.stamp_ns,
                             timeout=args.max_state_age_ms/1000,
                             max_age=args.max_state_age_ms/1000, stop=stop,
                             monotonic=monotonic, sleep=sleep)
        if initial is None:
            return dict(mode='live', steps=0)
        session = ReferenceSession(policy, initial, clip, args.stand_seconds)
        previous_ns = 0
        start = deadline = monotonic()
        while not stop.is_set() and monotonic()-start < args.duration:
            status = controller.status
            if status is None or any(getattr(status, k) != getattr(entered, k) for k in (
                    'current_action', 'phase', 'generation', 'client_id', 'lease_id',
                    'reject_count', 'ingress_drop_count')) or status.timeout_hold:
                raise RuntimeError('EXTERNAL ownership, rejection/drop counters or running state changed')
            state = wait_fresh(controller, policy, previous_ns=previous_ns,
                               timeout=args.max_state_age_ms/1000,
                               max_age=args.max_state_age_ms/1000, stop=stop,
                               monotonic=monotonic, sleep=sleep)
            if state is None:
                break
            previous_ns = state.stamp_ns
            command, row = infer_step(model, policy, session, state, steps % 65535+1)
            if stop.is_set():
                break
            if time.monotonic_ns()-row['acquired_monotonic_ns'] > args.max_compute_ms*1e6:
                recorder.write(dict(row, published=False))
                raise TimeoutError('observation plus inference exceeded --max-compute-ms')
            # Recheck ownership after inference, before the transport's own guard.
            status = controller.status
            if status is None or any(getattr(status, k) != getattr(entered, k) for k in (
                    'current_action', 'phase', 'generation', 'client_id', 'lease_id',
                    'reject_count', 'ingress_drop_count')) or status.timeout_hold:
                raise RuntimeError('EXTERNAL state changed during inference')
            row['before_publish_monotonic_ns'] = time.monotonic_ns()
            controller.publish(command)
            row['after_publish_monotonic_ns'] = time.monotonic_ns()
            row['published'] = True
            steps += 1
            next_deadline = deadline+policy.contract.period
            now = monotonic()
            row['deadline_miss_ms'] = max(0., now-next_deadline)*1000
            misses += int(now > next_deadline)
            recorder.write(row)
            # Never send a burst to catch up after an overrun.
            deadline = next_deadline if next_deadline > now else now+policy.contract.period
            sleep(max(0., deadline-monotonic()))
        return dict(mode='live', steps=steps, overruns=misses)
    finally:
        if attempted:
            original_error = sys.exc_info()[1]
            try:
                result = controller.exit_external(timeout=args.timeout)
                recorder.write(dict(kind='exit', current_action=result.current_action,
                                    phase=result.phase.value))
                if result.current_action != 'PASSIVE':
                    raise RuntimeError('CANCEL has not confirmed PASSIVE')
            except BaseException as error:
                recorder.write(dict(kind='exit_failure', error_type=type(error).__name__, message=str(error)))
                print(f'CANCEL/exit incomplete: {error}; inspect actual robot state.', file=sys.stderr, flush=True)
                if original_error is None:
                    raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--profile', type=Path, default=Path(__file__).with_name('groot_tracking_asample.json'))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--dry-run', action='store_true', help='synthetic default-pose feedback; no transport')
    mode.add_argument('--replay-mcap', type=Path, help='offline ROS lowstate replay; no transport')
    mode.add_argument('--live', action='store_true', help='request EXTERNAL and run the body policy')
    parser.add_argument('--confirm-motion', action='store_true')
    parser.add_argument('--motion', type=Path, help='optional named-joint NPZ; default is balanced stand reference')
    parser.add_argument('--stand-seconds', type=float, default=3., help='policy-on stand before optional clip')
    parser.add_argument('--steps', type=int, default=100, help='maximum offline inference steps')
    parser.add_argument('--duration', type=float, default=30., help='bounded live run, seconds')
    parser.add_argument('--threads', type=int, default=1)
    parser.add_argument('--group', default='default')
    parser.add_argument('--timeout', type=float, default=5., help='session entry/exit deadline, seconds')
    parser.add_argument('--max-state-age-ms', type=float, default=50., help='local callback age/new-feedback deadline')
    parser.add_argument('--max-compute-ms', type=float, default=50.)
    parser.add_argument('--log', type=Path, required=True, help='new JSONL evidence file; never overwritten')
    args = parser.parse_args()
    if args.live and not args.confirm_motion:
        parser.error('--live requires --confirm-motion and separately authorized on-site conditions')
    if args.steps <= 0 or args.threads <= 0 or not math.isfinite(args.stand_seconds) or args.stand_seconds < 0:
        parser.error('steps/threads must be positive; stand-seconds must be finite and nonnegative')
    if any(not math.isfinite(x) or x <= 0 for x in (
            args.duration, args.timeout, args.max_state_age_ms, args.max_compute_ms)):
        parser.error('durations/deadlines must be finite and positive')
    from locomotion_aorta.tracking_policy import Model, TrackingPolicy
    model = Model(args.model, args.threads)
    profile = json.loads(args.profile.read_text())
    policy = TrackingPolicy(model.contract, profile)
    header = dict(format='vita.groot_tracking.v1', model=str(args.model.resolve()),
                  model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
                  profile=profile, model_contract=json.loads(model.session.get_modelmeta().custom_metadata_map[
                      'groot_wbc.tracking_contract.v1']),
                  input_clock='robot Vita host microseconds copied unchanged',
                  inference_wire_clock='client CLOCK_REALTIME microseconds; cross-host sync not assumed',
                  local_latency_clock='client CLOCK_MONOTONIC nanoseconds',
                  args={k:str(v) if isinstance(v, Path) else v for k,v in vars(args).items()})
    recorder = Recorder(args.log, header)
    try:
        if args.live:
            import os
            if not os.environ.get('ZENOH_SESSION_CONFIG_URI'):
                raise ValueError('set the deployment-owned PC ZENOH_SESSION_CONFIG_URI before --live')
            from locomotion_aorta import ExternalController
            print('EXTERNAL requires robot virtual_pd strategy and ankle_virtual_pd_intent=true. '
                  'This program does not change robot configuration. Exit CANCEL confirms PASSIVE.', flush=True)
            stop = threading.Event()
            previous = {sig:signal.signal(sig, lambda *_:stop.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
            try:
                with ExternalController(group=args.group) as controller:
                    summary = live(model, policy, args, recorder, controller, stop=stop)
            finally:
                for sig, handler in previous.items():
                    signal.signal(sig, handler)
        else:
            summary = offline(model, policy, args, recorder)
        recorder.write(dict(kind='summary', **summary))
        print(json.dumps(summary, allow_nan=False))
    except BaseException as error:
        recorder.write(dict(kind='failure', error_type=type(error).__name__, message=str(error)))
        raise
    finally:
        recorder.close()


if __name__ == '__main__':
    main()
