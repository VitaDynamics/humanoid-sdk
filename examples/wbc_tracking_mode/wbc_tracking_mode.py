#!/usr/bin/env python3
"""WBC tracking mode through public body31 EXTERNAL; defaults to offline inference.

Live mode requests EXTERNAL ownership and moves the body. Exit sends CANCEL
and must confirm PASSIVE, which can let loaded limbs fall. Installation and
--confirm-motion do not replace authorization for a specific robot and trial.
"""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from policy_support import (state_log_fields, publish_log_fields,
                            feedback_age_ns, NOT_MEASURED)


class ReferenceSession:
    """Advance one 20 ms reference frame per successful inference, as C++."""
    def __init__(self, policy, measured, motion=None, stand_seconds=3.):
        from locomotion_aorta.wbc_tracking_policy import Motion, yaw
        self.policy, self.motion = policy, motion
        self.stand_ticks = math.ceil(stand_seconds/policy.contract.period)
        q, _, quat, _ = policy.state(measured)
        self.reference = Motion.hold(policy.contract.default).lead_in(q, quat, policy.profile['lead_in'])
        self.offset = yaw(quat)
        self.frame, self.tick = 0, 0
        self.playing = False

    def observation(self, state):
        from locomotion_aorta.wbc_tracking_policy import REF_DT, yaw
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
    from locomotion_aorta import BodyJointState, BodyImuSample, BodyImuState
    from locomotion_aorta._policy_state import PolicyFeedback
    positions = [0.] * 31
    for p, i in enumerate(policy.mapping):
        positions[i] = float(policy.contract.default[p])
    return PolicyFeedback(1, tuple(BodyJointState(q=q) for q in positions),
                          ts_state_real_us=1_000_000, attitude_valid=True,
                          imu_states=(BodyImuSample(0, BodyImuState((1., 0., 0., 0.),
                                      (0., 0., 0.), (0., 0., 9.8), (0., 0., 0.)), 1),))


def replay_states(path, steps, period):
    from mcap.reader import make_reader
    from lowlevel.BodyState import BodyState
    from locomotion_aorta import BODY_STATE_TOPIC
    from locomotion_aorta._body_codec import decode_body_state
    from locomotion_aorta._policy_state import PolicyFeedback
    from locomotion_aorta.brainco_capture import binding_schema_hash
    from dataclasses import fields
    next_source_ns, previous_ns, emitted = None, 0, 0
    with open(path, 'rb') as stream:
        for schema, channel, message in make_reader(stream).iter_messages(topics=[BODY_STATE_TOPIC]):
            if (schema is None or schema.name != 'lowlevel.BodyState'
                    or schema.encoding != 'flatbuffer' or channel.message_encoding != 'flatbuffer'
                    or hashlib.sha256(schema.data).hexdigest() != binding_schema_hash('body_state')):
                raise ValueError('MCAP BodyState must match the installed public schema/BFBS')
            body = decode_body_state(BodyState.GetRootAs(message.data))
            source_ns = body.ts_state_real_us * 1000
            if source_ns <= previous_ns:
                continue
            previous_ns = source_ns
            if next_source_ns is not None and source_ns < next_source_ns:
                continue
            if next_source_ns is None:
                next_source_ns = source_ns
            while next_source_ns <= source_ns:
                next_source_ns += round(period*1e9)
            # Offline: host receipt clocks were not recorded; never synthesize them.
            yield PolicyFeedback(**{f.name: getattr(body, f.name) for f in fields(type(body))}), message.log_time
            emitted += 1
            if emitted >= steps:
                return
    if not emitted:
        raise ValueError('MCAP contains no usable public BodyState samples')


class Recorder:
    def __init__(self, path, header, *, log_observation=False, capacity=256):
        # Refuse overwrites so a failed trial does not erase the previous evidence.
        if not isinstance(capacity, int) or isinstance(capacity, bool) or capacity <= 0:
            raise ValueError('log queue capacity must be a positive integer')
        self.file = Path(path).open('x', buffering=1024*1024)
        self.log_observation = log_observation
        self.capacity = capacity
        self._condition = threading.Condition()
        self._rows = deque()
        self._closing = False
        self._summary = None
        self._error = None
        self._submitted = self._enqueued = self._written = self._dropped = self._max_depth = 0
        self._thread = threading.Thread(target=self._run, name='wbc-jsonl-writer', daemon=True)
        self._thread.start()
        self.write(dict(kind='header', **header))

    def write(self, row):
        with self._condition:
            if self._closing:
                raise RuntimeError('recorder is closed')
            self._submitted += 1
            if row.get('kind') == 'summary':
                # Keep the final summary out of the lossy control-loop queue.
                self._summary = dict(row)
                return True
            if self._error is not None or len(self._rows) >= self.capacity:
                self._dropped += 1
                return False
            self._rows.append(dict(row))
            self._enqueued += 1
            self._max_depth = max(self._max_depth, len(self._rows))
            self._condition.notify()
            return True

    def statistics(self):
        with self._condition:
            return dict(records_submitted=self._submitted, records_enqueued=self._enqueued,
                        records_written=self._written, records_dropped=self._dropped,
                        queue_max_depth=self._max_depth, queue_capacity=self.capacity,
                        writer_error=str(self._error) if self._error else None)

    def _format(self, row):
        row = dict(row)
        if 'observation' in row:
            import numpy as np
            observation = np.asarray(row.pop('observation'), dtype='<f4').reshape(-1)
            row.update(observation_length=int(observation.size),
                       observation_sha256=hashlib.sha256(observation.tobytes()).hexdigest(),
                       observation_encoding='little-endian float32, C order')
            if self.log_observation:
                row['observation'] = observation.tolist()
        # Array materialization and JSON formatting run only on the writer thread.
        return json.dumps(row, allow_nan=False, separators=(',', ':'),
                          default=lambda value: value.tolist())+'\n'

    def _failed(self, error, *, current_dropped=False):
        with self._condition:
            if self._error is None:
                self._error = error
            self._dropped += len(self._rows)+int(current_dropped)
            self._rows.clear()

    def _run(self):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._rows or self._closing)
                    if not self._rows:
                        break
                    row = self._rows.popleft()
                try:
                    self.file.write(self._format(row))
                    with self._condition:
                        self._written += 1
                except Exception as error:
                    self._failed(error, current_dropped=True)
            if self._error is None:
                summary = dict(self._summary or dict(kind='logging_summary'))
                counts = self.statistics()
                counts['records_written'] += 1  # Includes this final summary on success.
                summary['logging'] = counts
                self.file.write(self._format(summary))
                with self._condition:
                    self._written += 1
                self.file.flush()
        except Exception as error:
            self._failed(error)
        finally:
            try:
                self.file.close()
            except Exception as error:
                self._failed(error)

    def close(self):
        # Called after the live finally block has attempted CANCEL/PASSIVE.
        with self._condition:
            self._closing = True
            self._condition.notify()
        self._thread.join()
        if self._error is not None:
            raise OSError('WBC log writer failed: '+json.dumps(self.statistics())) from self._error


def finish_recording(recorder):
    original_error = sys.exc_info()[1]
    try:
        recorder.close()
    except Exception as error:
        print(str(error), file=sys.stderr, flush=True)
        if original_error is None:
            raise


def infer_step(model, policy, session, state, cycle, *, record_time=0):
    acquired_mono = time.monotonic_ns()
    acquired_wall = time.time_ns()
    obs = session.observation(state)
    before_mono, before_wall = time.monotonic_ns(), time.time_ns()
    action = model.infer(obs)
    after_mono, after_wall = time.monotonic_ns(), time.time_ns()
    command, decoded = policy.decode(action, input_state_id=state.state_id,
                                      input_state_ts_real_us=state.ts_state_real_us)
    row = dict(kind='step', cycle=cycle, input_state_id=state.state_id,
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
               observation=obs.copy(), decoded=decoded,
               max_abs_obs=float(abs(obs).max()),
               applied_session_id=state.applied_session_id,
               applied_sequence=state.applied_sequence,
               ts_first_output_us=state.ts_first_output_us,
               joint_error_codes=(state.error_code.tolist() if hasattr(state, "error_code")
                                  else [j.error_code for j in state.joints]))
    row.update(state_log_fields(state))
    row.update(S2=acquired_wall,S3=before_wall,S4=after_wall,S5=NOT_MEASURED,S6=NOT_MEASURED,publish_result='not_called')
    # Capture real input evidence: no inference-input/state-ID placeholders.
    q, dq, quat, gyro = policy.state(state)
    row['state'] = dict(q=q.copy(), dq=dq.copy(), quaternion=quat.copy(), gyro=gyro.copy())
    return command, row


def offline(model, policy, args, recorder):
    from locomotion_aorta.wbc_tracking_policy import Motion
    clip = Motion.from_npz(args.motion, policy.contract.names) if args.motion else None
    if args.replay_mcap:
        states = replay_states(args.replay_mcap, args.steps, policy.contract.period)
    else:
        state = synthetic_state(policy)
        from dataclasses import replace
        states = ((replace(state, state_id=i+1,
                           ts_state_real_us=state.ts_state_real_us+round(i*policy.contract.period*1e6)), 0)
                  for i in range(args.steps))
    session, count, costs = None, 0, []
    for state, stamp in states:
        if session is None:
            session = ReferenceSession(policy, state, clip, args.stand_seconds)
        _, row = infer_step(model, policy, session, state, count+1, record_time=stamp)
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
        state = controller.wait_state(timeout=remaining)
        if state.stamp_ns > previous_ns:
            age_ns = feedback_age_ns(state, round(monotonic()*1e9))
            if not 0 <= age_ns <= max_age*1e9:
                raise TimeoutError('feedback cache is stale or has a future timestamp')
            policy.state(state)
            return state
        sleep(min(.001, remaining))
    return None


def live(model, policy, args, recorder, controller, *,
         source, stop=None, monotonic=time.monotonic, sleep=time.sleep):
    from locomotion_aorta import ExternalPhase
    from locomotion_aorta.wbc_tracking_policy import Motion
    stop = stop if stop is not None else threading.Event()
    clip = Motion.from_npz(args.motion, policy.contract.names) if args.motion else None
    initial = wait_fresh(source, policy, previous_ns=0, timeout=args.timeout,
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
        initial = wait_fresh(source, policy, previous_ns=initial.stamp_ns,
                             timeout=args.max_state_age_ms/1000,
                             max_age=args.max_state_age_ms/1000, stop=stop,
                             monotonic=monotonic, sleep=sleep)
        if initial is None:
            return dict(mode='live', steps=0)
        session = ReferenceSession(policy, initial, clip, args.stand_seconds)
        previous_ns = 0
        start = deadline = monotonic()
        previous_publish_ns = None
        while not stop.is_set() and monotonic()-start < args.duration:
            status = controller.status
            if status is None or any(getattr(status, k) != getattr(entered, k) for k in (
                    'current_action', 'phase', 'generation', 'client_id', 'control_id',
                    'reject_count', 'ingress_drop_count')) or status.timeout_hold:
                raise RuntimeError('EXTERNAL ownership, rejection/drop counters or running state changed')
            state = wait_fresh(source, policy, previous_ns=previous_ns,
                               timeout=args.max_state_age_ms/1000,
                               max_age=args.max_state_age_ms/1000, stop=stop,
                               monotonic=monotonic, sleep=sleep)
            if state is None:
                break
            previous_ns = state.stamp_ns
            command, row = infer_step(model, policy, session, state, steps+1)
            if stop.is_set():
                break
            if time.monotonic_ns()-row['acquired_monotonic_ns'] > args.max_compute_ms*1e6:
                recorder.write(dict(row, published=False))
                raise TimeoutError('observation plus inference exceeded --max-compute-ms')
            # Recheck ownership after inference, before the transport's own guard.
            status = controller.status
            if status is None or any(getattr(status, k) != getattr(entered, k) for k in (
                    'current_action', 'phase', 'generation', 'client_id', 'control_id',
                    'reject_count', 'ingress_drop_count')) or status.timeout_hold:
                raise RuntimeError('EXTERNAL state changed during inference')
            row['before_publish_monotonic_ns'] = time.monotonic_ns()
            row['S5'] = time.time_ns()
            publish_mono = round(monotonic()*1e9)
            row['control_interval_ns'] = (publish_mono-previous_publish_ns if previous_publish_ns is not None else NOT_MEASURED)
            previous_publish_ns = publish_mono
            try:
                sent = controller.publish(command)
            except BaseException:
                row['S6'] = time.time_ns()
                row['publish_result'] = 'raised'
                try:
                    recorder.write(dict(row, published=False,
                                        publish_timing=getattr(controller, 'publish_timing', None)))
                except Exception:
                    pass  # Preserve the original publish failure if diagnostics also fail.
                raise
            timing = getattr(controller, 'publish_timing', None)
            if timing is not None:
                row['publish_timing'] = timing
            row['S6'] = time.time_ns()
            row['publish_result'] = 'returned'
            publish_log_fields(row,sent,timing)
            row.update(session_id=sent.session_id, sequence=sent.sequence)
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
    parser.add_argument('--profile', type=Path, default=Path(__file__).with_name('wbc_tracking_asample.json'))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--dry-run', action='store_true', help='synthetic default-pose feedback; no transport')
    mode.add_argument('--replay-mcap', type=Path, help='offline public BodyState FlatBuffer MCAP replay; no transport')
    mode.add_argument('--live', action='store_true', help='request EXTERNAL and run the body policy')
    parser.add_argument('--confirm-motion', action='store_true')
    parser.add_argument('--motion', type=Path, help='optional named-joint NPZ; default is balanced stand reference')
    parser.add_argument('--stand-seconds', type=float, default=3., help='policy-on stand before optional clip')
    parser.add_argument('--steps', type=int, default=100, help='maximum offline inference steps')
    parser.add_argument('--duration', type=float, default=30., help='bounded live run, seconds')
    parser.add_argument('--threads', type=int, default=1)
    parser.add_argument('--group', default='default')
    parser.add_argument('--timeout', type=float, default=5., help='session entry/exit deadline, seconds')
    parser.add_argument('--max-state-age-ms', type=float, default=50.,
                        help='local receipt age and new-feedback deadline')
    parser.add_argument('--max-compute-ms', type=float, default=50.)
    parser.add_argument('--log', type=Path, required=True, help='new JSONL evidence file; never overwritten')
    parser.add_argument('--log-observation', action='store_true',
                        help='include full observation; default logs float32 length/SHA256 only')
    args = parser.parse_args()
    if args.live and not args.confirm_motion:
        parser.error('--live requires --confirm-motion and separately authorized on-site conditions')
    if args.steps <= 0 or args.threads <= 0 or not math.isfinite(args.stand_seconds) or args.stand_seconds < 0:
        parser.error('steps/threads must be positive; stand-seconds must be finite and nonnegative')
    if any(not math.isfinite(x) or x <= 0 for x in (
            args.duration, args.timeout, args.max_state_age_ms, args.max_compute_ms)):
        parser.error('durations/deadlines must be finite and positive')
    from locomotion_aorta.wbc_tracking_policy import Model, WbcTrackingPolicy
    model = Model(args.model, args.threads)
    profile = json.loads(args.profile.read_text())
    policy = WbcTrackingPolicy(model.contract, profile)
    header = dict(format='vita.wbc_tracking.v1', model=str(args.model.resolve()),
                  model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
                  profile=profile, model_contract=json.loads(model.session.metadata[
                      'groot_wbc.tracking_contract.v1']),
                  input_clock='robot Vita host microseconds copied unchanged',
                  inference_wire_clock='ts_client_created_us/ts_client_send_us: client CLOCK_MONOTONIC microseconds; cross-host sync not assumed',
                  local_latency_clock='client CLOCK_MONOTONIC nanoseconds',
                  args={k:str(v) if isinstance(v, Path) else v for k,v in vars(args).items()})
    recorder = Recorder(args.log, header, log_observation=args.log_observation)
    try:
        if args.live:
            import os
            if not os.environ.get('ZENOH_SESSION_CONFIG_URI'):
                raise ValueError('set the deployment-owned PC ZENOH_SESSION_CONFIG_URI before --live')
            from contextlib import closing
            from locomotion_aorta import PullController, PullStateSource
            print('EXTERNAL requires robot virtual_pd strategy and ankle_virtual_pd_intent=true. '
                  'This program does not change robot configuration. Exit CANCEL confirms PASSIVE.', flush=True)
            stop = threading.Event()
            previous = {sig:signal.signal(sig, lambda *_:stop.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
            try:
                with PullController(group=args.group) as controller, closing(PullStateSource(group=args.group)) as source:
                    response = controller.get_robot_info(args.timeout)
                    if response.info is None or response.info.body is None:
                        raise RuntimeError(response.error_message or 'RobotInfo has no body roster')
                    source_info = source.get_robot_info(args.timeout)
                    if source_info.runtime_id != response.runtime_id:
                        raise RuntimeError('BodyState source and controller have different robot runtimes')
                    policy.bind_body_info(response.info.body)
                    recorder.write(dict(kind='roster', names=policy.order,
                                        roster_version=response.info.body.roster_version,
                                        populated=[j.populated for j in response.info.body.joints],
                                        is_mock=[j.is_mock for j in response.info.body.joints]))
                    summary = live(model, policy, args, recorder, controller, stop=stop,
                                   source=source)
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
        finish_recording(recorder)


if __name__ == '__main__':
    main()
