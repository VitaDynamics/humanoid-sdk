#!/usr/bin/env python3
"""One shared EXTERNAL owner for Body and generic end effectors; read-only by default."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack, contextmanager
from dataclasses import replace
import importlib.util
import math
import signal
from pathlib import Path
import time

from locomotion_aorta import PullController, PullStateSource
from locomotion_aorta.end_effector import EndEffectorClient
from locomotion_aorta import _end_effector_codec as codec
from locomotion_aorta.end_effector_types import (
    ControlMode, DeviceModel, EndEffectorTarget, Field, InhibitReason,
    PositionDomain, SampleStatus, SampleStream,
)

# The existing SDK body example remains the authority for its unchanged guards.
_body_spec = importlib.util.spec_from_file_location(
    'ee_body_hold_baseline',
    Path(__file__).resolve().parents[1] / 'external_control/external_control.py')
body = importlib.util.module_from_spec(_body_spec)
_body_spec.loader.exec_module(body)


@contextmanager
def stop_signals():
    """Same signal policy as the hand demo: interrupt once and allow CANCEL."""
    stopped = False
    previous = {}

    def stop(signum, _frame):
        nonlocal stopped
        if not stopped:
            stopped = True
            raise KeyboardInterrupt(f'received {signal.Signals(signum).name}')

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, stop)
        yield
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def _observe_zero_dwell(frame, target, *, body, body_info, monotonic, emit):
    """Record the existing example dwell without a feedback-arrival gate."""
    began = monotonic()
    peak_error = 0.0
    peak_speed = 0.0
    samples = math.ceil(body.DWELL_S / body.PERIOD_S)
    for sample in range(samples):
        state = frame(target)  # Retains the lifecycle's safety/protocol checks.
        error = max((abs(actual - q) for actual, q, cap in zip(state.q, target, body_info.joints)
                     if cap.populated), default=0.0)
        speed = max((abs(dq) for dq, cap in zip(state.dq, body_info.joints) if cap.populated), default=0.0)
        peak_error = max(peak_error, error)
        peak_speed = max(peak_speed, speed)
        emit(f'ZERO_DWELL observation_only=true sample={sample + 1} '
             f'elapsed_s={monotonic() - began:.6f} '
             f'max_abs_q_error_rad={error:.6f} max_abs_dq_rad_s={speed:.6f}')
    emit(f'ZERO_DWELL complete observation_only=true samples={samples} '
         f'requested_dwell_s={body.DWELL_S} elapsed_s={monotonic() - began:.6f} '
         f'peak_abs_q_error_rad={peak_error:.6f} peak_abs_dq_rad_s={peak_speed:.6f}')


@stop_signals()
def run_external(controller, command_loop, *, body, timeout=5.0, body_gains='hold',
                 body_pose='measured', body_enabled=True, strict_hold=False,
                 sleep=time.sleep, monotonic=time.monotonic, emit=print):
    if isinstance(timeout, bool) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('timeout must be finite and positive')
    if body_gains not in ('hold', 'zero'):
        raise ValueError('body_gains must be hold or zero')
    if body_pose not in ('measured', 'zero'):
        raise ValueError('body_pose must be measured or zero')
    if body_pose == 'zero' and body_gains != 'hold':
        raise ValueError('zero pose requires frozen FIXED_STAND gains; do not select zero gains')
    if not body_enabled and body_pose != 'measured':
        raise ValueError('EE-only cannot select a Body zero trajectory')
    info = plan = initial = hold = None
    pose = ()
    if body_enabled:
        info = body.require_body_info(controller, timeout)
        plan = None
        if body_pose == 'zero':
            spec = importlib.util.spec_from_file_location(
                'ee_fixed_stand_pose', Path(__file__).resolve().parents[1] / 'fixed_stand_pose/fixed_stand_pose.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            plan = module.FixedStandPose(info)  # Reject all bad/missing zero limits before PREPARE.
        initial = body._wait_for_stationary_body_state(
            controller, body_info=info, timeout=timeout, sleep=sleep, monotonic=monotonic)
        pose = tuple(float(q) if cap.populated else 0.0 for q, cap in zip(initial.q, info.joints))
        if plan is not None:
            plan.measured(initial)
        hold = body._position_command(pose)
        if body_gains == 'zero':
            hold = replace(hold, joints=tuple(replace(j, kp=0.0, kd=0.0, dq=0.0, tau=0.0)
                                              for j in hold.joints))
    attempted = False
    active = False
    traces = []
    hold_samples = 0
    hold_peak_abs_error = [0.0] * len(pose)
    hold_slots = tuple(i for i, cap in enumerate(info.joints) if cap.populated) if info else ()
    try:
        if plan is not None:
            from locomotion_aorta import CONTROL_ACK_TOPIC, CONTROL_STATUS_TOPIC
            for topic in (CONTROL_ACK_TOPIC, CONTROL_STATUS_TOPIC):
                traces.append(controller.transport.subscribe(
                    topic, lambda message, topic=topic: emit(f'{topic}: {message!r}')))
        attempted = True
        entered = controller.enter_external(timeout)
        active = True
        if plan is not None:
            # Use the latest real sample after the owned RUNNING transition,
            # never the model pose or a pre-entry commanded pose.
            initial = controller.wait_state(timeout)
            body._validate_body_state(initial, body_info=info, require_stationary=False)
            pose = plan.measured(initial)
        last_id = initial.state_id if initial is not None else None
        last_change = monotonic()
        next_publish = monotonic()
        check_hold = plan is None
        stage = 'hold'

        def frame(target=None, *, wait=True):
            nonlocal last_id, last_change, next_publish, hold_samples
            if not active:
                raise RuntimeError('EXTERNAL frame callback is no longer active')
            delay = next_publish - monotonic()
            if wait and delay > 0:
                sleep(delay)
            status = controller.status
            if (status is None or status.current_action != 'EXTERNAL'
                    or status.phase not in ((body.ExternalPhase.RUNNING,) if body_enabled else
                                           (body.ExternalPhase.RUNNING, body.ExternalPhase.TIMEOUT_HOLD))
                    or (status.generation, status.client_id, status.lease_id) !=
                       (entered.generation, entered.client_id, entered.lease_id)
                    or status.reject_count != entered.reject_count):
                raise RuntimeError(f'body EXTERNAL ownership/phase/reject_count changed: {status!r}')
            if not body_enabled:
                if monotonic() >= next_publish:
                    next_publish = monotonic() + body.PERIOD_S
                return None
            state = controller.wait_state(timeout)
            body._validate_body_state(state, body_info=info, require_stationary=False)
            now = monotonic()
            if state.state_id != last_id:
                last_id, last_change = state.state_id, now
            elif now - last_change >= body.LOWSTATE_WINDOW_S:
                raise RuntimeError('body lowstate became stale')
            if check_hold:
                errors = tuple(float(state.q[i]) - pose[i] for i in hold_slots)
                for i, error in zip(hold_slots, errors):
                    hold_peak_abs_error[i] = max(hold_peak_abs_error[i], abs(error))
                if strict_hold and any(abs(e) > body.RETURN_ENDPOINT_POSITION_TOLERANCE_RAD for e in errors):
                    raise RuntimeError('body departed from its held pose')
                hold_samples += 1
                emit(f'BODY_HOLD observation_only=true sample={hold_samples} '
                     f'slots={hold_slots!r} q_error_rad={errors!r} '
                     f'max_abs_q_error_rad={max(map(abs, errors), default=0.0):.6f}')
            if not wait and now < next_publish:
                return state
            if plan is None:
                command = replace(hold, input_state_id=state.state_id,
                                  input_state_ts_real_us=state.ts_state_real_us)
            else:
                plan.measured(state)
                q = pose if target is None else target(now) if callable(target) else target
                command = plan.command(q, state)
            published = controller.publish(command)
            if plan is not None:
                emit(f'BODY stage={stage} session_id={published.session_id} sequence={published.sequence} input_state={state.state_id} '
                     f'q_cmd={tuple(j.q for j in command.joints)!r} '
                     f'control={status!r} applied={state.applied_session_id}/{state.applied_sequence}@{state.ts_first_output_us}')
            next_publish = max(next_publish + body.PERIOD_S, monotonic() + body.PERIOD_S)
            return state

        # Read the existing Body deadline; the EE loop must not restart it on
        # each endpoint wakeup. Existing single-callback callers stay unchanged.
        frame.next_deadline = lambda: next_publish
        if plan is not None:
            start = pose
            duration = plan.duration(start)
            emit(f'FIXED_STAND source={plan.profile["robot_source_head"]} '
                 f'zero range preflight passed; linear duration={duration}s; '
                 f'PC publish period={body.PERIOD_S}s, real-time interpolation; no hard realtime guarantee')
            stage = 'ramp'
            frame(start)
            began = monotonic()
            while monotonic() - began < duration:
                frame(lambda now: plan.position(start, now - began, duration))
            pose = (0.0,) * len(start)
            frame(pose)
            stage = 'settle-zero'
            _observe_zero_dwell(frame, pose, body=body, body_info=info, monotonic=monotonic, emit=emit)
            check_hold = True
            stage = 'hold-zero'
            emit('zero endpoint settled check disabled (observation only); '
                 'holding EXTERNAL in the same session before end-effector commands')
        command_loop(frame)
    finally:
        active = False
        try:
            if hold_samples:
                emit(f'BODY_HOLD summary observation_only=true samples={hold_samples} '
                     f'slots={hold_slots!r} '
                     f'peak_abs_q_error_rad={tuple(hold_peak_abs_error[i] for i in hold_slots)!r} '
                     f'max_abs_q_error_rad={max(hold_peak_abs_error, default=0.0):.6f}')
        finally:
            try:
                if attempted:
                    for signum in (signal.SIGINT, signal.SIGTERM):
                        signal.signal(signum, signal.SIG_IGN)
                    try:
                        status = controller.exit_external(timeout)
                        if status.current_action != 'PASSIVE':
                            raise RuntimeError(f'CANCEL returned {status.current_action}, expected PASSIVE')
                    except BaseException as error:
                        emit(f'CANCEL UNCONFIRMED: {error}; use the on-site emergency procedure')
                        raise
                    emit('CANCEL confirmed PASSIVE; no further end-effector targets; body may fall without support')
            finally:
                for trace in traces:
                    trace.close()


def validate_position(info, state):
    codec.validate_info(info)
    if info.device_model not in (DeviceModel.AGX, DeviceModel.LINKER_L6, DeviceModel.BRAINCO_REVO2):
        raise ValueError('this demo requires actual AGX/Linker/BrainCo capability; OP3 is not enabled')
    if state is None:
        raise ValueError('end-effector feedback is absent')
    codec.validate_state(info, state)
    if state.sample_status != SampleStatus.OK or any(
            not mask & Field.POS or not math.isfinite(pos) or not lo <= pos <= hi
            for mask, pos, lo, hi in zip(state.valid_fields, state.pos, info.pos_min, info.pos_max)):
        raise ValueError('end-effector measured position is unavailable or outside proven limits')
    if not all(mask & Field.POS for mask in info.valid_limits):
        raise ValueError('position limits are unavailable')


def validate_status(info, state, *, require_output=False):
    if state is None:
        raise ValueError('end-effector feedback is absent')
    codec.validate_state(info, state)
    if state.status_valid_mask & 4 and state.inhibit_reason in (
            InhibitReason.EMERGENCY, InhibitReason.DOMAIN_FAULT, InhibitReason.NOT_READY):
        raise RuntimeError(f'end-effector inhibited: {state.inhibit_reason}')
    if any(mask & Field.ERROR_CODE and code for mask, code in zip(state.valid_fields, state.error_code)):
        raise RuntimeError('end-effector reports a valid nonzero error code')
    if require_output and not output_ready(state):
        raise RuntimeError('end-effector output was inhibited or permission is unavailable')


def motion_feedback(client, info, moving, monotonic):
    latest, position, fault, received_at = client.motion_feedback()
    if latest is not None:
        validate_status(info, latest, require_output=moving)
    if fault is not None:
        codec.validate_state(info, fault)
        if any(mask & Field.ERROR_CODE and code for mask, code in zip(fault.valid_fields, fault.error_code)):
            raise RuntimeError('end-effector reports a valid nonzero error code')
    if position is not None:
        # Status/admission belong to latest; these checks use only this original
        # position sample's measurement and source validity.
        validate_position(info, position)
        if received_at is None or not 0 <= monotonic() - received_at < body.LOWSTATE_WINDOW_S:
            raise RuntimeError('end-effector position became stale')
    return latest, position


def wait_position(client, info, timeout, *, sleep, monotonic):
    deadline = monotonic() + timeout
    while True:
        _, position = motion_feedback(client, info, False, monotonic)
        if position is not None:
            return position
        remaining = deadline - monotonic()
        if remaining <= 0:
            raise TimeoutError('end-effector measured position was not observed')
        sleep(min(.01, remaining))


def output_ready(state):
    return (state.status_valid_mask & 7 == 7 and state.ready and
            state.output_allowed and state.inhibit_reason == InhibitReason.NONE)


def position_target(info, positions, *, unit, vel_limit=None, torque=None):
    """Use declared SI domains and capability columns; never clip or guess force."""
    codec.validate_info(info)
    expected = PositionDomain.TOTAL_OPENING if unit == 'm' else PositionDomain.JOINT_ANGLE if unit == 'rad' else None
    if expected is None or any(domain != expected for domain in info.position_domain):
        raise ValueError('explicit position unit differs from actual capability domain')
    n = len(info.axis_names)
    if vel_limit is None and any(mask & Field.VEL_LIMIT for mask in info.supported_command_fields):
        # Keep the old angular demo's documented20deg/s default as float32.
        import struct
        speed = 0.0 if unit == 'm' else struct.unpack('<f', struct.pack('<f', math.radians(20.0)))[0]
        vel_limit = (speed,) * n
    modes = tuple(ControlMode.POSITION_TORQUE if torque is not None or
                  not support & (1 << ControlMode.POSITION) else ControlMode.POSITION
                  for support in info.supported_modes)
    fields = (Field.POS | (Field.VEL_LIMIT if vel_limit is not None else 0) |
              (Field.TAU if torque is not None else 0),) * n
    return codec.prepare_target(info, EndEffectorTarget(modes, fields, pos=tuple(positions),
        vel_limit=None if vel_limit is None else tuple(vel_limit),
        tau_cmd=None if torque is None else tuple(torque)))


def print_feedback(client, emit=print):
    state = client.state
    if state is None:
        emit(f'{client.side}: no EndEffectorState')
        return
    emit(f'{client.side}: ref={state.descriptor_ref} sample={state.state_id} '
         f'pos={state.pos} vel={state.vel} tau={state.tau} current_A={state.current} '
         f'valid={state.valid_fields} status_valid={state.status_valid_mask} '
         f'output={state.output_allowed} inhibit={state.inhibit_reason} '
         f'received={state.received_key} host_target={state.host_target_key} key_valid={state.key_valid_mask} '
         f'actual_output={state.actual_output_key} actual_valid={state.actual_output_key_valid} '
         f'feedback_applied={state.feedback_applied_key} feedback_target={state.feedback_applied_target} '
         f'progress_complete={state.progress_complete} diagnostic_valid={state.diagnostic_valid_mask} '
         f'reject={state.reject_count}/{state.last_reject_code} drop={state.ingress_drop_count} '
         f'timeout={state.timeout_count} crc={state.crc_error_count} raw_pos={state.raw_pos} '
         f'raw_current={state.raw_current} raw_status={state.raw_status} raw_valid={state.raw_valid_fields}')
    for entry in state.progress:
        stamps = (entry.ts_received_us, entry.ts_accepted_us, entry.ts_first_output_us,
                  entry.ts_write_ack_us, entry.ts_first_feedback_us)
        deltas = tuple(b-a if entry.stage_valid_mask & (3 << i) == (3 << i) and a <= b
                       else 'NOT_MEASURED' for i, (a, b) in enumerate(zip(stamps, stamps[1:])))
        emit(f'{client.side}: key={entry.key} stages_us={stamps} stage_valid={entry.stage_valid_mask} '
             f'adjacent_deltas_us={deltas}; no SDK clock subtraction or physical-arrival inference')


def observe(clients, *, timeout, emit=print):
    for client in clients:
        info = client.wait_info(timeout)
        state = client.wait_state(timeout)
        if state.descriptor_ref != info.descriptor_ref:
            raise ValueError('State belongs to another descriptor')
        emit(f'{client.side}: Info={info}; read-only, no lease/cmd/set_touch')
        print_feedback(client, emit)
        if client.touch is not None:
            emit(f'{client.side}: Touch={client.touch}')


def set_touch_and_observe(client, enabled, timeout, *, monotonic=time.monotonic, sleep=time.sleep):
    """Confirm via new State/Touch within the remaining observation budget.

    The existing synchronous set_touch API separately bounds matching and RPC;
    it may exceed this observation deadline. No strict total-call deadline or
    hard thread cancellation is promised here.
    """
    before_state, before_touch = client.state, client.touch
    if before_state is None:
        raise ValueError('touch change requires an actual State snapshot')
    deadline = monotonic() + timeout
    if not before_state.time_valid_mask & 2 or not before_state.ts_state_pub_us:
        raise ValueError('touch confirmation requires a valid State publication time')
    client.set_touch(enabled, timeout=timeout)
    while True:
        remaining = deadline - monotonic()
        if remaining <= 0:
            raise TimeoutError('touch switch not confirmed by a new State')
        state = client.state
        if state is None:
            raise ValueError('State disappeared while confirming touch switch')
        if state.descriptor_ref != before_state.descriptor_ref:
            raise ValueError('descriptor changed while confirming touch switch')
        if (state.time_valid_mask & 2 and state.ts_state_pub_us > before_state.ts_state_pub_us
                and state.status_valid_mask & 8 and state.touch_enabled == enabled):
            break
        sleep(min(.01, remaining))
    if not enabled:
        return state, None  # Disabling cannot require a sample that must stop arriving.
    previous = None if before_touch is None else before_touch.sample_ref
    while True:
        remaining = deadline - monotonic()
        if remaining <= 0:
            raise TimeoutError('touch enabled but no new valid Touch was observed')
        touch = client.wait_touch(remaining, after_sample=previous)
        if touch.descriptor_ref != state.descriptor_ref:
            raise ValueError('Touch belongs to another descriptor')
        previous = touch.sample_ref
        if touch.touch_state_id and touch.region_stamps and all(
                region.validity & 1 and not region.validity & 64 for region in touch.region_stamps):
            return state, touch


class _EndpointGuard:
    def __init__(self, client, info, timeout, monotonic):
        self.client, self.info = client, info
        self.timeout, self.monotonic = timeout, monotonic
        self.latest = client.motion_feedback()[0]
        self.rejects = (self.latest.reject_count if self.latest is not None and
                        self.latest.diagnostic_valid_mask & 4 else None)
        self.submitted = None
        self.echoed = None
        self.progress_time = monotonic()

    def feedback(self, moving):
        state, position = motion_feedback(self.client, self.info, moving, self.monotonic)
        if state is None or position is None:
            raise ValueError('end-effector measured position is unavailable')
        self.latest = state
        now = self.monotonic()
        if state.diagnostic_valid_mask & 4:
            if self.rejects is not None and state.reject_count != self.rejects:
                raise RuntimeError('end-effector reject_count changed')
            self.rejects = state.reject_count
        if self.submitted is not None:
            accepted = state.host_target_key if state.key_valid_mask & 2 else None
            if accepted is not None and accepted.session_id == self.submitted.key.session_id:
                if accepted.sequence > self.submitted.key.sequence:
                    raise RuntimeError('end-effector echoed an unsent sequence')
                if self.echoed is None or accepted.sequence > self.echoed:
                    self.echoed, self.progress_time = accepted.sequence, now
            if not self.admitted(state) and now - self.progress_time >= self.timeout:
                raise TimeoutError('end-effector input admission did not advance')
        return position

    def admitted(self, state):
        return self.submitted is None or (state.key_valid_mask & 2 and state.host_target_key == self.submitted.key)

    def publish(self, target, state):
        if self.submitted is None:
            self.progress_time = self.monotonic()
        self.submitted = self.client.publish(target, input_observations=(state.sample_ref,))
        return self.submitted


def _transition_duration(start, target, rate, minimum=0.0):
    delta = abs(target-start)
    if delta == 0:
        return minimum
    if rate <= 0:
        raise ValueError('explicit positive speed is required for a finite transition duration')
    return max(minimum, delta/rate)


def run(controller, clients, targets, *, timeout=5.0, duration=1.0,
        rates=None, body_enabled=True, body_gains='hold', body_pose='measured',
        return_to_start=False, touch='keep', feedback_interval=1.0,
        pattern='target', period=10.0, sine_min=.01, sine_max=.08,
        low_frac=.25, high_frac=.75, loop=False,
        sleep=time.sleep, monotonic=time.monotonic, emit=print):
    """Independent member deadlines; targets are intent, never arrival evidence."""
    if pattern not in ('target', 'sine', 'simultaneous', 'sequential'):
        raise ValueError('unknown motion pattern')
    if loop and pattern not in ('simultaneous', 'sequential'):
        raise ValueError('loop applies only to angular fraction patterns')
    if (not math.isfinite(period) or period <= 0 or not math.isfinite(sine_min)
            or not math.isfinite(sine_max) or sine_min >= sine_max
            or not math.isfinite(low_frac) or not math.isfinite(high_frac)
            or not 0 <= low_frac < high_frac <= 1):
        raise ValueError('invalid period, sine bounds or fractions')
    if pattern == 'sine' and tuple(c.side for c in clients) != ('left', 'right'):
        raise ValueError('sine requires left and right AGX endpoints')
    if pattern != 'target' and not clients:
        raise ValueError('pattern requires end effectors')
    if touch not in ('keep', 'on', 'off'):
        raise ValueError('touch must be keep/on/off')
    if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0
           for v in (timeout, duration, feedback_interval)):
        raise ValueError('timeout/duration/feedback interval must be finite and positive')
    if not clients and not body_enabled:
        raise ValueError('none requires Body; EE-only requires a selected endpoint')
    if len({c.side for c in clients}) != len(clients) or set(targets) != {c.side for c in clients}:
        raise ValueError('exactly one target and client per selected endpoint required')
    if any(c.connection is not controller for c in clients):
        raise ValueError('all end effectors must borrow this one controller')
    rates = {c.side: 50.0 for c in clients} if rates is None else rates
    if set(rates) != set(targets) or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in rates.values()):
        raise ValueError('one finite positive command rate per endpoint required')
    infos, prepared, returns, limits = {}, {}, {}, {}
    for client in clients:
        info = client.wait_info(timeout)
        codec.validate_info(info)
        if info.device_model not in (DeviceModel.AGX, DeviceModel.LINKER_L6, DeviceModel.BRAINCO_REVO2):
            raise ValueError('this demo requires actual AGX/Linker/BrainCo capability; OP3 is not enabled')
        state = wait_position(client, info, timeout, sleep=sleep, monotonic=monotonic)
        if client.observation_sources != ((SampleStream.EE_STATE, info.side),):
            raise ValueError('demo requires its actual endpoint State as fixed K=1 input')
        if touch != 'keep' and not info.touch_available:
            raise ValueError('selected endpoint has no touch capability')
        infos[client.side] = info
        prepared[client.side] = codec.prepare_target(info, targets[client.side])
        returns[client.side] = codec.prepare_target(info, replace(targets[client.side], pos=state.pos))
        if pattern == 'sine' and (info.device_model != DeviceModel.AGX or len(info.axis_names) != 1
                                 or info.position_domain != (PositionDomain.TOTAL_OPENING,)):
            raise ValueError('sine requires actual one-axis AGX total-opening capability')
        if pattern in ('simultaneous', 'sequential') and (
                info.device_model not in (DeviceModel.BRAINCO_REVO2, DeviceModel.LINKER_L6)
                or len(info.axis_names) != 6 or any(d != PositionDomain.JOINT_ANGLE for d in info.position_domain)):
            raise ValueError('fraction patterns require actual six-axis angular hand capability')
        lows = (sine_min,) if pattern == 'sine' else tuple(a+low_frac*(b-a) for a,b in zip(info.pos_min,info.pos_max))
        highs = (sine_max,) if pattern == 'sine' else tuple(a+high_frac*(b-a) for a,b in zip(info.pos_min,info.pos_max))
        if pattern != 'target':
            for points in (lows, highs):
                codec.prepare_target(info, replace(prepared[client.side], pos=points))
        # These are declared command slope limits, not a measured backend default
        # or evidence of physical arrival. Explicit zero intent remains zero.
        speed = tuple(v if v > 0 else maximum for v,maximum in
                      zip(prepared[client.side].vel_limit or (0.0,)*len(info.axis_names), info.vel_limit_max))
        if pattern != 'target' and (len(speed) != len(info.axis_names) or any(v <= 0 for v in speed)):
            raise ValueError('a proven positive speed limit is required for a trajectory')
        limits[client.side] = (lows, highs, speed)
        emit(f'{client.side}: trajectory slope limits={speed}; configured zero-speed default is not measured')
        if pattern == 'sine' and math.pi*(sine_max-sine_min)/period > speed[0]:
            raise ValueError('sine peak command slope exceeds the declared velocity limit')
    strict_hold = any(i.device_model in (DeviceModel.BRAINCO_REVO2, DeviceModel.LINKER_L6) for i in infos.values())
    if strict_hold and feedback_interval < 1.0:
        raise ValueError('hand feedback interval must be at least 1 second')

    def command_loop(frame):
        guards = {c.side: _EndpointGuard(c, infos[c.side], timeout, monotonic) for c in clients}
        deadline = monotonic() + timeout
        while True:
            frame(wait=False)
            for guard in guards.values():
                guard.feedback(False)
            if all(output_ready(g.latest) for g in guards.values()):
                break
            if monotonic() >= deadline:
                raise TimeoutError('end-effector output did not become ready after EXTERNAL entry')
            sleep(max(0.0, min(frame.next_deadline(), deadline)-monotonic()))
        if touch != 'keep':
            for client in clients:
                result = touch_worker.submit(set_touch_and_observe, client, touch == 'on',
                                             timeout, monotonic=monotonic)
                while not result.done():
                    frame(wait=False)  # Same owner/body watchdog continues during bounded RPC.
                    for guard in guards.values():
                        guard.feedback(True)
                    sleep(max(0.0, frame.next_deadline()-monotonic()))
                state, sample = result.result()
                emit(f'{client.side}: touch_state={state.sample_ref} touch_sample={None if sample is None else sample.sample_ref}')
        # Preparation and RPC may outlive the preflight measurement. Keep the
        # saved return target, but enter interpolation from current guarded data.
        positions = {side: guard.feedback(True).pos for side, guard in guards.items()}

        def transition(end, minimum=0.0):
            start = dict(positions)
            seconds = max([minimum, *(_transition_duration(a, b, rate)
                for side in end for a, b, rate in zip(start[side], end[side], limits[side][2]))])
            def targets_at(elapsed):
                alpha = 1.0 if seconds == 0 else min(1.0, elapsed/seconds)
                return {side: codec.prepare_target(infos[side], replace(prepared[side],
                    pos=tuple(a*(1-alpha)+b*alpha for a,b in zip(start[side], end[side])))) for side in end}
            positions.update(end)
            return targets_at, seconds

        def phases():
            if pattern == 'target':
                for selected in ((prepared, returns) if return_to_start else (prepared,)):
                    # Preserve explicit target intent, including zero/default speed;
                    # duration bounds streaming only, never measured arrival.
                    yield 'target', lambda _elapsed, values=selected: values, duration
                return
            if pattern == 'sine':
                middle = {side: ((sine_min+sine_max)/2,) for side in guards}
                target_at, seconds = transition(middle)
                yield 'entry', target_at, seconds
                def sinusoid(elapsed):
                    weight = (1+math.sin(2*math.pi*elapsed/period))/2
                    pos = (sine_min*(1-weight)+sine_max*weight,)
                    return {side: codec.prepare_target(infos[side], replace(prepared[side],pos=pos)) for side in guards}
                yield 'sine', sinusoid, duration
                positions.update({side: sinusoid(duration)[side].pos for side in guards})
                target_at, seconds = transition({side: value.pos for side,value in returns.items()})
                yield 'return', target_at, seconds
                return
            low = {side: limits[side][0] for side in guards}
            high = {side: limits[side][1] for side in guards}
            target_at, seconds = transition(low)
            yield 'low', target_at, seconds+body.DWELL_S
            while True:
                if pattern == 'simultaneous':
                    for label,end in (('high',high),('low',low)):
                        target_at, seconds = transition(end)
                        yield label, target_at, seconds+body.DWELL_S
                else:
                    for side in guards:
                        for index in range(len(infos[side].axis_names)):
                            end = dict(low); values=list(end[side]); values[index]=high[side][index]; end[side]=tuple(values)
                            target_at, seconds = transition(end)
                            yield side+'/'+infos[side].axis_names[index], target_at, seconds+body.DWELL_S
                            target_at, seconds = transition(low)
                            yield 'low', target_at, seconds+body.DWELL_S
                if not loop:
                    return

        next_due = {side: monotonic() for side in guards}
        for leg, target_at, seconds in phases():
            began = monotonic()
            deadline, next_print = began+seconds, began
            final_sent = set()
            while len(final_sent) != len(guards) or monotonic() < deadline:
                frame(wait=False)
                states = {side: g.feedback(True) for side, g in guards.items()}
                now = monotonic()
                selected = target_at(min(seconds, now-began))
                for side, guard in guards.items():
                    if now >= next_due[side]:
                        command = guard.publish(selected[side], states[side])
                        emit(f'EE leg={leg} side={side} submitted={command.key} observations={command.input_observations}; admission/arrival not inferred')
                        if now >= deadline:
                            final_sent.add(side)
                        next_due[side] = monotonic() + 1.0/rates[side]  # Never catch-up burst.
                if now >= next_print:
                    for client in clients:
                        print_feedback(client, emit)
                    next_print = now+feedback_interval
                if len(final_sent) != len(guards) or monotonic() < deadline:
                    sleep(max(0.0, min([frame.next_deadline(), *next_due.values(),
                        *([deadline] if now < deadline else [])])-monotonic()))
            final_deadline = monotonic()+timeout
            while True:
                frame(wait=False)
                for guard in guards.values():
                    guard.feedback(True)
                if all(g.admitted(g.latest) for g in guards.values()):
                    break
                if monotonic() >= final_deadline:
                    raise TimeoutError('final end-effector input admission was not observed')
                sleep(max(0.0, min(frame.next_deadline(), final_deadline)-monotonic()))
    touch_worker = ThreadPoolExecutor(max_workers=1) if touch != 'keep' else None
    try:
        run_external(controller, command_loop, body=body, timeout=timeout, body_enabled=body_enabled,
                     body_gains=body_gains, body_pose=body_pose, strict_hold=strict_hold,
                     sleep=sleep, monotonic=monotonic, emit=emit)
    finally:
        # run_external attempts CANCEL before waiting for a failed/inflight RPC.
        # The worker only performs one bounded RPC and observes samples; no motion
        # writer, second lease, retry or autonomous heartbeat exists here.
        if touch_worker is not None:
            touch_worker.shutdown(wait=True, cancel_futures=True)
    emit('Target streaming ended; physical completion and hold remain NOT_MEASURED')


def main(argv=None, *, _client_factory=None, _controller_factory=None, _state_source_factory=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('none', 'left', 'right', 'both'), default='both')
    parser.add_argument('--pattern', choices=('target', 'sine', 'simultaneous', 'sequential'), default='target')
    parser.add_argument('--period-s', type=float, default=10.0)
    parser.add_argument('--sine-min-m', type=float, default=.01)
    parser.add_argument('--sine-max-m', type=float, default=.08)
    parser.add_argument('--low-frac', type=float, default=.25)
    parser.add_argument('--high-frac', type=float, default=.75)
    parser.add_argument('--loop', action='store_true')
    parser.add_argument('--group', default='default')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--duration', type=float, default=1.0, help='streaming duration, not physical arrival')
    parser.add_argument('--feedback-interval', type=float, default=1.0)
    parser.add_argument('--ee-only', action='store_true', help='no implicit Body command or Body preflight')
    parser.add_argument('--body-gains', choices=('hold', 'zero'), default='hold')
    parser.add_argument('--body-pose', choices=('measured', 'zero'), default='measured')
    parser.add_argument('--return-to-start', action='store_true')
    parser.add_argument('--touch', choices=('keep', 'on', 'off'), default='keep')
    parser.add_argument('--dry-run', action='store_true', help='read-only; no lease, target or touch switch')
    parser.add_argument('--confirm-motion', action='store_true', help='requires separate current on-site authorization')
    for side in ('left', 'right'):
        parser.add_argument('--'+side+'-pos', type=float, nargs='+', help='all axes in declared fixed Info order')
        parser.add_argument('--'+side+'-unit', choices=('m', 'rad'))
        parser.add_argument('--'+side+'-vel-limit', type=float, nargs='+', help='nonnegative magnitude in m/s or rad/s; zero retains robot default')
        parser.add_argument('--'+side+'-torque-nm', type=float, nargs='+', help='declared torque domain; omit means no intent, explicit zero remains zero')
        parser.add_argument('--'+side+'-hz', type=float, default=50.0)
    args = parser.parse_args(argv)
    readonly = args.dry_run or not args.confirm_motion
    sides = () if args.side == 'none' else ('left', 'right') if args.side == 'both' else (args.side,)
    if args.pattern != 'target' and not sides:
        parser.error('pattern requires a selected endpoint')
    if args.pattern == 'sine' and args.side != 'both':
        parser.error('sine requires --side both')
    if args.loop and args.pattern not in ('simultaneous', 'sequential'):
        parser.error('--loop requires an angular fraction pattern')
    if not all(math.isfinite(v) for v in (args.period_s,args.sine_min_m,args.sine_max_m,args.low_frac,args.high_frac)) or args.period_s <= 0 or args.sine_min_m >= args.sine_max_m or not 0 <= args.low_frac < args.high_frac <= 1:
        parser.error('invalid pattern bounds/period')
    if args.dry_run and args.confirm_motion:
        parser.error('--dry-run and --confirm-motion are mutually exclusive')
    if args.ee_only and not sides:
        parser.error('--ee-only requires a selected endpoint')
    if args.ee_only and args.body_pose != 'measured':
        parser.error('--ee-only cannot request a Body zero trajectory')
    if args.body_pose == 'zero' and args.body_gains != 'hold':
        parser.error('--body-pose zero requires --body-gains hold')
    if args.touch != 'keep' and (readonly or not sides):
        parser.error('--touch on/off requires a selected endpoint and --confirm-motion')
    for name in ('timeout', 'duration', 'feedback_interval', 'left_hz', 'right_hz'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(name+' must be finite and positive')
    for side in ('left', 'right'):
        pos, unit = getattr(args, side+'_pos'), getattr(args, side+'_unit')
        for name in ('pos', 'vel_limit', 'torque_nm'):
            values = getattr(args, side+'_'+name)
            if values is not None and (side not in sides or not all(math.isfinite(v) for v in values)):
                parser.error('targets must be finite and belong to a selected endpoint')
        if not readonly and side in sides and (unit is None or (args.pattern == 'target' and pos is None)):
            parser.error('motion requires explicit --'+side+'-pos and --'+side+'-unit')
        speeds = getattr(args, side+'_vel_limit')
        if speeds is not None and any(v < 0 for v in speeds):
            parser.error('vel-limit is a nonnegative magnitude, not signed velocity')
    client_factory = EndEffectorClient if _client_factory is None else _client_factory
    controller_factory = PullController if _controller_factory is None else _controller_factory
    state_factory = PullStateSource if _state_source_factory is None else _state_source_factory
    with ExitStack() as stack:
        connection = stack.enter_context((state_factory if readonly else controller_factory)(group=args.group))
        clients = [stack.enter_context(client_factory(side, connection=connection, read_only=readonly,
            observation_sources=() if readonly else ((SampleStream.EE_STATE, 1 if side == 'left' else 2),))) for side in sides]
        if readonly:
            if clients:
                observe(clients, timeout=args.timeout)
            else:
                print(f'none: BodyState={connection.wait_state(args.timeout)}; no EE wait/lease/write')
            return
        targets = {}
        for client in clients:
            info = client.wait_info(args.timeout)
            positions = getattr(args, client.side+'_pos')
            if positions is None:
                positions = ((args.sine_min_m+args.sine_max_m)/2,) if args.pattern == 'sine' else tuple(
                    a+args.low_frac*(b-a) for a,b in zip(info.pos_min,info.pos_max))
            targets[client.side] = position_target(info, positions,
                unit=getattr(args, client.side+'_unit'), vel_limit=getattr(args, client.side+'_vel_limit'),
                torque=getattr(args, client.side+'_torque_nm'))
        try:
            run(connection, clients, targets, timeout=args.timeout, duration=args.duration,
                rates={side: getattr(args, side+'_hz') for side in sides}, body_enabled=not args.ee_only,
                body_gains=args.body_gains, body_pose=args.body_pose, return_to_start=args.return_to_start,
                touch=args.touch, feedback_interval=args.feedback_interval,
                pattern=args.pattern, period=args.period_s, sine_min=args.sine_min_m, sine_max=args.sine_max_m,
                low_frac=args.low_frac, high_frac=args.high_frac, loop=args.loop)
        except KeyboardInterrupt:
            print('interrupted; CANCEL attempted once, no neutral end-effector target sent')


if __name__ == '__main__':
    main()
