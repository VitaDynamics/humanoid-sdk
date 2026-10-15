"""EXTERNAL body hold/robot-sourced zero ramp and the shared CANCEL guard."""
from contextlib import contextmanager
from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import signal
import time


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


def _observe_zero_dwell(frame, target, *, body, monotonic, emit):
    """Record the existing example dwell without a feedback-arrival gate."""
    began = monotonic()
    peak_error = 0.0
    peak_speed = 0.0
    samples = math.ceil(body.DWELL_S / body.PERIOD_S)
    for sample in range(samples):
        state = frame(target)  # Retains the lifecycle's safety/protocol checks.
        error = max((abs(j.q - q) for j, q in zip(state.joints, target)
                     if j.populated), default=0.0)
        speed = max((abs(j.dq) for j in state.joints if j.populated), default=0.0)
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
                 body_pose='measured', sleep=time.sleep, monotonic=time.monotonic, emit=print):
    if isinstance(timeout, bool) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('timeout must be finite and positive')
    if body_gains not in ('hold', 'zero'):
        raise ValueError('body_gains must be hold or zero')
    if body_pose not in ('measured', 'zero'):
        raise ValueError('body_pose must be measured or zero')
    if body_pose == 'zero' and body_gains != 'hold':
        raise ValueError('zero pose requires frozen FIXED_STAND gains; do not select zero gains')
    info = body.require_body_info(controller, timeout)
    plan = None
    if body_pose == 'zero':
        spec = importlib.util.spec_from_file_location(
            'gripper_fixed_stand_pose', Path(__file__).with_name('fixed_stand_pose.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        plan = module.FixedStandPose(info)  # Reject all bad/missing zero limits before PREPARE.
    initial = body._wait_for_stationary_body_state(
        controller, timeout=timeout, sleep=sleep, monotonic=monotonic)
    pose = tuple(j.q if j.populated else 0.0 for j in initial.joints)
    if plan is not None:
        plan.measured(initial)
    hold = body._position_command(pose)
    if body_gains == 'zero':
        hold = replace(hold, joints=tuple(replace(j, kp=0.0, kd=0.0, dq=0.0, tau=0.0)
                                          for j in hold.joints))
    attempted = False
    active = False
    traces = []
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
            initial = controller.wait_body_state(timeout)
            body._validate_body_state(initial, require_stationary=False)
            pose = plan.measured(initial)
        last_id = initial.state_id
        last_change = monotonic()
        next_publish = monotonic()
        next_cmd_id = 1
        check_hold = plan is None
        stage = 'hold'

        def frame(target=None):
            nonlocal last_id, last_change, next_publish, next_cmd_id
            if not active:
                raise RuntimeError('EXTERNAL frame callback is no longer active')
            delay = next_publish - monotonic()
            if delay > 0:
                sleep(delay)
            status = controller.status
            if (status is None or status.current_action != 'EXTERNAL'
                    or status.phase != body.ExternalPhase.RUNNING
                    or (status.generation, status.client_id, status.lease_id) !=
                       (entered.generation, entered.client_id, entered.lease_id)
                    or status.reject_count != entered.reject_count):
                raise RuntimeError(f'body EXTERNAL ownership/phase/reject_count changed: {status!r}')
            state = controller.wait_body_state(timeout)
            body._validate_body_state(state, require_stationary=False)
            now = monotonic()
            if state.state_id != last_id:
                last_id, last_change = state.state_id, now
            elif now - last_change >= body.LOWSTATE_WINDOW_S:
                raise RuntimeError('body lowstate became stale')
            if check_hold and any(abs(j.q - q) > body.RETURN_ENDPOINT_POSITION_TOLERANCE_RAD
                                  for j, q in zip(state.joints, pose) if j.populated):
                raise RuntimeError('body departed from its held pose')
            if plan is None:
                command = replace(hold, input_state_id=state.state_id,
                                  input_state_ts_real_us=state.ts_state_real_us)
            else:
                plan.measured(state)
                q = pose if target is None else target(now) if callable(target) else target
                command = plan.command(q, state, next_cmd_id)
            controller.publish(command)
            if plan is not None:
                emit(f'BODY stage={stage} cmd_id={command.cmd_id} input_state={state.state_id} '
                     f'q_cmd={tuple(j.q for j in command.joints)!r} '
                     f'control={status!r} applied={state.applied_cmd!r}')
                next_cmd_id = next_cmd_id % 0xffff + 1
            next_publish = max(next_publish + body.PERIOD_S, monotonic() + body.PERIOD_S)
            return state

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
            _observe_zero_dwell(frame, pose, body=body, monotonic=monotonic, emit=emit)
            check_hold = True
            stage = 'hold-zero'
            emit('zero endpoint settled check disabled (observation only); '
                 'holding EXTERNAL in the same session before gripper commands')
        command_loop(frame)
    finally:
        active = False
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
                emit('CANCEL confirmed PASSIVE; no further gripper targets; body may fall without support')
        finally:
            for trace in traces:
                trace.close()
