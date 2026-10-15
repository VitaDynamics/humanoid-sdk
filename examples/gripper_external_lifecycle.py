"""EXTERNAL body hold and CANCEL lifecycle used by the gripper example."""
from contextlib import contextmanager
from dataclasses import replace
import math
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


@stop_signals()
def run_external(controller, command_loop, *, body, timeout=5.0, body_gains='hold',
                 sleep=time.sleep, monotonic=time.monotonic, emit=print):
    if isinstance(timeout, bool) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('timeout must be finite and positive')
    if body_gains not in ('hold', 'zero'):
        raise ValueError('body_gains must be hold or zero')
    body.require_body_info(controller, timeout)
    initial = body._wait_for_stationary_body_state(
        controller, timeout=timeout, sleep=sleep, monotonic=monotonic)
    pose = tuple(j.q if j.populated else 0.0 for j in initial.joints)
    hold = body._position_command(pose)
    if body_gains == 'zero':
        hold = replace(hold, joints=tuple(replace(j, kp=0.0, kd=0.0, dq=0.0, tau=0.0)
                                          for j in hold.joints))
    attempted = False
    active = False
    try:
        attempted = True
        entered = controller.enter_external(timeout)
        active = True
        last_id = initial.state_id
        last_change = monotonic()
        next_publish = monotonic()

        def frame():
            nonlocal last_id, last_change, next_publish
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
                raise RuntimeError('body EXTERNAL ownership/phase/reject_count changed')
            state = controller.wait_body_state(timeout)
            body._validate_body_state(state, require_stationary=False)
            now = monotonic()
            if state.state_id != last_id:
                last_id, last_change = state.state_id, now
            elif now - last_change >= body.LOWSTATE_WINDOW_S:
                raise RuntimeError('body lowstate became stale')
            if any(abs(j.q - q) > body.RETURN_ENDPOINT_POSITION_TOLERANCE_RAD
                   for j, q in zip(state.joints, pose) if j.populated):
                raise RuntimeError('body departed from its held pose')
            controller.publish(replace(hold, input_state_id=state.state_id,
                                       input_state_ts_real_us=state.ts_state_real_us))
            next_publish = max(next_publish + body.PERIOD_S, monotonic() + body.PERIOD_S)
            return state

        command_loop(frame)
    finally:
        active = False
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
