#!/usr/bin/env python3
"""Select body gains and exercise independent BrainCo hands."""
import argparse
import importlib.util
from contextlib import ExitStack, contextmanager
from dataclasses import replace
import math
from pathlib import Path
import struct
import signal
import time

from locomotion_aorta import ExternalController, ExternalPhase, HandClient, HandType, InhibitReason
from locomotion_aorta.hand import validate_info
# Load the existing sibling example even in isolated Python (-I) bundle checks.
_body_spec = importlib.util.spec_from_file_location(
    'brainco_body_hold_baseline', Path(__file__).with_name('external_control.py'))
body = importlib.util.module_from_spec(_body_spec)
_body_spec.loader.exec_module(body)

# Owner-selected default; each joint must still advertise sufficient dq_max.
# Use the schema float32 representation, matching the advertised float limits.
HAND_SPEED_RAD_S = struct.unpack("<f", struct.pack("<f", math.radians(20.0)))[0]


@contextmanager
def stop_signals():
    """Interrupt once; keep repeated stop signals from interrupting CANCEL."""
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


def validate_motion_options(speed_deg_s, pattern, low_frac, high_frac, feedback_interval):
    if not math.isfinite(speed_deg_s) or speed_deg_s <= 0:
        raise ValueError('--speed-deg-s must be finite and positive')
    try:
        speed = struct.unpack('<f', struct.pack('<f', math.radians(speed_deg_s)))[0]
    except OverflowError as error:
        raise ValueError('--speed-deg-s must fit float32 rad/s') from error
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError('--speed-deg-s must fit positive float32 rad/s')
    if pattern not in ('simultaneous', 'sequential'):
        raise ValueError('--pattern must be simultaneous or sequential')
    if not (math.isfinite(low_frac) and math.isfinite(high_frac)
            and 0 <= low_frac < high_frac <= 1):
        raise ValueError('requires 0 <= --low-frac < --high-frac <= 1')
    if not math.isfinite(feedback_interval) or feedback_interval < 1:
        raise ValueError('--feedback-interval must be finite and at least 1 second')
    return speed


def validate_hand(info, state, *, require_output=False, speed=HAND_SPEED_RAD_S):
    validate_info(info)
    if (info.model != 'BRAINCO_REVO2' or info.hand_type != HandType.BRAINCO_REVO2
            or info.dof != 6 or info.joint_names !=
               ('thumb_pitch', 'thumb_roll', 'index', 'middle', 'ring', 'little')):
        raise ValueError('requires BRAINCO_REVO2 with six joints')
    if (state.side != info.side or state.hand_type != info.hand_type
            or state.dof != info.dof or len(state.joints) != info.dof):
        raise ValueError('HandState identity/DOF differs from HandInfo')
    if state.inhibit_reason in (InhibitReason.EMERGENCY, InhibitReason.DOMAIN_FAULT,
                               InhibitReason.NOT_READY):
        raise RuntimeError(f'hand inhibited: reason={state.inhibit_reason}')
    if any(not math.isfinite(j.q) or not math.isfinite(j.dq)
           or not low <= j.q <= high
           for j, low, high in zip(state.joints, info.q_min, info.q_max)):
        raise ValueError('hand feedback is nonfinite or outside info limits')
    if any(limit < speed for limit in info.dq_max):
        raise ValueError('requested speed exceeds advertised speed limit')
    if require_output and (not state.output_allowed or state.inhibit_reason != InhibitReason.NONE):
        raise RuntimeError('hand output was inhibited during motion')


def print_feedback(client, emit=print, *, command=None):
    state = client.state
    if state is None:
        emit(f'{client.side}: no HandState')
        return
    chain = {name: getattr(state, 'cmd_' + name + '_us') for name in (
        'middleware_received', 'received', 'dequeued', 'accepted', 'first_output',
        'write_ack', 'first_feedback')}
    # Only subtract timestamps from the same robot VitaNow-us clock. Zero
    # stages remain unmeasured; PC creation/send and sensor clocks are separate.
    delta = {}
    for start, end in (('received', 'accepted'), ('accepted', 'first_output'),
                       ('first_output', 'write_ack'), ('write_ack', 'first_feedback')):
        a, b = chain[start], chain[end]
        delta[start + '->' + end] = b - a if a and b and b >= a else 'NOT_MEASURED'
    if command is not None:
        emit(f'{client.side}: client_last_submitted={command.session_id}/{command.sequence}/{command.cmd_id} '
             f'input_state={command.input_state_id}@{command.input_state_ts_real_us} '
             f'client_created_us={command.ts_client_created_us} client_send_us={command.ts_client_send_us} '
             '(client clock, diagnostic only; state echo below identifies the robot chain)')
    emit(f'{client.side}: state={state.state_id} echo={state.last_cmd_session_id}/'
         f'{state.last_cmd_sequence}/{state.last_cmd_id} output={state.output_allowed} '
         f'inhibit={state.inhibit_reason} q_rad={[j.q for j in state.joints]} '
         f'q_cmd_rad={[j.q_cmd for j in state.joints]} '
         f'dq_rad_s={[j.dq for j in state.joints]} '
         f'reject={state.reject_count}/{state.last_reject_code} '
         f'age_us={state.last_valid_age_us} robot_chain_us={chain} delta_us={delta}')


def wait_touch_info(client, enabled, timeout):
    # Info timestamps stay unchanged after set_touch. Require a later topic
    # event and the acknowledged value, keeping the existing caller timeout.
    previous = client.info
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('timed out waiting for hand touch info update')
        info = client.wait_info(remaining, after_info=previous)
        if info.touch_enabled == enabled:
            return info
        previous = info


def observe(clients, *, timeout, emit=print):
    for client in clients:
        info = client.wait_info(timeout)
        state = client.wait_state(timeout)
        emit(f'{client.side}: info={info} state_identity='
             f'{state.hand_type}/{state.side}/{state.dof}; read-only, no lease/cmd/set_touch')
        print_feedback(client, emit)
        touch = client.touch
        if touch is not None:
            emit(f'{client.side}: touch={touch}')


@stop_signals()
def run(controller, clients, *, timeout, body_gains='hold', speed_deg_s=20.0,
        pattern='simultaneous', loop=False, low_frac=0.25, high_frac=0.75,
        feedback_interval=1.0, sleep=time.sleep, monotonic=time.monotonic, emit=print):
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('timeout must be finite and positive')
    if body_gains not in ('zero', 'hold'):
        raise ValueError('body_gains must be zero or hold')
    speed = validate_motion_options(speed_deg_s, pattern, low_frac, high_frac, feedback_interval)
    infos = {}
    for client in clients:
        info = client.wait_info(timeout)
        validate_hand(info, client.wait_state(timeout), speed=speed)
        infos[client.side] = info
        emit(f'{client.side}: model={info.model}, joints={info.joint_names}, '
             f'limits_rad={list(zip(info.q_min, info.q_max))}, speed={speed_deg_s:g} deg/s, '
             f'pattern={pattern}, loop={loop}, fractions={low_frac:g}/{high_frac:g}')
    body.require_body_info(controller, timeout)
    start_state = body._wait_for_stationary_body_state(
        controller, timeout=timeout, sleep=sleep, monotonic=monotonic)
    start_q = tuple(m.q if m.populated else 0.0 for m in start_state.joints)
    hold = body._position_command(start_q)
    if body_gains == 'zero':
        hold = replace(hold, joints=tuple(replace(cmd, kp=0.0, kd=0.0, dq=0.0, tau=0.0)
                                              for cmd in hold.joints))
    gains = 'kp=kd=0' if body_gains == 'zero' else 'existing DEMO_KP/FIXED_STAND_KD'
    emit(f'body: --body-gains {body_gains}, 31-slot measured q, {gains}, dq=tau=0; '
         'hands use independent EE commands; no body zero/return trajectory')
    attempted = False
    try:
        attempted = True  # CANCEL even if entry is interrupted after PREPARE/COMMIT.
        entered = controller.enter_external(timeout)
        last_body_id, last_body_change = start_state.state_id, monotonic()
        last_hand = {c.side: (c.state.state_id, monotonic(), c.state.reject_count) for c in clients}
        next_publish = monotonic()
        next_print = monotonic()
        sent = {}
        last_commands = {}
        stats = {}

        def frame(*, moving):
            nonlocal next_publish, next_print, last_body_id, last_body_change
            delay = next_publish - monotonic()
            if delay > 0:
                sleep(delay)
            status = controller.status
            if (status is None or status.current_action != 'EXTERNAL'
                    or status.phase != ExternalPhase.RUNNING
                    or (status.generation, status.client_id, status.lease_id) !=
                       (entered.generation, entered.client_id, entered.lease_id)
                    or status.reject_count != entered.reject_count):
                raise RuntimeError('body EXTERNAL ownership/phase/reject_count changed')
            state = controller.wait_body_state(timeout)
            body._validate_body_state(state, require_stationary=False)
            now = monotonic()
            if state.state_id != last_body_id:
                last_body_id, last_body_change = state.state_id, now
            elif now - last_body_change >= body.LOWSTATE_WINDOW_S:
                raise RuntimeError('body lowstate became stale')
            # Retain the current pose using the inherited endpoint tolerance;
            # abort instead of continuing hand movement if the body drifts.
            if any(abs(m.q - q) > body.RETURN_ENDPOINT_POSITION_TOLERANCE_RAD
                   for m, q in zip(state.joints, start_q) if m.populated):
                raise RuntimeError('body departed from its held pose')
            controller.publish(replace(hold, input_state_id=state.state_id,
                                       input_state_ts_real_us=state.ts_state_real_us))
            ready = True
            for client in clients:
                feedback = client.state
                validate_hand(infos[client.side], feedback, require_output=moving, speed=speed)
                previous_id, changed, rejects = last_hand[client.side]
                if feedback.reject_count != rejects:
                    raise RuntimeError('hand reject_count changed')
                if feedback.state_id != previous_id:
                    changed = now
                elif now - changed >= body.LOWSTATE_WINDOW_S:
                    raise RuntimeError('hand state became stale')
                last_hand[client.side] = feedback.state_id, changed, rejects
                if client.side in sent:
                    sequence, progress_time, echoed = sent[client.side]
                    if feedback.last_cmd_session_id == client.session_id:
                        if feedback.last_cmd_sequence > sequence:
                            raise RuntimeError('hand echoed an unsent sequence')
                        if feedback.last_cmd_sequence > echoed:
                            echoed, progress_time = feedback.last_cmd_sequence, now
                    pending_echo = (feedback.last_cmd_session_id != client.session_id
                                    or feedback.last_cmd_sequence < sequence)
                    if pending_echo and now - progress_time >= timeout:
                        raise TimeoutError('hand command echo did not advance within --timeout')
                    sent[client.side] = sequence, progress_time, echoed
                ready = ready and feedback.output_allowed and feedback.inhibit_reason == InhibitReason.NONE
                if client.side in stats:
                    summary = stats[client.side]
                    summary['measured'] = [max(a, j.q) for a, j in zip(summary['measured'], feedback.joints)]
                    summary['reject_end'] = feedback.reject_count
            if now >= next_print:
                for client in clients:
                    print_feedback(client, emit, command=last_commands.get(client.side))
                next_print = now + feedback_interval
            next_publish = max(next_publish + body.PERIOD_S, monotonic() + body.PERIOD_S)
            return ready

        deadline = monotonic() + timeout
        while not frame(moving=False):
            if monotonic() >= deadline:
                raise TimeoutError('hand output did not become enabled after EXTERNAL entry')
        positions = {c.side: tuple(j.q for j in c.state.joints) for c in clients}

        def move(targets):
            nonlocal positions
            steps = max(1, math.ceil(max(abs(target - current)
                        for side in targets for target, current in
                        zip(targets[side], positions[side])) / speed / body.PERIOD_S))
            for step in range(1, steps + 1):
                frame(moving=True)
                for client in clients:
                    side = client.side
                    # Keep legal endpoint fractions in range despite floating
                    # point interpolation rounding; feedback checks stay strict.
                    q = tuple(min(high, max(low, a + (b - a) * step / steps))
                              for a, b, low, high in zip(positions[side], targets[side],
                                                       infos[side].q_min, infos[side].q_max))
                    command = client.publish(q, (speed,) * infos[side].dof)
                    if side in stats:
                        stats[side]['commanded'] = [max(a, b) for a, b in zip(stats[side]['commanded'], q)]
                    _, progress_time, echoed = sent.get(side, (0, monotonic(), 0))
                    sent[side] = command.sequence, progress_time, echoed
                    last_commands[side] = command
            positions = targets
            for _ in range(math.ceil(body.DWELL_S / body.PERIOD_S)):
                frame(moving=True)

        low_targets = {side: tuple(min(b, max(a, a + low_frac * (b - a))) for a, b in zip(info.q_min, info.q_max))
                       for side, info in infos.items()}
        high_targets = {side: tuple(min(b, max(a, a + high_frac * (b - a))) for a, b in zip(info.q_min, info.q_max))
                        for side, info in infos.items()}
        # Establish the low pose before sequential motion; preserve the original
        # simultaneous low/high/low trajectory and dwell with default options.
        move(low_targets)

        def begin_summary():
            stats.clear()
            for client in clients:
                state = client.state
                stats[client.side] = dict(measured=[j.q for j in state.joints],
                                         commanded=list(positions[client.side]),
                                         reject_start=state.reject_count, reject_end=state.reject_count)

        def joint_summary(side, index, started, round_number):
            summary = stats[side]
            emit(f'{side}: round={round_number} joint={infos[side].joint_names[index]} '
                 f'measured_max_rad={summary["measured"][index]:.6g} '
                 f'commanded_max_rad={summary["commanded"][index]:.6g} '
                 f'roundtrip_s={monotonic() - started:.3f} '
                 f'reject_count={summary["reject_start"]}->{summary["reject_end"]} '
                 f'body_reject_count={entered.reject_count}->{controller.status.reject_count}')

        round_number = 0
        while True:
            round_number += 1
            round_start = monotonic()
            if pattern == 'simultaneous':
                begin_summary()
                move(high_targets)
                move(low_targets)
                for side, info in infos.items():
                    for index in range(info.dof):
                        joint_summary(side, index, round_start, round_number)
            else:
                # Across both hands too, only one joint leaves its low target.
                for side, info in infos.items():
                    for index in range(info.dof):
                        started = monotonic()
                        begin_summary()
                        targets = dict(low_targets)
                        q = list(targets[side])
                        q[index] = high_targets[side][index]
                        targets[side] = tuple(q)
                        move(targets)
                        move(low_targets)
                        joint_summary(side, index, started, round_number)
            emit(f'round={round_number} complete pattern={pattern} '
                 f'joints={sum(info.dof for info in infos.values())} '
                 f'elapsed_s={monotonic() - round_start:.3f} '
                 f'hand_reject_counts={[(c.side, last_hand[c.side][2]) for c in clients]} '
                 f'body_reject_count={entered.reject_count}->{controller.status.reject_count}')
            if not loop:
                break
    finally:
        if attempted:
            # A stop during normal/fault cleanup must also allow CANCEL to finish.
            for signum in (signal.SIGINT, signal.SIGTERM):
                signal.signal(signum, signal.SIG_IGN)
            try:
                status = controller.exit_external(timeout)
                if status.current_action != 'PASSIVE':
                    raise RuntimeError(f'CANCEL returned {status.current_action}, expected PASSIVE')
            except BaseException as error:
                emit(f'CANCEL UNCONFIRMED: {error}; use the on-site emergency procedure')
                raise
            emit('CANCEL confirmed PASSIVE; no further hand target sent; hands retain last target '
                 '(physical behavior NOT_MEASURED); body may fall without support')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right', 'both'), default='both')
    parser.add_argument('--group', default='default')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--body-gains', choices=('zero', 'hold'), default='hold',
                        help='zero: kp=kd=0 with measured q; hold: existing pose gains (default)')
    parser.add_argument('--dry-run', action='store_true', help='read-only subscriptions; no lease or writes')
    parser.add_argument('--confirm-motion', action='store_true', help='requires separate current on-site approval')
    parser.add_argument('--touch', choices=('keep', 'on', 'off'), default='keep')
    parser.add_argument('--speed-deg-s', type=float, default=20.0, help='positive speed within info dq_max (default: 20)')
    parser.add_argument('--pattern', choices=('simultaneous', 'sequential'), default='simultaneous')
    parser.add_argument('--loop', action='store_true', help='repeat until SIGINT/SIGTERM or a safety failure')
    parser.add_argument('--low-frac', type=float, default=0.25)
    parser.add_argument('--high-frac', type=float, default=0.75)
    parser.add_argument('--feedback-interval', type=float, default=1.0, help='feedback interval in seconds, at least 1')
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    try:
        validate_motion_options(args.speed_deg_s, args.pattern, args.low_frac,
                                args.high_frac, args.feedback_interval)
    except ValueError as error:
        parser.error(str(error))
    if args.dry_run and args.confirm_motion:
        parser.error('--dry-run and --confirm-motion are mutually exclusive')
    readonly = args.dry_run or not args.confirm_motion
    if readonly and args.touch != 'keep':
        parser.error('--touch on/off writes require --confirm-motion')
    sides = ('left', 'right') if args.side == 'both' else (args.side,)
    with ExitStack() as stack:
        if readonly:
            clients = [stack.enter_context(HandClient(side, group=args.group, read_only=True)) for side in sides]
            observe(clients, timeout=args.timeout)
        else:
            controller = stack.enter_context(ExternalController(group=args.group))
            clients = [stack.enter_context(HandClient(side, group=args.group, node=controller.transport.node))
                       for side in sides]
            if args.touch != 'keep':
                for client in clients:
                    enabled = client.set_touch(args.touch == 'on', args.timeout)
                    wait_touch_info(client, enabled, args.timeout)
            try:
                run(controller, clients, timeout=args.timeout, body_gains=args.body_gains,
                    speed_deg_s=args.speed_deg_s, pattern=args.pattern, loop=args.loop,
                    low_frac=args.low_frac, high_frac=args.high_frac,
                    feedback_interval=args.feedback_interval)
            except KeyboardInterrupt:
                print('interrupted; lease release attempted, no neutral hand command')


if __name__ == '__main__':
    main()
