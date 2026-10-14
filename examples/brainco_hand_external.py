#!/usr/bin/env python3
"""HIL only: hold the measured body pose and exercise independent BrainCo hands."""
import argparse
import importlib.util
from contextlib import ExitStack
import math
from pathlib import Path
import struct
import time

from locomotion_aorta import ExternalController, ExternalPhase, HandClient, HandType, InhibitReason
from locomotion_aorta.hand import validate_info
# Load the existing sibling example even in isolated Python (-I) bundle checks.
_body_spec = importlib.util.spec_from_file_location(
    'brainco_body_hold_baseline', Path(__file__).with_name('external_control.py'))
body = importlib.util.module_from_spec(_body_spec)
_body_spec.loader.exec_module(body)

# Existing Revo2 motor JSON placeholder: brainco_revo2.speed_deg_s = [1, 1, 1, 1, 1, 1].
# This is a HIL placeholder, not a measured hardware performance claim.
# Use the schema float32 representation, matching the advertised float limits.
HAND_SPEED_RAD_S = struct.unpack("<f", struct.pack("<f", math.radians(1.0)))[0]


def validate_hand(info, state, *, require_output=False):
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
    if any(limit < HAND_SPEED_RAD_S for limit in info.dq_max):
        raise ValueError('configured 1 deg/s placeholder exceeds advertised speed limit')
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


def observe(clients, *, timeout, emit=print):
    for client in clients:
        info = client.get_info(timeout)
        state = client.wait_state(timeout)
        emit(f'{client.side}: HIL only info={info} state_identity='
             f'{state.hand_type}/{state.side}/{state.dof}; read-only, no lease/cmd/set_touch')
        print_feedback(client, emit)
        touch = client.touch
        if touch is not None:
            emit(f'{client.side}: touch={touch}')


def run(controller, clients, *, timeout, sleep=time.sleep, monotonic=time.monotonic, emit=print):
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('timeout must be finite and positive')
    infos = {}
    for client in clients:
        info = client.get_info(timeout)
        validate_hand(info, client.wait_state(timeout))
        infos[client.side] = info
        emit(f'{client.side}: model={info.model}, joints={info.joint_names}, '
             f'limits_rad={list(zip(info.q_min, info.q_max))}, speed=1 deg/s placeholder')
    start_state = body._wait_for_stationary_lowstate(
        controller, timeout=timeout, sleep=sleep, monotonic=monotonic)
    start_q = tuple(m.q for m in start_state.motor_states[:body.COMMAND_SLOTS])
    hold = body._position_command(start_q)
    emit('body: 42-slot measured-pose hold, existing DEMO_KP/FIXED_STAND_KD, dq=tau=0; '
         'finger slots reserved; no body zero/return trajectory')
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
            state = controller.wait_lowstate(timeout)
            body._validate_lowstate(state, require_stationary=False)
            now = monotonic()
            if state.state_id != last_body_id:
                last_body_id, last_body_change = state.state_id, now
            elif now - last_body_change >= body.LOWSTATE_WINDOW_S:
                raise RuntimeError('body lowstate became stale')
            # Retain the current pose using the inherited endpoint tolerance;
            # abort instead of continuing hand movement if the body drifts.
            if any(abs(m.q - q) > body.RETURN_ENDPOINT_POSITION_TOLERANCE_RAD
                   for m, q in zip(state.motor_states[:body.COMMAND_SLOTS], start_q)):
                raise RuntimeError('body departed from its held pose')
            controller.publish(hold)
            ready = True
            for client in clients:
                feedback = client.state
                validate_hand(infos[client.side], feedback, require_output=moving)
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
            if now >= next_print:
                for client in clients:
                    print_feedback(client, emit, command=last_commands.get(client.side))
                next_print = now + body.DWELL_S
            next_publish = max(next_publish + body.PERIOD_S, monotonic() + body.PERIOD_S)
            return ready

        deadline = monotonic() + timeout
        while not frame(moving=False):
            if monotonic() >= deadline:
                raise TimeoutError('hand output did not become enabled after EXTERNAL entry')
        positions = {c.side: tuple(j.q for j in c.state.joints) for c in clients}
        for fraction in (0.25, 0.75, 0.25):
            targets = {side: tuple(low + fraction * (high - low)
                                   for low, high in zip(info.q_min, info.q_max))
                       for side, info in infos.items()}
            steps = max(1, math.ceil(max(abs(target - current)
                        for side in targets for target, current in
                        zip(targets[side], positions[side])) / HAND_SPEED_RAD_S / body.PERIOD_S))
            for step in range(1, steps + 1):
                frame(moving=True)
                for client in clients:
                    side = client.side
                    q = tuple(a + (b - a) * step / steps
                              for a, b in zip(positions[side], targets[side]))
                    command = client.publish(q, (HAND_SPEED_RAD_S,) * infos[side].dof)
                    _, progress_time, echoed = sent.get(side, (0, monotonic(), 0))
                    sent[side] = command.sequence, progress_time, echoed
                    last_commands[side] = command
            positions = targets
            for _ in range(math.ceil(body.DWELL_S / body.PERIOD_S)):
                frame(moving=True)
        for client in clients:
            print_feedback(client, emit, command=last_commands.get(client.side))
    finally:
        if attempted:
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
    parser.add_argument('--dry-run', action='store_true', help='read-only RPC/subscriptions; no lease or writes')
    parser.add_argument('--confirm-motion', action='store_true', help='requires separate current on-site approval')
    parser.add_argument('--touch', choices=('keep', 'on', 'off'), default='keep')
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
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
                    client.set_touch(args.touch == 'on', args.timeout)
            try:
                run(controller, clients, timeout=args.timeout)
            except KeyboardInterrupt:
                print('interrupted; lease release attempted, no neutral hand command')


if __name__ == '__main__':
    main()
