#!/usr/bin/env python3
"""Independent AGX demo; defaults to read-only info/state subscriptions."""
import argparse
from contextlib import ExitStack
import importlib.util
import math
from pathlib import Path
import time

from locomotion_aorta.gripper import validate_info, validate_target
_lifecycle_spec = importlib.util.spec_from_file_location(
    'gripper_external_lifecycle', Path(__file__).with_name('gripper_external_lifecycle.py'))
_lifecycle = importlib.util.module_from_spec(_lifecycle_spec)
_lifecycle_spec.loader.exec_module(_lifecycle)
run_external = _lifecycle.run_external
from locomotion_aorta import InhibitReason

# The existing SDK body example remains the authority for its unchanged guards.
_body_spec = importlib.util.spec_from_file_location(
    'gripper_body_hold_baseline',
    Path(__file__).with_name('external_control.py'))
body = importlib.util.module_from_spec(_body_spec)
_body_spec.loader.exec_module(body)


def validate_state(info, state, *, require_output=False):
    validate_info(info)
    if state is None or (state.side, state.gripper_type) != (info.side, info.gripper_type):
        raise ValueError('GripperState identity differs from Info')
    if state.inhibit_reason in (InhibitReason.EMERGENCY, InhibitReason.DOMAIN_FAULT,
                               InhibitReason.NOT_READY):
        raise RuntimeError(f'gripper inhibited: reason={state.inhibit_reason}')
    if (not state.sample_valid or state.sample is None or not math.isfinite(state.sample.q)
            or not info.opening_min_m <= state.sample.q <= info.opening_max_m):
        raise ValueError('gripper opening sample is unavailable or outside Info limits')
    if require_output and (not state.output_allowed or state.inhibit_reason != InhibitReason.NONE):
        raise RuntimeError('gripper output was inhibited during motion')


def print_feedback(client, emit=print):
    state = client.state
    if state is None:
        emit(f'{client.side}: no GripperState')
        return
    sample = state.sample if state.sample_valid else None
    target = state.host_target if state.host_target_valid else None
    chain = {name: getattr(state, 'ts_cmd_' + name + '_us') for name in (
        'received', 'accepted', 'first_output', 'write_ack', 'first_feedback')}
    delta = {}
    for start, end in (('received', 'accepted'), ('accepted', 'first_output'),
                       ('first_output', 'write_ack'), ('write_ack', 'first_feedback')):
        a, b = chain[start], chain[end]
        delta[start + '->' + end] = b - a if a and b and b >= a else 'NOT_MEASURED'
    emit(f'{client.side}: state={state.state_id} measured_opening_m='
         f'{None if sample is None else sample.q} reported_torque_nm='
         f'{None if sample is None else sample.reported_torque_nm} '
         f'status_raw={None if sample is None else sample.status_raw} '
         f'host_target={target} host_key={state.host_target_session_id}/'
         f'{state.host_target_sequence}/{state.host_target_cmd_id} '
         f'trace_received_key={state.trace_session_id}/{state.trace_sequence}/{state.trace_cmd_id} '
         f'output={state.output_allowed} inhibit={state.inhibit_reason} '
         f'robot_chain_us={chain} delta_us={delta}; host target is not device execution')


def observe(client, *, timeout, emit=print):
    info = client.wait_info(timeout)
    client.wait_state(timeout)
    emit(f'{client.side}: Info={info}; read-only, no lease/cmd; unknown sources remain unknown')
    print_feedback(client, emit)


def run(controller, client, *, q, dq=0.0, torque_cmd_nm=None,
        timeout=5.0, duration=body.DWELL_S, body_gains='hold',
        body_pose='measured', return_to_start=False,
        sleep=time.sleep, monotonic=time.monotonic, emit=print):
    if not math.isfinite(duration) or isinstance(duration, bool) or duration <= 0:
        raise ValueError('duration must be finite and positive')
    info = client.wait_info(timeout)
    validate_target(info, q, dq, torque_cmd_nm)
    initial = client.wait_state(timeout)
    validate_state(info, initial)
    if return_to_start:
        validate_target(info, initial.sample.q, dq, torque_cmd_nm)
    emit(f'{client.side}: q={q} m, dq={dq} m/s host cap '
         f'(0 remains robot configuration default), torque_cmd_nm={torque_cmd_nm} '
         f'(None uses robot hold; explicit0 is0), duration={duration}s; '
         'signed host motor-side Nm, not force N/device limit; no SDK trajectory interpolation')

    def command_loop(frame):
        previous_id, last_change = initial.state_id, monotonic()
        submitted = None
        echoed = 0
        last_echo_change = monotonic()

        def feedback(moving):
            nonlocal previous_id, last_change, echoed, last_echo_change
            state = client.state
            validate_state(info, state, require_output=moving)
            now = monotonic()
            if state.state_id != previous_id:
                previous_id, last_change = state.state_id, now
            elif now - last_change >= body.LOWSTATE_WINDOW_S:
                raise RuntimeError('gripper state became stale')
            if submitted is not None:
                if state.trace_session_id == client.session_id:
                    if state.trace_sequence > submitted.sequence:
                        raise RuntimeError('gripper trace echoed an unsent sequence')
                    if state.trace_sequence > echoed:
                        echoed, last_echo_change = state.trace_sequence, now
                    if state.ts_cmd_received_us and not state.ts_cmd_accepted_us:
                        raise RuntimeError(f'gripper input not accepted: trace='
                                           f'{state.trace_session_id}/{state.trace_sequence}/{state.trace_cmd_id}')
                if echoed < submitted.sequence and now - last_echo_change >= timeout:
                    raise TimeoutError('gripper received/accepted trace did not advance within --timeout')
            return state

        ready_deadline = monotonic() + timeout
        while True:
            frame()
            state = feedback(False)
            if state.output_allowed and state.inhibit_reason == InhibitReason.NONE:
                break
            if monotonic() >= ready_deadline:
                raise TimeoutError('gripper output did not become enabled after EXTERNAL entry')
        targets = (q, initial.sample.q) if return_to_start else (q,)
        for target in targets:
            emit(f'gripper leg target={target} m; same EXTERNAL session')
            deadline = monotonic() + duration
            next_print = monotonic()
            while monotonic() < deadline:
                frame()
                state = feedback(True)
                submitted = client.publish_target(target, dq, torque_cmd_nm=torque_cmd_nm, input_state=state)
                if monotonic() >= next_print:
                    print_feedback(client, emit)
                    next_print = monotonic() + body.DWELL_S
            # Confirm admission of the final input without equating it with TX or
            # physical arrival, and keep the body watchdog alive while waiting.
            final_deadline = monotonic() + timeout
            while submitted is not None:
                frame()
                state = feedback(True)
                if (state.trace_session_id == submitted.session_id
                        and state.trace_sequence == submitted.sequence
                        and state.ts_cmd_accepted_us):
                    break
                if monotonic() >= final_deadline:
                    raise TimeoutError('final gripper input admission was not observed')

    run_external(controller, command_loop, body=body, timeout=timeout, body_gains=body_gains, body_pose=body_pose,
                 sleep=sleep, monotonic=monotonic, emit=emit)
    print_feedback(client, emit)
    emit('Target streaming ended; measured endpoint/completion is not inferred from host target or trace')


def main(argv=None, *, _client_factory=None, _controller_factory=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right'), required=True)
    parser.add_argument('--group', default='default')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--target-m', type=float, help='absolute total opening target; required for motion')
    parser.add_argument('--speed-m-s', type=float, default=0.0, help='host cap; 0 leaves robot configuration default')
    parser.add_argument('--torque-cmd-nm', type=float, default=None,
                        help='signed software motor-side Nm; omitted=config hold, explicit0=0')
    parser.add_argument('--duration', type=float, default=body.DWELL_S, help='target streaming time in seconds')
    parser.add_argument('--body-gains', choices=('hold', 'zero'), default='hold')
    parser.add_argument('--body-pose', choices=('measured', 'zero'), default='measured',
                        help='zero uses frozen FIXED_STAND gains and real-time linear entry')
    parser.add_argument('--return-to-start', action='store_true',
                        help='after target streaming, return once to the initial measured gripper opening')
    parser.add_argument('--dry-run', action='store_true', help='read-only subscriptions, no lease or commands')
    parser.add_argument('--confirm-motion', action='store_true', help='requires separate current on-site authorization')
    args = parser.parse_args(argv)
    if args.body_pose == 'zero' and args.body_gains != 'hold':
        parser.error('--body-pose zero requires FIXED_STAND gains; select --body-gains hold')
    if args.dry_run and args.confirm_motion:
        parser.error('--dry-run and --confirm-motion are mutually exclusive')
    if not all(math.isfinite(v) and v > 0 for v in (args.timeout, args.duration)):
        parser.error('--timeout and --duration must be finite and positive')
    if not math.isfinite(args.speed_m_s) or args.speed_m_s < 0:
        parser.error('--speed-m-s must be finite and nonnegative')
    if args.torque_cmd_nm is not None and not math.isfinite(args.torque_cmd_nm):
        parser.error('--torque-cmd-nm must be finite')
    if args.confirm_motion and (args.target_m is None or not math.isfinite(args.target_m)):
        parser.error('motion requires a finite --target-m')
    if _client_factory is None:
        from locomotion_aorta import GripperClient
        _client_factory = GripperClient
    if _controller_factory is None:
        from locomotion_aorta import ExternalController
        _controller_factory = ExternalController
    readonly = args.dry_run or not args.confirm_motion
    with ExitStack() as stack:
        if readonly:
            client = stack.enter_context(_client_factory(args.side, group=args.group, read_only=True))
            observe(client, timeout=args.timeout)
        else:
            controller = stack.enter_context(_controller_factory(group=args.group))
            client = stack.enter_context(_client_factory(args.side, group=args.group,
                                                        node=controller.transport.node))
            try:
                run(controller, client, q=args.target_m, dq=args.speed_m_s,
                    torque_cmd_nm=args.torque_cmd_nm, timeout=args.timeout,
                    duration=args.duration, body_gains=args.body_gains,
                    body_pose=args.body_pose, return_to_start=args.return_to_start)
            except KeyboardInterrupt:
                print('interrupted; CANCEL attempted; no neutral gripper command')


if __name__ == '__main__':
    main()
