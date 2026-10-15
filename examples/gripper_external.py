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


class _GripperGuard:
    """The existing single-side state/trace guards, shared by the dual demo."""

    def __init__(self, client, info, initial, timeout, monotonic):
        self.client, self.info = client, info
        self.timeout, self.monotonic = timeout, monotonic
        self.previous_id = initial.state_id
        self.last_change = self.last_echo_change = monotonic()
        self.submitted = None
        self.echoed = 0

    def feedback(self, moving):
        state = self.client.state
        validate_state(self.info, state, require_output=moving)
        now = self.monotonic()
        if state.state_id != self.previous_id:
            self.previous_id, self.last_change = state.state_id, now
        elif now - self.last_change >= body.LOWSTATE_WINDOW_S:
            raise RuntimeError('gripper state became stale')
        if self.submitted is not None:
            if state.trace_session_id == self.client.session_id:
                if state.trace_sequence > self.submitted.sequence:
                    raise RuntimeError('gripper trace echoed an unsent sequence')
                if state.trace_sequence > self.echoed:
                    self.echoed, self.last_echo_change = state.trace_sequence, now
                if state.ts_cmd_received_us and not state.ts_cmd_accepted_us:
                    raise RuntimeError(f'gripper input not accepted: trace='
                                       f'{state.trace_session_id}/{state.trace_sequence}/{state.trace_cmd_id}')
            if self.echoed < self.submitted.sequence and now - self.last_echo_change >= self.timeout:
                raise TimeoutError('gripper received/accepted trace did not advance within --timeout')
        return state

    def publish(self, q, dq, torque_cmd_nm, state):
        self.submitted = self.client.publish_target(q, dq, torque_cmd_nm=torque_cmd_nm, input_state=state)
        return self.submitted

    def admitted(self, state):
        return (self.submitted is None or
                (state.trace_session_id == self.submitted.session_id
                 and state.trace_sequence == self.submitted.sequence and state.ts_cmd_accepted_us))


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
        guard = _GripperGuard(client, info, initial, timeout, monotonic)

        ready_deadline = monotonic() + timeout
        while True:
            frame()
            state = guard.feedback(False)
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
                state = guard.feedback(True)
                guard.publish(target, dq, torque_cmd_nm, state)
                if monotonic() >= next_print:
                    print_feedback(client, emit)
                    next_print = monotonic() + body.DWELL_S
            # Confirm admission of the final input without equating it with TX or
            # physical arrival, and keep the body watchdog alive while waiting.
            final_deadline = monotonic() + timeout
            while guard.submitted is not None:
                frame()
                state = guard.feedback(True)
                if guard.admitted(state):
                    break
                if monotonic() >= final_deadline:
                    raise TimeoutError('final gripper input admission was not observed')

    run_external(controller, command_loop, body=body, timeout=timeout, body_gains=body_gains, body_pose=body_pose,
                 sleep=sleep, monotonic=monotonic, emit=emit)
    print_feedback(client, emit)
    emit('Target streaming ended; measured endpoint/completion is not inferred from host target or trace')


def sine_opening(elapsed, *, low=.01, high=.08, period=10.0):
    """One shared real-time phase for both topics; q is total opening in m."""
    weight = (1 + math.sin(2 * math.pi * elapsed / period)) / 2
    return low * (1 - weight) + high * weight


def _sine_preflight(controller, clients, *, low, high, torque_cmd_nm, timeout):
    reply = controller.get_robot_info(timeout)
    if reply.status not in (body.ServiceStatus.SUCCESS, body.ServiceStatus.PARTIAL_SUCCESS) or reply.info is None:
        raise RuntimeError(f'get_robot_info failed: status={reply.status!r} {reply.error_message}')
    infos, initials, problems = [], [], []
    for client in clients:
        info = client.wait_info(timeout)
        initial = client.wait_state(timeout)
        infos.append(info)
        initials.append(initial)
        aggregate = [cap.gripper for cap in reply.info.end_effectors
                     if cap.configured and cap.gripper is not None and cap.side == info.side]
        checks = [('topic Info', info)] + [('RobotInfo', cap) for cap in aggregate]
        if len(aggregate) != 1:
            problems.append(f'{client.side}: RobotInfo must provide one configured gripper')
        for source, cap in checks:
            if (cap.side, cap.gripper_type) != (info.side, info.gripper_type):
                problems.append(f'{client.side} {source}: gripper identity differs from topic Info')
            for name, q in (('low', low), ('high', high), ('return', initial.sample.q
                                                         if initial.sample_valid and initial.sample else math.nan)):
                try:
                    validate_target(cap, q, 0.0, torque_cmd_nm)
                except ValueError as error:
                    problems.append(f'{client.side} {source} {name}={q}: {error}')
        try:
            validate_state(info, initial)
        except (ValueError, RuntimeError) as error:
            problems.append(f'{client.side}: {error}')
        if info.max_opening_rate_m_s == 0:
            problems.append(f'{client.side}: configured default opening rate is zero; no finite linear transition')
    if problems:
        raise ValueError('; '.join(problems))
    return infos, initials


def run_sine(controller, clients, *, low=.01, high=.08, period=10.0, duration=30.0,
             torque_cmd_nm=None, timeout=5.0, body_gains='hold', body_pose='zero',
             sleep=time.sleep, monotonic=time.monotonic, emit=print):
    """Zero/hold, dual sine, return, CANCEL in the same EXTERNAL session."""
    if (len(clients) != 2 or tuple(c.side for c in clients) != ('left', 'right')):
        raise ValueError('dual sine requires left and right clients sharing the controller node')
    for name, value in (('period', period), ('duration', duration)):
        if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and positive')
    if not all(math.isfinite(v) for v in (low, high)) or low >= high:
        raise ValueError('sine opening bounds must be finite and low < high')
    infos, initials = _sine_preflight(controller, clients, low=low, high=high,
                                    torque_cmd_nm=torque_cmd_nm, timeout=timeout)
    entry = tuple(state.sample.q for state in initials)
    mid = (low + high) / 2
    stats = {c.side: dict(samples=0, peak_abs_error_m=0., peak_abs_reported_torque_nm=0.,
                         measured_min_m=math.inf, measured_max_m=-math.inf,
                         target_min_m=math.inf, target_max_m=-math.inf,
                         status_raw=set(), inhibit=set()) for c in clients}
    emit(f'DUAL_SINE low_m={low} high_m={high} mid_m={mid} amplitude_m={(high-low)/2} '
         f'period_s={period} duration_s={duration} dq=0 config_default_rates_m_s='
         f'{tuple(info.max_opening_rate_m_s for info in infos)!r} return_opening_m={entry!r}; '
         'shared real-time phase, separate topic writes, not atomic bus output; deviation observation only')

    def command_loop(frame):
        guards = [_GripperGuard(c, i, s, timeout, monotonic) for c, i, s in zip(clients, infos, initials)]
        ready_deadline = monotonic() + timeout
        while True:
            frame()
            states = [g.feedback(False) for g in guards]
            if all(s.output_allowed and s.inhibit_reason == InhibitReason.NONE for s in states):
                break
            if monotonic() >= ready_deadline:
                raise TimeoutError('gripper output did not become enabled after EXTERNAL entry')

        def stream(stage, target_at, seconds):
            began = monotonic()
            while True:
                frame()  # Existing body publication cadence and ownership/phase/state guards.
                elapsed = min(seconds, monotonic() - began)
                targets = target_at(elapsed)
                states = [guard.feedback(True) for guard in guards]  # Check both before either write.
                for client, info, guard, state, q in zip(clients, infos, guards, states, targets):
                    validate_target(info, q, 0.0, torque_cmd_nm)  # Never clip targets.
                    command = guard.publish(q, 0.0, torque_cmd_nm, state)
                    sample = state.sample
                    error = sample.q - q
                    st = stats[client.side]
                    st['samples'] += 1
                    st['peak_abs_error_m'] = max(st['peak_abs_error_m'], abs(error))
                    st['peak_abs_reported_torque_nm'] = max(st['peak_abs_reported_torque_nm'], abs(sample.reported_torque_nm))
                    st['measured_min_m'] = min(st['measured_min_m'], sample.q)
                    st['measured_max_m'] = max(st['measured_max_m'], sample.q)
                    st['target_min_m'] = min(st['target_min_m'], q)
                    st['target_max_m'] = max(st['target_max_m'], q)
                    st['status_raw'].add(sample.status_raw)
                    st['inhibit'].add(state.inhibit_reason)
                    emit(f'GRIPPER_SINE stage={stage} side={client.side} elapsed_s={elapsed:.6f} '
                         f'target_m={q:.9f} measured_m={sample.q:.9f} error_m={error:.9f} '
                         f'reported_torque_nm={sample.reported_torque_nm} status_raw={sample.status_raw} '
                         f'output_allowed={state.output_allowed} inhibit={state.inhibit_reason} '
                         f'cmd_key={command.session_id}/{command.sequence}/{command.cmd_id} '
                         f'trace_key={state.trace_session_id}/{state.trace_sequence}/{state.trace_cmd_id} '
                         f'accepted_us={state.ts_cmd_accepted_us}; observation_only=true')
                if elapsed >= seconds:
                    break

        def transition(stage, start, end):
            # AGX dq=0 resolves to the same effective max_opening_rate published
            # in Info. Use that configured rate for a common linear duration;
            # the robot still owns its per-frame rate limiter.
            seconds = max(abs(b-a) / i.max_opening_rate_m_s for a, b, i in zip(start, end, infos))
            emit(f'GRIPPER_SINE stage={stage} linear_duration_s={seconds} dq=0')
            def position(elapsed):
                if seconds == 0 or elapsed >= seconds:
                    return end
                alpha = elapsed / seconds
                return tuple(a + (b-a)*alpha for a, b in zip(start, end))
            stream(stage, position, seconds)

        transition('entry', tuple(s.sample.q for s in states), (mid, mid))
        stream('sine', lambda elapsed: (sine_opening(elapsed, low=low, high=high, period=period),)*2, duration)
        endpoint = sine_opening(duration, low=low, high=high, period=period)
        transition('return', (endpoint, endpoint), entry)
        final_deadline = monotonic() + timeout
        while True:
            frame()
            states = [guard.feedback(True) for guard in guards]
            if all(guard.admitted(state) for guard, state in zip(guards, states)):
                break
            if monotonic() >= final_deadline:
                raise TimeoutError('final gripper input admission was not observed')
        emit('dual sine return commands admitted; physical arrival is observation only, not inferred')

    try:
        run_external(controller, command_loop, body=body, timeout=timeout, body_gains=body_gains,
                     body_pose=body_pose, sleep=sleep, monotonic=monotonic, emit=emit)
    finally:
        for side, st in stats.items():
            summary = {key: sorted(value) if isinstance(value, set) else
                       None if isinstance(value, float) and not math.isfinite(value) else value
                       for key, value in st.items()}
            emit(f'GRIPPER_SINE summary side={side} observation_only=true {summary!r}')


def main(argv=None, *, _client_factory=None, _controller_factory=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right', 'both'), required=True)
    parser.add_argument('--pattern', choices=('target', 'sine'), default='target',
                        help='sine: both sides, same EXTERNAL session, always return to entry openings')
    parser.add_argument('--sine-min-m', type=float, default=.01)
    parser.add_argument('--sine-max-m', type=float, default=.08)
    parser.add_argument('--period-s', type=float, default=10., help='sine period; coordinator default 10s, owner may change')
    parser.add_argument('--group', default='default')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--target-m', type=float, help='absolute total opening target; required for motion')
    parser.add_argument('--speed-m-s', type=float, default=0.0, help='host cap; 0 leaves robot configuration default')
    parser.add_argument('--torque-cmd-nm', type=float, default=None,
                        help='signed software motor-side Nm; omitted=config hold, explicit0=0')
    parser.add_argument('--duration', type=float, default=None, help='streaming seconds: sine default30, target default1')
    parser.add_argument('--body-gains', choices=('hold', 'zero'), default='hold')
    parser.add_argument('--body-pose', choices=('measured', 'zero'), default='measured',
                        help='zero uses frozen FIXED_STAND gains and real-time linear entry')
    parser.add_argument('--return-to-start', action='store_true',
                        help='after target streaming, return once to the initial measured gripper opening')
    parser.add_argument('--dry-run', action='store_true', help='read-only subscriptions, no lease or commands')
    parser.add_argument('--confirm-motion', action='store_true', help='requires separate current on-site authorization')
    args = parser.parse_args(argv)
    if args.duration is None:
        args.duration = 30.0 if args.pattern == 'sine' else body.DWELL_S
    if args.pattern == 'sine':
        if args.side != 'both' or args.speed_m_s != 0 or args.target_m is not None:
            parser.error('sine requires --side both, --speed-m-s 0 and no --target-m')
        if not math.isfinite(args.period_s) or args.period_s <= 0:
            parser.error('--period-s must be finite and positive')
        if not all(math.isfinite(v) for v in (args.sine_min_m, args.sine_max_m)) or args.sine_min_m >= args.sine_max_m:
            parser.error('sine bounds must be finite and min < max')
    elif args.side == 'both' and args.confirm_motion:
        parser.error('--side both motion requires --pattern sine')
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
    if args.confirm_motion and args.pattern == 'target' and (args.target_m is None or not math.isfinite(args.target_m)):
        parser.error('motion requires a finite --target-m')
    if _client_factory is None:
        from locomotion_aorta import GripperClient
        _client_factory = GripperClient
    if _controller_factory is None:
        from locomotion_aorta import ExternalController
        _controller_factory = ExternalController
    readonly = args.dry_run or not args.confirm_motion
    with ExitStack() as stack:
        sides = ('left', 'right') if args.side == 'both' else (args.side,)
        if readonly:
            for side in sides:
                client = stack.enter_context(_client_factory(side, group=args.group, read_only=True))
                observe(client, timeout=args.timeout)
        else:
            controller = stack.enter_context(_controller_factory(group=args.group))
            clients = [stack.enter_context(_client_factory(side, group=args.group,
                                                          node=controller.transport.node)) for side in sides]
            try:
                if args.pattern == 'sine':
                    run_sine(controller, clients, low=args.sine_min_m, high=args.sine_max_m,
                             period=args.period_s, duration=args.duration, torque_cmd_nm=args.torque_cmd_nm,
                             timeout=args.timeout, body_gains=args.body_gains, body_pose=args.body_pose)
                else:
                    run(controller, clients[0], q=args.target_m, dq=args.speed_m_s,
                        torque_cmd_nm=args.torque_cmd_nm, timeout=args.timeout,
                        duration=args.duration, body_gains=args.body_gains,
                        body_pose=args.body_pose, return_to_start=args.return_to_start)
            except KeyboardInterrupt:
                print('interrupted; CANCEL attempted; no neutral gripper command')


if __name__ == '__main__':
    main()
