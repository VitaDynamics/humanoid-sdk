#!/usr/bin/env python3
"""Read one new EE sample on a publisher-free borrowed connection.

Development candidate: requires the matching new EE pack, not public0.1.5.
No ownership request, motion publish, touch switch or device discovery writes.
"""
import argparse
from contextlib import closing


def run_once(client, *, timeout, include_touch=False, emit=print):
    from locomotion_aorta.end_effector_types import Field, PositionDomain

    info = client.wait_info(timeout)
    state = client.wait_state(timeout)
    if (state.runtime_id, state.layout_id, state.side) != (info.runtime_id, info.layout_id, info.side):
        raise ValueError('EE State runtime_id/layout/side changed while reading; rerun for matching Info')
    touch = client.wait_touch(timeout) if include_touch else None
    if touch is not None and (touch.runtime_id, touch.layout_id, touch.side) != (
            info.runtime_id, info.layout_id, info.side):
        raise ValueError('EE Touch runtime_id/layout/side changed while reading; rerun for matching Info')
    emit(f'side={client.side} runtime_id={state.runtime_id} layout={state.layout_id} '
         f'state_id={state.state_id} sample_status={state.sample_status}')
    for i, name in enumerate(info.axis_names):
        # valid_fields is authoritative: a placeholder zero is not a reading.
        unit = {PositionDomain.JOINT_ANGLE: 'rad', PositionDomain.TOTAL_OPENING: 'm',
                PositionDomain.MOTOR_SHAFT: 'rad'}.get(info.position_domain[i], 'unknown')
        pos = f'{state.pos[i]:.6f} {unit}' if state.valid_fields[i] & Field.POS else 'n/a'
        fault = str(state.error_code[i]) if state.valid_fields[i] & Field.ERROR_CODE else 'unknown'
        emit(f'axis[{i}] {name} pos={pos} error_code={fault} mock={info.is_mock[i]}')
    if touch is not None:
        emit(f'side={client.side} runtime_id={touch.runtime_id} layout={touch.layout_id} '
             f'touch_state_id={touch.touch_state_id} payload_bytes={len(touch.payload)} '
             f'region_validity={[stamp.validity for stamp in touch.region_stamps]}')


def main(argv=None):
    parser = argparse.ArgumentParser(description='Read new EE Info/State/optional Touch; no control publishers')
    parser.add_argument('--side', choices=('left', 'right'), required=True)
    parser.add_argument('--group', default='default')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--touch', action='store_true')
    args = parser.parse_args(argv)
    from locomotion_aorta import PullStateSource
    from locomotion_aorta.end_effector import EndEffectorClient
    with closing(PullStateSource(group=args.group)) as connection:
        connection.get_robot_info(args.timeout)
        with EndEffectorClient(args.side, connection=connection, read_only=True) as client:
            run_once(client, timeout=args.timeout, include_touch=args.touch)


if __name__ == '__main__':
    main()
