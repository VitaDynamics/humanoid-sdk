"""Example-only FIXED_STAND target, from the recorded robot source profile."""
import json
import math
from pathlib import Path

from locomotion_aorta import BodyCmd, BodyJointCmd, BodyCmdAction


class FixedStandPose:
    def __init__(self, info):
        self.profile = json.loads(Path(__file__).with_name('fixed_stand_profile.json').read_text())
        self.info = info
        names = tuple(info.joint_names)
        if len(names) != 31 or len(set(names)) != 31 or len(info.joints) != 31:
            raise ValueError('FIXED_STAND requires the unique public31 roster')
        problems = []
        for name, cap in zip(names, info.joints):
            if not cap.populated:
                continue
            if name not in self.profile['joint_gains']:
                problems.append(f'{name}: no frozen FIXED_STAND gain mapping')
            if (not cap.valid_limits & 1 or not math.isfinite(cap.q_min)
                    or not math.isfinite(cap.q_max) or cap.q_min > cap.q_max):
                problems.append(f'{name}: valid q range unavailable')
            elif not cap.q_min <= 0.0 <= cap.q_max:
                problems.append(f'{name}: zero outside [{cap.q_min}, {cap.q_max}]')
        if problems:
            raise ValueError('; '.join(problems))

    def measured(self, state):
        values = []
        if len(state.joints) != len(self.info.joints):
            raise ValueError('BodyState differs from Info roster')
        for name, cap, sample in zip(self.info.joint_names, self.info.joints, state.joints):
            if sample.populated != cap.populated or sample.body_index != cap.body_index:
                raise ValueError(f'{name}: BodyState population/index differs from Info')
            values.append(sample.q if cap.populated else 0.0)
        self.check(values)
        return tuple(values)

    def check(self, values):
        if len(values) != len(self.info.joints):
            raise ValueError('target must have31 slots')
        for name, cap, q in zip(self.info.joint_names, self.info.joints, values):
            if cap.populated and (not math.isfinite(q) or not cap.q_min <= q <= cap.q_max):
                raise ValueError(f'{name}: q={q} outside [{cap.q_min}, {cap.q_max}]')

    def duration(self, start):
        self.check(start)
        delta = max((abs(q) for q, cap in zip(start, self.info.joints) if cap.populated), default=0.0)
        t = self.profile['transition']
        if delta <= t['small_delta_skip_threshold_q']:
            return 0.0
        return min(t['max_duration_s'], max(t['min_duration_s'], delta / t['speed_q_per_s']))

    def position(self, start, elapsed, duration):
        # Native StartTransition/TickTransition interpolate q only. PC scheduling
        # samples that same linear curve in real time at the existing SDK rate.
        alpha = 1.0 if duration == 0 else min(1.0, max(0.0, elapsed / duration))
        q = tuple(value * (1.0 - alpha) if cap.populated else 0.0
                  for value, cap in zip(start, self.info.joints))
        self.check(q)
        return q

    def command(self, q, state, cmd_id):
        self.check(q)
        commands = []
        for name, cap, value in zip(self.info.joint_names, self.info.joints, q):
            if cap.populated:
                gains = self.profile['joint_gains'][name]
                commands.append(BodyJointCmd(q=value, dq=0.0, tau=0.0,
                                             kp=gains['kp'], kd=gains['kd']))
            else:
                # Fixed public31 wire placeholder; server populated mask prevents
                # physical output. Never use an uninstalled slot's q or gains.
                commands.append(BodyJointCmd())
        return BodyCmd(cmd_id=cmd_id, joints=tuple(commands), action=BodyCmdAction.EXTERNAL,
                       input_state_id=state.state_id, input_state_ts_real_us=state.ts_state_real_us)
