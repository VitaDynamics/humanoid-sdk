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
        if (len(names) != 31 or len(set(names)) != 31 or len(info.joints) != 31
                or any(cap.body_index != i for i, cap in enumerate(info.joints))):
            raise ValueError('FIXED_STAND requires the unique public 31 roster')
        self.targets = frozenset(self.profile['joint_gains'])
        if len(self.targets) != 30 or not self.targets.issubset(names):
            raise ValueError('FIXED_STAND requires its full 30-joint profile')
        unavailable = [name for name, cap in zip(names, info.joints)
                       if name in self.targets and cap.is_mock]
        if unavailable:
            raise ValueError('FIXED_STAND targets unavailable: ' + ', '.join(unavailable))
        problems = []
        for name, cap in zip(names, info.joints):
            if name not in self.targets:
                continue
            if (cap.supported_fields & 0x19 != 0x19 or not cap.valid_limits & 1
                    or not math.isfinite(cap.q_min)
                    or not math.isfinite(cap.q_max) or cap.q_min > cap.q_max):
                problems.append(f'{name}: valid q range unavailable')
            elif not cap.q_min <= 0.0 <= cap.q_max:
                problems.append(f'{name}: zero outside [{cap.q_min}, {cap.q_max}]')
        if problems:
            raise ValueError('; '.join(problems))

    def measured(self, state):
        values = []
        positions = state.q
        if len(positions) != len(self.info.joints):
            raise ValueError('BodyState differs from Info roster')
        for name, cap, q in zip(self.info.joint_names, self.info.joints, positions):
            values.append(float(q) if name in self.targets else 0.0)
        self.check(values)
        return tuple(values)

    def check(self, values):
        if len(values) != len(self.info.joints):
            raise ValueError('target must have 31 slots')
        for name, cap, q in zip(self.info.joint_names, self.info.joints, values):
            if name in self.targets and (not math.isfinite(q) or not cap.q_min <= q <= cap.q_max):
                raise ValueError(f'{name}: q={q} outside [{cap.q_min}, {cap.q_max}]')

    def duration(self, start):
        self.check(start)
        delta = max(abs(q) for name, q in zip(self.info.joint_names, start) if name in self.targets)
        t = self.profile['transition']
        if delta <= t['small_delta_skip_threshold_q']:
            return 0.0
        return min(t['max_duration_s'], max(t['min_duration_s'], delta / t['speed_q_per_s']))

    def position(self, start, elapsed, duration):
        # Native StartTransition/TickTransition interpolate q only. PC scheduling
        # samples that same linear curve in real time at the existing SDK rate.
        alpha = 1.0 if duration == 0 else min(1.0, max(0.0, elapsed / duration))
        q = tuple(value * (1.0 - alpha) if name in self.targets else 0.0
                  for name, value in zip(self.info.joint_names, start))
        self.check(q)
        return q

    def command(self, q, state):
        self.check(q)
        commands = []
        for name, cap, value in zip(self.info.joint_names, self.info.joints, q):
            if name in self.targets:
                gains = self.profile['joint_gains'][name]
                commands.append(BodyJointCmd(q=value, dq=0.0, tau=0.0,
                                             kp=gains['kp'], kd=gains['kd']))
            else:
                # The profile does not target this fixed wire slot.
                commands.append(BodyJointCmd())
        return BodyCmd(joints=tuple(commands), action=BodyCmdAction.EXTERNAL,
                       input_state_id=state.state_id, input_state_ts_real_us=state.ts_state_real_us)
