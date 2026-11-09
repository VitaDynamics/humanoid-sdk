"""Bounded terminal display of already-observed feedback and submitted intent."""
from collections import deque
import math
import re
import shutil
import sys
import threading
import time

from locomotion_aorta.end_effector_types import Field, SampleStatus


class LiveTable:
    """No SDK getters, subscriptions, control calls, sleeps or worker threads."""

    def __init__(self, *, stream=None, monotonic=time.monotonic, refresh_hz=5.0):
        if not math.isfinite(refresh_hz) or refresh_hz <= 0:
            raise ValueError('display refresh rate must be finite and positive')
        self.stream = stream if stream is not None else (
            sys.stderr if sys.stderr.isatty() else sys.stdout)
        self.monotonic = monotonic
        self.period = 1.0 / refresh_hz
        self.ansi = self.stream.isatty()
        self._lock = threading.Lock()
        self._render_thread = threading.get_ident()
        self._samples = {}
        self._targets = {}
        self._events = deque(maxlen=3)
        self._body_stage = self._leg = '--'
        self._body_sequence = self._body_error = '--'
        self._control = '--'
        self._next_refresh = float('-inf')
        self._rows = 0
        self._closed = False

    def __enter__(self):
        if self.ansi:
            self.stream.write('\x1b[?25l')
            self.stream.flush()
        return self

    def __exit__(self, kind, error, _traceback):
        if error is not None:
            with self._lock:
                self._events.append(f'ERROR: {error}')
        try:
            self.refresh(force=True)
        finally:
            self._closed = True
            if self.ansi:
                self.stream.write('\x1b[?25h')
                self.stream.flush()
        if error is not None:
            # Preserve the complete original diagnostic outside the compact table.
            self.stream.write(f'ERROR: {error}\n')
            self.stream.flush()

    def observe_state(self, side, info, state):
        with self._lock:
            self._samples[side] = (info, state)

    def submitted(self, side, target, key):
        with self._lock:
            self._targets[side] = (target, key)
        if self.ansi:
            self.refresh()

    def __call__(self, message):
        line = str(message)
        force = False
        with self._lock:
            if '/control/' in line:
                match = re.search(r"current_action='([^']+)'.*?ExternalPhase\.([A-Z_]+)", line)
                if match:
                    self._control = '/'.join(match.groups())
                # Callbacks only store text; they never render or read SDK objects.
                return
            if line.startswith('BODY stage='):
                match = re.search(r'stage=([^ ]+)', line)
                stage = match.group(1) if match else '--'
                force = stage != self._body_stage
                self._body_stage = stage
                match = re.search(r'sequence=(\d+)', line)
                self._body_sequence = match.group(1) if match else '--'
            elif line.startswith('BODY_HOLD '):
                match = re.search(r'max_abs_q_error_rad=([^ ]+)', line)
                if match:
                    self._body_error = match.group(1)
            elif line.startswith('EE leg='):
                match = re.search(r'leg=([^ ]+)', line)
                leg = match.group(1) if match else '--'
                force = leg != self._leg
                self._leg = leg
            elif ': ref=' in line or ': Info=' in line or ': Touch=' in line:
                pass
            elif line.startswith('ZERO_DWELL observation_only='):
                pass
            else:
                self._events.append(line)
                force = True
        # Redirected output contains phase/event snapshots, never per-frame logs.
        if self.ansi or force:
            self.refresh(force=force and not self.ansi)

    @staticmethod
    def _number(values, masks, index, bit):
        if values is None or not masks[index] & bit:
            return '--'
        value = values[index]
        return f'{value:.4f}' if math.isfinite(value) else '--'

    @staticmethod
    def _flag(state, name, bit):
        if not state.status_valid_mask & bit:
            return '?'
        return 'yes' if getattr(state, name) else 'no'

    def refresh(self, *, force=False):
        # Callback threads may update observations, but never format or write.
        if threading.get_ident() != self._render_thread:
            return
        now = self.monotonic()
        with self._lock:
            if self._closed or (not force and now < self._next_refresh):
                return
            self._next_refresh = now + self.period
            samples = dict(self._samples)
            targets = dict(self._targets)
            events = tuple(self._events)
            lines = [f'End effectors | control {self._control} | phase {self._leg}',
                     f'Body {self._body_stage} | sequence {self._body_sequence} | '
                     f'hold max error {self._body_error} rad']
        for side, (_, state) in sorted(samples.items()):
            reason = str(state.inhibit_reason) if state.status_valid_mask & 4 else '?'
            key = targets[side][1].sequence if side in targets else '--'
            lines.append(f'{side}: sample {state.state_id} | ready {self._flag(state, "ready", 1)} | '
                         f'output {self._flag(state, "output_allowed", 2)} | inhibit {reason} | cmd {key}')
        heading = f'{"Side":<6}{"Axis":<14}{"Target":>10}{"Position":>10}{"Velocity":>10}{"Current A":>10}{"Error":>8}'
        lines += [heading, '-' * len(heading)]
        for side, (info, state) in sorted(samples.items()):
            target = targets[side][0] if side in targets else None
            valid = state.sample_status == SampleStatus.OK
            for index, name in enumerate(info.axis_names):
                intended = '--' if target is None else self._number(
                    target.pos, target.command_fields, index, Field.POS)
                values = [self._number(getattr(state, name), state.valid_fields, index, bit)
                          if valid else '--' for name, bit in (
                              ('pos', Field.POS), ('vel', Field.VEL), ('current', Field.CURRENT))]
                error = str(state.error_code[index]) if valid and state.valid_fields[index] & Field.ERROR_CODE else '--'
                lines.append(f'{side:<6}{name[:14]:<14}{intended:>10}'
                             + ''.join(f'{value:>10}' for value in values) + f'{error:>8}')
        lines.append('SI units from Info (BrainCo: rad, rad/s); targets are intent, not arrival.')
        lines.extend('> ' + event.replace('\n', ' ') for event in events)
        width = max(20, shutil.get_terminal_size((100, 30)).columns - 1)
        if self.ansi and self._rows:
            self.stream.write(f'\x1b[{self._rows}A')
        count = max(self._rows, len(lines)) if self.ansi else len(lines)
        for index in range(count):
            line = lines[index][:width] if index < len(lines) else ''
            self.stream.write(('\r\x1b[2K' if self.ansi else '') + line + '\n')
        self._rows = count
        self.stream.flush()
