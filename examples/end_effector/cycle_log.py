"""Opt-in Body and end-effector timing; no device-output latency inference."""

from dataclasses import asdict
from importlib.metadata import version
import math
import time


def _json_value(value):
    # Invalid/unselected measurement columns can contain nonfinite values.
    # Keep their validity masks and represent unavailable numbers as JSON null.
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


class CycleRecorder:
    def __init__(self, path, metadata, *, support, monotonic=time.monotonic):
        # Fail before connecting/requesting control if the optional logging
        # dependency is absent, rather than first discovering it during CANCEL.
        import numpy as np

        self.np, self.support, self.monotonic = np, support, monotonic
        self.count = 0
        self.previous = {}
        self.metrics = {}
        self.body_reads = dict(attempts=0, returned=0, state_read_call_ns=[], feedback_age_ns=[])
        self.log = support.CycleLog(path, dict(metadata,
            schema='vita.end_effector_latency.v1',
            sdk_version=version('locomotion-aorta'),
            cycle_scope='body state read, BodyCmd publication, or selected endpoint publication attempt',
            duration_clock='local CLOCK_MONOTONIC',
            S0_source='input State.ts_state_pub_us * 1000; producer clock',
            S1_source=support.NOT_MEASURED,
            S5_source='local CLOCK_REALTIME immediately before client.publish',
            S6_source='local CLOCK_REALTIME after client.publish returns or raises',
            cross_clock_mapping=support.NOT_MEASURED,
            unavailable_numeric='null; consult the original validity masks',
            limitations='software call timing only; no admission, CAN execution, arrival or hold proof'))

    def __enter__(self):
        return self

    def describe(self, infos):
        self.log.write(dict(record_type='end_effector_info',
            endpoints={side: _json_value(asdict(info)) for side, info in infos.items()}))

    def describe_body(self, info):
        self.log.write(dict(record_type='body_info', info=_json_value(asdict(info)),
            position_unit='rad', velocity_unit='rad/s',
            read_scope='active EXTERNAL frame reads; call includes waiting, native decoding and validation'))

    def read_body(self, controller, timeout, *, phase):
        row = dict(record_type='body_state_read', stream='body', cycle=self.count, phase=phase)
        started = int(self.monotonic() * 1e9)
        row['read_started_monotonic_ns'] = started
        try:
            state = controller.wait_state(timeout)
        except BaseException as error:
            ended, realtime = int(self.monotonic()*1e9), self.support.realtime_ns()
            row.update(read_result='raised', error_type=type(error).__name__, error=str(error))
            self._finish_body_read(row, started, ended, realtime, suppress_errors=True)
            raise
        else:
            ended, realtime = int(self.monotonic()*1e9), self.support.realtime_ns()
            received = getattr(state, 'received_monotonic_ns', 0)
            row.update(self.support.state_log_fields(state))
            row.update(read_result='returned', runtime_id=state.runtime_id, state_id=state.state_id,
                state_received_monotonic_ns=received or self.support.NOT_MEASURED,
                feedback_age_ns=ended-received if received else self.support.NOT_MEASURED,
                applied_key_valid=getattr(state, 'applied_key_valid', False),
                association_source_type=getattr(state, 'association_source_type', 0))
            self._finish_body_read(row, started, ended, realtime)
            return state, row

    def _finish_body_read(self, row, started, ended, realtime, *, suppress_errors=False):
        row.update(read_finished_monotonic_ns=ended, S2=realtime, state_read_call_ns=ended-started)
        self.count += 1
        self.body_reads['attempts'] += 1
        self.body_reads['returned'] += row['read_result'] == 'returned'
        self.body_reads['state_read_call_ns'].append(row['state_read_call_ns'])
        if isinstance(row.get('feedback_age_ns'), int):
            self.body_reads['feedback_age_ns'].append(row['feedback_age_ns'])
        try:
            self.log.cycle(row)
        except BaseException:
            if not suppress_errors:
                raise

    def publish_body(self, controller, command, state, read, *, phase, target_names, required_fields):
        missing = self.support.NOT_MEASURED
        received = getattr(state, 'received_monotonic_ns', 0)
        observed = int(self.monotonic()*1e9)
        row = dict(record_type='body_cycle', stream='body', side='body', cycle=self.count,
            phase=phase, input_read_cycle=read['cycle'], runtime_id=state.runtime_id,
            state_id=state.state_id, input_state_id=command.input_state_id,
            input_state_ts_real_us=command.input_state_ts_real_us,
            state_read_call_ns=read['state_read_call_ns'], S2=read['S2'],
            S3=missing, S4=missing, L1=missing, L2=missing,
            session_id=missing, sequence=missing, publish_timing=missing,
            state_received_monotonic_ns=received or missing,
            feedback_observed_monotonic_ns=observed,
            feedback_age_ns=observed-received if received else missing,
            measured_q=_json_value([float(v) for v in state.q]),
            measured_dq=_json_value([float(v) for v in state.dq]),
            target_q=[float(j.q) for j in command.joints], target_names=target_names,
            required_fields=required_fields,
            applied_key_valid=getattr(state, 'applied_key_valid', False),
            association_source_type=getattr(state, 'association_source_type', 0))
        row.update(self.support.state_log_fields(state))
        started = int(self.monotonic()*1e9)
        row['publish_started_monotonic_ns'] = started
        row['control_interval_ns'] = started-self.previous['body'] if 'body' in self.previous else missing
        self.previous['body'] = started
        row['S5'] = self.support.realtime_ns()
        try:
            sent = controller.publish(command, target_names=target_names, required_fields=required_fields)
        except BaseException as error:
            ended, realtime = int(self.monotonic()*1e9), self.support.realtime_ns()
            row.update(publish_result='raised', error_type=type(error).__name__, error=str(error))
            self._finish(row, ended-started, realtime, suppress_errors=True)
            raise
        else:
            ended, realtime = int(self.monotonic()*1e9), self.support.realtime_ns()
            row['publish_result'] = 'returned'
            self.support.publish_log_fields(row, sent, getattr(controller, 'publish_timing', None))
            self._finish(row, ended-started, realtime)
            return sent

    def __exit__(self, kind, error, traceback):
        # The owning run has already attempted CANCEL before reaching this
        # finalization. A logging error must not replace an existing fault.
        try:
            if error is not None:
                self.log.write(dict(record_type='error', error_type=kind.__name__, error=str(error)))
            self.close()
        except BaseException:
            if error is None:
                raise
        finally:
            self.log.stream.close()
        return False

    def publish(self, guard, target, state, *, phase, rate):
        side = guard.client.side
        missing = self.support.NOT_MEASURED
        received = guard.position_received_at
        observed_ns = int(self.monotonic() * 1e9)
        receipt_ns = missing if received is None else int(received * 1e9)
        age_ns = missing if received is None else observed_ns - receipt_ns
        latest = guard.latest
        row = dict(cycle=self.count, stream='end_effector', side=side, phase=phase, requested_hz=rate,
            S0=state.ts_state_pub_us * 1000 or missing,
            S1=missing, S2=missing, S3=missing, S4=missing,
            L1=missing, L2=missing, publish_timing=missing,
            session_id=missing, sequence=missing,
            target_intent=_json_value(asdict(target)),
            position_sample_ref=asdict(state.sample_ref),
            position_state=_json_value(asdict(state)),
            latest_status=dict(state_id=latest.state_id, status_valid_mask=latest.status_valid_mask,
                ready=latest.ready, output_allowed=latest.output_allowed,
                inhibit_reason=latest.inhibit_reason, touch_enabled=latest.touch_enabled),
            feedback_observed_monotonic_ns=observed_ns,
            position_received_monotonic_ns=receipt_ns, feedback_age_ns=age_ns)
        started_ns = int(self.monotonic() * 1e9)
        row['control_interval_ns'] = (started_ns - self.previous[side]
                                      if side in self.previous else missing)
        self.previous[side] = started_ns
        row['publish_started_monotonic_ns'] = started_ns
        row['S5'] = self.support.realtime_ns()
        try:
            sent = guard.publish(target, state)
        except BaseException as error:
            ended_ns, ended_realtime_ns = int(self.monotonic() * 1e9), self.support.realtime_ns()
            row.update(publish_result='raised', error_type=type(error).__name__, error=str(error))
            self._finish(row, ended_ns - started_ns, ended_realtime_ns, suppress_errors=True)
            raise
        else:
            ended_ns, ended_realtime_ns = int(self.monotonic() * 1e9), self.support.realtime_ns()
            row.update(publish_result='returned', runtime_id=sent.runtime_id,
                layout_id=sent.layout_id, control_id=sent.control_id,
                session_id=sent.key.session_id, sequence=sent.key.sequence,
                ts_client_created_us=sent.ts_client_created_us,
                ts_client_send_us=sent.ts_client_send_us,
                submitted_target=_json_value(asdict(sent.target)),
                input_observations=[asdict(ref) for ref in sent.input_observations])
            self._finish(row, ended_ns - started_ns, ended_realtime_ns)
            return sent

    def _finish(self, row, elapsed_ns, ended_realtime_ns, *, suppress_errors=False):
        row['S6'] = ended_realtime_ns
        row['publish_call_ns'] = elapsed_ns
        received = row.get('state_received_monotonic_ns', row.get('position_received_monotonic_ns'))
        row['receipt_to_publish_return_ns'] = (
            row['publish_started_monotonic_ns'] + elapsed_ns - received
            if isinstance(received, int) else self.support.NOT_MEASURED)
        self.count += 1
        stats = self.metrics.setdefault(row['side'], dict(
            attempts=0, returned=0, publish_call_ns=[], control_interval_ns=[],
            feedback_age_ns=[], receipt_to_publish_return_ns=[]))
        stats['attempts'] += 1
        stats['returned'] += row['publish_result'] == 'returned'
        for name in ('publish_call_ns', 'control_interval_ns', 'feedback_age_ns', 'receipt_to_publish_return_ns'):
            value = row[name]
            if isinstance(value, int):
                stats[name].append(value)
        try:
            self.log.cycle(row)
        except BaseException:
            if not suppress_errors:
                raise

    def _percentiles_ms(self, samples):
        if not samples:
            return self.support.NOT_MEASURED
        return dict(zip(('p50', 'p95', 'p99', 'max'),
                        [float(value) / 1e6 for value in
                         (*self.np.percentile(samples, [50, 95, 99]), max(samples))]))

    def close(self):
        sides = {}
        for side, metrics in self.metrics.items():
            intervals = metrics['control_interval_ns']
            sides[side] = dict(attempts=metrics['attempts'], returned=metrics['returned'],
                observed_attempt_hz=(len(intervals) * 1e9 / sum(intervals)
                    if intervals and sum(intervals) > 0 else self.support.NOT_MEASURED),
                **{name.replace('_ns', '_ms'): self._percentiles_ms(metrics.get(name, []))
                   for name in ('publish_call_ns', 'control_interval_ns', 'feedback_age_ns', 'receipt_to_publish_return_ns')})
        if self.body_reads['attempts'] or 'body' in sides:
            self.log.write(dict(record_type='body_summary',
                publications=sides.pop('body', self.support.NOT_MEASURED),
                reads=dict(attempts=self.body_reads['attempts'], returned=self.body_reads['returned'],
                    state_read_call_ms=self._percentiles_ms(self.body_reads['state_read_call_ns']),
                    feedback_age_ms=self._percentiles_ms(self.body_reads['feedback_age_ns'])),
                scope='active BodyState reads and BodyCmd calls; pure decode and physical latency not isolated'))
        self.log.write(dict(record_type='end_effector_summary', sides=sides,
            scope='local publication attempts and cached position age; includes logging effect on cadence'))
        self.log.close()
