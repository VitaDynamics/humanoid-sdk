"""Exact-ID reports for synthetic production endpoints; unavailable stages stay null."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def stats(values):
    values = sorted(values)
    if not values:
        return {"status": "NOT_MEASURED", "n": 0}
    if any(not math.isfinite(v) or v < 0 for v in values):
        raise ValueError("negative/nonfinite measurement")
    def q(p):
        x = (len(values) - 1) * p
        lo, hi = math.floor(x), math.ceil(x)
        return values[lo] + (values[hi] - values[lo]) * (x - lo)
    return {"n": len(values), "mean_ms": statistics.fmean(values), "p50_ms": q(.5),
            "p95_ms": q(.95), "p99_ms": q(.99), "max_ms": values[-1]}


def rows(path):
    with Path(path).open(newline="") as stream:
        return [{k: int(v) for k, v in row.items()} for row in csv.DictReader(stream)]


def index(records, keys):
    out = {}
    for record in records:
        key = tuple(record[k] for k in keys)
        if any(v < 0 or v >= 2**64 for v in key) or key in out:
            raise ValueError("duplicate/out-of-range exact identity: " + repr(key))
        out[key] = record
    return out


def frequency(records, key):
    times = [r[key] for r in records]
    if len(times) < 2:
        return None
    if any(t <= 0 for t in times) or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("non-increasing frequency clock")
    return (len(times) - 1) * 1e9 / (times[-1] - times[0])


def analyze(directory, client_log=None):
    directory = Path(directory)
    sent_rows = rows(directory / "states.csv")
    received_rows = rows(directory / "received.csv")
    applied_rows = rows(directory / "applied.csv")
    sent = index(sent_rows, ("state_id",))
    valid_rows = [r for r in received_rows if r["valid"] == 1]
    accepted = index(valid_rows, ("session_id", "sequence"))
    applied = index(applied_rows, ("session_id", "sequence"))
    if not applied.keys() <= accepted.keys():
        raise ValueError("orphan applied identity")
    stage_names = ("S0_to_L1", "L1_to_G", "S0_to_G", "S0_to_S1", "S1_to_S2",
                   "S2_to_S3", "S3_to_S4", "S4_to_S5", "S5_to_S6", "S6_to_L1", "S6_to_G")
    durations = {name: [] for name in stage_names}
    joined, unmatched, clock_errors = [], [], []
    client = {}
    if client_log:
        for line in Path(client_log).read_text().splitlines():
            r = json.loads(line)
            if r.get("record_type") == "cycle_timing" or not all(k in r for k in ("session_id", "sequence", "input_state_id", "input_state_ts_real_us")):
                continue
            key = (r["session_id"], r["sequence"])
            if key in client:
                raise ValueError("duplicate client cycle identity")
            client[key] = r
    for key, cmd in accepted.items():
        source = sent.get((cmd["input_state_id"],))
        if source is None or source["state_real_us"] != cmd["input_state_ts_real_us"]:
            unmatched.append({"session_id": key[0], "sequence": key[1], "reason": "input state identity/stamp not in this run"})
            continue
        stamps = {"S0": source["publish_ns"], "L1": cmd["received_ns"]}
        if key in applied:
            stamps["G"] = applied[key]["first_output_ns"]
        c = client.get(key)
        if c:
            if c.get("input_state_id") != cmd["input_state_id"] or c.get("input_state_ts_real_us") != cmd["input_state_ts_real_us"]:
                raise ValueError("client correlation disagrees with wire input state")
            t = c.get("timing_wall_ns", {})
            # S3 must be observation-finished/inference-start, not whole policy entry.
            aliases = {"S1": "state_received_wall_ns", "S2": "state_decode_finished_wall_ns",
                       "S3": "inference_started_wall_ns", "S4": "inference_finished_wall_ns",
                       "S5": "sdk_publish_started_wall_ns", "S6": "sdk_returned_wall_ns"}
            for point, name in aliases.items():
                value = t.get(name)
                if isinstance(value, int) and not isinstance(value, bool) and value > 0:
                    stamps[point] = value
        row = {"session_id": key[0], "sequence": key[1], "input_state_id": cmd["input_state_id"], **stamps}
        for name in stage_names:
            begin, end = name.split("_to_")
            if begin in stamps and end in stamps:
                delta = stamps[end] - stamps[begin]
                if delta < 0:
                    clock_errors.append({"session_id": key[0], "sequence": key[1], "segment": name, "delta_ns": delta})
                else:
                    durations[name].append(delta / 1e6)
        joined.append(row)
    echo_keys = {(r["applied_session_id"], r["applied_sequence"]) for r in sent_rows if r["applied_session_id"] and r["applied_sequence"]}
    if not echo_keys <= applied.keys():
        raise ValueError("state reports an unapplied command")
    for r in sent_rows:
        key = (r["applied_session_id"], r["applied_sequence"])
        if key in applied and r["ts_first_output_us"] != applied[key]["first_output_ns"] // 1000:
            raise ValueError("state first-output echo differs from applied record")
    intervals = [(b["received_ns"] - a["received_ns"]) / 1e6 for a, b in zip(valid_rows, valid_rows[1:])]
    missing = sorted(accepted.keys() - applied.keys())
    metadata = json.loads((directory / "mock-result.json").read_text())
    measured = bool(accepted) and bool(sent) and bool(applied)
    complete = measured and not clock_errors and not unmatched and not metadata.get("capture_overflow") and not metadata.get("info_errors") and not metadata.get("endpoint_rc")
    sequence_lag = [r["sequence"] - sent[(r["input_state_id"],)]["applied_sequence"] for r in valid_rows
                    if (r["input_state_id"],) in sent and sent[(r["input_state_id"],)]["applied_session_id"] == r["session_id"]
                    and 0 < sent[(r["input_state_id"],)]["applied_sequence"] <= r["sequence"]]
    first_echo = {}
    for r in sent_rows:
        k = (r["applied_session_id"], r["applied_sequence"])
        if k in applied:
            first_echo.setdefault(k, r["publish_ns"])
    echo_delay = [(stamp - applied[k]["first_output_ns"]) / 1e6 for k, stamp in first_echo.items()]
    if any(v < 0 for v in echo_delay):
        raise ValueError("state echo precedes synthetic output")
    cycles = [r for r in client.values() if r.get("cycle_start_monotonic_ns", 0) > 0]
    cycle_intervals = [(b["cycle_start_monotonic_ns"] - a["cycle_start_monotonic_ns"]) / 1e6 for a, b in zip(cycles, cycles[1:])]
    auxiliary = None
    if (directory / "auxiliary.json").is_file():
        raw = json.loads((directory / "auxiliary.json").read_text())
        streams = {}
        for r in raw["records"]:
            if r["kind"] == "publish":
                streams.setdefault(r["side"] + "/" + r["role"], []).append(r)
        auxiliary = {"synthetic": True, "streams": {k: {"n": len(v), "hz": frequency(v, "wall_ns")} for k, v in streams.items()},
                     "commands_observed": sum(r["kind"] == "command" for r in raw["records"]),
                     "commands_rejected": sum(r["kind"] == "command" and not r["accepted"] for r in raw["records"]),
                     "sequence_gaps_observed": sum(r.get("sequence_gap") or 0 for r in raw["records"]),
                     "joy_frames_published": sum(r["kind"] == "joy" for r in raw["records"]),
                     "limits": raw["limits"]}
    return {"status": "MEASURED_SYNTHETIC_ONLY" if complete else "INCOMPLETE_OR_CLOCK_ANOMALY",
            "clock": "same-host CLOCK_REALTIME ns; no offsets or cross-clock conversion",
            "counts": {"states": len(sent), "commands_received": len(received_rows), "commands_accepted": len(accepted),
                       "commands_rejected": len(received_rows) - len(valid_rows), "commands_applied": len(applied),
                       "applied_echo_observed": len(echo_keys), "accepted_not_applied": len(missing)},
            "accepted_not_applied": [list(k) for k in missing], "unmatched_input_states": unmatched,
            "clock_anomalies": clock_errors, "segments": {k: stats(v) for k, v in durations.items()},
            "mock_state_hz": frequency(sent_rows, "publish_ns"), "accepted_command_hz": frequency(valid_rows, "received_ns"),
            "command_arrival_interval": stats(intervals), "joined": joined, "mock": metadata,
            "client_cycle_hz": frequency(cycles, "cycle_start_monotonic_ns"), "client_cycle_start_interval": stats(cycle_intervals),
            "applied_echo_delay": stats(echo_delay),
            "applied_sequence_lag": {"n": len(sequence_lag), "unit": "commands", "values": sequence_lag,
                                     "max": max(sequence_lag) if sequence_lag else None},
            "auxiliary": auxiliary,
            "frequency_policy": "Report observed rates/counts/intervals; no new pass threshold. Arrival intervals are not client tick-start intervals.",
            "unmeasured": ["real_locomotion", "CAN", "hardware_output", "client_tick_starts_without_client_trace"],
            "G_semantics": "first configurable synthetic apply tick; never real robot/motor output",
            "overlap": "publish-call and native substage durations overlap transport; do not add percentiles"}


def write_report(directory, client_log=None):
    directory = Path(directory)
    result = analyze(directory, client_log)
    (directory / "report.json").write_text(json.dumps(result, indent=2) + "\n")
    text = ["# Production-topic Mock — synthetic measurements", "", result["status"], "",
            "G is a synthetic apply tick; ARM execution, real locomotion and hardware are separate evidence.", "",
            "| Segment | N | mean ms | p50 ms | p95 ms | p99 ms | max ms |", "|---|---:|---:|---:|---:|---:|---:|"]
    for name, s in result["segments"].items():
        text.append("| " + name + " | " + str(s["n"]) + " | " + " | ".join(str(s.get(k, "NOT_MEASURED")) for k in ("mean_ms", "p50_ms", "p95_ms", "p99_ms", "max_ms")) + " |")
    text += ["", "Counts: " + json.dumps(result["counts"]), "", "State Hz: " + str(result["mock_state_hz"]),
             "Command arrival Hz: " + str(result["accepted_command_hz"]), "", result["frequency_policy"]]
    (directory / "report.md").write_text("\n".join(text) + "\n")
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--client-log", type=Path)
    args = p.parse_args()
    print(json.dumps(write_report(args.output, args.client_log)["counts"]))
