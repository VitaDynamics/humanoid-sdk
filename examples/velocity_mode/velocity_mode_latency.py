"""TONIGHT/071 JSONL contract. All S/L points use CLOCK_REALTIME nanoseconds."""

import json
import time


NOT_MEASURED = "NOT_MEASURED"


def realtime_ns() -> int:
    return time.time_ns()


class CycleLog:
    def __init__(self, path, metadata: dict) -> None:
        self.stream = open(path, "x", encoding="utf-8")
        self.overheads: list[int] = []
        self.write({"record_type": "run", "schema": "vita.velocity_mode_latency.v1",
                    "clock": "CLOCK_REALTIME", "started_ns": realtime_ns(), **metadata})

    def write(self, record: dict) -> int:
        started = realtime_ns()
        self.stream.write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")
        self.stream.flush()
        return realtime_ns() - started

    def cycle(self, record: dict) -> None:
        # A second record reports the cost of serializing/writing/flushing the
        # cycle record; it cannot be inserted into the already-written record.
        overhead = self.write({"record_type": "cycle", **record})
        self.overheads.append(overhead)
        extra = self.write({"record_type": "logging_overhead", "cycle": record["cycle"],
                           "cycle_record_write_ns": overhead})
        self.overheads[-1] += extra

    def close(self) -> None:
        import numpy as np

        self.write({"record_type": "logging_summary", "finished_ns": realtime_ns(),
                    "clock": "CLOCK_REALTIME",
                    "scope": "cycle and overhead JSON serialization/write/flush; excludes this summary",
                    "N": len(self.overheads),
                    "ms": dict(zip(("p50", "p95", "p99", "max"),
                                   [float(v) / 1e6 for v in
                                    (*np.percentile(self.overheads, [50, 95, 99]), max(self.overheads))]))
                    if self.overheads else NOT_MEASURED})
        self.stream.close()
