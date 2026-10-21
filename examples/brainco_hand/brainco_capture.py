#!/usr/bin/env python3
"""Read-only BrainCo capture or offline MCAP export; no control requests."""
import argparse
import json
from pathlib import Path
import select
import sys

from locomotion_aorta.brainco_capture import capture_live, export_mcap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="new JSONL path; never overwritten")
    modes = parser.add_subparsers(dest="mode", required=True)
    live = modes.add_parser("live", help="requires current permission to connect")
    live.add_argument("--duration-s", type=float, required=True)
    live.add_argument("--group", default="default")
    live.add_argument("--queue-capacity", type=int, default=256)
    live.add_argument("--stdin-markers", action="store_true", help="line labels in observer clock, not robot clock")
    offline = modes.add_parser("mcap-export")
    offline.add_argument("input", type=Path)
    offline.add_argument("--topic-map", type=Path, required=True)
    offline.add_argument("--schema-hashes", type=Path, required=True)
    args = parser.parse_args()
    metadata = json.loads(args.metadata.read_text())
    if args.mode == "live":
        def marker_reader():
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.readline(514).strip() or None
            return None
        capture_live(args.output, metadata, args.duration_s, group=args.group, capacity=args.queue_capacity,
                     marker_reader=marker_reader if args.stdin_markers else None)
    else:
        export_mcap(args.input, args.output, metadata,
                    json.loads(args.topic_map.read_text()), json.loads(args.schema_hashes.read_text()))


if __name__ == "__main__":
    main()
