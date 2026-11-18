"""Cold CLI options for the single SDK Trace recording format."""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import uuid


def _enabled(value):
    if value not in ('on', 'off'):
        raise argparse.ArgumentTypeError('trace must be on or off')
    return value == 'on'


def add_trace_arguments(parser):
    parser.add_argument('--trace', type=_enabled, metavar='{on,off}', default=False,
                        help='enable FBS recording; default off (false)')
    parser.add_argument('--trace-output', type=Path,
                        help='new .ftrace file; default /tmp/sdk-trace-<UTC timestamp>-<unique id>.ftrace')


def resolve_trace_output(parser, args):
    if not args.trace:
        if args.trace_output is not None:
            parser.error('--trace-output requires --trace on')
        return None
    if args.trace_output is None:
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        args.trace_output = Path('/tmp') / f'sdk-trace-{stamp}-{os.getpid()}-{uuid.uuid4().hex[:8]}.ftrace'
    if args.trace_output.suffix != '.ftrace':
        parser.error('--trace-output must end in .ftrace')
    return args.trace_output
