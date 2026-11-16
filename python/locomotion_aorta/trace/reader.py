"""Streaming, bounded, checksum- and native-verifier-checked FTRACE13 input."""
from dataclasses import dataclass, field
import json
from pathlib import Path
import struct
import zlib

from .session import _metadata, _native

MAGIC = b'FTRACE13'
HEADER = struct.Struct('<IIIB3xQQ')
MAX_BLOCK_BYTES = 4 * 1024 * 1024


@dataclass
class Integrity:
    complete: bool = False
    blocks: int = 0
    events: int = 0
    valid_bytes: int = 8
    issues: list = field(default_factory=list)
    run_header: dict | None = None
    run_end: dict | None = None


class Reader:
    def __init__(self, path):
        self.path = Path(path)
        self.integrity = Integrity()

    def __iter__(self):
        status = self.integrity = Integrity()
        native = _native()
        sequences = {}
        chunk_sequence = 0
        with self.path.open('rb') as source:
            if source.read(8) != MAGIC:
                raise ValueError('not a FTRACE13 container')
            while True:
                header = source.read(HEADER.size)
                if not header:
                    break
                if len(header) != HEADER.size:
                    status.issues.append('truncated_block_header')
                    break
                size, raw_size, crc, codec, sequence, base = HEADER.unpack(header)
                if (not 0 < size <= MAX_BLOCK_BYTES or not 0 < raw_size <= MAX_BLOCK_BYTES
                        or codec not in (0, 1) or sequence != chunk_sequence
                        or header[13:16] != b'\0\0\0'):
                    status.issues.append('invalid_block_header')
                    break
                encoded = source.read(size)
                if len(encoded) != size:
                    status.issues.append('truncated_block_payload')
                    break
                try:
                    if status.run_end is not None:
                        raise ValueError('data after run end')
                    if codec:
                        import zstandard
                        raw = zstandard.ZstdDecompressor().decompress(encoded, max_output_size=raw_size)
                    else:
                        raw = encoded
                    if len(raw) != raw_size or zlib.crc32(raw) != crc:
                        raise ValueError('block length/checksum mismatch')
                    block = native.decode_chunk(raw)
                    if (block['sequence'], block['base_us']) != (sequence, base):
                        raise ValueError('container/FlatBuffer identity mismatch')
                    metadata = json.loads(block['metadata']) if block['metadata'] else None
                    if metadata:
                        _metadata(metadata)
                        if metadata.get('record_type') == 'run_header':
                            if status.run_header is not None or sequence != 0:
                                raise ValueError('duplicate/late run header')
                            status.run_header = metadata
                        elif metadata.get('record_type') == 'run_end':
                            if status.run_end is not None:
                                raise ValueError('duplicate run end')
                            status.run_end = metadata
                    if status.run_header is None:
                        raise ValueError('missing run header')
                    if status.run_end is not None and block['events']:
                        raise ValueError('events after run end')
                    for row in block['events']:
                        producer, sequence_number = row['producer'], row['sequence']
                        if producer >= len(status.run_header['producers']):
                            raise ValueError('unregistered producer')
                        if row['stage'] > len(status.run_header['stages']):
                            raise ValueError('unregistered stage')
                        previous = sequences.get(producer, -1)
                        if sequence_number <= previous:
                            raise ValueError('duplicate/reordered producer sequence')
                        if sequence_number != previous + 1:
                            status.issues.append('producer_sequence_gap')
                        sequences[producer] = sequence_number
                    # A bad block contributes no partial records.
                except Exception as error:
                    status.issues.append('invalid_block:' + str(error))
                    break
                chunk_sequence += 1
                status.blocks += 1
                status.valid_bytes = source.tell()
                for row in block['events']:
                    status.events += 1
                    yield row
        if status.run_end is None:
            status.issues.append('missing_run_end')
        else:
            producers = status.run_end.get('producers', [])
            if len(producers) != len(status.run_header['producers']):
                status.issues.append('health_producer_count_mismatch')
            for counts in producers:
                if (not counts['counters_valid'] or counts['dropped_events'] or
                        counts['unpersisted_events'] or counts['wrong_thread_events'] or
                        counts['invalid_events']):
                    status.issues.append('incomplete_producer')
                if (counts['attempted_events'] != counts['enqueued_events'] + counts['dropped_events'] or
                        counts['enqueued_events'] != counts['persisted_events'] + counts['unpersisted_events']):
                    status.issues.append('health_ledger_mismatch')
            if sum(c['persisted_events'] for c in producers) != status.events:
                status.issues.append('persisted_count_mismatch')
        status.complete = not status.issues
