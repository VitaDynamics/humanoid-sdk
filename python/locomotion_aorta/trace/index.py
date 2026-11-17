"""Disk-backed offline index. No database or analysis runs in the recorder."""
from contextlib import AbstractContextManager
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import tempfile

from .reader import Reader


def file_identity(path):
    path = Path(path)
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return dict(name=path.name, bytes=path.stat().st_size, sha256=digest.hexdigest())


def uint_text(value):
    # Fixed-width decimal retains unsigned 64-bit ordering in SQLite. SQLite
    # INTEGER is signed and JavaScript Number cannot represent these identities.
    return None if value is None else f'{value:020d}'


class TraceIndex(AbstractContextManager):
    """Bounded input chunks and SQLite page cache; sorting spills to disk."""
    def __init__(self, path, *, directory=None):
        self._temp = tempfile.TemporaryDirectory(prefix='ftrace-index-', dir=directory)
        try:
            self.source = file_identity(path)
            self.db = sqlite3.connect(str(Path(self._temp.name) / 'index.sqlite'))
            self.db.execute('PRAGMA cache_size=-4096')
            self.db.execute('PRAGMA temp_store=FILE')
            self.db.execute('''CREATE TABLE events (
                id INTEGER PRIMARY KEY, producer INTEGER, sequence TEXT, kind INTEGER,
                stage TEXT, stream INTEGER, outcome INTEGER, stamp TEXT, duration TEXT,
                attempt TEXT, span TEXT, inference TEXT, reference TEXT, row_json TEXT)''')
            reader = Reader(path)
            for row in reader:
                nonfinite = {str(i): repr(v) for i, v in enumerate(row['values']) if not math.isfinite(v)}
                if nonfinite:
                    row['nonfinite_values'] = nonfinite
                    row['values'] = [v if math.isfinite(v) else None for v in row['values']]
                name = reader.integrity.run_header['stages'][row['stage'] - 1] if row['stage'] else '<unknown>'
                self.db.execute('INSERT INTO events VALUES (NULL,?,?,?,?,?,?,?,?,?,?,?,?,?)', (
                    row['producer'], uint_text(row['sequence']), row['kind'], name,
                    row['stream'], row['outcome'], uint_text(row['timestamp_us']),
                    uint_text(row['duration_us']), uint_text(row['attempt_id']),
                    uint_text(row['span_id']), uint_text(row['inference_id']),
                    uint_text(row['reference_id']), json.dumps(row, allow_nan=False)))
            self.integrity = asdict(reader.integrity)
            self.header = reader.integrity.run_header or {}
            if self.source != file_identity(path):
                raise ValueError('source recording changed while indexing')
            self.db.executescript('''
                CREATE INDEX event_attempt ON events(producer,attempt,id);
                CREATE INDEX event_reference ON events(attempt,kind);
                CREATE INDEX event_span ON events(producer,span,kind);
                CREATE INDEX event_stage ON events(stage,stream,outcome,duration);
                CREATE INDEX event_stamp ON events(stamp,id);
                CREATE INDEX event_inference ON events(inference,id);
            ''')
            self.db.commit()
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def rows(self, where='1', parameters=()):
        for event_id, raw in self.db.execute(
                'SELECT id,row_json FROM events WHERE ' + where + ' ORDER BY id', parameters):
            yield event_id, json.loads(raw)

    def definitions(self):
        """Reassemble bounded cold metadata with exact producer/attempt identity."""
        for producer, attempt, kind in self.db.execute(
                'SELECT DISTINCT producer,attempt,kind FROM events WHERE kind IN (2,13,14) ORDER BY id'):
            raw, expected, issue = bytearray(), None, None
            for _, row in self.rows('producer=? AND attempt=? AND kind=?', (producer, attempt, kind)):
                words = row['integers']
                if len(words) < 2:
                    issue = 'missing_metadata_chunk_header'
                    break
                offset, total = words[:2]
                if total > 256*1024 or offset != len(raw) or (expected is not None and total != expected):
                    issue = 'metadata_chunk_gap_overlap_or_size'
                    break
                expected = total
                payload = b''.join(int(w).to_bytes(8, 'little') for w in words[2:])
                raw.extend(payload[:min(216, total-offset)])
            value = None
            if issue is None:
                if len(raw) != expected:
                    issue = 'incomplete_metadata'
                else:
                    try:
                        value = json.loads(raw)
                    except (ValueError, UnicodeError):
                        issue = 'invalid_metadata_json'
            yield dict(producer=producer, attempt_id=int(attempt), kind=kind, value=value, issue=issue)

    def __exit__(self, *_):
        if hasattr(self, 'db'):
            self.db.close()
        self._temp.cleanup()
