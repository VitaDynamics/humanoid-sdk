"""Bounded MCAP input using installed, hash-matched public message contracts.

An observed command or application status is not a physical-output receipt.
"""
from dataclasses import asdict, replace
import hashlib
import importlib
import io
import json
import zlib

from ..body_types import (
    BODY_CMD_PROTOCOL_VERSION, BODY_CMD_MOTOR_THETA_PROTOCOL_VERSION, BodyCmdAction,
)
from .index import file_identity, uint_text

MAX_RECORD = 16 * 1024 * 1024
MAX_PAYLOAD = 1024 * 1024
CONTRACTS = {
    'lowlevel.BodyCmd': ('body_cmd', BODY_CMD_PROTOCOL_VERSION),
    'lowlevel.BodyState': ('body_state', 3),
    'lowlevel.ee.EndEffectorCmd': ('end_effector_cmd', 2),
    'lowlevel.ee.EndEffectorState': ('end_effector_state', 2),
    'lowlevel.ee.EndEffectorInfo': ('end_effector_info', 1),
    'locomotion_sdk.ControlStatus': ('control_status', 4),
}


def _records(source):
    from mcap.records import Chunk
    from mcap.stream_reader import StreamReader, breakup_chunk
    for record in StreamReader(source, emit_chunks=True, validate_crcs=True,
                               record_size_limit=MAX_RECORD).records:
        if not isinstance(record, Chunk):
            yield record
            continue
        if record.uncompressed_size > MAX_RECORD:
            raise ValueError('MCAP uncompressed chunk exceeds 16 MiB analysis bound')
        if record.compression == 'zstd':
            import zstandard
            with zstandard.ZstdDecompressor().stream_reader(io.BytesIO(record.data)) as stream:
                raw = stream.read(MAX_RECORD+1)
        elif record.compression == 'lz4':
            import lz4.frame
            decoder = lz4.frame.LZ4FrameDecompressor()
            raw = decoder.decompress(record.data, max_length=MAX_RECORD+1)
            if not decoder.eof:
                raise ValueError('incomplete/oversized lz4 MCAP chunk')
        elif not record.compression:
            raw = record.data
        else:
            raise ValueError('unsupported MCAP compression: ' + record.compression)
        if len(raw) != record.uncompressed_size or len(raw) > MAX_RECORD:
            raise ValueError('MCAP chunk size mismatch')
        if record.uncompressed_crc and zlib.crc32(raw) != record.uncompressed_crc:
            raise ValueError('MCAP chunk CRC mismatch')
        yield from breakup_chunk(replace(record, compression='', data=raw), validate_crc=True)


def _decoder(schema):
    if schema.name not in CONTRACTS:
        return None
    stem, version = CONTRACTS[schema.name]
    meta = importlib.import_module(stem + '_schema_meta')
    digest = hashlib.sha256(schema.data).hexdigest()
    if (schema.encoding != 'flatbuffer' or meta.ROOT_TYPE_NAME != schema.name or
            digest != meta.SCHEMA_HASH_HEX or schema.data != meta.SCHEMA_BFBS or
            hashlib.sha256(meta.SCHEMA_BFBS).hexdigest() != meta.SCHEMA_HASH_HEX):
        raise ValueError('MCAP schema identity/BFBS mismatch: ' + schema.name)
    module = importlib.import_module(schema.name)
    root = getattr(module, schema.name.rsplit('.', 1)[1])

    def decode(raw):
        if not 8 <= len(raw) <= MAX_PAYLOAD:
            raise ValueError('MCAP message outside payload bound')
        view = root.GetRootAs(raw)
        wire_version = view.ProtocolVersion()
        if (wire_version != version and not
                (stem == 'body_cmd' and wire_version == BODY_CMD_MOTOR_THETA_PROTOCOL_VERSION)):
            raise ValueError('MCAP protocol version mismatch: ' + schema.name)
        if (stem == 'body_cmd' and wire_version == BODY_CMD_MOTOR_THETA_PROTOCOL_VERSION
                and view.Action() != BodyCmdAction.EXTERNAL):
            raise ValueError('MCAP BodyCmd version 4 requires EXTERNAL')
        # Finite vector bounds precede DTO construction. Actual SDK validators
        # and native state decoding still run; no second protocol implementation.
        for name, limit in (('Joints', 31), ('Axes', 32), ('InputObservations', 256), ('ImuStates', 8), ('AxisNames', 32)):
            length = getattr(view, name + 'Length', None)
            if length is not None and length() > limit:
                raise ValueError('MCAP vector outside analysis bound: ' + name)
        from .. import _body_codec, _end_effector_codec
        if stem == 'end_effector_info':
            for name in _end_effector_codec._INFO_VECTORS:
                getter = _body_codec._getter(name) + 'Length'
                if getattr(view, getter)() > 32:
                    raise ValueError('MCAP Info vector outside analysis bound: ' + name)
            if view.TouchRegionsLength() > 32:
                raise ValueError('MCAP touch region vector outside analysis bound')
        if stem == 'body_state':
            return asdict(_body_codec.decode_body_state(view))
        if stem == 'body_cmd':
            joints = _body_codec.unpack_body_cmd(view)
            if len(joints) != 31:
                raise ValueError('BodyCmd requires 31 joints')
            return dict(protocol_version=wire_version, action=view.Action(),
                        runtime_id=view.RuntimeId(), control_id=view.ControlId(),
                        session_id=view.SessionId(), command_sequence=view.Sequence(),
                        ts_client_created_us=view.TsClientCreatedUs(), ts_client_send_us=view.TsClientSendUs(),
                        joints=[asdict(joint) for joint in joints])
        if stem == 'end_effector_cmd':
            result = asdict(_end_effector_codec.decode_cmd(view))
            result.update(session_id=result['key']['session_id'], command_sequence=result['key']['sequence'])
            return result
        if stem == 'end_effector_state':
            return asdict(_end_effector_codec.decode_state(view))
        if stem == 'end_effector_info':
            return asdict(_end_effector_codec.decode_info(view))
        from ..transport import _decode_control_status
        result = asdict(_decode_control_status(view))
        result['stamp_monotonic_us'] = result.pop('stamp_monotonic_ns') // 1000
        result['last_valid_age_us'] = result.pop('last_valid_age_ms') * 1000
        return result
    return decode


def ingest(index, paths):
    """Store each message on disk. Unknown/mismatched schemas remain explicit."""
    from mcap.records import Schema, Channel, Message, Footer
    db = index.db
    db.execute('''CREATE TEMP TABLE mcap_messages(id INTEGER PRIMARY KEY, source TEXT, topic TEXT,
        root TEXT, log_us TEXT, publish_us TEXT, status TEXT, reason TEXT, body TEXT)''')
    sources = []
    for path in paths:
        identity = file_identity(path)
        schemas, channels, errors, footer = {}, {}, [], False
        try:
            with open(path, 'rb') as source:
                for record in _records(source):
                    if isinstance(record, Schema):
                        if len(schemas) >= 4096 or len(record.data) > MAX_PAYLOAD:
                            raise ValueError('MCAP schema registry exceeds analysis bound')
                        previous = schemas.get(record.id)
                        if previous is not None and previous[0] != record:
                            raise ValueError('conflicting MCAP schema id')
                        try:
                            decoder = _decoder(record)
                            error = '' if decoder else 'unsupported_schema'
                        except (ImportError, ValueError, AttributeError) as exc:
                            decoder, error = None, str(exc)
                        schemas[record.id] = (record, decoder, error)
                    elif isinstance(record, Channel):
                        if len(channels) >= 4096:
                            raise ValueError('MCAP channel registry exceeds analysis bound')
                        if record.id in channels and channels[record.id] != record:
                            raise ValueError('conflicting MCAP channel id')
                        channels[record.id] = record
                    elif isinstance(record, Message):
                        channel = channels.get(record.channel_id)
                        if channel is None:
                            raise ValueError('MCAP message before channel declaration')
                        schema, decoder, error = schemas.get(channel.schema_id, (None, None, 'missing_schema'))
                        data, status = None, 'NOT_AVAILABLE'
                        if channel.message_encoding != 'flatbuffer':
                            error, decoder = 'unsupported_message_encoding', None
                        if decoder:
                            try:
                                data = decoder(record.data)
                                status = 'DECODED'
                            except Exception as exc:
                                status, error = 'INVALID', str(exc)
                        db.execute('INSERT INTO mcap_messages VALUES(NULL,?,?,?,?,?,?,?,?)', (
                            identity['sha256'], channel.topic, schema.name if schema else None,
                            uint_text(record.log_time//1000), uint_text(record.publish_time//1000),
                            status, error, json.dumps(data)))
                    elif isinstance(record, Footer):
                        footer = True
        except Exception as exc:
            errors.append(type(exc).__name__ + ': ' + str(exc))
        if not footer:
            errors.append('missing_footer')
        if identity != file_identity(path):
            errors.append('source_changed_during_analysis')
        sources.append(dict(**identity, complete=not errors, issues=errors,
            schemas=[dict(root=item[0].name, sha256=hashlib.sha256(item[0].data).hexdigest(),
                          status='MATCHED' if item[1] else 'NOT_AVAILABLE', reason=item[2])
                     for item in schemas.values()]))
    return sources
