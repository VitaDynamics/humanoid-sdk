"""Explicit synthetic EE input for the existing main Mock, never an authority."""
import json
import math
from pathlib import Path


def validate_ee_spec(spec):
    if not isinstance(spec, dict) or not spec or set(spec) - {'left', 'right'}:
        raise ValueError('EE fixture requires left/right endpoints')
    required = {'info', 'initial_pos', 'state_hz', 'info_hz', 'observation_sources'}
    for side, value in spec.items():
        if not isinstance(value, dict) or not required <= set(value) or set(value) - required - {'touch', 'touch_hz'}:
            raise ValueError('explicit EE Info/position/rates/observation sources required')
        for key in ('info', *(['touch'] if 'touch' in value else [])):
            if not isinstance(value[key], str) or not value[key].strip():
                raise ValueError('fixture file path must be explicit: '+key)
        if ('touch' in value) != ('touch_hz' in value):
            raise ValueError('touch fixture and explicit rate must appear together')
        for key in ('state_hz', 'info_hz', *(['touch_hz'] if 'touch' in value else [])):
            hz = value[key]
            if (type(hz) not in (int, float) or not math.isfinite(hz) or not 0 < hz <= 2000
                    or not math.isfinite(1e9/hz) or 1e9/hz >= 2**64):
                raise ValueError('explicit finite EE rate required: '+key)
        pos = value['initial_pos']
        if not isinstance(pos, list) or not 1 <= len(pos) <= 64 or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in pos):
            raise ValueError('initial_pos requires explicit finite synthetic samples')
        sources = value['observation_sources']
        if not isinstance(sources, list) or len(sources) > 5:
            raise ValueError('fixed ordered observation_sources required')
        seen = set()
        for source in sources:
            if (not isinstance(source, list) or len(source) != 2 or
                    any(type(v) is not int for v in source) or
                    tuple(source) not in ((1, 0), (2, 1), (2, 2), (3, 1), (3, 2)) or
                    tuple(source) in seen):
                raise ValueError('invalid/duplicate fixed observation stream/side')
            seen.add(tuple(source))
    for value in spec.values():
        for stream, side in value['observation_sources']:
            if stream == 1:
                continue
            source = spec.get('left' if side == 1 else 'right')
            if source is None or (stream == 3 and 'touch' not in source):
                raise ValueError('observation source is not configured in this main Mock')
    return spec


def prepare_ee_fixture(path, output):
    """Validate actual generated packets before any child/Node is opened.

    Only the main Protocol creates the runtime identity and consumes control
    ownership. Input packets supply synthetic capabilities/payload, not a lease.
    """
    from locomotion_aorta import _end_effector_codec as codec
    from locomotion_aorta.end_effector_types import DeviceModel, Field
    path, output = Path(path), Path(output)
    spec = validate_ee_spec(json.loads(path.read_text()))
    bindings = codec.load_bindings()
    files, lines, kinds = {}, [], {}
    def read(name):
        with (path.parent / name).open('rb') as stream:
            payload = stream.read(16*1024*1024+1)
        if not payload or len(payload) > 16*1024*1024:
            raise ValueError('EE fixture packet size exceeds bound')
        return payload
    for side, cfg in spec.items():
        payload = read(cfg['info'])
        info = codec.decode_info(bindings.EndEffectorInfo.EndEffectorInfo.GetRootAs(payload))
        codec.validate_info(info)
        if info.side != (1 if side == 'left' else 2) or info.device_model not in (
                DeviceModel.AGX, DeviceModel.BRAINCO_REVO2, DeviceModel.LINKER_L6):
            raise ValueError('EE fixture model/side unsupported; no OP3 fallback')
        pos = cfg['initial_pos']
        if len(pos) != len(info.axis_names) or any(
                not limits & Field.POS or not fields & Field.POS or not lo <= q <= hi
                for q, lo, hi, limits, fields in zip(pos, info.pos_min, info.pos_max,
                                                   info.valid_limits, info.supported_state_fields)):
            raise ValueError('synthetic initial position disagrees with actual Info')
        if info.touch_available != ('touch' in cfg):
            raise ValueError('Touch fixture/rate and actual Info capability disagree')
        files['ee-'+side+'-info.bin'] = payload
        if info.touch_available:
            payload = read(cfg['touch'])
            touch = codec.decode_touch(bindings.EndEffectorTouch.EndEffectorTouch.GetRootAs(payload))
            codec.validate_touch(info, touch)
            files['ee-'+side+'-touch.bin'] = payload
        sources = cfg['observation_sources']
        fields = [side, cfg['state_hz'], cfg['info_hz'], cfg.get('touch_hz', 0),
                  *pos, len(sources), *(v for source in sources for v in source)]
        lines.append(' '.join(map(str, fields)))
        kinds[side] = 2 if info.device_model == DeviceModel.AGX else 1
    for name, payload in files.items():
        with (output / name).open('xb') as stream:
            stream.write(payload)
    with (output / 'ee-config.txt').open('x') as stream:
        stream.write('\n'.join(lines)+'\n')
    return kinds
