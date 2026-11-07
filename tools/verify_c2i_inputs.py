"""Verify the reviewed 25-file C2I capture; never generate or download inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import re

REPO = Path(__file__).resolve().parents[1]
MANIFEST = REPO / 'tests/fixtures/c2i-producer.manifest.json'
ROOT = REPO / 'tests/fixtures/c2i-producer'
COMPATIBILITY = REPO / 'compatibility.json'
TARGET = '//src/application/locomotion/humanoid:end_effector_command_runtime_test'
SCHEMAS = ('end_effector_info', 'end_effector_state', 'end_effector_touch', 'get_robot_info')
REQUIRED = frozenset('''
static/left_info.fb static/right_info.fb static/left_state.fb static/right_state.fb
static/left_info_republished.fb static/left_next_state.fb static/right_inhibited.fb
static/left_new_runtime_info.fb static/robot_info.fb static/new_runtime_robot_info.fb
linker/info.fb linker/robot_info.fb linker/position.fb linker/zero_fault.fb
linker/temperature.fb linker/next_position.fb linker/nonzero_fault.fb
linker/temperature_after_fault.fb linker/permission_revoked.fb linker/position_stale.fb
brainco/info.fb brainco/robot_info.fb brainco/touch.fb brainco/touch_disabled.fb brainco/touch_new.fb
'''.split())


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError('duplicate JSON key: ' + key)
        result[key] = value
    return result


def _json(path):
    value = json.loads(path.read_text(), object_pairs_hook=_pairs)
    if not isinstance(value, dict):
        raise ValueError('JSON object required: ' + str(path))
    return value


def _hex(value, length, name):
    if not isinstance(value, str) or re.fullmatch('[0-9a-f]{' + str(length) + '}', value) is None:
        raise ValueError('fixed hexadecimal identity required: ' + name)


def _file(path, expected):
    if set(expected) != {'bytes', 'sha256'}:
        raise ValueError('bytes and sha256 required: ' + str(path))
    if type(expected['bytes']) is not int or expected['bytes'] <= 0:
        raise ValueError('positive captured byte count required: ' + str(path))
    _hex(expected['sha256'], 64, str(path))
    if path.is_symlink() or not path.is_file():
        raise ValueError('regular captured file required: ' + str(path))
    if path.stat().st_size != expected['bytes']:
        raise ValueError('byte count mismatch: ' + str(path))
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(65536), b''):
            digest.update(chunk)
    if digest.hexdigest() != expected['sha256'] or path.stat().st_size != expected['bytes']:
        raise ValueError('SHA256/byte count mismatch: ' + str(path))


def verify(root=ROOT, manifest=MANIFEST, compatibility=COMPATIBILITY):
    # Expected values are reviewed SDK source, not learned from the capture.
    if manifest.is_symlink() or not manifest.is_file():
        raise ValueError('reviewed C2I manifest missing: ' + str(manifest))
    expected = _json(manifest)
    if type(expected.get('format_version')) is not int or expected['format_version'] != 1:
        raise ValueError('unsupported C2I manifest format')
    if set(expected['files']) != REQUIRED:
        raise ValueError('expected manifest must name exactly the 25 C2I inputs')
    producer = expected['producer']
    if producer['repository'] != 'VitaDynamics/vita-robot' or producer['target'] != TARGET:
        raise ValueError('wrong fixed C2I producer')
    for key in ('commit', 'tree'):
        _hex(producer[key], 40, 'producer.' + key)
    for key in ('elf_sha256', 'invocation_sha256'):
        _hex(producer[key], 64, 'producer.' + key)
    schemas = expected['schema_sha256']
    pinned = _json(compatibility)['schema_sha256']
    if set(schemas) != set(SCHEMAS) or any(schemas[k] != pinned[k] for k in SCHEMAS):
        raise ValueError('C2I schema identity differs from pinned SDK bindings')
    for key in SCHEMAS:
        _hex(schemas[key], 64, 'schema.' + key)
    if root.is_symlink() or not root.is_dir():
        raise ValueError('actual C2I capture directory missing: ' + str(root))
    root = root.resolve(strict=True)
    if '\n' in str(root) or '\r' in str(root):
        raise ValueError('C2I path cannot contain line breaks')
    files = set()
    for path in root.rglob('*'):
        relative = path.relative_to(root).as_posix()
        if path.is_symlink():
            raise ValueError('symlink in C2I capture: ' + relative)
        if path.is_dir():
            if relative not in ('static', 'linker', 'brainco'):
                raise ValueError('unexpected C2I directory: ' + relative)
        elif path.is_file():
            files.add(relative)
        else:
            raise ValueError('non-regular C2I input: ' + relative)
    if files != REQUIRED | {'capture.json'}:
        raise ValueError('capture must contain exactly 25 inputs and capture.json')
    for name in sorted(REQUIRED):
        _file(root / name, expected['files'][name])
    _file(root / 'capture.json', expected['capture'])
    capture = _json(root / 'capture.json')
    if capture['producer'] != producer or capture['schema_sha256'] != schemas:
        raise ValueError('captured producer/schema identity differs from reviewed manifest')
    return root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    args = parser.parse_args()
    try:
        print(verify(args.root))
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f'C2I inputs unavailable or invalid: {error}\n')


if __name__ == '__main__':
    main()
