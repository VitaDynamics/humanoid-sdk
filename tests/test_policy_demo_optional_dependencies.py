"""CLI/discovery remain usable without the optional policy math packages."""
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BLOCK_OPTIONAL = '''
import importlib.machinery, json, runpy, sys
missing = json.loads(sys.argv[1])
original = importlib.machinery.PathFinder.find_spec
def find_spec(fullname, path=None, target=None):
    if fullname.split('.')[0] in missing:
        return None
    return original(fullname, path, target)
importlib.machinery.PathFinder.find_spec = staticmethod(find_spec)
mode = sys.argv[2]
sys.argv = sys.argv[2:]
if mode == 'unittest':
    runpy.run_module('unittest', run_name='__main__')
else:
    from pathlib import Path
    sys.path.insert(0, str(Path(mode).resolve().parent))
    runpy.run_path(mode, run_name='__main__')
'''


class OptionalPolicyDependenciesTest(unittest.TestCase):
    def run_without(self, missing, *args):
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1',
                   PYTHONPATH=os.pathsep.join(str(ROOT/p) for p in ('python', 'tests', '.')))
        return subprocess.run([sys.executable, '-c', BLOCK_OPTIONAL, json.dumps(missing), *args],
                              cwd=ROOT, env=env, capture_output=True, text=True, timeout=30)

    def test_help_does_not_load_optional_policy_dependencies(self):
        for missing in (['numpy'], ['yaml'], ['numpy', 'yaml']):
            for script in ('velocity_mode', 'wbc_tracking_mode'):
                with self.subTest(missing=missing, script=script):
                    result = self.run_without(missing, f'examples/{script}/{script}.py', '--help')
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertIn('usage:', result.stdout)

    def test_body31_policy_discovery_skips_missing_optional_dependencies(self):
        for missing in (['numpy'], ['yaml'], ['numpy', 'yaml']):
            with self.subTest(missing=missing):
                result = self.run_without(missing, 'unittest', 'discover', '-s', 'tests',
                                          '-p', 'test_policy_body31.py')
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('skipped=', result.stderr)
