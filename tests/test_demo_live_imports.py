"""Execute live startup imports with fake inputs/transport, never a real Node."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CHECK = r'''
import importlib.util, json, runpy, sys
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock, patch
root, source, mode = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
sys.path.insert(0, str(source))
import aorta
aorta.Node = Mock(side_effect=AssertionError('real Node forbidden'))
import locomotion_aorta as api
script = {'brainco_hand':'brainco_hand_external','gripper':'gripper_external'}.get(mode,mode)
path = root/'examples'/mode/(script+'.py')
# run_path has no script-dir or repository sys.path: the isolated bundle case.
namespace = runpy.run_path(str(path))
main = namespace['main']
g = main.__globals__
entered = []
class Reached(Exception): pass
class Controller:
    def __init__(self, **kw): entered.append(kw)
    def __enter__(self): return self
    def __exit__(self, *args): pass
    transport = S(node=S(create_subscriber_pull=Mock(return_value=S(close=lambda:None))))
def reached(*args, **kwargs):
    entered.append('live-imports-passed')
    raise Reached()
if mode == 'teleop_policy':
    g['Model'] = lambda path:S(metadata={})
    g['Policy'] = lambda *args:S(period=.02)
    g['live'] = reached
    # Preserve the release's input choice: 0.2.0 pygame, 0.2.1 Aorta Joy.
    pygame = S(init=lambda:None, quit=lambda:None,
        joystick=S(init=lambda:None, Joystick=lambda n:S(init=lambda:None,quit=lambda:None)))
    argv = [str(path),'--model',str(root/'compatibility.json'),'--confirm-motion']
    if '--log' in path.read_text():
        argv += ['--log', str(root/'startup.jsonl')]
    with patch.dict(sys.modules, {'pygame':pygame}), \
         patch.object(api,'ExternalController',Controller), \
         patch.object(api,'PullController',Controller,create=True), \
         patch.object(sys,'argv',argv):
        try: main()
        except Reached: pass
elif mode == 'velocity_mode':
    # Stop only after the real script and policy imports have succeeded and
    # execution has entered main's non-help startup branch.
    g['VelocityModePolicy'] = reached
    argv = [str(path),'--locomotion-dir',str(root),'--log',str(root/'velocity.jsonl'),
            '--shadow']
    with patch.object(sys,'argv',argv):
        try: main()
        except Reached: pass
elif mode == 'wbc_tracking_mode':
    model = S(contract={},session=S(metadata={'groot_wbc.tracking_contract.v1':'{}'}))
    backend = S(Model=lambda *args:model,WbcTrackingPolicy=lambda *args:S())
    g['Recorder'] = lambda *args,**kwargs:S(write=lambda *args:None,close=lambda:None)
    argv = [str(path),'--model',str(root/'compatibility.json'),
            '--live','--confirm-motion','--log',str(root/'wbc.jsonl')]
    with patch.dict(sys.modules,{'locomotion_aorta.wbc_tracking_policy':backend}), \
         patch.dict(__import__('os').environ,{'ZENOH_SESSION_CONFIG_URI':'/unused/fake-only'}), \
         patch.object(api,'ExternalController',reached), \
         patch.object(api,'PullController',reached,create=True), \
         patch.object(sys,'argv',argv):
        try: main()
        except Reached: pass
elif mode in ('external_control','brainco_hand','gripper'):
    g['PullController'] = reached
    argv = [str(path),'--confirm-motion']
    if mode == 'gripper':
        argv += ['--side','left','--target-m','.03']
    with patch.object(api,'ExternalController',reached), \
         patch.object(api,'PullController',reached,create=True), \
         patch.object(sys,'argv',argv):
        try: main()
        except Reached: pass
else:
    raise AssertionError(mode)
assert 'live-imports-passed' in entered, entered
aorta.Node.assert_not_called()
print(json.dumps({'mode':mode,'reached':entered,'Node':0}))
'''


class DemoLiveImportTest(unittest.TestCase):
    def check_layout(self, bundle):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            shutil.copytree(ROOT/'examples',root/'examples',
                            ignore=shutil.ignore_patterns('__pycache__'))
            shutil.copyfile(ROOT/'compatibility.json',root/'compatibility.json')
            if bundle:
                # Same source/locomotion_aorta + examples layout as package_release.
                shutil.copytree(ROOT/'python/locomotion_aorta',root/'source/locomotion_aorta',
                                ignore=shutil.ignore_patterns('__pycache__','*.so','.libs'))
                source = root/'source'
            else:
                source = ROOT/'python'
            for mode in ('teleop_policy','velocity_mode','wbc_tracking_mode',
                         'external_control','brainco_hand','gripper'):
                with self.subTest(bundle=bundle,mode=mode):
                    result = subprocess.run([sys.executable,'-I','-B','-c',CHECK,
                        str(root),str(source),mode],cwd=root,
                        env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'),
                        capture_output=True,text=True,timeout=30)
                    self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                    self.assertIn('"Node": 0',result.stdout)

    def test_source_tree_live_startup_imports(self):
        self.check_layout(False)

    def test_offline_bundle_layout_live_startup_imports(self):
        self.check_layout(True)


if __name__ == '__main__':
    unittest.main()
