#!/usr/bin/env python3
"""Legacy entry redirected to independent HandCmd topics."""
try:
    from examples.brainco_hand_external import main
except ModuleNotFoundError:
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location(
        'brainco_hand_external', Path(__file__).with_name('brainco_hand_external.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    main = module.main

if __name__ == '__main__':
    main()
