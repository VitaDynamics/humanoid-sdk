#!/usr/bin/env python3
"""Offline ORT 1.20.1 exact oracle; never constructs an Aorta node.

The oracle interpreter is a separate environment with Python ORT 1.20.1.
The main interpreter uses the installed SDK's native backend. No demo fallback.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def oracle(model, inputs, output, threads):
    import numpy as np
    import onnxruntime as ort
    if ort.__version__ != '1.20.1':
        raise ValueError('oracle must use ORT 1.20.1')
    options = ort.SessionOptions()
    options.intra_op_num_threads, options.inter_op_num_threads = threads, 1
    session = ort.InferenceSession(str(model), options, providers=['CPUExecutionProvider'])
    values = np.load(inputs, allow_pickle=False)
    name, out = session.get_inputs()[0].name, session.get_outputs()[0].name
    np.save(output, np.stack([session.run([out], {name: x[None]})[0][0] for x in values]))


def collect(args):
    import numpy as np
    from locomotion_aorta._onnx import ORT_VERSION
    if ORT_VERSION != '1.20.1':
        raise ValueError('SDK native backend must use ORT 1.20.1')
    observations, outputs = [], []
    if args.kind == 'velocity':
        # Source example is an input to the acceptance runner, not an import of
        # a second SDK: sys.path still resolves the installed SDK under test.
        sys.path.insert(0, str(Path(args.sdk_source)/'examples'))
        from velocity_mode_policy import VelocityModePolicy
        policy = VelocityModePolicy(args.locomotion_dir)
        model = policy.model_path
        for i in range(128):
            q, dq = np.zeros(31), np.zeros(31)
            q[policy.policy_to_body] = policy.default_q + .001*np.sin(np.arange(len(policy.names))+i)
            dq[policy.policy_to_body] = .01*np.cos(np.arange(len(policy.names))+i)
            command = policy.command((0., float(np.sin(i*.13)), 0., float(np.cos(i*.09))))
            obs = policy.observe(q, dq, (0.,0.,0.), (1.,0.,0.,0.), command)
            action = policy.infer(obs)
            policy.decode(action)
            observations.append(obs[0].copy())
            outputs.append(action)
    else:
        from locomotion_aorta.wbc_tracking_policy import Model, WbcTrackingPolicy, Motion
        model_runner = Model(args.model, args.threads)
        policy = WbcTrackingPolicy(model_runner.contract, json.loads(args.profile.read_text()))
        c = policy.contract
        motion = Motion(np.repeat(c.default[None], 160, axis=0),
                        np.zeros((160,3)), np.tile([1.,0.,0.,0.], (160,1)))
        model = args.model
        for i in range(128):
            obs = policy.observe(motion, i*.02,
                                 c.default+.001*np.sin(np.arange(28)+i),
                                 .01*np.cos(np.arange(28)+i), (1.,0.,0.,0.), (0.,0.,0.))
            action = model_runner.infer(obs)
            policy.decode(action)
            observations.append(obs.copy())
            outputs.append(action)
    return model, np.asarray(observations, np.float32), np.asarray(outputs, np.float32)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind', choices=('velocity','wbc'), required=True)
    parser.add_argument('--sdk-source', type=Path)
    parser.add_argument('--locomotion-dir', type=Path)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--profile', type=Path)
    parser.add_argument('--threads', type=int, required=True)
    parser.add_argument('--oracle-python', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--oracle-input', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.oracle_input:
        oracle(args.model, args.oracle_input, args.output, args.threads)
        return
    if args.threads <= 0:
        parser.error('threads must be positive')
    if args.oracle_python is None:
        parser.error('--oracle-python is required')
    if args.kind == 'velocity' and (args.sdk_source is None or args.locomotion_dir is None):
        parser.error('velocity requires --sdk-source and --locomotion-dir')
    if args.kind == 'wbc' and (args.model is None or args.profile is None):
        parser.error('wbc requires --model and --profile')
    if args.kind == 'velocity' and args.threads != 1:
        parser.error('velocity source contract requires one intra-op thread')
    args.output.mkdir(parents=True, exist_ok=False)
    model, inputs, native = collect(args)
    import numpy as np
    np.save(args.output/'input.npy', inputs)
    np.save(args.output/'native.npy', native)
    subprocess.run([str(args.oracle_python), str(Path(__file__).resolve()), '--kind', args.kind,
                    '--threads', str(args.threads), '--model', str(model),
                    '--oracle-input', str(args.output/'input.npy'),
                    '--output', str(args.output/'oracle.npy')], check=True)
    expected = np.load(args.output/'oracle.npy', allow_pickle=False)
    finite = bool(np.isfinite(native).all() and np.isfinite(expected).all())
    equal = bool(finite and np.array_equal(native, expected))
    report = dict(kind=args.kind, frames=len(inputs), dtype=str(inputs.dtype),
                  model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                  input_sha256=hashlib.sha256((args.output/'input.npy').read_bytes()).hexdigest(),
                  native_ort='1.20.1', oracle_ort='1.20.1', intra=args.threads, inter=1,
                  finite=finite, exact_equal=equal, signed_zero_equal=True,
                  max_abs_error=(float(np.max(abs(native-expected))) if finite and native.shape==expected.shape else None))
    (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    if not equal:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
