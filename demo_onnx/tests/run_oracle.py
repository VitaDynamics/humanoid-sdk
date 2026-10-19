"""Run one real backend in its own environment; no inference fallback."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("native", "python"), required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.backend == "native":
        from locomotion_aorta_demo_onnx import Session, ORT_VERSION
        session = Session(str(args.model), intra_threads=1, inter_threads=1)
        input_name, output_name = session.inputs[0].name, session.outputs[0].name
        infer = lambda values: session.run(input_name, output_name, values)
        version = ORT_VERSION
    else:
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        session = ort.InferenceSession(str(args.model), sess_options=options,
                                       providers=["CPUExecutionProvider"])
        input_name, output_name = session.get_inputs()[0].name, session.get_outputs()[0].name
        infer = lambda values: session.run([output_name], {input_name: values})[0]
        version = ort.__version__
    rows = [json.loads(line) for line in args.reference.read_text().splitlines()]
    values = np.asarray([row["observation"] for row in rows], dtype=np.float32)
    outputs = np.concatenate([infer(frame.reshape(1, -1)) for frame in values])
    assert outputs.dtype == np.float32
    np.save(args.output, outputs, allow_pickle=False)
    report = dict(backend=args.backend, version=version, threads=[1, 1],
                  model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
                  reference_sha256=hashlib.sha256(args.reference.read_bytes()).hexdigest(),
                  float32_input_sha256=hashlib.sha256(values.tobytes()).hexdigest(),
                  shape=list(outputs.shape), finite=bool(np.isfinite(outputs).all()),
                  output_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest())
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
