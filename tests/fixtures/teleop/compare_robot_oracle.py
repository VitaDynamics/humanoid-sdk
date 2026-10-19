"""Generate shared synthetic BodyState input or compare C++ float32 results."""
import argparse
import dataclasses
import hashlib
import json
import math
from pathlib import Path
import struct

from examples.teleop_policy import Model, Policy, objects
from test_teleop_policy import NAMES


def csv(md, key, convert=float):
    return [convert(v.strip()) for v in md[key].split(',') if v.strip()]


def generate(model_path, root):
    model = Model(model_path)
    contract = json.loads((root/'action17_contract.json').read_text())
    md = model.metadata
    profile = json.loads((root/'oracle-profile.json').read_text())
    body = dict(slot_count=31, joint_names=list(NAMES),
                joints=[dict(populated=n!='waist_roll_joint') for n in NAMES])
    legs, upper = csv(md,'joint_names',str),csv(md,'upper_joint_names',str)
    names = legs + upper
    defaults = csv(md,'default_joint_pos') + csv(md,'upper_default_pos')
    resolved = dict(legs=legs,upper=upper,defaults=defaults,scales=csv(md,'action_scale'),
                    kp=csv(md,'joint_stiffness')+csv(md,'upper_joint_stiffness'),
                    kd=csv(md,'joint_damping')+csv(md,'upper_joint_damping'),
                    lower=csv(md,'joint_pos_limit_lower') if 'joint_pos_limit_lower' in md else [],
                    upper_limits=csv(md,'joint_pos_limit_upper') if 'joint_pos_limit_upper' in md else [],
                    clip_actions=float(md['clip_actions']) if 'clip_actions' in md else None)
    p = Policy(md,contract)
    lookup={n:default for n,default in zip(names,defaults)}
    frames,reference=[],[]
    for k in range(10):
        quat=(1.,0.,0.,0.)
        if k==3:quat=(math.sqrt(.5),0.,math.sqrt(.5),0.)
        elif k>3:
            raw=(1.,.07*k,-.03*k,.02*k)
            length=math.sqrt(sum(v*v for v in raw));quat=tuple(v/length for v in raw)
        command=(.02*k,-.01*k,.03*k,.855-.01*k)
        state=dict(state_id=1000+k,ts_state_real_us=1792000000000000+k*20000,attitude_valid=True,
                   imu_states=[dict(location='TRUNK',status=1,state=dict(quaternion=(0,1,0,0),gyroscope=(9,8,7))),
                               dict(location='HIP',status=1,state=dict(quaternion=quat,gyroscope=(.1*k,-.2*k,.3*k)))],
                   joints=[dict(body_index=i,status=7 if n=='waist_roll_joint' else 1,
                                      q=lookup.get(n,0.)+.001*i*(k+1),dq=-.003*(i+1)*k)
                                 for i,n in enumerate(NAMES)])
        observation=p.observation(objects(state),objects(body),command,'HIP')
        actions=model.infer(observation)
        targets=p.targets(actions,objects(body))
        frames.append(dict(body_state=state,command=command,actions=actions))
        reference.append(dict(observation=observation,targets=[dataclasses.asdict(m) for m in targets],
                              physical_names=[n for n in NAMES if n!='waist_roll_joint']))
    shared=dict(evidence_kind='SYNTHETIC_BODY_STATE_NOT_DEVICE_CAPTURE',model_sha256=hashlib.sha256(Path(model_path).read_bytes()).hexdigest(),
                metadata=md,resolved_model=resolved,profile=profile,body_info=body,frames=frames)
    (root/'shared-input.json').write_text(json.dumps(shared,indent=2)+'\n')
    (root/'python-reference.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in reference))
    print(json.dumps({'generated_frames':len(frames),'observation_values':len(frames)*492,'target_fields':len(frames)*31*5}))


def float32(x):
    return struct.unpack('<f',struct.pack('<f',x))[0]


def equal_float32(a,b):
    # Numeric equality at the production/wire float32 boundary, with no epsilon.
    # IEEE +0 and -0 compare equal. NaN/Inf must not make an oracle pass.
    a,b=float32(a),float32(b)
    return math.isfinite(a) and math.isfinite(b) and a==b


def compare(root,cpp_path):
    python=[json.loads(l) for l in (root/'python-reference.jsonl').read_text().splitlines()]
    cpp=[json.loads(l) for l in cpp_path.read_text().splitlines()]
    assert len(cpp)==len(python)==10
    observations=fields=signed_zeros=0
    for frame,(actual,expected) in enumerate(zip(cpp,python)):
        assert actual['physical_names']==expected['physical_names'],(frame,'physical roster')
        assert len(actual['observation'])==len(expected['observation'])==492
        for i,(a,b) in enumerate(zip(actual['observation'],expected['observation'])):
            assert equal_float32(a,b),(frame,'observation',i,a,b)
            if a==b==0 and math.copysign(1,a)!=math.copysign(1,b):signed_zeros+=1
            observations+=1
        assert len(actual['targets'])==len(expected['targets'])==31
        for i,(a,b) in enumerate(zip(actual['targets'],expected['targets'])):
            for field in ('q','dq','tau','kp','kd'):
                assert equal_float32(a[field],b[field]),(frame,'target',i,field,a[field],b[field])
                fields+=1
    report=dict(result='PASS',frames=len(cpp),observation_float32_values=observations,
                target_float32_fields=fields,comparison='exact finite float32 numeric equality; signed zeros equal; no tolerance',
                signed_zero_observation_differences=signed_zeros,
                input_sha256=hashlib.sha256((root/'shared-input.json').read_bytes()).hexdigest(),
                python_output_sha256=hashlib.sha256((root/'python-reference.jsonl').read_bytes()).hexdigest(),
                cpp_output_sha256=hashlib.sha256(cpp_path.read_bytes()).hexdigest(),
                robot_device_capture='NOT_MEASURED',full_cpp_runtime='NOT_MEASURED',
                cpp_onnx_runner='NOT_MEASURED',formal_H1_D2_codec='NOT_MEASURED')
    (cpp_path.parent/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--generate',action='store_true')
    parser.add_argument('--model')
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--cpp-output',type=Path)
    args=parser.parse_args()
    if args.generate:
        if not args.model:parser.error('--generate requires --model')
        generate(args.model,args.root)
    else:
        if args.cpp_output is None:parser.error('--cpp-output is required')
        compare(args.root,args.cpp_output)
