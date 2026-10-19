"""Same-payload decode A/B, split by output contract; no transport or robot."""
import argparse
from array import array
import csv
import dataclasses
import hashlib
import importlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'python'), str(ROOT / 'tests')]

from locomotion_aorta import BodyStateSnapshot, BodyStateMetadata, FrameToken, BodyImuLocation
from locomotion_aorta._body_codec import decode_body_state
from decode_fixtures import state_object, pack_state
from latency.report import stats as elapsed_stats

MASKS = (('body_index','H'), ('populated','B'), ('is_mock','B'), ('status','B'),
         ('valid_fields','H'), ('ts_acquired_hw_us','Q'), ('ts_ingested_us','Q'),
         ('ts_assembled_us','Q'), ('lost','I'))


def signed_stats(values):
    """Paired B-A differences may be negative; elapsed-time stats remain strict."""
    values = sorted(values)
    if not values or any(not math.isfinite(v) for v in values):
        raise ValueError('empty/nonfinite signed measurement')
    def q(p):
        x=(len(values)-1)*p;lo=int(x);hi=min(lo+1,len(values)-1)
        return values[lo]+(values[hi]-values[lo])*(x-lo)
    return dict(n=len(values),mean_ms=statistics.fmean(values),p50_ms=q(.5),
                p95_ms=q(.95),p99_ms=q(.99),max_ms=values[-1])


def equal(a,b):
    if isinstance(a,float) and math.isnan(a):
        return isinstance(b,float) and math.isnan(b)
    if dataclasses.is_dataclass(a):
        return type(a) is type(b) and all(equal(getattr(a,f.name),getattr(b,f.name))
                                          for f in dataclasses.fields(a))
    if isinstance(a,tuple):
        return isinstance(b,tuple) and len(a)==len(b) and all(equal(x,y) for x,y in zip(a,b))
    return a==b


def eager_selected(state, indices, fields, location, echo):
    metadata=BodyStateMetadata(**{f.name:getattr(state,f.name) for f in dataclasses.fields(BodyStateMetadata)})
    joints=[state.joints[i] for i in indices] if state.joints else []
    values=tuple((name,array('f',(getattr(j,name) for j in joints)).tobytes()) for name in fields)
    masks=tuple((name,array(fmt,(getattr(j,name) for j in joints)).tobytes()) for name,fmt in MASKS)
    imus=[s for s in state.imu_states if s.location==location] if location is not None else []
    if len(imus)>1:raise ValueError('ambiguous IMU location')
    return metadata,indices,values,masks,imus[0] if imus else None,state.applied_cmd if echo else None


def selected_result(selection, echo):
    columns=selection.columns()
    return (columns.metadata,columns.joint_indices,
            tuple((name,bytes(columns.values[name])) for name in columns.values),
            tuple((name,bytes(getattr(columns,name))) for name,_ in MASKS),
            selection.imu_sample(),echo)


def frame(payload):
    decoder=importlib.import_module('lowlevel.BodyState').BodyState.GetRootAs
    return BodyStateSnapshot(payload,decoder,FrameToken('decode-profile',1),
                             time.monotonic_ns(),time.time_ns())


def candidate(payload, case, *, existing=None):
    start=time.monotonic_ns()
    snapshot=frame(payload) if existing is None else existing
    created=time.monotonic_ns()
    if case['contract']=='full DTO':
        result=snapshot.body_state()
        return result,dict(snapshot_ns=created-start,full_dto_ns=time.monotonic_ns()-created)
    selection=snapshot.select(joint_indices=case['indices'],fields=case['fields'],imu_location=case['location'])
    selected=time.monotonic_ns()
    selection.columns()
    gathered=time.monotonic_ns()
    selection.imu_sample()
    imu=time.monotonic_ns()
    echo=snapshot.applied_cmd() if case['echo'] else None
    applied=time.monotonic_ns()
    result=selected_result(selection,echo)
    end=time.monotonic_ns()
    stages=dict(snapshot_ns=created-start,selection_api_ns=selected-created,
                columns_check_gather_wrap_ns=gathered-selected,contract_bytes_ns=end-applied)
    if case['location'] is not None:stages['imu_requested_ns']=imu-gathered
    if case['echo']:stages['applied_requested_ns']=applied-imu
    return result,stages


def baseline(payload,case):
    view=importlib.import_module('lowlevel.BodyState').BodyState.GetRootAs(payload)
    state=decode_body_state(view)
    return state if case['contract']=='full DTO' else eager_selected(
        state,case['indices'],case['fields'],case['location'],case['echo'])


def measure(operation):
    wall=time.monotonic_ns();cpu=time.thread_time_ns()
    value=operation()
    cpu_end=time.thread_time_ns();wall_end=time.monotonic_ns()
    return value,wall_end-wall,cpu_end-cpu


def cases():
    return [dict(name='full',contract='full DTO'),
            dict(name='columns31',contract='selected columns+metadata',indices=tuple(range(31)),
                 fields=('q','dq'),location=None,echo=False),
            dict(name='subset_hip_echo',contract='selected columns+metadata+HIP+applied',
                 indices=(30,0,14,5),fields=('dq','q'),location=BodyImuLocation.HIP,echo=True)]


def run(output,*,samples=1000,warmup=50,profile=False):
    output.mkdir(parents=True,exist_ok=False)
    rows=[];stage_rows=[];fixtures=[]
    for normal in (False,True):
        for applied in (False,True):
            fixture=f'{"normal" if normal else "slot14_nan"}-applied{int(applied)}'
            payload=pack_state(state_object(normal=normal,applied=applied))
            sha=hashlib.sha256(payload).hexdigest()
            (output/f'{fixture}.bin').write_bytes(payload)
            fixtures.append(dict(name=fixture,sha256=sha,bytes=len(payload)))
            for case in cases():
                expected=baseline(payload,case)
                actual,_=candidate(payload,case)
                if not equal(actual,expected):raise AssertionError('oracle mismatch before timed run')
                cached=frame(payload);candidate(payload,case,existing=cached)
                for _ in range(warmup):baseline(payload,case);candidate(payload,case)
                for i in range(samples):
                    order=('baseline','candidate') if i%2==0 else ('candidate','baseline')
                    measured={}
                    for mode in order:
                        if mode=='baseline':
                            value,wall,cpu=measure(lambda:baseline(payload,case))
                            stages={}
                        else:
                            wrapped,wall,cpu=measure(lambda:candidate(payload,case))
                            value,stages=wrapped
                        measured[mode]=(wall,cpu)
                        for name,duration in stages.items():
                            stage_rows.append(dict(fixture=fixture,case=case['name'],sample=i,
                                                   stage=name,elapsed_ns=duration))
                    wrapped,cache_wall,cache_cpu=measure(lambda:candidate(payload,case,existing=cached))
                    for mode,(wall,cpu) in {**measured,'candidate_cached':(cache_wall,cache_cpu)}.items():
                        rows.append(dict(fixture=fixture,payload_sha256=sha,case=case['name'],
                                         contract=case['contract'],sample=i,order='/'.join(order),
                                         mode=mode,wall_ns=wall,thread_cpu_ns=cpu))
                if not equal(baseline(payload,case),candidate(payload,case)[0]):
                    raise AssertionError('oracle mismatch after timed run')
    for name,data in [('samples.csv',rows),('stages.csv',stage_rows)]:
        with (output/name).open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(data[0]));writer.writeheader();writer.writerows(data)
    summary=[]
    for fixture in fixtures:
        for case in cases():
            group=[r for r in rows if r['fixture']==fixture['name'] and r['case']==case['name']]
            item=dict(fixture=fixture['name'],case=case,statistics={},signed_candidate_minus_baseline={})
            for mode in ('baseline','candidate','candidate_cached'):
                selected=[r for r in group if r['mode']==mode]
                item['statistics'][mode]={metric:elapsed_stats([r[metric]/1e6 for r in selected])
                                          for metric in ('wall_ns','thread_cpu_ns')}
            a=[r for r in group if r['mode']=='baseline'];b=[r for r in group if r['mode']=='candidate']
            for metric in ('wall_ns','thread_cpu_ns'):
                item['signed_candidate_minus_baseline'][metric]=signed_stats(
                    [(y[metric]-x[metric])/1e6 for x,y in zip(a,b)])
            item['candidate_stages_ms']={name:elapsed_stats([r['elapsed_ns']/1e6 for r in stage_rows
                if r['fixture']==fixture['name'] and r['case']==case['name'] and r['stage']==name])
                for name in sorted({r['stage'] for r in stage_rows if r['case']==case['name']})}
            summary.append(item)
    metadata=dict(status='ORACLE_PASS',samples=samples,warmup=warmup,profile=profile,
                  clock='same-process CLOCK_MONOTONIC elapsed ns + thread_time_ns; no cross-process measurement',
                  fixtures=fixtures,results=summary,platform=platform.platform(),python=sys.version,
                  argv=sys.argv,cpu_count=os.cpu_count(),loadavg=os.getloadavg(),
                  versions={name:importlib.metadata.version(name) for name in
                            ('aorta-sdk','aorta-msgs','vbot-humanoid-msgs','flatbuffers')},
                  request_counts=dict(first_use_pairs=len(fixtures)*len(cases())*samples,
                                      cached_reads=len(fixtures)*len(cases())*samples,
                                      errors=0,wire_received='N/A; no transport',overwritten='N/A; independent frames'),
                  profiler_attached_during_timed_samples=False,
                  source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                      for base in (ROOT/'python/locomotion_aorta',ROOT/'tests/latency')
                      for p in sorted(base.rglob('*.py'))},
                  schema_sha256=hashlib.sha256(importlib.import_module('body_state_schema_meta').SCHEMA_BFBS).hexdigest(),
                  source_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                  source_dirty=subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True),
                  publishers_created=0,network=False,
                  boundaries='First-use pairs share exact bytes and output contract. Cached candidate separate, no paired cache speedup. Instrumentation included. Columns layout validation/gather/wrapping fused; do not sum stage quantiles. Profile off during timed samples. No policy/FK/adapter/model/communications.')
    if profile:
        import cProfile,tracemalloc
        payload=pack_state(state_object(applied=True));case=cases()[2]
        tracemalloc.start();profiler=cProfile.Profile();profiler.enable()
        for _ in range(100):candidate(payload,case)
        profiler.disable();current,peak=tracemalloc.get_traced_memory();tracemalloc.stop()
        profiler.dump_stats(str(output/'separate-profile.pstats'))
        metadata['separate_profile']=dict(calls=100,current_bytes=current,peak_bytes=peak,
                                         excluded_from_timed_samples=True)
    (output/'summary.json').write_text(json.dumps(metadata,indent=2)+'\n')
    lines=['# Body31 decode A/B', '', 'Status: ORACLE_PASS; no performance threshold.', '',
           'Same payload and output contract within each row; 50 warmup/1000 pairs in the standard run.',
           'Wall/thread CPU are same-process MONOTONIC. Cached reads are separate; signed delta is first-use candidate minus baseline.', '',
           '|Fixture|Output contract|Baseline mean/P99 ms|Candidate mean/P99 ms|Paired delta mean/P99 ms|Cached mean ms|',
           '|---|---|---:|---:|---:|---:|']
    for item in summary:
        a=item['statistics']['baseline']['wall_ns'];b=item['statistics']['candidate']['wall_ns']
        d=item['signed_candidate_minus_baseline']['wall_ns'];c=item['statistics']['candidate_cached']['wall_ns']
        lines.append(f"|{item['fixture']}|{item['case']['contract']}|{a['mean_ms']:.6f}/{a['p99_ms']:.6f}|"
                     f"{b['mean_ms']:.6f}/{b['p99_ms']:.6f}|{d['mean_ms']:+.6f}/{d['p99_ms']:+.6f}|{c['mean_ms']:.6f}|")
    lines += ['', metadata['boundaries'], '', 'Raw CSV/JSON include CPU statistics and fused/optional stages. No robot, hardware, aarch64 or package acceptance.']
    (output/'summary.md').write_text('\n'.join(lines)+'\n')
    return metadata


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--samples',type=int,default=1000)
    parser.add_argument('--warmup',type=int,default=50)
    parser.add_argument('--profile',action='store_true')
    args=parser.parse_args()
    if args.samples<1 or args.warmup<0:parser.error('invalid measurement sample count')
    result=run(args.output,samples=args.samples,warmup=args.warmup,profile=args.profile)
    print(result['status'],args.output/'summary.json')


if __name__=='__main__':main()
