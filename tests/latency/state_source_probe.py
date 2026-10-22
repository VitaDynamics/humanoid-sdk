"""Real Aorta state-only subscription integration, private loopback, not a latency sample."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import threading

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'tests'),str(ROOT/'tests/latency')]


def publisher(directory,session):
    import aorta,body_state_schema_meta
    from decode_fixtures import state_object,pack_state
    node=aorta.Node('decode-source-probe-publisher','default')
    rows=[]
    try:
        pub=node.create_publisher_typed(body_state_schema_meta,f'/bench/sdk_state_source/{session}/body_state',qos=aorta.QoS.sensor_data())
        if not pub.wait_for_matching(15000):raise RuntimeError('source subscriber did not match')
        for seq,state_id in enumerate((91,91,0),1):
            obj=state_object(applied=True);obj.stateId=state_id
            payload=pack_state(obj)
            before=time.time_ns()
            pub.publish_bytes(payload)
            rows.append(dict(sequence=seq,state_id=state_id,publish_realtime_ns=before))
            deadline=time.monotonic()+5
            while not (directory/f'ack-{seq}').exists():
                if time.monotonic()>deadline:raise TimeoutError('source reader ack')
                time.sleep(.001)
    finally:
        node.close()
        (directory/'publisher.json').write_text(json.dumps(rows,indent=2)+'\n')


def run(output, *, pull=False):
    from run import config_files,terminate
    from locomotion_aorta import BodyStateSource, PullStateSource
    import locomotion_aorta._state_stream as stream
    output.mkdir(parents=True,exist_ok=False)
    for key in tuple(os.environ):
        if key.startswith(('AORTA_','ZENOH_')):os.environ.pop(key)
    os.environ['AORTA_LOG_TOPIC_ENABLED']='false'
    results=[]
    directory=output;session=time.time_ns()
    with socket.socket() as probe:
        probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
    config_files(directory,port)
    clean={k:v for k,v in os.environ.items() if not k.startswith(('ZENOH_','AORTA_'))}
    # The caller starts this parent in an isolated environment; the config is set before Node construction.
    os.environ['ZENOH_SESSION_CONFIG_URI']=str(directory/'python.json')
    stream.BODY_STATE_TOPIC=f'/bench/sdk_state_source/{session}/body_state'
    if pull:
        import locomotion_aorta.pull as pull_api
        pull_api.BODY_STATE_TOPIC=stream.BODY_STATE_TOPIC
        pull_api.CONTROL_STATUS_TOPIC=f'/bench/sdk_state_source/{session}/control_status'
    source=(PullStateSource if pull else BodyStateSource)(group='default')
    rows=[];child=None
    with (directory/'publisher.log').open('w') as log:
        try:
            child=subprocess.Popen([sys.executable,__file__,'--output',str(directory),'--publisher',str(session)],
                env={**clean,'ZENOH_SESSION_CONFIG_URI':str(directory/'mock.json'),'RUST_LOG':'error'},
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            previous=None
            for seq,state_id in enumerate((91,91,0),1):
                if pull:
                    deadline = time.monotonic()+5
                    while True:
                        remaining = deadline-time.monotonic()
                        if remaining <= 0:
                            raise TimeoutError('pull probe did not receive a new snapshot')
                        snapshot = source.wait_state(remaining)
                        if snapshot is not previous:
                            break
                        time.sleep(.001)
                else:
                    snapshot=source.snapshot(5,after=previous)
                assert snapshot.body_state().state_id==state_id
                rows.append(dict(sequence=seq,state_id=state_id,
                                 received_monotonic_ns=snapshot.received_monotonic_ns,
                                 received_realtime_ns=snapshot.received_realtime_ns))
                previous=snapshot
                (directory/f'ack-{seq}').touch()
            assert child.wait(timeout=5)==0
            close_wait = {}
            if pull:
                assert source.wait_state(0) is snapshot
                # A source with an existing snapshot returns it immediately.
                # Exercise close/unblock on a fresh source waiting for its first frame.
                wait_config=json.loads((directory/'python.json').read_text())
                wait_config['listen']['endpoints']=['tcp/127.0.0.1:0']
                (directory/'waiter.json').write_text(json.dumps(wait_config))
                os.environ['ZENOH_SESSION_CONFIG_URI']=str(directory/'waiter.json')
                waiting_source = PullStateSource(group='default')
                entered = threading.Event()
                def wait_after_publisher():
                    entered.set()
                    try:
                        waiting_source.wait_state(5)
                        close_wait['unexpected_frame'] = True
                    except BaseException as error:
                        close_wait['error'] = type(error).__name__
                waiting = threading.Thread(target=wait_after_publisher)
                waiting.start()
                assert entered.wait(5)
                waiting_source.close()
                waiting.join(5)
                assert not waiting.is_alive() and 'error' in close_wait, close_wait
            source.close()
            assert snapshot.body_state().joints[30].q==30.125
            sent=json.loads((directory/'publisher.json').read_text())
            assert len(sent)==len(rows)==3
            for a,b in zip(sent,rows):
                assert b['received_realtime_ns']>=a['publish_realtime_ns']
            results.append(dict(received=rows,private_state_publishers=1,
                                control_publishers=0,
                                closed_snapshot_valid=True, **close_wait))
        finally:
            source.close()
            if child is not None:terminate(child)
    result=dict(status='PASS',results=results,clock='CLOCK_REALTIME direct on both processes',
                boundary='IPC integration with per-frame file handshake; not timed performance evidence. No lease or robot topic.')
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--publisher',type=int)
    parser.add_argument('--pull',action='store_true')
    args=parser.parse_args()
    if args.publisher:publisher(args.output,args.publisher)
    else:print(run(args.output,pull=args.pull)['status'])


if __name__=='__main__':main()
