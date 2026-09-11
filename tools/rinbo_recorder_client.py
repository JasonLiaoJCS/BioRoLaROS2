#!/usr/bin/env python3
"""Native recording API client. No hardware command publisher or SSH to sbRIO."""
import argparse
import hashlib
import json
import mmap
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import time
import uuid

WORKSPACE = Path(__file__).resolve().parents[1]
VERSION = 'native-recorder-v2-20260911'


def running_recorders(domain):
    found=[]
    for path in Path('/proc').iterdir():
        if not path.name.isdecimal(): continue
        try:
            if path.stat().st_uid != os.getuid(): continue
            exe=os.readlink(path/'exe').removesuffix(' (deleted)')
            if Path(exe).name != 'rinbo_data_recorder': continue
            env=(path/'environ').read_bytes().split(b'\0')
            value=next((v.split(b'=',1)[1].decode() for v in env if v.startswith(b'ROS_DOMAIN_ID=')), '0')
            if value==str(domain):
                with (path/'exe').open('rb') as binary, mmap.mmap(binary.fileno(),0,access=mmap.ACCESS_READ) as contents:
                    native=contents.find(VERSION.encode())>=0
                found.append(dict(pid=int(path.name), executable=exe, native_v2=native))
        except (OSError, UnicodeError): pass
    return found


def file_sha256(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def snapshot_export(response):
    """Copy only byte counts from the recorder's serialized flush barrier."""
    run_dir=Path(response['run_dir']).resolve()
    if not run_dir.is_dir(): raise RuntimeError('no_run_to_export')
    export_root=run_dir.parent/'exports';export_root.mkdir(exist_ok=True)
    output=export_root/(run_dir.name+'-'+uuid.uuid4().hex+'.tar.gz')
    with tempfile.TemporaryDirectory(prefix='rinbo-export-') as tmp:
        for entry in response['files']:
            source=Path(entry['path'])
            if source.is_symlink() or source.resolve().parent != run_dir or entry['name'] != source.name:
                raise RuntimeError('invalid_export_file')
            remaining=entry['bytes']
            if remaining is None: raise RuntimeError('export_file_unavailable: '+source.name)
            with source.open('rb') as src, (Path(tmp)/source.name).open('xb') as dst:
                while remaining:
                    data=src.read(min(1024*1024,remaining))
                    if not data: raise RuntimeError('export_source_truncated: '+source.name)
                    dst.write(data);remaining-=len(data)
        (Path(tmp)/'export_manifest.json').write_text(json.dumps(response,indent=2)+'\n')
        temporary=output.with_suffix('.partial')
        try:
            with tarfile.open(temporary,'w:gz') as archive:
                for p in sorted(Path(tmp).iterdir()): archive.add(p,arcname=run_dir.name+'/'+p.name)
            os.replace(temporary,output)
        finally:
            temporary.unlink(missing_ok=True)
    response['export']=dict(path=str(output),bytes=output.stat().st_size,
        sha256=file_sha256(output),snapshot=True)
    return response


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['start','stop','status','export','version'])
    parser.add_argument('--name')
    parser.add_argument('--domain',type=int,default=99)
    parser.add_argument('--timeout',type=float,default=8)
    parser.add_argument('--request-id',default=None)
    parser.add_argument('--no-start',action='store_true',help='never start a recorder service')
    parser.add_argument('--json',action='store_true',help='JSON is always emitted')
    args=parser.parse_args(argv)
    rid=args.request_id or uuid.uuid4().hex
    result=dict(protocol=1,version=VERSION,request_id=rid,action=args.action,recording=None)
    if args.action=='version':
        result.update(status='client_version',exit_code=0)
        print(json.dumps(result));return 0
    if args.action=='start' and (not args.name or len(args.name)>128):
        result.update(status='not_sent',exit_code=2,reason='start requires --name (1..128 characters)')
        print(json.dumps(result));return 2
    if not 0<=args.domain<=232 or not 0<args.timeout<=30:
        result.update(status='not_sent',exit_code=2,reason='invalid domain or timeout')
        print(json.dumps(result));return 2
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    # Plain python3 over SSH need not have sourced ROS. Arguments remain argv,
    # never interpolated as shell code, and credentials are never accepted.
    if os.environ.get('RINBO_RECORDER_ENV')!='1':
        env=dict(os.environ,RINBO_RECORDER_ENV='1')
        script='source /opt/ros/humble/setup.bash; source /home/jetson/rinbo_ros_ws/install/setup.bash; exec /usr/bin/python3 "$@"'
        os.execvpe('bash',['bash','-c',script,'rinbo-recorder',str(Path(__file__).resolve()),*(argv if argv is not None else sys.argv[1:])],env)
    import rclpy
    from rcl_interfaces.srv import SetParametersAtomically
    from rcl_interfaces.msg import Parameter,ParameterValue
    rclpy.init(args=[])
    node=rclpy.create_node('rinbo_recorder_client_'+uuid.uuid4().hex[:10],enable_rosout=False)
    client=node.create_client(SetParametersAtomically,'/rinbo/recorder/control')
    try:
        if not client.wait_for_service(timeout_sec=args.timeout):
            existing=running_recorders(args.domain)
            if existing:
                native=any(p['native_v2'] for p in existing)
                result.update(status='api_unavailable' if native else 'legacy_recorder_active',exit_code=31 if native else 21,processes=existing,
                    reason=('Existing native recorder API is unreachable; recording/closed state is unknown; query again after reconnection.' if native else 'Current recorder has no native v2 API; keep its adapter and finish it with its existing control path. No process stopped.'))
                return emit(result)
            if args.action!='start' or args.no_start:
                result.update(status='unavailable',exit_code=10,reason='native recorder service unavailable; recording state unknown')
                return emit(result)
            if args.domain!=99:
                raise RuntimeError('automatic service startup is domain99 only; use --no-start with an isolated recorder')
            startup=subprocess.run(['systemctl','--user','start','rinbo-recorder.service'],capture_output=True,text=True,timeout=8)
            if startup.returncode: raise RuntimeError('recorder_service_start_failed: '+startup.stderr)
            if not client.wait_for_service(timeout_sec=args.timeout): raise RuntimeError('recorder_start_timeout; query status before retry')
        # A duplicate root recorder must not make a service response ambiguous.
        rclpy.spin_once(node,timeout_sec=.15)
        if sum(name=='rinbo_data_recorder' and namespace=='/' for name,namespace in node.get_node_names_and_namespaces())!=1:
            raise RuntimeError('recorder_not_unique: no control request sent')
        request=SetParametersAtomically.Request()
        values=dict(action=args.action,request_id=rid)
        if args.name is not None:values['name']=args.name
        request.parameters=[Parameter(name=k,value=ParameterValue(type=4,string_value=v)) for k,v in values.items()]
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=args.timeout)
        if not future.done():
            result.update(status='state_unknown',exit_code=31,reason='recorder ACK timeout; query status, do not infer stopped/closed')
        else:
            result=json.loads(future.result().result.reason)
            if result.get('version')!=VERSION or result.get('request_id')!=rid:
                raise RuntimeError('invalid native recorder response')
            if args.action=='export' and not result['exit_code']: result=snapshot_export(result)
    except Exception as exc:
        result.update(status='failed',exit_code=50 if isinstance(exc,OSError) else 31,reason=str(exc))
    finally:
        node.destroy_node();rclpy.shutdown()
    return emit(result)


def emit(result):
    print(json.dumps(result,ensure_ascii=False),flush=True)
    if result.get('exit_code'): print(result.get('reason',result['status']),file=sys.stderr)
    return result['exit_code']


if __name__=='__main__':
    raise SystemExit(main())
