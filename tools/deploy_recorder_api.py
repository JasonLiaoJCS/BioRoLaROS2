#!/usr/bin/env python3
"""Install tested recorder/FSM artifacts without signalling any live process.

The legacy recorder binary is deliberately retained at its original path so
existing Windows process identity and open CSV files remain valid.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
EVIDENCE=ROOT/'docs/diagnostics/recorder_native_20260911'
FSM=('rinbo_cali','rinbo_standing','rinbo_tripod','rinbo_manual')
VERSION='native-recorder-v2-20260911'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replace_file(src,dst):
    dst.parent.mkdir(parents=True,exist_ok=True)
    tmp=dst.with_name(dst.name+'.deploy-'+str(os.getpid()))
    shutil.copy2(src,tmp)
    os.replace(tmp,dst)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply',action='store_true')
    args=parser.parse_args()
    sources={name:Path('/tmp/rinbo-fsm-recorder-v2-build')/name for name in FSM}
    sources['rinbo_data_recorder']=Path('/tmp/rinbo-recorder-v2-build/rinbo_data_recorder')
    for name,path in sources.items():
        marker=VERSION if name=='rinbo_data_recorder' else 'native-observer-v2-20260911'
        assert marker.encode() in path.read_bytes(),f'unverified artifact: {path}'
    # Never replace a motion executable that is in use, and never signal it.
    for proc in Path('/proc').iterdir():
        if not proc.name.isdecimal():continue
        try: executable=Path(os.readlink(proc/'exe')).name
        except OSError:continue
        if executable in FSM:raise RuntimeError(f'active motion PID {proc.name}; no deployment performed')
    print(json.dumps({'artifacts':{n:sha(p) for n,p in sources.items()},'apply':args.apply}))
    if not args.apply:return
    release=ROOT/'build/rinbo_data_recorder/releases'/VERSION/'rinbo_data_recorder'
    if release.exists():raise RuntimeError('release already exists; review before replacing it')
    backup=Path.home()/'.local/state/rinbo-deploy-backups'/time.strftime('%Y%m%d-%H%M%S-recorder')
    backup.mkdir(parents=True,exist_ok=False)
    manifest={'version':VERSION,'backup':str(backup),'files':[],'service_started':False,'processes_signalled':[]}
    def save(path):
        entry={'path':str(path),'exists':path.exists(),'symlink':os.readlink(path) if path.is_symlink() else None}
        if path.exists():
            saved=backup/str(len(manifest['files']))
            shutil.copy2(path,saved)
            entry.update(backup=str(saved),before_sha256=sha(path))
        manifest['files'].append(entry)
        return entry
    for name in FSM:
        dst=ROOT/'build/rinbo_fsm'/name
        entry=save(dst);replace_file(sources[name],dst);entry['after_sha256']=sha(dst)
    replace_file(sources['rinbo_data_recorder'],release)
    installed=ROOT/'install/rinbo_data_recorder/lib/rinbo_data_recorder/rinbo_data_recorder'
    entry=save(installed)
    temporary=installed.with_name(installed.name+'.deploy-'+str(os.getpid()))
    temporary.symlink_to(release);os.replace(temporary,installed)
    entry.update(after_sha256=sha(installed),after_symlink=str(release))
    # Existing share files may be source symlinks. Replace the link atomically,
    # never overwrite its source target or copy unrelated package contents.
    for category in ('config','launch'):
        for src in sorted((ROOT/'src/rinbo_data_recorder'/category).iterdir()):
            if not src.is_file() or src.suffix not in ('.yaml','.py'):continue
            dst=ROOT/'install/rinbo_data_recorder/share/rinbo_data_recorder'/category/src.name
            entry=save(dst);replace_file(src,dst);entry['after_sha256']=sha(dst)
    src=ROOT/'src/rinbo_data_recorder/package.xml'
    dst=ROOT/'install/rinbo_data_recorder/share/rinbo_data_recorder/package.xml'
    entry=save(dst);replace_file(src,dst);entry['after_sha256']=sha(dst)
    src=ROOT/'src/rinbo_control/rinbo_control/connection_recovery.py'
    dst=ROOT/'install/rinbo_control/lib/python3.10/site-packages/rinbo_control/connection_recovery.py'
    entry=save(dst);replace_file(src,dst);entry['after_sha256']=sha(dst)
    dst=Path.home()/'.config/systemd/user/rinbo-recorder.service'
    entry=save(dst);replace_file(ROOT/'config/systemd/rinbo-recorder.service',dst);entry['after_sha256']=sha(dst)
    subprocess.run(['systemctl','--user','daemon-reload'],check=True)
    manifest['source_sha256']={str(p.relative_to(ROOT)):sha(p) for directory in
        ('src/rinbo_fsm/src','src/rinbo_data_recorder/src','src/rinbo_data_recorder/config','src/rinbo_data_recorder/launch')
        for p in sorted((ROOT/directory).iterdir()) if p.is_file()}
    (EVIDENCE/'deployment.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'deployed':True,'backup':str(backup),'live_recorder_unchanged':True}))


if __name__=='__main__':main()
