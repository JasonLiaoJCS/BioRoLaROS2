#!/usr/bin/env python3
"""Install the stop update only. Never restart a live Runtime or its backend."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import time

ROOT=Path(__file__).resolve().parents[1]
NAMES=('feedback.py','panel_native.py','panel_server.py','panel_protocol.py')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply',action='store_true')
    args=parser.parse_args()
    source=ROOT/'src/rinbo_control/rinbo_control'
    dest=ROOT/'install/rinbo_control/lib/python3.10/site-packages/rinbo_control'
    for name in NAMES:compile((source/name).read_text(),str(source/name),'exec')
    if not args.apply:
        print(json.dumps({'files':list(NAMES),'restart':False}));return
    backup=Path.home()/'.local/state/rinbo-deploy-backups'/time.strftime('%Y%m%d-%H%M%S-panel-stop')
    backup.mkdir(parents=True,exist_ok=False)
    manifest={'version':'panel-stop-v2-20260911','backup':str(backup),
              'files_installed':True,'service_restarted':False,'hardware_commands_sent':False,
              'runtime_activation':'pending_operator_verified_stop','files':[]}
    for name in NAMES:
        src=source/name;dst=dest/name
        shutil.copy2(dst,backup/name)
        entry={'source':str(src),'installed':str(dst),'before_sha256':sha(dst),'sha256':sha(src)}
        tmp=dst.with_name(dst.name+'.stop-update-'+str(os.getpid()))
        shutil.copy2(src,tmp);os.replace(tmp,dst)
        assert sha(dst)==entry['sha256']
        manifest['files'].append(entry)
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    out=ROOT/'docs/diagnostics/panel_stop_20260911/deployment.json'
    out.write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':main()
