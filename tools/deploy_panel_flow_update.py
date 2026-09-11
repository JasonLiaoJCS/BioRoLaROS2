#!/usr/bin/env python3
"""Install panel-flow-v4 files. Never reload a controller or operate hardware."""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import shutil
import time

ROOT = Path(__file__).resolve().parents[1]
NAMES = ('panel_server.py', 'panel_native.py', 'panel_protocol.py', 'runtime.py')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    source = ROOT/'src/rinbo_control/rinbo_control'
    dest = ROOT/'install/rinbo_control/lib/python3.10/site-packages/rinbo_control'
    for name in NAMES:
        compile((source/name).read_text(), str(source/name), 'exec')
    version = next(ast.literal_eval(n.value) for n in ast.parse((source/'panel_server.py').read_text()).body
                   if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'VERSION' for t in n.targets))
    if version != 'panel-flow-v4-20260911':
        raise SystemExit('source version changed; review the appropriate deployment procedure')
    if not args.apply:
        print(json.dumps(dict(version=version, files=list(NAMES), service_restart=False))); return
    backup = Path.home()/'.local/state/rinbo-deploy-backups'/time.strftime('%Y%m%d-%H%M%S-panel-flow')
    backup.mkdir(parents=True, exist_ok=False)
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = dict(version=version, backup=str(backup), files=[], files_installed=True,
                    service_restarted=False, hardware_commands_sent=False,
                    runtime_activation='pending_operator_reload')
    for name in NAMES:
        src, dst = source/name, dest/name
        shutil.copy2(dst, backup/name)
        entry = dict(source=str(src), installed=str(dst), before_sha256=sha(dst), sha256=sha(src))
        tmp = dst.with_name(dst.name+'.flow-update-'+str(os.getpid()))
        shutil.copy2(src,tmp); os.replace(tmp,dst)
        assert sha(dst) == entry['sha256']
        manifest['files'].append(entry)
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    evidence = ROOT/'docs/diagnostics/panel_flow_20260911'
    evidence.mkdir(parents=True, exist_ok=True)
    (evidence/'deployment.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__ == '__main__':
    main()
