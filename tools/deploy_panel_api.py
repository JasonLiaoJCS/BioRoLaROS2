#!/usr/bin/env python3
"""Install the tested paired artifacts; never starts robot/backend services."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
CANDIDATE = Path('/tmp/rinbo-power-handoff-build/rinbo_ros_bridge')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if not CANDIDATE.is_file():
        raise SystemExit('Build and test the candidate first')
    # Never replace a running controller/Bridge or depend on a stale PID list.
    blocked = {'rinbo_ros_bridge', 'rinbo_cali', 'rinbo_standing', 'rinbo_tripod', 'rinbo_manual'}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdecimal(): continue
        try:
            exe = Path(os.readlink(proc/'exe')).name
            args = (proc/'cmdline').read_bytes().split(b'\0')
            if exe in blocked or b'rinbo_control.console' in args or b'rinbo_control.panel_server' in args:
                raise SystemExit(f'Active native controller PID={proc.name}; complete native Stop before deployment')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    lock_path = Path.home()/'.local/state/rinbo_control/console.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    owner_lock = lock_path.open('a')
    fcntl.flock(owner_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    stamp = time.strftime('%Y%m%d-%H%M%S')
    backup = Path.home()/'.local/state/rinbo-deploy-backups'/stamp
    backup.mkdir(parents=True, mode=0o700)
    bridge = ROOT/'build/rinbo_ros_bridge/rinbo_ros_bridge'
    files = {bridge: CANDIDATE}
    # Deploy these exact Python files without rebuilding unrelated FSM sources.
    for package in ('redrhex_lowlevel_bridge','rinbo_control'):
        source = ROOT/'src'/package/package
        dest = ROOT/'install'/package/'lib/python3.10/site-packages'/package
        for path in source.glob('*.py'):
            files[dest/path.name] = path
    manifest = dict(created=stamp, backup=str(backup), files=[], hardware_started=False)
    for dest, source in files.items():
        relative = dest.relative_to(ROOT)
        previous = backup/relative
        old = None
        if dest.exists():
            previous.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(dest,previous)
            old = sha(dest)
        manifest['files'].append(dict(path=str(dest), source=str(source), before_sha256=old, sha256=sha(source)))
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    try:
        for dest, source in files.items():
            temp = dest.with_name(dest.name+'.panel-new')
            shutil.copy2(source,temp)
            os.replace(temp,dest)
            if sha(dest) != sha(source): raise RuntimeError('Installed hash mismatch: '+str(dest))
    except BaseException:
        for row in manifest['files']:
            dest = Path(row['path'])
            previous = backup/dest.relative_to(ROOT)
            if previous.exists(): shutil.copy2(previous,dest)
            else: dest.unlink(missing_ok=True)
        raise
    unit = Path.home()/'.config/systemd/user/rinbo-panel.service'
    unit.parent.mkdir(parents=True, exist_ok=True)
    if unit.exists(): shutil.copy2(unit,backup/'rinbo-panel.service')
    shutil.copy2(ROOT/'config/systemd/rinbo-panel.service',unit)
    subprocess.run(['systemctl','--user','daemon-reload'],check=True)
    # Inert owner only: no Runtime.connect/probe, ROS, SSH, Core or FPGA here.
    owner_lock.close()
    subprocess.run(['systemctl','--user','enable','--now','rinbo-panel.service'],check=True)
    time.sleep(1)
    subprocess.run(['systemctl','--user','is-active','--quiet','rinbo-panel.service'],check=True)
    manifest['controller_service'] = 'enabled; starts inert, never restores actions'
    manifest['deployed'] = True
    (ROOT/'docs/diagnostics/panel_api_20260911/deployment.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(deployed=True,backup=str(backup),files=len(files)),ensure_ascii=False))


if __name__ == '__main__':
    main()
