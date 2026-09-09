#!/usr/bin/env python3
"""Install verified offline candidates and atomically tune site; no ROS startup."""
from pathlib import Path
import copy
import fcntl
import hashlib
import json
import os
import shutil
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import yaml

ROOT = Path('/home/jetson/rinbo_ros_ws')
OUT = ROOT/'docs/diagnostics/tripod_restore_20260909'
BUILD = Path('/tmp/rinbo-tripod-slew-build')
SITE = Path('/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml')
NAMES = ('rinbo_cali','rinbo_standing','rinbo_tripod','rinbo_manual','rinbo_legs')
UPDATES = {'kp':.38, 'kd':.003, 'k_ff':.005, 'friction_pwm':0.,
           'startup_duration':8., 'start_ratio':8., 'target_ratio':1.,
           'ratio_step':-.0002, 'velocity_filter_time_constant_s':.005}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(args):
    return subprocess.run([str(x) for x in args],check=True,text=True,capture_output=True).stdout


def main():
    before = json.loads((OUT/'before.json').read_text())
    for relative, expected in json.loads((OUT/'candidate-source-manifest.json').read_text()).items():
        assert digest(ROOT/relative)==expected,f'Concurrent source edit: {relative}'
    tests={}
    for path in sorted((BUILD/'test_results/rinbo_fsm').glob('*.gtest.xml')):
        result=ET.parse(path).getroot()
        assert int(result.get('failures','0'))==int(result.get('errors','0'))==0, path
        tests[path.stem]=int(result.get('tests','0'))
    assert len(tests)==10,tests
    assert '100% tests passed' in (OUT/'native-tests.log').read_text()
    panel=ET.parse(OUT/'panel-tests.xml').getroot()
    assert all(int(s.get('failures','0'))==int(s.get('errors','0'))==0 for s in panel.iter('testsuite'))
    assert digest(SITE)==before[str(SITE)],'Concurrent site edit'
    previous=yaml.safe_load(SITE.read_text())
    assert previous['disabled_legs']==['L3']
    assert previous['parameters']['rinbo_tripod_rslip']['max_pwm']==3300
    assert not previous['parameters']['rinbo_tripod_rslip']['safety']['enable_pwm_slew_limit']
    for relative in ('src/rinbo_fsm/src/rinbo_cali.cpp','src/rinbo_fsm/src/rinbo_standing.cpp',
                     'src/rinbo_fsm/src/motion_effort.hpp','src/rinbo_fsm/src/tripod_reference.hpp',
                     'src/rinbo_ros_bridge/config/redrhex_safe.yaml','build/rinbo_ros_bridge/rinbo_ros_bridge'):
        p=(ROOT/relative).resolve();assert digest(p)==before[str(p)],p
    pairs=[(BUILD/name,ROOT/'build/rinbo_fsm'/name) for name in NAMES]
    for source,dest in pairs:
        assert digest(dest)==before[str(dest.resolve())],f'Concurrent binary edit: {dest}'
    args=['tune-tripod']+[f'{k}={v}' for k,v in UPDATES.items()]
    (OUT/'config-preview.json').write_text(run([BUILD/'rinbo_legs',args[0],'--dry-run',*args[1:]]))
    run([BUILD/'rinbo_legs','assert-idle'])
    backup_root=Path('/home/jetson/.local/state/rinbo-control-backups')
    backup_root.mkdir(parents=True,exist_ok=True)
    backup=Path(tempfile.mkdtemp(prefix='tripod-restore-',dir=backup_root))
    with open(str(SITE)+'.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        assert digest(SITE)==before[str(SITE)]
        (backup/'site.yaml').write_bytes(SITE.read_bytes())
        for suffix in ('.calibration.json','.standing.json'):
            receipt=Path(str(SITE)+suffix)
            if receipt.exists():(backup/receipt.name).write_bytes(receipt.read_bytes())
        staged=[]
        for source,dest in pairs:
            assert digest(dest)==before[str(dest.resolve())]
            saved=backup/dest.name;shutil.copy2(dest,saved)
            assert digest(saved)==digest(dest)
            temporary=dest.with_name(dest.name+'.restore-new');shutil.copy2(source,temporary)
            with temporary.open('rb') as f:os.fsync(f.fileno())
            assert digest(temporary)==digest(source)
            staged.append((temporary,dest))
        for temporary,dest in staged:os.replace(temporary,dest)
        fd=os.open(ROOT/'build/rinbo_fsm',os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(fd)
        finally:os.close(fd)
    # Native writer takes the same lock, checks idle, preserves only valid
    # existing prerequisite receipts, and atomically increments revision/hash.
    applied=run([ROOT/'build/rinbo_fsm/rinbo_legs',*args])
    (OUT/'config-applied.json').write_text(applied)
    expected=copy.deepcopy(previous);expected['revision']+=1
    expected['parameters']['rinbo_tripod_rslip'].update(UPDATES)
    actual=yaml.safe_load(SITE.read_text());assert actual==expected,'Unexpected config change'
    for name in ('rinbo_cali','rinbo_standing','rinbo_tripod'):
        (OUT/(name+'-deployed-check.json')).write_text(run([ROOT/'build/rinbo_fsm'/name,'--check-config']))
    (OUT/'deployed-config.json').write_text(run([ROOT/'build/rinbo_fsm/rinbo_legs','status','--json']))
    result={'deployed_at':time.strftime('%Y-%m-%d %H:%M:%S %z'),
            'backup':str(backup),'revision':actual['revision'],'site_sha256':digest(SITE),
            'tripod_updates':UPDATES,'binaries':{str(dest):digest(dest) for _,dest in pairs},
            'native_tests':tests,'hardware_actions':[]}
    (OUT/'deployment.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
