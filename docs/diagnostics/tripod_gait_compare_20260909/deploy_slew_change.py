"""Deploy reviewed local binaries/config only; never launch ROS/motor services."""
from pathlib import Path
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

ROOT=Path('/home/jetson/rinbo_ros_ws')
OUT=ROOT/'docs/diagnostics/tripod_gait_compare_20260909'
BUILD=Path('/tmp/rinbo-tripod-slew-build')
SITE=Path('/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml')
names=('rinbo_cali','rinbo_standing','rinbo_tripod','rinbo_manual','rinbo_legs')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(args):
    result=subprocess.run([str(a) for a in args],capture_output=True,text=True,timeout=30)
    if result.returncode:
        raise RuntimeError(f'{args[0]} returned {result.returncode}: {result.stderr}')
    return result.stdout


def main():
    before=json.loads((OUT/'pre-deploy.json').read_text())['sha256']
    results={}
    for path in sorted((BUILD/'test_results/rinbo_fsm').glob('*.gtest.xml')):
        result=ET.parse(path).getroot()
        assert int(result.get('failures','0'))==0 and int(result.get('errors','0'))==0,path
        results[path.stem]=int(result.get('tests','0'))
    assert len(results)==10,results
    assert '100% tests passed' in (OUT/'native-tests.txt').read_text()
    assert '100% tests passed' in (OUT/'manual-controller-test.txt').read_text()
    panel=ET.parse(OUT/'panel-tests.xml').getroot()
    assert all(int(s.get('failures','0'))==0 and int(s.get('errors','0'))==0 for s in panel.iter('testsuite'))
    pairs=[(BUILD/n,ROOT/'build/rinbo_fsm'/n) for n in names]
    for _,path in pairs:
        assert digest(path)==before[str(path.resolve())],f'Concurrent binary change: {path}'
    assert digest(SITE)==before[str(SITE)],'Concurrent configuration change'
    previous=yaml.safe_load(SITE.read_text())
    backup_root=Path('/home/jetson/.local/state/rinbo-control-backups')
    backup_root.mkdir(parents=True,exist_ok=True)
    backup=Path(tempfile.mkdtemp(prefix='tripod-slew-',dir=backup_root))
    (backup/'site.yaml').write_bytes(SITE.read_bytes())
    for suffix in ('.calibration.json','.standing.json'):
        receipt=Path(str(SITE)+suffix)
        if receipt.exists():
            (backup/receipt.name).write_bytes(receipt.read_bytes())
    run([BUILD/'rinbo_legs','assert-idle'])
    with open(str(SITE)+'.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        # assert-idle itself takes this lock; do not recursively acquire it in
        # a child. Holding it here excludes native MotionSession construction.
        assert digest(SITE)==before[str(SITE)]
        staged=[]
        for source,destination in pairs:
            assert digest(destination)==before[str(destination.resolve())]
            saved=backup/destination.name
            saved.write_bytes(destination.read_bytes())
            os.chmod(saved,destination.stat().st_mode & 0o777)
            assert digest(saved)==digest(destination)
            temporary=destination.with_name(destination.name+'.slew-new')
            shutil.copy2(source,temporary)
            with temporary.open('rb') as f:os.fsync(f.fileno())
            assert digest(temporary)==digest(source)
            staged.append((temporary,destination))
        for temporary,destination in staged:
            os.replace(temporary,destination)
        fd=os.open(ROOT/'build/rinbo_fsm',os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(fd)
        finally:os.close(fd)
    # New binaries accept both old and new flags. The native atomic writer
    # owns revision/receipt migration and refuses a concurrently started motion.
    cli=ROOT/'build/rinbo_fsm/rinbo_legs'
    applied=run([cli,'tune-limits','tripod','--expect-revision',str(previous['revision']),
                 'enable_pwm_slew_limit=0'])
    (OUT/'config-applied.json').write_text(applied)
    now=yaml.safe_load(SITE.read_text())
    expected=previous.copy()
    expected=yaml.safe_load(yaml.safe_dump(expected))
    expected['revision']+=1
    expected['parameters']['rinbo_tripod_rslip']['safety']['enable_pwm_slew_limit']=False
    assert now==expected,'Unexpected site parameter change'
    assert digest(SITE)==json.loads((OUT/'config-preview.json').read_text())['hash']
    for name in ('rinbo_cali','rinbo_standing','rinbo_tripod'):
        (OUT/(name+'-deployed-check.txt')).write_text(run([ROOT/'build/rinbo_fsm'/name,'--check-config']))
    (OUT/'deployed-config.json').write_text(run([cli,'status','--json']))
    record={'deployed_at':time.strftime('%Y-%m-%d %H:%M:%S %z'),
            'backup':str(backup),'revision':now['revision'],'site_sha256':digest(SITE),
            'binaries':{str(dst):digest(dst) for _,dst in pairs},
            'only_site_change':'Tripod safety.enable_pwm_slew_limit: true -> false; revision increment',
            'native_test_cases':results,'hardware_actions':[]}
    (OUT/'deployment.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
