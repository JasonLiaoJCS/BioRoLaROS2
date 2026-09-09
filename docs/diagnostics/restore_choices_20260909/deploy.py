"""Apply accepted choices after isolated tests. No ROS init, power or motion."""
from pathlib import Path
import contextlib,datetime,fcntl,hashlib,json,os,shutil,subprocess,tempfile
import xml.etree.ElementTree as ET
import yaml
ROOT=Path('/home/jetson/rinbo_ros_ws');OUT=ROOT/'docs/diagnostics/restore_choices_20260909'
AUDIT=ROOT/'docs/diagnostics/workspace_audit_20260909';BUILD=Path('/tmp/rinbo-tripod-slew-build')
SITE=Path('/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml')
NAMES=('rinbo_cali','rinbo_standing','rinbo_tripod','rinbo_manual','rinbo_legs')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def run(args):return subprocess.run([str(x) for x in args],text=True,capture_output=True,check=True).stdout
def sync_dir(p):
    fd=os.open(p,os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(fd)
    finally:os.close(fd)
def stage_file(src,dest):
    fd,name=tempfile.mkstemp(prefix=dest.name+'.choices-',dir=dest.parent);os.close(fd)
    tmp=Path(name);shutil.copy2(src,tmp)
    with tmp.open('rb') as f:os.fsync(f.fileno())
    assert sha(tmp)==sha(src)
    return tmp

def main():
    assert not (OUT/'deployment.json').exists(),'This deployment was already applied'
    manifests=json.loads((OUT/'candidate-source-manifest.json').read_text())
    for relative,digest in manifests.items():assert sha(ROOT/relative)==digest,relative
    old_binaries=json.loads((AUDIT/'binary-manifest.json').read_text())
    for name,digest in old_binaries.items():assert sha(Path(name))==digest,name
    assert SITE.read_bytes()==(OUT/'site-before.yaml').read_bytes(),'Site changed concurrently'
    candidate=OUT/'site-candidate.yaml';before=yaml.safe_load(SITE.read_text());after=yaml.safe_load(candidate.read_text())
    assert after['revision']==before['revision']+1
    assert after['parameters']['rinbo_tripod_rslip']==before['parameters']['rinbo_tripod_rslip']
    assert after['disabled_legs']==before['disabled_legs']==['L3']
    verified=json.loads(run([OUT/'validate_candidate',candidate]))
    assert verified['hash']==sha(candidate)
    (OUT/'candidate-validated.json').write_text(json.dumps(verified,ensure_ascii=False,indent=2)+'\n')
    assert '100% tests passed' in (OUT/'native-tests.log').read_text()
    tests={}
    for p in (BUILD/'test_results/rinbo_fsm').glob('*.gtest.xml'):
        t=ET.parse(p).getroot();assert int(t.get('failures',0))==int(t.get('errors',0))==0,p
        tests[p.stem]=int(t.get('tests',0))
    assert len(tests)==10
    panel=ET.parse(OUT/'panel-tests.xml').getroot()
    assert all(int(s.get('failures',0))==int(s.get('errors',0))==0 for s in panel.iter('testsuite'))
    backup_root=Path('/home/jetson/.local/state/rinbo-control-backups');backup_root.mkdir(exist_ok=True,parents=True)
    backup=Path(tempfile.mkdtemp(prefix='restore-choices-',dir=backup_root))
    pairs=[(BUILD/n,ROOT/'build/rinbo_fsm'/n) for n in NAMES]
    with contextlib.ExitStack() as stack:
        for suffix in ('.lock','.motion.lock'):
            lock=stack.enter_context(open(str(SITE)+suffix,'a'))
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        run([OUT/'validate_candidate',candidate])
        assert SITE.read_bytes()==(OUT/'site-before.yaml').read_bytes()
        shutil.copy2(SITE,backup/'site.yaml')
        for suffix in ('.calibration.json','.standing.json'):
            receipt=Path(str(SITE)+suffix)
            if receipt.exists():shutil.copy2(receipt,backup/receipt.name)
        staged=[]
        for source,dest in pairs:
            assert sha(dest)==old_binaries[str(dest)],dest
            shutil.copy2(dest,backup/dest.name)
            staged.append((stage_file(source,dest),dest))
        site_tmp=stage_file(candidate,SITE)
        os.chmod(site_tmp,SITE.stat().st_mode & 0o777)
        # Changed Calibration/Standing means old success receipts must not carry forward.
        for suffix in ('.calibration.json','.standing.json'):Path(str(SITE)+suffix).unlink(missing_ok=True)
        try:
            for tmp,dest in staged:os.replace(tmp,dest)
            os.replace(site_tmp,SITE)
            sync_dir(SITE.parent);sync_dir(ROOT/'build/rinbo_fsm')
            readback=json.loads(run([ROOT/'build/rinbo_fsm/rinbo_legs','status','--json']))
            assert readback==verified | {'path':str(SITE)}
            for name in ('rinbo_cali','rinbo_standing','rinbo_tripod'):
                (OUT/(name+'-check-config.txt')).write_text(run([ROOT/'build/rinbo_fsm'/name,'--check-config']))
            (OUT/'deployed-config.json').write_text(json.dumps(readback,ensure_ascii=False,indent=2)+'\n')
        except BaseException:
            # Restore files on deployment failure; never restore stale success receipts.
            for _,dest in pairs:os.replace(stage_file(backup/dest.name,dest),dest)
            os.replace(stage_file(backup/'site.yaml',SITE),SITE)
            raise
        finally:
            for tmp,_ in staged:tmp.unlink(missing_ok=True)
            site_tmp.unlink(missing_ok=True)
    result={'deployed_at':datetime.datetime.now().astimezone().isoformat(),'backup':str(backup),
            'revision':after['revision'],'site_sha256':sha(SITE),'native_tests':tests,
            'panel_passed':256,'panel_skipped':1,'binaries':{str(dest):sha(dest) for _,dest in pairs},
            'receipts':'Calibration and Standing invalidated; no motion performed',
            'hardware_actions':[],'control_panel_entry':'robot.sh imports src/rinbo_control directly'}
    (OUT/'deployment.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
