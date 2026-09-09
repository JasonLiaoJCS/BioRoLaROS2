from pathlib import Path
import fcntl,hashlib,json,os,shutil,subprocess,time,xml.etree.ElementTree as ET
root=Path('/home/jetson/rinbo_ros_ws'); candidate=Path('/tmp/rinbo-motion-limits-build')
evidence=root/'docs/diagnostics/motion_limits_20260909'; backup=root/'.codex-backups/motion_limits_20260909_1940'
before=json.loads((backup/'before.json').read_text());site=Path('/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml')
names=['rinbo_cali','rinbo_standing','rinbo_tripod','rinbo_manual','rinbo_legs']
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def run(argv):
 result=subprocess.run([str(x) for x in argv],capture_output=True,text=True,timeout=15)
 if result.returncode:raise RuntimeError(f'{argv}: {result.returncode}: {result.stderr}\n{result.stdout}')
 return result.stdout
for suite in ['standing','cali_multileg','robot_config','tripod_multileg','motor_arbiter_handshake']:
 tree=ET.parse(evidence/(suite+'.xml')).getroot()
 assert int(tree.attrib.get('failures',0))==0 and int(tree.attrib.get('errors',0))==0,suite
run([candidate/'rinbo_legs','assert-idle'])
for name in names:
 assert digest(root/'build/rinbo_fsm'/name)==before[str(root/'build/rinbo_fsm'/name)],'Deployment changed: '+name
assert digest(site)==before[str(site)],'Site configuration changed, do not overwrite'
# Other sessions' Tripod controller and shared electrical/communication guards stay exact.
for name in ['rinbo_tripod.cpp','safety_invariants.hpp']:
 assert digest(root/'src/rinbo_fsm/src'/name)==before[str(root/'src/rinbo_fsm/src'/name)],name
for name in names[:3]:
 out=run([candidate/name,'--check-config']);(evidence/(name+'-candidate-check.txt')).write_text(out)
with open(str(site)+'.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 assert digest(site)==before[str(site)]
 # Stage all replacement files before publishing any of them. MotionSession
 # requires a shared lock on this same file and cannot start during this block.
 staged=[]
 for name in names:
  dst=root/'build/rinbo_fsm'/name; tmp=dst.with_name(name+'.motion-limits-new')
  shutil.copy2(candidate/name,tmp)
  with tmp.open('rb') as handle:os.fsync(handle.fileno())
  assert digest(tmp)==digest(candidate/name)
  staged.append((tmp,dst))
 for tmp,dst in staged:os.replace(tmp,dst)
 fd=os.open(root/'build/rinbo_fsm',os.O_RDONLY|os.O_DIRECTORY)
 try:os.fsync(fd)
 finally:os.close(fd)
# Apply through the tested native API; do not rewrite production YAML in Python.
cli=root/'build/rinbo_fsm/rinbo_legs'
standing=run([cli,'tune-limits','standing','--expect-revision','9','position_tolerance_counts=1000','rotate_timeout_s=60','hall_search_timeout_s=60'])
(evidence/'standing-apply.json').write_text(standing)
cal=run([cli,'tune-limits','calibration','--expect-revision','10','servo_homing_timeout_s=60','hall_search_timeout_s=60','stop_timeout_s=15'])
(evidence/'calibration-apply.json').write_text(cal)
status=run([cli,'status','--json']);(evidence/'deployed-config.json').write_text(status)
for name in names[:3]:
 out=run([root/'build/rinbo_fsm'/name,'--check-config']);(evidence/(name+'-deployed-check.txt')).write_text(out)
record={'deployed_at':time.strftime('%Y-%m-%d %H:%M:%S %z'),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'binaries':{name:digest(root/'build/rinbo_fsm'/name) for name in names},'config_sha256':digest(site),'revision':json.loads(status)['revision'],'hardware_actions':[],'backup':str(backup)}
(evidence/'deployment.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record,indent=2))
