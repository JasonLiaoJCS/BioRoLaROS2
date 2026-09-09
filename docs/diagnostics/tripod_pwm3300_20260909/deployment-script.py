from pathlib import Path
import fcntl,hashlib,json,os,shutil,subprocess,time,xml.etree.ElementTree as ET
import yaml
root=Path('/home/jetson/rinbo_ros_ws');fsm=Path('/tmp/rinbo-motion-limits-build');bridge=Path('/tmp/rinbo-bridge3300-build')
e=root/'docs/diagnostics/tripod_pwm3300_20260909';backup=root/'.codex-backups/tripod_pwm3300_20260909'
before=json.loads((backup/'before.json').read_text());site=Path('/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml')
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def run(args):
 r=subprocess.run([str(x) for x in args],capture_output=True,text=True,timeout=20)
 if r.returncode:raise RuntimeError(f'{args}: {r.returncode}: {r.stderr}')
 return r.stdout
for suite in ['tripod_multileg','robot_config','motor_arbiter_handshake','standing_multileg','cali_multileg','motor_output_limits','power_command_epoch_guard']:
 d=ET.parse(e/(suite+'.xml')).getroot()
 assert int(d.attrib.get('failures',0))==0 and int(d.attrib.get('errors',0))==0,suite
pairs=[(fsm/n,root/'build/rinbo_fsm'/n) for n in ['rinbo_cali','rinbo_standing','rinbo_tripod','rinbo_manual','rinbo_legs']]
pairs.append((bridge/'rinbo_ros_bridge',root/'build/rinbo_ros_bridge/rinbo_ros_bridge'))
for _,dst in pairs:assert digest(dst)==before[str(dst)],'Deployed binary changed: '+str(dst)
assert digest(site)==before[str(site)],'Site changed'
config=root/'src/rinbo_ros_bridge/config/redrhex_safe.yaml';assert digest(config)==before[str(config)],'Bridge config changed'
# Observe only. An existing bridge is not killed or restarted by deployment.
active_bridge=[]
for p in Path('/proc').iterdir():
 if not p.name.isdecimal():continue
 try:
  if (p/'comm').read_text().strip().startswith('rinbo_ros_brid'):active_bridge.append(int(p.name))
 except OSError:pass
run([fsm/'rinbo_legs','assert-idle'])
for n in ['rinbo_cali','rinbo_standing','rinbo_tripod']:
 (e/(n+'-candidate-check.txt')).write_text(run([fsm/n,'--check-config']))
with open(str(site)+'.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 assert digest(site)==before[str(site)]
 staged=[]
 for src,dst in pairs:
  tmp=dst.with_name(dst.name+'.pwm3300-new');shutil.copy2(src,tmp)
  with tmp.open('rb') as f:os.fsync(f.fileno())
  assert digest(tmp)==digest(src);staged.append((tmp,dst))
 for tmp,dst in staged:os.replace(tmp,dst)
 # Publish the matching Bridge config only after its executable accepts 3300.
 text=config.read_text();assert text.count('motor_command_max_pwm: 80.0')==1
 tmp=config.with_name(config.name+'.pwm3300-new')
 with tmp.open('w') as f:
  f.write(text.replace('motor_command_max_pwm: 80.0','motor_command_max_pwm: 3300.0'));f.flush();os.fsync(f.fileno())
 os.chmod(tmp,config.stat().st_mode & 0o777);os.replace(tmp,config)
 for directory in {dst.parent for _,dst in pairs}|{config.parent}:
  fd=os.open(directory,os.O_RDONLY|os.O_DIRECTORY)
  try:os.fsync(fd)
  finally:os.close(fd)
cli=root/'build/rinbo_fsm/rinbo_legs'
prior=json.loads(run([cli,'status','--json']))
result=run([cli,'tune-limits','tripod','--expect-revision',str(prior['revision']),'max_pwm=3300']);(e/'apply.json').write_text(result)
status=run([cli,'status','--json']);(e/'deployed-config.json').write_text(status)
for n in ['rinbo_cali','rinbo_standing','rinbo_tripod']:
 (e/(n+'-deployed-check.txt')).write_text(run([root/'build/rinbo_fsm'/n,'--check-config']))
after=yaml.safe_load(site.read_text());old=yaml.safe_load((backup/str(site).lstrip('/')).read_text())
old['revision']+=1;old['parameters']['rinbo_tripod_rslip']['max_pwm']=3300.0
assert after==old,'Unexpected changes in site YAML'
assert yaml.safe_load((root/'install/rinbo_ros_bridge/share/rinbo_ros_bridge/config/redrhex_safe.yaml').read_text())['rinbo_ros2_bridge']['ros__parameters']['motor_command_max_pwm']==3300
record={'deployed_at':time.strftime('%Y-%m-%d %H:%M:%S %z'),'binaries':{str(dst):digest(dst) for _,dst in pairs},'revision':after['revision'],'site_sha256':digest(site),'bridge_config_sha256':digest(config),'bridge_pids_observed_before_deployment':active_bridge,'hardware_actions':[],'only_site_change':'rinbo_tripod_rslip.max_pwm=3300 plus revision'}
(e/'deployment.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record,indent=2))
