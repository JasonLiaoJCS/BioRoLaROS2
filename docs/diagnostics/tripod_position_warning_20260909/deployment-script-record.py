# Applies an already-tested release. Never starts a controller or changes power.
from pathlib import Path
import os,signal,time,fcntl,hashlib,json,shutil,subprocess
ws=Path('/home/jetson/rinbo_ros_ws')
backup=ws/'.codex-backups/tripod_position_warning_20260909'
report=ws/'docs/diagnostics/tripod_position_warning_20260909'
candidate=Path('/tmp/rinbo-tripod-fix-build')
site=Path('/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml')
names=['rinbo_tripod','rinbo_cali','rinbo_standing','rinbo_manual','rinbo_legs']
before=json.loads((backup/'before.json').read_text())
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
# Refuse to overwrite another session's changes to deployed files/config.
for p in [site]+[ws/'build/rinbo_fsm'/n for n in names]:
 if digest(p)!=before[str(p)]:raise RuntimeError(f'Live file changed since baseline: {p}')
for name in ['rinbo_cali.cpp','rinbo_standing.cpp','rinbo_manual.cpp']:
 p=ws/'src/rinbo_fsm/src'/name
 if digest(p)!=before[str(p)]:raise RuntimeError(f'Validated controller source changed: {p}')
# Release only the previously identified SAFETY_STOP controller, if still alive.
pid=75799;p=Path(f'/proc/{pid}');stopped=[]
if p.exists():
 fd=os.pidfd_open(pid)
 try:
  ticks=p.joinpath('stat').read_text().rsplit(')',1)[1].split()[19]
  boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
  if ticks!='814097' or boot!='b0008f58-5739-453d-be5f-aa3fa7d3bd72':raise RuntimeError('Tripod PID identity changed')
  if p.joinpath('exe').readlink()!=ws/'build/rinbo_fsm/rinbo_tripod':raise RuntimeError('Tripod executable path changed')
  if digest(p/'exe')!=before[str(ws/'build/rinbo_fsm/rinbo_tripod')]:raise RuntimeError('Tripod executable image changed')
  logfile=Path('/tmp/rslip-launcher/20260909-125159-092/rinbo_tripod.log')
  if p.joinpath('fd/1').readlink()!=logfile:raise RuntimeError('Tripod log identity changed')
  lines=logfile.read_text().splitlines()
  if not lines[-1].startswith('TRIPOD SAFETY STOP: hard position error: L2 counts=18020.730469'):
   raise RuntimeError('Expected previously stopped Tripod; refusing to interrupt an unknown active action')
  signal.pidfd_send_signal(fd,signal.SIGINT)
  for _ in range(50):
   if not p.exists() or p.joinpath('stat').read_text().rsplit(')',1)[1].split()[0]=='Z':break
   time.sleep(.1)
  else:raise RuntimeError('Old Tripod did not exit after SIGINT')
  stopped.append(pid)
 finally:os.close(fd)
# Shared lock blocks startup writers while the executable set is replaced.
# Actual mutation helper below takes exclusive lock after this one is released.
lock=os.open(str(site)+'.lock',os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW,0o600)
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
replaced=[]
try:
 for n in names:
  dest=ws/'build/rinbo_fsm'/n
  if digest(dest)!=before[str(dest)]:raise RuntimeError(f'Deployed binary changed: {dest}')
  stage=dest.with_name('.'+n+'.position-warning-stage')
  shutil.copy2(candidate/n,stage)
  with stage.open('rb') as f:os.fsync(f.fileno())
  os.replace(stage,dest);replaced.append(n)
 # Directory sync makes the new executable set durable before policy change.
 directory_fd=os.open(ws/'build/rinbo_fsm',os.O_RDONLY|os.O_DIRECTORY)
 os.fsync(directory_fd);os.close(directory_fd)
except BaseException:
 for n in reversed(replaced):
  dest=ws/'build/rinbo_fsm'/n;stage=dest.with_name('.'+n+'.rollback-stage')
  shutil.copy2(backup/'binaries'/n,stage);os.replace(stage,dest)
 raise
finally:
 fcntl.flock(lock,fcntl.LOCK_UN);os.close(lock)
# This command only edits configuration. It refuses any concurrently active
# action, and never enables power or starts a controller.
r=subprocess.run([str(ws/'build/rinbo_fsm/rinbo_legs'),'tripod-position-policy','warn'],capture_output=True,text=True)
if r.returncode:raise RuntimeError(f'Binaries deployed, policy update failed: {r.stderr}')
(report/'policy-apply.json').write_text(r.stdout)
# --check-config exits before ROS initialization and sends no motor commands.
checks={}
for n in ['rinbo_cali','rinbo_standing','rinbo_tripod']:
 r=subprocess.run([str(ws/'build/rinbo_fsm'/n),'--check-config'],capture_output=True,text=True)
 (report/(n+'-check-config.txt')).write_text(r.stdout+r.stderr)
 checks[n]=r.returncode
 if r.returncode:raise RuntimeError(f'New configuration rejected by {n}: {r.stderr}')
r=subprocess.run([str(ws/'build/rinbo_fsm/rinbo_legs'),'status','--json'],capture_output=True,text=True,check=True)
(report/'deployed-config.json').write_text(r.stdout)
config=json.loads(r.stdout)
assert config['parameters']['rinbo_tripod_rslip']['safety.stop_on_position_error'] is False
result={'stopped_old_safety_stop_pids':stopped,'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'deployed_sha256':{n:digest(ws/'build/rinbo_fsm'/n) for n in names},'check_config_exit_codes':checks,'revision':config['revision'],'hash':config['hash'],'policy':'warn_only','motion_started':False,'power_commands_sent':False}
(report/'deployment.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
