python3 - <<'PY'
import os,subprocess,signal,time,pty,fcntl,termios,struct,json
root='/tmp/rslip-diagnosis-offline-20260909'
exe=root+'/harness'
env=dict(os.environ,TERM='unknown')
results=[]
def ticks(p):
    s=open('/proc/%d/stat'%p.pid).read().split(') ',1)[1].split()
    return int(s[11])+int(s[12])
def finish(p):
    if p.poll() is None: p.send_signal(signal.SIGINT)
    out,err=p.communicate(timeout=4)
    return {'exit':p.returncode,'escape_output':b'\x1b' in (out or b''),'finished':b'HARNESS_EXIT_OK' in (out or b'')}
for name,stdin,stdout in [('devnull_unknown_TERM',subprocess.DEVNULL,subprocess.PIPE),('pipe_EOF',subprocess.PIPE,subprocess.PIPE)]:
    p=subprocess.Popen([exe],stdin=stdin,stdout=stdout,stderr=subprocess.PIPE,env=env)
    if name=='pipe_EOF': p.stdin.close(); p.stdin=None
    time.sleep(.4)
    alive=p.poll() is None
    r=finish(p);r.update(case=name,alive_before_stop=alive);results.append(r)
for ignored in [False,True]:
    def disposition():
        signal.signal(signal.SIGHUP,signal.SIG_IGN if ignored else signal.SIG_DFL)
    p=subprocess.Popen([exe],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=env,preexec_fn=disposition)
    time.sleep(.3);p.send_signal(signal.SIGHUP);time.sleep(.2)
    alive=p.poll() is None
    r=finish(p);r.update(case='SIGHUP_ignored' if ignored else 'SIGHUP_default',alive_after_hup=alive);results.append(r)
# PTY loss independently from SIGHUP: no controlling terminal, close both parent FDs.
m,s=pty.openpty();fcntl.ioctl(s,termios.TIOCSWINSZ,struct.pack('HHHH',30,100,0,0))
p=subprocess.Popen([exe],stdin=s,stdout=s,stderr=s,env=dict(env,TERM='xterm'),start_new_session=True)
os.close(s);time.sleep(.5)
os.close(m)
time.sleep(.2)
a=ticks(p);time.sleep(.6);b=ticks(p)
alive=p.poll() is None
p.send_signal(signal.SIGINT);p.wait(timeout=4)
results.append({'case':'PTY_disappears_without_SIGHUP','alive_after_loss':alive,'cpu_ticks_in_0_6_seconds':b-a,'CLK_TCK':os.sysconf('SC_CLK_TCK'),'exit_after_INT':p.returncode})
# Recreate controlling-terminal hangup, matching foreground SSH rather than nohup.
m,s=pty.openpty();fcntl.ioctl(s,termios.TIOCSWINSZ,struct.pack('HHHH',30,100,0,0))
def terminal_session():
    os.setsid();fcntl.ioctl(0,termios.TIOCSCTTY,0);signal.signal(signal.SIGHUP,signal.SIG_DFL)
p=subprocess.Popen([exe],stdin=s,stdout=s,stderr=s,env=dict(env,TERM='xterm'),preexec_fn=terminal_session)
os.close(s);time.sleep(.5);os.close(m)
try: p.wait(timeout=4)
except subprocess.TimeoutExpired: p.send_signal(signal.SIGINT);p.wait(timeout=4)
results.append({'case':'controlling_PTY_disconnect','exit':p.returncode})
print(json.dumps(results,indent=2))
open(root+'/lifecycle-results.json','w').write(json.dumps(results,indent=2))
PY
