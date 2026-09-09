#!/usr/bin/env python3
"""Run ONLY the built fake_driver (no NI runtime and no networking)."""
import fcntl
import json
import os
from pathlib import Path
import pty
import resource
import signal
import struct
import subprocess
import sys
import termios
import time

ROOT = Path(sys.argv[1]).resolve()
EXE = ROOT / 'fake_driver'
RESULTS = []
children = []

def launch(name, mode='headless', extra=None, controlling=False, pipe=False, tty=False):
    trace = ROOT / (name + '.trace')
    trace.write_text('')
    log = ROOT / (name + '.log')
    env = dict(os.environ, TERM='xterm', FAKE_TRACE=str(trace)); env.update(extra or {})
    master = slave = None
    console = ROOT / 'console.log'
    offset = console.stat().st_size if console.exists() else 0
    if tty:
        master, slave = pty.openpty()
        fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 30, 100, 0, 0))
    def session():
        os.setsid()
        if controlling: fcntl.ioctl(0, termios.TIOCSCTTY, 0)
        if env.get('IGNORE_HUP'): signal.signal(signal.SIGHUP, signal.SIG_IGN)
    with log.open('wb') as output:
        p = subprocess.Popen([str(EXE)] + ([] if mode == 'auto' else ['--' + mode]),
            stdin=slave if tty else (subprocess.PIPE if pipe else subprocess.DEVNULL),
            stdout=slave if tty else output, stderr=slave if tty else output,
            env=env, preexec_fn=session)
    children.append(p)
    if slave is not None: os.close(slave)
    p.trace, p.log, p.offset, p.master = trace, log, offset, master
    # Wait for the driver loop or a deliberate startup refusal.
    end = time.monotonic() + 3
    screen = b''
    while time.monotonic() < end:
        if master is not None: screen += drain(master)
        if p.poll() is not None: break
        if 'Entering loop' in logs(p) and (not tty or mode=='headless' or b'Command>' in screen): break
        time.sleep(.01)
    return p

def drain(fd):
    os.set_blocking(fd, False)
    data = b''
    try:
        while True:
            chunk = os.read(fd, 65536)
            if not chunk: break
            data += chunk
    except (BlockingIOError, OSError): pass
    return data

def logs(p):
    console = ROOT / 'console.log'
    return p.log.read_text(errors='replace') + (console.read_bytes()[p.offset:].decode(errors='replace') if console.exists() else '')

def ticks(p):
    parts = Path(f'/proc/{p.pid}/stat').read_text().split(') ', 1)[1].split()
    return int(parts[11]) + int(parts[12])

def finish(name, p, expected=0, sig=signal.SIGINT, verify=True):
    if sig and p.poll() is None: p.send_signal(sig)
    p.wait(timeout=3)
    if p.master is not None:
        os.close(p.master); p.master = None
    assert p.returncode == expected, (name, p.returncode, logs(p)[-2000:])
    data = p.trace.read_text()
    if verify:
        assert 'shutdown_controls_verified' in logs(p), (name, logs(p)[-2000:])
        writes = [line for line in data.splitlines() if line.startswith(('WRITE ', 'OFF_WRITE '))]
        assert len(writes) >= 21
        assert all(line.endswith(' 0') for line in writes[-21:]), writes[-21:]
        assert len({line.split()[1] for line in writes[-21:]}) == 21
        assert data.rstrip().endswith('FINALIZE 0 0') and 'CLOSE 0 0' in data
        assert data.count('INITIALIZE ') == data.count('OPEN ') == data.count('RUN ') == 1
    RESULTS.append(dict(case=name, exit=p.returncode))

try:
    for name,mode,extra,pipe in [('devnull_unknown_TERM','auto',{'TERM':'unknown'},False),
                                ('pipe_EOF_headless','auto',{},True),
                                ('explicit_headless','headless',{},False)]:
        p = launch(name,mode,extra,pipe=pipe)
        if pipe: p.stdin.close()
        a=ticks(p); time.sleep(.6); cpu=ticks(p)-a
        assert p.poll() is None and cpu <= 12, (name,cpu)
        assert '\x1b' not in logs(p)
        finish(name,p); RESULTS[-1]['cpu_ticks_0_6s']=cpu
    for sig in [signal.SIGHUP, signal.SIGTERM, signal.SIGINT]:
        name='signal_'+sig.name
        p=launch(name,extra={'IGNORE_HUP':'1','FAKE_COMMANDS':'1'})
        finish(name,p,2 if sig==signal.SIGHUP else 0,sig)
    for control in [False,True]:
        name='controlling_PTY_disconnect' if control else 'PTY_loss_without_SIGHUP'
        p=launch(name,'interactive',controlling=control,tty=True)
        a_ticks=ticks(p); a=resource.getrusage(resource.RUSAGE_CHILDREN); start=time.monotonic();os.close(p.master);p.master=None
        p.wait(timeout=3)
        elapsed=time.monotonic()-start
        finish(name,p,2,sig=None)
        assert elapsed<1.5
        RESULTS[-1]['exit_latency_s']=round(elapsed,3)
        b=resource.getrusage(resource.RUSAGE_CHILDREN)
        cpu=(b.ru_utime+b.ru_stime-a.ru_utime-a.ru_stime)*os.sysconf('SC_CLK_TCK')-a_ticks
        RESULTS[-1]['cpu_ticks_after_loss']=round(cpu,2)
        assert cpu<12
    p=launch('interactive_CtrlD','interactive',tty=True)
    os.write(p.master,b'\x04');finish('interactive_CtrlD',p,2,sig=None)
    p=launch('interactive_unknown_TERM','interactive',{'TERM':'unknown'},tty=True)
    finish('interactive_unknown_TERM',p,2,sig=None)
    p=launch('interactive_SIGINT','interactive',tty=True)
    os.write(p.master,b':P D 1\n'); time.sleep(.15)
    finish('interactive_SIGINT',p)
    p=launch('immediate_ERR_bounded_CPU','interactive',{'FAKE_GETCH_ERR':'1'},tty=True)
    a=ticks(p); time.sleep(.6); cpu=ticks(p)-a
    assert p.poll() is None and cpu<=12
    finish('immediate_ERR_bounded_CPU',p);RESULTS[-1]['cpu_ticks_0_6s']=cpu
    p=launch('readable_but_ERR','interactive',{'FAKE_GETCH_ERR':'1'},tty=True)
    os.write(p.master,b'x');finish('readable_but_ERR',p,2,sig=None)
    p=launch('poll_ERR','interactive',{'FAKE_POLL_ERR':'1'},tty=True)
    finish('poll_ERR',p,2,sig=None)
    # Nonblocking UI output must not leak flags back into its parent shell.
    master,slave=pty.openpty()
    saved=fcntl.fcntl(slave,fcntl.F_GETFL)
    trace=ROOT/'parent_flags.trace';trace.write_text('')
    child=subprocess.Popen([str(EXE),'--interactive'],stdin=slave,stdout=slave,stderr=slave,
        env=dict(os.environ,TERM='xterm',FAKE_TRACE=str(trace)),start_new_session=True)
    children.append(child)
    end=time.monotonic()+3
    while time.monotonic()<end:
        if b'Command>' in drain(master): break
        time.sleep(.01)
    child.send_signal(signal.SIGINT);child.wait(timeout=3)
    assert child.returncode==0 and fcntl.fcntl(slave,fcntl.F_GETFL)==saved
    os.close(slave);os.close(master)
    RESULTS.append(dict(case='parent_TTY_flags_restored',exit=child.returncode))
    p=launch('explicit_interactive_no_TTY','interactive')
    finish('explicit_interactive_no_TTY',p,64,sig=None,verify=False)
    assert p.trace.read_text()==''
    p=launch('duplicate_owner')
    q=launch('duplicate_refusal')
    finish('duplicate_refusal',q,3,sig=None,verify=False)
    assert q.trace.read_text()==''
    finish('duplicate_owner',p)
    for injection in ['FAKE_WRITE_FAIL','FAKE_READ_FAIL','FAKE_MISMATCH']:
        p=launch(injection,extra={injection:'1'})
        finish(injection,p,3,verify=False)
        assert 'shutdown_FAILED PowerUnknown' in logs(p)
        assert 'shutdown_controls_verified' not in logs(p)
        lines=p.trace.read_text().splitlines()
        assert sum(x.startswith('OFF_WRITE') for x in lines)>=21
        off_writes=[x for x in lines if x.startswith('OFF_WRITE')][-21:]
        assert len({x.split()[1] for x in off_writes})==21
        assert all(x.endswith(' 0') for x in off_writes)
        assert sum(x.startswith('OFF_READ') for x in lines)>=21
    p=launch('module_IO_fault',extra={'FAKE_MODULE_FAIL':'1'})
    finish('module_IO_fault',p,2,sig=None)
    p=launch('open_failure',extra={'FAKE_OPEN_FAIL':'1'})
    finish('open_failure',p,3,sig=None,verify=False)
    assert 'RUN ' not in p.trace.read_text() and 'CLOSE ' not in p.trace.read_text()
    # Monitoring is a separate read-only process. Its closure cannot signal driver.
    p=launch('monitor_closure')
    monitor=subprocess.Popen(['tail','-f',str(p.log)], stdout=subprocess.DEVNULL)
    monitor.terminate();monitor.wait(timeout=2)
    time.sleep(.2);assert p.poll() is None
    finish('monitor_closure',p)
    # Curses output backpressure must not block safe shutdown.
    p=launch('unread_TTY_output','interactive',tty=True)
    time.sleep(.5)
    finish('unread_TTY_output',p)
finally:
    for p in children:
        if p.poll() is None:
            p.kill();p.wait()
    (ROOT/'results.json').write_text(json.dumps(RESULTS,indent=2)+'\n')
print(json.dumps({'CLK_TCK':os.sysconf('SC_CLK_TCK'),'passed':len(RESULTS),'results':RESULTS},indent=2))
