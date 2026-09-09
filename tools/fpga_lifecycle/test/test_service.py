#!/usr/bin/env python3
"""Exercise the actual lifecycle shell script against fake_driver, in temp state."""
import os
import signal
from pathlib import Path
import subprocess
import sys
import tempfile
import time
root=Path(__file__).resolve().parents[1]
build=Path(sys.argv[1]).resolve()
with tempfile.TemporaryDirectory(prefix='rslip-fake-service-') as directory:
    state=Path(directory)/'state'
    script=Path(directory)/'fpga-service'
    source=(root/'fpga-service').read_text().replace(
        'root=/home/admin/.local/state/rslip-fpga-service',f'root={state}').replace(
        'driver=/home/admin/rinbo_sbRIO_ws/rinbo_fpga_driver/build/fpga_driver',f'driver={build}/fake_driver')
    script.write_text(source);script.chmod(0o700)
    env=dict(os.environ,FAKE_TRACE=str(Path(directory)/'trace'))
    def call(action,good=True):
        p=subprocess.run([str(script),action],env=env,text=True,capture_output=True,timeout=15)
        if good: assert p.returncode==0,(action,p.stdout,p.stderr)
        else: assert p.returncode!=0,(action,p.stdout,p.stderr)
        return p
    # Read-only and stop paths cannot bootstrap a missing service.
    for action in ['watch','status','stop']: call(action,False)
    assert not (Path(directory)/'trace').exists()
    legacy=Path(directory)/'legacy-service'
    legacy.write_text(source.replace(f'driver={build}/fake_driver','driver=/bin/true'))
    legacy.chmod(0o700)
    rejected=subprocess.run([str(legacy),'start'],env=env,text=True,capture_output=True,timeout=3)
    assert rejected.returncode==30 and 'Lifecycle-capable driver not installed' in rejected.stderr
    assert not state.exists()

    pid=None
    try:
        call('start');call('status')
        run=state/(state/'current').read_text().strip()
        identity=(run/'identity').read_text()
        pid=int(identity.split()[1])
        stat=Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split()
        assert stat[4]=='0',stat  # controlling TTY number
        assert os.readlink(f'/proc/{pid}/fd/0')=='/dev/null'
        call('start');assert (run/'identity').read_text()==identity
        watcher=subprocess.Popen([str(script),'watch'],env=env,stdout=subprocess.DEVNULL)
        time.sleep(.2);watcher.terminate();watcher.wait(timeout=2)
        call('status')
        call('stop');call('status',False)
        assert (run/'exitcode').read_text().strip()=='0'
        log=(run/'driver.log').read_text()
        assert 'FPGA_DRIVER_WAIT_EXIT=0' in log and 'shutdown_controls_verified' in log
        assert (Path(directory)/'trace').read_text().count('INITIALIZE ')==1
        call('stop',False)
        print('PASS: legacy-binary refusal, missing-service refusal, detached launch, idempotent start, monitor close, identity stop, wait exit receipt; one fake initialization')
    finally:
        if pid and Path(f'/proc/{pid}/exe').exists() and os.readlink(f'/proc/{pid}/exe')==str(build/'fake_driver'):
            os.kill(pid,signal.SIGINT)
            time.sleep(.2)
