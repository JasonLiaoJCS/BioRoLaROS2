import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
import pytest
from rinbo_control.runtime import Child, Runtime


class Harness(Runtime):
    def __init__(self, tmp_path, failure=None):
        super().__init__(tmp_path, progress=lambda _:None)
        self.events=[]; self.failure=failure; self.sensors=False; self.valid=False
        self.feedback=SimpleNamespace(fresh=lambda _:True,spin=lambda _:None,close=lambda:None,
                                      wait_motor_disabled=lambda _:self.events.append('motor-safe'),sensor_epoch=0)

    def site(self): return {'hash':'current', 'disabled_legs':[]}
    def idle(self): self.events.append('idle')
    def status(self): return {'sensors_on':self.sensors, 'motor_fresh':True, 'relay_on':False}
    def needs_calibration(self,path): return not self.valid
    def calibration_ready(self,path): return self.valid
    def _run(self, package, name, *args, **kwargs):
        step = args[0] if name=='rinbo_power_tool' else ('calibration' if name=='rinbo_cali' else 'motion')
        self.events.append(step)
        if step==self.failure:
            raise RuntimeError('injected failure')
        if step=='sequence': self.sensors=True
        if step=='calibration': self.valid=True


def test_full_workflow_order_and_success_retains_sensors(tmp_path):
    h=Harness(tmp_path)
    h.execute('plan','current',True)
    assert h.events==['idle','off','sequence','motor-safe','relay','calibration','idle','motor-safe','motion','motor-safe','sensors']
    assert h.power_touched  # exit still needs to turn sensor supply off
    assert h.close()
    assert h.events[-1]=='off'


@pytest.mark.parametrize('failure',['sequence','relay','calibration','motion','sensors'])
def test_every_powered_failure_attempts_verified_off(tmp_path,failure):
    h=Harness(tmp_path,failure)
    with pytest.raises(RuntimeError):
        h.execute('plan','current',True)
    assert h.events[-1]=='off'
    assert not h.power_touched


def test_no_hidden_calibration_when_review_approved_reuse(tmp_path):
    h=Harness(tmp_path)
    with pytest.raises(RuntimeError,match='狀態已改變'):
        h.execute('plan','current',False)
    assert h.events==['idle']


def test_changed_site_never_starts_power(tmp_path):
    h=Harness(tmp_path)
    with pytest.raises(RuntimeError,match='已改變'):
        h.execute('plan','previous',True)
    assert not h.events


def test_reuse_does_not_cycle_sensor_power(tmp_path):
    h=Harness(tmp_path); h.sensors=h.valid=True
    h.calibration_epoch=0
    h.execute('plan','current',False)
    assert h.events==['idle','motor-safe','relay','motion','motor-safe','sensors']


def test_sensor_interruption_after_calibration_prevents_motion(tmp_path):
    h=Harness(tmp_path)
    def check_receipt(_):
        h.feedback.sensor_epoch += 1
        return True
    h.calibration_ready=check_receipt
    with pytest.raises(RuntimeError,match='回讀曾中斷'):
        h.execute('plan','current',True)
    assert 'motion' not in h.events
    assert h.events[-1]=='off'


def test_off_failure_does_not_claim_success(tmp_path):
    h=Harness(tmp_path,'off')
    messages=[]; h.progress=messages.append
    assert not h.safe_off()
    assert h.power_touched
    assert '無法確認關電成功' in messages[-1]


def test_process_output_markers_and_owned_shutdown(tmp_path):
    script=tmp_path/'fake.py'
    script.write_text("import signal,time\nsignal.signal(signal.SIGINT,lambda *a:exit(0))\nprint('State: DONE',flush=True)\ntime.sleep(60)\n")
    child=Child([sys.executable,str(script)],dict(os.environ),tmp_path/'child.log')
    try:
        assert child.done_marker.wait(3)
        assert not child.fatal.is_set()
    finally:
        child.stop()
    assert child.process.returncode==0
    assert 'State: DONE' in (tmp_path/'child.log').read_text()


@pytest.mark.parametrize('ignore_interrupt', [False, True])
def test_parent_death_terminates_owned_child(tmp_path, ignore_interrupt):
    pidfile=tmp_path/'pid'
    script=tmp_path/'parent.py'
    script.write_text("from rinbo_control.runtime import Child\nimport os,sys,time,signal\nfrom pathlib import Path\n"
                      + ("signal.signal(signal.SIGINT,signal.SIG_IGN)\n" if ignore_interrupt else "") +
                      "c=Child([sys.executable,'-c','import time; time.sleep(60)'],dict(os.environ),sys.argv[2])\n"
                      "Path(sys.argv[1]).write_text(str(c.process.pid))\ntime.sleep(60)\n")
    parent=subprocess.Popen([sys.executable,str(script),str(pidfile),str(tmp_path/'child.log')])
    child_pid=None
    try:
        end=time.monotonic()+3
        while not pidfile.exists() and time.monotonic()<end: time.sleep(.02)
        assert pidfile.exists()
        child_pid=int(pidfile.read_text())
        time.sleep(.15)  # allow guardian to exec its payload
        parent.kill(); parent.wait(timeout=3)
        end=time.monotonic()+3
        while time.monotonic()<end:
            status=Path(f'/proc/{child_pid}/status')
            if not status.exists() or '\nState:\tZ' in status.read_text():
                break
            time.sleep(.02)
        else:
            pytest.fail('child survived parent death')
    finally:
        if parent.poll() is None: parent.kill(); parent.wait(timeout=3)
        if child_pid:
            try: os.kill(child_pid,signal.SIGKILL)
            except ProcessLookupError: pass


@pytest.mark.parametrize('status, message', [('refused','拒絕連線'), ('timeout','等待逾時'),
                                            ('unreachable','網路無法到達')])
def test_failed_connection_cannot_start_bridge_or_power(tmp_path,monkeypatch,status,message):
    import json
    monkeypatch.setattr('rinbo_control.runtime.route_source',lambda _: '192.168.30.8')
    monkeypatch.setattr('rinbo_control.runtime.tcp_probe',lambda *a:dict(status=status,error='test'))
    h=Harness(tmp_path)
    h.remote=SimpleNamespace(start=lambda *a:None)
    def forbidden(*a):
        pytest.fail('failed TCP connection must not initialize ROS or start processes')
    h.probe=h._start=forbidden
    with pytest.raises(RuntimeError,match=message):
        h.connect('192.168.30.254')
    assert not h.events
    assert not h.power_touched
    logs=list(h.log_dir.glob('connection-*.json'))
    assert len(logs)==1 and json.loads(logs[0].read_text())['status']==status


def test_already_powered_only_reads_relay_feedback(tmp_path):
    h=Harness(tmp_path); h.sensors=h.valid=True; h.calibration_epoch=0
    h.status=lambda:dict(sensors_on=True,motor_fresh=True,relay_on=True)
    h.feedback.wait_power_ready=lambda *a:h.events.append('read-power')
    h.execute('plan','current',False)
    assert h.events==['idle','motor-safe','read-power','motion','motor-safe','sensors']


def test_power_waits_for_subscriber_without_retrying_command(tmp_path):
    h=Runtime(tmp_path)
    calls=[]
    h._run=lambda *a,**k:calls.append(a)
    h.power('relay')
    assert len(calls)==1
    assert calls[0][2:]==('relay','--wait-for-subscriber-s','8','--confirm-relay')


def test_unknown_off_retains_remote_services(tmp_path):
    h=Harness(tmp_path,'off')
    calls=[]
    h.remote=SimpleNamespace(owned=True,stop=lambda:calls.append('stop'),close=lambda:calls.append('close'))
    assert not h.close()
    assert calls==['close']


def test_normal_close_order_is_off_bridge_driver_core(tmp_path):
    h=Harness(tmp_path)
    h.remote=SimpleNamespace(owned=True,stop=lambda:h.events.append('remote-stop'),close=lambda:None)
    h.bridge=SimpleNamespace(stop=lambda:h.events.append('bridge-stop'),process=SimpleNamespace(returncode=0))
    assert h.close()
    assert h.events==['off','bridge-stop','remote-stop']


@pytest.mark.parametrize('fresh', [True, False])
def test_remote_bootstrap_precedes_tcp_and_bridge_needs_real_state(tmp_path,monkeypatch,fresh):
    from rinbo_control.runtime import Cancelled
    events=[]
    monkeypatch.setattr('rinbo_control.runtime.route_source',lambda _: '192.168.30.8')
    def tcp(*a):
        assert events==['remote']
        events.append('tcp')
        return dict(status='open',error='')
    monkeypatch.setattr('rinbo_control.runtime.tcp_probe',tcp)
    h=Runtime(tmp_path,progress=lambda _:None,cancel=lambda:True)
    h.remote=SimpleNamespace(start=lambda *a:events.append('remote'))
    h.feedback=SimpleNamespace(spin=lambda _:None,bridge_count=lambda:1,
                               bridge_ip=lambda:'192.168.30.254',
                               status=lambda:dict(bridge_count=1,motor_fresh=fresh,power_fresh=fresh))
    h._start=lambda *a:pytest.fail('must reuse existing Bridge; never start power here')
    if fresh:
        assert h.connect('192.168.30.254')['motor_fresh']
    else:
        with pytest.raises(Cancelled,match='通訊檢查'):
            h.connect('192.168.30.254')
    assert events==['remote','tcp'] and not h.power_touched


def test_tcp_probe_preserves_socket_failure_types(monkeypatch):
    import errno
    from rinbo_control.runtime import tcp_probe
    for error, status in [(ConnectionRefusedError(errno.ECONNREFUSED,'refused'),'refused'),
                          (TimeoutError('timed out'),'timeout'),
                          (OSError(errno.EHOSTUNREACH,'unreachable'),'unreachable'),
                          (OSError(errno.EACCES,'denied'),'error')]:
        def fail(*a, **kw):
            raise error
        monkeypatch.setattr('socket.create_connection',fail)
        assert tcp_probe('192.168.30.254',50051)['status']==status


def test_receipt_alone_cannot_reuse_calibration_after_sensor_gap(tmp_path):
    runtime=Runtime(tmp_path)
    runtime.feedback=SimpleNamespace(sensor_epoch=2)
    runtime.status=lambda:dict(sensors_on=True,motor_fresh=True)
    runtime.calibration_ready=lambda _:True
    assert runtime.needs_calibration('plan')  # new console must first calibrate
    runtime.calibration_epoch=2
    assert not runtime.needs_calibration('plan')
    runtime.feedback.sensor_epoch=3
    assert runtime.needs_calibration('plan')  # even after current telemetry recovers
