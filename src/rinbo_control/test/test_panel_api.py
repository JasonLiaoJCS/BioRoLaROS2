"""Offline controller/socket tests. Fake owns no ROS or hardware handles."""
import base64
import importlib.util
import json
from pathlib import Path
import socket
import threading
import time
import uuid

import pytest
from rinbo_control.panel_native import NativeFailure
from rinbo_control.panel_protocol import decode, validate
from rinbo_control.panel_server import Controller, Handler, Server


def request(action='Communications', **values):
    return dict(protocol=1, request_id=uuid.uuid4().hex, action=action,
                sbrio_ip='192.168.30.254', orin_ip='192.168.30.8', ros_domain_id=99,
                stream=False, **values)


class Fake:
    def __init__(self, root, emit):
        self.cancel = threading.Event()
        self.events = []
        self.powered = False
        self.target = None
        self.block = threading.Event()
        self.block.set()
        self.failed = None
        self.off_failed = None
        self.finish = None
        self.emit = emit
    def bind(self, req):
        value = (req['sbrio_ip'], req['orin_ip'], req['ros_domain_id'])
        if self.target and self.target != value:
            raise NativeFailure('target_changed')
        self.target = value
    def session_quiescent(self):return self.target is None
    def recover_stop(self, req):
        if self.off_failed:raise self.off_failed
        return self.urgent_off(req['request_id'])
    def poll(self): pass
    def status(self): return {'readiness': 'ready' if self.target else 'backend_unavailable'}
    def interrupt(self, action):
        self.cancel.set()
        self.events.append('interrupt:'+action)
    def urgent_off(self, rid, emergency=False):
        self.events.append('off')
        self.powered = False
        if self.off_failed: raise self.off_failed
    def stop_motion(self, **kwargs): self.events.append('stop_motion')
    def execute(self, req):
        self.events.append(req['action'])
        self.block.wait(3)
        if req['action'] not in ('Stop', 'StopMotion', 'EmergencyStop') and self.cancel.is_set():
            raise NativeFailure('preempted', 40)
        if self.failed: raise self.failed
        if req['action'] == 'PowerOn':
            previous = self.powered
            self.powered = True
            return {'status': 'already_satisfied' if previous else 'success'}
        if req['action'] == 'Communications':
            return {'recovery_verified': True, 'backend_retained': True}
        if req['action'] == 'ResetEmergency':
            return {'emergency_reset_verified': True, 'backend_retained': True}
        if req['action'] == 'Stop':
            self.target = None
            return {'verified_off': True, 'native_owned_services_cleanup': 'completed'}
        return {}


@pytest.fixture
def ctl(tmp_path):
    c = Controller(tmp_path, Fake)
    yield c
    c.native.block.set()
    c.shutdown.set()
    c.worker.join(4)


def done(c, req, statuses=('completed', 'failed', 'already_satisfied', 'running', 'state_unknown', 'preempted')):
    end = time.monotonic()+4
    while time.monotonic() < end:
        r = c.submit(req)
        if r['status'] in statuses:
            return r
        time.sleep(.01)
    pytest.fail(str(r))


@pytest.mark.parametrize('action', ['Communications','Calibration','Standing','Tripod','StopMotion','Stop','EmergencyStop'])
def test_native_dispatch_and_no_windows_session(ctl, action):
    req = request(action)
    assert ctl.submit(req)['status'] == 'started'
    assert done(ctl, req)['exit_code'] == 0
    assert action in ctl.native.events


def test_readonly_and_monitor_never_initialize(ctl):
    for action in ('ReadLogs', 'CheckConnection'):
        assert ctl.submit(request(action))['exit_code'] == 0
    assert ctl.native.events == []
    assert ctl.native.target is None


def test_duplicate_and_client_lost_reply_do_not_reexecute(ctl):
    req = request()
    ctl.submit(req)
    assert done(ctl, req)['replayed']
    for _ in range(5): ctl.submit(req)
    assert ctl.native.events.count('Communications') == 1
    changed = dict(req, action='PowerOn')
    assert ctl.submit(changed)['reason'] == 'request_id_conflict'


def test_repeated_power_keeps_on(ctl):
    first, second = request('PowerOn'), request('PowerOn')
    ctl.submit(first); assert done(ctl, first)['status'] == 'completed'
    ctl.submit(second); assert done(ctl, second)['status'] == 'already_satisfied'
    assert 'off' not in ctl.native.events


@pytest.mark.parametrize('action', ['Stop','EmergencyStop'])
def test_stop_preempts_blocked_start_and_cannot_be_overwritten(ctl, action):
    ctl.native.block.clear()
    power = request('PowerOn'); ctl.submit(power)
    stop = request(action); ctl.submit(stop)
    end = time.monotonic()+1
    while 'off' not in ctl.native.events and time.monotonic() < end: time.sleep(.01)
    assert 'off' in ctl.native.events  # Off before blocked worker is released
    assert ctl.submit(request('PowerOn'))['exit_code'] == 20
    ctl.native.block.set()
    assert done(ctl, power)['exit_code'] == 40
    assert done(ctl, stop)['exit_code'] == 0
    assert not ctl.native.powered
    if action == 'EmergencyStop':
        assert ctl.submit(request('PowerOn'))['exit_code'] == 20
        motionstop = request('StopMotion'); ctl.submit(motionstop); done(ctl, motionstop)
        assert ctl.latched


def test_native_exit_reason_and_off_failure_preserved(ctl):
    ctl.native.failed = NativeFailure('specific native rejection', 49, {'epoch':'real-epoch'})
    req = request(); ctl.submit(req)
    value = done(ctl, req)
    assert value['exit_code'] == 49 and value['reason'] == 'specific native rejection'
    ctl.finish(req['request_id'], 'failed', 31, 'later cleanup', None)
    assert done(ctl, req)['exit_code'] == 49
    ctl.native.failed = None
    ctl.native.off_failed = NativeFailure('ACK timeout', 30)
    stop = request('Stop'); ctl.submit(stop)
    assert done(ctl, stop)['exit_code'] == 30
    assert ctl.latched


def test_reboot_marks_pending_unknown_no_replay(tmp_path):
    c = Controller(tmp_path, Fake)
    req = request('Tripod'); c.submit(req); done(c, req)
    c.shutdown.set(); c.worker.join(2)
    second = Controller(tmp_path, Fake)
    try:
        value = second.submit(req)
        assert value['exit_code'] == 31
        assert second.native.events == [] and second.latched
    finally:
        second.shutdown.set(); second.worker.join(2)


def test_unsupported_is_not_success(ctl):
    for action in ('FpgaConsole','PhysicalCleanup'):
        assert ctl.submit(request(action))['exit_code'] == 21


@pytest.mark.parametrize('patch', [{'protocol':2}, {'password':'secret'}, {'ros_domain_id':True},
                                  {'sbrio_ip':'127.0.0.1; reboot'}, {'stream':True}, {'request_id':'x'}])
def test_strict_input(patch):
    with pytest.raises((ValueError, TypeError)):
        validate(dict(request(), **patch))


def test_real_socket_and_base64_client(ctl, tmp_path, capsys):
    path = tmp_path/'api.sock'
    server = Server(str(path), Handler); server.controller = ctl
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    client_path = Path(__file__).resolve().parents[3]/'tools/rslip_panel_client.py'
    spec = importlib.util.spec_from_file_location('panel_client_test', client_path)
    client = importlib.util.module_from_spec(spec); spec.loader.exec_module(client)
    client.SOCKET = path
    try:
        req = request()
        encoded = base64.b64encode(json.dumps(req).encode()).decode()
        assert decode(encoded) == req
        assert client.main(['--request-base64', encoded]) == 0
        assert json.loads(capsys.readouterr().out)['status'] == 'started'
        done(ctl, req)
        # Drop read-only streams repeatedly; native operation count cannot change.
        for _ in range(2):
            with socket.socket(socket.AF_UNIX) as s:
                s.settimeout(3); s.connect(str(path))
                monitor = request('ReadLogs'); monitor['stream'] = True
                s.sendall((json.dumps(monitor)+'\n').encode())
                received = b''
                deadline = time.monotonic()+3
                while b'RSLIP_MONITOR_HEARTBEAT=1' not in received and time.monotonic() < deadline:
                    received += s.recv(16384)
                assert b'RSLIP_MONITOR_HEARTBEAT=1' in received
        assert ctl.native.events.count('Communications') == 1
    finally:
        server.shutdown(); server.server_close(); thread.join(2)


def test_actual_subprocess_client_and_disconnected_request(ctl, tmp_path):
    import subprocess
    import sys
    path = tmp_path/'wire.sock'
    server = Server(str(path), Handler); server.controller = ctl
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    client = Path(__file__).resolve().parents[3]/'tools/rslip_panel_client.py'
    try:
        req = request('PowerOn')
        with socket.socket(socket.AF_UNIX) as s:
            s.connect(str(path)); s.sendall((json.dumps(req)+'\n').encode())
            # Simulate lost SSH reply. The operation remains owned by controller.
        assert done(ctl, req)['status'] == 'completed'
        encoded = base64.b64encode(json.dumps(req).encode()).decode()
        child = subprocess.run([sys.executable,str(client),'--socket',str(path),
                                '--request-base64',encoded], text=True, capture_output=True, timeout=4)
        assert child.returncode == 0
        assert json.loads(child.stdout)['replayed']
        assert ctl.native.events.count('PowerOn') == 1
    finally:
        server.shutdown(); server.server_close(); thread.join(2)


def test_emergency_can_preempt_stopmotion(ctl):
    ctl.native.block.clear()
    motionstop = request('StopMotion'); ctl.submit(motionstop)
    emergency = request('EmergencyStop'); assert ctl.submit(emergency)['status'] == 'started'
    ctl.native.block.set()
    assert done(ctl, emergency)['exit_code'] == 0
    assert ctl.latched and not ctl.native.powered


def test_reboot_keeps_target_and_emergency_latch(tmp_path):
    first = Controller(tmp_path, Fake)
    req = request('EmergencyStop'); first.submit(req); done(first, req)
    first.shutdown.set(); first.worker.join(2)
    second = Controller(tmp_path, Fake)
    try:
        changed = request('Stop'); changed['sbrio_ip'] = '192.168.30.2'
        assert second.submit(changed)['exit_code'] == 20
        motionstop = request('StopMotion'); second.submit(motionstop); done(second, motionstop)
        assert second.latched
    finally:
        second.shutdown.set(); second.worker.join(2)


def test_explicit_communications_recovers_stale_socket_without_replaying(ctl, tmp_path, monkeypatch, capsys):
    path = tmp_path/'stale.sock'
    with socket.socket(socket.AF_UNIX) as stale:
        stale.bind(str(path))
    spec=importlib.util.spec_from_file_location('panel_stale_client',Path(__file__).resolve().parents[3]/'tools/rslip_panel_client.py')
    client=importlib.util.module_from_spec(spec); spec.loader.exec_module(client)
    client.SOCKET=path
    calls=[]; servers=[]
    def start(command, **kwargs):
        from types import SimpleNamespace
        calls.append(command)
        path.unlink()
        server=Server(str(path),Handler); server.controller=ctl
        thread=threading.Thread(target=server.serve_forever,daemon=True); thread.start()
        servers.append((server,thread))
        return SimpleNamespace(returncode=0,stderr='')
    monkeypatch.setattr(client.subprocess,'run',start)
    req=request()
    try:
        encoded=base64.b64encode(json.dumps(req).encode()).decode()
        assert client.main(['--request-base64',encoded]) == 0
        done(ctl,req)
        assert len(calls)==1 and ctl.native.events.count('Communications')==1
    finally:
        for server,thread in servers:
            server.shutdown();server.server_close();thread.join(2)
