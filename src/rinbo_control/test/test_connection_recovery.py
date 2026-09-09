"""Local fake processes and injected runtime collaborators; no robot services."""
from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from rinbo_control import connection_recovery as recovery
from rinbo_control.runtime import Runtime


@pytest.fixture
def fake_motion(tmp_path):
    source = tmp_path/'fake.c'
    executable = tmp_path/'fake_motion'
    source.write_text('''
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
int main(int argc,char **argv) {
  signal(SIGINT, argc > 1 ? SIG_IGN : SIG_DFL);
  puts("ready"); fflush(stdout);
  for (;;) pause();
}
''')
    subprocess.run(['cc', str(source), '-o', str(executable)], check=True)
    children = []
    def start(ignore=False, domain='231'):
        child = subprocess.Popen([str(executable), *(['ignore'] if ignore else [])],
                                 stdout=subprocess.PIPE, text=True,
                                 env=dict(os.environ, ROS_DOMAIN_ID=domain))
        children.append(child)
        assert child.stdout.readline().strip() == 'ready'
        return child
    yield executable, start
    for child in children:
        if child.poll() is None:
            child.kill()
        child.wait(timeout=2)
        child.stdout.close()


def test_only_exact_executable_and_domain_qualify(fake_motion):
    executable, start = fake_motion
    matching, other_domain = start(), start(domain='232')
    matches = recovery.find_motion([executable], '231')
    assert [p.pid for p in matches] == [matching.pid]
    assert not recovery.find_motion([executable.with_name('other')], '231')
    assert other_domain.poll() is None


def test_normal_sigint_stops_only_selected_process_and_logs_identity(fake_motion, tmp_path):
    executable, start = fake_motion
    selected, other = start(), start(domain='232')
    recovery.stop_motion(recovery.find_motion([executable], '231'), tmp_path, lambda _:None)
    assert selected.wait(timeout=1) == -2
    assert other.poll() is None
    log, = tmp_path.glob('connection-stop-*.json')
    data = json.loads(log.read_text())
    assert data['processes'][0]['start_ticks'] > 0
    assert [event['event'] for event in data['events']] == ['SIGINT', 'exited']


def test_reused_identity_cannot_signal_live_process(fake_motion, tmp_path):
    _, start = fake_motion
    child = start()
    identity = recovery.read_process(child.pid)
    with pytest.raises(RuntimeError, match='身分已改變'):
        recovery.stop_motion([replace(identity, start_ticks=identity.start_ticks+1)],
                             tmp_path, lambda _:None)
    assert child.poll() is None


def test_hung_motion_is_not_force_killed(fake_motion, tmp_path):
    _, start = fake_motion
    child = start(ignore=True)
    with pytest.raises(RuntimeError, match='未正常停止'):
        recovery.stop_motion([recovery.read_process(child.pid)], tmp_path, lambda _:None, timeout=.05)
    assert child.poll() is None
    log, = tmp_path.glob('connection-stop-*.json')
    assert [e['event'] for e in json.loads(log.read_text())['events']] == ['SIGINT', 'incomplete']


def test_disappeared_process_is_harmless(fake_motion, tmp_path):
    _, start = fake_motion
    child = start()
    identity = recovery.read_process(child.pid)
    child.terminate(); child.wait(timeout=1)
    recovery.stop_motion([identity], tmp_path, lambda _:None)
    log, = tmp_path.glob('connection-stop-*.json')
    assert json.loads(log.read_text())['events'][0]['event'] == 'already-exited'


def test_connection_recovery_serializes_panels(tmp_path):
    with recovery.recovery_lock(tmp_path):
        with pytest.raises(RuntimeError, match='另一個控制台'):
            with recovery.recovery_lock(tmp_path):
                pytest.fail('concurrent recovery must not run')
    with recovery.recovery_lock(tmp_path):
        pass


@pytest.fixture
def runtime(tmp_path, monkeypatch):
    events = []
    h = Runtime(tmp_path, progress=lambda _:None)
    h.executable = lambda package, name:'/fake/'+name
    monkeypatch.setattr('rinbo_control.runtime.route_source', lambda _:'192.168.30.8')
    monkeypatch.setattr(recovery, 'find_motion', lambda *a:[SimpleNamespace(pid=123)])
    monkeypatch.setattr(recovery, 'stop_motion', lambda *a:events.append('stop-motion'))
    h.feedback = SimpleNamespace(spin=lambda _:None, bridge_count=lambda:1,
                                 bridge_ip=lambda:'192.168.30.254',
                                 wait_motor_disabled=lambda:events.append('disabled'))
    h.idle = lambda:events.append('idle')
    h.connect = lambda *a:events.append('connect') or {'motor_fresh':True}
    return h, events


def test_step1_stops_then_verifies_then_connects(runtime):
    h, events = runtime
    h.calibration_epoch = 1
    assert h.reconnect('192.168.30.254')['motor_fresh']
    assert events == ['stop-motion', 'idle', 'disabled', 'connect', 'idle', 'disabled']
    assert h.calibration_epoch is None


def test_step1_no_motion_preserves_calibration_and_does_not_power_cycle(runtime, monkeypatch):
    h, events = runtime
    h.calibration_epoch = 1
    monkeypatch.setattr(recovery, 'find_motion', lambda *a:[])
    h.reconnect('192.168.30.254')
    assert events == ['stop-motion', 'idle', 'connect', 'idle']
    assert h.calibration_epoch == 1


def test_step1_unknown_publisher_blocks_bootstrap(runtime):
    h, events = runtime
    def busy(): raise RuntimeError('unknown publisher')
    h.idle = busy
    with pytest.raises(RuntimeError, match='unknown publisher'):
        h.reconnect('192.168.30.254')
    assert events == ['stop-motion']


def test_step1_failed_stop_blocks_bootstrap(runtime, monkeypatch):
    h, events = runtime
    def failed(*args): raise RuntimeError('stop failed')
    monkeypatch.setattr(recovery, 'stop_motion', failed)
    with pytest.raises(RuntimeError, match='stop failed'):
        h.reconnect('192.168.30.254')
    assert events == []


def test_step1_unverified_disabled_blocks_bootstrap(runtime):
    h, events = runtime
    def failed(): raise RuntimeError('disable unverified')
    h.feedback.wait_motor_disabled = failed
    with pytest.raises(RuntimeError, match='disable unverified'):
        h.reconnect('192.168.30.254')
    assert events == ['stop-motion', 'idle']


def test_step1_different_robot_does_not_stop_processes(runtime):
    h, events = runtime
    with pytest.raises(RuntimeError, match='另一個 sbRIO'):
        h.reconnect('192.168.30.2')
    assert not events


def test_step1_refreshes_only_its_own_ssh_master(runtime, monkeypatch):
    h, events = runtime
    h.remote = SimpleNamespace(refresh_transport=lambda:events.append('close-private-ssh'), owned=True)
    original = h.remote
    h.reconnect('192.168.30.254')
    assert h.remote is original and h.remote.owned
    assert events.index('close-private-ssh') < events.index('connect')
