"""Native adapter tests; no ROS creation, SSH, motor or power commands."""
from types import SimpleNamespace
from pathlib import Path
import threading
import time

import pytest
from rinbo_control.panel_native import Native, NativeFailure
from rinbo_control.runtime import Child, Runtime
from rinbo_control.sbrio import RemoteServices


def test_constructor_is_inert_and_domain_is_explicit(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','99')
    native = Native(tmp_path, lambda *args:None)
    assert native.runtime.feedback is None and native.runtime.remote is None
    assert native.status()['readiness'] == 'backend_unavailable'
    with pytest.raises(NativeFailure, match='domain_mismatch'):
        native.bind(dict(sbrio_ip='192.168.30.254',orin_ip='192.168.30.8',ros_domain_id=12))


def test_emergency_fences_new_launch_before_off(tmp_path):
    native = Native(tmp_path, lambda *args:None)
    native.interrupt('EmergencyStop')
    native.runtime._start = lambda *args: pytest.fail('power child must not be started')
    with pytest.raises(NativeFailure, match='preempted'):
        native.power('ensure-on')


def test_noninteractive_ssh_uses_native_credentials_or_batch(monkeypatch, tmp_path):
    from rinbo_control import sbrio
    remote = RemoteServices(tmp_path, lambda *args:None, noninteractive=True)
    remote._load('192.168.30.254',50051)
    monkeypatch.setattr(sbrio,'automatic_auth',lambda *args:([],None))
    calls=[]
    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=255,stdout='Permission denied (publickey,password).')
    monkeypatch.setattr(sbrio.subprocess,'run',run)
    with pytest.raises(RuntimeError, match='Permission denied'):
        remote._run('start')
    assert 'BatchMode=yes' in calls[0] and 'StrictHostKeyChecking=yes' in calls[0]


def test_first_safety_reason_survives_large_later_output(tmp_path):
    import os
    import sys
    env = dict(os.environ)
    child=Child([sys.executable,'-c',"print('TRIPOD SAFETY STOP: first cause'); print('later output'*2000)"],env,tmp_path/'child.log')
    child.process.wait(3); child.reader.join(1)
    assert child.fatal.is_set()
    assert child.first_failure == 'TRIPOD SAFETY STOP: first cause'
    assert 'first cause' not in child.tail


def test_normal_motion_stop_uses_native_eight_second_pidfd_path(tmp_path, monkeypatch):
    from rinbo_control import panel_native as module
    native=Native(tmp_path,lambda *args:None)
    events=[]
    native.runtime.executable=lambda *args:args[-1]
    native.runtime.idle=lambda:events.append('idle')
    native.runtime.probe=lambda:SimpleNamespace(wait_motor_stopped=lambda cancel:(events.append('fresh_disabled') or {'verified':True,'motor_output_enabled':False}))
    monkeypatch.setattr(module,'find_motion',lambda *args:['identity'])
    monkeypatch.setattr(module,'stop_motion',lambda processes,*args:events.append(('native_stop',processes)))
    assert native.stop_motion()['verified']
    assert events == [('native_stop',['identity']),'fresh_disabled']


def test_explicit_stop_preserves_motion_fault_and_still_verifies_disabled(tmp_path, monkeypatch):
    from rinbo_control import panel_native as module
    native = Native(tmp_path, lambda *args:None)
    native.runtime.executable = lambda *args:args[-1]
    native.runtime.idle = lambda:None
    observed=[]
    native.runtime.probe=lambda:SimpleNamespace(wait_motor_stopped=lambda cancel:(observed.append('fresh_disabled') or {'verified':True,'motor_output_enabled':False}))
    monkeypatch.setattr(module,'find_motion',lambda *args:[])
    monkeypatch.setattr(module,'stop_motion',lambda *args:None)
    failure=threading.Event(); failure.set()
    native.motion=SimpleNamespace(process=SimpleNamespace(wait=lambda **kw:1,returncode=1),
                                 reader=SimpleNamespace(join=lambda *args:None),fatal=failure,
                                 first_failure='STANDING SAFETY STOP: original reason',tail='later output')
    native.motion_id='prior'; native.motion_kind='Standing'
    native.finish=lambda *args:observed.append(args)
    value=native.stop_motion(tolerate_exit_failure=True)
    assert value['verified'] and value['prior_motion_failure']=='STANDING SAFETY STOP: original reason'
    assert observed[0][1:4] == ('failed',1,'STANDING SAFETY STOP: original reason')
    assert observed[1]=='fresh_disabled'
