"""Button-sequence tests run the real Native/Controller with a fake wire/runtime.
No ROS initialization, SSH, service restart, recorder or hardware processes.
"""
import json
import threading
from types import SimpleNamespace as NS
import pytest
from rinbo_control.panel_native import Native, NativeFailure
from rinbo_control.panel_server import Controller
from test_panel_api import request, done


@pytest.fixture
def rig(tmp_path,monkeypatch):
    from rinbo_control import panel_native as module
    monkeypatch.setenv('ROS_DOMAIN_ID','99')
    h=NS(bridge=False,core=False,driver=False,motion=False,rails=0,estop=False,
         calls=[],bridge_starts=0,recorder=True,mirror=True,files=['unchanged.csv'],
         offline=False,fault=None,foreign=False,seq=0)
    class Remote:
        owned=True
        def stop(self):h.calls.append('stop_backend');h.core=h.driver=False
        def close(self):pass
    class Feedback:
        node=object()
        def close(self):h.calls.append('close_feedback')
        def wait_motor_stopped(self,cancel):
            if cancel():raise NativeFailure('cancelled',40)
            assert h.bridge
            if h.foreign:raise NativeFailure('other owner',31)
            return dict(verified=True,motor_output_enabled=False)
    class Runtime:
        def __init__(self,root,emit,cancel,**kw):
            self.cancel=cancel;self.env={'ROS_DOMAIN_ID':'99'};self.log_dir=tmp_path
            self.bridge=self.remote=self.feedback=self.connected_target=None;self.power_touched=False
        def executable(self,package,name):return name
        def reconnect(self,ip,local_ip,**kw):
            h.calls.append(('connect',kw))
            assert kw=={'verify_stopped':False,'verify_feedback':False,'recover_owned_bridge':True}
            if self.cancel():raise NativeFailure('cancelled',40)
            if h.offline:raise NativeFailure('網路無法到達',31)
            if h.foreign:raise NativeFailure('other owner',20)
            h.core=h.driver=True
            if not h.bridge:
                h.bridge=True;h.estop=False;h.bridge_starts+=1
            self.bridge=NS(process=NS(pid=999,poll=lambda:None))
            self.remote=Remote();self.feedback=Feedback();self.connected_target=(ip,50051,local_ip)
            return {'readiness':'starting'}
        def probe(self):return self.feedback
        def idle(self):
            if h.foreign:raise NativeFailure('other owner',31)
        def close(self):
            assert not self.power_touched
            self.connected_target=None
            return True
        def status(self):return dict(bridge_count=int(h.bridge),motor_fresh=h.bridge,power_fresh=h.bridge)
    monkeypatch.setattr(module,'Runtime',Runtime)
    def find(names,domain):
        return (['bridge'] if 'rinbo_ros_bridge' in names and h.bridge else [])+(['motion'] if any('rinbo_tripod' in n for n in names) and h.motion else [])
    def stop(processes,*args):
        for p in processes:
            h.calls.append('stop_'+p)
            if p=='bridge':h.bridge=False
            elif p=='motion':h.motion=False
    monkeypatch.setattr(module,'find_motion',find);monkeypatch.setattr(module,'stop_motion',stop)
    def factory(root,emit):
        n=Native(root,emit);n.watch_power=lambda:None
        def power(mode,rid=None,urgent=False,assert_estop=False):
            h.calls.append((mode,rid));h.seq+=1
            if mode=='off':
                if not h.bridge:raise NativeFailure('ACK timeout: no backend',30,{'command_sent':True,'readiness':'backend_unavailable'})
                h.rails=0
                if assert_estop:h.estop=True
                return dict(exit_code=0,request_id=rid,epoch=str(h.bridge_starts),generation=1,
                            state_source='mock_bridge',acknowledgement=dict(request_id=rid,epoch=str(h.bridge_starts),generation=1,
                            digital=False,signal=False,power=False,ack_kind='command_correlated',estop_asserted=assert_estop,feedback_stamp_ns=h.seq))
            if not h.bridge:raise NativeFailure('backend unavailable',10,{'command_sent':False})
            if h.estop:raise NativeFailure('software_estop_latched',20,{'command_sent':False})
            if h.fault:raise NativeFailure(h.fault,20,{'command_sent':False})
            if mode=='ready':return dict(readiness='ready',command_sent=False,epoch=str(h.bridge_starts),generation=1,
                                        acknowledgement=dict(digital=bool(h.rails&1),signal=bool(h.rails&2),power=bool(h.rails&4),feedback_stamp_ns=h.seq))
            assert mode=='ensure-on'
            status='already_satisfied' if h.rails==7 else 'success';h.rails=7
            return dict(status=status,readiness='ready',command_sent=status!='already_satisfied')
        n.power=power
        return n
    c=Controller(tmp_path,factory)
    yield c,h,factory
    c.shutdown.set();c.worker.join(4)
    assert h.recorder and h.mirror and h.files==['unchanged.csv']


def click(c,action):
    r=request(action);v=c.submit(r)
    return done(c,r) if v['status']=='started' else v


def test_normal_and_repeated_power_stop_converge(rig):
    c,h,_=rig
    assert click(c,'Communications')['exit_code']==0 and h.rails==0
    assert c.native.off_results=={}
    assert click(c,'PowerOn')['status']=='completed' and h.rails==7
    calls=list(h.calls)
    assert click(c,'PowerOn')['status']=='already_satisfied' and h.rails==7
    assert not any(x[0]=='off' for x in h.calls[len(calls):] if isinstance(x,tuple))
    h.motion=True
    assert click(c,'StopMotion')['exit_code']==0 and not h.motion and h.rails==7
    assert click(c,'Stop')['data']['verified_off'] and not h.bridge
    before=list(h.calls)
    for _ in range(3):
        v=click(c,'Stop');assert v['status']=='already_satisfied' and v['data']['verified_off'] is None
    assert h.calls==before and not c.latched
    assert click(c,'Communications')['exit_code']==0
    assert click(c,'PowerOn')['exit_code']==0 and h.rails==7


def test_first_stop_without_backend_recovers_inside_one_operation(rig):
    c,h,_=rig
    v=click(c,'Stop')
    assert v['status']=='completed' and v['data']['verified_off']
    assert v['data']['initial_off_failure']['native_exit_code']==30
    assert not h.bridge and not h.core and not c.latched


def test_recovery_failure_then_one_communications_then_power(rig):
    c,h,_=rig;h.offline=True
    v=click(c,'Stop');assert v['status']=='state_unknown' and c.latched
    assert not v['emergency_latched'] and v['control']['recovery_action']=='Communications'
    h.offline=False
    v=click(c,'Communications')
    assert v['data']['recovery_verified'] and not c.latched and h.bridge and h.rails==0
    assert click(c,'PowerOn')['exit_code']==0


def test_legacy_residual_latch_recovers_without_stop_or_bridge_restart(rig):
    c,h,_=rig;click(c,'Communications');before=h.bridge_starts
    c.set_latch(True,reason='legacy stale metadata')
    assert click(c,'Communications')['status']=='completed'
    assert not c.latched and h.bridge_starts==before and h.rails==0
    assert click(c,'PowerOn')['exit_code']==0


def test_true_emergency_needs_only_explicit_reset_then_power(rig):
    c,h,_=rig;click(c,'Communications');click(c,'PowerOn')
    assert click(c,'EmergencyStop')['exit_code']==0 and h.estop
    assert click(c,'PowerOn')['status']=='rejected'
    before=h.bridge_starts
    v=click(c,'Communications');assert v['exit_code']==20 and h.estop
    assert h.bridge_starts==before and c.emergency_context
    reset=click(c,'ResetEmergency')
    assert reset['exit_code']==0 and reset['data']['emergency_reset_verified']
    assert h.bridge_starts==before+1 and h.core and h.driver and h.rails==0 and not h.estop
    assert 'stop_backend' not in h.calls
    assert not c.latched and not c.emergency_context
    assert click(c,'PowerOn')['exit_code']==0


def test_stop_cleanup_does_not_reset_true_emergency(rig):
    c,h,_=rig;click(c,'Communications');click(c,'EmergencyStop')
    assert click(c,'Stop')['exit_code']==0 and not h.bridge
    assert c.emergency_context and click(c,'PowerOn')['status']=='rejected'
    reset=click(c,'ResetEmergency')
    assert reset['exit_code']==0 and h.bridge and not c.latched
    assert reset['data']['connection']['readiness']=='ready'


def test_reset_does_not_restart_bridge_to_hide_other_fault(rig):
    c,h,_=rig;click(c,'Communications');h.fault='overcurrent_latched';before=h.bridge_starts
    v=click(c,'ResetEmergency')
    assert v['exit_code']==20 and 'overcurrent' in v['reason']
    assert h.bridge_starts==before and h.rails==0


def test_no_motion_without_backend_separates_process_and_hardware(rig):
    c,h,_=rig;v=click(c,'StopMotion')
    assert v['status']=='already_satisfied' and v['data']['no_motion']
    assert v['data']['hardware_output']=='unknown' and not v['data']['verified']
    assert not h.bridge and not c.latched and not v['emergency_latched']


def test_reload_closed_session_does_not_manufacture_new_ack(rig):
    c,h,factory=rig;click(c,'Communications');click(c,'Stop')
    c.shutdown.set();c.worker.join(4)
    c2=Controller(c.root,factory)
    try:
        before=list(h.calls);v=click(c2,'Stop')
        assert v['status']=='already_satisfied' and v['data']['current_hardware_confirmed'] is False
        assert h.calls==before
        assert click(c2,'Communications')['exit_code']==0 and not c2.latched
    finally:c2.shutdown.set();c2.worker.join(4)


def test_foreign_owner_is_not_bypassed_for_recovery(rig):
    c,h,_=rig;h.foreign=True
    v=click(c,'Communications')
    assert v['exit_code']!=0 and not h.bridge and h.rails==0
    assert not any(x[0]=='ensure-on' for x in h.calls if isinstance(x,tuple))


def test_reloaded_emergency_is_reasserted_when_communications_rebuilds(rig):
    c,h,factory=rig;click(c,'Communications');click(c,'EmergencyStop');click(c,'Stop')
    c.shutdown.set();c.worker.join(4)
    c2=Controller(c.root,factory)
    try:
        assert c2.emergency_context and not h.bridge
        before=list(h.calls);v=click(c2,'Communications')
        assert v['exit_code']==20 and h.estop and c2.emergency_context
        assert h.calls==before and not h.bridge
        assert click(c2,'PowerOn')['status']=='rejected'
        assert click(c2,'ResetEmergency')['exit_code']==0 and not c2.emergency_context
    finally:c2.shutdown.set();c2.worker.join(4)


def test_emergency_during_blocked_communications_sends_off_without_waiting(rig):
    c,h,_=rig;click(c,'Communications');click(c,'PowerOn')
    entered=threading.Event();release=threading.Event();original=c.native.runtime.reconnect
    def blocked(*args,**kwargs):
        entered.set();assert release.wait(3)
        return original(*args,**kwargs)
    c.native.runtime.reconnect=blocked
    comm=request('Communications');c.submit(comm);assert entered.wait(1)
    emergency=request('EmergencyStop');assert c.submit(emergency)['status']=='started'
    import time
    until=time.monotonic()+1
    while not h.estop and time.monotonic()<until:time.sleep(.01)
    assert h.estop and h.rails==0  # before SSH/bootstrap worker is released
    release.set()
    assert done(c,comm)['exit_code']==40
    assert done(c,emergency)['exit_code']==0 and c.emergency_context
    assert click(c,'PowerOn')['status']=='rejected'


def test_reload_interrupted_recovery_converges_in_one_communications(rig):
    c,h,factory=rig;c.shutdown.set();c.worker.join(4)
    r=request('Communications')
    with c.db:
        c.db.execute('INSERT INTO requests VALUES (?,?,?)', (r['request_id'],json.dumps(r,sort_keys=True),json.dumps({'status':'started','exit_code':0})))
    before=list(h.calls);c2=Controller(c.root,factory)
    try:
        assert c2.latched and not c2.emergency_context and h.calls==before
        assert c2.submit(r)['status']=='state_unknown'
        assert click(c2,'Communications')['exit_code']==0 and not c2.latched
        assert click(c2,'PowerOn')['exit_code']==0
    finally:c2.shutdown.set();c2.worker.join(4)


def test_bridge_refusal_without_command_does_not_add_panel_stop_gate(rig):
    c,h,_=rig;click(c,'Communications');h.fault='foreign_power_owner'
    v=click(c,'PowerOn')
    assert v['exit_code']==20 and not c.latched
    assert v['reason']=='foreign_power_owner' and '原生操作未完成' in v['operator_message']
    h.fault=None
    assert click(c,'PowerOn')['exit_code']==0


def test_closed_session_does_not_answer_stop_for_different_target(rig):
    c,h,_=rig;click(c,'Communications');click(c,'Stop')
    r=request('Stop');r['sbrio_ip']='192.168.30.253'
    answer=c.submit(r)
    assert answer['status']=='started'  # cannot reuse the other target's lifecycle
    assert done(c,r)['data']['verified_off'] is True


def test_reappeared_native_backend_invalidates_closed_stop_shortcut(rig):
    c,h,_=rig;click(c,'Communications');click(c,'Stop')
    h.bridge=True;h.rails=7  # another native owner restarted a backend
    v=click(c,'Stop')
    assert v['status']=='completed' and v['data']['verified_off'] is True and h.rails==0


@pytest.mark.parametrize('discovers,exits',[(True,True),(False,True),(False,False)])
def test_owned_bridge_discovery_recovery_is_bounded_and_unique(monkeypatch,discovers,exits):
    from rinbo_control import runtime as module
    ticks=[0.];events=[];alive=[True]
    monkeypatch.setattr(module,'time',NS(monotonic=lambda:ticks[0]))
    r=module.Runtime.__new__(module.Runtime);r.cancel=lambda:False;r.progress=events.append
    def stop():events.append('stop_owned_child');alive[0]=not exits
    r.bridge=NS(stop=stop,process=NS(poll=lambda:None if alive[0] else 0))
    feedback=NS(bridge_count=lambda:1 if discovers and ticks[0]>.1 else 0,
                spin=lambda dt:ticks.__setitem__(0,ticks[0]+dt))
    if not discovers and not exits:
        with pytest.raises(RuntimeError,match='不能啟動第二份'):r.recover_owned_bridge(feedback)
        assert r.bridge is not None
    else:
        assert r.recover_owned_bridge(feedback)==int(discovers)
        assert ('stop_owned_child' in events)==(not discovers)
    assert ticks[0]<5.2


@pytest.mark.parametrize('change',['rails','epoch','generation','old_stamp'])
def test_preparation_cannot_succeed_if_state_changes_after_off(rig,change):
    c,h,_=rig;original=c.native.power
    def changed(mode,*args,**kwargs):
        value=original(mode,*args,**kwargs)
        if mode=='ready':
            if change=='rails':value['acknowledgement']['power']=True
            elif change=='epoch':value['epoch']='another-bridge'
            elif change=='generation':value['generation']=99
            else:value['acknowledgement']['feedback_stamp_ns']=0
        return value
    c.native.power=changed
    v=click(c,'Communications')
    assert v['status']=='state_unknown' and c.latched and not c.emergency_context
    assert 'preparation_changed' in v['reason']
    assert click(c,'PowerOn')['status']=='rejected'


def test_real_base64_client_reset_contract_uses_same_native_owner(rig,tmp_path):
    import base64
    import subprocess
    import sys
    from pathlib import Path
    from rinbo_control.panel_server import Server, Handler
    c,h,_=rig;click(c,'Communications');click(c,'EmergencyStop')
    path=tmp_path/'mock-panel.sock';server=Server(str(path),Handler);server.controller=c
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        r=request('ResetEmergency')
        client=Path(__file__).resolve().parents[3]/'tools/rslip_panel_client.py'
        p=subprocess.run([sys.executable,str(client),'--socket',str(path),'--request-base64',
                          base64.b64encode(json.dumps(r).encode()).decode()],capture_output=True,text=True,timeout=5)
        assert p.returncode==0 and json.loads(p.stdout)['request_id']==r['request_id']
        v=done(c,r)
        assert v['data']['emergency_reset_verified'] and not v['emergency_latched']
        assert h.rails==0 and h.bridge
    finally:server.shutdown();server.server_close();thread.join(2)
