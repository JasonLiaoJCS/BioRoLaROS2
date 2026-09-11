"""Stop lifecycle regressions: no ROS initialization, SSH or robot process."""
import json
from types import SimpleNamespace
import threading
import time
import pytest

from rinbo_control.feedback import Feedback, MotionStopUnknown
from rinbo_control.panel_native import Native, NativeFailure
from rinbo_control.panel_server import Controller
from test_panel_api import Fake, request, done


@pytest.fixture
def controller(tmp_path):
    c=Controller(tmp_path,Fake)
    yield c
    c.native.block.set();c.shutdown.set();c.worker.join(4)


def test_stop_escalates_stopmotion_without_windows_retry(controller):
    c=controller;c.native.block.clear()
    motion=request('StopMotion');c.submit(motion)
    stop=request('Stop');assert c.submit(stop)['status']=='started'
    end=time.monotonic()+1
    while 'off' not in c.native.events and time.monotonic()<end:time.sleep(.01)
    assert 'off' in c.native.events
    c.native.block.set()
    assert done(c,stop)['exit_code']==0
    assert done(c,motion)['status']=='failed' and done(c,motion)['exit_code']==40
    assert not c.latched


def test_repeat_stop_joins_one_off_with_distinct_durable_ids(controller):
    c=controller;c.native.block.clear()
    first,second=request('Stop'),request('Stop')
    c.submit(first);joined=c.submit(second)
    assert joined['coalesced_with']==first['request_id']
    assert joined['operation_id']==second['request_id']
    c.native.block.set()
    final=done(c,second)
    assert final['exit_code']==0 and final['data']['fulfilled_by_operation_id']==first['request_id']
    assert c.native.events.count('off')==1
    assert c.submit(second)['replayed']
    replay=c.submit(request('ReadLogs'))
    assert any(x['operation_id']==second['request_id'] and x['status']=='completed' for x in replay['operations'])


def test_emergency_preempts_stop_and_keeps_latch(controller):
    c=controller;c.native.block.clear()
    stop=request('Stop');c.submit(stop)
    emergency=request('EmergencyStop');assert c.submit(emergency)['status']=='started'
    c.native.block.set()
    assert done(c,stop)['status']=='failed' and done(c,stop)['exit_code']==40
    assert done(c,emergency)['exit_code']==0
    assert c.latched


def test_explicit_stop_after_emergency_completes_cleanup_in_order(controller):
    c=controller;c.native.block.clear()
    emergency=request('EmergencyStop');c.submit(emergency)
    stop=request('Stop');assert c.submit(stop)['status']=='started'
    c.native.block.set()
    assert done(c,emergency)['exit_code']==0
    assert done(c,stop)['exit_code']==0 and c.latched
    reset=request('ResetEmergency'); c.submit(reset)
    assert done(c,reset)['exit_code']==0 and not c.latched
    assert c.native.events.index('EmergencyStop') < c.native.events.index('Stop')


def test_motion_poll_does_not_race_an_off_in_progress(controller):
    c=controller;release=threading.Event();entered=threading.Event();polled=[]
    def off(*a,**kw):entered.set();release.wait(2)
    c.native.urgent_off=off
    stop=request('Stop');c.submit(stop)
    assert entered.wait(1)
    c.native.poll=lambda:polled.append(True)
    time.sleep(.12)
    assert polled==[]
    release.set();assert done(c,stop)['exit_code']==0


def test_missing_backend_preserves_raw_ack_timeout_and_motion_result(controller):
    c=controller
    raw={'operation':'off','command_sent':True,'readiness':'backend_unavailable',
         'epoch':None,'generation':None,'acknowledgement':None,'bridge_rejection':{'reason':None}}
    c.native.off_failed=NativeFailure('ACK timeout: relay state UNKNOWN',30,raw)
    req=request('Stop');c.submit(req);value=done(c,req)
    assert value['status']=='state_unknown' and value['exit_code']==value['native_exit_code']==30
    assert all(value['data'][k]==v for k,v in raw.items())
    assert value['data']['verified_off'] is None
    assert value['data']['native_owned_services_cleanup']=='not_attempted'
    assert c.latched and 'Stop' not in c.native.events
    assert c.submit(req)['reason']==value['reason']


def test_cleanup_failure_keeps_this_off_ack(controller):
    c=controller
    power={'request_id':'native','epoch':'epoch','generation':3,'acknowledgement':{'power':False}}
    c.native.urgent_off=lambda *a,**kw:power
    c.native.failed=NativeFailure('remote cleanup failed',49,{'service':'core'})
    req=request('Stop');c.submit(req);value=done(c,req)
    assert value['native_exit_code']==49
    assert value['data']['power']==power and value['data']['service']=='core'


def feedback_without_backend(monkeypatch):
    from rinbo_control import feedback as module
    tick=[0.]
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:tick[0]))
    f=Feedback.__new__(Feedback);f.motor_output=(False,-1.)
    f.bridge_count=lambda:0;f.writers=lambda:[]
    f.spin=lambda seconds:tick.__setitem__(0,tick[0]+seconds)
    return f,tick


def test_absent_backend_and_stale_disabled_are_not_completed(monkeypatch):
    f,tick=feedback_without_backend(monkeypatch)
    with pytest.raises(MotionStopUnknown) as error:f.wait_motor_stopped()
    assert error.value.code==31 and '不開 Relay' not in str(error.value)
    assert error.value.data['verified'] is False
    assert error.value.data['bridge_count']==0 and error.value.data['power_action']=='none'


def test_current_disabled_after_entry_is_required(monkeypatch):
    f,tick=feedback_without_backend(monkeypatch)
    def spin(seconds):
        tick[0]+=seconds;f.motor_output=(False,tick[0])
    f.spin=spin;f.bridge_count=lambda:1
    assert f.wait_motor_stopped()['verified']


def test_new_stop_cancels_only_the_wait_not_the_protection(monkeypatch):
    f,tick=feedback_without_backend(monkeypatch)
    with pytest.raises(MotionStopUnknown) as error:f.wait_motor_stopped(lambda:True)
    assert error.value.code==40


def ack(rid,epoch='current'):
    return dict(request_id=rid,operation='off',exit_code=0,epoch=epoch,generation=2,
                acknowledgement=dict(request_id=rid,epoch=epoch,generation=2,
                    digital=False,signal=False,power=False,ack_kind='command_correlated',estop_asserted=True))


@pytest.mark.parametrize('fault',['missing','old_id','epoch','generation','partial','estop'])
def test_off_requires_this_operation_and_epoch(tmp_path,fault):
    n=Native(tmp_path,lambda *a:None);value=ack('new')
    n.runtime.power_touched=True
    if fault=='missing':value['acknowledgement']=None
    elif fault=='old_id':value['acknowledgement']['request_id']='old'
    elif fault=='epoch':value['acknowledgement']['epoch']='previous-boot'
    elif fault=='generation':value['acknowledgement']['generation']=1
    elif fault=='partial':value['acknowledgement']['signal']=True
    else:value['acknowledgement']['estop_asserted']=False
    n.power=lambda *a,**kw:value
    with pytest.raises(NativeFailure) as error:n.urgent_off('new',emergency=True)
    assert error.value.code==31 and n.off_results=={} and n.runtime.power_touched


def test_old_completed_stop_does_not_authorize_new_cleanup(tmp_path,monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','99')
    n=Native(tmp_path,lambda *a:None)
    n.last_completed_stop={'operation_id':'old','epoch':'old','historical_only':True}
    n.stop_motion=lambda **kw:pytest.fail('cleanup must not start without this Off ACK')
    with pytest.raises(NativeFailure) as error:n.execute(request('Stop'))
    assert error.value.code==31 and error.value.data['verified_off'] is None


def test_lost_enabling_reply_retains_power_cleanup_obligation(tmp_path):
    n=Native(tmp_path,lambda *a:None)
    payload={'request_id':'lost','operation':'ensure-on','exit_code':30,'reason':'ACK timeout'}
    line=json.dumps(payload)
    child=SimpleNamespace(process=SimpleNamespace(poll=lambda:30),
                          reader=SimpleNamespace(join=lambda *a:None),tail=line,
                          drain=lambda:[line],stop=lambda:None)
    n.runtime._start=lambda *a:child
    with pytest.raises(NativeFailure) as error:n.power('ensure-on','lost')
    assert error.value.code==30 and n.runtime.power_touched


def test_verified_off_precedes_cleanup_and_recorder_is_not_targeted(tmp_path,monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','99')
    from rinbo_control import panel_native as module
    n=Native(tmp_path,lambda *a:None);req=request('Stop');events=[]
    n.power=lambda *a,**kw:(events.append('off_ack') or ack(req['request_id']))
    n.urgent_off(req['request_id'])
    n.stop_motion=lambda **kw:(events.append('motion_verified') or {'verified':True})
    n.runtime.executable=lambda package,name:name
    monkeypatch.setattr(module,'find_motion',lambda names,*a:(events.append(tuple(names)) or []))
    monkeypatch.setattr(module,'stop_motion',lambda *a:events.append('bridge_stop'))
    n.runtime.remote=SimpleNamespace(stop=lambda:events.append('remote_stop'),close=lambda:events.append('ssh_close'))
    n.runtime.feedback=SimpleNamespace(close=lambda:events.append('feedback_close'))
    n.runtime.close=lambda:True
    value=n.execute(req)
    assert value['verified_off'] and value['native_owned_services_cleanup']=='completed'
    assert events[:2]==['off_ack','motion_verified']
    assert ('rinbo_ros_bridge',) in events
    assert all('recorder' not in str(x) and 'mirror' not in str(x) for x in events)
    assert n.runtime.feedback is None and n.last_completed_stop['historical_only']


def test_off_publishers_are_serialized_across_clients(tmp_path):
    n=Native(tmp_path,lambda *a:None);count=[0,0];lock=threading.Lock()
    def power(mode,rid,**kwargs):
        with lock:count[0]+=1;count[1]=max(count)
        time.sleep(.03)
        with lock:count[0]-=1
        return ack(rid)
    n.power=power
    threads=[threading.Thread(target=n.urgent_off,args=(str(i),)) for i in range(4)]
    for t in threads:t.start()
    for t in threads:t.join()
    assert count[1]==1 and len(n.off_results)==4


def test_recreated_feedback_resubscribes_and_ignores_old_node(tmp_path,monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules,'std_msgs.msg',SimpleNamespace(String=object))
    class Node:
        def create_subscription(self,kind,topic,cb,qos):self.callback=cb;return object()
        def get_publishers_info_by_topic(self,topic):return [SimpleNamespace(node_name='rinbo_ros2_bridge',node_namespace='/')]
        def get_clock(self):return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=123000000000))
    n=Native(tmp_path,lambda *a:None);old=Node();new=Node();probe=SimpleNamespace(node=old)
    n.runtime.probe=lambda:probe
    n.watch_power()
    msg=lambda epoch,seq:SimpleNamespace(data=json.dumps(dict(protocol=2,epoch=epoch,status_sequence=seq,stamp_ns=123000000000,readiness='ready')))
    old.callback(msg('old',100));assert n.power_observation[1]['epoch']=='old'
    probe.node=new;n.watch_power();assert n.power_observation is None
    old.callback(msg('old',101));assert n.power_observation is None
    new.callback(msg('new',1));assert n.power_observation[1]['epoch']=='new'


def test_reboot_and_readlogs_keep_failure_evidence_without_replay(tmp_path):
    c=Controller(tmp_path,Fake);req=request('Stop')
    c.native.off_failed=NativeFailure('ACK timeout',30,{'epoch':None,'acknowledgement':None})
    c.submit(req);value=done(c,req)
    c.shutdown.set();c.worker.join(2)
    again=Controller(tmp_path,Fake)
    try:
        assert again.submit(req)['data']==value['data']
        assert again.native.events==[] and again.latched
        assert any('ACK timeout' in x['line'] for x in again.submit(request('ReadLogs'))['events'])
    finally:again.shutdown.set();again.worker.join(2)
