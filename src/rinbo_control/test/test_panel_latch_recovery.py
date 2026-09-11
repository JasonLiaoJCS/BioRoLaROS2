"""Durable stop/latch contract, with fake backend only (no ROS or SSH)."""
import json
import sqlite3
import time
import pytest
from rinbo_control.panel_server import Controller
from rinbo_control.panel_native import NativeFailure
from test_panel_api import Fake, request, done


def close(c):
    c.native.block.set(); c.shutdown.set(); c.worker.join(4)


@pytest.fixture
def ctl(tmp_path):
    c=Controller(tmp_path,Fake)
    yield c
    close(c)


def metadata(c):
    reader=sqlite3.connect(c.root/'panel.sqlite3')
    try:return dict(reader.execute('SELECT * FROM metadata'))
    finally:reader.close()


def test_completed_stop_is_visible_only_with_latch_and_target_commit(ctl):
    c=ctl; observed=[]; emit=c.emit
    def inspect(kind,line):
        if 'RSLIP_ACTION_STATUS=' in line:
            value=json.loads(line.split('RSLIP_ACTION_STATUS=',1)[1])
            if value.get('action')=='Stop' and value.get('status')=='completed':
                m=metadata(c)
                observed.append((m['latch'],m.get('target'),m['last_stop_commit']))
        emit(kind,line)
    c.emit=inspect
    r=request('Stop'); c.submit(r); v=done(c,r)
    assert observed and observed[0][:2]==('0',None)
    assert json.loads(observed[0][2])['operation_id']==r['request_id']
    assert v['control']['emergency_latched'] is False
    assert v['control']['recovery_action']=='Communications'


def test_successful_stop_reload_then_communications_does_not_relock(tmp_path):
    c=Controller(tmp_path,Fake);r=request('Stop');c.submit(r);done(c,r);close(c)
    c=Controller(tmp_path,Fake)
    try:
        assert not c.latched and c.native.target is None
        assert c.submit(r)['replayed'] and c.native.events==[]
        comm=request('Communications');c.submit(comm);v=done(c,comm)
        assert not v['emergency_latched'] and v['control']['control_state']=='ready'
        power=request('PowerOn');assert c.submit(power)['status']=='started';done(c,power)
    finally:close(c)


def test_old_completed_stop_does_not_clear_later_failed_stop_or_legacy_latch(tmp_path):
    c=Controller(tmp_path,Fake);r=request('Stop');c.submit(r);done(c,r)
    comm=request('Communications');c.submit(comm);done(c,comm)
    c.native.off_failed=NativeFailure('backend missing; ACK timeout',30)
    later=request('Stop');c.submit(later);done(c,later);close(c)
    c=Controller(tmp_path,Fake)
    try:
        assert c.latched
        assert c.latch_context['operation_id']==later['request_id']
        assert c.latch_context['reason']=='backend missing; ACK timeout'
        assert c.submit(r)['status']=='completed' and c.latched
    finally:close(c)
    with sqlite3.connect(tmp_path/'panel.sqlite3') as db:
        db.execute("DELETE FROM metadata WHERE key='latch_context'")
    c=Controller(tmp_path,Fake)
    try:
        assert c.latched and c.latch_context['source']=='legacy_metadata'
        assert c.latch_context['operation_id'] is None
        assert c.native.events==[]
    finally:close(c)


def test_communications_recovers_general_stop_in_one_click(ctl):
    c=ctl;c.native.off_failed=NativeFailure('ACK timeout',30)
    stop=request('Stop');c.submit(stop);failed=done(c,stop)
    assert failed['stop_blocked'] and not failed['emergency_latched']
    refusal=c.submit(request('PowerOn'))
    assert refusal['control']['recovery_action']=='Communications'
    c.native.off_failed=None
    comm=request('Communications');c.submit(comm);v=done(c,comm)
    assert v['status']=='completed' and v['control']['control_state']=='ready'
    assert not v['stop_blocked'] and not v['emergency_latched']
    power=request('PowerOn');c.submit(power);assert done(c,power)['exit_code']==0


def test_new_stop_after_cleanup_is_new_confirmation_not_historical_success(ctl):
    c=ctl;r=request('Stop');c.submit(r);done(c,r)
    count=c.native.events.count('off')
    assert c.submit(r)['replayed'] and c.native.events.count('off')==count
    c.native.off_failed=NativeFailure('no new ACK',30)
    new=request('Stop');c.submit(new);v=done(c,new)
    assert v['status']=='already_satisfied' and v['control']['recovery_action']=='Communications'
    assert v['data']['verified_off'] is None and not v['data']['current_hardware_confirmed']
    assert not c.latched and c.native.events.count('off')==count
    assert c.submit(r)['status']=='completed'


def test_transaction_failure_cannot_publish_completed_or_retire_ownership(ctl):
    c=ctl
    # Failure at the LAST metadata write must roll back all earlier writes.
    c.db.execute("CREATE TRIGGER fail_commit BEFORE INSERT ON metadata WHEN NEW.key='last_stop_commit' BEGIN SELECT RAISE(ABORT,'simulated disk failure'); END")
    c.db.commit()
    r=request('Stop');c.submit(r);v=done(c,r)
    m=metadata(c)
    assert v['status']=='state_unknown' and v['exit_code']==31 and m['latch']=='1' and 'target' in m
    assert v['data']['state_commit']=='failed'
    assert not any('"status": "completed"' in e['line'] and r['request_id'] in e['line'] for e in c.events)
    assert c.latched


def test_interrupted_stop_reboot_keeps_origin_unknown_no_replay(tmp_path):
    c=Controller(tmp_path,Fake);close(c)
    r=request('Stop')
    with c.db:
        c.db.execute('INSERT INTO requests VALUES (?,?,?)',(r['request_id'],json.dumps(r,sort_keys=True),json.dumps(dict(status='started',exit_code=0))))
    c=Controller(tmp_path,Fake)
    try:
        v=c.submit(r)
        assert v['status']=='state_unknown' and c.latched and c.native.events==[]
        assert v['control']['latch']['operation_id']==r['request_id']
        assert 'controller_restarted' in v['control']['latch']['reason']
    finally:close(c)


def test_completed_stop_never_clears_later_emergency_latch(ctl):
    c=ctl;c.native.block.clear()
    stop=request('Stop');c.submit(stop)
    emergency=request('EmergencyStop');c.submit(emergency)
    c.native.block.set();done(c,stop);done(c,emergency)
    assert c.latched and c.latch_context['operation_id']==emergency['request_id']
    assert metadata(c)['latch']=='1'


def test_admission_persists_stop_origin_before_interrupt(ctl):
    c=ctl;seen=[];interrupt=c.native.interrupt
    def inspect(action):
        m=metadata(c);seen.append((m['latch'],json.loads(m['latch_context'])))
        interrupt(action)
    c.native.interrupt=inspect
    r=request('Stop');c.submit(r);done(c,r)
    assert seen[0][0]=='1' and seen[0][1]['operation_id']==r['request_id']


def test_failed_admission_has_no_interrupt_and_no_half_request(ctl):
    c=ctl
    c.db.execute("CREATE TRIGGER fail_latch BEFORE INSERT ON metadata WHEN NEW.key='latch_context' BEGIN SELECT RAISE(ABORT,'disk failure'); END")
    c.db.commit()
    r=request('Stop')
    with pytest.raises(sqlite3.Error):c.submit(r)
    # Independent readers must see neither uncommitted admission nor its latch.
    reader=sqlite3.connect(c.root/'panel.sqlite3')
    try:
        assert reader.execute('SELECT id FROM requests WHERE id=?',(r['request_id'],)).fetchone() is None
        assert reader.execute("SELECT value FROM metadata WHERE key='latch'").fetchone() is None
    finally:reader.close()
    assert c.native.events==[]
    assert not c.db.in_transaction


def test_coalesced_results_commit_together_before_completion_event(ctl):
    c=ctl;c.native.block.clear();first=request('Stop');second=request('Stop')
    c.submit(first);c.submit(second);emit=c.emit;observed=[]
    def inspect(kind,line):
        if 'RSLIP_ACTION_STATUS=' in line:
            v=json.loads(line.split('RSLIP_ACTION_STATUS=',1)[1])
            if v.get('operation_id')==first['request_id'] and v.get('status')=='completed':
                with sqlite3.connect(c.root/'panel.sqlite3') as reader:
                    alias=json.loads(reader.execute('SELECT response FROM requests WHERE id=?',(second['request_id'],)).fetchone()[0])
                    observed.append(alias)
        emit(kind,line)
    c.emit=inspect;c.native.block.set();done(c,first);done(c,second)
    assert observed[0]['status']=='completed'
    assert observed[0]['data']['fulfilled_by_operation_id']==first['request_id']
    assert metadata(c)['latch']=='0'


def test_readlogs_does_not_depend_on_backend_status(ctl):
    c=ctl
    def broken():raise RuntimeError('backend observer unavailable')
    c.native.status=broken
    value=c.submit(request('ReadLogs'))
    assert value['status']=='completed' and c.native.events==[]
