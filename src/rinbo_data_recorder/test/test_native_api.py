"""Production recorder, isolated ROS transport only. Never sends hardware commands."""
import csv
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import pytest

BIN=Path(os.environ.get('RINBO_RECORDER_TEST_BINARY','/tmp/rinbo-recorder-v2-build/rinbo_data_recorder'))
pytestmark=pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID')!='232' or os.environ.get('ROS_LOCALHOST_ONLY')!='1',reason='requires isolated localhost domain232')


class Fixture:
    def __init__(self,path,output=None,limited=False):
        import rclpy
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from rinbo_msgs.msg import MotorCmdStamped,MotorStateStamped,PowerStateStamped,ControllerDebugStamped,SafetyEventStamped
        from std_msgs.msg import String
        self.path=path;self.ros=rclpy;rclpy.init()
        self.node=rclpy.create_node('recorder_isolated_fixture',enable_rosout=False)
        self.service_type=SetParametersAtomically
        self.client=self.node.create_client(SetParametersAtomically,'/rinbo/recorder/control')
        qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.requested=self.node.create_publisher(MotorCmdStamped,'/rinbo/monitor/motor_requested',qos)
        self.forwarded=self.node.create_publisher(MotorCmdStamped,'/rinbo/monitor/motor_forwarded',qos)
        self.motor=self.node.create_publisher(MotorStateStamped,'/motor/state',10)
        self.power=self.node.create_publisher(PowerStateStamped,'/power/state',10)
        self.debug=self.node.create_publisher(ControllerDebugStamped,'/rinbo/controller_debug',10)
        self.safety=self.node.create_publisher(SafetyEventStamped,'/rinbo/safety_event',10)
        self.detail=self.node.create_publisher(String,'/rinbo/safety_detail',10)
        self.seq=0
        self.output=output or path/'data'
        self.log=(path/'recorder.log').open('w')
        def limit():
            import resource
            resource.setrlimit(resource.RLIMIT_FSIZE,(20000,20000));signal.signal(signal.SIGXFSZ,signal.SIG_IGN)
        self.command=[str(BIN),'--ros-args','-p','auto_start:=false','-p',f'output_root:={self.output}',
                      '-p',f'lock_file:={path}/recorder.lock']
        self.p=subprocess.Popen(self.command,stdout=self.log,stderr=subprocess.STDOUT,preexec_fn=limit if limited else None)
        assert self.client.wait_for_service(timeout_sec=6)
    def call(self,action,name=None):
        from rcl_interfaces.msg import Parameter,ParameterValue
        values=dict(action=action,request_id=uuid.uuid4().hex)
        if name is not None:values['name']=name
        request=self.service_type.Request(parameters=[Parameter(name=k,value=ParameterValue(type=4,string_value=v)) for k,v in values.items()])
        f=self.client.call_async(request);self.ros.spin_until_future_complete(self.node,f,timeout_sec=4)
        assert f.done(),self.log.name
        data=json.loads(f.result().result.reason)
        assert f.result().result.successful==(data['exit_code']==0)
        return data
    def sample(self):
        from rinbo_msgs.msg import MotorCmdStamped,MotorStateStamped,PowerStateStamped,ControllerDebugStamped
        self.seq+=1
        stamp=self.node.get_clock().now().to_msg()
        for pub,value in ((self.requested,7.25),(self.forwarded,0.0)):
            m=MotorCmdStamped();m.header.stamp=stamp;m.header.seq=self.seq;m.l1.voltage=value;pub.publish(m)
        p=PowerStateStamped();p.header.stamp=stamp;p.header.seq=self.seq;p.i_2=6.25;p.v_7=24.;self.power.publish(p)
        m=MotorStateStamped();m.header.stamp=stamp;m.header.seq=self.seq;m.l1.position=55.;self.motor.publish(m)
        d=ControllerDebugStamped();d.header.stamp=stamp;d.header.seq=self.seq;d.controller_state='RUNNING';d.tau=.4;self.debug.publish(d)
        self.ros.spin_once(self.node,timeout_sec=.03)
    def feed(self,seconds=.4):
        end=time.monotonic()+seconds
        while time.monotonic()<end:self.sample()
    def close(self):
        self.p.send_signal(signal.SIGINT);self.p.wait(6);self.log.close()
        self.node.destroy_node();self.ros.shutdown()


@pytest.fixture
def recorder(tmp_path):
    f=Fixture(tmp_path)
    try:yield f
    finally:f.close()


def test_atomic_name_repeated_start_stop_qos_and_distinct_streams(recorder):
    r=recorder
    initial=r.call('status')
    assert not initial['recording'] and initial['topics']['power']['state']=='unavailable'
    r.feed()
    started=r.call('start','my_test')
    assert started['recording'] and started['name']=='my_test'
    same=r.call('start','different_name')
    assert same['run_dir']==started['run_dir'] and same['name']=='my_test'
    r.feed()
    status=r.call('status');assert status['rows']['power_samples']>0 and status['rows']['commands']>0
    for topic in ('/motor/command','/power/command'):
        assert not r.node.get_subscriptions_info_by_topic(topic)
        assert not r.node.get_publishers_info_by_topic(topic)
    r.call('stop');assert r.call('stop')['files_closed']
    directory=Path(started['run_dir'])
    rows=list(csv.DictReader((directory/'summary.csv').open()))
    assert any(x['requested_cmd_voltage_l1']=='7.250000' and x['forwarded_cmd_voltage_l1']=='0.000000' for x in rows)
    commands=list(csv.DictReader((directory/'commands.csv').open()))
    assert {x['source'] for x in commands}=={'requested','forwarded'}
    again=r.call('start','my_test');assert again['run_dir']!=started['run_dir']
    assert (directory/'summary.csv').stat().st_size>0


def test_safety_event_keeps_peak_seq_channel_and_native_detail(recorder):
    from std_msgs.msg import String
    from rinbo_msgs.msg import SafetyEventStamped
    r=recorder;r.call('start','peak');r.feed()
    detail=dict(schema_version=1,source='rinbo_tripod_rslip',event_seq=44,event_stamp_ns=1234000,
                power_seq=999,power_stamp_ns=1233999,quantity='leg_current',channel=2,measured=6.25,
                threshold=5,controller_state='RUNNING',tau=.4,ratio=5.9,reason='leg current limit max=6.25A threshold=5A')
    event=SafetyEventStamped();event.header.seq=44;event.header.stamp=r.node.get_clock().now().to_msg()
    event.source=detail['source'];event.reason=detail['reason'];event.max_current=6.25
    # Direct event callbacks, not summary timer sampling, carry the trigger.
    for _ in range(3):r.detail.publish(String(data=json.dumps(detail)));r.safety.publish(event);r.ros.spin_once(r.node,timeout_sec=.05)
    stopped=r.call('stop');rows=list(csv.DictReader((Path(stopped['run_dir'])/'events.csv').open()))
    raw=[json.loads(x['detail_json']) for x in rows if x['event_kind']=='safety_detail']
    assert any(x['power_seq']==999 and x['channel']==2 and x['threshold']==5 and x['measured']==6.25 for x in raw)
    assert any(x['event_seq']=='44' and x['event_kind']=='safety_event' for x in rows)


def test_stale_data_does_not_stop_and_reconnect_resumes(recorder):
    r=recorder;r.call('start','stale');r.feed()
    time.sleep(1.15)
    stale=r.call('status');assert stale['recording'] and stale['topics']['power']['state']=='stale'
    assert stale['topics']['safety']['state']=='unavailable'
    r.feed();assert r.call('status')['topics']['power']['state']=='received'


def test_restart_does_not_overwrite_history(recorder):
    r=recorder;start=r.call('start','history');r.feed();r.call('stop')
    path=Path(start['run_dir'])/'summary.csv';before=path.read_bytes()
    r.p.send_signal(signal.SIGINT);r.p.wait(5)
    r.p=subprocess.Popen(r.command,stdout=r.log,stderr=subprocess.STDOUT)
    time.sleep(.5);assert r.client.wait_for_service(timeout_sec=5)
    assert not r.call('status')['recording']
    new=r.call('start','history');assert new['run_dir']!=start['run_dir'] and path.read_bytes()==before


def test_disk_error_at_start_is_reported_and_node_survives(tmp_path):
    output=tmp_path/'not_a_directory';output.write_text('keep')
    r=Fixture(tmp_path,output)
    try:
        failed=r.call('start','test');assert failed['exit_code']==50 and not failed['recording']
        assert r.call('status')['disk_error']
        assert r.call('stop')['files_closed']
        assert output.read_text()=='keep'
    finally:r.close()


def test_disk_failure_while_writing_not_false_stop_success(tmp_path):
    r=Fixture(tmp_path,limited=True)
    try:
        r.call('start','full')
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            r.feed(.2);status=r.call('status')
            if status['disk_error']:break
        assert status['disk_error'] and not status['recording']
        assert r.call('stop')['exit_code']==50
    finally:r.close()


def test_noninteractive_client_export_does_not_stop(recorder):
    r=recorder;r.call('start','download');r.feed()
    client=Path(__file__).resolve().parents[3]/'tools/rinbo_recorder_client.py'
    result=subprocess.run([sys.executable,str(client),'export','--domain','232','--no-start','--json'],
        text=True,capture_output=True,timeout=20)
    assert result.returncode==0,result.stderr
    data=json.loads(result.stdout);assert data['recording'] and Path(data['export']['path']).is_file()
    assert r.call('status')['recording']


def test_real_power_guard_trigger_to_events_csv(recorder):
    r=recorder;r.call('start','native_protection')
    fixture=Path('/tmp/rinbo-fsm-recorder-v2-build/recorder_safety_fixture')
    assert fixture.exists(),'build real safety fixture first'
    process=subprocess.run([str(fixture)],text=True,capture_output=True,timeout=8)
    assert process.returncode==0,process.stderr
    result=r.call('stop')
    events=list(csv.DictReader((Path(result['run_dir'])/'events.csv').open()))
    detail=[json.loads(row['detail_json']) for row in events if row['event_kind']=='safety_detail']
    assert any(d['quantity']=='leg_current' and d['channel']==2 and d['power_seq']==129
               and d['measured']==6.25 and d['threshold']==5 and d['current_trip_samples']==25
               and d['disabled_legs'][2] for d in detail)
    assert not r.node.get_publishers_info_by_topic('/motor/command')
    assert not r.node.get_publishers_info_by_topic('/power/command')
