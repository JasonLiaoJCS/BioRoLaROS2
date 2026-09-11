"""Real DDS + production Bridge callbacks; test-only Core-free executable."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

BIN=Path('/tmp/rinbo-power-handoff-build/power_operation_mock_bridge')
pytestmark=pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID')!='232' or
    os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not BIN.exists(),reason='isolated compiled mock required')
FIXTURE=Path(__file__).with_name('power_cli_fixture.py')


def cli(mode,*options,expect=0):
    r=subprocess.run([sys.executable,str(FIXTURE),mode,'--json','--wait-for-subscriber-s','4',
                      '--verify-timeout-s','1',*options],text=True,capture_output=True,timeout=35)
    try:result=json.loads(r.stdout.strip().splitlines()[-1])
    except Exception:pytest.fail(f'No JSON: {r.returncode} {r.stdout} {r.stderr}')
    if os.environ.get('RINBO_POWER_TEST_RESULTS'):
        with Path(os.environ['RINBO_POWER_TEST_RESULTS']).open('a') as stream:
            stream.write(json.dumps(dict(mode=mode,options=options,exit_code=r.returncode,result=result))+'\n')
    assert r.returncode==expect,(result,r.stderr)
    return result


class Mock:
    def __init__(self,path):
        self.log=path.open('w+');self.p=subprocess.Popen([str(BIN)],stdout=self.log,stderr=subprocess.STDOUT)
        self.node=None
        import rclpy
        from std_msgs.msg import String
        rclpy.init();self.ros=rclpy;self.node=rclpy.create_node('power_test_observer')
        self.status=None
        self.sub=self.node.create_subscription(String,'/rinbo/power/operation_status',lambda m:setattr(self,'status',json.loads(m.data)),10)
        self.wait(lambda:self.status is not None)
    def wait(self,predicate,seconds=6):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.ros.spin_once(self.node,timeout_sec=.02)
            if predicate():return
        raise AssertionError('mock condition timeout')
    def refresh(self):
        seq=self.status['status_sequence'];self.wait(lambda:self.status['status_sequence']>seq)
        return self.status
    def param(self,name,value):
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        c=self.node.create_client(SetParameters,'/rinbo_ros2_bridge/set_parameters')
        assert c.wait_for_service(timeout_sec=3)
        f=c.call_async(SetParameters.Request(parameters=[Parameter(name,value=value).to_parameter_msg()]))
        self.wait(f.done);assert f.result().results[0].successful
        self.node.destroy_client(c);self.refresh()
    def restart(self):
        epoch=self.refresh()['epoch']
        self.p.terminate();self.p.wait(timeout=5)
        self.p=subprocess.Popen([str(BIN)],stdout=self.log,stderr=subprocess.STDOUT)
        self.wait(lambda:self.status['epoch']!=epoch)
    def close(self):
        self.node.destroy_node();self.ros.shutdown()
        self.p.terminate();self.p.wait(timeout=5);self.log.close()
        time.sleep(.4)

@pytest.fixture
def mock(tmp_path):
    m=Mock(tmp_path/'bridge.log')
    try:yield m
    finally:m.close()


def test_sequence_handoff_repeat_and_lost_reply(mock):
    assert cli('off')['status']=='success'
    sensors=cli('sequence');assert sensors['acknowledgement']['power'] is False
    relay=cli('relay','--confirm-relay','--request-id','lost-reply')
    assert relay['status']=='success'
    assert sensors['acknowledgement']['publisher_gid']!=relay['acknowledgement']['publisher_gid']
    sequence=mock.refresh()['accepted_sequence'];request=mock.status['accepted_request_id']
    for mode,options in [('sequence',()),('ensure-on',('--confirm-relay','--request-id','lost-reply'))]:
        result=cli(mode,*options)
        assert result['status']=='already_satisfied' and result['command_sent'] is False
        assert mock.refresh()['accepted_request_id']==request
        assert mock.status['accepted_sequence']==sequence and mock.status['feedback_mask']==7
    assert cli('off')['acknowledgement']['power'] is False


def test_single_operation_and_partial_sequence(mock):
    first=cli('sequence');assert first['acknowledgement']['signal']
    # Original sequence process is gone; no relay command was sent.
    assert cli('ensure-on','--confirm-relay')['status']=='success'
    assert mock.refresh()['feedback_mask']==7
    cli('off')
    assert cli('ensure-on','--confirm-relay')['status']=='success'


def test_backend_and_fault_preflight_send_nothing(mock):
    mock.param('feedback',False);time.sleep(.5)
    result=cli('ensure-on','--confirm-relay',expect=10)
    assert not result['command_sent']
    mock.param('feedback',True);mock.param('fault',True)
    result=cli('ensure-on','--confirm-relay',expect=20)
    assert not result['command_sent'] and 'estop' in result['reason']
    assert cli('off')['status']=='success'
    assert cli('ensure-on','--confirm-relay',expect=20)['status']=='rejected'


def test_partial_ack_timeout_cleanup_and_unknown_off(mock):
    mock.param('ack',False)
    result=cli('ensure-on','--confirm-relay',expect=30)
    assert result['status']=='ack_timeout' and 'cleanup Off acknowledged' in result['reason']
    mock.param('feedback',False)
    result=cli('off',expect=30)
    assert result['command_sent'] and result['acknowledgement'] is None


def test_extra_publisher_cannot_enable_but_off_can_ack(mock):
    from rinbo_msgs.msg import PowerCmdStamped
    rogue=mock.node.create_publisher(PowerCmdStamped,'/power/command',10)
    result=cli('ensure-on','--confirm-relay',expect=20)
    assert not result['command_sent']
    assert cli('off')['status']=='success'
    mock.node.destroy_publisher(rogue)


def test_off_preempts_between_stages(mock):
    p=subprocess.Popen([sys.executable,str(FIXTURE),'ensure-on','--confirm-relay','--json',
        '--wait-for-subscriber-s','4','--step-delay-s','3'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        mock.wait(lambda:mock.status['feedback_mask']==1,8)
        assert cli('off')['status']=='success'
        out,err=p.communicate(timeout=15)
        result=json.loads(out.strip().splitlines()[-1])
        assert p.returncode!=0 and 'off_preempted' in result['reason'],(out,err)
        assert mock.refresh()['feedback_mask']==0
    finally:
        if p.poll() is None:p.kill();p.wait()


def test_unsafe_current_blocks_already_satisfied_without_power_toggle(mock):
    cli('ensure-on','--confirm-relay');request=mock.refresh()['accepted_request_id']
    mock.param('current',4.0)
    result=cli('ensure-on','--confirm-relay',expect=20)
    assert not result['command_sent'] and 'power_guard' in result['reason']
    assert mock.refresh()['accepted_request_id']==request
    cli('off')


def test_bridge_restart_fences_inflight_operation_and_fresh_off_still_works(mock):
    p=subprocess.Popen([sys.executable,str(FIXTURE),'ensure-on','--confirm-relay','--json',
        '--wait-for-subscriber-s','4','--step-delay-s','3'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        mock.wait(lambda:mock.status['feedback_mask']==1,8)
        mock.restart()
        out,err=p.communicate(timeout=20)
        result=json.loads(out.strip().splitlines()[-1])
        assert p.returncode==31 and ('source_changed' in result['reason'] or 'epoch_changed' in result['reason']),(out,err)
        assert cli('off')['status']=='success'
        assert mock.refresh()['feedback_mask']==0
    finally:
        if p.poll() is None:p.kill();p.wait()


def test_nonadjacent_old_backend_ack_cannot_authorize_retry(mock):
    cli('ensure-on','--confirm-relay')
    mock.param('replay',True);time.sleep(.5)
    result=cli('ensure-on','--confirm-relay',expect=10)
    assert not result['command_sent']
    assert cli('off',expect=30)['status']=='ack_timeout'
    mock.param('replay',False)
    assert cli('off')['status']=='success'


def test_bridge_rejection_has_request_and_source_and_off_bypasses_rogue(mock):
    from rinbo_msgs.msg import PowerCmdStamped
    status=mock.refresh()
    rogue=mock.node.create_publisher(PowerCmdStamped,'/power/command',10)
    mock.wait(lambda:rogue.get_subscription_count()>0)
    msg=PowerCmdStamped();msg.header.seq=1;msg.header.stamp=mock.node.get_clock().now().to_msg()
    msg.header.frame_id=f"P2|{status['epoch']}|{status['generation']}|rogue-request"
    msg.digital=True;rogue.publish(msg)
    mock.wait(lambda:mock.status['rejected_request_id']=='rogue-request')
    assert 'exact node' in mock.status['rejection_reason']
    assert mock.status['rejected_gid'] and mock.status['rejected_sequence']==1
    result=cli('ensure-on','--confirm-relay',expect=20)
    assert 'exact node' in result['reason']
    assert cli('off')['status']=='success'
    mock.node.destroy_publisher(rogue)


def test_native_manual_finish_releases_relay_without_sensor_power_cycle(mock):
    cli('ensure-on', '--confirm-relay')
    epoch = mock.refresh()['epoch']
    release = cli('sensors')
    assert release['acknowledgement']['digital'] is True
    assert release['acknowledgement']['signal'] is True
    assert release['acknowledgement']['power'] is False
    assert release['epoch'] == epoch
    assert cli('ensure-on', '--confirm-relay')['status'] == 'success'


def test_emergency_latches_bridge_against_later_different_cli(mock):
    cli('ensure-on', '--confirm-relay')
    emergency = cli('off', '--assert-estop')
    assert emergency['acknowledgement']['estop_asserted'] is True
    assert mock.refresh()['reason'] == 'software_estop_latched'
    refused = cli('ensure-on', '--confirm-relay', expect=20)
    assert not refused['command_sent']
    assert 'software_estop_latched' in refused['reason']
    cli('off')  # verified ordinary Off still works; it does not erase E-stop
    assert mock.refresh()['reason'] == 'software_estop_latched'
