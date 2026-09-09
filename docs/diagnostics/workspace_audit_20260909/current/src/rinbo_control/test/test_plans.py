import copy
import json
import pytest
from rinbo_control.plans import (atomic_json, default_plan, describe, friendly_error,
                                 ipv4, load_settings, number, quick_plan, selected_legs, validate_plan)


def test_quick_settings_are_explicit_and_preserve_unrelated_limits():
    original = default_plan()
    p = quick_plan('L2 v -25', original, ['L1'])
    assert p['legs'] == {'L2': {'mode': 'velocity', 'speed_deg_s': -25}}
    assert p['max_speed_deg_s'] == 25
    assert p['max_pwm'] == original['max_pwm'] == 80
    assert original == default_plan()
    assert quick_plan('time 8', p)['duration_s'] == 8
    assert quick_plan('pwm 30', p)['max_pwm'] == 30
    assert quick_plan('acc 25', p)['acceleration_deg_s2'] == 25
    assert quick_plan('L2 -5', p)['legs']['L2']['move_deg'] == -5
    assert quick_plan('R2 a 90', p)['legs']['R2']['angle_deg'] == 90
    assert quick_plan('R2 p 180 30', p)['legs']['R2']['phase_deg'] == 180
    for command in ('L1 +5', 'L2 +31', 'L2 v nan', 'L2 v 91', 'pwm 81', 'time 0', 'speed 5'):
        with pytest.raises(ValueError):
            quick_plan(command, p, ['L1'])


def test_default_is_relative_and_never_selects_disabled_leg():
    for bits in range(64):
        names = ['L1','L2','L3','R1','R2','R3']
        disabled = [n for i,n in enumerate(names) if bits & (1<<i)]
        p = default_plan(disabled)
        assert not set(p['legs']).intersection(disabled)
        if bits != 63:
            validate_plan(p, disabled)
            assert next(iter(p['legs'].values())) == {'mode':'relative','move_deg':5.0}
        else:
            with pytest.raises(ValueError):
                validate_plan(p, disabled)


@pytest.mark.parametrize('value', ['nan','inf','-inf',True,91,-1])
def test_number_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        number(value,0,90)


@pytest.mark.parametrize('value', ['127.0.0.1','0.0.0.0','224.1.1.1','255.255.255.255','host; reboot',''])
def test_ip_rejects_non_device_or_shell_text(value):
    with pytest.raises(ValueError):
        ipv4(value)


def test_selection_and_plan_validation():
    assert selected_legs('r2，l2', ['L1']) == ['L2','R2']
    for value in ['L1','L2 l2','L7','']:
        with pytest.raises(ValueError):
            selected_legs(value,['L1'])
    p = default_plan()
    assert '從當前位置轉 +5°' in describe(p)
    for entry in [{'mode':'relative','move_deg':31}, {'mode':'relative','move_deg':'5'},
                  {'mode':'relative','move_deg':True}, {'mode':'relative','angle_deg':5},
                  {'mode':'velocity','speed_deg_s':11}, {'mode':'cycle','speed_deg_s':5}]:
        bad=copy.deepcopy(p); bad['legs']['L2']=entry
        with pytest.raises(ValueError):
            validate_plan(bad)


def test_settings_are_preferences_only_and_atomic(tmp_path):
    path=tmp_path/'prefs.json'
    value=dict(ip='192.168.30.254',jetson_ip='192.168.30.8',port=50051,plan=default_plan())
    atomic_json(path,value)
    assert load_settings(path)==value
    assert (path.stat().st_mode & 0o077)==0
    assert not list(tmp_path.glob('.save-*'))
    value['calibrated']=True
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError):
        load_settings(path)


def test_expected_startup_message_is_not_reported_as_a_connection_success():
    message=friendly_error('MOTOR SAFETY LATCHED: bridge startup requires consecutive fresh all-disabled commands')
    assert '不是連線成功證明' in message


def test_context_shutdown_error_keeps_cause_instead_of_tail_of_parameter_dump():
    log = "CALIBRATION SAFETY STOP: rcl node's context is invalid, at ./src/rcl/node.c:428: error not set\n"
    log += 'Effective parameters: max_pwm: 80\n' * 80
    assert 'ROS 通訊' in friendly_error(log)
    assert 'max_pwm' not in friendly_error(log)


def test_opposite_encoder_feedback_is_not_mistaken_for_insufficient_pwm():
    log = 'CALIBRATION SAFETY STOP: opposite encoder travel: L1 start=0 actual=501\n'
    result = friendly_error(log + 'Effective parameters max_pwm:80\n' * 100)
    assert 'L1' in result and '相反方向' in result
    assert '不能只當成出力太小' in result
    assert '站立保持' in friendly_error('standing hold position lost: R2 actual=50000 target=27648')


@pytest.mark.parametrize('stage', ['final pose not reached', 'alignment not reached'])
def test_stationary_at_pwm_cap_explains_evidence_without_claiming_hardware_cause(stage):
    log = (f'MANUAL SAFETY STOP: {stage}: L2; target_deg=20.00 actual_deg=0.00 '
           'velocity_deg_s=0.00 pwm=20.00 cap=20.00 encoder_span_deg=0.00 peak_pwm=20.00\n')
    result = friendly_error(log + 'Effective parameters: max_pwm: 80\n' * 80)
    assert 'L2 未到位' in result
    assert '目標 20°，實際 0°' in result
    assert '可能是起轉出力不足' in result
    assert '不是實測馬達出力' in result
    assert '選單 7' in result


@pytest.mark.parametrize('span,peak', [(5,20), (0,10), (0.2,20)])
def test_tracking_failure_does_not_mislabel_moving_or_unsaturated_motor(span, peak):
    result = friendly_error(
        'final pose not reached: R2; target_deg=20 actual_deg=0 velocity_deg_s=0 '
        f'pwm=10 cap=20 encoder_span_deg={span} peak_pwm={peak}')
    assert 'R2 未到位' in result
    assert '可能是起轉出力不足' not in result
    assert '編碼器變化範圍' in result
