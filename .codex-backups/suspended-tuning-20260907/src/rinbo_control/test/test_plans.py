import copy
import json
import pytest
from rinbo_control.plans import (atomic_json, default_plan, describe, friendly_error,
                                 ipv4, load_settings, number, selected_legs, validate_plan)


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
