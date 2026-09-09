"""Parameter tuning must not become a fixed-preset connection gate."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from rinbo_control.motion_limits import FIELDS, NODES, current_values
from rinbo_control.plans import default_plan
from rinbo_control.presentation import settings_summary
from rinbo_control.runtime import Runtime, calibration_wait_seconds

ROOT = Path(__file__).resolve().parents[3]


def standard():
    return json.loads((ROOT/'config/parameter_baselines/standard_20260909_r15/effective.json').read_text())


def test_explicit_restore_matches_frozen_approved_values():
    site = standard()
    for stage, fields in FIELDS.items():
        assert {key: field.default for key, field in fields.items()} == current_values(site, stage)


@pytest.mark.parametrize('cap,kp,timeout', [(55,.12,30), (260,.42,120), (500,.35,600)])
def test_native_tuning_is_shown_without_baseline_equality_or_plan_rewrite(cap,kp,timeout):
    site = standard()
    site['revision'] += 1
    site['parameters']['rinbo_manual'].update(max_pwm=cap,kp=kp)
    site['parameters']['rinbo_cali']['safety.hall_search_timeout_s'] = timeout
    plan = default_plan()
    before = copy.deepcopy(plan)
    text = settings_summary(site, plan)
    assert f'Manual：KP={kp:g}' in text
    assert f'現場 Manual {cap:g}' in text
    assert f'逐腳本次有效 PWM 上限：{min(80,cap):g}' in text
    assert 'Tripod：KP=0.38' in text and 'PWM=3300' in text
    assert calibration_wait_seconds(site) == 15+60+timeout+2*15
    assert plan == before


@pytest.mark.parametrize('pwm', [80,500,3300])
def test_mock_connection_reuses_bridge_after_pwm_tuning(tmp_path,monkeypatch,pwm):
    monkeypatch.setattr('rinbo_control.runtime.route_source',lambda _: '192.168.30.8')
    monkeypatch.setattr('rinbo_control.runtime.tcp_probe',lambda *a: dict(status='open',error=''))
    runtime = Runtime(tmp_path,progress=lambda _:None)
    calls = []
    runtime.remote = SimpleNamespace(start=lambda *a:calls.append('remote-check'))
    runtime.feedback = SimpleNamespace(
        spin=lambda _:None, bridge_count=lambda:1, bridge_ip=lambda:'192.168.30.254',
        motor_command_max_pwm=pwm,
        status=lambda:dict(bridge_count=1,motor_fresh=True,power_fresh=True))
    def forbidden(*a,**kw):
        pytest.fail('must not spawn, change power, or compare settings with a fixed preset')
    runtime._start = runtime.power = runtime.site = forbidden
    assert runtime.connect('192.168.30.254')['motor_fresh']
    assert calls == ['remote-check'] and not runtime.power_touched
