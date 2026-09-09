import copy
import json

import pytest

from rinbo_control.console import menu, quick_power
from rinbo_control.demo import DemoRuntime
from rinbo_control.plans import default_plan, preset_plan
from rinbo_control.runtime import Cancelled


def replies(monkeypatch, values):
    values = iter(values)
    monkeypatch.setattr('builtins.input', lambda _: next(values))


def test_workshop_changes_and_new_leg_execute_on_three_only(tmp_path, monkeypatch, capsys):
    runtime = DemoRuntime(tmp_path)
    calls = []
    execute = runtime.execute
    def run(path, site_hash, calibrate):
        calls.append((json.loads(path.read_text()), calibrate))
        execute(path, site_hash, calibrate)
    runtime.execute = run
    replies(monkeypatch, ['14', '1', '3', '13', '2', '3', 'R2 v 5', '3', '0'])
    menu(runtime, {}, tmp_path/'settings.json', demo=True)
    assert [entry[1] for entry in calls] == [True, False, True]
    assert [entry[0]['max_pwm'] for entry in calls] == [80, 70, 70]
    assert list(calls[-1][0]['legs']) == ['R2']
    assert '現場調機｜按 3 直接開始' in capsys.readouterr().out
    assert 'workshop' not in json.loads((tmp_path/'settings.json').read_text())


@pytest.mark.parametrize('stop', ['off', 'failure', 'cancel'])
def test_workshop_stop_or_failure_requires_review_again(tmp_path, monkeypatch, stop):
    runtime = DemoRuntime(tmp_path)
    calls = []
    execute = runtime.execute
    def run(*args):
        calls.append(1)
        if stop == 'failure':
            raise RuntimeError('test fault')
        if stop == 'cancel':
            raise Cancelled('Q')
        return execute(*args)
    runtime.execute = run
    sequence = ['14', '1', '3'] + (['4'] if stop == 'off' else []) + ['3', '', '0']
    replies(monkeypatch, sequence)
    menu(runtime, {}, tmp_path/'settings.json', demo=True)
    assert len(calls) == 1


def test_workshop_settings_alone_never_connect_or_move(tmp_path, monkeypatch):
    runtime = DemoRuntime(tmp_path)
    def forbidden(*args):
        raise AssertionError('settings must not actuate')
    runtime.execute = runtime.connect = runtime.safe_off = forbidden
    replies(monkeypatch, ['14', '1', '13', '2', '9', '2', '3', '0'])
    menu(runtime, {}, tmp_path/'settings.json', demo=True)
    plan = json.loads((tmp_path/'settings.json').read_text())['plan']
    assert plan['max_pwm'] == 70
    assert plan['legs']['L2']['speed_deg_s'] == 5


def test_disabling_workshop_restores_normal_confirmation(tmp_path, monkeypatch):
    runtime = DemoRuntime(tmp_path)
    runtime.execute = lambda *args: pytest.fail('cancelled confirmation executed')
    replies(monkeypatch, ['14', '1', '14', '2', '3', '', '0'])
    menu(runtime, {}, tmp_path/'settings.json', demo=True)


def test_site_change_between_dashboard_and_go_revokes_workshop(tmp_path, monkeypatch):
    runtime = DemoRuntime(tmp_path)
    runtime.execute = lambda *args: pytest.fail('site change skipped review')
    sequence = iter(['14', '1', '3', '', '0'])
    def answer(_):
        value = next(sequence)
        if value == '3':
            runtime.revision += 1
        return value
    monkeypatch.setattr('builtins.input', answer)
    menu(runtime, {}, tmp_path/'settings.json', demo=True)


def test_mask_still_blocks_in_workshop(tmp_path, monkeypatch):
    runtime = DemoRuntime(tmp_path)
    runtime.disabled = ['L2']
    runtime.execute = lambda *args: pytest.fail('masked leg executed')
    replies(monkeypatch, ['14', '1', '3', '0'])
    menu(runtime, {'plan': default_plan()}, tmp_path/'settings.json', demo=True)


@pytest.mark.parametrize('choice,expected', [('1', 45), ('2', 25), ('3', 47), ('4', 55)])
def test_quick_power_uses_motion_site_cap_and_preserves_other_fields(monkeypatch, choice, expected):
    plan = default_plan()
    plan['max_pwm'] = 35
    original = copy.deepcopy(plan)
    site = {'parameters': {'rinbo_standing': {'max_pwm': 55}, 'rinbo_cali': {'max_pwm': 80}}}
    replies(monkeypatch, [choice] + (['47'] if choice == '3' else []))
    updated = quick_power(plan, site)
    assert updated['max_pwm'] == expected
    updated['max_pwm'] = 35
    assert updated == original == plan


def test_quick_power_never_silently_clamps_increment(monkeypatch):
    plan = default_plan()
    plan['max_pwm'] = 75
    replies(monkeypatch, ['1'])
    with pytest.raises(ValueError, match='原設定保留'):
        quick_power(plan, {})
    assert plan['max_pwm'] == 75


def test_presets_retain_tuned_limits_and_duration():
    plan = default_plan()
    plan.update(max_pwm=55, duration_s=12, max_speed_deg_s=2, acceleration_deg_s2=30)
    for choice in ('1', '2', '3', '4'):
        updated = preset_plan(['R2'], choice, current=plan)
        assert updated['max_pwm'] == 55
        assert updated['duration_s'] == 12
        assert updated['acceleration_deg_s2'] == 30
        assert updated['max_speed_deg_s'] == (5 if choice in ('3', '4') else 2)
    assert plan['max_speed_deg_s'] == 2
