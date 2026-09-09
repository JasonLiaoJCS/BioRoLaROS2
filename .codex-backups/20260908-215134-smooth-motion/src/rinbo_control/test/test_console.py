import builtins
from pathlib import Path
import socket
import subprocess
import pytest
from rinbo_control.console import edit_plan, menu, numeric
from rinbo_control.demo import DemoRuntime
from rinbo_control.plans import default_plan


def answers(monkeypatch, text):
    items=iter(text)
    monkeypatch.setattr(builtins,'input',lambda _:next(items))


def test_demo_full_menu_has_no_ros_network_or_process_calls(tmp_path,monkeypatch,capsys):
    def forbidden(*a,**k): raise AssertionError('demo touched an external process/network')
    monkeypatch.setattr(subprocess,'Popen',forbidden)
    monkeypatch.setattr(subprocess,'run',forbidden)
    monkeypatch.setattr(socket,'create_connection',forbidden)
    answers(monkeypatch,['3','開始','5','4','0'])
    runtime=DemoRuntime(tmp_path)
    menu(runtime,{},tmp_path/'settings.json',demo=True)
    text=capsys.readouterr().out
    assert '介面演練' in text and '模擬執行所選腳的動作' in text
    assert '192.168.30.254' in text and '192.168.30.8' in text
    assert not list(tmp_path.rglob('*.calibration.json'))


def test_cancelled_review_never_executes(tmp_path,monkeypatch):
    runtime=DemoRuntime(tmp_path)
    def forbidden(*a): raise AssertionError('unapproved execution')
    monkeypatch.setattr(runtime,'execute',forbidden)
    answers(monkeypatch,['3','','0'])
    menu(runtime,{},tmp_path/'settings.json',demo=True)


def test_successful_unchanged_action_repeats_without_second_confirmation(tmp_path,monkeypatch):
    runtime=DemoRuntime(tmp_path)
    calls=[]
    execute=runtime.execute
    def run(path,site_hash,calibrate):
        calls.append(calibrate)
        execute(path,site_hash,calibrate)
    runtime.execute=run
    answers(monkeypatch,['L2 +5','3','y','3','0'])
    menu(runtime,{},tmp_path/'settings.json',demo=True)
    assert calls==[True,False]


def test_changing_selected_leg_requires_new_calibration_and_review(tmp_path,monkeypatch):
    runtime=DemoRuntime(tmp_path)
    calls=[]
    execute=runtime.execute
    def run(path,site_hash,calibrate):
        calls.append(calibrate)
        execute(path,site_hash,calibrate)
    runtime.execute=run
    answers(monkeypatch,['L2 +5','3','y','R2 v 10','3','','3','y','0'])
    menu(runtime,{},tmp_path/'settings.json',demo=True)
    assert calls==[True,True]
    assert runtime.calibrated_legs=={'R2'}


@pytest.mark.parametrize('change', ['L2 -5', 'L2 v 10', 'L2 a 90', 'L2 p 180 10', 'time 8', 'pwm 30'])
def test_same_leg_settings_changes_reuse_valid_calibration(tmp_path, monkeypatch, change):
    runtime = DemoRuntime(tmp_path)
    calls = []
    execute = runtime.execute
    def record(path, site_hash, calibrate):
        calls.append(calibrate)
        execute(path, site_hash, calibrate)
    runtime.execute = record
    answers(monkeypatch, ['3', '1', change, '3', '1', '0'])
    menu(runtime, {}, tmp_path/'settings.json', demo=True)
    assert calls == [True, False]
    assert runtime.calibrated_legs == {'L2'}


def test_selecting_subset_of_calibrated_legs_does_not_home_again(tmp_path, monkeypatch, capsys):
    runtime = DemoRuntime(tmp_path)
    calls = []
    execute = runtime.execute
    def record(path, site_hash, calibrate):
        calls.append(calibrate)
        execute(path, site_hash, calibrate)
    runtime.execute = record
    answers(monkeypatch, ['9', '2 5', '1', '3', '1', 'L2 +5', '3', '1', '0'])
    menu(runtime, {}, tmp_path/'settings.json', demo=True)
    assert calls == [True, False]
    assert runtime.calibrated_legs == {'L2', 'R2'}
    text = capsys.readouterr().out
    assert '本次校正：只校正 L2 R2\n' in text
    assert '本次校正：不需要' in text


def test_power_off_revokes_repeat_confirmation(tmp_path,monkeypatch):
    runtime=DemoRuntime(tmp_path)
    answers(monkeypatch,['3','y','4','3','','0'])
    menu(runtime,{},tmp_path/'settings.json',demo=True)
    assert not runtime.calibrated


def test_edit_supports_different_modes_per_leg(monkeypatch,tmp_path):
    answers(monkeypatch,['2 5','1','-5','4','10','180','4'])
    p=edit_plan(default_plan(),DemoRuntime(tmp_path).site())
    assert p['legs']['L2']=={'mode':'relative','move_deg':-5.0}
    assert p['legs']['R2']=={'mode':'cycle','speed_deg_s':10.0,'phase_deg':180.0}
    assert p['duration_s']==4


def test_numeric_wizard_retries_only_invalid_field_and_preserves_other_settings(monkeypatch,tmp_path):
    old=default_plan()
    old['max_pwm']=35
    answers(monkeypatch,['7','2','7','3','abc','20',''])
    p=edit_plan(old,DemoRuntime(tmp_path).site())
    assert p['legs']=={'L2':dict(mode='velocity',speed_deg_s=20)}
    assert p['max_speed_deg_s']==20
    assert p['max_pwm']==35 and p['duration_s']==3
    assert old['legs']['L2']['mode']=='relative'


def test_cancelling_last_wizard_step_keeps_original_plan(monkeypatch,tmp_path):
    import pytest
    from rinbo_control.runtime import Cancelled
    old=default_plan()
    answers(monkeypatch,['2','3','20','q'])
    with pytest.raises(Cancelled):
        edit_plan(old,DemoRuntime(tmp_path).site())
    assert old==default_plan()


def test_number_only_preset_and_execution_support_multiple_legs(tmp_path,monkeypatch):
    import json
    runtime=DemoRuntime(tmp_path)
    replies=['9','2 5','4','3','1','0']
    answers(monkeypatch,replies)
    path=tmp_path/'settings.json'
    menu(runtime,{},path,demo=True)
    plan=json.loads(path.read_text())['plan']
    assert plan['legs']=={'L2':dict(mode='velocity',speed_deg_s=-5),
                          'R2':dict(mode='velocity',speed_deg_s=-5)}
    assert runtime.calibrated_legs=={'L2','R2'}
    assert plan['max_pwm']==20 and plan['duration_s']==3


def test_refresh_and_cancelled_preset_do_not_actuate_or_save(tmp_path,monkeypatch):
    runtime=DemoRuntime(tmp_path)
    def forbidden(*a): raise AssertionError('display/cancel must not actuate')
    runtime.connect=runtime.execute=runtime.safe_off=runtime.change_legs=forbidden
    answers(monkeypatch,['','9','q','0'])
    path=tmp_path/'settings.json'
    menu(runtime,{},path,demo=True)
    assert not path.exists()


def test_enter_keeps_displayed_value_even_when_suggestion_differs(monkeypatch):
    prompts=[]
    def reply(prompt):
        prompts.append(prompt)
        return ''
    monkeypatch.setattr(builtins,'input',reply)
    assert numeric('速度上限',37,1,90,suggestion='先用 10 度／秒。')==37
    assert '直接 Enter 用 37' in prompts[0]


def test_help_and_starter_preset_never_actuate_and_respect_disabled_legs(tmp_path,monkeypatch):
    import json
    runtime=DemoRuntime(tmp_path)
    runtime.site=lambda:dict(disabled_legs=['L1','L2','L3'],enabled_legs=['R1','R2','R3'],hash='DEMO')
    def forbidden(*a):
        raise AssertionError('help/preset must not connect, power or move')
    runtime.connect=runtime.execute=runtime.safe_off=forbidden
    old=default_plan(['L1','L2','L3'])
    old.update(duration_s=8,max_pwm=35,max_speed_deg_s=30,acceleration_deg_s2=25)
    old['legs']={'R1':dict(mode='velocity',speed_deg_s=-12)}
    answers(monkeypatch,['h','r','0'])
    path=tmp_path/'settings.json'
    menu(runtime,dict(plan=old),path,demo=True)
    saved=json.loads(path.read_text())['plan']
    assert saved==default_plan(['L1','L2','L3'])
    assert list(saved['legs'])==['R2']


def test_network_retries_bad_field_without_losing_previous_values(monkeypatch):
    from rinbo_control.console import edit_network
    answers(monkeypatch, ['bad', '192.168.30.253', '127.0.0.1', '', '0', '５００５１', '50052'])
    assert edit_network('192.168.30.254', '192.168.30.8', 50051, demo=True) == (
        '192.168.30.253', '192.168.30.8', 50052)


def test_limit_retry_keeps_other_fields_and_does_not_reduce_requested_velocity(monkeypatch):
    from rinbo_control.console import edit_limits
    from rinbo_control.plans import quick_plan
    old = quick_plan('L2 v -25', default_plan())
    answers(monkeypatch, ['35', '20', '10', '30'])
    updated = edit_limits(old)
    assert updated['max_pwm'] == 35 and updated['acceleration_deg_s2'] == 20
    assert updated['max_speed_deg_s'] == 30
    assert updated['legs'] == old['legs'] and old['max_pwm'] == 20


@pytest.mark.parametrize('entry', [
    ['L2 -5'], ['9', '5', '2'], ['2', '5', '1', '-5', '4'],
    ['7', '30', '20', '20'], ['r'],
])
def test_failed_preference_save_retains_original_action_and_reports_failure(tmp_path, monkeypatch, capsys, entry):
    import json
    runtime = DemoRuntime(tmp_path)
    old = default_plan()
    old['duration_s'] = 6
    def fail(*args):
        raise OSError('disk full')
    monkeypatch.setattr('rinbo_control.console.atomic_json', fail)
    executed = []
    def execute(path, site_hash, calibrate):
        executed.append(json.loads(Path(path).read_text()))
    runtime.execute = execute
    answers(monkeypatch, entry + ['3', '1', '0'])
    menu(runtime, dict(plan=old), tmp_path/'settings.json', demo=True)
    assert executed == [old]
    output = capsys.readouterr().out
    assert '設定無法儲存' in output
    assert '動作已儲存' not in output


def test_failed_network_save_retains_original_target(tmp_path, monkeypatch):
    runtime = DemoRuntime(tmp_path)
    def fail(*args): raise OSError('disk full')
    monkeypatch.setattr('rinbo_control.console.atomic_json', fail)
    calls = []
    runtime.ensure_connected = lambda *args: calls.append(args)
    answers(monkeypatch, ['6', '192.168.30.253', '', '', '3', '', '0'])
    menu(runtime, {}, tmp_path/'settings.json', demo=True)
    assert calls == [('192.168.30.254', 50051, '192.168.30.8')]


def test_view_refresh_and_connection_do_not_rewrite_preferences(tmp_path, monkeypatch):
    def fail(*args): raise AssertionError('read-only action tried saving preferences')
    monkeypatch.setattr('rinbo_control.console.atomic_json', fail)
    answers(monkeypatch, ['1', '5', '', 'h', '0'])
    menu(DemoRuntime(tmp_path), {}, tmp_path/'settings.json', demo=True)


def test_masked_selection_can_edit_limits_but_cannot_execute(tmp_path, monkeypatch):
    from rinbo_control.plans import load_settings
    runtime = DemoRuntime(tmp_path)
    runtime.disabled = ['L1', 'L2']
    answers(monkeypatch, ['7', '30', '20', '20', '3', '0'])
    path = tmp_path/'settings.json'
    menu(runtime, dict(plan=default_plan()), path, demo=True)
    assert load_settings(path)['plan']['max_pwm'] == 30
    assert not runtime.connected
