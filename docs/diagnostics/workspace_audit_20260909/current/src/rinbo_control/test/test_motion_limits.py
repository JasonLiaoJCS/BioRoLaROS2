import copy
import json
import subprocess
from types import SimpleNamespace

import pytest
import yaml
from rinbo_control.demo import DemoRuntime
from rinbo_control.motion_limits import edit_motion_limits, current_values
from rinbo_control.runtime import Runtime
from rinbo_control.sim_motion_limits import read_profile, update_profile


def run_editor(runtime, choices, outputs=None, number=None):
    answers = iter(choices)
    edit_motion_limits(runtime, lambda *args: next(answers), (outputs if outputs is not None else []).append,
                       lambda *args, **kwargs: number)


def test_presets_preview_apply_and_restore_never_use_hardware(tmp_path, monkeypatch):
    runtime = DemoRuntime(tmp_path)
    def forbidden(*args, **kwargs): raise AssertionError('unexpected hardware or subprocess access')
    monkeypatch.setattr(subprocess, 'run', forbidden)
    runtime.connect = runtime.execute = runtime.safe_off = forbidden
    original = copy.deepcopy(runtime.parameters)
    outputs = []
    run_editor(runtime, ['2', 'p', '', '0', '1', 'p', '', '0', '3', 'p', '', '0', '4', '0'], outputs)
    assert current_values(runtime.site(), 'standing')['position_tolerance_counts'] == 1000
    assert current_values(runtime.site(), 'standing')['rotate_timeout_s'] == 60
    assert current_values(runtime.site(), 'calibration')['stop_timeout_s'] == 15
    assert current_values(runtime.site(), 'tripod')['stop_on_position_error'] == 0
    assert current_values(runtime.site(), 'tripod')['max_pwm'] == 3300
    assert current_values(runtime.site(), 'tripod')['enable_pwm_slew_limit'] == 0
    assert any('已儲存並核對' in line for line in outputs)
    run_editor(runtime, ['2', 'r', '', '0', '1', 'r', '', '0', '3', 'r', '', '0', '0'])
    assert runtime.parameters == original


def test_cancel_and_custom_integer_validation_do_not_save(tmp_path):
    runtime = DemoRuntime(tmp_path)
    original = copy.deepcopy(runtime.parameters)
    run_editor(runtime, ['2', 'p', 'q', '0', '3', '4', '0', '0'], number=1.5)
    assert runtime.parameters == original
    run_editor(runtime, ['2', '1', '', '0', '0'], number=1500)
    assert current_values(runtime.site(), 'standing')['position_tolerance_counts'] == 1500


def test_tripod_optional_slew_switch_and_rate_are_editable(tmp_path):
    runtime=DemoRuntime(tmp_path)
    run_editor(runtime,['3','6','','0','0'],number=0)
    assert current_values(runtime.site(),'tripod')['enable_pwm_slew_limit']==0
    run_editor(runtime,['3','7','','0','0'],number=2000)
    assert current_values(runtime.site(),'tripod')['pwm_slew_rate_per_sec']==2000


def test_native_adapter_sends_revision_and_never_power_commands(tmp_path, monkeypatch):
    runtime = Runtime(tmp_path)
    calls = []
    def query(*args):
        calls.append(args)
        return SimpleNamespace(returncode=0,stdout=json.dumps({'changed':True,'hash':'new'}))
    monkeypatch.setattr(runtime, 'query', query)
    runtime.tune_limits('standing', {'position_tolerance_counts':1000},True,9)
    runtime.tune_limits('standing', {'position_tolerance_counts':1000},False,9)
    assert calls[0] == ('rinbo_fsm','rinbo_legs','tune-limits','standing','--expect-revision','9','--dry-run','position_tolerance_counts=1000')
    assert '--dry-run' not in calls[1]
    monkeypatch.setattr(runtime, 'query', lambda *args: SimpleNamespace(returncode=2,stderr='busy'))
    with pytest.raises(RuntimeError,match='busy'):
        runtime.tune_limits('standing', {'rotate_timeout_s':60},False,9)


def test_sim_profile_keeps_comments_other_guards_and_model_metadata(tmp_path):
    path=tmp_path/'profile.yaml'
    raw='''redrhex_rl_controller:
  ros__parameters:
    policy: {expected_sha256: verified-hash}
    safety: {sensor_timeout_s: 0.1, max_current: 5}
    state_machine:
      init_stand_timeout_s: 12.0 # preserve this comment
      init_stand_position_tolerance_rad: 0.12
      init_stand_velocity_tolerance_rad_s: 0.25
'''
    path.write_text(raw)
    _, digest, _=read_profile(path)
    assert update_profile(path,{'init_stand_timeout_s':60},digest)['changed']
    assert path.read_text()==raw
    with pytest.raises(RuntimeError): update_profile(path,{'init_stand_timeout_s':60},'wrong',False)
    for update in ({'max_current':10},{'init_stand_timeout_s':float('nan')},{'init_stand_timeout_s':61}):
        with pytest.raises(ValueError): update_profile(path,update,digest,False)
    result=update_profile(path,{'init_stand_timeout_s':60},digest,False)
    assert 'preserve this comment' in path.read_text()
    assert result['model_revalidation_required']
    before=yaml.safe_load(raw)['redrhex_rl_controller']['ros__parameters']
    after=yaml.safe_load(path.read_text())['redrhex_rl_controller']['ros__parameters']
    assert after['policy']==before['policy'] and after['safety']==before['safety']
    assert read_profile(path)[2]['init_stand_timeout_s']==60
    assert open(result['backup']).read()==raw


def test_motion_limits_are_reachable_from_actual_main_menu(tmp_path, monkeypatch, capsys):
    import builtins
    from rinbo_control.console import menu
    from rinbo_control.plans import load_settings
    runtime=DemoRuntime(tmp_path)
    inputs=iter(['15','2','p','','0','0','0'])
    monkeypatch.setattr(builtins,'input',lambda _:next(inputs))
    settings_path=tmp_path/'settings.json'
    menu(runtime,load_settings(settings_path),settings_path,demo=True)
    assert current_values(runtime.site(),'standing')['position_tolerance_counts']==1000
    assert '15 到位／追蹤限制' in capsys.readouterr().out


def test_sim_invalid_profile_is_a_recoverable_error(tmp_path):
    path=tmp_path/'bad.yaml';path.write_text('invalid: true')
    with pytest.raises(ValueError,match='不是支援的'):
        read_profile(path)
