import builtins
import copy
import json
import socket
import subprocess

import pytest

from rinbo_control.console import menu
from rinbo_control.demo import DemoRuntime
from rinbo_control.library import load_library, save_library
from rinbo_control.plans import adjust_plan, default_plan, load_settings, quick_plan


def replies(monkeypatch, values):
    values = iter(values)
    monkeypatch.setattr(builtins, 'input', lambda _: next(values))


def test_reverse_and_scale_preserve_phase_angle_time_and_effort():
    plan = default_plan()
    plan['legs'] = {'L2': dict(mode='relative', move_deg=5),
                    'R2': dict(mode='cycle', phase_deg=180, speed_deg_s=-5)}
    reverse = adjust_plan(plan, 'reverse')
    assert reverse['legs']['L2']['move_deg'] == -5
    assert reverse['legs']['R2'] == dict(mode='cycle', phase_deg=180, speed_deg_s=5)
    half = adjust_plan(plan, 'half')
    assert half['max_speed_deg_s'] == 5
    assert half['legs']['R2']['speed_deg_s'] == -2.5
    assert half['legs']['L2'] == plan['legs']['L2']
    assert adjust_plan(half, 'double') == plan
    for key in ('duration_s', 'max_pwm', 'acceleration_deg_s2'):
        assert half[key] == reverse[key] == plan[key]


def test_invalid_adjustment_never_partially_changes_plan():
    plan = quick_plan('L2 v 60', default_plan())
    before = copy.deepcopy(plan)
    with pytest.raises(ValueError): adjust_plan(plan, 'double')
    assert plan == before
    plan['legs']['R2'] = dict(mode='position', angle_deg=90)
    before = copy.deepcopy(plan)
    with pytest.raises(ValueError): adjust_plan(plan, 'reverse')
    assert plan == before
    with pytest.raises(ValueError): adjust_plan(plan, 'half', ['L2'])


def test_copy_replaces_selection_and_independently_copies_each_target():
    plan = quick_plan('L2 p 180 10', default_plan())
    copied = adjust_plan(plan, 'copy', ['L2'], source='L2', targets=['R1', 'R2'])
    assert set(copied['legs']) == {'R1', 'R2'}
    assert copied['legs']['R2'] == plan['legs']['L2']
    copied['legs']['R1']['phase_deg'] = 90
    assert copied['legs']['R2']['phase_deg'] == plan['legs']['L2']['phase_deg'] == 180
    with pytest.raises(ValueError):
        adjust_plan(plan, 'copy', ['L1'], source='L2', targets=['L1'])


def test_library_is_separate_and_survives_reload(tmp_path):
    path = tmp_path/'motions.json'
    data = {'慢速測試': quick_plan('L2 v 5', default_plan())}
    save_library(path, data)
    assert load_library(path) == data
    assert path.stat().st_mode & 0o077 == 0
    assert not list(tmp_path.glob('*.calibration.json'))


@pytest.mark.parametrize('bad', [[], {'\x1b[0m': default_plan()},
    {'broken': dict(default_plan(), calibration=True)},
    {'broken': dict(default_plan(), legs={'L2': dict(mode=[])})},
    {'broken': dict(default_plan(), legs={'l2': dict(mode='relative', move_deg=5)})}])
def test_bad_library_is_rejected_without_overwriting(tmp_path, bad):
    path = tmp_path/'motions.json'
    original = {'試轉': default_plan()}
    save_library(path, original)
    with pytest.raises(ValueError): save_library(path, bad)
    assert load_library(path) == original
    path.write_text(json.dumps(bad))
    with pytest.raises(ValueError): load_library(path)


def test_all_new_features_are_settings_only(tmp_path, monkeypatch):
    def forbidden(*args, **kwargs): raise AssertionError('settings touched hardware')
    runtime = DemoRuntime(tmp_path)
    runtime.connect = runtime.execute = runtime.safe_off = runtime.change_legs = forbidden
    monkeypatch.setattr(subprocess, 'run', forbidden)
    monkeypatch.setattr(subprocess, 'Popen', forbidden)
    monkeypatch.setattr(socket, 'create_connection', forbidden)
    replies(monkeypatch, ['10', '1', '我的測試', '0', '11', '1', '12',
                          '11', '4', '5', '10', '2', '1', '0'])
    menu(runtime, {}, tmp_path/'settings.json', demo=True)
    assert load_settings(tmp_path/'settings.json')['plan'] == default_plan()
    assert load_library(tmp_path/'motions.json') == {'我的測試': default_plan()}
    assert runtime.disabled == ['L1']


def test_undo_is_settings_only_and_requires_review_again(tmp_path, monkeypatch):
    runtime = DemoRuntime(tmp_path)
    calls = []
    execute = runtime.execute
    def record(*args):
        calls.append(args)
        execute(*args)
    runtime.execute = record
    replies(monkeypatch, ['3', '1', '11', '1', '12', '3', '', '0'])
    menu(runtime, {}, tmp_path/'settings.json', demo=True)
    assert len(calls) == 1
    assert load_settings(tmp_path/'settings.json')['plan'] == default_plan()


def test_load_same_favorite_does_not_reuse_previous_confirmation(tmp_path, monkeypatch):
    runtime = DemoRuntime(tmp_path)
    calls = []
    execute = runtime.execute
    def record(*args):
        calls.append(args)
        execute(*args)
    runtime.execute = record
    save_library(tmp_path/'motions.json', {'原動作': default_plan()})
    replies(monkeypatch, ['3', '1', '10', '2', '1', '3', '', '0'])
    menu(runtime, {}, tmp_path/'settings.json', demo=True)
    assert len(calls) == 1


def test_masked_favorite_is_visible_but_cannot_be_loaded(tmp_path, monkeypatch, capsys):
    save_library(tmp_path/'motions.json', {'停用脚': quick_plan('L1 +5', default_plan())})
    runtime = DemoRuntime(tmp_path)
    replies(monkeypatch, ['10', '2', '1', '0', '0'])
    menu(runtime, {}, tmp_path/'settings.json', demo=True)
    assert '已屏蔽：L1' in capsys.readouterr().out
    assert not (tmp_path/'settings.json').exists()
    assert not runtime.connected


def test_cancel_overwrite_and_delete_preserves_saved_motion(tmp_path, monkeypatch):
    saved = quick_plan('L2 -5', default_plan())
    path = tmp_path/'motions.json'
    save_library(path, {'已存': saved})
    replies(monkeypatch, ['10', '1', '已存', '', '3', '1', '', '0', '0'])
    menu(DemoRuntime(tmp_path), {}, tmp_path/'settings.json', demo=True)
    assert load_library(path) == {'已存': saved}


def test_confirmed_delete_removes_only_chosen_favorite(tmp_path, monkeypatch):
    path = tmp_path/'motions.json'
    save_library(path, {'保留': default_plan(), '刪除': default_plan()})
    replies(monkeypatch, ['10', '3', '2', '1', '0', '0'])
    menu(DemoRuntime(tmp_path), {}, tmp_path/'settings.json', demo=True)
    assert list(load_library(path)) == ['保留']


def test_failed_undo_keeps_history_for_retry(tmp_path, monkeypatch):
    import rinbo_control.console as console
    save = console.atomic_json
    writes = 0
    def flaky(*args):
        nonlocal writes
        writes += 1
        if writes == 2: raise OSError('disk full')
        save(*args)
    monkeypatch.setattr(console, 'atomic_json', flaky)
    replies(monkeypatch, ['11', '1', '12', '12', '0'])
    menu(DemoRuntime(tmp_path), {}, tmp_path/'settings.json', demo=True)
    assert writes == 3
    assert load_settings(tmp_path/'settings.json')['plan'] == default_plan()


def test_undo_never_restores_mask_or_silently_changes_selected_leg(tmp_path, monkeypatch):
    runtime = DemoRuntime(tmp_path)
    replies(monkeypatch, ['R2 +5', '8', '1', '2', '0', '12', '3', '0'])
    menu(runtime, {}, tmp_path/'settings.json', demo=True)
    assert runtime.disabled == ['L1', 'L2']
    assert load_settings(tmp_path/'settings.json')['plan'] == default_plan()
    assert not runtime.connected
