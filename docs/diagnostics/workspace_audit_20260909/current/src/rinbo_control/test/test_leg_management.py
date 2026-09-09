import builtins
import copy
import socket
import subprocess
from types import SimpleNamespace

import pytest

from rinbo_control.console import manage_legs, menu
from rinbo_control.demo import DemoRuntime
from rinbo_control.plans import LEGS, default_plan, load_settings, updated_mask
from rinbo_control.runtime import Runtime


def replies(monkeypatch, values):
    iterator = iter(values)
    monkeypatch.setattr(builtins, 'input', lambda _: next(iterator))


def test_numbered_panel_adds_and_removes_multiple_legs_without_external_calls(tmp_path, monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        raise AssertionError('demo accessed external processes or hardware')
    monkeypatch.setattr(subprocess, 'run', forbidden)
    monkeypatch.setattr(subprocess, 'Popen', forbidden)
    monkeypatch.setattr(socket, 'create_connection', forbidden)
    runtime = DemoRuntime(tmp_path)
    replies(monkeypatch, ['1', '2 3', '2', '1 2', '5', '0'])
    manage_legs(runtime)
    assert runtime.site()['disabled_legs'] == ['L3']
    assert runtime.revision == 3
    output = capsys.readouterr().out
    assert 'L3（左後）：已屏蔽' in output
    assert 'L1（左前）：可用' in output
    assert not runtime.connected


def test_cancel_invalid_and_view_only_do_not_change_mask(tmp_path, monkeypatch):
    runtime = DemoRuntime(tmp_path)
    before = runtime.site()
    replies(monkeypatch, ['1', '', '1', '7', '1', '2 2', '5', '0'])
    manage_legs(runtime)
    assert runtime.site() == before


def test_all_masked_remains_manageable_across_restart_and_does_not_switch_selected_leg(tmp_path, monkeypatch, capsys):
    runtime = DemoRuntime(tmp_path)
    path = tmp_path/'settings.json'
    replies(monkeypatch, ['8', '3', '0', '3', '0'])
    menu(runtime, {}, path, demo=True)
    assert runtime.disabled == list(LEGS)
    assert not runtime.connected  # blocked plan is rejected before connecting
    prefs = load_settings(path)
    assert prefs['plan'] == default_plan(['L1'])
    assert '目前動作中的 L2 已屏蔽' in capsys.readouterr().out
    replies(monkeypatch, ['8', '2', '2', '0', '0'])
    menu(runtime, prefs, path, demo=True)
    assert runtime.site()['enabled_legs'] == ['L2']
    assert load_settings(path)['plan'] == prefs['plan']


def test_fresh_all_masked_console_can_save_preferences_and_enable_all(tmp_path, monkeypatch):
    runtime = DemoRuntime(tmp_path)
    runtime.disabled = list(LEGS)
    path = tmp_path/'settings.json'
    replies(monkeypatch, ['8', '0', '0'])
    menu(runtime, {}, path, demo=True)
    prefs = load_settings(path)  # no invalid empty plan persisted
    replies(monkeypatch, ['8', '4', '0', '0'])
    menu(runtime, prefs, path, demo=True)
    assert not runtime.disabled


def test_mask_change_requires_new_review_and_calibration(tmp_path, monkeypatch):
    runtime = DemoRuntime(tmp_path)
    replies(monkeypatch, ['3', 'y', '8', '1', '3', '0', '3', '', '0'])
    menu(runtime, {}, tmp_path/'settings.json', demo=True)
    assert runtime.disabled == ['L1', 'L3']
    assert not runtime.calibrated  # last attempt cancelled at renewed review


def runtime_harness(tmp_path, monkeypatch, connected=False):
    runtime = Runtime(tmp_path, progress=lambda _: None)
    state = dict(disabled_legs=['L1'], enabled_legs=list(LEGS[1:]), hash='first')
    events = []
    runtime.site = lambda: copy.deepcopy(state)
    runtime.idle = lambda: events.append('idle')
    runtime.status = lambda: dict(bridge_count=int(connected))
    runtime.safe_off = lambda: events.append('off') or True
    def query(package, name, operation, *legs):
        events.append((package, name, operation, legs))
        state['disabled_legs'] = updated_mask(state['disabled_legs'], operation, legs)
        state['hash'] = 'changed'
        return SimpleNamespace(returncode=0, stdout='', stderr='')
    runtime.query = query
    return runtime, state, events


@pytest.mark.parametrize('connected', [False, True])
def test_runtime_uses_native_delta_command_and_reads_actual_result(tmp_path, monkeypatch, connected):
    runtime, state, events = runtime_harness(tmp_path, monkeypatch, connected)
    runtime.calibration_epoch = 20
    actual = runtime.change_legs('disable', ['L3'])
    assert events == ['idle'] + (['off'] if connected else []) + [
        ('rinbo_fsm', 'rinbo_legs', 'disable', ('L3',))]
    assert actual['disabled_legs'] == ['L1', 'L3']
    assert runtime.calibration_epoch is None


def test_noop_and_invalid_request_do_not_power_off_or_write(tmp_path, monkeypatch):
    runtime, state, events = runtime_harness(tmp_path, monkeypatch, True)
    runtime.calibration_epoch = 20
    runtime.change_legs('disable', ['L1'])
    assert not events and runtime.calibration_epoch == 20
    for operation, names in [('set', ['L3']), ('disable', []), ('enable', ['L7']), ('enable-all', ['L2'])]:
        with pytest.raises(ValueError):
            runtime.change_legs(operation, names)
    assert not events


@pytest.mark.parametrize('failure', ['busy', 'off', 'write'])
def test_failed_change_never_reports_a_new_mask(tmp_path, monkeypatch, failure):
    runtime, state, events = runtime_harness(tmp_path, monkeypatch, True)
    if failure == 'busy':
        runtime.idle = lambda: (_ for _ in ()).throw(RuntimeError('another controller is moving'))
    elif failure == 'off':
        runtime.safe_off = lambda: False
    else:
        runtime.query = lambda *args: SimpleNamespace(returncode=2, stdout='', stderr='Configuration/action is busy')
    with pytest.raises(RuntimeError):
        runtime.change_legs('disable', ['L3'])
    assert state['disabled_legs'] == ['L1']
    assert not any(isinstance(event, tuple) for event in events)
