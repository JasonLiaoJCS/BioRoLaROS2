import builtins
from pathlib import Path
import socket
import subprocess
from rinbo_control.console import edit_plan, menu
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


def test_edit_supports_different_modes_per_leg(monkeypatch,tmp_path):
    answers(monkeypatch,['L2 R2','4','20','1','-5','4','10','180'])
    p=edit_plan(default_plan(),DemoRuntime(tmp_path).site())
    assert p['legs']['L2']=={'mode':'relative','move_deg':-5.0}
    assert p['legs']['R2']=={'mode':'cycle','speed_deg_s':10.0,'phase_deg':180.0}
    assert p['duration_s']==4
