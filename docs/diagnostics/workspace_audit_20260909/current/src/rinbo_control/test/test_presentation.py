from rinbo_control.plans import default_plan, numbered_legs, preset_plan
from rinbo_control.presentation import dashboard
import pytest


def test_numbers_and_names_are_consistent_and_cannot_duplicate_or_select_masked():
    assert numbered_legs('2 5', ['L1']) == ['L2','R2']
    assert numbered_legs('l2 5', ['L1']) == ['L2','R2']
    for choice in ('0', '7', '2 L2', '1', ''):
        with pytest.raises(ValueError): numbered_legs(choice,['L1'])
    with pytest.raises(ValueError): preset_plan(['L1'],'1',['L1'])


def test_dashboard_never_claims_connection_from_bridge_presence_alone():
    site=dict(disabled_legs=['L1'],enabled_legs=['L2','L3','R1','R2','R3'])
    plan=default_plan()
    snapshot=dict(bridge_count=1,motor_fresh=False,power_fresh=False,relay_on=None,voltage=None)
    text=dashboard(site,plan,snapshot,'')
    assert '連線：正常' not in text
    assert '馬達電源：未知' in text
    assert '2 L2 左中 [可用] *' in text
    assert '1 L1 左前 [屏蔽]' in text
    assert '尚未檢查' in dashboard(site,plan,None,'')


def test_missing_or_fully_masked_site_gives_actionable_next_step():
    plan=default_plan()
    assert '讀取失敗，狀態未知' in dashboard(None,plan,None,'')
    site=dict(disabled_legs=['L1','L2','L3','R1','R2','R3'],enabled_legs=[])
    text=dashboard(site,plan,None,'')
    assert '目前動作中的 L2 已屏蔽' in text
    assert '先選 8' in text


def test_stale_feedback_never_shows_cached_angles_or_hall():
    from rinbo_control.presentation import leg_feedback
    snapshot = dict(motor_fresh=True, legs={'L2': dict(angle_deg=180, hall=True)})
    text = leg_feedback(snapshot)
    assert '+180.0' in text and '未觸發（1）' in text
    snapshot['motor_fresh'] = False
    text = leg_feedback(snapshot)
    assert '+180.0' not in text and 'L2 左中' in text
    assert text.count('未知') == 12


def test_invalid_angle_is_unknown_and_does_not_hide_hall():
    from rinbo_control.presentation import leg_feedback
    text = leg_feedback(dict(motor_fresh=True, legs={'L2': dict(angle_deg=float('nan'), hall=True)}))
    assert 'nan' not in text and '未觸發（1）' in text


def test_hall_display_matches_active_low_calibration_and_does_not_certify_completion():
    from rinbo_control.presentation import leg_feedback
    text = leg_feedback(dict(motor_fresh=True, legs={
        'L2':dict(angle_deg=0, hall=False), 'R2':dict(angle_deg=0, hall=True)}))
    assert '已觸發（0）' in next(line for line in text.splitlines() if 'L2 左中' in line)
    assert '未觸發（1）' in next(line for line in text.splitlines() if 'R2 右中' in line)
    assert '還需停穩與位置歸零回讀' in text


def test_execution_scope_separates_selected_legs_from_site_enabled_legs():
    from rinbo_control.presentation import execution_scope
    site = dict(disabled_legs=['L1'], enabled_legs=['L2','L3','R1','R2','R3'],
                parameters={'rinbo_cali':{'max_pwm':80}})
    plan = default_plan()
    plan['max_pwm'] = 35
    text = execution_scope(plan, site, True, '本次尚未校正')
    assert '本次校正：只校正 L2\n' in text
    assert '不參與的主馬達：L1 L3 R1 R2 R3' in text
    assert '校正出力上限：80；校正後逐腳動作出力上限：35' in text
    text = execution_scope(default_plan(), site, False, '有效')
    assert '本次校正：不需要' in text
    assert '校正出力上限' not in text


def test_next_step_identifies_other_controller_before_suggesting_execution():
    site = dict(disabled_legs=[], enabled_legs=['L2'])
    snapshot = dict(bridge_count=1, motor_fresh=True, power_fresh=True,
                    relay_on=True, voltage=24, writers=['/rinbo_manual'])
    text = dashboard(site, default_plan(), snapshot, '', target='sbRIO 192.168.30.254')
    assert '下一步：已有控制程式' in text
    assert '設定目標：sbRIO 192.168.30.254' in text
