import pytest

from rinbo_control.calibration_progress import CalibrationProgress
from rinbo_control.plans import LEGS, friendly_error


def status(done=4, total=5, skipped=1):
    return (f'DC_SPINNING | t: 24.54 | Healthy complete: {done}/{total} | '
            f'SKIPPED: {skipped} | Hall: [1 0 1 0 0 0]')


def test_site_log_reports_l3_pending_and_l1_skipped():
    progress = CalibrationProgress()
    assert '跳過' in progress.consume('L1: SKIPPED (main drive disabled)')
    for leg in ('L2', 'R1', 'R2', 'R3'):
        progress.consume(f'{leg}: DONE! Zero feedback confirmed.')
    message = progress.consume(status())
    assert '4／5' in message
    assert 'L3（尋找零點訊號）' in message
    assert 'L1' not in message
    assert '24.5 秒' in message
    error = friendly_error(status() + '\nCALIBRATION SAFETY STOP: hall search timeout: L3')
    assert 'L3' in error and '零點感測器' in error and '屏蔿名單' in error
    assert 'Hall: [' not in error


@pytest.mark.parametrize('mask', range(63))
def test_display_uses_reported_mask_without_hardcoded_disabled_legs(mask):
    progress = CalibrationProgress()
    disabled = [leg for i, leg in enumerate(LEGS) if mask & (1 << i)]
    active = [leg for leg in LEGS if leg not in disabled]
    for leg in disabled:
        progress.consume(f'{leg}: SKIPPED (main drive disabled)')
    for leg in active[:-1]:
        progress.consume(f'{leg}: DONE! Zero feedback confirmed.')
    message = progress.consume(status(len(active)-1, len(active), len(disabled)))
    assert f'{active[-1]}（尋找零點訊號）' in message
    assert all(leg not in message for leg in disabled)


def test_missing_ui_events_never_guess_pending_legs():
    progress = CalibrationProgress()
    assert '仍在等' not in progress.consume(status())
    progress.consume('L1: SKIPPED (main drive disabled)')
    assert '仍在等' not in progress.consume(status())


def test_sensor_detection_does_not_mean_calibration_done():
    progress = CalibrationProgress()
    progress.consume('L2: Hall detected! Stopping...')
    assert 'L2（等待停穩）' in progress.consume(status(0, 6, 0))
    progress.consume('L2: RESETTING (waiting for zero feedback)')
    assert 'L2（等待位置歸零回讀）' in progress.consume(status(0, 6, 0))
    assert not progress.done


@pytest.mark.parametrize('error, meaning', [
    ('healthy servo homing timeout: L2,R2', '伺服尚未到達'),
    ('motor stop timeout: L2', '尚未確認停穩'),
    ('position reset timeout: L2 pos=1000', '尚未確認歸零'),
])
def test_each_calibration_wait_has_a_distinct_plain_language_error(error, meaning):
    text = friendly_error(error)
    assert 'L2' in text and meaning in text
    assert '不是等待其他未選中的腳' in text
