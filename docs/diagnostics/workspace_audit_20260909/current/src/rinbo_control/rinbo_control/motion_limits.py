"""Operator edits to arrival/tracking limits. No power or motion commands."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Field:
    label: str
    unit: str
    default: float
    minimum: float
    maximum: float
    help: str


NODES = {'calibration': 'rinbo_cali', 'standing': 'rinbo_standing', 'tripod': 'rinbo_tripod_rslip'}
FIELDS = {
    'calibration': {
        'servo_homing_timeout_s': Field('伺服定位最多等待', '秒', 20, .1, 600, '延長等待；仍需收到伺服到位回讀。'),
        'hall_search_timeout_s': Field('尋找零點最多等待', '秒', 30, .1, 600, '延長尋零；沒有找到 Hall 零點仍不算校正完成。'),
        'stop_timeout_s': Field('停穩／確認歸零最多等待', '秒', 5, .1, 600, '每階段各自計時，仍需實際停穩與歸零。'),
    },
    'standing': {
        'position_tolerance_counts': Field('到位容許誤差', 'counts', 200, 1, 12000, '越大越容易判定到位；1000 counts 約 6.51°（採 55296 counts/rev）。'),
        'rotate_timeout_s': Field('轉到站姿最多等待', '秒', 20, .1, 600, '只限制這次到位等待，並非持續站立的總時間。'),
        'hall_search_timeout_s': Field('尋找零點最多等待', '秒', 30, .1, 600, '沒找到零點仍會停止，避免把未知位置當完成。'),
        'settle_velocity_counts_s': Field('判定停穩的速度上限', 'counts/s', 500, 1, 5000, '位置到位還需要低於這個速度；500 counts/s 約 3.26°/s。'),
        'settle_time_s': Field('到位且停穩需維持', '秒', .3, .01, 10, '必須連續符合位置與速度條件；不要把高速經過目標當成完成。'),
        'hold_error_counts': Field('站穩後偏離停止門檻', 'counts', 12000, 1, 55296, '適用已到位後；必須不小於到位容許誤差。'),
    },
    'tripod': {
        'stop_on_position_error': Field('位置誤差處理', '0=只警告；1=停止', 1, 0, 1, '0：有限位置誤差只警告。1：恢復軟門檻與 18000 counts 硬停止。無效回饋兩者都停止。'),
        'max_position_error_counts': Field('追蹤誤差警告／軟門檻', 'counts', 9000, 1, 12000, '只警告模式下不會因此停機；採 54984.83 counts/rev 換算，硬體比例尚待確認。'),
        'position_error_trip_seconds': Field('持續超過軟門檻的時間', '秒', .5, .01, 2, '停止模式必須同時滿足持續時間與次數；硬上限不等待。'),
        'position_error_trip_samples': Field('連續超過軟門檻的次數', '次', 10, 1, 10, '整數。只警告模式仍記錄計數，持續執行。'),
        'max_pwm': Field('Tripod 出力上限 Max PWM', '原始命令值', 80, 1, 3300, '最大允許出力；不是固定輸出或百分比。Bridge 需使用本次支援 3300 的版本。'),
        'enable_pwm_slew_limit': Field('額外限制出力變化速度', '0=不限制；1=限制', 1, 0, 1, '目前建議 0：讓控制器即時修正。舊版 250 PWM/s 會拖延加速與反向修正；3300 出力上限仍保留。'),
        'pwm_slew_rate_per_sec': Field('出力每秒最多改變', 'PWM/s', 250, 1, 1000000, '只有上一項為 1 才生效。數字越大，對控制修正的延遲越少；這不是出力上限。'),
    },
}
PRESETS = {
    'calibration': {'servo_homing_timeout_s': 60, 'hall_search_timeout_s': 60, 'stop_timeout_s': 15},
    'standing': {'position_tolerance_counts': 1000, 'rotate_timeout_s': 60, 'hall_search_timeout_s': 60},
    'tripod': {'stop_on_position_error': 0, 'max_pwm': 3300, 'enable_pwm_slew_limit': 0},
}


def current_values(site, stage):
    params = site['parameters'][NODES[stage]]
    # Missing keys mean an old executable; never pretend new limits took effect.
    return {key: params[key if key == 'max_pwm' else 'safety.' + key] for key in FIELDS[stage]}


def edit_motion_limits(runtime, ask, say, numeric):
    while True:
        say('\n到位／追蹤限制｜只存設定；下次啟動生效。供電、通訊、無效回饋與手動停止保留。')
        say('1 Calibration 校正　2 Standing 站姿　3 Tripod 步態　4 sim-to-real 設定　0 返回')
        stage = {'1': 'calibration', '2': 'standing', '3': 'tripod'}.get(choice := ask('選階段'))
        if choice.lower() in ('0', 'q', ''):
            return
        if choice == '4':
            if getattr(runtime, 'motion_limits_demo', False):
                say('示範模式不讀寫實際 sim-to-real 檔案。'); continue
            from .sim_motion_limits import edit_sim_motion_limits
            edit_sim_motion_limits(runtime, ask, say, numeric)
            continue
        if stage is None:
            say('請選 0～4。'); continue
        while True:
            site = runtime.site()
            try:
                values = current_values(site, stage)
            except KeyError:
                raise RuntimeError('目前 rinbo_legs 尚未支援新限制，請先安裝本次編譯版本。')
            say(f'\n{stage}｜設定版本 {site["revision"]}')
            keys = list(FIELDS[stage])
            for index, key in enumerate(keys, 1):
                f = FIELDS[stage][key]
                value = float(values[key])
                angle = f' ≈ {value * 360 / (54984.83 if stage == "tripod" else 55296):.2f}°' if f.unit == 'counts' else ''
                say(f'{index} {f.label}：{value:g} {f.unit}{angle}')
            say('p 套用較寬調機設定　r 還原本頁舊版數值　0 返回；或選一項自行填值')
            selected = ask('選項').lower()
            if selected in ('0', 'q', ''):
                break
            if selected == 'p':
                updates = dict(PRESETS[stage])
            elif selected == 'r':
                updates = {key: field.default for key, field in FIELDS[stage].items()}
            elif selected.isdecimal() and 1 <= int(selected) <= len(keys):
                key = keys[int(selected)-1]; field = FIELDS[stage][key]
                value = numeric(field.label + '（' + field.unit + '）', float(values[key]), field.minimum, field.maximum, explanation=field.help)
                if key in ('stop_on_position_error', 'enable_pwm_slew_limit', 'position_error_trip_samples') and value != int(value):
                    say('這一欄需要整數，沒有儲存。'); continue
                updates = {key: value}
            else:
                say('請選畫面中的編號、p、r 或 0。'); continue
            preview = runtime.tune_limits(stage, updates, True, site['revision'])
            for key, change in preview['changes'].items():
                say(f'{FIELDS[stage][key].label}：{float(change["before"]):g} → {float(change["after"]):g} {FIELDS[stage][key].unit}')
            if not preview['changed']:
                say('設定已是這個值，沒有變更。'); continue
            say('修改校正限制需重新校正；修改站姿限制需重做 Standing。既有錯誤不會被自動清除。')
            if ask('Enter 儲存；q 取消', '') != '':
                say('已取消，原設定保留。'); continue
            applied = runtime.tune_limits(stage, updates, False, site['revision'])
            after = runtime.site()
            if after['hash'] != applied['hash']:
                raise RuntimeError('儲存後設定又有變更，請重新整理；尚未啟動動作。')
            say(f'已儲存並核對，版本 {after["revision"]}。下次啟動使用新值；本次沒有送出馬達或電源命令。')
