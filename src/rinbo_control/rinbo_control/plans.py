"""Pure input validation and readable plans; C++ remains the final validator."""
import ipaddress
import copy
import json
import math
import os
import re
from pathlib import Path
import tempfile

LEGS = ('L1', 'L2', 'L3', 'R1', 'R2', 'R3')
MODES = {'relative': '小轉一下', 'position': '移到角度', 'velocity': '速度旋轉', 'cycle': '相位旋轉'}


def updated_mask(disabled, operation, names):
    """Validate a mask request without mutating preferences or site settings."""
    if operation not in ('disable', 'enable', 'enable-all'):
        raise ValueError('不支援的屏蔽操作')
    if operation == 'enable-all':
        if names:
            raise ValueError('全部解除屏蔽不接受額外腳名')
        return []
    names = selected_legs(' '.join(names), ())
    blocked = set(disabled)
    if operation == 'disable':
        blocked.update(names)
    else:
        blocked.difference_update(names)
    return [leg for leg in LEGS if leg in blocked]


def number(value, lo, hi):
    if isinstance(value, bool):
        raise ValueError('請輸入數字')
    result = float(value)
    if not math.isfinite(result) or not lo <= result <= hi:
        raise ValueError(f'數字須在 {lo:g}～{hi:g} 之間')
    return result


def ipv4(value):
    ip = ipaddress.IPv4Address(value.strip())
    if ip.is_unspecified or ip.is_multicast or ip.is_loopback or int(ip) == 0xffffffff:
        raise ValueError('請填設備的 IPv4 位址')
    return str(ip)


def selected_legs(value, disabled):
    names = value.upper().replace(',', ' ').replace('，', ' ').split()
    if not names or len(set(names)) != len(names) or any(n not in LEGS for n in names):
        raise ValueError('請輸入不重複的腳名，例如 L2 R2')
    blocked = set(names).intersection(disabled)
    if blocked:
        raise ValueError('這些腳已禁用，不能選：' + ' '.join(sorted(blocked)))
    return [n for n in LEGS if n in names]


def numbered_legs(value, disabled=()):
    words = value.upper().replace(',', ' ').replace('，', ' ').split()
    names = [LEGS[int(word)-1] if word in ('1','2','3','4','5','6') else word for word in words]
    return selected_legs(' '.join(names), disabled)


def preset_plan(legs, preset, disabled=(), current=None):
    selected = selected_legs(' '.join(legs), disabled)
    specs = {'1': dict(mode='relative', move_deg=5.0),
             '2': dict(mode='relative', move_deg=-5.0),
             '3': dict(mode='velocity', speed_deg_s=5.0),
             '4': dict(mode='velocity', speed_deg_s=-5.0)}
    if preset not in specs:
        raise ValueError('請選 1～4 的常用動作')
    plan = copy.deepcopy(validate_plan(current)) if current is not None else default_plan(disabled)
    plan['legs'] = {leg: dict(specs[preset]) for leg in selected}
    if preset in ('3', '4'):
        plan['max_speed_deg_s'] = max(plan['max_speed_deg_s'], 5.0)
    return validate_plan(plan, disabled)


def motion_pwm_cap(site):
    """Explicit Manual cap; old site files retain Standing inheritance."""
    nodes = site.get('parameters', {})
    value = nodes.get('rinbo_manual', nodes.get('rinbo_standing', {})).get('max_pwm', 80.0)
    return number(value, 1, 500)


def default_plan(disabled=()):
    leg = next((n for n in ('L2', 'R2', 'L3', 'R3', 'R1', 'L1') if n not in disabled), None)
    return dict(duration_s=3.0, max_pwm=80.0, max_speed_deg_s=10.0,
                acceleration_deg_s2=10.0,
                legs={leg: dict(mode='relative', move_deg=5.0)} if leg else {})


def adjust_plan(current, operation, disabled=(), *, source=None, targets=()):
    """Explicit whole-plan edits; no commands or changes to hardware masks."""
    plan = copy.deepcopy(validate_plan(current))
    if operation == 'reverse':
        if any(spec['mode'] == 'position' for spec in plan['legs'].values()):
            raise ValueError('「移到角度」沒有正反轉設定；請選 2 修改目標角度。原動作保留。')
        for spec in plan['legs'].values():
            key = 'move_deg' if spec['mode'] == 'relative' else 'speed_deg_s'
            spec[key] = -spec[key]
    elif operation in ('half', 'double'):
        factor = .5 if operation == 'half' else 2.0
        cap = plan['max_speed_deg_s'] * factor
        speeds = [abs(spec.get('speed_deg_s', 0) * factor) for spec in plan['legs'].values()]
        if not 1 <= cap <= 90 or any(speed > 90 for speed in speeds):
            raise ValueError('調整後會超出速度上限可填的 1～90 度／秒；請用 2 或 7 自訂，原動作保留。')
        plan['max_speed_deg_s'] = cap
        for spec in plan['legs'].values():
            if 'speed_deg_s' in spec:
                spec['speed_deg_s'] *= factor
    elif operation == 'copy':
        if source not in plan['legs']:
            raise ValueError('請選目前動作中的一隻腳作為來源。')
        selected = selected_legs(' '.join(targets), disabled)
        spec = plan['legs'][source]
        plan['legs'] = {leg: copy.deepcopy(spec) for leg in selected}
    else:
        raise ValueError('不支援這個快速調整方式。')
    return validate_plan(plan, disabled)


def quick_plan(text, current, disabled=()):
    """Edit settings only. Explicit leg commands select exactly that leg."""
    words = text.lower().split()
    plan = copy.deepcopy(current)
    settings = {'time': ('duration_s', 1, 60), 'pwm': ('max_pwm', 1, 500),
                'speed': ('max_speed_deg_s', 1, 90), 'acc': ('acceleration_deg_s2', 1, 90)}
    if len(words) == 2 and words[0] in settings:
        key, lo, hi = settings[words[0]]
        plan[key] = number(words[1], lo, hi)
    elif words and words[0].upper() in LEGS:
        leg = selected_legs(words[0], disabled)[0]
        if len(words) == 2:
            spec = dict(mode='relative', move_deg=number(words[1], -30, 30))
        elif len(words) == 3 and words[1] == 'v':
            spec = dict(mode='velocity', speed_deg_s=number(words[2], -90, 90))
        elif len(words) == 3 and words[1] == 'a':
            spec = dict(mode='position', angle_deg=number(words[2], -360, 360))
        elif len(words) == 4 and words[1] == 'p':
            spec = dict(mode='cycle', phase_deg=number(words[2], -360, 360),
                        speed_deg_s=number(words[3], -90, 90))
        else:
            raise ValueError('格式：L2 +5｜L2 v 10｜L2 a 90｜L2 p 180 10')
        # An explicit speed request also raises the plan cap enough to allow it.
        # Never changes the PWM cap, duration or hardware settings.
        if 'speed_deg_s' in spec:
            plan['max_speed_deg_s'] = max(plan['max_speed_deg_s'], abs(spec['speed_deg_s']))
        plan['legs'] = {leg: spec}
    else:
        raise ValueError('請選選單項目，或輸入 L2 +5、L2 v 10；h 查看全部寫法。')
    validate_plan(plan, disabled)
    return plan


def validate_plan(plan, disabled=()):
    if not isinstance(plan, dict) or set(plan) != {
        'duration_s', 'max_pwm', 'max_speed_deg_s', 'acceleration_deg_s2', 'legs'
    }:
        raise ValueError('動作設定格式不正確，請重新設定動作')
    for key, value in plan.items():
        if key != 'legs' and (isinstance(value, bool) or not isinstance(value, (int,float))):
            raise ValueError('動作設定的數值欄位必須是數字')
    number(plan['duration_s'], 1, 60)
    number(plan['max_pwm'], 1, 500)
    speed = number(plan['max_speed_deg_s'], 1, 90)
    number(plan['acceleration_deg_s2'], 1, 90)
    if not isinstance(plan['legs'], dict):
        raise ValueError('腳的設定格式不正確')
    if any(name not in LEGS for name in plan['legs']):
        raise ValueError('設定檔中的腳名須使用 L1～L3、R1～R3 大寫名稱')
    selected_legs(' '.join(plan['legs']), disabled)
    for leg in plan['legs'].values():
        if not isinstance(leg, dict):
            raise ValueError('腳的設定格式不正確')
        mode = leg.get('mode')
        specs = {'relative': {'move_deg': (-30, 30)}, 'position': {'angle_deg': (-360, 360)},
                 'velocity': {'speed_deg_s': (-speed, speed)},
                 'cycle': {'speed_deg_s': (-speed, speed), 'phase_deg': (-360, 360)}}
        if not isinstance(mode, str) or mode not in specs or set(leg) != {'mode', *specs[mode]}:
            raise ValueError('動作模式或欄位不正確')
        for key, bounds in specs[mode].items():
            if isinstance(leg[key], bool) or not isinstance(leg[key], (int,float)):
                raise ValueError('腳的角度／速度必須是數字')
            number(leg[key], *bounds)
    return plan


def describe(plan):
    lines = []
    for name in LEGS:
        if name not in plan['legs']:
            continue
        leg = plan['legs'][name]
        mode = leg['mode']
        if mode == 'relative':
            detail = f"從當前位置轉 {leg['move_deg']:+g}°"
        elif mode == 'position':
            detail = f"移到校正零點的 {leg['angle_deg']:g}°"
        elif mode == 'velocity':
            detail = f"每秒 {leg['speed_deg_s']:+g}°"
        else:
            detail = f"起始相位 {leg['phase_deg']:g}°，每秒 {leg['speed_deg_s']:+g}°"
        lines.append(f'  {name}：{MODES[mode]}，{detail}')
    lines.append(f"  保持／勻速時間：{plan['duration_s']:g} 秒（另加對齊、加減速與收尾）")
    lines.append(f"  逐腳動作速度上限：{plan['max_speed_deg_s']:g} 度／秒｜加速度：{plan['acceleration_deg_s2']:g} 度／秒²")
    lines.append(f"  逐腳動作出力上限：{plan['max_pwm']:g}（仍受現場上限限制）")
    return '\n'.join(lines)


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix='.save-')
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(data, f, ensure_ascii=False, allow_nan=False, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def load_settings(path):
    path = Path(path)
    if not path.exists():
        return {}
    if path.stat().st_size > 65536:
        raise ValueError('操作設定檔過大')
    value = json.loads(path.read_text())
    if not isinstance(value, dict) or set(value) - {'ip', 'jetson_ip', 'port', 'plan'}:
        raise ValueError('操作設定檔格式不正確')
    if 'ip' in value:
        ipv4(value['ip'])
    if 'jetson_ip' in value:
        ipv4(value['jetson_ip'])
    if 'port' in value:
        if type(value['port']) is not int or not 1 <= value['port'] <= 65535:
            raise ValueError('通訊埠格式不正確')
    if 'plan' in value:
        validate_plan(value['plan'])
    return value


def friendly_error(text):
    opposite = re.search(r'opposite encoder travel: ([LR][123])\b', text)
    if opposite:
        return (f'{opposite[1]} 的編碼器回讀朝預期相反方向移動，已停止。'
                '請核對馬達方向、編碼器正負號、接線及是否受外力推動；不能只當成出力太小。')
    hold = re.search(r'standing hold position lost: ([LR][123])\b', text)
    if hold:
        return f'{hold[1]} 在站立保持時已大幅偏離目標，已停止；請保留 standing 日誌核對方向與回讀。'
    if "rcl node's context is invalid" in text:
        return 'ROS 通訊在程式交接或退出時已失效，無法繼續流程。請重開更新後的控制台；若仍出現，保留本次詳細日誌。'
    pose = re.search(
        r'(final pose not reached|alignment not reached): ([LR][123]); '
        r'target_deg=([-\d.]+) actual_deg=([-\d.]+) velocity_deg_s=([-\d.]+) '
        r'pwm=([-\d.]+) cap=([-\d.]+) encoder_span_deg=([-\d.]+) peak_pwm=([-\d.]+)', text)
    if pose:
        try:
            target, actual, speed, pwm, cap, span, peak = map(float, pose.groups()[2:])
        except ValueError:
            pass  # Keep the original error category for malformed diagnostic fields.
        else:
            if all(math.isfinite(v) for v in (target, actual, speed, pwm, cap, span, peak)):
                result = (f'{pose[2]} 未到位，已停止：目標 {target:g}°，實際 {actual:g}°；'
                          f'PWM 命令 {pwm:g}／上限 {cap:g}。')
                if cap > 0 and 0 <= span < 0.1 and peak >= 0.95 * cap:
                    return (result + '\n整段編碼器變化小於 0.1°，命令出力曾達上限。'
                            '可能是起轉出力不足，也可能是驅動或回讀問題；PWM 是命令值，不是實測馬達出力。'
                            '\n若同次接線下校正確實能轉，可用選單 7 小幅提高動作出力上限後比較；'
                            '速度與時間先保持不變。')
                return result + f'\n整段編碼器變化範圍 {span:g}°；請確認是否跟不上目標或尚未停穩。'
    for token, explanation in (
        ('motor stop timeout', '已觸發零點，但尚未確認停穩'),
        ('position reset timeout', '正在等待編碼器歸零回讀，尚未確認歸零'),
        ('healthy servo homing timeout', '伺服尚未到達校正位置並保持穩定'),
    ):
        match = re.search(re.escape(token)+r': ([LR][123](?:,[LR][123])*)\b', text)
        if match:
            return f'校正未完成：{match[1]} {explanation}，已停止。這是所選腳的到位問題，不是等待其他未選中的腳。'
    hall_timeout = re.search(r'hall search timeout: ([LR][123])\b', text)
    if hall_timeout:
        leg = hall_timeout[1]
        return (f'校正未完成：{leg} 在時限內沒有收到零點感測器（Hall）的到位訊號，已觸發停止。\n'
                f'{leg} 目前被列為要校正的腳。若這隻腳本來就不使用，請先確認屏蔿名單；'
                '若要使用，請在關電後檢查感測器、磁鐵與接線。腳看起來到位仍需感測器確認。')
    rules = [
        ('Bridge discovery timed out', '等待 Bridge 來源辨識逾時，尚未開始馬達命令；請檢查重複 Bridge 或 ROS 通訊。'),
        ('Bridge discovery cancelled', '已取消來源辨識，尚未開始馬達命令。'),
        ('bridge startup requires consecutive fresh all-disabled commands', 'Bridge 已啟動：正在等待控制程式安全確認，這不是連線成功證明。'),
        ('Calibration result missing/invalid', '需要重新校正；程式會使用真正的校正完成紀錄。'),
        ('Configuration/action is busy', '另一個動作程式仍在執行，請先讓它停止並退出。'),
        ('is disabled in the site', '動作選到了禁用腳，請重新選腳。'),
        ('alignment not reached', '腳沒有到達對齊位置：請檢查負載、方向和回授，不要直接加大出力。'),
        ('final pose not reached', '收尾時腳仍未到位，已停止。'),
        ('tracking error', '腳跟不上目標，已停止；請檢查是否卡住或方向不對。'),
        ('missing/stale /motor/state', '收不到新的腳位置資料，請檢查 sbRIO、感測器電源與網路。'),
        ('handshake timeout', '控制程式未取得 Bridge 的安全確認；請檢查是否有重複 Bridge 或其他控制程式。'),
        ('software /estop asserted', '軟體急停已觸發。排除原因、確認關電後，需重新啟動 Bridge。'),
        ('control loop delayed', 'Jetson 更新控制太慢，已停止。測試時請避免大量編譯或其他重負載。'),
    ]
    return next((meaning for token, meaning in rules if token in text), text[-1500:])
