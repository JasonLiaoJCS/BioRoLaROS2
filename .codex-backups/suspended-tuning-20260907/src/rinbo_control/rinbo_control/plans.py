"""Pure input validation and readable plans; C++ remains the final validator."""
import ipaddress
import json
import math
import os
from pathlib import Path
import tempfile

LEGS = ('L1', 'L2', 'L3', 'R1', 'R2', 'R3')
MODES = {'relative': '小轉一下', 'position': '移到角度', 'velocity': '速度旋轉', 'cycle': '相位旋轉'}


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


def default_plan(disabled=()):
    leg = next((n for n in ('L2', 'R2', 'L3', 'R3', 'R1', 'L1') if n not in disabled), None)
    return dict(duration_s=3.0, max_pwm=20.0, max_speed_deg_s=10.0,
                acceleration_deg_s2=10.0,
                legs={leg: dict(mode='relative', move_deg=5.0)} if leg else {})


def validate_plan(plan, disabled=()):
    if not isinstance(plan, dict) or set(plan) != {
        'duration_s', 'max_pwm', 'max_speed_deg_s', 'acceleration_deg_s2', 'legs'
    }:
        raise ValueError('動作設定格式不正確，請重新設定動作')
    for key, value in plan.items():
        if key != 'legs' and (isinstance(value, bool) or not isinstance(value, (int,float))):
            raise ValueError('動作設定的數值欄位必須是數字')
    number(plan['duration_s'], 1, 60)
    number(plan['max_pwm'], 1, 80)
    speed = number(plan['max_speed_deg_s'], 1, 90)
    number(plan['acceleration_deg_s2'], 1, 90)
    if not isinstance(plan['legs'], dict):
        raise ValueError('腳的設定格式不正確')
    selected_legs(' '.join(plan['legs']), disabled)
    for leg in plan['legs'].values():
        if not isinstance(leg, dict):
            raise ValueError('腳的設定格式不正確')
        mode = leg.get('mode')
        specs = {'relative': {'move_deg': (-30, 30)}, 'position': {'angle_deg': (-360, 360)},
                 'velocity': {'speed_deg_s': (-speed, speed)},
                 'cycle': {'speed_deg_s': (-speed, speed), 'phase_deg': (-360, 360)}}
        if mode not in specs or set(leg) != {'mode', *specs[mode]}:
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
