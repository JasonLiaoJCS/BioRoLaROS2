"""Edit an explicitly selected future sim-to-real profile, preserving other lines."""
import hashlib
import math
import os
from pathlib import Path
import re
import tempfile
import yaml

from .motion_limits import Field

FIELDS = {
    'init_stand_timeout_s': Field('初始站姿最多等待', '秒', 12, .1, 60, '僅初始到位階段，之後由策略控制。'),
    'init_stand_position_tolerance_rad': Field('初始站姿到位容許誤差', 'rad', .12, .001, .12, '0.12 rad 約 6.88°；這是 sim-to-real 的參數，不套用 Tripod counts。'),
    'init_stand_velocity_tolerance_rad_s': Field('初始站姿停穩速度上限', 'rad/s', .25, .001, .25, '仍需低速且連續到位才能完成。'),
}
ROOTS = [Path('/home/jetson/rinbo_ros_ws/install/redrhex_rl_controller/share/redrhex_rl_controller/config'),
         Path('/home/jetson/rinbo_ros_ws/src/redrhex_rl_controller/config'), Path('/home/jetson/redrhex_site')]


def read_profile(path):
    raw = Path(path).read_bytes()
    try:
        data = yaml.safe_load(raw)
        state = data['redrhex_rl_controller']['ros__parameters']['state_machine']
        values = {key: state[key] for key in FIELDS}
    except (yaml.YAMLError, KeyError, TypeError) as exc:
        raise ValueError('此檔不是支援的 sim-to-real controller 設定，請選啟動時使用的 controller YAML。') from exc
    for key, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('無效 sim-to-real 數值：' + key)
    return raw, hashlib.sha256(raw).hexdigest(), values


def update_profile(path, updates, expected_hash, dry_run=True):
    path = Path(path)
    raw, digest, before = read_profile(path)
    if digest != expected_hash:
        raise RuntimeError('設定檔已變更，請重新讀取；沒有覆寫。')
    text = raw.decode('utf-8'); changes = {}
    for key, value in updates.items():
        if key not in FIELDS:
            raise ValueError('不支援的 sim-to-real 動作限制：' + key)
        field = FIELDS[key]
        if isinstance(value, bool) or not math.isfinite(value) or not field.minimum <= value <= field.maximum:
            raise ValueError(f'{field.label} 範圍 {field.minimum}～{field.maximum}')
        # Modify only the existing unique numeric line; no whole-YAML rewrite,
        # policy metadata edits, or accidental command/telemetry setting changes.
        pattern = re.compile(r'^(\s*' + re.escape(key) + r':\s*)[^#\r\n]+', re.M)
        if len(pattern.findall(text)) != 1:
            raise ValueError('設定欄位缺漏或不唯一：' + key)
        text = pattern.sub(lambda match: match.group(1) + f'{value:.17g} ', text)
        changes[key] = {'before': before[key], 'after': value}
    changed = any(item['before'] != item['after'] for item in changes.values())
    result = {'changed': changed, 'changes': changes, 'model_revalidation_required': changed}
    if not dry_run and changed:
        # Back up exact bytes, then replace atomically. Never rewrite model
        # metadata or claim that a changed profile passed policy preflight.
        backup = path.with_name(path.name + '.' + digest[:16] + '.bak')
        if backup.exists() and backup.read_bytes() != raw:
            raise RuntimeError('備份名稱已存在且內容不同，未修改設定。')
        if not backup.exists():
            with backup.open('xb') as handle:
                handle.write(raw); handle.flush(); os.fsync(handle.fileno())
        fd, temp = tempfile.mkstemp(prefix=path.name+'.', dir=path.parent)
        try:
            with os.fdopen(fd, 'w') as handle:
                handle.write(text); handle.flush(); os.fsync(handle.fileno())
            os.chmod(temp, path.stat().st_mode & 0o777)
            if hashlib.sha256(path.read_bytes()).hexdigest() != expected_hash:
                raise RuntimeError('儲存前設定已變更，沒有覆寫。')
            os.replace(temp, path)
            directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if os.path.exists(temp): os.unlink(temp)
        result['backup'] = str(backup)
    return result


def edit_sim_motion_limits(runtime, ask, say, numeric):
    paths = sorted({path.resolve() for root in ROOTS for path in root.glob('redrhex_policy*.yaml')})
    say('sim-to-real 使用獨立設定；先選實際啟動會使用的檔案，不推測目前是哪個 profile。')
    for index, path in enumerate(paths, 1): say(f'{index} {path}')
    choice = ask('設定檔編號，或貼上啟動指令使用的絕對 YAML 路徑；0 返回')
    if Path(choice).is_absolute():
        path = Path(choice).resolve(strict=True)
    elif choice.isdecimal() and 1 <= int(choice) <= len(paths):
        path = paths[int(choice)-1]
    else:
        return
    while True:
        _, digest, values = read_profile(path)
        say(str(path))
        keys = list(FIELDS)
        for index, key in enumerate(keys, 1):
            f = FIELDS[key]; say(f'{index} {f.label}：{values[key]:g} {f.unit}')
        choice = ask('選欄位；p 延長到位等待到 60 秒；0 返回')
        if choice in ('0', '', 'q'): return
        if choice == 'p': updates = {'init_stand_timeout_s': 60}
        elif choice.isdecimal() and 1 <= int(choice) <= len(keys):
            key = keys[int(choice)-1]; f = FIELDS[key]
            updates = {key: numeric(f.label, values[key], f.minimum, f.maximum, explanation=f.help)}
        else: continue
        preview = update_profile(path, updates, digest)
        for key, item in preview['changes'].items(): say(f'{FIELDS[key].label}：{item["before"]} → {item["after"]}')
        if not preview['changed']: say('設定未改變。'); continue
        say('此檔與模型驗證雜湊綁定。修改後需重新封裝／preflight，不能把舊模型驗證當作仍有效。')
        if ask('Enter 儲存並保留備份；q 取消', '') != '': continue
        checked = runtime.query('rinbo_fsm', 'rinbo_legs', 'assert-idle')
        if checked.returncode: raise RuntimeError('請先停止控制程序：' + checked.stderr)
        result = update_profile(path, updates, digest, False)
        _, _, after = read_profile(path)
        if any(after[key] != value for key, value in updates.items()):
            raise RuntimeError('儲存後讀回不同，請重新整理。')
        say('已儲存供下次啟動使用；未啟動動作。備份：' + result['backup'])
