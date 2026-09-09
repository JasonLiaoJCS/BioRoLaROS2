"""Explicit interface rehearsal: no ROS, sockets, subprocesses or receipts."""
from pathlib import Path
import json
import time
from .plans import LEGS, atomic_json, updated_mask, validate_plan
from .runtime import Cancelled


class DemoRuntime:
    def __init__(self, state_dir, progress=print, cancel=lambda: False):
        self.log_dir = Path(state_dir)/'demo'
        self.progress, self.cancel = progress, cancel
        self.connected = False
        self.calibrated = False
        self.calibration_reason = '尚未檢查校正。'
        self.calibrated_legs = set()
        self.sensors = False
        self.disabled = ['L1']
        self.revision = 1
        from .motion_limits import FIELDS, NODES
        self.parameters = {NODES[stage]: {(key if key == 'max_pwm' else 'safety.'+key): field.default for key,field in fields.items()}
                           for stage,fields in FIELDS.items()}
        self.motion_limits_demo = True

    def check(self):
        return self.site()

    def site(self):
        return {'revision': self.revision, 'parameters': self.parameters, 'disabled_legs': list(self.disabled),
                'enabled_legs': [leg for leg in LEGS if leg not in self.disabled],
                'hash': f'DEMO-{self.revision}'}

    def tune_limits(self, stage, updates, dry_run, revision):
        from .motion_limits import NODES
        if revision != self.revision: raise RuntimeError('設定版本已改變')
        parameters = self.parameters[NODES[stage]]
        changes = {key: {'before': parameters[(key if key == 'max_pwm' else 'safety.'+key)], 'after': value} for key,value in updates.items()}
        changed = any(item['before'] != item['after'] for item in changes.values())
        if changed and not dry_run:
            parameters.update({(key if key == 'max_pwm' else 'safety.'+key): value for key,value in updates.items()})
            self.revision += 1
        return {'changes': changes, 'changed': changed, 'hash': self.site()['hash']}

    def change_legs(self, operation, names=()):
        desired = updated_mask(self.disabled, operation, names)
        if desired != self.disabled:
            if self.connected:
                self.safe_off()
            self.disabled = desired
            self.calibrated = False
            self.calibrated_legs = set()
            self.revision += 1
            self.progress('【介面演練】屏蔿名單已更新；沒有修改實體機器人的設定。')
        else:
            self.progress('名單已經是這個狀態，沒有變更。')
        return self.site()

    def connect(self, *args):
        self.connected = True
        self.progress('【介面演練】模擬 sbRIO 雜湊檢查 → core → FPGA 成功 → Bridge → 新回讀；沒有連線設備。')
        return self.status()

    def reconnect(self, *args):
        self.progress('【介面演練】模擬停止舊動作、整理過期紀錄並沿用通訊；沒有操作設備。')
        return self.connect(*args)

    def status(self):
        return {'bridge_count': int(self.connected), 'writers': [], 'motor_fresh': self.connected,
                'power_fresh': self.connected, 'sensors_on': self.sensors,
                'relay_on': False if self.connected else None, 'voltage': 24.0 if self.connected else None,
                'legs': {leg: dict(angle_deg=0.0, hall=False) for leg in LEGS} if self.connected else {}}

    def panel_status(self):
        return self.status() if self.connected else None

    def ensure_connected(self, *args):
        return self.status() if self.connected else self.connect(*args)

    def save_plan(self, plan, site):
        validate_plan(plan, site['disabled_legs'])
        path = self.log_dir/'demo-plan.json'
        atomic_json(path, plan)
        return path

    def needs_calibration(self, path):
        selected = set(json.loads(Path(path).read_text())['legs'])
        needed = not (self.calibrated and selected <= self.calibrated_legs)
        self.calibration_reason = ('演練：所選腳尚無有效校正，將只校正本次選中的腳。' if needed else
                                   '演練：所選腳已有有效校正，不重新尋零。')
        return needed

    def execute(self, path, site_hash, calibrate):
        legs = set(json.loads(Path(path).read_text())['legs'])
        for step in ['【2／5 電源】模擬 Off → Digital → Signal → 馬達停用確認 → Relay 回讀',
                     '【3／5 校正】'+(('模擬只校正：'+' '.join(sorted(legs))) if calibrate else '模擬沿用校正'),
                     '【4／5 動作】模擬執行所選腳的動作', '【5／5 收尾】模擬關閉馬達電源']:
            if self.cancel():
                raise Cancelled('介面演練已停止')
            self.progress('【介面演練】'+step)
            time.sleep(.05)
        self.calibrated = self.sensors = True
        if calibrate:
            self.calibrated_legs = legs

    def safe_off(self):
        self.calibrated = self.sensors = False
        self.calibrated_legs = set()
        self.progress('【介面演練】模擬全部關電')
        return True

    def close(self):
        return True
