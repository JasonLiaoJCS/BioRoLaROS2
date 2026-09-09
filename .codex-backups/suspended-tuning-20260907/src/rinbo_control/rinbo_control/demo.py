"""Explicit interface rehearsal: no ROS, sockets, subprocesses or receipts."""
from pathlib import Path
import time
from .plans import atomic_json, validate_plan
from .runtime import Cancelled


class DemoRuntime:
    def __init__(self, state_dir, progress=print, cancel=lambda: False):
        self.log_dir = Path(state_dir)/'demo'
        self.progress, self.cancel = progress, cancel
        self.connected = False
        self.calibrated = False
        self.sensors = False

    def check(self):
        return self.site()

    def site(self):
        return {'disabled_legs': ['L1'], 'enabled_legs': ['L2','L3','R1','R2','R3'], 'hash': 'DEMO'}

    def connect(self, *args):
        self.connected = True
        self.progress('【介面演練】模擬 sbRIO 雜湊檢查 → core → FPGA 成功 → Bridge → 新回讀；沒有連線設備。')
        return self.status()

    def status(self):
        return {'bridge_count': int(self.connected), 'writers': [], 'motor_fresh': self.connected,
                'power_fresh': self.connected, 'sensors_on': self.sensors,
                'relay_on': False if self.connected else None, 'voltage': 24.0 if self.connected else None}

    def save_plan(self, plan, site):
        validate_plan(plan, site['disabled_legs'])
        path = self.log_dir/'demo-plan.json'
        atomic_json(path, plan)
        return path

    def needs_calibration(self, path):
        return not self.calibrated

    def execute(self, path, site_hash, calibrate):
        for step in ['模擬 Off → Digital → Signal → 馬達停用確認 → Relay 回讀', '模擬校正' if calibrate else '模擬沿用校正',
                     '模擬執行所選腳的動作', '模擬關閉馬達電源']:
            if self.cancel():
                raise Cancelled('介面演練已停止')
            self.progress('【介面演練】'+step)
            time.sleep(.05)
        self.calibrated = self.sensors = True

    def safe_off(self):
        self.calibrated = self.sensors = False
        self.progress('【介面演練】模擬全部關電')
        return True

    def close(self):
        return True
