"""Display controller-reported progress only; never authorizes motion."""
import re

from .plans import LEGS


class CalibrationProgress:
    def __init__(self):
        self.skipped = set()
        self.done = set()
        self.phases = {}

    def consume(self, line):
        event = re.search(r'\b([LR][123]): (SKIPPED|Hall detected!|RESETTING|DONE!)', line)
        if event:
            leg, stage = event.groups()
            if stage == 'SKIPPED':
                self.skipped.add(leg)
                reason = '本次未選中' if 'not selected' in line else '已屏蔽'
                return f'{leg}：{reason}，跳過主馬達校正，不等待這隻腳。'
            if stage == 'DONE!':
                self.done.add(leg)
                return f'{leg}：校正完成，已收到零點位置回讀。'
            self.phases[leg] = ('等待停穩' if stage == 'Hall detected!' else '等待位置歸零回讀')
            return f'{leg}：{self.phases[leg]}。'
        if 'DC_SPINNING |' in line:
            counts = re.search(r'Healthy complete: (\d+)/(\d+) \| SKIPPED: (\d+)', line)
            elapsed = re.search(r'\bt: ([\d.]+)', line)
            if not counts:
                return '校正中：正在尋找零點；進度格式無法辨識，請查看詳細日誌。'
            completed, total, skipped = map(int, counts.groups())
            message = f'校正中：已完成 {completed}／{total} 隻；已跳過 {skipped} 隻'
            # The UI queue is bounded. If events were lost, report the counts
            # from the controller without guessing which legs remain.
            if (len(self.skipped) == skipped and total + skipped == len(LEGS)
                    and len(self.done) == completed):
                pending = [f'{leg}（{self.phases.get(leg, "尋找零點訊號")}）'
                           for leg in LEGS if leg not in self.skipped | self.done]
                if pending:
                    message += '；仍在等：' + '、'.join(pending)
            if elapsed:
                message += f'；尋零已過 {float(elapsed[1]):.1f} 秒'
            return message
        if 'WAIT_SERVO |' in line:
            return '校正中：正在定位伺服，完成後才開始尋找主馬達零點。'
        return None
