"""Readable terminal snapshots, with no ROS or device access."""
from datetime import datetime
import math

from .plans import LEGS, describe

POSITIONS = ('左前', '左中', '左後', '右前', '右中', '右後')


def leg_map(site, selected=()):
    lines = ['                    機頭方向 ↑       * 表示本次選中']
    def cell(index):
        leg = LEGS[index]
        state = '屏蔽' if leg in site['disabled_legs'] else '可用'
        return f'{index+1} {leg} {POSITIONS[index]} [{state}] '+('*' if leg in selected else ' ')
    for left in range(3):
        lines.append('  '+cell(left)+'      '+cell(left+3))
    return '\n'.join(lines)


def leg_feedback(snapshot):
    lines = ['\n腳的回讀｜角度是編碼器目前回報值；完成校正後才以校正零點解讀。',
             '  腳／位置       角度（度）    零點感測器（Hall）']
    if snapshot.get('sensors_on') is False:
        lines.insert(1, '感測器未確認上電：以下僅為控制器回報，不代表有效的實體位置。')
    values = snapshot.get('legs', {}) if snapshot.get('motor_fresh') else {}
    for leg, position in zip(LEGS, POSITIONS):
        value = values.get(leg, {})
        angle = value.get('angle_deg')
        shown = f'{angle:+.1f}' if angle is not None and math.isfinite(angle) else '未知'
        # The production calibration FSM detects Hall on !hall_effect (low).
        hall = {None:'未知', True:'未觸發（1）', False:'已觸發（0）'}[value.get('hall')]
        lines.append(f'  {leg} {position}       {shown:>8}      {hall}')
    lines.append('Hall=0 表示觸發零點；還需停穩與位置歸零回讀，才算校正完成。')
    lines.append('目前回讀沒有直接提供速度；設定速度可在主畫面的「本次動作」查看。')
    return '\n'.join(lines)


def execution_scope(plan, site, calibrate, reason):
    selected = [leg for leg in LEGS if leg in plan['legs']]
    others = [leg for leg in LEGS if leg not in selected]
    lines = ['\n【本次執行範圍】', '動作主馬達：'+' '.join(selected),
             '本次校正：'+('只校正 '+' '.join(selected) if calibrate else '不需要，沿用所選腳的有效校正'),
             '原因：'+reason,
             '不參與的主馬達：'+(' '.join(others) or '無'),
             '共用設定屏蔽：'+(' '.join(site['disabled_legs']) or '無')]
    if calibrate:
        lines.append('完成條件：只等待所選腳的伺服到位、零點觸發、停穩及歸零回讀。')
        cap = site.get('parameters', {}).get('rinbo_cali', {}).get('max_pwm')
        if isinstance(cap, (int, float)) and not isinstance(cap, bool) and math.isfinite(cap):
            lines.append(f'校正出力上限：{cap:g}；校正後逐腳動作出力上限：{plan["max_pwm"]:g}（另受現場上限限制）。')
        lines.append('校正速度／出力依現場校正設定；未選中伺服保持回讀位置，伺服電源仍共用。')
    lines.append('共用供電與通訊仍須正常；正常結束關馬達電源、保留感測器供電。')
    return '\n'.join(lines)


def dashboard(site, plan, snapshot, last_result, repeat=False, demo=False, target=None, workshop=False):
    title = '介面演練（不連線、不上電）' if demo else 'Rinbo 機器人控制台'
    lines = ['\n'+'─'*58, f'{title}  ｜更新 {datetime.now():%H:%M:%S}']
    if target:
        lines.append('設定目標：'+target)
    lines.append('操作模式：'+('現場調機｜按 3 直接開始（含需要的上電／校正）' if workshop else
                              '一般｜新動作先確認；選 14 可開啟現場調機'))
    if snapshot is None:
        lines.append('連線／電源：尚未檢查（選 1 檢查；開啟選單不會上電）')
    else:
        count = snapshot['bridge_count']
        connected = count == 1 and snapshot['motor_fresh'] and snapshot['power_fresh']
        link = '正常' if connected else ('未連線' if count == 0 else '需要檢查回讀／重複連線')
        relay = {None:'未知', True:'開啟', False:'關閉'}[snapshot['relay_on']]
        voltage = '未知' if snapshot['voltage'] is None else f'{snapshot["voltage"]:.1f} V'
        lines.append(f'連線：{link}　｜馬達電源：{relay}　｜電壓：{voltage}')
        if snapshot.get('writers'):
            lines.append('控制程式正在發布命令：'+', '.join(snapshot['writers']))
    blocked = []
    if site is None:
        lines.append('腳的設定：讀取失敗，狀態未知')
    else:
        lines.append(leg_map(site, plan['legs']))
        blocked = sorted(set(plan['legs']).intersection(site['disabled_legs']))
        if blocked:
            lines.append('目前動作中的 '+' '.join(blocked)+' 已屏蔽，不能執行。')
    lines.extend(['\n【本次動作】', describe(plan)])
    if last_result:
        lines.append('最近結果：'+last_result.splitlines()[0][:140])
    if site is None:
        next_step = '選 5 重新檢查；需要關電選 4。'
    elif not site['enabled_legs']:
        next_step = '六隻腳皆已屏蔽；先選 8，解除要使用的腳。'
    elif blocked:
        next_step = '選 2 換腳，或選 8 解除屏蔽。'
    elif snapshot and snapshot.get('writers'):
        next_step = '已有控制程式正在執行；選 5 查看，需停止供電選 4。'
    elif snapshot and snapshot['bridge_count'] > 1:
        next_step = '偵測到重複 Bridge；選 5 查看，先關閉多開的通訊程式。'
    elif snapshot is None or not (snapshot['motor_fresh'] and snapshot['power_fresh']):
        next_step = '選 1 檢查連線；也可先用 2 或 9 設定動作。'
    else:
        next_step = '選 3 '+('重做同一動作；選 2 修改。' if repeat else '執行；想簡單試動可選 9 常用範例。')
    lines.extend(['下一步：'+next_step,
                  '\n【操作】1 連線檢查　2 選腳與動作　3 '+('再做一次' if repeat else '執行動作'),
                  '【管理】4 全部關電　5 角度／零點／狀態　8 屏蔽／解除腳',
                  '【設定】6 網路位址　7 出力／加速度／速度上限',
                  '【入門】9 常用動作　h 使用說明　　0 關電並離開',
                  '【常用】10 我的動作　11 快速調整　12 復原上次設定',
                  '【調機】13 出力快調　14 現場模式（按 3 直接開始）　15 到位／追蹤限制',
                  'Enter 更新畫面；設定中 q 取消；校正／動作中 Q 直接停止（不用 Enter）。'])
    return '\n'.join(lines)
