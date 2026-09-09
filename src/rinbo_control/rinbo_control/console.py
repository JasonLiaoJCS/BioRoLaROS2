"""One terminal, plain Chinese, explicit review before powering or moving."""
import argparse
import copy
import fcntl
import os
from pathlib import Path
import signal
import sys
import tempfile

from .plans import (LEGS, MODES, adjust_plan, atomic_json, default_plan, describe, friendly_error,
                    ipv4, load_settings, motion_pwm_cap, number, numbered_legs, preset_plan,
                    quick_plan, validate_plan)
from .runtime import (DEFAULT_JETSON_IP, DEFAULT_SBRIO_IP, Cancelled, Runtime, discover)
from .terminal import stop_keys
from .presentation import dashboard, leg_map, leg_feedback, execution_scope
from .library import load_library, save_library, validate_name


class ConsoleExit(BaseException):
    pass


def say(text):
    print(text, flush=True)


def ask(prompt, default=None):
    shown = f'{default:g}' if isinstance(default, (int, float)) else str(default)
    suffix = f' [直接 Enter 用 {shown}]' if default is not None else ''
    value = input(prompt+suffix+'：').strip()
    return value if value or default is None else str(default)


def setting_input(prompt, default=None):
    value = ask(prompt, default)
    if value.lower() == 'q':
        raise Cancelled('已取消設定')
    return value


def numeric(prompt, default, lo, hi, *, suggestion=None, explanation=None):
    if explanation:
        say(explanation)
    if suggestion:
        say('入門建議：'+suggestion)
    say(f'範圍 {lo:g}～{hi:g}；q 取消這次設定。')
    while True:
        try:
            value = setting_input(prompt, default)
            return number(value, lo, hi)
        except ValueError:
            say(f'請填 {lo:g}～{hi:g} 之間的數字，例如 {default:g}；不用加單位。')


def edit_network(ip, jetson, port, demo=False):
    say('連得上就直接 Enter 保留。q 取消整次修改；? 搜尋 sbRIO 候選位址。')
    values = []
    for label, default in (('sbRIO IP', ip), ('Jetson 有線 IP', jetson)):
        while True:
            value = setting_input(label, default)
            if value == '?' and label == 'sbRIO IP':
                matches = [] if demo else discover(port)
                say('可連通的候選位址：'+(', '.join(matches) or '沒有找到'))
                say('請依設備標籤選自己的 sbRIO；搜尋不會自動更換位址。')
                continue
            try:
                values.append(ipv4(value))
                break
            except ValueError:
                say(f'位址格式不正確，例如 {default}；只重填這一欄。')
    while True:
        value = setting_input('sbRIO 通訊埠', port)
        if value.isascii() and value.isdecimal() and 1 <= int(value) <= 65535:
            return (*values, int(value))
        say('通訊埠請填 1～65535 的整數，通常是 50051；前面填的位址仍保留。')


def edit_limits(plan, site=None):
    updated = copy.deepcopy(plan)
    say('這裡只調整校正後的逐腳動作。q 取消整次修改；Enter 保留目前值。')
    if site is not None:
        cap = motion_pwm_cap(site)
        say(f'目前現場 Manual 上限：{cap:g}；計畫與現場上限取較小值。調整計畫不會改動現場控制參數。')
    updated['max_pwm'] = numeric('逐腳動作出力上限', plan['max_pwm'], 1, 500,
        suggestion='可填 1～500；實際出力也受現場 Manual 上限限制。選 13 可查看並採用現場上限。',
        explanation='這是允許的驅動出力上限；慢轉也需要足夠出力。它不是轉速、伏特或百分比。')
    updated['acceleration_deg_s2'] = numeric('加速度（度／秒²）', plan['acceleration_deg_s2'], 1, 90,
        suggestion='初次試動可用 10。',
        explanation='填 10，目標速度從 0 增加到 10 度／秒約需 1 秒。')
    minimum = max(1, *(abs(spec.get('speed_deg_s', 0)) for spec in plan['legs'].values()))
    updated['max_speed_deg_s'] = numeric('速度上限（度／秒）', plan['max_speed_deg_s'], minimum, 90,
        suggestion=f'目前動作至少需要 {minimum:g}；初次小角度試動可用 10。',
        explanation='這是速度的上限；要改旋轉速度本身，請用選單 2。')
    # Limits remain editable while a selected leg is masked; actual execution
    # still validates against the current site mask.
    return validate_plan(updated)


def quick_power(plan, site):
    cap = motion_pwm_cap(site)
    current = plan['max_pwm']
    say(f'\n出力快調｜目前設定 {current:g}；現場動作上限 {cap:g}。速度、時間與其他設定保留。')
    say('1 加 10　2 減 10　3 自行填值　4 採用現場上限　0 返回')
    choice = choose('要怎麼調整', ('1', '2', '3', '4', '0'))
    if choice == '0':
        raise Cancelled('已取消出力調整')
    if choice in ('1', '2'):
        value = current + (10 if choice == '1' else -10)
        if not 1 <= value <= cap:
            raise ValueError(f'調整後為 {value:g}，超出現場範圍 1～{cap:g}；原設定保留，可選 13 → 3 自行填值。')
    elif choice == '3':
        value = numeric('出力上限', min(current, cap), 1, cap)
    else:
        value = cap
    updated = copy.deepcopy(plan)
    updated['max_pwm'] = value
    return validate_plan(updated)


def beginner_help():
    say('''
第一次使用：照 1 → 2 → 3 做。
開啟方式：在 Jetson 的 /home/jetson/rinbo_ros_ws 執行 ./robot.sh。
簡明教學：/home/jetson/rinbo_ros_ws/docs/control_panel_zh_TW.md

1 自動整理並接通機器人：讓這台 Jetson 電腦能讀到腳的位置。
  先正常停止可辨識的舊動作、核對停止回讀，再登入 sbRIO 整理過期紀錄。
  沿用正確的現有通訊，缺少服務才依既有流程啟動；不會上電或啟動新動作。
  已設定密碼時會自動登入。登入電腦本身不會操作機器人，也不關閉其他 SSH。
2 設定動作：用編號選腳 → 設定每隻腳的動作 → 填時間。q 可取消，原設定保留。
3 執行：第一次或更改設定後，輸入 1 確認；同設定成功後再選 3 就直接重做。
  若你在現場連續調機，選 14 → 1，之後按 3 直接開始，不再多問一次。
  只校正這次選中的腳。選 L2 就不等 L3，屏蔽腳仍不能選。
  校正後才執行你選的腳。你填的速度／出力限制用於校正後的逐腳動作。
  換到尚未校正的腳，或感測器供電／回讀中斷，才會需要重新校正。

主畫面：六隻腳按位置排列，* 是本次選中的腳；上方是連線／電源狀態。
  回到主畫面時更新，也可直接 Enter 重新整理，不會執行動作。
9 常用動作：選腳 → 選 +5°、-5°、慢速正轉或反轉。只改設定，選 3 才執行。
  範例保留你調好的時間、出力和加速度；速度範例只在上限不足 5 時提高到 5。
7 進階限制：出力、加速度及速度上限集中在這裡，通常不用每次重填。

10 我的動作：把目前設定取名收藏，下次按編號選用；支援刪除收藏。
  收藏包含腳、動作、時間及限制。已屏蔽的腳不能直接載入執行。
11 快速調整：反轉方向、半速、兩倍速，或把一隻腳的動作複製到其他腳。
  反轉不保證回到上次起點；速度調整保留出力與加速度。
12 復原上次設定：本次開啟後最多可退回 20 次；只改設定，不讓腳倒退。
  不會復原屏蔿名單、電源或校正。載入與復原後仍選 3 確認執行。
13 出力快調：加 10、減 10、自行填值，或直接採用現場動作上限。
  只改出力，不必重填速度／加速度；按 3 才試動。
15 到位／追蹤限制：調整 Calibration、Standing、Tripod 的誤差與等待時間。
  可逐項填數字、套用較寬設定或還原；只改設定，不啟動動作。
14 現場模式：開啟後，按 3 直接開始本次動作及所需校正。
  關電、執行失敗／取消、修改現場設定或重開後，會回到一般模式。

快速設定（在主選單直接輸入；只改設定，選 3 才會動）：
  L2 +5        只選 L2，從現在再轉 5 度
  L2 -5        只選 L2，從現在反轉 5 度
  L2 v 10      只選 L2，每秒轉 10 度；速度上限不足時一併提高
  L2 a 90      只選 L2，移到校正零點的 90 度
  L2 p 180 10  只選 L2，先對齊 180 度，再每秒轉 10 度
  time 5       保持／勻速 5 秒
  pwm 30       逐腳動作出力上限 30
  speed 20     速度上限 20 度／秒
  acc 20       加速度 20 度／秒²
  多隻腳不同動作，仍用選單 2。這些指令不更改共用屏蔿名單。

8 腳的屏蔽管理：查看六隻腳、屏蔽、解除屏蔽，全部用編號操作。
  1=L1、2=L2、3=L3、4=R1、5=R2、6=R3；可一次輸入多個編號。
  修改後立即儲存到共用設定，重開仍保留。已連線時會先自動關電。
  解除屏蔽只恢復可選狀態，不會自行上電或轉動。

不知道怎麼填，可以回選單輸入 r，套用入門範例：
  只選目前可用的一隻腳 → 從現在的位置小轉 +5°。
  保持 3 秒、速度上限 10°／秒、出力上限 80、加速度 10°／秒²。
  這些是程式的入門起點，實際是否到位仍由回讀檢查。

四種動作怎麼選：
  小轉一下：從現在轉幾度。先用 +5；反方向填 -5。
  移到角度：以校正零點為基準，0 是回到零點。先了解零點再使用。
  按速度旋轉：先用 5°／秒，轉一整圈約 72 秒；負數是反方向。
  按相位旋轉：先對齊再一起轉。0 與 0 同相，0 與 180 相差半圈。
    想維持固定相位差，各腳速度要相同；對齊時可能先轉半圈。

填數字時：直接 Enter 沿用畫面顯示的值，可能是你上次儲存的設定。
  「入門建議」是參考起點；「可填範圍」只是程式允許的範圍。
  第一次先跳過選單 6、7。網路位址已填好，出力與加速度也有預設。

怎麼停：校正／動作中按 Q、空白鍵、Esc 或 Ctrl+C，會停止並嘗試關電，不用 Enter。
  選 4：關電、保留連線，留在選單。選 0：關電並結束本次操作。
  正常動作結束會關馬達電源，保留感測器供電，方便再調整。
''')


def stop_requested():
    return stop_keys.requested()


def choose(prompt, choices, default=None):
    while True:
        value = ask(prompt+'（q 取消）', default).lower()
        if value == 'q':
            raise Cancelled('已取消設定')
        if value in choices:
            return value
        say('請選 '+ '、'.join(choices)+'；前面填的內容仍保留。')


def pick_legs(current, site):
    if not site['enabled_legs']:
        raise ValueError('六隻腳皆已屏蔽；先選 8 解除要使用的腳。')
    say(leg_map(site, current['legs']))
    say('填腳的編號；例如 2 是 L2，2 5 是 L2 與 R2。q 取消。')
    selected = list(current['legs'])
    default = (' '.join(str(LEGS.index(leg)+1) for leg in selected)
               if selected and not set(selected).intersection(site['disabled_legs']) else None)
    while True:
        value = ask('要使用哪些腳', default)
        if value.lower() == 'q' or not value:
            raise Cancelled('已取消選腳')
        try:
            return numbered_legs(value, site['disabled_legs'])
        except ValueError as exc:
            say(str(exc)+'；請重新選編號。')


def choose_preset(current, site):
    say('\n常用動作｜選腳 → 選範例。現在只改設定。')
    legs = pick_legs(current, site)
    say('1 小轉 +5°　2 小轉 -5°　3 慢速正轉 5°／秒　4 慢速反轉 5°／秒')
    say(f'沿用時間 {current["duration_s"]:g} 秒、出力 {current["max_pwm"]:g}、加速度 {current["acceleration_deg_s2"]:g}。不會重設出力。')
    preset = choose('套用哪個範例', ('1','2','3','4'), '1')
    return preset_plan(legs, preset, site['disabled_legs'], current=current)


def quick_adjust(current, site):
    say('\n快速調整｜套用後先看主畫面，選 3 才執行。')
    say('1 反轉方向　2 半速　3 兩倍速　4 把某腳的動作複製到其他腳　0 返回')
    choice = choose('要怎麼調整', ('1', '2', '3', '4', '0'))
    if choice == '0':
        raise Cancelled('已取消快速調整')
    if choice == '4':
        sources = [leg for leg in LEGS if leg in current['legs']]
        source = sources[0]
        if len(sources) > 1:
            say('目前各腳動作：\n'+describe(current))
            say('來源編號：'+'、'.join(f'{LEGS.index(leg)+1}={leg}' for leg in sources))
            value = choose('要複製哪一隻腳的動作', tuple(str(LEGS.index(leg)+1) for leg in sources))
            source = LEGS[int(value)-1]
        say(f'複製 {source} 的動作；本次將只控制接著選中的腳。時間、出力與限制沿用。')
        targets = pick_legs(current, site)
        return adjust_plan(current, 'copy', site['disabled_legs'], source=source, targets=targets)
    if choice == '1':
        say('小轉角度／旋轉速度改正負號；相位起點不變。這不保證回到上次起點。')
    else:
        say('旋轉速度與移動速度上限一起調整；角度、相位、保持／勻速時間、出力與加速度保留。')
        say('加減速仍依原設定，所以整段動作時間不一定變成一半或兩倍。')
    return adjust_plan(current, {'1':'reverse', '2':'half', '3':'double'}[choice], site['disabled_legs'])


def motion_library(path, current, site):
    while True:
        try:
            entries = load_library(path)
        except (ValueError, OSError) as exc:
            raise RuntimeError(f'無法讀取我的動作，原收藏檔保留：{path}。原因：{exc}') from exc
        say(f'\n我的動作｜已存 {len(entries)}／30 組；重開控制台仍保留。')
        say('1 收藏目前動作　2 選用已存動作　3 刪除收藏　0 返回')
        say('收藏包含腳、動作、時間及限制；不儲存屏蔿名單、校正或上電狀態。')
        try:
            choice = choose('要做什麼', ('1', '2', '3', '0'))
            if choice == '0':
                return None
            if choice == '1':
                say('準備收藏：\n'+describe(current))
                while True:
                    name = setting_input('取個名字，例如 L2 慢速正轉（q 返回）')
                    try:
                        validate_name(name)
                        break
                    except ValueError as exc:
                        say(str(exc))
                if name in entries:
                    say('同名收藏原本是：\n'+describe(entries[name]))
                    if ask('輸入 1 覆蓋；Enter 取消') != '1':
                        continue
                entries[name] = copy.deepcopy(current)
                save_library(path, entries)
                say(f'已收藏「{name}」，尚未執行。')
                continue
            if not entries:
                say('還沒有收藏，先選 1 收藏目前動作。')
                continue
            names = list(entries)
            for index, name in enumerate(names, 1):
                saved = entries[name]
                blocked = set(saved['legs']).intersection(site['disabled_legs'])
                note = '｜已屏蔽：'+' '.join(sorted(blocked)) if blocked else ''
                say(f'{index} {name}｜'+ ' '.join(saved['legs'])+note)
            number = choose('選收藏編號，0 返回', tuple(str(i) for i in range(len(names)+1)))
            if number == '0':
                continue
            name = names[int(number)-1]
            say(f'「{name}」內容：\n'+describe(entries[name]))
            if choice == '2':
                candidate = copy.deepcopy(entries[name])
                validate_plan(candidate, site['disabled_legs'])
                return candidate
            if ask(f'輸入 1 刪除「{name}」；Enter 取消') == '1':
                del entries[name]
                save_library(path, entries)
                say('收藏已刪除；主畫面的動作設定保留。')
        except Cancelled:
            return None
        except (ValueError, OSError) as exc:
            say('未完成：'+str(exc))


def edit_plan(current, site):
    plan = copy.deepcopy(current)
    say('\n設定動作 1／3｜先選腳；q 可取消，原設定不變。')
    legs = pick_legs(plan, site)
    new = {}
    modes = list(MODES)
    for name in legs:
        old = plan['legs'].get(name, {'mode': 'relative', 'move_deg': 5.0})
        say(f'\n設定動作 2／3｜{name} 要怎麼動？')
        say('  1 小轉一下（第一次建議）　2 移到指定角度\n  3 按速度旋轉　　　　　　4 先對齊相位再旋轉')
        choice = choose('動作種類', ('1','2','3','4'), modes.index(old['mode'])+1)
        mode = modes[int(choice)-1]
        spec = {'mode': mode}
        if mode == 'relative':
            spec['move_deg'] = numeric('從現在轉幾度', old.get('move_deg',5), -30, 30,
                suggestion='先用 +5 度；要反方向就填 -5。',
                explanation='例如現在在 170 度，填 5 就以 175 度為目標。正負號表示相反的轉向。')
        elif mode == 'position':
            spec['angle_deg'] = numeric('目標角度（相對校正零點，度）', old.get('angle_deg',5), -360, 360,
                suggestion='已了解校正零點時可先用 5 度；只想小幅試動，建議用「小轉一下」。',
                explanation='0 度是校正零點。填 5 是移到零點旁的 5 度，不是從現在只轉 5 度；可能有大幅移動。')
        else:
            spec['speed_deg_s'] = numeric('每秒轉幾度？負數是反方向',
                old.get('speed_deg_s',5), -90, 90,
                suggestion='先用 5 度／秒；負數往反方向轉，0 不持續旋轉。',
                explanation='每秒 5 度，轉一圈約 72 秒。若指定速度超過原速度上限，會一併提高上限。')
            plan['max_speed_deg_s'] = max(plan['max_speed_deg_s'], abs(spec['speed_deg_s']))
            if mode == 'cycle':
                spec['phase_deg'] = numeric('起始相位（度）', old.get('phase_deg',0), -360, 360,
                    suggestion='單腳先用 0；兩腳同相用 0、0，半圈錯開用 0、180。',
                    explanation='相位就是開始旋轉前先對齊的角度；對齊可能轉半圈。\n要維持相位差，兩腳速度必須相同。')
        new[name] = spec
    plan['legs'] = new
    say('\n設定動作 3／3｜最後填時間。')
    plan['duration_s'] = numeric('保持／勻速時間（秒）', plan['duration_s'], 1, 60,
        suggestion='先用 3 秒。',
        explanation='定點：到位後保持多久；旋轉：固定速度轉多久。另加對齊及加減速時間。')
    validate_plan(plan, site['disabled_legs'])
    say('設定完成；出力與加速度沿用畫面中的值，需要調整可選主選單 7。')
    return plan


def print_status(status):
    bridge = '未啟動' if not status['bridge_count'] else ('已啟動' if status['bridge_count']==1 else '重複！')
    relay = {None:'未知（沒有新回讀）', True:'已開啟', False:'已關閉'}[status['relay_on']]
    say(f"連線程式（Bridge）：{bridge}｜腳位置：{'有新資料' if status['motor_fresh'] else '沒有新資料'}｜馬達電源：{relay}")
    if status['voltage'] is not None:
        say(f"量到的電壓：{status['voltage']:.1f} V（這是機器回報的數字，不用填）")
        if status['relay_on'] is False:
            say('馬達電源目前關閉；上電後程式會另外檢查電壓與電流是否符合條件。')
    if status['writers']:
        say('目前的馬達控制程式：'+', '.join(status['writers']))
    say('感測器供電：'+('已開啟' if status['sensors_on'] else
        ('未完整開啟' if status['power_fresh'] else '未知（沒有新回讀）')))
    say(leg_feedback(status))
    if status.get('problem'):
        say('回讀問題：'+status['problem'])


def print_leg_status(site):
    say('\n腳的狀態（共用設定；可用不代表正在通電）')
    for index, (leg, position) in enumerate(zip(LEGS, ('左前','左中','左後','右前','右中','右後')), 1):
        state = '已屏蔽' if leg in site['disabled_legs'] else '可用'
        say(f'  {index}  {leg}（{position}）：{state}')


def manage_legs(runtime):
    last_action = '已查看腳的屏蔽狀態。'
    while True:
        site = runtime.site()
        print_leg_status(site)
        say('  1 屏蔽指定腳　　2 解除指定腳的屏蔽')
        say('  3 全部屏蔽　　　4 全部解除屏蔽　　5 重新整理　　0 回主選單')
        say('選好腳後立即儲存；已連線會先關電。解除屏蔽不會自行啟動馬達。')
        say('屏蔽限制主馬達，伺服電源仍共用。')
        choice = ask('要做什麼').lower()
        if choice in ('0', 'q', ''):
            return last_action
        if choice == '5':
            continue
        try:
            if choice in ('1', '2'):
                prompt = '要屏蔽哪些腳' if choice == '1' else '要解除哪些腳的屏蔽'
                text = ask(prompt+'？填編號，例如 1 3；Enter 取消')
                if not text or text.lower() == 'q':
                    continue
                indices = text.replace(',', ' ').replace('，', ' ').split()
                if not indices or any(i not in ('1','2','3','4','5','6') for i in indices) or len(set(indices)) != len(indices):
                    raise ValueError('請填不重複的 1～6 編號，例如 1 3 代表 L1、L3。')
                names = [LEGS[int(i)-1] for i in indices]
                updated = runtime.change_legs('disable' if choice == '1' else 'enable', names)
            elif choice == '3':
                updated = runtime.change_legs('disable', list(LEGS))
            elif choice == '4':
                updated = runtime.change_legs('enable-all')
            else:
                raise ValueError('請選 0～5。')
            last_action = '目前已屏蔽：'+(' '.join(updated['disabled_legs']) or '無')+'。'
        except (ValueError, RuntimeError, OSError) as exc:
            last_action = '屏蔽管理未完成：'+friendly_error(str(exc))
            say(last_action)


def menu(runtime, settings, settings_path, demo=False):
    site = runtime.check()
    # Keep a structurally valid preference even with every leg disabled. A
    # blocked selection remains visible until the operator changes it; never
    # silently switch a previously selected physical leg.
    fallback = default_plan(site['disabled_legs']) if site['enabled_legs'] else default_plan()
    plan = settings.get('plan', fallback)
    try:
        validate_plan(plan)
    except ValueError:
        say('上次動作格式無法使用，已載入小角度範例；執行前仍需確認。')
        plan = fallback
    ip = settings.get('ip', DEFAULT_SBRIO_IP)
    jetson = settings.get('jetson_ip', DEFAULT_JETSON_IP)
    port = settings.get('port', 50051)
    def save_preferences(candidate=None, target=None):
        new_ip, new_jetson, new_port = target or (ip, jetson, port)
        try:
            atomic_json(settings_path, dict(ip=new_ip, jetson_ip=new_jetson, port=new_port,
                                           plan=plan if candidate is None else candidate))
        except OSError as exc:
            raise RuntimeError(f'設定無法儲存，這次動作／網路修改未套用，原設定保留。請檢查 {settings_path}：{exc}') from exc
    last_run = None
    workshop = False
    workshop_site = None
    history = []
    def apply_plan(candidate):
        nonlocal plan, last_run
        save_preferences(candidate)
        if candidate != plan:
            history.append(copy.deepcopy(plan))
            del history[:-20]
        plan = copy.deepcopy(candidate)
        last_run = None
    extra_view = None
    last_result = '尚未執行動作。第一次可選 9，套用常用範例。'
    say('只用編號即可操作：2 選腳與動作 → 3 執行。9 有常用動作範例。')
    say(f'設備：sbRIO {ip}:{port} → Jetson {jetson}｜ROS 群組 {os.environ.get("ROS_DOMAIN_ID","99")}')
    while True:
        try:
            site = runtime.site()
            if workshop and workshop_site != site['hash']:
                workshop = False
                last_result = '現場設定已變更，已回到一般模式；選 14 可重新開啟現場調機。'
            if last_run is not None and last_run[1] != site['hash']:
                last_run = None
        except (ValueError, RuntimeError, OSError) as exc:
            site = None
            last_run = None
            workshop = False
            last_result = '腳的名單暫時無法讀取：'+str(exc)
        try:
            snapshot = runtime.panel_status()
        except (ValueError, RuntimeError, OSError) as exc:
            snapshot = None
            last_result = '回讀檢查未完成：'+str(exc)
        repeat = bool(last_run is not None and last_run[0] == plan and snapshot
                      and snapshot['bridge_count'] == 1 and not snapshot['writers']
                      and snapshot['sensors_on'] and snapshot['motor_fresh'] and snapshot['power_fresh'])
        say(dashboard(site, plan, snapshot, last_result, repeat, demo,
                      target=f'sbRIO {ip}:{port} → Jetson {jetson}', workshop=workshop))
        # Keep the requested details nearest the next input prompt instead of
        # immediately scrolling them away behind a newly printed dashboard.
        if extra_view == 'status':
            if snapshot is not None:
                print_status(snapshot)
            else:
                say('目前無法取得回讀；選 1 檢查連線。')
            say(f'本次詳細日誌：{runtime.log_dir}')
        elif extra_view == 'help':
            beginner_help()
        extra_view = None
        executing = False
        try:
            choice = ask('請選編號').lower()
            if choice in ('0', 'q'):
                return
            if choice == '':
                continue
            if choice == '1':
                say('\n第 1 步｜自動整理並檢查連線。會先正常停止可辨識的舊動作、整理過期紀錄；不會上電或啟動新動作。')
                print_status(runtime.reconnect(ip, port, jetson))
                say('連線檢查完成。接下來選 2 設定動作；設定已確認就選 3。')
                last_result = '連線檢查完成，已收到腳位置與電源資料。'
            elif choice == '2':
                site = runtime.site()
                updated = edit_plan(plan, site)
                apply_plan(updated)
                say('動作已儲存，尚未執行。\n'+describe(plan))
                say('接下來選 3 看完整清單，再決定是否執行。')
                last_result = '動作設定完成，尚未執行。選 3 執行。'
            elif choice in ('3', 'go'):
                executing = True
                site = runtime.site()
                if workshop and workshop_site != site['hash']:
                    workshop = False
                    last_run = None
                    say('現場設定已變更，這次先回到一般模式確認。')
                path = runtime.save_plan(plan, site)
                say('\n【1／5 連線】確認 sbRIO、Bridge 與新回讀；尚未上電。')
                runtime.ensure_connected(ip, port, jetson)
                calibrate = runtime.needs_calibration(path)
                repeat = last_run == (plan, site['hash']) and not calibrate
                say('\n即將執行：\n'+describe(plan))
                if workshop:
                    say('校正：'+('只校正 '+' '.join(plan['legs']) if calibrate else '沿用所選腳的有效校正')
                        +'。'+runtime.calibration_reason)
                else:
                    say(execution_scope(plan, site, calibrate, runtime.calibration_reason))
                say('程式將開啟馬達電源。完成後關閉馬達電源、保留感測器供電。')
                if not repeat and not workshop:
                    say('確認機身已固定、腳可自由轉動。')
                if not repeat and not workshop and ask('輸入 1 開始執行；直接 Enter 取消（也接受 y／開始）') not in ('1', '開始', 'y', 'Y'):
                    say('已取消，沒有執行上電／校正／動作。')
                    last_result = '已取消執行，沒有開始動作。'
                    continue
                last_run = None
                if workshop:
                    say('現場調機：已按 3 開始，本次不再詢問確認。')
                say('執行中按 Q、空白鍵、Esc 或 Ctrl+C 停止並關電；不用按 Enter。')
                with stop_keys:
                    runtime.execute(path, site['hash'], calibrate)
                last_run = (copy.deepcopy(plan), site['hash'])
                say('完成。再按 3 可重做；也可直接輸入新動作，例如 L2 -5。')
                last_result = '動作完成，馬達電源已關閉。相同設定可選 3 重做。'
            elif choice == '4':
                last_run = None
                workshop = False
                okay = runtime.safe_off()
                last_result = '全部關電已確認。' if okay else '未確認關電成功，請使用實體斷電。'
            elif choice == '5':
                extra_view = 'status'
                continue
            elif choice == '8':
                workshop = False
                last_result = manage_legs(runtime)
                # Mask changes are committed separately by the native tool.
                # A preference-write failure must not claim to undo that mask.
                try:
                    save_preferences()
                except RuntimeError as exc:
                    say('屏蔿名單以畫面重新讀取的結果為準；操作偏好另存失敗。'+str(exc))
            elif choice == '9':
                updated = choose_preset(plan, runtime.site())
                apply_plan(updated)
                last_result = '常用動作已套用，尚未執行。選 3 執行。'
            elif choice == '10':
                updated = motion_library(Path(settings_path).with_name('motions.json'), plan, runtime.site())
                if updated is not None:
                    apply_plan(updated)
                    last_result = '已載入收藏動作，尚未執行。請查看本次動作，選 3 執行。'
                else:
                    last_result = '已返回主畫面，目前動作保留。'
            elif choice == '11':
                apply_plan(quick_adjust(plan, runtime.site()))
                last_result = '快速調整已儲存。請查看本次動作，選 3 才執行；選 12 可復原設定。'
            elif choice == '12':
                if not history:
                    last_result = '沒有更早的動作設定。本次開啟後的修改，最多可復原 20 次。'
                    continue
                restored = copy.deepcopy(history[-1])
                save_preferences(restored)
                plan = restored
                history.pop()
                last_run = None
                last_result = '已復原上一份動作設定；實體位置、屏蔿名單與電源沒有改變。選 3 才執行。'
            elif choice == '13':
                apply_plan(quick_power(plan, runtime.site()))
                last_result = f'出力上限已設為 {plan["max_pwm"]:g}。速度與時間保留；按 3 執行。'
            elif choice == '15':
                from .motion_limits import edit_motion_limits
                edit_motion_limits(runtime, ask, say, numeric)
                last_run = None
                workshop = False
                last_result = '到位／追蹤限制已查看；下次啟動使用儲存的設定。'
            elif choice == '14':
                say('\n現場調機模式｜適合你在機器人旁連續調整與試動。')
                say('開啟後：主畫面按 3 就開始，可能上電並校正所選腳；改參數或換腳後也不再多問一次。')
                say('改設定本身不會動；每次仍由你按 3。Q 停止；關電、執行失敗、修改現場設定或重開會回到一般模式。')
                value = choose('1 開啟現場調機　2 一般模式　0 返回', ('1', '2', '0'))
                if value != '0':
                    checked_site = runtime.site()['hash'] if value == '1' else None
                    workshop = value == '1'
                    workshop_site = checked_site
                    last_result = ('現場調機已開啟：確認主畫面設定後，按 3 直接開始。' if workshop else
                                   '已回到一般模式：新動作會先詢問確認。')
            elif choice == '6':
                say('目前現場建議：sbRIO 192.168.30.254、Jetson 192.168.30.8、通訊埠 50051。')
                target = edit_network(ip, jetson, port, demo)
                save_preferences(target=target)
                if target != (ip, jetson, port):
                    last_run = None
                    workshop = False
                ip, jetson, port = target
                last_result = f'網路位址已儲存：sbRIO {ip}:{port}，Jetson {jetson}。'
            elif choice == '7':
                updated = edit_limits(plan, runtime.site())
                apply_plan(updated)
                say('限制已儲存，尚未執行；可選 3 檢查動作。')
                last_result = '出力、加速度與速度上限已儲存。'
            elif choice == 'h':
                extra_view = 'help'
                continue
            elif choice == 'r':
                site = runtime.site()
                updated = default_plan(site['disabled_legs'])
                validate_plan(updated, site['disabled_legs'])
                apply_plan(updated)
                say('已套用入門範例，只改設定，沒有執行動作。\n'+describe(plan))
                say('要修改選 2；執行選 3，會按需要校正本次選中的腳。')
                last_result = '已套用入門範例，尚未執行。'
            else:
                updated = quick_plan(choice, plan, runtime.site()['disabled_legs'])
                apply_plan(updated)
                say('設定已更新，尚未執行。選 3 執行。')
                last_result = '設定已更新，尚未執行。'
        except (Cancelled, KeyboardInterrupt):
            if executing:
                workshop = False
                last_run = None
            say('\n已取消目前操作。')
            last_result = '已取消目前操作。'
        except (ValueError, RuntimeError, OSError) as exc:
            if executing:
                workshop = False
                last_run = None
            last_result = '未完成：'+friendly_error(str(exc))
            say('\n'+last_result)
            say(f'詳細日誌：{runtime.log_dir}')


def main(argv=None):
    parser = argparse.ArgumentParser(description='中文機器人操作入口；啟動選單本身不會上電或動作。')
    parser.add_argument('--demo', action='store_true', help='純介面演練，不連線硬體，不產生校正紀錄')
    parser.add_argument('--check', action='store_true', help='只檢查安裝與現場設定，不初始化 ROS、不連線硬體')
    args = parser.parse_args(argv)
    if not args.demo and not args.check and not sys.stdin.isatty():
        parser.error('實機操作需要互動終端機；自動化檢查請用 --check，介面演練請用 --demo')
    if args.demo and args.check:
        parser.error('--demo 與 --check 請擇一使用')
    runtime = None
    temporary = tempfile.TemporaryDirectory(prefix='rinbo-demo-') if args.demo else None
    root = Path(temporary.name) if temporary else Path.home()/'.local/state/rinbo_control'
    settings_path = root/'settings.json'
    lock = None
    code = 0
    previous = {}
    def terminate(_signum, _frame):
        raise ConsoleExit
    try:
        root.mkdir(parents=True, exist_ok=True, mode=0o700)
        if not args.check:
            lock = (root/'console.lock').open('a')
            try:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('已有一個操作台正在執行，請回到原本的視窗')
        if args.demo:
            from .demo import DemoRuntime
            runtime = DemoRuntime(root, say, stop_requested)
        else:
            runtime = Runtime(root, say, stop_requested)
        for sig in (signal.SIGTERM, signal.SIGHUP):
            previous[sig] = signal.signal(sig, terminate)
        if args.check:
            site = runtime.check()
            say('安裝檢查通過；沒有初始化 ROS 或連線設備。')
            say('目前禁用腳：'+(' '.join(site['disabled_legs']) or '無'))
            return 0
        try:
            settings = load_settings(settings_path)
        except (ValueError, OSError) as exc:
            raise RuntimeError(f'無法讀取操作偏好：{exc}。請檢查 {settings_path}；未啟動硬體。')
        menu(runtime, settings, settings_path, args.demo)
    except (KeyboardInterrupt, EOFError, ConsoleExit):
        say('\n正在結束操作台 …')
    except Exception as exc:
        say('無法啟動／完成：'+friendly_error(str(exc)))
        say('首次更新後，請在機器人停止動作時執行：\n  CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --packages-select rinbo_fsm rinbo_control --parallel-workers 1')
        code = 1
    finally:
        if runtime:
            try:
                if not runtime.close():
                    code = 2
            except Exception as exc:
                say('退出清理未完成，請確認實體電源：'+str(exc))
                code = 2
        if lock:
            lock.close()
        if temporary:
            temporary.cleanup()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
