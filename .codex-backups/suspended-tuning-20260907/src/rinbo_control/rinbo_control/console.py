"""One terminal, plain Chinese, explicit review before powering or moving."""
import argparse
import copy
import fcntl
import os
from pathlib import Path
import select
import signal
import sys
import tempfile

from .plans import (MODES, atomic_json, default_plan, describe, friendly_error,
                    ipv4, load_settings, number, selected_legs, validate_plan)
from .runtime import (DEFAULT_JETSON_IP, DEFAULT_SBRIO_IP, Cancelled, Runtime, discover)


class ConsoleExit(BaseException):
    pass


def say(text):
    print(text, flush=True)


def ask(prompt, default=None):
    shown = f'{default:g}' if isinstance(default, (int, float)) else str(default)
    suffix = f' [直接 Enter 用 {shown}]' if default is not None else ''
    value = input(prompt+suffix+'：').strip()
    return value if value or default is None else str(default)


def numeric(prompt, default, lo, hi, *, suggestion=None, explanation=None):
    if explanation:
        say(explanation)
    if suggestion:
        say('入門建議：'+suggestion)
    say(f'可填範圍：{lo:g}～{hi:g}。下方會顯示直接 Enter 採用的值。')
    while True:
        try:
            return number(ask(prompt, default), lo, hi)
        except ValueError as exc:
            say(f'請填 {lo:g}～{hi:g} 之間的數字，例如 {default:g}；不用加單位。')


def beginner_help():
    say('''
第一次使用：照 1 → 2 → 3 做。

1 接通機器人：讓這台 Jetson 電腦能讀到腳的位置。
  程式會登入 sbRIO（機器人上的控制器）、啟動通訊。
  可能需要 sbRIO 的 admin 密碼；這一步不送上電或動作指令。
2 設定動作：選哪隻腳、怎麼轉。這一步只改設定。
3 確認並執行：先列出完整內容，輸入「開始」後才處理電源與動作。
  首次會先校正，也就是找機器的零點；會涉及全部未禁用主馬達及相關伺服。
  校正後才執行你選的腳。你填的速度／出力限制只用於校正後的逐腳動作。

不知道怎麼填，可以回選單輸入 r，套用入門範例：
  只選目前可用的一隻腳 → 從現在的位置小轉 +5°。
  保持 3 秒、速度上限 10°／秒、出力上限 20、加速度 10°／秒²。
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

怎麼停：動作中按 Ctrl+C，或輸入 s 再 Enter，會停止並嘗試關電。
  選 4：關電、保留連線，留在選單。選 0：關電並結束本次操作。
  正常動作結束會關馬達電源，保留感測器供電，方便再調整。
''')


def stop_requested():
    # Avoid consuming scripted menu inputs in --demo tests; live operation is
    # TTY-only. Ctrl+C is handled as KeyboardInterrupt in either case.
    if not sys.stdin.isatty():
        return False
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        line = sys.stdin.readline()
        if not line or line.strip().lower() in ('s', 'q', 'stop', '停止'):
            return True
    return False


def edit_plan(current, site):
    plan = copy.deepcopy(current)
    say('\n第 2 步｜設定動作。現在只填設定，之後選 3 才執行。')
    say('可選腳：'+' '.join(site['enabled_legs']))
    say('禁用腳：'+(' '.join(site['disabled_legs']) or '無'))
    say('第一次先選一隻可用的腳；多隻用空白隔開，例如 L2 R2。禁用腳不能選。')
    legs = selected_legs(ask('要控制哪些腳？用空白分開', ' '.join(plan['legs'])), site['disabled_legs'])
    plan['duration_s'] = numeric('保持／勻速時間（秒）', plan['duration_s'], 1, 60,
        suggestion='先用 3 秒，做一次短測試。',
        explanation='定點動作：到位後保持多久。旋轉動作：以固定速度轉多久。\n總時間還會加上對齊、加減速與收尾。')
    plan['max_speed_deg_s'] = numeric('速度上限（度／秒）', plan['max_speed_deg_s'], 1, 90,
        suggestion='先用 10 度／秒。',
        explanation='這是目標速度的上限；後面選「按速度旋轉」時，才填那隻腳實際要用的速度。')
    new = {}
    modes = list(MODES)
    for name in legs:
        old = plan['legs'].get(name, {'mode': 'relative', 'move_deg': 5.0})
        say(f'\n{name} 要怎麼動？\n  1 小轉一下：從現在再轉幾度（第一次建議選這個）\n  2 移到角度：移到相對校正零點的位置\n  3 按速度旋轉：每秒轉幾度，轉到時間結束\n  4 按相位旋轉：先對齊再轉，適合比較多隻腳的角度差')
        choice = ask('選擇', modes.index(old['mode'])+1)
        if choice not in ('1','2','3','4'):
            raise ValueError('請輸入 1～4；原本的設定尚未改動')
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
            maximum = plan['max_speed_deg_s']
            spec['speed_deg_s'] = numeric('每秒轉幾度？負數是反方向',
                min(maximum, max(-maximum, old.get('speed_deg_s',5))), -maximum, maximum,
                suggestion=f'先用 {min(5,maximum):g} 度／秒；負數往反方向轉，0 不持續旋轉。',
                explanation='例如 5 度／秒，固定速度轉一整圈約需 72 秒。')
            if mode == 'cycle':
                spec['phase_deg'] = numeric('起始相位（度）', old.get('phase_deg',0), -360, 360,
                    suggestion='單腳先用 0；兩腳同相用 0、0，半圈錯開用 0、180。',
                    explanation='相位就是開始旋轉前先對齊的角度；對齊可能轉半圈。\n要維持相位差，兩腳速度必須相同。')
        new[name] = spec
    plan['legs'] = new
    validate_plan(plan, site['disabled_legs'])
    return plan


def print_status(status):
    bridge = '未啟動' if not status['bridge_count'] else ('已啟動' if status['bridge_count']==1 else '重複！')
    relay = {None:'未知（沒有新回讀）', True:'已開啟', False:'已關閉'}[status['relay_on']]
    say(f"Bridge：{bridge}｜腳位置：{'有新資料' if status['motor_fresh'] else '沒有新資料'}｜馬達電源：{relay}")
    if status['voltage'] is not None:
        say(f"母線電壓：{status['voltage']:.1f} V")
    if status['writers']:
        say('目前的馬達控制程式：'+', '.join(status['writers']))


def menu(runtime, settings, settings_path, demo=False):
    site = runtime.check()
    plan = settings.get('plan', default_plan(site['disabled_legs']))
    try:
        validate_plan(plan, site['disabled_legs'])
    except ValueError:
        say('上次動作與目前禁用腳不符，已改成可用腳的小角度範例；執行前仍需確認。')
        plan = default_plan(site['disabled_legs'])
    ip = settings.get('ip', DEFAULT_SBRIO_IP)
    jetson = settings.get('jetson_ip', DEFAULT_JETSON_IP)
    port = settings.get('port', 50051)
    say('第一次操作：1 接通機器人 → 2 填動作 → 3 看清單後執行。')
    say('不知道怎麼填，輸入 h 看說明；輸入 r 可套用單腳小转 5° 的入門範例。')
    say('填數值時，直接 Enter 沿用顯示值；那可能是你上次改過的設定。')
    while True:
        say('\n'+('【介面演練：不連線、不上電、不動馬達】' if demo else '機器人控制台'))
        say(f'sbRIO {ip}:{port} → Jetson {jetson}｜ROS 群組 {os.environ.get("ROS_DOMAIN_ID","99")}')
        say('目前的動作設定（選 3 才會執行）：')
        say(describe(plan))
        say('\n  1 接通機器人：檢查能否讀到腳的位置，不送上電指令\n  2 設定怎麼動：選腳、角度或速度，只改設定\n  3 確認並執行：輸入「開始」後才上電／校正／動作\n  4 全部關電：保留連線，留在選單\n  5 看目前狀態：有沒有連上、電源有沒有開\n  6 網路設定：位址已填好，通常不用改\n  7 出力與加速度：已有預設，第一次可跳過\n  h 新手說明：每一步的意思與建議數值\n  r 套用入門範例：一隻可用腳小轉 +5°，只改設定\n  0 結束操作：關電、停止本次通訊並離開')
        try:
            choice = ask('請選擇（第一次先選 1；想看說明輸入 h）').lower()
            if choice == '0':
                return
            if choice == '1':
                say('\n第 1 步｜接通機器人。可能需要 sbRIO 的 admin 密碼；這一步不送上電或動作指令。')
                print_status(runtime.connect(ip, port, jetson))
                say('連線檢查完成。接下來選 2 設定動作；設定已確認就選 3。')
            elif choice == '2':
                site = runtime.site()
                plan = edit_plan(plan, site)
                say('動作已儲存，尚未執行。\n'+describe(plan))
                say('接下來選 3 看完整清單，再決定是否執行。')
            elif choice == '3':
                say('\n第 3 步｜先檢查並列出內容；稍後輸入「開始」才執行。')
                site = runtime.site()
                path = runtime.save_plan(plan, site)
                runtime.connect(ip, port, jetson)
                calibrate = runtime.needs_calibration(path)
                say('\n即將執行：\n'+describe(plan))
                say('禁用腳：'+(' '.join(site['disabled_legs']) or '無'))
                if calibrate:
                    say('校正 = 尋找機器的零點。這次會先校正，再執行你選的腳。')
                    say('需要先校正，會動到全部未禁用主馬達：'+' '.join(site['enabled_legs']))
                    say('校正也會控制連接的伺服；禁用主馬達不代表該腳伺服已斷電。')
                    say('你填的速度／出力限制用於校正後的逐腳動作；校正依現場設定執行。')
                else:
                    say('有效校正紀錄與感測器供電已確認，可沿用校正。')
                say('程式將開啟馬達電源。完成後關閉馬達電源、保留感測器供電。')
                say('請固定機身、腳懸空、清空轉動範圍，並準備實體急停。')
                if ask('輸入「開始」才執行；直接 Enter 取消') != '開始':
                    say('已取消，沒有執行上電／校正／動作。')
                    continue
                runtime.execute(path, site['hash'], calibrate)
                say('本次動作已結束。要改動作選 2，再做一次選 3；結束實驗選 0。')
            elif choice == '4':
                runtime.safe_off()
            elif choice == '5':
                print_status(runtime.status())
            elif choice == '6':
                say('目前現場建議：sbRIO 192.168.30.254、Jetson 192.168.30.8、通訊埠 50051。')
                say('已經能連線就保留原值。這些是設備位址，不是控制腳的數值。')
                say('直接 Enter 保留位址；輸入 ? 搜尋已知網路鄰居（只測通訊埠，不自動選設備）。')
                value = ask('sbRIO IP', ip)
                if value == '?':
                    matches = [] if demo else discover(port)
                    say('可連通的候選位址：'+(', '.join(matches) or '沒有找到'))
                    say('通訊埠可連通不等於已確認設備身分，請選你自己的 sbRIO。')
                    value = ask('sbRIO IP', ip)
                new_ip = ipv4(value)
                new_jetson = ipv4(ask('Jetson 有線 IP', jetson))
                new_port = ask('sbRIO 通訊埠', port)
                if not new_port.isdecimal() or not 1 <= int(new_port) <= 65535:
                    raise ValueError('通訊埠須為 1～65535 的整數')
                ip, jetson, port = new_ip, new_jetson, int(new_port)
            elif choice == '7':
                updated = copy.deepcopy(plan)
                say('這裡只調整校正後的逐腳動作。沒有特別需求，保留入門值即可。')
                updated['max_pwm'] = numeric('逐腳動作出力上限',plan['max_pwm'],1,80,
                    suggestion='先用 20。腳不動或跟不上時，先查原因，不要直接調大。',
                    explanation='出力上限限制控制器最多能給多少驅動；它不是轉動速度，也不是伏特或百分比。')
                updated['acceleration_deg_s2'] = numeric('目標加速度（度／秒²）',plan['acceleration_deg_s2'],1,90,
                    suggestion='先用 10。',
                    explanation='決定目標速度增加或減少得多快。填 10，從 0 到 10 度／秒至少約需 1 秒。')
                plan = updated
                say('限制已儲存，尚未執行；可選 3 檢查動作。')
            elif choice == 'h':
                beginner_help()
                continue
            elif choice == 'r':
                site = runtime.site()
                updated = default_plan(site['disabled_legs'])
                validate_plan(updated, site['disabled_legs'])
                plan = updated
                say('已套用入門範例，只改設定，沒有執行動作。\n'+describe(plan))
                say('要修改選 2；要檢查並執行選 3。首次校正仍涉及全部未禁用主馬達與相關伺服。')
            else:
                say('請輸入選單上的數字；h 是說明，r 是入門範例。')
            atomic_json(settings_path, dict(ip=ip, jetson_ip=jetson, port=port, plan=plan))
        except (Cancelled, KeyboardInterrupt):
            say('\n已取消目前操作。')
        except (ValueError, RuntimeError, OSError) as exc:
            say('\n未完成：'+friendly_error(str(exc)))
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
