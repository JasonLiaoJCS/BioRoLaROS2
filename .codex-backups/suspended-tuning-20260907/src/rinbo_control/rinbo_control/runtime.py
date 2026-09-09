"""Supervise existing trusted controllers; no new motor/power publisher here."""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
import errno
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import queue
import signal
import socket
import subprocess
import sys
import threading
import time

from .plans import atomic_json, ipv4, friendly_error, validate_plan

DEFAULT_SBRIO_IP = '192.168.30.254'
DEFAULT_JETSON_IP = '192.168.30.8'


class Cancelled(RuntimeError):
    pass


def tcp_probe(ip, port, timeout=1.0):
    """Keep the socket failure reason; refusal is different from no response."""
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return {'status': 'open', 'error': ''}
    except OSError as exc:
        if exc.errno == errno.ECONNREFUSED:
            status = 'refused'
        elif isinstance(exc, TimeoutError) or exc.errno == errno.ETIMEDOUT:
            status = 'timeout'
        elif exc.errno in (errno.ENETUNREACH, errno.EHOSTUNREACH):
            status = 'unreachable'
        else:
            status = 'error'
        return {'status': status, 'error': str(exc), 'errno': exc.errno}


def tcp_available(ip, port, timeout=1.0):
    return tcp_probe(ip, port, timeout)['status'] == 'open'


def connection_error(ip, port, result):
    target = f'{ip}:{port}'
    if result['status'] == 'refused':
        return (f'{target} 拒絕連線：通常是 sbRIO 的 grpccore 通訊服務尚未啟動，'
                '或未監聽這個位址／通訊埠；防火牆主動拒絕也可能造成此結果。\n'
                f'請先登入 sbRIO（ssh admin@{ip}）檢查 grpccore，完成後再選 1。'
                '只開 Jetson 的 Bridge 無法啟動 sbRIO 端服務。')
    if result['status'] == 'timeout':
        return f'{target} 等待逾時：沒有收到回應。請檢查 sbRIO 位址、供電、網路線及防火牆；不能據此認定服務未啟動。'
    if result['status'] == 'unreachable':
        return f'{target} 網路無法到達：請檢查 Jetson 網卡、網路線與 sbRIO 位址。'
    return f'{target} 連線失敗：{result["error"]}'


def route_source(ip):
    result = subprocess.run(['ip', '-j', '-4', 'route', 'get', ipv4(ip)], capture_output=True, text=True, timeout=3)
    if result.returncode:
        raise RuntimeError('找不到通往 sbRIO 的網路路由，請檢查網路線與網卡設定')
    routes = json.loads(result.stdout)
    if not routes or not routes[0].get('prefsrc'):
        raise RuntimeError('無法判斷 Jetson 要使用哪個網卡位址')
    return ipv4(routes[0]['prefsrc'])


def discover(port=50051):
    """Only inspect known local neighbours, never sweep a subnet or adopt a host."""
    result = subprocess.run(['ip', '-j', '-4', 'neigh', 'show'], capture_output=True, text=True, timeout=3)
    if result.returncode:
        raise RuntimeError('無法讀取網路鄰居資料')
    candidates = {DEFAULT_SBRIO_IP}
    for row in json.loads(result.stdout):
        if row.get('dev', '').startswith(('docker', 'veth', 'tailscale', 'lo')):
            continue
        try:
            candidates.add(ipv4(row['dst']))
        except (ValueError, KeyError):
            continue
    with ThreadPoolExecutor(max_workers=8) as executor:
        addresses = sorted(candidates)[:32]
        reachable = list(executor.map(lambda ip: tcp_available(ip, port, .4), addresses))
    return [ip for ip, ok in zip(addresses, reachable) if ok]


class Child:
    """Owned process group, bounded UI queue and rotating complete raw logs."""
    def __init__(self, command, env, log_path):
        self.lines = queue.Queue(maxsize=128)
        self.done_marker = threading.Event()
        self.fatal = threading.Event()
        self.tail = ''
        self.log_path = Path(log_path)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.handler = RotatingFileHandler(self.log_path, maxBytes=2*1024*1024, backupCount=2)
        self.handler.setFormatter(logging.Formatter('%(message)s'))
        try:
            guarded = [sys.executable, '-m', 'rinbo_control.guardian', str(os.getpid()), *command]
            self.process = subprocess.Popen(guarded, env=env, stdin=subprocess.DEVNULL,
                                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                            text=True, errors='replace', bufsize=1, start_new_session=True)
        except BaseException:
            self.handler.close()
            raise
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        try:
            for line in self.process.stdout:
                line = line.rstrip()
                self.handler.emit(logging.LogRecord('child', logging.INFO, '', 0, line, (), None))
                self.tail = (self.tail+'\n'+line)[-6000:]
                if 'State: DONE' in line:
                    self.done_marker.set()
                if '[FATAL]' in line or 'SAFETY STOP' in line:
                    self.fatal.set()
                try:
                    self.lines.put_nowait(line)
                except queue.Full:
                    pass
        finally:
            self.process.stdout.close()
            self.handler.close()

    def stop(self):
        def send(sig):
            try:
                os.killpg(self.process.pid, sig)
            except ProcessLookupError:
                pass
        if self.process.poll() is None:
            send(signal.SIGINT)
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                send(signal.SIGTERM)
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    send(signal.SIGKILL)
                    self.process.wait(timeout=2)
        self.reader.join(timeout=1)

    def drain(self):
        while True:
            try:
                yield self.lines.get_nowait()
            except queue.Empty:
                break


class Runtime:
    def __init__(self, state_dir, progress=print, cancel=lambda: False):
        self.state_dir = Path(state_dir)
        self.log_dir = self.state_dir/'logs'/f'{datetime.now():%Y%m%d-%H%M%S}-{os.getpid()}'
        self.progress, self.cancel = progress, cancel
        self.bridge = None
        self.child = None
        self.feedback = None
        self.remote = None
        self.power_touched = False
        self.calibration_epoch = None
        self.env = dict(os.environ, RCUTILS_COLORIZED_OUTPUT='0', PYTHONUNBUFFERED='1')
        self.serial = 0

    @staticmethod
    def executable(package, name):
        from ament_index_python.packages import get_package_prefix
        path = Path(get_package_prefix(package))/'lib'/package/name
        if not path.is_file() or not os.access(path, os.X_OK):
            raise RuntimeError(f'缺少 {package}/{name}，請先完成編譯')
        return str(path)

    def command(self, package, name, *args):
        return [self.executable(package, name), *map(str, args)]

    def query(self, package, name, *args):
        return subprocess.run(self.command(package, name, *args), env=self.env,
                              capture_output=True, text=True, timeout=8)

    def site(self):
        result = self.query('rinbo_fsm', 'rinbo_legs', 'status', '--json')
        if result.returncode:
            raise RuntimeError(friendly_error(result.stderr or result.stdout))
        return json.loads(result.stdout)

    def check(self):
        for package, name in [('rinbo_fsm', 'rinbo_cali'), ('rinbo_ros_bridge', 'rinbo_ros_bridge'),
                              ('redrhex_lowlevel_bridge', 'rinbo_power_tool')]:
            self.executable(package, name)
        result = self.query('rinbo_fsm', 'rinbo_manual', '--help')
        if result.returncode or '--check-ready' not in result.stdout:
            raise RuntimeError('rinbo_manual 版本較舊，請先重新編譯 rinbo_fsm')
        return self.site()

    def probe(self):
        if self.feedback is None:
            from .feedback import Feedback
            self.feedback = Feedback()
        return self.feedback

    def _start(self, package, name, *args):
        self.serial += 1
        return Child(self.command(package, name, *args), self.env,
                     self.log_dir/f'{self.serial:02d}-{name}.log')

    def connect(self, ip, port=50051, local_ip=DEFAULT_JETSON_IP):
        ip, local_ip = ipv4(ip), ipv4(local_ip)
        self.progress(f'檢查 sbRIO {ip}:{port} …')
        source = route_source(ip)
        if source != local_ip:
            raise RuntimeError(f'目前路由使用 Jetson {source}，設定卻是 {local_ip}；請先確認網卡／位址設定')
        self.progress(f'Jetson 網路位址正確：{source}')
        if self.remote is None:
            from .sbrio import RemoteServices
            self.remote = RemoteServices(self.state_dir, self.progress, self.log_dir)
        self.progress('依 v6.6 檢查／啟動 sbRIO core 與 FPGA（不上電）…')
        self.remote.start(ip, port)
        result = tcp_probe(ip, port, 2)
        atomic_json(self.log_dir/f'connection-{time.time_ns()}.json',
                    dict(sbrio_ip=ip, port=port, jetson_ip=source, **result))
        if result['status'] != 'open':
            raise RuntimeError(connection_error(ip, port, result)+'\n本次沒有啟動 Bridge 或開啟馬達電源。')
        self.progress(f'sbRIO 通訊埠 {port} 可連線；接著檢查 Bridge 與實際回讀。')
        feedback = self.probe()
        feedback.spin(.5)
        count = feedback.bridge_count()
        if count > 1:
            raise RuntimeError('偵測到多個 Bridge，請先關閉重複的程式')
        if count == 1:
            actual = feedback.bridge_ip()
            if actual != ip:
                raise RuntimeError(f'既有 Bridge 指向 {actual}，與這次選的 {ip} 不同；不會另開一份')
            self.progress(f'沿用既有 Bridge（{actual}）；退出時只會關閉由本工具啟動的 Bridge。')
        else:
            if self.bridge and self.bridge.process.poll() is None:
                raise RuntimeError('本工具的 Bridge 程序還在，但 ROS 看不到它；請先退出並查看日誌')
            self.env.update(CORE_MASTER_ADDR=f'{ip}:{port}', CORE_LOCAL_IP=local_ip, CORE_IP=ip)
            from ament_index_python.packages import get_package_share_directory
            cfg = Path(get_package_share_directory('rinbo_ros_bridge'))/'config/redrhex_safe.yaml'
            self.bridge = self._start('rinbo_ros_bridge', 'rinbo_ros_bridge', '--ros-args',
                                      '--params-file', cfg, '-p', f'core_ip:={ip}')
            end = time.monotonic()+5
            while feedback.bridge_count() != 1 and time.monotonic() < end:
                if self.cancel():
                    raise Cancelled('已取消連線')
                if self.bridge.process.poll() is not None:
                    raise RuntimeError(friendly_error(self.bridge.tail))
                feedback.spin(.05)
            if feedback.bridge_count() != 1 or feedback.bridge_ip() != ip:
                raise RuntimeError('Bridge 未正常出現在 ROS 中，請查看日誌')
            self.progress('Bridge 已啟動。啟動保護是正常等待；接下來仍需確認實際回讀。')
        # The SOP requires actual state messages before any power command.
        end = time.monotonic()+8
        while True:
            snapshot = self.status()
            if snapshot['bridge_count'] != 1:
                raise RuntimeError('Bridge 數量改變，停止通訊檢查')
            if snapshot['motor_fresh'] and snapshot['power_fresh']:
                self.progress('通訊就緒：motor/state、power/state 都有新回讀；本次沒有上電。')
                return snapshot
            if self.cancel():
                raise Cancelled('已取消通訊檢查')
            if time.monotonic() >= end:
                raise RuntimeError('Bridge 已開，但 8 秒內未收到 motor/state 與 power/state 新回讀；請檢查 sbRIO driver 日誌。尚未上電。')

    def status(self):
        return self.probe().status()

    def idle(self):
        end = time.monotonic()+2
        while True:
            self.probe().spin(.05)
            writers = self.probe().writers()
            if not writers:
                return
            if time.monotonic() > end:
                raise RuntimeError('其他控制程式仍在發布馬達命令：'+', '.join(writers)+'；請先停止它')

    def save_plan(self, plan, site):
        import yaml
        validate_plan(plan, site['disabled_legs'])
        self.log_dir.mkdir(parents=True, exist_ok=True)
        path = self.log_dir/f'plan-{time.time_ns()}.yaml'
        with path.open('x') as f:
            yaml.safe_dump(plan, f, allow_unicode=True, sort_keys=False)
        result = self.query('rinbo_fsm', 'rinbo_manual', '--plan', path)
        if result.returncode:
            raise RuntimeError(friendly_error(result.stderr or result.stdout))
        return path

    def calibration_ready(self, path):
        result = self.query('rinbo_fsm', 'rinbo_manual', '--plan', path, '--check-ready')
        return result.returncode == 0 and 'CALIBRATION READY' in result.stdout

    def needs_calibration(self, path):
        status = self.status()
        return not (self.calibration_epoch is not None and
                    self.calibration_epoch == self.probe().sensor_epoch and
                    status['sensors_on'] and status['motor_fresh'] and self.calibration_ready(path))

    def _run(self, package, name, *args, timeout=30, until_calibrated=False, allow_cancel=True):
        child = self._start(package, name, *args)
        self.child = child
        deadline = time.monotonic()+timeout
        try:
            while True:
                if allow_cancel and self.cancel():
                    raise Cancelled('已要求停止')
                if self.feedback:
                    self.feedback.spin(.02)
                else:
                    time.sleep(.02)
                for line in child.drain():
                    if 'DISCOVERY WAIT:' in line:
                        self.progress('正在辨識 Bridge 的資料來源；尚未建立馬達命令發布者 …')
                    elif 'DISCOVERY READY:' in line:
                        self.progress('Bridge 來源已確認，接著檢查新回讀與安全握手。')
                    elif 'State: DONE' in line:
                        self.progress('校正已完成。')
                    elif 'DC_SPINNING |' in line:
                        self.progress('校正中：正在尋找各脚零點 …')
                    elif 'WAIT_SERVO |' in line:
                        self.progress('校正中：正在定位伺服 …')
                    elif 't=' in line and 'deg' in line:
                        self.progress(line.split(']: ', 1)[-1])
                    elif '[FATAL]' in line or 'SAFETY STOP' in line:
                        self.progress(friendly_error(line))
                if child.fatal.is_set():
                    raise RuntimeError(friendly_error(child.tail))
                if until_calibrated and child.done_marker.is_set():
                    child.stop()
                    if child.fatal.is_set() or child.process.returncode != 0:
                        raise RuntimeError('校正結束程序異常：'+friendly_error(child.tail))
                    return
                code = child.process.poll()
                if code is not None:
                    child.reader.join(timeout=1)
                    if code != 0 or until_calibrated or child.fatal.is_set():
                        raise RuntimeError(friendly_error(child.tail or f'{name} 提前結束，代碼 {code}'))
                    return
                if time.monotonic() > deadline:
                    raise RuntimeError(f'{name} 等待逾時，已要求停止')
                if self.bridge and self.bridge.process.poll() is not None:
                    raise RuntimeError('Bridge 意外退出，請確認實體電源並查看日誌')
        finally:
            child.stop()
            self.child = None

    def power(self, mode, cancellable=True):
        self.power_touched = True  # even a failed request needs cleanup
        args = [mode, '--wait-for-subscriber-s', '8'] + (['--confirm-relay'] if mode == 'relay' else [])
        self._run('redrhex_lowlevel_bridge', 'rinbo_power_tool', *args,
                  timeout=30, allow_cancel=cancellable)
        if mode == 'off':
            self.power_touched = False

    def execute(self, path, site_hash, calibrate):
        """Called only after a concrete operator review. Every failure goes off."""
        success = False
        try:
            if self.site()['hash'] != site_hash:
                raise RuntimeError('禁用腳或安全設定已改變，請重新檢查並確認動作')
            self.idle()
            status = self.status()
            if not calibrate and self.needs_calibration(path):
                raise RuntimeError('校正／感測器狀態已改變，請重新確認操作')
            if not status['sensors_on']:
                self.progress('首次上電：先確認 Digital、Signal、Relay 全部關閉 …')
                self.power('off')
                self.calibration_epoch = None
                self.progress('開啟控制與感測器電源 …')
                self.power('sequence')  # sensors only, never implicitly relay-on
            end = time.monotonic()+3
            while not (self.probe().fresh('/motor/state') and self.probe().fresh('/power/state')):
                if self.cancel():
                    raise Cancelled('已取消')
                if time.monotonic() > end:
                    raise RuntimeError('感測器上電後仍收不到新資料，請檢查 sbRIO 與網路')
                self.probe().spin(.03)
            self.probe().wait_motor_disabled(self.cancel)
            if self.status()['relay_on'] is True:
                self.progress('Relay 已開啟：只核對連續電源回讀，不重送上電指令。')
                self.probe().wait_power_ready(self.site()['disabled_legs'], self.cancel)
                self.power_touched = True
            else:
                self.progress('開啟馬達電源，等待連續 3 筆電壓／電流確認 …')
                self.power('relay')
            epoch = self.probe().sensor_epoch if calibrate else self.calibration_epoch
            if calibrate:
                self.progress('正在校正全部未禁用腳；不是只有這次選中的腳。')
                self._run('rinbo_fsm', 'rinbo_cali', timeout=60, until_calibrated=True)
                self.idle()
                self.probe().wait_motor_disabled(self.cancel)
            if not self.calibration_ready(path):
                raise RuntimeError('校正完成紀錄未通過檢查，停止操作')
            status = self.status()
            if (epoch != self.probe().sensor_epoch or
                    not status['sensors_on'] or not status['motor_fresh']):
                raise RuntimeError('感測器回讀曾中斷或已失效，請重新確認校正')
            if calibrate:
                self.calibration_epoch = epoch
            self.progress('開始執行設定的動作。輸入 s 再按 Enter，或按 Ctrl+C 可停止。')
            self._run('rinbo_fsm', 'rinbo_manual', '--plan', path, '--execute', timeout=600)
            self.probe().wait_motor_disabled(self.cancel)
            self.progress('動作結束，正在關閉馬達電源 …')
            self.power('sensors', cancellable=False)
            self.progress('馬達電源已關閉。感測器保持開啟，方便下一次操作。')
            success = True
        finally:
            if not success and self.power_touched:
                self.safe_off()

    def safe_off(self):
        self.calibration_epoch = None
        try:
            self.power('off', cancellable=False)
            self.progress('已收到關電確認。')
            return True
        except BaseException as exc:
            self.progress('無法確認關電成功，請使用實體急停／切斷馬達電源。原因：'+str(exc))
            return False

    def close(self):
        okay = True
        if self.child:
            self.child.stop()
        if self.power_touched or (self.remote and self.remote.owned):
            okay = self.safe_off()
        if self.bridge:
            self.bridge.stop()  # only our own child, never another operator's PID
            okay = okay and self.bridge.process.returncode == 0
        if self.remote:
            try:
                if okay:
                    self.remote.stop()
                else:
                    self.progress('尚未確認關電／Bridge 正常退出，保留 sbRIO 通訊與程序紀錄。')
            except Exception as exc:
                okay = False
                self.progress('sbRIO 通訊停止未完成：'+str(exc))
            finally:
                try:
                    self.remote.close()
                except Exception as exc:
                    okay = False
                    self.progress('SSH 連線清理未完成：'+str(exc))
        if self.feedback:
            self.feedback.close()
        return okay
