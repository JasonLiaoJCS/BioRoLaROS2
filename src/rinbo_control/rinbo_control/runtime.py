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
import re
import signal
import socket
import subprocess
import sys
import threading
import time

from .plans import atomic_json, ipv4, friendly_error, number, updated_mask, validate_plan
from .calibration_progress import CalibrationProgress

DEFAULT_SBRIO_IP = '192.168.30.254'
DEFAULT_JETSON_IP = '192.168.30.8'


class Cancelled(RuntimeError):
    pass


def calibration_wait_seconds(site):
    """Selected legs home concurrently; budget stages, not number of legs.

    Allow startup/discovery plus servo homing, Hall search, stopping and reset.
    Native per-stage deadlines remain authoritative and finish/fail earlier.
    """
    parameters = site.get('parameters', {}).get('rinbo_cali', {})
    def timeout(key, default):
        value = number(parameters.get('safety.'+key, default), 0, 600)
        if value == 0:
            raise ValueError('校正階段等待時間必須大於 0')
        return value
    return (15 + timeout('servo_homing_timeout_s', 20) + timeout('hall_search_timeout_s', 30)
            + 2 * timeout('stop_timeout_s', 5))


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
        self.first_failure = ''
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
                    if not self.first_failure:
                        self.first_failure = line
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
    def __init__(self, state_dir, progress=print, cancel=lambda: False, noninteractive=False):
        self.noninteractive = noninteractive
        self.state_dir = Path(state_dir)
        self.log_dir = self.state_dir/'logs'/f'{datetime.now():%Y%m%d-%H%M%S}-{os.getpid()}'
        self.progress, self.cancel = progress, cancel
        self.bridge = None
        self.child = None
        self.feedback = None
        self.remote = None
        self.power_touched = False
        self.calibration_epoch = None
        self.calibration_reason = '尚未檢查校正。'
        self.calibration_problem = ''
        self.env = dict(os.environ, RCUTILS_COLORIZED_OUTPUT='0', PYTHONUNBUFFERED='1')
        self.serial = 0
        self.connected_target = None

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
        try:
            return subprocess.run(self.command(package, name, *args), env=self.env,
                                  capture_output=True, text=True, timeout=8)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f'{name} 檢查等待逾時，請重新查看狀態後再試。') from exc

    def site(self):
        result = self.query('rinbo_fsm', 'rinbo_legs', 'status', '--json')
        if result.returncode:
            raise RuntimeError(friendly_error(result.stderr or result.stdout))
        return json.loads(result.stdout)

    def tune_limits(self, stage, updates, dry_run, revision):
        args = ['tune-limits', stage, '--expect-revision', str(revision)]
        if dry_run:
            args.append('--dry-run')
        args.extend(f'{key}={float(value):.17g}' for key, value in updates.items())
        result = self.query('rinbo_fsm', 'rinbo_legs', *args)
        if result.returncode:
            raise RuntimeError('設定沒有套用：' + (result.stderr or result.stdout).strip())
        payload = json.loads(result.stdout)
        if not dry_run and payload['changed'] and stage == 'calibration':
            self.calibration_epoch = None
        return payload

    def change_legs(self, operation, names=()):
        before = self.site()
        desired = updated_mask(before['disabled_legs'], operation, names)
        if desired == before['disabled_legs']:
            self.progress('名單已經是這個狀態，沒有變更。')
            return before
        # Do not stop another controller on the operator's behalf. The native
        # tool also checks running actions and takes the shared motion locks.
        self.idle()
        snapshot = self.status()
        if snapshot['bridge_count'] > 1:
            raise RuntimeError('偵測到多個 Bridge，請先關閉重複程式再修改屏蔿名單。')
        if snapshot['bridge_count'] == 1 or self.power_touched:
            self.progress('先關電，再儲存屏蔿名單；通訊保持連線。')
            if not self.safe_off():
                raise RuntimeError('尚未確認關電，屏蔿名單沒有修改。')
        # Use atomic native enable/disable operations so unrelated changes by
        # other tools are preserved. Never rewrite the site YAML from Python.
        self.calibration_epoch = None
        try:
            result = self.query('rinbo_fsm', 'rinbo_legs', operation, *names)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError('儲存屏蔿名單等待逾時；請重新整理查看實際名單。') from exc
        if result.returncode:
            raise RuntimeError(friendly_error(result.stderr or result.stdout))
        after = self.site()
        self.progress('屏蔿名單已儲存。下次執行會依新設定校正所選腳。')
        return after

    def check(self):
        for package, name in [('rinbo_fsm', 'rinbo_cali'), ('rinbo_ros_bridge', 'rinbo_ros_bridge'),
                              ('redrhex_lowlevel_bridge', 'rinbo_power_tool')]:
            self.executable(package, name)
        result = self.query('rinbo_fsm', 'rinbo_manual', '--help')
        if result.returncode or '--check-ready' not in result.stdout:
            raise RuntimeError('rinbo_manual 版本較舊，請先重新編譯 rinbo_fsm')
        result = self.query('rinbo_fsm', 'rinbo_cali', '--help')
        if result.returncode or '--plan' not in result.stdout:
            raise RuntimeError('rinbo_cali 尚未支援只校正選中腳，請先重新編譯 rinbo_fsm')
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

    def connect(self, ip, port=50051, local_ip=DEFAULT_JETSON_IP, *, verify_feedback=True, recover_owned_bridge=False):
        self.connected_target = None
        ip, local_ip = ipv4(ip), ipv4(local_ip)
        self.progress(f'檢查 sbRIO {ip}:{port} …')
        source = route_source(ip)
        if source != local_ip:
            raise RuntimeError(f'目前路由使用 Jetson {source}，設定卻是 {local_ip}；請先確認網卡／位址設定')
        self.progress(f'Jetson 網路位址正確：{source}')
        if self.remote is None:
            from .sbrio import RemoteServices
            self.remote = RemoteServices(self.state_dir, self.progress, self.log_dir, noninteractive=self.noninteractive)
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
        if count == 0 and recover_owned_bridge and self.bridge and self.bridge.process.poll() is None:
            count = self.recover_owned_bridge(feedback)
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
            if recover_owned_bridge and self.cancel():
                raise Cancelled('停止已接手，不啟動新的 Bridge')
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
        if not verify_feedback:
            # Panel's authenticated power protocol owns the fresh motor/power
            # readiness gate. This return only confirms transport/discovery.
            self.connected_target = (ip, port, local_ip)
            return {'bridge_count': 1, 'readiness': 'starting'}
        # The SOP requires actual state messages before any power command.
        end = time.monotonic()+8
        while True:
            snapshot = self.status()
            if snapshot['bridge_count'] != 1:
                raise RuntimeError('Bridge 數量改變，停止通訊檢查')
            if snapshot['motor_fresh'] and snapshot['power_fresh']:
                self.connected_target = (ip, port, local_ip)
                self.progress('通訊就緒：motor/state、power/state 都有新回讀；本次沒有上電。')
                return snapshot
            if self.cancel():
                raise Cancelled('已取消通訊檢查')
            if time.monotonic() >= end:
                raise RuntimeError('Bridge 已開，但 8 秒內未收到 motor/state 與 power/state 新回讀；請檢查 sbRIO driver 日誌。尚未上電。')

    def recover_owned_bridge(self, feedback):
        """Bounded discovery/reap of our own Child only, before one replacement."""
        deadline = time.monotonic()+5
        while feedback.bridge_count() == 0 and time.monotonic() < deadline:
            if self.cancel():
                raise Cancelled('已取消通訊恢復')
            feedback.spin(.05)
        count = feedback.bridge_count()
        if count:
            return count  # caller checks uniqueness and destination
        if self.cancel():
            raise Cancelled('已取消通訊恢復')
        self.progress('本工具的 Bridge 未恢復 DDS 探索；收尾這個已持有的程序，再建立一次通訊。')
        child = self.bridge
        child.stop()
        if child.process.poll() is None:
            raise RuntimeError('Bridge 收尾未完成，不能啟動第二份；請查看原生程序退出日誌。')
        self.bridge = None
        if self.cancel():
            raise Cancelled('已取消通訊恢復')
        return feedback.bridge_count()

    def reconnect(self, ip, port=50051, local_ip=DEFAULT_JETSON_IP, *, verify_stopped=True, verify_feedback=True, recover_owned_bridge=False):
        """Operator's explicit step 1; action startup retains the idle-only gate."""
        from .connection_recovery import MOTION_NAMES, find_motion, recovery_lock, stop_motion
        ip, local_ip = ipv4(ip), ipv4(local_ip)
        if route_source(ip) != local_ip:
            raise RuntimeError('Jetson 路由位址與設定不符；尚未整理程序，請先核對選單 6。')
        with recovery_lock(self.state_dir):
            self.connected_target = None
            feedback = self.probe()
            feedback.spin(.5)
            if feedback.bridge_count() == 1 and feedback.bridge_ip() != ip:
                raise RuntimeError('既有 Bridge 指向另一個 sbRIO；請用原位址完成停止後再切換。')
            executables = [self.executable('rinbo_fsm', name) for name in MOTION_NAMES]
            processes = find_motion(executables, self.env.get('ROS_DOMAIN_ID', '0'))
            if processes:
                self.calibration_epoch = None
            stop_motion(processes, self.log_dir, self.progress)
            # Unknown/Python/other-host publishers are never guessed from a name.
            self.idle()
            if verify_stopped and processes and feedback.bridge_count() == 1:
                feedback.wait_motor_disabled()
                self.progress('舊動作已停止，已收到新的 motor output=false 回讀。')
            if self.remote is not None:
                # Refresh only this console's private SSH master, not other logins.
                self.remote.refresh_transport()
            self.progress('自動整理：核對 sbRIO 開機／程序身分，封存過期紀錄，沿用正確的通訊。')
            snapshot = (self.connect(ip, port, local_ip) if verify_feedback and not recover_owned_bridge else
                        self.connect(ip, port, local_ip, verify_feedback=verify_feedback,
                                     recover_owned_bridge=recover_owned_bridge))
            # A controller can appear during discovery/bootstrap; do not declare
            # a clean connection based only on the earlier graph snapshot.
            self.idle()
            if verify_stopped and processes:
                self.probe().wait_motor_disabled()
            return snapshot

    def ensure_connected(self, ip, port=50051, local_ip=DEFAULT_JETSON_IP):
        # Refuse an overlapping action before SSH/bootstrap or power review.
        self.idle()
        target = (ipv4(ip), port, ipv4(local_ip))
        if self.connected_target == target:
            snapshot = self.status()
            if (snapshot['bridge_count'] == 1 and snapshot['motor_fresh']
                    and snapshot['power_fresh'] and self.probe().bridge_ip() == ip
                    and route_source(ip) == local_ip):
                self.progress('沿用目前連線：Bridge 與腳位置／電源回讀正常。')
                return snapshot
        return self.connect(*target)

    def status(self):
        return self.probe().status()

    def panel_status(self):
        return self.status()

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
        ready = result.returncode == 0 and 'CALIBRATION READY' in result.stdout
        self.calibration_problem = ''
        if not ready:
            missing = re.search(r'calibration does not cover ([LR][123])\b', result.stderr or result.stdout)
            self.calibration_problem = (f'目前的完成紀錄未涵蓋 {missing[1]}。' if missing else
                '目前沒有涵蓋本次所選腳、且符合現場設定的有效校正紀錄。')
        return ready

    def needs_calibration(self, path):
        status = self.status()
        if not status['sensors_on']:
            self.calibration_reason = '感測器尚未確認供電；上電後需建立所選腳的零點。'
        elif self.calibration_epoch is None:
            self.calibration_reason = '本次控制台尚未建立可沿用的校正狀態。'
        elif self.calibration_epoch != self.probe().sensor_epoch:
            self.calibration_reason = '腳位置或電源回讀曾中斷／更換來源，需要重新確認零點。'
        elif not status['motor_fresh'] or not status.get('power_fresh', True):
            self.calibration_reason = '目前沒有完整的新回讀，不能沿用先前校正。'
        elif not self.calibration_ready(path):
            self.calibration_reason = self.calibration_problem or '所選腳沒有有效的校正完成紀錄。'
        else:
            self.calibration_reason = '所選腳已有有效紀錄，且感測器供電與回讀持續正常；不重新尋零。'
            return False
        return True

    def _run(self, package, name, *args, timeout=30, until_calibrated=False, allow_cancel=True):
        child = self._start(package, name, *args)
        self.child = child
        deadline = time.monotonic()+timeout
        calibration_progress = CalibrationProgress() if until_calibrated else None
        try:
            while True:
                if allow_cancel and self.cancel():
                    raise Cancelled('已要求停止')
                if self.feedback:
                    self.feedback.spin(.02)
                else:
                    time.sleep(.02)
                for line in child.drain():
                    detail = calibration_progress.consume(line) if calibration_progress else None
                    if 'DISCOVERY WAIT:' in line:
                        self.progress('正在辨識 Bridge 的資料來源；尚未建立馬達命令發布者 …')
                    elif 'DISCOVERY READY:' in line:
                        self.progress('Bridge 來源已確認，接著檢查新回讀與安全握手。')
                    elif 'State: DONE' in line:
                        self.progress('所選腳校正完成，正在結束校正程式並確認交接。')
                    elif detail:
                        self.progress(detail)
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
                    self.progress('校正程式已正常結束，接著驗證完成紀錄。')
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
        """Review precedes entry; preflight is inert, powered failures go off."""
        # A preflight rejection has not started this action. In particular,
        # don't power off another controller just because this panel retained
        # sensor power after a previous completed test.
        site = self.site()
        if site['hash'] != site_hash:
            raise RuntimeError('禁用腳或安全設定已改變，請重新檢查並確認動作')
        calibration_timeout = calibration_wait_seconds(site) if calibrate else None
        self.idle()
        success = False
        try:
            status = self.status()
            if not calibrate and self.needs_calibration(path):
                raise RuntimeError('校正／感測器狀態已改變，請重新確認操作')
            self.progress('【2／5 電源】確認感測器供電，再確認馬達電源。')
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
                self.progress('【3／5 校正】只校正本次選中的腳；其他主馬達不參與，也不等待它們的零點。')
                self._run('rinbo_fsm', 'rinbo_cali', '--plan', path,
                          timeout=calibration_timeout, until_calibrated=True)
                self.idle()
                self.probe().wait_motor_disabled(self.cancel)
            else:
                self.progress('【3／5 校正】沿用已確認的校正，不重新尋零。')
            if not self.calibration_ready(path):
                raise RuntimeError('校正完成紀錄未通過檢查，停止操作')
            status = self.status()
            if (epoch != self.probe().sensor_epoch or
                    not status['sensors_on'] or not status['motor_fresh']):
                raise RuntimeError('感測器回讀曾中斷或已失效，請重新確認校正')
            if calibrate:
                self.calibration_epoch = epoch
            self.progress('【4／5 動作】開始執行設定的動作。按 Q、空白鍵、Esc 或 Ctrl+C 可停止。')
            self._run('rinbo_fsm', 'rinbo_manual', '--plan', path, '--execute', timeout=600)
            self.probe().wait_motor_disabled(self.cancel)
            self.progress('【5／5 收尾】動作結束，正在關閉馬達電源 …')
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
        observed_bridge = False
        if self.feedback:
            try:
                observed_bridge = self.status().get('bridge_count', 0) > 0
            except Exception as exc:
                self.progress('退出時無法讀取電源狀態：'+str(exc))
                okay = False
        if self.power_touched or (self.remote and self.remote.owned) or observed_bridge:
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
