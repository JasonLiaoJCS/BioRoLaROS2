"""R-Slip v6.6 sbRIO communication bootstrap, through the user's OpenSSH TTY.

No password handling, power commands, remote pkill, or remote motion commands.
Remote receipts bind owned services to boot ID, PID, start ticks and executable.
"""
import json
from pathlib import Path
import re
import shlex
import subprocess
import tempfile
import time
import uuid

from .plans import atomic_json, ipv4

BITFILE_SHA256 = '78975be61bf8b65db6744835626fdb071e41b45c3d8d8cd29065cb0e21e762f7'

# Keep this script self-contained: NI Linux RT need not have Python installed.
SCRIPT = r'''
set -eu
umask 077
action=$1
ip=$2
port=$3
token=$4
expected_hash=$5
export LD_LIBRARY_PATH=/home/admin/rinbo_sbRIO_ws/install/lib:/home/admin/kilin_sbRIO_ws/install/lib
export PATH=/home/admin/rinbo_sbRIO_ws/install/bin:/home/admin/kilin_sbRIO_ws/install/bin:$PATH
export TERM=xterm
export CORE_LOCAL_IP="$ip"
export CORE_MASTER_ADDR="$ip:$port"
root=/home/admin/.local/state/rinbo-control
state="$root/$token"
core=/home/admin/rinbo_sbRIO_ws/install/bin/grpccore
work=/home/admin/rinbo_sbRIO_ws/rinbo_fpga_driver/build
driver="$work/fpga_driver"
bitfile="$work/NiFpga_FPGA_POWER_RS485_v2.lvbitx"
fail() { echo "sbRIO: $*" >&2; exit 30; }
command -v flock >/dev/null || fail '缺少 flock，無法排除同時啟動；尚未啟動服務。'
mkdir -p "$state"
exec 9>"$root/services.lock"
flock -n 9 || fail '另一個 sbRIO 啟動／停止正在進行，請稍後重試。'
boot=$(cat /proc/sys/kernel/random/boot_id)
if [ -e "$state/core.starting" ] || [ -e "$state/driver.starting" ]; then
    fail '上次啟動的程序身分尚未完整記錄，需人工檢查；不會猜測 PID 或重複啟動。'
fi
ticks() { sed 's/^.*) //' "/proc/$1/stat" | awk '{print $20}'; }
alive() {
    [ -r "/proc/$1/stat" ] || return 1
    [ "$(sed 's/^.*) //' "/proc/$1/stat" | awk '{print $1}')" != Z ]
}
identity() {
    # record fields are written only by this script, never evaluated as shell.
    read rboot rpid rticks rexe < "$state/$1.pid" || fail '程序紀錄損壞。'
    case "$rpid:$rticks" in *[!0-9:]*|:*|*:) fail '程序紀錄的 PID／啟動時間不合法。' ;; esac
    [ "$rpid" -gt 1 ] || fail '拒絕不合法的程序 PID。'
    case "$1" in
        core) expected_exe=$(readlink -f "$core") ;;
        driver) expected_exe=$(readlink -f "$driver") ;;
        *) fail '未知程序種類。' ;;
    esac
    [ "$rexe" = "$expected_exe" ] || fail '程序紀錄不屬於指定的 core／driver。'
    [ "$rboot" = "$boot" ] || return 1
    alive "$rpid" || return 1
    [ "$(ticks "$rpid")" = "$rticks" ] || fail 'PID 已被重用；不會停止不明程序。'
    [ "$(readlink "/proc/$rpid/exe")" = "$rexe" ] || fail '程序執行檔已改變。'
}
find_service() {
    found=''
    for entry in /proc/[0-9]*/exe; do
        if ! exe=$(readlink "$entry" 2>/dev/null); then
            comm=${entry%/exe}/comm
            if [ -r "$comm" ] && [ "$(cat "$comm")" = "$1" ]; then
                fail "$1 存在但無法核對執行檔；不會重複啟動。"
            fi
            continue
        fi
        case "$exe" in
            */"$1"|*/"$1 (deleted)")
                pid=${entry#/proc/}; pid=${pid%/exe}
                alive "$pid" || continue
                [ -z "$found" ] || fail "發現多份 $1；請先確認既有程序。"
                [ "$exe" = "$2" ] || fail "$1 使用了不同執行檔；不會覆蓋或停止它。"
                found=$pid ;;
        esac
    done
}
check_env() {
    tr '\000' '\n' < "/proc/$1/environ" | grep -Fx "CORE_LOCAL_IP=$ip" >/dev/null || fail '既有服務的 CORE_LOCAL_IP 不符，請先處理舊服務。'
    tr '\000' '\n' < "/proc/$1/environ" | grep -Fx "CORE_MASTER_ADDR=$ip:$port" >/dev/null || fail '既有服務的 CORE_MASTER_ADDR 不符，請先處理舊服務。'
}
listening() {
    netstat -ltn | awk -v port="$port" '$6 == "LISTEN" && $4 ~ (":" port "$") {ok=1} END {exit !ok}'
}
save_identity() {
    st=$(ticks "$2") || fail '啟動後無法讀取程序身分。'
    printf '%s %s %s %s\n' "$boot" "$2" "$st" "$3" > "$state/$1.pid.tmp"
    mv "$state/$1.pid.tmp" "$state/$1.pid"
}
stop_owned() {
    [ -f "$state/$1.pid" ] || return 0
    if ! identity "$1"; then
        rm "$state/$1.pid"
        return 0
    fi
    target=$rpid
    kill -INT "$target" || fail "無法停止 $1。"
    n=0
    while alive "$target"; do
        n=$((n+1))
        # Non-interactive shell background jobs may inherit ignored SIGINT.
        # Power is already confirmed off; TERM can stop an owned service.
        if [ "$n" -eq 4 ]; then
            identity "$1" || break
            kill -TERM "$target" || fail "無法終止 $1。"
        fi
        [ "$n" -le 8 ] || fail "$1 未退出；保留通訊並回報停止未完成。"
        sleep 1
        identity "$1" || break
    done
    rm "$state/$1.pid"
    echo "sbRIO: 已停止本操作台啟動的 $1。"
}
case "$action" in
start)
    [ -x "$core" ] && [ -x "$driver" ] || fail '找不到 SOP 指定的 grpccore／fpga_driver，尚未啟動。'
    [ -f "$bitfile" ] || fail '找不到 SOP 指定的 FPGA bitfile。'
    actual=$(sha256sum "$bitfile" | awk '{print $1}')
    [ "$actual" = "$expected_hash" ] || fail "FPGA SHA-256 不符：$actual；不會啟動 driver。"
    echo 'sbRIO: FPGA SHA-256 符合 v6.6。'
    core=$(readlink -f "$core")
    driver=$(readlink -f "$driver")
    find_service grpccore "$core"; core_pid=$found
    find_service fpga_driver "$driver"; driver_pid=$found
    if [ -n "$driver_pid" ] && [ -z "$core_pid" ]; then
        fail 'driver 還在但 core 已退出；請先處理舊通訊，不自動重啟。'
    fi
    # Validate all existing services before changing anything.
    if [ -n "$core_pid" ]; then check_env "$core_pid"; fi
    if [ -n "$driver_pid" ]; then
        check_env "$driver_pid"
        [ "$(readlink "/proc/$driver_pid/cwd")" = "$work" ] || fail '既有 driver 工作目錄不符。'
    fi
    # A partial previous start needs explicit inspection, not a duplicate launch.
    for name in core driver; do
        if [ -f "$state/$name.pid" ]; then
            identity "$name" || fail "上次 $name 已退出；請先完成停止再重試。"
        fi
    done
    if [ -z "$core_pid" ]; then
        listening && fail '通訊埠已被其他程序使用。'
        core_log=$(mktemp "$state/core-log.XXXXXX")
        touch "$state/core.starting"
        nohup "$core" 9>&- </dev/null >"$core_log" 2>&1 &
        core_pid=$!
        save_identity core "$core_pid" "$core"
        rm "$state/core.starting"
        echo "sbRIO: 啟動 core PID=$core_pid；日誌 $core_log"
    else
        echo "sbRIO: 沿用 core PID=$core_pid，不重啟。"
    fi
    n=0
    until listening; do
        alive "$core_pid" || fail "core 已退出，請查看 $state/core-log.*。"
        n=$((n+1)); [ "$n" -le 8 ] || fail 'core 未開啟通訊埠。'
        sleep 1
    done
    if [ -z "$driver_pid" ]; then
        driver_log=$(mktemp "$state/driver-log.XXXXXX")
        printf '%s\n' "$driver_log" > "$state/driver.logpath"
        cd "$work"
        touch "$state/driver.starting"
        nohup "$driver" 9>&- </dev/null >"$driver_log" 2>&1 &
        driver_pid=$!
        save_identity driver "$driver_pid" "$driver"
        rm "$state/driver.starting"
        echo "sbRIO: 啟動 driver PID=$driver_pid；日誌 $driver_log"
    else
        echo "sbRIO: 沿用 driver PID=$driver_pid，不重啟；接著仍須驗證 ROS 新回讀。"
    fi
    if [ -f "$state/driver.pid" ]; then
        driver_log=$(cat "$state/driver.logpath")
        n=0
        while :; do
            alive "$driver_pid" || fail "driver 已退出，請查看 $driver_log。"
            if grep -Eq 'Open Failed|-63101' "$driver_log"; then
                tail -n 40 "$driver_log" >&2
                fail 'FPGA 開啟失敗，不可上電或執行動作。'
            fi
            if grep -F 'Session opened (Success)' "$driver_log" >/dev/null; then break; fi
            n=$((n+1)); [ "$n" -le 10 ] || fail "沒看到 Session opened (Success)，請查看 $driver_log。"
            sleep 1
        done
        echo 'sbRIO: Session opened (Success) 已確認。'
    fi
    if [ -f "$state/core.pid" ] || [ -f "$state/driver.pid" ]; then
        echo RINBO_OWNED=1
    else
        echo RINBO_OWNED=0
    fi
    echo RINBO_SBRIO_READY
    ;;
stop)
    # Caller must first confirm motor-safe/all-off, then stop its own Bridge.
    for name in driver core; do
        if [ -f "$state/$name.pid" ]; then identity "$name" || :; fi
    done
    stop_owned driver
    find_service fpga_driver "$(readlink -f "$driver")"
    if [ -n "$found" ] && [ -f "$state/core.pid" ]; then
        fail '另有 driver 仍在使用 core，保留 core；請由原操作者完成停止。'
    fi
    stop_owned core
    echo RINBO_SBRIO_STOPPED
    ;;
*) fail '不支援的操作。' ;;
esac
'''


class RemoteServices:
    def __init__(self, state_dir, progress=print, log_dir=None):
        self.record = Path(state_dir)/'sbrio-session.json'
        self.log_dir = Path(log_dir) if log_dir else Path(state_dir)/'logs'
        self.progress = progress
        self.transport = tempfile.TemporaryDirectory(prefix='rinbo-ssh-')
        self.control_path = str(Path(self.transport.name)/'cm-%C')
        self.owned = False
        self.session = None

    def _load(self, ip, port):
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError('sbRIO 通訊埠須為 1～65535 的整數')
        if self.record.exists():
            value = json.loads(self.record.read_text())
            if (set(value) != {'ip','port','token'} or
                    not re.fullmatch('[0-9a-f]{32}', value.get('token',''))):
                raise RuntimeError('sbRIO 程序紀錄損壞，請先檢查；不會猜測程序身分')
            if value['ip'] != ip or value['port'] != port:
                raise RuntimeError('尚有另一個 sbRIO 位址的通訊紀錄，請先用原位址完成停止')
        else:
            value = dict(ip=ipv4(ip), port=port, token=uuid.uuid4().hex)
            atomic_json(self.record, value)  # before any remote mutation
        self.session = value

    def _command(self, action):
        s = self.session
        remote = shlex.join(['sh', '-s', '--', action, s['ip'], str(s['port']),
                             s['token'], BITFILE_SHA256])
        return ['ssh', '-T', '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=5',
                '-o', 'ServerAliveCountMax=3', '-o', 'StrictHostKeyChecking=ask',
                '-o', 'ControlMaster=auto', '-o', 'ControlPersist=300',
                '-o', f'ControlPath={self.control_path}', f'admin@{s["ip"]}', remote]

    def _run(self, action):
        self.progress('正在登入 sbRIO；若出現密碼提示，直接在此輸入 sbRIO 密碼（不會顯示或儲存）。')
        log = self.log_dir/f'sbrio-{time.time_ns()}.json'
        atomic_json(log, dict(action=action, status='started', **self.session))
        try:
            result = subprocess.run(self._command(action), input=SCRIPT, text=True,
                                    stdout=subprocess.PIPE, timeout=120)
        except BaseException:
            atomic_json(log, dict(action=action, status='interrupted-or-unknown', **self.session))
            raise
        atomic_json(log, dict(action=action, returncode=result.returncode,
                             output=result.stdout, **self.session))
        self.progress(result.stdout.strip())
        marker = 'RINBO_SBRIO_READY' if action == 'start' else 'RINBO_SBRIO_STOPPED'
        if result.returncode or marker not in result.stdout.splitlines():
            raise RuntimeError('sbRIO 通訊處理未完成，請看上方第一個錯誤；不會繼續上電。'
                               '已啟動的程序保留身分紀錄，請用原位址重試或完成停止。')
        return result.stdout

    def start(self, ip, port):
        self._load(ipv4(ip), port)
        self.owned = True  # failure/SSH loss may leave an owned service
        output = self._run('start')
        self.owned = 'RINBO_OWNED=1' in output.splitlines()

    def stop(self):
        if self.session and self.owned:
            self._run('stop')
        self.record.unlink(missing_ok=True)
        self.owned = False

    def close(self):
        try:
            if self.session:
                subprocess.run(['ssh', '-S', self.control_path, '-O', 'exit',
                                f'admin@{self.session["ip"]}'], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, timeout=5)
        finally:
            self.transport.cleanup()
