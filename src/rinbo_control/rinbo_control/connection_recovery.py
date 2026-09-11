"""Explicit step-1 housekeeping. No power, network reset, or service startup.

Only current-user native motion controllers in the current ROS domain qualify.
pidfds bind SIGINT to the inspected process, including across PID reuse.
"""
from contextlib import contextmanager
from dataclasses import asdict, dataclass
import fcntl
import os
from pathlib import Path
import select
import signal
import time

from .plans import atomic_json

MOTION_NAMES = ('rinbo_cali', 'rinbo_standing', 'rinbo_tripod', 'rinbo_manual')


@contextmanager
def recovery_lock(state_dir):
    path = Path(state_dir)/'connection-recovery.lock'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('另一個控制台正在整理連線，請等它完成再按 1。') from exc
        yield


@dataclass(frozen=True)
class Process:
    pid: int
    start_ticks: int
    exe: str
    uid: int
    domain: str


def read_process(pid, proc_root=Path('/proc')):
    directory = proc_root/str(pid)
    # Check start ticks on both sides of reading executable/environment.
    before = (directory/'stat').read_text().rsplit(') ', 1)[1].split()
    if before[0] == 'Z':
        raise ProcessLookupError(pid)
    uid = directory.stat().st_uid
    exe = os.readlink(directory/'exe')
    environment = (directory/'environ').read_bytes().split(b'\0')
    domain = next((v.split(b'=', 1)[1].decode() for v in environment
                   if v.startswith(b'ROS_DOMAIN_ID=')), '0')
    after = (directory/'stat').read_text().rsplit(') ', 1)[1].split()
    if before[19] != after[19] or after[0] == 'Z':
        raise ProcessLookupError(pid)
    return Process(pid, int(before[19]), exe, uid, domain)


def find_motion(executables, domain, proc_root=Path('/proc')):
    allowed = {str(Path(p).resolve()) for p in executables}
    result = []
    for directory in proc_root.iterdir():
        if not directory.name.isdecimal():
            continue
        try:
            if directory.stat().st_uid != os.getuid():
                continue
            if os.readlink(directory/'exe') not in allowed:
                continue
            process = read_process(int(directory.name), proc_root)
            if process.exe not in allowed or process.domain != str(domain):
                continue
            args = (directory/'cmdline').read_bytes().split(b'\0')
            if any(flag in args for flag in (b'--help', b'--check-config', b'--check-ready', b'--version')):
                continue
            if Path(process.exe).name == 'rinbo_manual' and b'--execute' not in args:
                continue  # Plan validation/CSV preview is an offline query.
            result.append(process)
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            # Unknown publishers are caught separately by the ROS idle gate.
            continue
    return result


def stop_motion(processes, log_dir, progress, timeout=8):
    if not processes:
        return
    if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
        raise RuntimeError('此系統不支援核對程序身分的停止方式；請先用原控制台停止動作。')
    log = Path(log_dir)/f'connection-stop-{time.time_ns()}.json'
    events = []
    atomic_json(log, dict(processes=[asdict(p) for p in processes], events=events))
    for process in processes:
        fd = None
        try:
            fd = os.pidfd_open(process.pid)
            if read_process(process.pid) != process:
                raise RuntimeError(f'PID {process.pid} 身分已改變；本次沒有對它送停止訊號。')
            progress(f'自動整理：正常停止 {Path(process.exe).name} PID={process.pid} …')
            signal.pidfd_send_signal(fd, signal.SIGINT)
            events.append(dict(pid=process.pid, event='SIGINT'))
            atomic_json(log, dict(processes=[asdict(p) for p in processes], events=events))
            poller = select.poll()
            poller.register(fd, select.POLLIN)
            if not poller.poll(int(timeout*1000)):
                raise RuntimeError(f'PID {process.pid} 在 {timeout:g} 秒內未正常停止；'
                                   '保留通訊，請用原控制台停止動作後再按 1。')
            events.append(dict(pid=process.pid, event='exited'))
        except (ProcessLookupError, FileNotFoundError):
            events.append(dict(pid=process.pid, event='already-exited'))
        except BaseException as exc:
            events.append(dict(pid=process.pid, event='incomplete', reason=str(exc)))
            raise
        finally:
            if fd is not None:
                os.close(fd)
            atomic_json(log, dict(processes=[asdict(p) for p in processes], events=events))
