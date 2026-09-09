"""Exercise the actual remote shell with local fake ELF services, never SSH."""
import hashlib
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
from types import SimpleNamespace
import pytest
from rinbo_control.sbrio import RemoteServices, SCRIPT, BITFILE_SHA256


@pytest.fixture
def remote(tmp_path):
    # Unique names prevent matching any unrelated process in the host /proc.
    suffix = str(os.getpid())+'_'+tmp_path.name
    names = ('fakecore_'+suffix, 'fakedriver_'+suffix)
    base = tmp_path/'admin'
    work = base/'rinbo_sbRIO_ws/rinbo_fpga_driver/build'
    bindir = base/'rinbo_sbRIO_ws/install/bin'
    work.mkdir(parents=True); bindir.mkdir(parents=True)
    source = tmp_path/'fake.c'
    source.write_text(r'''
#include <arpa/inet.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
int main(int argc,char **argv) {
  signal(SIGINT,SIG_DFL);
  if(getenv("FAKE_IGNORE_INT")) signal(SIGINT,SIG_IGN);
  if(strstr(argv[0],"fakecore_")) {
    const char *addr=getenv("CORE_MASTER_ADDR");
    int fd=socket(AF_INET,SOCK_STREAM,0), opt=1;
    setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(opt));
    struct sockaddr_in a={.sin_family=AF_INET,.sin_addr.s_addr=htonl(INADDR_LOOPBACK),.sin_port=htons(atoi(strrchr(addr,':')+1))};
    if(bind(fd,(struct sockaddr*)&a,sizeof(a)) || listen(fd,1)) return 2;
  } else {
    puts(getenv("FAKE_FAIL") ? "Open Failed -63101" : "Session opened (Success)");
    fflush(stdout);
  }
  while(1) pause();
}
''')
    subprocess.run(['cc',str(source),'-o',str(bindir/names[0])],check=True)
    shutil.copy2(bindir/names[0],work/names[1])
    bitfile=work/'NiFpga_FPGA_POWER_RS485_v2.lvbitx'
    bitfile.write_bytes(b'FAKE FPGA TEST FILE - not deployable')
    digest=hashlib.sha256(bitfile.read_bytes()).hexdigest()
    script=SCRIPT.replace('/home/admin',str(base)).replace('grpccore',names[0]).replace('fpga_driver"',names[1]+'"')
    # Replace the literal process name in find_service, not its workspace path.
    script=script.replace('find_service fpga_driver ',f'find_service {names[1]} ')
    token='a'*32
    state=base/'.local/state/rinbo-control'/token
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0)); port=sock.getsockname()[1]
    def run(action='start', expected=digest, env=None, token_arg=token):
        return subprocess.run(['sh','-s','--',action,'127.0.0.1',str(port),token_arg,expected],
                              input=script,text=True,capture_output=True,timeout=30,
                              env=dict(os.environ,**(env or {})))
    try:
        yield SimpleNamespace(run=run,state=state,base=base,names=names)
    finally:
        for record in base.rglob('*.pid'):
            try:
                _,pid,_,exe=record.read_text().split()
                if os.readlink(f'/proc/{pid}/exe')==exe:
                    os.kill(int(pid),signal.SIGKILL)
            except (OSError, ValueError):
                pass


def test_bad_bitfile_never_starts_service(remote):
    result=remote.run(expected=BITFILE_SHA256)
    assert result.returncode==30 and 'SHA-256 不符' in result.stderr
    assert not list(remote.base.rglob('*.pid'))


def test_start_reuse_and_stop_keep_exact_identity(remote):
    result=remote.run()
    assert result.returncode==0, result.stdout+result.stderr
    before={p.name:p.read_text() for p in remote.state.glob('*.pid')}
    assert set(before)=={'core.pid','driver.pid'}
    assert 'Session opened (Success)' in result.stdout
    result=remote.run()
    assert result.returncode==0, result.stderr
    assert before=={p.name:p.read_text() for p in remote.state.glob('*.pid')}
    assert '沿用' in result.stdout
    result=remote.run('stop')
    assert result.returncode==0, result.stderr
    assert not list(remote.state.glob('*.pid'))


def test_another_session_reuses_but_never_stops_external_services(remote):
    assert remote.run().returncode==0
    before={p.name:p.read_text() for p in remote.state.glob('*.pid')}
    result=remote.run(token_arg='b'*32)
    assert result.returncode==0 and 'RINBO_OWNED=0' in result.stdout
    assert remote.run('stop',token_arg='b'*32).returncode==0
    for data in before.values():
        _,pid,_,exe=data.split()
        assert os.readlink(f'/proc/{pid}/exe')==exe


def test_fpga_open_failure_blocks_readiness(remote):
    result=remote.run(env={'FAKE_FAIL':'1'})
    assert result.returncode==30
    assert 'RINBO_SBRIO_READY' not in result.stdout
    assert 'FPGA 開啟失敗' in result.stderr


def test_incomplete_launch_record_never_claims_stopped(remote):
    remote.state.mkdir(parents=True)
    (remote.state/'driver.starting').touch()
    for action in ('start','stop'):
        result=remote.run(action)
        assert result.returncode==30 and '尚未完整記錄' in result.stderr
        assert 'RINBO_SBRIO_STOPPED' not in result.stdout


def test_pid_reuse_record_cannot_kill_process(remote):
    assert remote.run().returncode==0
    record=remote.state/'driver.pid'
    original=record.read_text()
    fields=original.split(); fields[2]=str(int(fields[2])+1)
    record.write_text(' '.join(fields)+'\n')
    try:
        result=remote.run('stop')
        assert result.returncode==30 and 'PID 已被重用' in result.stderr
        assert Path(f'/proc/{fields[1]}').exists()
    finally:
        record.write_text(original)


def test_owned_background_services_can_stop_when_sigint_was_ignored(remote):
    assert remote.run(env={'FAKE_IGNORE_INT':'1'}).returncode==0
    result=remote.run('stop')
    assert result.returncode==0, result.stderr
    assert not list(remote.state.glob('*.pid'))


def test_ssh_uses_native_auth_and_persists_recovery_before_start(tmp_path,monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    calls=[]
    def run(command,**kwargs):
        calls.append((command,kwargs))
        if '-O' in command:
            return SimpleNamespace(returncode=0)
        assert (tmp_path/'sbrio-session.json').exists()
        return SimpleNamespace(returncode=0,stdout='RINBO_OWNED=1\nRINBO_SBRIO_READY\n')
    monkeypatch.setattr(subprocess,'run',run)
    remote=RemoteServices(tmp_path,lambda _:None)
    try:
        remote.start('192.168.30.254',50051)
        assert remote.owned
        command,kwargs=calls[0]
        assert command[0]=='ssh' and 'StrictHostKeyChecking=ask' in command
        assert 'admin@192.168.30.254' in command
        assert kwargs['input']==SCRIPT
        assert not kwargs.get('shell') and kwargs['stderr'] == subprocess.STDOUT
        with pytest.raises(RuntimeError,match='另一個 sbRIO'):
            remote.start('192.168.30.2',50051)
        assert len(calls)==1
    finally:
        remote.close()


def test_old_boot_receipts_archive_and_reuse_windows_pair_without_ownership(remote):
    # Session b stands for a different launcher after a reboot. Session a has
    # persisted obsolete identities and a misleading old FPGA log.
    assert remote.run(token_arg='b'*32).returncode == 0
    other = remote.state.with_name('b'*32)
    before = {p.name:p.read_text() for p in other.glob('*.pid')}
    remote.state.mkdir()
    for name, value in before.items():
        fields = value.split(); fields[0] = '00000000-0000-0000-0000-000000000000'
        (remote.state/name).write_text(' '.join(fields)+'\n')
    old_log = remote.state/'old-failed.log'
    old_log.write_text('Open Failed -63101\n')
    (remote.state/'driver.logpath').write_text(str(old_log)+'\n')
    result = remote.run()
    assert result.returncode == 0, result.stderr
    assert 'RINBO_OWNED=0' in result.stdout and '自動封存' in result.stdout
    assert 'Session opened (Success) 已確認' not in result.stdout
    assert not list(remote.state.glob('*.pid'))
    assert len(list(remote.state.glob('recovered.*/*.pid'))) == 2
    assert old_log.read_text() == 'Open Failed -63101\n'
    assert remote.run('stop').returncode == 0
    assert before == {p.name:p.read_text() for p in other.glob('*.pid')}
    for value in before.values():
        _, pid, _, exe = value.split()
        assert os.readlink(f'/proc/{pid}/exe') == exe


def test_reconcile_only_never_launches_missing_services(remote):
    assert remote.run().returncode == 0
    before = [p.read_text() for p in remote.state.glob('*.pid')]
    for value in before:
        os.kill(int(value.split()[1]), signal.SIGKILL)
    import time
    time.sleep(.05)
    result = remote.run('reconcile')
    assert result.returncode == 0, result.stderr
    assert 'RINBO_SBRIO_RECONCILED' in result.stdout
    assert '啟動 core PID=' not in result.stdout and '啟動 driver PID=' not in result.stdout
    assert not list(remote.state.glob('*.pid'))
    assert len(list(remote.state.glob('recovered.*/*.pid'))) == 2


def test_old_boot_start_marker_can_be_archived_without_launch(remote):
    remote.state.mkdir(parents=True)
    marker = remote.state/'driver.starting'
    marker.write_text('00000000-0000-0000-0000-000000000000\n')
    result = remote.run('reconcile')
    assert result.returncode == 0, result.stderr
    assert not marker.exists()
    assert len(list(remote.state.glob('recovered.*/driver.starting'))) == 1
    assert not list(remote.state.glob('*.pid'))


def test_live_driver_without_core_is_not_restarted_or_stopped(remote):
    assert remote.run().returncode == 0
    core = (remote.state/'core.pid').read_text().split()
    driver = (remote.state/'driver.pid').read_text()
    os.kill(int(core[1]), signal.SIGKILL)
    import time
    time.sleep(.05)
    result = remote.run()
    assert result.returncode == 30 and 'driver 還在但 core 已退出' in result.stderr
    assert driver == (remote.state/'driver.pid').read_text()
    assert os.readlink(f'/proc/{driver.split()[1]}/exe') == driver.split()[3]


def test_recovery_refuses_reused_live_pid_identity(remote):
    assert remote.run().returncode == 0
    record = remote.state/'core.pid'
    original = record.read_text()
    fields = original.split(); fields[2] = str(int(fields[2])+1)
    record.write_text(' '.join(fields)+'\n')
    try:
        result = remote.run()
        assert result.returncode == 30 and 'PID 已被重用' in result.stderr
        assert not list(remote.state.glob('recovered.*'))
        assert os.readlink(f'/proc/{fields[1]}/exe') == fields[3]
    finally:
        record.write_text(original)


def test_remote_first_error_is_preserved_in_exception_and_log(tmp_path, monkeypatch):
    import json
    monkeypatch.setenv('HOME', str(tmp_path))
    output = 'sbRIO: hash OK\nsbRIO: specific failure\nRINBO_ERROR=specific failure\ntrailing diagnostic\n'
    monkeypatch.setattr(subprocess, 'run', lambda *a,**k: SimpleNamespace(returncode=30,stdout=output))
    remote = RemoteServices(tmp_path, lambda _:None)
    try:
        with pytest.raises(RuntimeError, match='^specific failure'):
            remote.start('192.168.30.254', 50051)
        log, = list((tmp_path/'logs').glob('sbrio-*.json'))
        assert json.loads(log.read_text())['output'] == output
    finally:
        remote.close()


def test_refresh_private_ssh_preserves_pending_service_ownership(tmp_path, monkeypatch):
    monkeypatch.setattr(subprocess, 'run', lambda *a,**k: SimpleNamespace(returncode=0))
    remote = RemoteServices(tmp_path, lambda _:None)
    try:
        remote._load('192.168.30.254', 50051)
        remote.owned = True
        original_session, original_path = dict(remote.session), remote.control_path
        remote.refresh_transport()
        assert remote.owned and remote.session == original_session
        assert remote.control_path != original_path
        assert not Path(original_path).parent.exists()
    finally:
        remote.close()


def test_corrupt_boot_id_is_not_silently_retired(remote):
    assert remote.run().returncode == 0
    record = remote.state/'core.pid'
    fields = record.read_text().split(); fields[0] = 'truncated-boot'
    record.write_text(' '.join(fields)+'\n')
    result = remote.run('reconcile')
    assert result.returncode == 30 and 'boot ID 損壞' in result.stderr
    assert record.exists() and not list(remote.state.glob('recovered.*'))
