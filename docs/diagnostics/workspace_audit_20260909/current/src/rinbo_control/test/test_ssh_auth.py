import os
from pathlib import Path
import subprocess
import sys

import pytest

from rinbo_control.ssh_auth import automatic_auth, password_path, read_password
from rinbo_control.sbrio import RemoteServices, SCRIPT


def credential(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    path = password_path('192.168.30.254')
    path.parent.mkdir(parents=True, mode=0o700)
    path.write_text('test-only-secret\n')
    path.chmod(0o600)
    return path


def test_credentials_are_host_specific_and_private(tmp_path, monkeypatch):
    path = credential(tmp_path, monkeypatch)
    assert automatic_auth('192.168.30.2', tmp_path) == ([], None)
    assert read_password(path) == 'test-only-secret'
    path.chmod(0o644)
    with pytest.raises(RuntimeError, match='600'):
        automatic_auth('192.168.30.254', tmp_path)
    path.unlink()
    target = tmp_path/'target'
    target.write_text('test-only-secret')
    target.chmod(0o600)
    path.symlink_to(target)
    with pytest.raises(OSError):
        read_password(path)


def test_askpass_answers_password_only_and_never_accepts_host_key(tmp_path, monkeypatch):
    credential(tmp_path, monkeypatch)
    options, env = automatic_auth('192.168.30.254', tmp_path)
    env['PYTHONPATH'] = str(Path(__file__).resolve().parents[1])
    assert 'StrictHostKeyChecking=yes' in options
    assert 'test-only-secret' not in repr(options) + repr(env)
    for prompt in ("admin@192.168.30.254's password: ", '(admin@192.168.30.254) Password:'):
        result = subprocess.run([env['SSH_ASKPASS'], prompt], env=env, capture_output=True, text=True)
        assert result.returncode == 0 and result.stdout == 'test-only-secret\n'
        assert result.stderr == ''
    for prompt in ('Are you sure you want to continue connecting (yes/no)?', 'Enter passphrase for key:', 'Verification code:'):
        result = subprocess.run([env['SSH_ASKPASS'], prompt], env=env, capture_output=True, text=True)
        assert result.returncode != 0 and not result.stdout
    env['SSH_ASKPASS_PROMPT'] = 'confirm'
    result = subprocess.run([env['SSH_ASKPASS'], 'Password:'], env=env, capture_output=True, text=True)
    assert result.returncode != 0 and not result.stdout


def test_bootstrap_keeps_remote_script_stdin_and_password_out_of_logs(tmp_path, monkeypatch):
    credential(tmp_path, monkeypatch)
    calls = []
    def run(command, **kwargs):
        calls.append((command, kwargs))
        from types import SimpleNamespace
        return SimpleNamespace(returncode=0, stdout='RINBO_OWNED=1\nRINBO_SBRIO_READY\n')
    monkeypatch.setattr(subprocess, 'run', run)
    messages = []
    remote = RemoteServices(tmp_path/'state', messages.append)
    try:
        remote.start('192.168.30.254', 50051)
        command, kwargs = calls[0]
        assert kwargs['input'] == SCRIPT
        assert kwargs['env']['SSH_ASKPASS_REQUIRE'] == 'force'
        assert 'StrictHostKeyChecking=yes' in command
        assert 'test-only-secret' not in repr(calls) + repr(messages)
        for path in (tmp_path/'state').rglob('*.json'):
            assert 'test-only-secret' not in path.read_text()
    finally:
        remote.close()
