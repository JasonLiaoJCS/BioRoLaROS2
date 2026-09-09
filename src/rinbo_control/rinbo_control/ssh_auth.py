"""Host-specific OpenSSH askpass; credentials stay outside the workspace."""
import os
from pathlib import Path
import re
import stat
import sys

from .plans import ipv4


def password_path(ip):
    return Path.home()/'.config/rinbo_control/ssh'/f'{ipv4(ip)}.password'


def read_password(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(fd, 'r') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise RuntimeError('sbRIO 密碼檔必須屬於目前使用者，且權限為 600。')
        password = stream.read(513).rstrip('\n')
    if not password or len(password) > 512 or '\n' in password or '\r' in password:
        raise RuntimeError('sbRIO 密碼檔須為單行、非空白密碼。')
    return password


def automatic_auth(ip, transport_dir):
    path = password_path(ip)
    if not path.exists():
        return [], None
    read_password(path)  # Validate before launching SSH; never log its contents.
    helper = Path(transport_dir)/'askpass'
    helper.write_text(f'#!{sys.executable}\nfrom rinbo_control.ssh_auth import askpass_main\naskpass_main()\n')
    helper.chmod(0o700)
    env = dict(os.environ, SSH_ASKPASS=str(helper), SSH_ASKPASS_REQUIRE='force',
               RINBO_SSH_PASSWORD_FILE=str(path))
    env.setdefault('DISPLAY', ':0')
    # Unknown host keys require a normal manual SSH connection first. A
    # password must never be returned as the answer to a host-key question.
    return ['-o', 'StrictHostKeyChecking=yes', '-o', 'NumberOfPasswordPrompts=1'], env


def askpass_main():
    prompt = sys.argv[1] if len(sys.argv) == 2 else ''
    if (os.environ.get('SSH_ASKPASS_PROMPT') == 'confirm'
            or not re.search(r'\bpassword\s*(?::|for\b)', prompt, re.IGNORECASE)):
        raise SystemExit(1)
    try:
        password = read_password(os.environ['RINBO_SSH_PASSWORD_FILE'])
    except (OSError, RuntimeError, KeyError):
        raise SystemExit(1)
    sys.stdout.write(password+'\n')
