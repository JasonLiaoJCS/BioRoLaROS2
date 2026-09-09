import os
import pty
import sys
import termios

import pytest

from rinbo_control.terminal import StopKeys


@pytest.mark.parametrize('key', [b'q', b'Q', b's', b'S', b' ', b'\x1b'])
def test_stop_key_works_without_enter_and_restores_terminal(monkeypatch, key):
    master, slave = pty.openpty()
    try:
        with os.fdopen(os.dup(slave), 'r') as stream:
            monkeypatch.setattr(sys, 'stdin', stream)
            before = termios.tcgetattr(slave)
            keys = StopKeys()
            with pytest.raises(RuntimeError):
                with keys:
                    assert termios.tcgetattr(slave)[3] & termios.ISIG
                    os.write(master, key)
                    assert keys.requested()
                    os.write(master, b'3\n')
                    raise RuntimeError('simulated operation failed')
            assert termios.tcgetattr(slave) == before
            assert not keys.requested()  # queued typing cannot execute the next menu item
    finally:
        os.close(master)
        os.close(slave)
