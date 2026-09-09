"""Single-key stop while running; restore normal line editing afterwards."""
import os
import select
import sys
import termios


class StopKeys:
    def __init__(self):
        self.fd = None
        self.saved = None

    def __enter__(self):
        if not sys.stdin.isatty():
            return self
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        mode = termios.tcgetattr(self.fd)
        # Keep ISIG so Ctrl+C still raises KeyboardInterrupt.
        mode[3] &= ~(termios.ICANON | termios.ECHO)
        mode[6][termios.VMIN] = 1
        mode[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, mode)
        return self

    def __exit__(self, *exc):
        if self.saved is not None:
            try:
                # A stop key or accidental typing must not become a menu action.
                termios.tcflush(self.fd, termios.TCIFLUSH)
            finally:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
                self.fd = self.saved = None

    def requested(self):
        if self.fd is not None:
            if select.select([self.fd], [], [], 0)[0]:
                keys = os.read(self.fd, 256)
                return not keys or any(key in b'qQsS \x1b' for key in keys)
            return False
        if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            return not line or line.strip().lower() in ('s', 'q', 'stop', '停止')
        return False


stop_keys = StopKeys()
