"""Linux parent-death protection, installed before exec without preexec_fn."""
import ctypes
import os
import signal
import sys


def main():
    expected_parent = int(sys.argv[1])
    command = sys.argv[2:]
    if not command:
        raise SystemExit('No child command')
    # Ignored dispositions survive exec (e.g. a launcher that ignores Ctrl+C).
    # Our exit notification must still reach the controller's own handler.
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGINT, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        raise OSError(ctypes.get_errno(), 'Cannot install parent-death protection')
    if os.getppid() != expected_parent:
        raise SystemExit('Operator console exited before child startup')
    os.execvpe(command[0], command, os.environ)


if __name__ == '__main__':
    main()
