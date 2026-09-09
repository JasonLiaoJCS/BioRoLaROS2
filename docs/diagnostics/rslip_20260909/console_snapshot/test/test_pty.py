import fcntl
import os
import io
import pty
import select
import signal
import struct
import subprocess
import termios
import time

root = os.path.dirname(os.path.abspath(__file__))
log = os.path.join(root, 'console.log')
if os.path.exists(log):
    os.unlink(log)
master, slave = pty.openpty()
def resize(rows, cols):
    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', rows, cols, 0, 0))
resize(30, 100)
env = dict(os.environ, TERM='xterm')
proc = subprocess.Popen([os.path.join(root, 'harness')], stdin=slave, stdout=slave, stderr=slave, env=env)
capture = bytearray()
def drain(seconds=.35):
    end = time.monotonic() + seconds
    output = bytearray()
    while time.monotonic() < end:
        if select.select([master], [], [], .03)[0]:
            output.extend(os.read(master, 65536))
    capture.extend(output)
    return bytes(output)
def send(text):
    os.write(master, text)
    return drain()
def fresh():
    return send(b'\x0c')
try:
    startup = drain(.6)
    assert b'Command>' in startup and b'Console v2' in startup
    assert b'TEST_LOG_NOISE' not in startup
    assert b'STATE=000' in io.open(log, 'rb').read()
    # Partial, malformed and out-of-range commands never touch fake hardware.
    send(b':\n')
    for bad in [b':P\n', b':M 9 E 1\n', b':M 0 I 999999\n', b':P D -1\n', b':P D 1 garbage\n', b':S 9 P 100\n']:
        send(bad)
        assert b'ERROR' in fresh(), bad
    assert b'WRITE' not in io.open(log, 'rb').read()
    assert b'STATE=100' not in io.open(log, 'rb').read()
    # Single-line bracketed paste is only edited until a real Enter arrives.
    send(b'\x1b[200~:P D 1\x1b[201~')
    assert b'STATE=100' not in io.open(log, 'rb').read()
    send(b'\n')
    assert b'STATE=100' in io.open(log, 'rb').read()
    # Multiline paste is rejected; no power command can be smuggled through it.
    send(b'\x1b[200~:P P 1\n:P S 1\n\x1b[201~')
    send(b'\n')
    assert b'STATE=101' not in io.open(log, 'rb').read() and b'STATE=111' not in io.open(log, 'rb').read()
    send(b':P S 0\x7f1\n')
    assert b'STATE=110' in io.open(log, 'rb').read()
    # Cancel leaves hardware unchanged.
    send(b':P P 1\x1b')
    send(b'\n')
    assert b'STATE=111' not in io.open(log, 'rb').read()
    # Resize down/up keeps a visible input field and no detached input thread.
    resize(12, 50)
    proc.send_signal(signal.SIGWINCH)
    drain()
    small = fresh()
    assert b'Window too small' in small and b'Command>' in small
    resize(30, 100)
    proc.send_signal(signal.SIGWINCH)
    drain()
    assert b'Command>' in fresh()
    send(b':M 2 I 100\n')
    assert b'WRITE I 2 100' in io.open(log, 'rb').read()
    assert b'TEST_LOG_NOISE' not in capture
    proc.send_signal(signal.SIGINT)
    proc.wait(timeout=4)
    ending = drain()
    assert proc.returncode == 0 and b'HARNESS_EXIT_OK' in ending
    io.open(os.path.join(root, 'terminal_capture.bin'), 'wb').write(capture)
    print('PASS: visible input, log isolation, validation, paste, editing, resize, shutdown')
finally:
    if proc.poll() is None:
        proc.kill()
        proc.wait()
    os.close(master)
    os.close(slave)
# No terminal: no curses sequences or input thread, including unknown TERM.
headless = subprocess.Popen([os.path.join(root, 'harness')], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=dict(os.environ, TERM='unknown'))
time.sleep(.3)
headless.send_signal(signal.SIGINT)
out, err = headless.communicate(timeout=4)
assert headless.returncode == 0 and b'\x1b' not in out and b'HARNESS_EXIT_OK' in out
print('PASS: headless operation without a terminal')
