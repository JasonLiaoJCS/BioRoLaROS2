#!/usr/bin/env python3
"""Windows SSH transport. No ROS imports, process ownership, or robot rules."""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rinbo_control'))
from rinbo_control.panel_protocol import SOCKET, decode, result


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--request-base64', required=True)
    parser.add_argument('--socket', type=Path, default=None, help='local socket override; disables service bootstrap')
    args = parser.parse_args(argv)
    endpoint = args.socket or SOCKET
    request = {}
    try:
        request = decode(args.request_base64)
        # Explicit Communications may start the inert owner. Read-only requests
        # never start a service, ROS node, or reconnect routine.
        with socket.socket(socket.AF_UNIX) as connection:
            connection.settimeout(10)
            try:
                connection.connect(str(endpoint))
            except (FileNotFoundError, ConnectionRefusedError):
                if args.socket is not None or request['action'] != 'Communications':
                    raise
                # A crash can leave a stale socket file. No request was sent;
                # explicit Communications may start the inert owner once.
                start = subprocess.run(['systemctl', '--user', 'start', 'rinbo-panel.service'],
                                       capture_output=True, text=True, timeout=8)
                if start.returncode:
                    raise RuntimeError('controller_start_failed: '+start.stderr.strip())
                end = time.monotonic()+3
                while True:
                    try:
                        connection.connect(str(endpoint))
                        break
                    except (FileNotFoundError, ConnectionRefusedError):
                        if time.monotonic() >= end:
                            raise RuntimeError('controller_not_ready: native console may own the lock; inspect rinbo-panel.service')
                        time.sleep(.05)
            connection.sendall((json.dumps(request)+'\n').encode())
            with connection.makefile('r', encoding='utf-8') as reader:
                if request.get('stream'):
                    for line in reader:
                        print(line, end='', flush=True)
                    raise RuntimeError('monitor_disconnected: reconnect read-only stream')
                line = reader.readline()
                if not line:
                    raise RuntimeError('controller_reply_lost: query ReadLogs; do not automatically retry an action')
                response = json.loads(line)
                print(json.dumps(response, ensure_ascii=False), flush=True)
                if response['exit_code'] and response.get('reason'):
                    print(response['reason'], file=sys.stderr, flush=True)
                return response['exit_code']
    except (FileNotFoundError, ConnectionRefusedError):
        if request.get('action') == 'ReadLogs' and request.get('stream'):
            # Service may be absent before the operator connects. Emit a clear
            # state once; Windows reconnects the read-only stream on termination.
            print('RSLIP_ACTION_CONTEXT=GUI', flush=True)
            print('RSLIP_ACTION_STATUS={"kind":"GUI","status":"controller_unavailable"}', flush=True)
            print('RSLIP_MONITOR_HEARTBEAT=1', flush=True)
        response = result(request, 'not_sent', 10, 'controller_unavailable: use Communications; no robot command sent')
    except (ValueError, RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
        response = result(request, 'not_sent' if not request else 'state_unknown', 2 if not request else 31, str(exc))
    print(json.dumps(response, ensure_ascii=False), flush=True)
    print(response['reason'], file=sys.stderr, flush=True)
    return response['exit_code']


if __name__ == '__main__':
    raise SystemExit(main())
