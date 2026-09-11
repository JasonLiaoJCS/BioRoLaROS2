"""Local GUI wire contract. This module has no ROS or hardware side effects."""
import base64
import ipaddress
import json
import re
from pathlib import Path

ACTIONS = {'Communications', 'PowerOn', 'Calibration', 'Standing', 'Tripod',
           'StopMotion', 'Stop', 'EmergencyStop', 'CheckConnection', 'ReadLogs',
           'FpgaConsole', 'PhysicalCleanup'}
READONLY = {'CheckConnection', 'ReadLogs'}
URGENT = {'Stop', 'EmergencyStop', 'StopMotion'}
MOTION = {'Calibration', 'Standing', 'Tripod'}
ROOT = Path.home()/'.local/state/rinbo_control'
SOCKET = ROOT/'panel.sock'
MAX_REQUEST = 8192


def validate(request):
    if not isinstance(request, dict):
        raise ValueError('request must be an object')
    allowed = {'protocol', 'request_id', 'action', 'sbrio_ip', 'orin_ip', 'ros_domain_id', 'stream'}
    if set(request)-allowed:
        raise ValueError('unknown fields; credentials and commands are not accepted')
    if type(request.get('protocol')) is not int or request['protocol'] != 1:
        raise ValueError('unsupported protocol')
    if not isinstance(request.get('request_id'), str) or not re.fullmatch('[0-9a-f]{32}', request['request_id']):
        raise ValueError('request_id must be 32 lowercase hex digits')
    if request.get('action') not in ACTIONS:
        raise ValueError('unsupported action')
    for key in ('sbrio_ip', 'orin_ip'):
        ipaddress.IPv4Address(request.get(key, ''))
    if type(request.get('ros_domain_id')) is not int or not 0 <= request['ros_domain_id'] <= 232:
        raise ValueError('ros_domain_id must be integer 0..232')
    if type(request.get('stream', False)) is not bool:
        raise ValueError('stream must be boolean')
    if request.get('stream') and request['action'] != 'ReadLogs':
        raise ValueError('only ReadLogs supports stream')
    return request


def decode(value):
    if len(value) > MAX_REQUEST*2:
        raise ValueError('request too large')
    return validate(json.loads(base64.b64decode(value, validate=True).decode('utf-8')))


def result(request, status, code=0, reason='', **extra):
    extra.setdefault('native_exit_code', code)
    return dict(protocol=1, request_id=request.get('request_id'), action=request.get('action'),
                operation_id=request.get('request_id'), status=status, reason=reason,
                exit_code=code, **extra)
