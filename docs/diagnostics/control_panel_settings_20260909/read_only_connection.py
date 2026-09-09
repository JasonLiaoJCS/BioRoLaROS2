"""Read-only snapshot: no Runtime.connect/close, commands, or service startup."""
import json
import os
from pathlib import Path
import time
from datetime import datetime
from rinbo_control.feedback import Feedback
from rinbo_control.plans import load_settings
from rinbo_control.runtime import route_source, tcp_probe

settings = load_settings(Path.home()/'.local/state/rinbo_control/settings.json')
ip = settings.get('ip','192.168.30.254')
local_ip = settings.get('jetson_ip','192.168.30.8')
port = settings.get('port',50051)
report = dict(time=datetime.now().astimezone().isoformat(), ros_domain=os.environ.get('ROS_DOMAIN_ID'),
              ip=ip,port=port,expected_local_ip=local_ip,read_only=True)
try:
    report['route_source'] = route_source(ip)
    report['tcp'] = tcp_probe(ip,port,2)
    feedback = Feedback()
    try:
        deadline = time.monotonic()+8
        while True:
            snapshot = feedback.status()
            if (snapshot['bridge_count']==1 and snapshot['motor_fresh'] and snapshot['power_fresh']) or time.monotonic()>=deadline:
                break
        report['feedback'] = snapshot
        if snapshot['bridge_count']==1:
            report['bridge_ip'] = feedback.bridge_ip()
    finally:
        # Only destroys this observer. Runtime.close() is deliberately not used.
        feedback.close()
except Exception as exc:
    report['error'] = str(exc)
report['communication_ok'] = bool(report.get('route_source')==local_ip and report.get('tcp',{}).get('status')=='open'
    and report.get('bridge_ip')==ip and report.get('feedback',{}).get('motor_fresh') and report.get('feedback',{}).get('power_fresh'))
Path(__file__).with_name('live_connection.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
