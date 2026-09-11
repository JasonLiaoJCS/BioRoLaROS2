"""Persistent GUI adapter around the same Runtime used by robot.sh.

Only the controller worker touches Runtime. The urgent lane may send all-off
and interrupt an enabling CLI; it never creates another Runtime or enables.
"""
import json
import os
import signal
import subprocess
import threading
import time
import uuid
from pathlib import Path

from .runtime import Runtime
from .connection_recovery import MOTION_NAMES, find_motion, stop_motion


class NativeFailure(RuntimeError):
    def __init__(self, reason, code=20, data=None):
        super().__init__(reason)
        self.code, self.data = code, data


class Native:
    def __init__(self, root, emit):
        self.root, self.emit = Path(root), emit
        self.cancel = threading.Event()
        self.runtime = Runtime(root, lambda line: emit('GUI', line), self.cancel.is_set, noninteractive=True)
        self.launch_lock = threading.RLock()
        self.power_child = None
        self.motion = None
        self.motion_kind = None
        self.motion_id = None
        self.motion_ready = False
        self.target = None
        self.power_observation = None
        self.power_subscription = None
        self.finish = lambda *args: None

    def bind(self, req):
        target = (req['sbrio_ip'], req['orin_ip'], req['ros_domain_id'])
        if self.target and target != self.target:
            raise NativeFailure('target_changed: complete Stop at the original address before switching')
        if target[2] != int(os.environ.get('ROS_DOMAIN_ID', '99')):
            raise NativeFailure('domain_mismatch: controller ROS_DOMAIN_ID is fixed for its lifetime')
        self.target = target

    def check_cancel(self):
        if self.cancel.is_set():
            raise NativeFailure('preempted_by_stop', 40)

    def power(self, mode, request_id=None, urgent=False, assert_estop=False):
        r = self.runtime
        request_id = request_id or uuid.uuid4().hex
        args = [mode, '--json', '--wait-for-subscriber-s', '8', '--request-id',
                request_id]
        if assert_estop:
            args.append('--assert-estop')
        if mode == 'ensure-on':
            args.append('--confirm-relay')
        with self.launch_lock:
            if not urgent:
                self.check_cancel()
            child = r._start('redrhex_lowlevel_bridge', 'rinbo_power_tool', *args)
            if not urgent:
                self.power_child = child
        deadline = time.monotonic() + (90 if mode == 'ensure-on' else 30)
        payload = None
        try:
            while True:
                for line in child.drain():
                    self.emit('GUI', line)
                    try:
                        value = json.loads(line)
                        if value.get('request_id') == request_id:
                            if 'exit_code' in value:
                                payload = value
                    except (ValueError, AttributeError):
                        pass
                code = child.process.poll()
                if code is not None:
                    child.reader.join(1)
                    # Reader tail survives bounded UI queue overflow.
                    for line in child.tail.splitlines():
                        try:
                            value = json.loads(line)
                            if 'exit_code' in value and value.get('operation') == mode and value.get('request_id') == request_id:
                                payload = value
                        except (ValueError, TypeError):
                            pass
                    if payload is None:
                        raise NativeFailure('native_result_missing: '+child.tail, code or 31)
                    if code or payload['exit_code']:
                        raise NativeFailure(payload.get('reason', 'native refusal'), code or payload['exit_code'], payload)
                    if mode not in ('ready', 'status'):
                        r.power_touched = mode != 'off'
                    return payload
                if time.monotonic() > deadline:
                    raise NativeFailure('native_power_deadline: state unknown', 31)
                time.sleep(.03)
        finally:
            child.stop()
            with self.launch_lock:
                if self.power_child is child:
                    self.power_child = None

    def interrupt(self, action):
        # Fence new starts before returning to the socket handler. Signals target
        # only our current child group; external motions use native pidfd checks.
        with self.launch_lock:
            self.cancel.set()
            children = [self.power_child]
            if action != 'StopMotion':
                children.append(self.motion)
            for child in children:
                if child and child.process.poll() is None:
                    try:
                        os.killpg(child.process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass

    def urgent_off(self, request_id, emergency=False):
        return self.power('off', request_id, urgent=True, assert_estop=emergency)

    def stop_motion(self, tolerate_exit_failure=False):
        r = self.runtime
        prior_failure = None
        processes = find_motion([r.executable('rinbo_fsm', name) for name in MOTION_NAMES],
                                r.env.get('ROS_DOMAIN_ID', '99'))
        stop_motion(processes, r.log_dir, lambda line: self.emit('GUI', line))
        if self.motion:
            child = self.motion
            child.process.wait(timeout=1)
            child.reader.join(1)
            code = child.process.returncode
            failed = child.fatal.is_set() or code != 0
            self.finish(self.motion_id, 'failed' if failed else 'stopped',
                        code or (20 if failed else 0), (child.first_failure or child.tail) if failed else '', None)
            self.motion = None
            self.motion_kind = self.motion_id = None
            if failed:
                prior_failure = child.first_failure or child.tail
                if not tolerate_exit_failure:
                    raise NativeFailure('native_motion_stop_failed: '+prior_failure, code or 20)
        r.idle()
        r.probe().wait_motor_disabled()
        return {'motor_output_enabled': False, 'verified': True, 'prior_motion_failure': prior_failure}

    def poll(self):
        if not self.motion:
            return
        child, kind, rid = self.motion, self.motion_kind, self.motion_id
        for line in child.drain():
            self.emit(kind, line)
            ready = ((kind == 'Calibration' and 'State: DONE' in line) or
                     (kind == 'Standing' and 'HEALTHY LEGS STANDING' in line))
            if ready and not self.motion_ready:
                self.motion_ready = True
                self.finish(rid, 'holding' if kind == 'Standing' else 'native_done', 0, '', None)
        code = child.process.poll()
        if child.fatal.is_set() or code is not None:
            child.reader.join(1)
            failed = child.fatal.is_set() or code != 0 or not self.motion_ready
            # Keep first raw safety reason; do not convert an unexpected exit=0
            # into a successful calibration/standing/tripod completion.
            self.finish(rid, 'failed' if failed else 'completed',
                        code or (20 if failed else 0), (child.first_failure or child.tail) if failed else '', None)
            child.stop()
            self.motion = None
            self.motion_kind = self.motion_id = None
        elif kind == 'Calibration' and self.motion_ready:
            self.stop_motion()
            self.finish(rid, 'completed', 0, '', None)

    def execute(self, req):
        action = req['action']
        self.bind(req)
        r = self.runtime
        if action in ('Stop', 'EmergencyStop'):
            # Urgent all-off already ran independently of any blocked SSH start.
            stopped = self.stop_motion(tolerate_exit_failure=True)
            if action == 'EmergencyStop':
                return {'verified_off': True, 'estop_asserted': True, 'backend_retained': True, 'motion': stopped}
            # Adopt only verified native executables in this UID/domain after
            # fresh Off acknowledgement, never Windows PID manifests.
            bridges = find_motion([r.executable('rinbo_ros_bridge', 'rinbo_ros_bridge')],
                                  r.env.get('ROS_DOMAIN_ID', '99'))
            stop_motion(bridges, r.log_dir, lambda line: self.emit('GUI', line))
            r.bridge = None
            if r.remote is None and (self.root/'sbrio-session.json').exists():
                from .sbrio import RemoteServices
                r.remote = RemoteServices(self.root, r.progress, r.log_dir, noninteractive=True)
                r.remote._load(req['sbrio_ip'], 50051)
                r.remote.owned = True  # stop script revalidates saved native token
            # The separately verified Off must precede closing the last Bridge.
            r.power_touched = False
            if r.feedback:
                r.feedback.close()
                r.feedback = None
            # close() otherwise requests Off after Bridge was closed.
            if r.remote:
                r.remote.stop()
                r.remote.close()
                r.remote = None
            if not r.close():
                raise NativeFailure('native_stop_incomplete: see native log; ownership retained', 31)
            self.runtime = Runtime(self.root, lambda line: self.emit('GUI', line), self.cancel.is_set, noninteractive=True)
            self.target = None
            self.power_subscription = self.power_observation = None
            return {'verified_off': True, 'native_owned_services_cleanup': 'completed', 'motion': stopped}
        if action == 'StopMotion':
            return self.stop_motion(tolerate_exit_failure=True)
        self.check_cancel()
        if action == 'Communications':
            if self.motion:
                self.stop_motion()
            snapshot = r.reconnect(req['sbrio_ip'], local_ip=req['orin_ip'])
            self.check_cancel()
            self.watch_power()
            ready = self.power('ready', req['request_id'])
            return {'connection': snapshot, 'native': ready}
        if action == 'PowerOn':
            return self.power('ensure-on', req['request_id'])
        if action in ('Calibration', 'Standing', 'Tripod'):
            self.stop_motion()
            self.check_cancel()
            # Native power/feedback and MotionSession own all thresholds,
            # configuration locks and prerequisite receipts.
            r.probe().wait_power_ready(r.site()['disabled_legs'], self.cancel.is_set)
            name = {'Calibration': 'rinbo_cali', 'Standing': 'rinbo_standing', 'Tripod': 'rinbo_tripod'}[action]
            with self.launch_lock:
                self.check_cancel()
                self.motion = r._start('rinbo_fsm', name)
                self.motion_kind, self.motion_id = action, req['request_id']
                self.motion_ready = False
            return {'motion': 'started', 'completion': 'not_yet_confirmed'}
        raise NativeFailure('unsupported_action', 21)

    def watch_power(self):
        if self.power_subscription is not None:
            return
        from std_msgs.msg import String
        node = self.runtime.probe().node
        def receive(message):
            try:
                peers = node.get_publishers_info_by_topic('/rinbo/power/operation_status')
                if len(peers) != 1 or peers[0].node_name != 'rinbo_ros2_bridge' or peers[0].node_namespace != '/':
                    self.power_observation = None
                    return
                value = json.loads(message.data)
                old = self.power_observation
                if value['protocol'] != 2 or abs(node.get_clock().now().nanoseconds-value['stamp_ns']) > 350000000:
                    return
                if old and old[1]['epoch'] == value['epoch'] and old[1]['status_sequence'] >= value['status_sequence']:
                    return
                self.power_observation = (time.monotonic(), value)
            except (ValueError, KeyError, TypeError):
                self.power_observation = None
        self.power_subscription = node.create_subscription(String, '/rinbo/power/operation_status', receive, 10)

    def status(self):
        if self.runtime.feedback is None:
            return {'readiness': 'backend_unavailable', 'reason': 'communications_not_established'}
        snapshot = self.runtime.status()
        observation = self.power_observation
        snapshot['readiness'] = 'backend_unavailable'
        snapshot['reason'] = 'no_fresh_native_power_protocol_status'
        if observation and time.monotonic()-observation[0] <= .35:
            snapshot['power_protocol'] = observation[1]
            snapshot['readiness'] = observation[1]['readiness']
            snapshot['reason'] = observation[1].get('reason', '')
        return snapshot
