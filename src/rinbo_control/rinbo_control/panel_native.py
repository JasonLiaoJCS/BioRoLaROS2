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
        self.off_lock = threading.Lock()
        self.off_child = None
        self.off_is_emergency = False
        self.off_results = {}
        self.stop_generation = 0
        self.emergency_required = False
        self.last_completed_stop = None
        self.motion = None
        self.motion_kind = None
        self.motion_id = None
        self.motion_ready = False
        self.target = None
        self.power_observation = None
        self.power_subscription = None
        self.power_node = None
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

    def backend_evidence(self):
        """Local observation only; absence is never a power-off assertion."""
        r = self.runtime
        bridge = r.bridge
        observation = self.power_observation
        return {'connected_target': r.connected_target,
                'owned_bridge': None if bridge is None else {
                    'pid': bridge.process.pid, 'exit_code': bridge.process.poll()},
                'feedback_initialized': r.feedback is not None,
                'remote_initialized': r.remote is not None,
                'remote_owned': bool(r.remote and r.remote.owned),
                'previous_completed_stop': self.last_completed_stop,
                'power_protocol_age_s': None if observation is None else time.monotonic()-observation[0],
                'power_protocol': None if observation is None else observation[1]}

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
            if mode not in ('ready', 'status', 'off'):
                r.power_touched = True  # A sent request can outlive a lost reply.
            child = r._start('redrhex_lowlevel_bridge', 'rinbo_power_tool', *args)
            if not urgent:
                self.power_child = child
            else:
                self.off_child, self.off_is_emergency = child, assert_estop
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
                        payload['panel_backend'] = self.backend_evidence()
                        if mode == 'off':
                            payload.update(verified_off=None, power_state='unknown',
                                           recovery_action='Communications')
                        raise NativeFailure(payload.get('reason') or f'native_power_exit_{code}: state unknown', code or payload['exit_code'], payload)
                    return payload
                if time.monotonic() > deadline:
                    raise NativeFailure('native_power_deadline: state unknown', 31)
                time.sleep(.03)
        finally:
            child.stop()
            with self.launch_lock:
                if self.power_child is child:
                    self.power_child = None
                if self.off_child is child:
                    self.off_child = None

    def interrupt(self, action):
        # Fence new starts before returning to the socket handler. Signals target
        # only our current child group; external motions use native pidfd checks.
        with self.launch_lock:
            self.cancel.set()
            self.stop_generation += 1
            children = [self.power_child]
            if action == 'EmergencyStop' and not self.off_is_emergency:
                children.append(self.off_child)
            if action != 'StopMotion':
                children.append(self.motion)
            for child in children:
                if child and child.process.poll() is None:
                    try:
                        os.killpg(child.process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass

    def urgent_off(self, request_id, emergency=False):
        # At most one native Off publisher. Emergency interrupts an ordinary
        # Off CLI before taking this lock; neither path ever enables power.
        with self.off_lock:
            value = self.power('off', request_id, urgent=True, assert_estop=emergency)
            ack = value.get('acknowledgement') or {}
            if (value.get('exit_code') != 0 or value.get('request_id') != request_id or
                    not value.get('epoch') or ack.get('request_id') != request_id or
                    ack.get('epoch') != value['epoch'] or ack.get('generation') != value.get('generation') or
                    any(ack.get(key) is not False for key in ('digital','signal','power')) or
                    ack.get('ack_kind') != 'command_correlated' or
                    (emergency and ack.get('estop_asserted') is not True)):
                raise NativeFailure('off_ack_invalid: no current correlated all-off evidence', 31, value)
            self.off_results[request_id] = value
            self.runtime.power_touched = False  # Only this authenticated Off clears it.
            return value

    def stop_motion(self, tolerate_exit_failure=False, *, verify_output=True, allow_no_motion=False, verified_power_off=None):
        r = self.runtime
        prior_failure = None
        generation = self.stop_generation
        processes = find_motion([r.executable('rinbo_fsm', name) for name in MOTION_NAMES],
                                r.env.get('ROS_DOMAIN_ID', '99'))
        had_motion = bool(processes or self.motion)
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
        if not verify_output or (allow_no_motion and not had_motion and r.feedback is None):
            return dict(status='already_satisfied' if not had_motion else 'processes_stopped',
                        completion_scope='motion_processes', processes_stopped=True,
                        no_motion=not had_motion, verified=False, hardware_output='unknown',
                        motor_output_enabled=None, power_action='none', prior_motion_failure=prior_failure)
        if verified_power_off is not None:
            # The caller holds this operation's authenticated all-rails-off ACK.
            # Do not wait a second time for a Bool from a now-disabled backend.
            r.idle()
            return dict(processes_stopped=True, verified=True, motor_output_enabled=None,
                        verified_by='current_all_off_ack', prior_motion_failure=prior_failure,
                        power_action='off')
        try:
            # wait_motor_stopped already verifies fresh disabled AND zero writers.
            evidence = r.probe().wait_motor_stopped(lambda: generation != self.stop_generation)
        except Exception as exc:
            data = dict(getattr(exc, 'data', None) or {})
            data.update(processes_stopped=True, prior_motion_failure=prior_failure,
                        panel_backend=self.backend_evidence(), power_action='none')
            raise NativeFailure(str(exc), getattr(exc, 'code', 31), data) from exc
        return dict(evidence, processes_stopped=True, no_motion=not had_motion, prior_motion_failure=prior_failure,
                    power_action='none')

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
            stop_generation = self.stop_generation
            power = self.off_results.pop(req['request_id'], None)
            if power is None:
                raise NativeFailure('off_evidence_missing: old ACK or absent backend cannot authorize cleanup', 31,
                                    {'verified_off': None, 'power_state': 'unknown',
                                     'panel_backend': self.backend_evidence(), 'recovery_action': 'Communications'})
            # Urgent all-off already ran independently of any blocked SSH start.
            stopped = self.stop_motion(tolerate_exit_failure=True, verified_power_off=power)
            if action == 'EmergencyStop':
                return {'verified_off': True, 'estop_asserted': True, 'backend_retained': True, 'motion': stopped}
            if stop_generation != self.stop_generation:
                raise NativeFailure('cleanup_superseded: retaining backend for higher priority stop', 40)
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
                self.power_subscription = self.power_observation = self.power_node = None
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
            self.power_subscription = self.power_observation = self.power_node = None
            self.last_completed_stop = {'operation_id': req['request_id'], 'epoch': power['epoch'],
                                        'generation': power['generation'], 'historical_only': True,
                                        'native_owned_services_cleanup': 'completed'}
            return {'verified_off': True, 'native_owned_services_cleanup': 'completed', 'motion': stopped}
        if action == 'StopMotion':
            return self.stop_motion(tolerate_exit_failure=True, allow_no_motion=True)
        self.check_cancel()
        if action == 'Communications':
            return self.connect_for_operation(req, verify_off=True)
        if action == 'ResetEmergency':
            return self.reset_emergency(req)
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

    def session_quiescent(self):
        """Only checks local ownership; this does NOT assert physical power off."""
        r = self.runtime
        if self.motion or r.bridge or r.remote or r.feedback or r.connected_target or r.power_touched:
            return False
        names = [r.executable('rinbo_fsm', name) for name in MOTION_NAMES]
        names.append(r.executable('rinbo_ros_bridge', 'rinbo_ros_bridge'))
        return not find_motion(names, r.env.get('ROS_DOMAIN_ID', '99'))

    @staticmethod
    def verify_prepared_off(power, ready):
        """Bind the final fresh observation to this operation's Off boundary."""
        ack, off_ack = ready.get('acknowledgement') or {}, power.get('acknowledgement') or {}
        stamp, off_stamp = ack.get('feedback_stamp_ns'), off_ack.get('feedback_stamp_ns')
        if (ready.get('epoch') != power.get('epoch') or ready.get('generation') != power.get('generation') or
                any(ack.get(k) is not False for k in ('digital','signal','power')) or
                not isinstance(stamp,int) or not isinstance(off_stamp,int) or stamp <= off_stamp):
            raise NativeFailure('preparation_changed: 關電後的來源、世代或供電狀態已改變，不能宣告準備完成',31,
                                {'recovery_off':power,'final_observation':ready,'recovery_action':'Communications'})

    def connect_for_operation(self, req, *, verify_off, ready=True):
        self.bind(req)
        self.check_cancel()
        # Stop/reap our old motion first, without requiring feedback from a
        # backend that this very operation may have to reconstruct.
        motion = self.stop_motion(tolerate_exit_failure=True, verify_output=False)
        snapshot = self.runtime.reconnect(req['sbrio_ip'], local_ip=req['orin_ip'],
                                          verify_stopped=False, verify_feedback=False, recover_owned_bridge=True)
        self.check_cancel()
        self.watch_power()
        power = None
        if verify_off:
            power = self.urgent_off(req['request_id'], emergency=(self.emergency_required and req['action'] != 'ResetEmergency'))
            self.check_cancel()
        try:
            native = self.power('ready', req['request_id']) if ready else None
            if native is not None and power is not None:
                self.verify_prepared_off(power, native)
            self.check_cancel()
        except NativeFailure as exc:
            detail = dict(exc.data or {}, recovery_action='Communications')
            if str(exc) == 'software_estop_latched':
                detail.update(emergency_latched=True, recovery_action='ResetEmergency')
            if power is not None:
                detail['recovery_off'] = power
            raise NativeFailure(str(exc), exc.code, detail) from exc
        if native is not None:
            snapshot = dict(snapshot, readiness=native.get('readiness', 'ready'))
        return dict(connection=snapshot, native=native, motion=motion,
                    recovery_verified=power is not None, power=power,
                    backend_retained=True)

    def recover_stop(self, req):
        # Called only by the worker after its immediate Off lane could not
        # obtain an ACK. It remains cancellable by a newer Stop/EmergencyStop.
        self.connect_for_operation(req, verify_off=True, ready=False)
        self.check_cancel()
        return self.off_results[req['request_id']]

    def reset_emergency(self, req):
        preparation = self.connect_for_operation(req, verify_off=True, ready=False)
        self.check_cancel()
        try:
            ready = self.power('ready', req['request_id'])
            self.verify_prepared_off(preparation['power'], ready)
            self.check_cancel()
        except NativeFailure as exc:
            if str(exc) != 'software_estop_latched':
                raise
            # The existing Bridge deliberately ignores /estop=false. Only this
            # explicit reset may retire that Bridge, after THIS all-off ACK.
            self.stop_motion(tolerate_exit_failure=True, verified_power_off=preparation['power'])
            self.check_cancel()
            r = self.runtime
            bridges = find_motion([r.executable('rinbo_ros_bridge', 'rinbo_ros_bridge')],
                                  r.env.get('ROS_DOMAIN_ID', '99'))
            stop_motion(bridges, r.log_dir, lambda line:self.emit('GUI', line))
            r.bridge = None
            if r.feedback:
                self.power_subscription = self.power_observation = self.power_node = None
                r.feedback.close(); r.feedback = None
            r.connected_target = None
            # Keep Core/FPGA and recorder alive. Reconnect adopts existing services.
            self.check_cancel()
            result = self.connect_for_operation(req, verify_off=True)
            result['before_reset_off'] = preparation['power']
            result.update(emergency_reset_verified=True, bridge_restarted_for_estop=True)
            return result
        preparation['connection'] = dict(preparation['connection'], readiness=ready.get('readiness', 'ready'))
        preparation.update(native=ready, emergency_reset_verified=True, bridge_restarted_for_estop=False)
        return preparation

    def watch_power(self):
        from std_msgs.msg import String
        node = self.runtime.probe().node
        if self.power_subscription is not None and self.power_node is node:
            return
        self.power_node = node
        self.power_observation = None
        def receive(message):
            if self.power_node is not node:
                return
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
            return {'readiness': 'backend_unavailable', 'reason': 'communications_not_established',
                    'power_state': 'unknown', 'panel_backend': self.backend_evidence()}
        snapshot = self.runtime.status()
        observation = self.power_observation
        snapshot['readiness'] = 'backend_unavailable'
        snapshot['reason'] = 'no_fresh_native_power_protocol_status'
        if observation and time.monotonic()-observation[0] <= .35:
            snapshot['power_protocol'] = observation[1]
            snapshot['readiness'] = observation[1]['readiness']
            snapshot['reason'] = observation[1].get('reason', '')
        return snapshot
