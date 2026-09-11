"""Single-user, persistent native controller. No hardware is started on boot."""
import argparse
from collections import deque
import fcntl
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import queue
import signal
import socket
import socketserver
import sqlite3
import struct
import threading
import time
import uuid

from .panel_protocol import ROOT, MAX_REQUEST, MOTION, READONLY, URGENT, result, validate
from .panel_native import Native, NativeFailure

VERSION = 'panel-flow-v4-20260911'
STOP_PRIORITY = {'StopMotion': 1, 'Stop': 2, 'EmergencyStop': 3}


class Controller:
    def __init__(self, root, factory=Native):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.lock = threading.RLock()
        self.epoch = uuid.uuid4().hex
        self.db = sqlite3.connect(self.root/'panel.sqlite3', check_same_thread=False)
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('CREATE TABLE IF NOT EXISTS requests (id TEXT PRIMARY KEY, request TEXT, response TEXT)')
        self.db.execute('CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value TEXT)')
        row = self.db.execute("SELECT value FROM metadata WHERE key='latch'").fetchone()
        self.latched = bool(row and row[0] == '1')
        row = self.db.execute("SELECT value FROM metadata WHERE key='latch_context'").fetchone()
        self.latch_context = json.loads(row[0]) if row else (
            {'source': 'legacy_metadata', 'operation_id': None, 'action': None,
             'reason': 'legacy_stop_latch_source_unrecorded', 'phase': 'unknown'} if self.latched else None)
        row = self.db.execute("SELECT value FROM metadata WHERE key='emergency_context'").fetchone()
        self.emergency_context = json.loads(row[0]) if row else None
        if row is None:
            # Migrate accepted explicit emergencies, NOT the old overloaded bool.
            # Old versions' successful Stop was their explicit reset boundary.
            for raw_req, raw_response in self.db.execute('SELECT request,response FROM requests ORDER BY rowid'):
                q, v = json.loads(raw_req), json.loads(raw_response)
                if q['action'] == 'EmergencyStop' and v['status'] not in ('rejected', 'unsupported'):
                    self.emergency_context = dict(operation_id=q['request_id'], action='EmergencyStop',
                                                  reason=v.get('reason') or 'explicit_emergency_stop', source='legacy_operation')
                elif (q['action'] == 'Stop' and not v.get('controller_version','').startswith('panel-flow-')
                      and v['status'] == 'completed' and (v.get('data') or {}).get('verified_off') is True):
                    self.emergency_context = None
                elif (q['action'] == 'ResetEmergency' and v['status'] == 'completed'
                      and (v.get('data') or {}).get('emergency_reset_verified') is True):
                    self.emergency_context = None
        self.latched = self.latched or self.emergency_context is not None
        row = self.db.execute("SELECT value FROM metadata WHERE key='closed_session'").fetchone()
        self.closed_session = json.loads(row[0]) if row else None
        self.events = deque(maxlen=2000)
        self.serial = 0
        self.logger = logging.getLogger('rinbo.panel.'+self.epoch)
        self.logger.setLevel(logging.INFO)
        handler = RotatingFileHandler(self.root/'panel-events.log', maxBytes=2_000_000, backupCount=3)
        self.logger.addHandler(handler)
        self.native = factory(self.root, self.emit)
        self.native.finish = self.finish
        self.native.emergency_required = bool(self.emergency_context)
        target = self.db.execute("SELECT value FROM metadata WHERE key='target'").fetchone()
        if target:
            self.native.target = tuple(json.loads(target[0]))
        log_path = self.root/'panel-events.log'
        if log_path.exists():
            for line in log_path.read_text(errors='replace').splitlines()[-400:]:
                try:
                    event = json.loads(line)
                    self.serial += 1
                    event['sequence'] = self.serial
                    self.events.append(event)
                except ValueError:
                    pass
        self.active = None
        self.urgent_pending = 0
        self.urgent_kind = None
        self.urgent_requests = {}
        self.aliases = {}
        self.superseded = {}
        self.stopmotion_may_clear = False
        self.jobs = queue.Queue()
        self.shutdown = threading.Event()
        for rid, request, response in self.db.execute('SELECT * FROM requests').fetchall():
            old = json.loads(response)
            if old['status'] in ('started', 'running', 'holding', 'native_done', 'accepted'):
                self.set_latch(True, rid, json.loads(request)['action'], 'controller_restarted: no automatic replay', 'unknown')
                self.finish(rid, 'state_unknown', 31, 'controller_restarted: no automatic replay',
                            {'previous_result': old, 'power_state': 'unknown', 'verified_off': None})
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()

    def latch_record(self, rid, action, reason, phase):
        return dict(source='native_controller', operation_id=rid, action=action,
                    reason=reason, phase=phase, controller_epoch=self.epoch,
                    updated_at_ns=time.time_ns())

    def set_latch(self, value, rid=None, action=None, reason='native_stop_required', phase='unknown'):
        with self.lock:
            context = self.latch_record(rid, action, reason, phase) if value else None
            with self.db:
                self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('latch', ?)", ('1' if value else '0',))
                self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('latch_context', ?)", (json.dumps(context),))
            self.latched, self.latch_context = value, context

    def control_info(self, readiness=None, *, latched=None, context=None, pending=None, emergency=None):
        latched = self.latched if latched is None else latched
        context = self.latch_context if context is None and latched else context
        emergency = self.emergency_context if emergency is None else emergency
        pending = bool(self.active or self.urgent_pending) if pending is None else pending
        if pending:
            state, recovery = 'operation_in_progress', 'ReadLogs'
            steps = ['原生操作正在處理；請查看同一操作的結果。停止與急停仍可搶占。']
        elif emergency:
            state, recovery = 'emergency_latched', 'ResetEmergency'
            steps = ['急停尚未復歸。請按「急停復歸」；Orin 會確認關電、恢復通訊並保持關電，不會自動開啟動力。']
        elif latched:
            state, recovery = 'recovery_required', 'Communications'
            steps = ['上次停止尚未確認完成。請按一次「建立通訊」，Orin 會完成恢復與新的關電確認，再接受開啟動力。']
        elif readiness == 'ready':
            state, recovery, steps = 'ready', None, []
        else:
            state, recovery = 'connection_required', 'Communications'
            steps = ['請按「建立通訊」，完成後可直接按「開啟動力」。']
        return dict(control_state=state, emergency_latched=bool(emergency),
                    stop_blocked=latched, stop_state=('in_progress' if pending and self.urgent_pending else ('unknown' if latched and not emergency else ('emergency' if emergency else 'clear'))),
                    latch=context if latched else None, emergency_source=emergency or None,
                    recoverable=bool(latched or emergency), recovery_action=recovery, recovery_steps=steps,
                    action_availability={**{action: ('blocked_emergency' if emergency else
                        'recovery_required' if latched else 'busy' if pending else 'subject_to_native_checks')
                        for action in ('PowerOn','Calibration','Standing','Tripod')},
                        'Communications': 'busy' if pending else ('requires_emergency_reset' if emergency else 'available'),
                        'ResetEmergency': 'busy' if pending else 'available',
                        **{action:'available' for action in ('Stop','StopMotion','EmergencyStop','CheckConnection','ReadLogs')}},
                    availability_scope='controller_admission_only; native hardware checks still apply')

    def observed_status(self):
        if self.active or self.urgent_pending:
            return {'readiness': 'starting', 'reason': 'operation_in_progress'}
        return self.native.status()

    def emit(self, kind, line):
        with self.lock:
            for part in str(line).splitlines():
                self.serial += 1
                row = dict(sequence=self.serial, epoch=self.epoch, kind=kind, line=part,
                           wall_time_ns=time.time_ns(), monotonic_ns=time.monotonic_ns())
                self.events.append(row)
                self.logger.info(json.dumps(row, ensure_ascii=False))

    def finish(self, rid, status, code=0, reason='', data=None):
        with self.lock:
            row = self.db.execute('SELECT request,response FROM requests WHERE id=?', (rid,)).fetchone()
            if not row:
                return
            req, previous = json.loads(row[0]), json.loads(row[1])
            # The first failure is authoritative even when later cleanup fails.
            if previous['exit_code'] and previous['status'] not in ('started', 'accepted'):
                return
            value = result(req, status, code if code >= 0 else 31, reason,
                           controller_epoch=self.epoch, native_exit_code=code, data=data,
                           controller_version=VERSION, updated_at_ns=time.time_ns())
            if previous.get('coalesced_with'):
                value['coalesced_with'] = previous['coalesced_with']
            next_latched, next_context = self.latched, self.latch_context
            other_stops = any(other != rid and other not in self.superseded
                              for other in self.urgent_requests)
            owner = (self.latch_context or {}).get('operation_id')
            owns_latch = owner == rid
            success = status in ('completed', 'already_satisfied') and code == 0
            verified_stop = (req['action'] == 'Stop' and (data or {}).get('verified_off') is True
                             and (data or {}).get('native_owned_services_cleanup') == 'completed')
            recovered = req['action'] == 'Communications' and (data or {}).get('recovery_verified') is True
            reset = req['action'] == 'ResetEmergency' and (data or {}).get('emergency_reset_verified') is True
            next_emergency = self.emergency_context
            if next_emergency and next_emergency.get('operation_id') == rid and req['action'] == 'EmergencyStop':
                next_emergency = dict(next_emergency, phase=status, result_reason=reason)
            if success and reset and not other_stops:
                next_emergency = None
            release = (success and rid not in self.superseded and not other_stops and not next_emergency and
                       ((owns_latch and (verified_stop or (req['action'] == 'StopMotion' and self.stopmotion_may_clear)))
                        or recovered or reset))
            if release:
                next_latched, next_context = False, None
            elif owns_latch and status not in ('started', 'accepted'):
                next_context = self.latch_record(rid, req['action'], reason or
                    ('emergency_stop_requires_verified_Stop' if req['action'] == 'EmergencyStop'
                     else 'stop_not_released'), status)
            readiness = ('backend_unavailable' if req['action'] == 'Stop' and success
                         else ('ready' if req['action'] in ('Communications','ResetEmergency') and success else None))
            control = self.control_info(readiness, latched=next_latched, context=next_context,
                                        pending=other_stops, emergency=next_emergency or False)
            if code == 0 and status == 'completed' and req['action'] in ('Communications', 'Stop'):
                if req['action'] == 'Communications' and next_latched:
                    value['reason'] = '通訊已建立，但仍需急停復歸。' + ' '.join(control['recovery_steps'])
                elif req['action'] == 'Stop' and release:
                    value['reason'] = '本次停止、關電確認與清理已完成，停止鎖已解除。' + ' '.join(control['recovery_steps'])
            value.update(emergency_latched=bool(next_emergency), stop_blocked=next_latched, control=control)
            if code == 0 and req['action'] == 'Communications' and recovered and not next_latched:
                value['reason'] = '通訊已就緒，已取得本次關電確認；可直接開啟動力。'
            elif code == 0 and reset and release:
                value['reason'] = '急停復歸完成，通訊已就緒並保持關電；可自行開啟動力。'
            elif code == 0 and req['action'] == 'StopMotion' and (data or {}).get('no_motion'):
                value['reason'] = ('目前沒有動作程序；硬體輸出狀態已確認。' if (data or {}).get('verified') else
                                   '目前沒有動作程序；未連線，硬體輸出狀態尚未確認。')
            message = (value['reason'] if code == 0 else
                       '原生操作未完成：'+reason+'。'+' '.join(control['recovery_steps']))
            value['operator_message'] = message
            value['data'] = dict(data or {}, control=control, operator_message=message, native_reason=reason)
            aliases = []
            if status not in ('started', 'accepted', 'running', 'holding', 'native_done'):
                for alias in self.aliases.get(rid, []):
                    stored = self.db.execute('SELECT request,response FROM requests WHERE id=?', (alias,)).fetchone()
                    alias_req, alias_previous = map(json.loads, stored)
                    if alias_previous['exit_code']:
                        continue
                    alias_value = dict(value, request_id=alias, operation_id=alias,
                                       action=alias_req['action'], coalesced_with=rid,
                                       data=dict(value['data'], fulfilled_by_operation_id=rid))
                    aliases.append((alias, alias_value))
            # One transaction: completed is never observable before its latch and
            # ownership retirement. On DB failure no completion event is emitted.
            with self.db:
                self.db.execute('UPDATE requests SET response=? WHERE id=?', (json.dumps(value), rid))
                for alias, alias_value in aliases:
                    self.db.execute('UPDATE requests SET response=? WHERE id=?', (json.dumps(alias_value), alias))
                self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('latch', ?)", ('1' if next_latched else '0',))
                self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('latch_context', ?)", (json.dumps(next_context),))
                self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('emergency_context', ?)", (json.dumps(next_emergency),))
                if success and verified_stop:
                    self.db.execute("DELETE FROM metadata WHERE key='target'")
                    self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('last_stop_commit', ?)",
                                    (json.dumps({'operation_id': rid, 'controller_epoch': self.epoch,
                                                 'historical_only': True, 'updated_at_ns': value['updated_at_ns']}),))
                    self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('closed_session', ?)",
                                    (json.dumps({'operation_id': rid, 'controller_epoch': self.epoch,
                                                 'scope': 'native_session_closed', 'hardware_confirmation': 'historical_only',
                                                 'target': [req['sbrio_ip'], req['orin_ip'], req['ros_domain_id']]}),))
            self.latched, self.latch_context, self.emergency_context = next_latched, next_context, next_emergency
            self.native.emergency_required = bool(next_emergency)
            if success and verified_stop:
                self.closed_session = {'operation_id': rid, 'controller_epoch': self.epoch,
                                       'scope': 'native_session_closed', 'hardware_confirmation': 'historical_only',
                                       'target': [req['sbrio_ip'], req['orin_ip'], req['ros_domain_id']]}
            kind = req['action'] if req['action'] in MOTION else 'GUI'
            self.emit(kind, 'RSLIP_ACTION_STATUS='+json.dumps(dict(kind=kind, **value), ensure_ascii=False))
            if status not in ('started', 'accepted', 'running', 'holding', 'native_done'):
                self.aliases.pop(rid, None)
                for alias, alias_value in aliases:
                    self.emit('GUI', 'RSLIP_ACTION_STATUS='+json.dumps(dict(kind='GUI', **alias_value), ensure_ascii=False))

    def submit(self, req):
        validate(req)
        with self.lock:
            if req['action'] == 'ReadLogs':
                return result(req, 'completed', controller_epoch=self.epoch,
                              controller_version=VERSION, events=list(self.events), operations=self.operations(),
                              emergency_latched=bool(self.emergency_context), stop_blocked=self.latched, latch=self.latch_context)
            if req['action'] == 'CheckConnection':
                snapshot = self.observed_status()
                control = self.control_info(snapshot.get('readiness'))
                return result(req, 'completed', controller_epoch=self.epoch,
                              controller_version=VERSION,
                              data=dict(snapshot, control=control), emergency_latched=bool(self.emergency_context), stop_blocked=self.latched, control=control,
                              active_operation=self.active, operations=self.operations())
            encoded = json.dumps(req, sort_keys=True)
            row = self.db.execute('SELECT request,response FROM requests WHERE id=?', (req['request_id'],)).fetchone()
            if row:
                if row[0] != encoded:
                    return result(req, 'rejected', 20, 'request_id_conflict')
                return dict(json.loads(row[1]), replayed=True)
            action = req['action']
            if (action == 'Stop' and self.closed_session and
                    self.closed_session.get('target') == [req['sbrio_ip'],req['orin_ip'],req['ros_domain_id']] and not self.latched and
                    not self.active and not self.urgent_pending and self.native.session_quiescent()):
                # Complete an already-ended native session, not a new physical
                # all-off assertion. No stale ACK is presented as current.
                control = self.control_info('backend_unavailable')
                answer = result(req, 'already_satisfied', 0,
                    '本控制工作階段已結束，沒有再次送出命令；目前未連線，未取得新的硬體供電回讀。',
                    controller_epoch=self.epoch, controller_version=VERSION,
                    emergency_latched=False, stop_blocked=False, control=control,
                    data=dict(completion_scope='native_session_closed', verified_off=None,
                              current_hardware_confirmed=False, power_state='unknown', command_sent=False,
                              prior_session=self.closed_session, control=control))
                with self.db:
                    self.db.execute('INSERT INTO requests VALUES (?,?,?)', (req['request_id'], encoded, json.dumps(answer)))
                self.emit('GUI', 'RSLIP_ACTION_STATUS='+json.dumps(dict(kind='GUI', **answer), ensure_ascii=False))
                return answer
            # One physical stop workflow per equivalent request. Every GUI click
            # retains its own durable operation ID and links to the leader ACK.
            leader = next((rid for rid, pending in self.urgent_requests.items()
                           if rid not in self.superseded and
                           (pending['action'] == action or
                            (action == 'StopMotion' and pending['action'] in ('Stop','EmergencyStop')))), None)
            if leader is not None:
                if any(req[k] != self.urgent_requests[leader][k] for k in ('sbrio_ip','orin_ip','ros_domain_id')):
                    return result(req, 'rejected', 20, 'target_changed: stop must use original target')
                leader_result = json.loads(self.db.execute('SELECT response FROM requests WHERE id=?', (leader,)).fetchone()[0])
                terminal = leader_result['status'] not in ('started','accepted','running','holding','native_done')
                if terminal:
                    answer = dict(leader_result, request_id=req['request_id'], operation_id=req['request_id'],
                                  action=action, coalesced_with=leader,
                                  data=dict(leader_result.get('data') or {}, fulfilled_by_operation_id=leader))
                else:
                    answer = result(req, 'started', controller_epoch=self.epoch, controller_version=VERSION,
                                    native_exit_code=0, coalesced_with=leader,
                                    reason='joined native stop; no duplicate hardware request')
                with self.db:
                    self.db.execute('INSERT INTO requests VALUES (?,?,?)', (req['request_id'], encoded, json.dumps(answer)))
                if not terminal:
                    self.aliases.setdefault(leader, []).append(req['request_id'])
                self.emit('GUI', 'RSLIP_ACTION_STATUS='+json.dumps(dict(kind='GUI', **answer)))
                return answer
            if action in ('FpgaConsole', 'PhysicalCleanup'):
                answer = result(req, 'unsupported', 21, 'manual handoff required; physical power is not inferred')
            elif (self.urgent_pending or self.active) and action not in URGENT:
                answer = result(req, 'rejected', 20, 'controller_busy: observe current operation; no request queued')
            elif self.emergency_context and action == 'Communications':
                control = self.control_info(self.observed_status().get('readiness'))
                answer = result(req, 'rejected', 20, '急停尚未復歸；請按「急停復歸」，一次完成關電確認與通訊恢復。',
                                emergency_latched=True, control=control,
                                data={'control':control,'recovery_action':'ResetEmergency',
                                      'recovery_steps':control['recovery_steps']})
            elif self.latched and action not in URGENT | {'Communications','ResetEmergency'}:
                control = self.control_info(self.observed_status().get('readiness'))
                answer = result(req, 'rejected', 20, ' '.join(control['recovery_steps']),
                                emergency_latched=bool(self.emergency_context), control=control,
                                data={'control': control, 'recovery_action': control['recovery_action'],
                                      'recovery_steps': control['recovery_steps']})
            else:
                try:
                    self.native.bind(req)
                    self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('target', ?)",
                                    (json.dumps(self.native.target),))
                except Exception as exc:
                    return result(req, 'rejected', 20, str(exc))
                answer = result(req, 'started', controller_epoch=self.epoch,
                                reason='accepted by Orin; completion will be reported in ReadLogs')
            answer.update(controller_version=VERSION, native_exit_code=answer['exit_code'])
            # Commit admission and its stop latch together, before side effects.
            with self.db:
                self.db.execute('INSERT INTO requests VALUES (?,?,?)', (req['request_id'], encoded, json.dumps(answer)))
                if answer['status'] == 'started' and action not in ('Stop','StopMotion'):
                    self.db.execute("DELETE FROM metadata WHERE key='closed_session'")
                if answer['status'] == 'started' and action == 'EmergencyStop':
                    emergency = self.latch_record(req['request_id'], action, 'explicit_emergency_stop', 'in_progress')
                    self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('emergency_context', ?)", (json.dumps(emergency),))
                if answer['status'] == 'started' and action in URGENT:
                    self.stopmotion_may_clear = not self.latched
                    context = self.latch_record(req['request_id'], action, 'stop_requested', 'in_progress')
                    self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('latch', '1')")
                    self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('latch_context', ?)", (json.dumps(context),))
            if answer['status'] == 'started' and action not in ('Stop','StopMotion'):
                self.closed_session = None
            if answer['status'] == 'started' and action == 'EmergencyStop':
                self.emergency_context = emergency
                self.native.emergency_required = True
            if answer['status'] == 'started' and action in URGENT:
                self.latched, self.latch_context = True, context
            if answer['status'] != 'started':
                return answer
            if action in URGENT:
                self.urgent_pending += 1
                self.urgent_kind = action
                for older, pending in self.urgent_requests.items():
                    if STOP_PRIORITY[action] > STOP_PRIORITY[pending['action']]:
                        self.superseded[older] = req['request_id']
                self.urgent_requests[req['request_id']] = req
                self.native.interrupt(action)
                threading.Thread(target=self.preempt, args=(req,), daemon=True).start()
            else:
                self.active = req['request_id']
                self.native.cancel.clear()
                self.jobs.put((req, None, None))
            self.emit('GUI', 'RSLIP_ACTION_STATUS='+json.dumps(dict(kind=action if action in MOTION else 'GUI', **answer)))
            return answer

    def preempt(self, req):
        failure = None
        off_result = None
        if req['action'] != 'StopMotion':
            try:
                off_result = self.native.urgent_off(req['request_id'], emergency=req['action'] == 'EmergencyStop')
            except Exception as exc:
                failure = exc
                self.emit('GUI', 'urgent_off: '+str(exc))
        # Startup may be blocked in SSH, but Off above was not queued behind it.
        self.jobs.put((req, failure, off_result))

    def operations(self):
        return [json.loads(row[0]) for row in self.db.execute(
            'SELECT response FROM requests ORDER BY rowid DESC LIMIT 40').fetchall()]

    def work(self):
        while not self.shutdown.is_set():
            try:
                req, off_failure, off_result = self.jobs.get(timeout=.05)
            except queue.Empty:
                with self.lock:
                    if self.urgent_pending:
                        continue  # Stop owns the motion exit result; no poll race.
                try:
                    self.native.poll()
                except Exception as exc:
                    self.emit('GUI', 'native_monitor_failed: '+str(exc))
                    self.set_latch(True, reason='native_monitor_failed: '+str(exc))
                continue
            rid, action = req['request_id'], req['action']
            with self.lock:
                # Never close Bridge while a higher-priority Off is still
                # acquiring its ACK, even if the ordinary Off finished first.
                higher = action in URGENT and any(
                    other != rid and STOP_PRIORITY[pending['action']] > STOP_PRIORITY[action]
                    for other, pending in self.urgent_requests.items())
                if higher and rid not in self.superseded:
                    self.jobs.put((req, off_failure, off_result))
                else:
                    higher = False
            if higher:
                time.sleep(.02)
                continue
            with self.lock:
                self.active = rid
            execution_data = None
            initial = None
            try:
                if rid in self.superseded and off_failure is None:
                    raise NativeFailure('superseded_by_higher_priority_stop', 40,
                                        {'superseded_by': self.superseded[rid], 'power': off_result})
                if off_failure and action == 'Stop':
                    initial = dict(reason=str(off_failure), native_exit_code=getattr(off_failure,'code',31),
                                   data=getattr(off_failure,'data',None))
                    with self.lock:
                        if rid in self.superseded or any(other != rid for other in self.urgent_requests):
                            raise NativeFailure('已由新的停止要求接手', 40, {'initial_off_failure': initial})
                        self.native.cancel.clear()
                    try:
                        off_result = self.native.recover_stop(req)
                        off_failure = None
                    except Exception as recovery_error:
                        raise NativeFailure(str(recovery_error), getattr(recovery_error,'code',31),
                            dict(getattr(recovery_error,'data',None) or {}, initial_off_failure=initial,
                                 verified_off=None, power_state='unknown', native_owned_services_cleanup='not_attempted',
                                 recovery_action='Communications')) from recovery_error
                if off_failure:
                    # Still request a native motion stop, preserving Off's first
                    # reason and retaining the backend for recovery.
                    try:
                        motion_result = self.native.stop_motion(tolerate_exit_failure=True)
                    except Exception as exc:
                        self.emit('GUI', str(exc))
                        motion_result = {'status': 'unverified', 'reason': str(exc),
                                         'native_exit_code': getattr(exc, 'code', 31),
                                         'data': getattr(exc, 'data', None)}
                    detail = dict(getattr(off_failure, 'data', None) or {})
                    detail.update(motion_stop=motion_result, verified_off=None, power_state='unknown',
                                  native_owned_services_cleanup='not_attempted', recovery_action='Communications')
                    raise NativeFailure(str(off_failure), getattr(off_failure, 'code', 31), detail)
                data = self.native.execute(req)
                execution_data = data
                if rid in self.superseded:
                    raise NativeFailure('superseded_by_higher_priority_stop', 40,
                                        {'superseded_by': self.superseded[rid], 'native_result': data})
                if off_result is not None:
                    data['power'] = off_result
                if action == 'Stop' and initial is not None:
                    data['initial_off_failure'] = initial
                status = 'running' if action in MOTION else 'completed'
                if data.get('status') == 'already_satisfied':
                    status = 'already_satisfied'
                with self.lock:
                    if action not in URGENT and self.urgent_pending:
                        raise NativeFailure('已由停止要求接手；不繼續開電或復歸', 40, {'native_result': data})
                    self.finish(rid, status, 0, '', data)
            except Exception as exc:
                commit_failed = isinstance(exc, sqlite3.Error)
                code = 31 if commit_failed else getattr(exc, 'code', 20)
                data = dict(execution_data or {}, state_commit='failed') if commit_failed else getattr(exc, 'data', None)
                if off_result is not None:
                    cleanup = (data or {}).get('native_owned_services_cleanup', 'incomplete') if commit_failed else 'incomplete'
                    data = dict(data or {}, power=off_result, native_owned_services_cleanup=cleanup)
                status = 'state_unknown' if code in (30,31) or code < 0 else 'failed'
                with self.lock:
                    if self.emergency_context is None and ((data or {}).get('emergency_latched') or str(exc) == 'software_estop_latched'):
                        emergency = self.latch_record(rid, action, 'software_estop_latched', 'rejected')
                        with self.db:
                            self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('emergency_context', ?)", (json.dumps(emergency),))
                        self.emergency_context = emergency
                        self.native.emergency_required = True
                    if not self.urgent_pending and (action in ('Communications','ResetEmergency') or
                            (action == 'PowerOn' and (code in (30,31) or (data or {}).get('command_sent') is True))):
                        self.set_latch(True, rid, action, str(exc), status)
                self.finish(rid, status, code, str(exc), data)
                self.emit('GUI', str(exc))
            finally:
                with self.lock:
                    if action == 'EmergencyStop':
                        self.emergency_seen = True
                    if action == 'Stop':
                        self.emergency_seen = self.latched
                    self.active = None
                    if action in URGENT:
                        self.urgent_pending -= 1
                        self.urgent_requests.pop(rid, None)
                        self.superseded.pop(rid, None)
                    if hasattr(self.native, 'off_results'):
                        self.native.off_results.pop(rid, None)
                    if not self.urgent_pending:
                        self.native.cancel.clear()


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        # Same user only. Socket directory and socket are private as well.
        _, uid, _ = struct.unpack('3i', self.request.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
        if uid != os.getuid():
            return
        self.request.settimeout(10)
        req = {}
        try:
            raw = self.rfile.readline(MAX_REQUEST+1)
            if len(raw) > MAX_REQUEST:
                raise ValueError('request too large')
            req = validate(json.loads(raw))
            controller = self.server.controller
            if req['action'] == 'ReadLogs' and req.get('stream'):
                cursor = 0
                while True:
                    with controller.lock:
                        events = [row for row in controller.events if row['sequence'] > cursor]
                    for row in events:
                        self.wfile.write(('RSLIP_ACTION_CONTEXT='+row['kind']+'\n'+row['line']+'\n').encode())
                        cursor = row['sequence']
                    self.wfile.write(b'RSLIP_MONITOR_HEARTBEAT=1\n')
                    self.wfile.flush()
                    time.sleep(2)
            else:
                response = controller.submit(req)
                self.wfile.write((json.dumps(response, ensure_ascii=False)+'\n').encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass  # client death never cancels an admitted operation
        except Exception as exc:
            self.wfile.write((json.dumps(result(req, 'not_sent', 2, str(exc)))+'\n').encode())


class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--state-dir', type=Path, default=ROOT)
    args = parser.parse_args(argv)
    os.umask(0o077)
    root = args.state_dir
    root.mkdir(parents=True, exist_ok=True)
    lock = (root/'console.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('native_console_busy: robot.sh already owns Runtime; exit that console before starting GUI controller')
    path = root/'panel.sock'
    path.unlink(missing_ok=True)
    controller = Controller(root)
    server = Server(str(path), Handler)
    server.controller = controller
    def terminate(signum, frame):
        # Stop admission immediately; native shutdown does not depend on SSH.
        if controller.native.target is not None:
            controller.set_latch(True, reason='controller_shutdown_with_owned_backend')
        controller.native.interrupt('EmergencyStop')
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGINT, terminate)
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        controller.shutdown.set()
        controller.worker.join(130)
        if controller.native.target is not None:
            try:
                controller.native.urgent_off(uuid.uuid4().hex)
                if controller.worker.is_alive():
                    # Do not concurrently close a Runtime still inside native
                    # SSH/bootstrap. Parent death guardians stop its children.
                    controller.emit('GUI', 'controller_shutdown_incomplete: native worker did not quiesce; ownership records retained')
                else:
                    controller.native.stop_motion(tolerate_exit_failure=True)
                    controller.native.runtime.close()
            except Exception as exc:
                controller.emit('GUI', 'controller_shutdown_incomplete: '+str(exc))
        server.server_close()
        path.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
