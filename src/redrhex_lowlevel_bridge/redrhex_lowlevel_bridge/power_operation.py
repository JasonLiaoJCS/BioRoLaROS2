"""Power protocol v2: short CLI lifetime, Bridge-owned epoch and Off fencing.

No cached acknowledgement authorizes a write. Already-satisfied operations
create no /power/command publisher. Legacy mode names remain supported.
"""
import json
import time
import uuid

import rclpy
from std_msgs.msg import String, Bool
from .rinbo_power_tool import (
    RinboPowerTool, POWER_STATES, _power_feedback_header_key,
    _power_state_matches, _relay_feedback_violation, RELAY_HEALTHY_SAMPLES,
    _classify_graph_endpoint_names,
)

STATUS_TOPIC = '/rinbo/power/operation_status'
EXIT_CODES = {'success': 0, 'already_satisfied': 0, 'ready': 0,
              'not_sent': 10, 'rejected': 20, 'ack_timeout': 30,
              'state_unknown': 31, 'interrupted': 40}


class PowerFailure(RuntimeError):
    def __init__(self, status, reason):
        self.status, self.reason = status, reason
        super().__init__(reason)


def plan_for(mode, mask, include_relay=False):
    """Sequence/ensure-on never lower an already enabled rail on retry."""
    if mask not in (0, 1, 3, 7):
        raise PowerFailure('rejected', 'invalid_rail_state: require verified Off')
    if mode == 'off': return ['off']
    if mode in ('ensure-on', 'relay') or (mode == 'sequence' and include_relay):
        return {0: ['digital', 'sensors', 'relay'], 1: ['sensors', 'relay'],
                3: ['relay'], 7: []}[mask]
    if mode == 'sequence':
        return {0: ['digital', 'sensors'], 1: ['sensors'], 3: [], 7: []}[mask]
    target = sum(bit for value,bit in zip(POWER_STATES[mode],(1,2,4)) if value)
    return [] if target == mask else [mode]


class PowerSession(RinboPowerTool):
    def __init__(self, args):
        super().__init__(args)
        self.request_id = args.request_id or uuid.uuid4().hex
        self.operation_started = time.monotonic()
        self.status = None
        self.status_receipt = 0.0
        self.status_key = None
        self.status_gids = None
        self.epoch = None
        self.generation = None
        self.sent = False
        self.last_sent_sequence = 0
        self.last_ack = None
        self.operation_result = None
        self.create_subscription(String, STATUS_TOPIC, self._on_status, 10)

    def _on_status(self, msg):
        try:
            value = json.loads(msg.data)
            required = ('epoch','generation','status_sequence','stamp_ns','readiness',
                        'feedback_mask','feedback_serial','power_age_ms','motor_age_ms',
                        'accepted_request_id','accepted_sequence','accepted_gid',
                        'accepted_stamp_ns','target_mask','acknowledged',
                        'can_handoff','owner_present','reason',
                        'rejected_request_id','rejection_reason')
            if value.get('protocol') != 2 or not all(k in value for k in required): return
            if type(value['epoch']) is not str or not value['epoch']: return
            for key in ('generation','status_sequence','stamp_ns','feedback_serial',
                        'feedback_mask','power_age_ms','motor_age_ms','accepted_sequence','accepted_stamp_ns','target_mask'):
                if type(value[key]) is not int: return
            if any(type(value[k]) is not bool for k in ('acknowledged','can_handoff','owner_present')): return
            if abs(self.get_clock().now().nanoseconds-value['stamp_ns']) > 350_000_000: return
            key = (value['epoch'],value['status_sequence'],value['stamp_ns'])
            if self.status_key and key[0]==self.status_key[0] and (
                    key[1]<=self.status_key[1] or key[2]<=self.status_key[2]): return
            self.status_key = key
            self.status, self.status_receipt = value, time.monotonic()
        except (ValueError, TypeError, KeyError):
            return

    def current_status(self):
        if self.status is None or time.monotonic()-self.status_receipt > .35:
            return None
        if abs(self.get_clock().now().nanoseconds-self.status['stamp_ns']) > 350_000_000:
            return None
        infos = self.get_publishers_info_by_topic(STATUS_TOPIC)
        states = self.get_publishers_info_by_topic(self.args.state_topic)
        for endpoints in (infos, states):
            classification=_classify_graph_endpoint_names(self._endpoint_names(endpoints),'/rinbo_ros2_bridge')
            if classification=='pending' and self.status_gids is None: return None
            if classification!='match':
                if self.status_gids is not None:
                    raise PowerFailure('state_unknown','bridge_source_changed: state graph no longer has the pinned sole Bridge')
                raise PowerFailure('rejected','untrusted_state_source: require exactly one Bridge')
        gids = (bytes(infos[0].endpoint_gid),bytes(states[0].endpoint_gid))
        if self.status_gids is not None and gids != self.status_gids:
            raise PowerFailure('state_unknown','bridge_source_changed: new operation required')
        self.status_gids = gids
        status = self.status
        if self.args.mode=='off' and self.epoch is None:
            self.epoch=status['epoch']
        if self.epoch is not None and status['epoch'] != self.epoch:
            raise PowerFailure('state_unknown','bridge_epoch_changed: new operation required')
        if (self.generation is not None and self.args.mode != 'off'
                and status['generation'] != self.generation):
            raise PowerFailure('rejected','off_preempted: cancelled; do not automatically retry')
        if self.args.mode!='off' and status['rejected_request_id']==self.request_id:
            raise PowerFailure('rejected',status['rejection_reason'])
        return status

    def fresh_sample(self, previous, since):
        msg = self.last_power_state
        receipt = self.last_power_state_received_monotonic
        if msg is None or receipt is None or receipt <= since or time.monotonic()-receipt > .35:
            return None
        try:
            key = _power_feedback_header_key(msg,self.get_clock().now().nanoseconds,previous)
        except RuntimeError:
            return None  # Cached/replayed samples never advance the sample count.
        self.assert_expected_state_graph()
        return msg,key

    def preflight(self):
        deadline = time.monotonic()+(self.args.status_timeout_s if self.args.mode=="status" else self.args.wait_for_subscriber_s)
        previous = None; healthy = 0; previous_mask = None
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.02)
            status = self.current_status()
            if status is None: continue
            if self.args.mode != 'off' and status['readiness']=='rejected':
                raise PowerFailure('rejected',status['reason'] or 'bridge_rejected')
            if self.args.mode=='off':
                self.epoch,self.generation=status['epoch'],status['generation']
                return status
            if status['readiness']!='ready': continue
            sample = self.fresh_sample(previous,self.operation_started)
            if sample is None: continue
            msg,previous=sample
            mask=(int(msg.digital) | (int(msg.signal)<<1) | (int(msg.power)<<2))
            if mask!=status['feedback_mask']: healthy=0;continue
            if msg.power:
                violation=_relay_feedback_violation(msg,self.args.disabled_legs)
                if violation: raise PowerFailure('rejected','power_guard: '+violation)
            healthy=healthy+1 if mask==previous_mask else 1
            previous_mask=mask
            if healthy>=RELAY_HEALTHY_SAMPLES:
                self.epoch,self.generation=status['epoch'],status['generation']
                self.last_ack=self.summarize_power_state(msg)
                self.last_ack.update(feedback_sequence=previous[0],feedback_stamp_ns=previous[1],
                                     request_id=self.request_id,epoch=status['epoch'],generation=status['generation'])
                return status
        status=self.status
        reason=('backend_unavailable_or_protocol_missing: no fresh Bridge v2 heartbeat; check services/paired installation' if status is None else
                status['readiness']+': new consistent motor/power samples not available')
        raise PowerFailure('not_sent',reason)

    def ensure_publisher(self):
        if self.pub is None:
            self.pub=self.create_publisher(self.PowerCmdStamped,self.args.topic,10)

    def build_msg(self,digital,signal,power):
        msg=super().build_msg(digital,signal,power)
        msg.header.frame_id=f'P2|{self.epoch or "unknown"}|{self.generation or 0}|{self.request_id}'
        return msg

    def send_stage(self,mode):
        self.ensure_publisher()
        estop_pub = (self.create_publisher(Bool, '/estop', 10)
                     if mode == 'off' and getattr(self.args, 'assert_estop', False) else None)
        self.wait_for_subscriber()
        if mode!='off': self.wait_for_expected_enable_graph()
        start=time.monotonic()
        for _ in range(self.args.repeat):
            if mode!='off':
                status=self.current_status()
                if status is None: raise PowerFailure('state_unknown','status_lost_before_command')
                if status['readiness']!='ready': raise PowerFailure('rejected',status['reason'] or status['readiness'])
                self.assert_expected_enable_graph()
            if estop_pub is not None:
                estop_pub.publish(Bool(data=True))
            msg=self.build_msg(*POWER_STATES[mode])
            self.pub.publish(msg);self.sent=True;self.last_sent_sequence=msg.header.seq
            rclpy.spin_once(self,timeout_sec=.02)
            time.sleep(self.args.repeat_delay_s)
        deadline=time.monotonic()+self.args.verify_timeout_s
        previous=None;count=0;unhealthy=0;last_violation=None
        while rclpy.ok() and time.monotonic()<deadline:
            if estop_pub is not None:
                estop_pub.publish(Bool(data=True))
            rclpy.spin_once(self,timeout_sec=.02)
            status=self.current_status()
            if status is None:continue
            if mode!='off':
                self.assert_expected_enable_graph()
                if status['readiness']=='rejected': raise PowerFailure('rejected',status['reason'])
            # Accepted receipt is tied to this request and last repeated packet;
            # Bridge requires a subsequent backend packet with matching rails.
            if (status['accepted_request_id']!=self.request_id or
                    status['accepted_sequence']!=self.last_sent_sequence or not status['acknowledged']):continue
            sample=self.fresh_sample(previous,start)
            if sample is None:continue
            state,previous=sample
            stamp=state.header.stamp.sec*1_000_000_000+state.header.stamp.nanosec
            if stamp<=status['accepted_stamp_ns']:continue
            if not _power_state_matches(state,*POWER_STATES[mode]):count=0;continue
            if mode=='relay':
                last_violation=_relay_feedback_violation(state,self.args.disabled_legs)
                if last_violation:
                    count=0;unhealthy+=1
                    if unhealthy>=3: raise PowerFailure('rejected','power_guard: '+last_violation)
                    continue
                unhealthy=0
            count+=1
            if count >= (RELAY_HEALTHY_SAMPLES if mode=='relay' else 1):
                if estop_pub is not None and status.get('reason') != 'software_estop_latched':
                    continue
                self.last_ack=self.summarize_power_state(state)
                self.last_ack.update(ack_kind='command_correlated',request_id=self.request_id,
                    epoch=status['epoch'],generation=status['generation'],
                    command_sequence=status['accepted_sequence'],publisher_gid=status['accepted_gid'],
                    feedback_sequence=state.header.seq,feedback_stamp_ns=stamp,healthy_samples=count,
                    estop_asserted=estop_pub is not None)
                return
        raise PowerFailure('ack_timeout',f'ACK timeout ({self.args.verify_timeout_s}s): stage={mode}; '
            f'last_guard_violation={last_violation}; relay state UNKNOWN')

    def result(self,status,reason=''):
        snapshot=self.status or {}
        return dict(schema_version=1,request_id=self.request_id,operation=self.args.mode,
            status=status,exit_code=EXIT_CODES[status],command_sent=self.sent,
            readiness=snapshot.get('readiness','backend_unavailable'),
            epoch=snapshot.get('epoch'),generation=snapshot.get('generation'),
            state_source='/rinbo_ros2_bridge',reason=reason,
            recovery=('verify connection/readiness; for ownership/fault require verified off; '
                      'do not automatically retry power' if EXIT_CODES[status] else None),
            acknowledgement=self.last_ack if EXIT_CODES[status]==0 else None,
            last_verified_state=self.last_ack if EXIT_CODES[status]!=0 else None,
            bridge_rejection=dict(request_id=snapshot.get('rejected_request_id'),
                publisher_gid=snapshot.get('rejected_gid'),sequence=snapshot.get('rejected_sequence'),
                reason=snapshot.get('rejection_reason')),
            leg_configuration=(self.args.leg_configuration.summary()
                if self.args.leg_configuration is not None else None))

    def run(self):
        try:
            if self.args.mode=='off':
                # Off never requires configuration, epoch availability, ready
                # backend, or absence of an enabling publisher to be sent.
                self.epoch=None; self.generation=None
                self.send_stage('off')
                return self.result('success')
            status=self.preflight()
            if self.args.mode in ('status','ready'):
                return self.result('ready')
            publishers=self.get_publishers_info_by_topic(self.args.topic)
            if publishers: raise PowerFailure('rejected','power_operation_busy: another /power/command publisher exists')
            plan=plan_for(self.args.mode,status['feedback_mask'],self.args.include_relay)
            if not plan:
                self.last_ack.update(ack_kind='fresh_already_satisfied',healthy_samples=RELAY_HEALTHY_SAMPLES)
                return self.result('already_satisfied')
            relay_release = (self.args.mode == 'sensors' and status['feedback_mask'] == 7
                             and status['acknowledged'])
            if not status['can_handoff'] and not relay_release:
                raise PowerFailure('rejected','ownership_handoff_required: use verified off before a new enabling operation')
            if status['owner_present'] and not status['acknowledged']:
                raise PowerFailure('rejected','handoff_not_acknowledged: use verified off')
            for mode in plan:
                self.send_stage(mode)
                if mode!=plan[-1]:
                    end=time.monotonic()+self.args.step_delay_s
                    while time.monotonic()<end:
                        rclpy.spin_once(self,timeout_sec=.02)
                        self.current_status()  # Off can cancel between stages.
            return self.result('success')
        except (RuntimeError,ValueError) as exc:
            status=exc.status if isinstance(exc,PowerFailure) else ('state_unknown' if self.sent else 'not_sent')
            # Interrupted/failed on-operation cleanup only sends Off after this
            # invocation actually sent commands. Read-only retry cannot drop power.
            reason=str(exc)
            if self.sent and self.args.mode!='off' and 'off_preempted' not in reason:
                try:
                    original_mode=self.args.mode
                    self.args.mode='off';self.epoch=None
                    self.send_stage('off')
                    reason+='; cleanup Off acknowledged'
                except RuntimeError as off_error:reason+='; cleanup Off unverified: '+str(off_error)
                finally: self.args.mode=original_mode
            return self.result(status,reason)

    def publish_unverified_emergency_off(self):
        if not self.sent:return False
        return super().publish_unverified_emergency_off()
