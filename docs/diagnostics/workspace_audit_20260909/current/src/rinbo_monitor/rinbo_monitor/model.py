"""Bounded, thread-safe diagnostics. No ROS or robot control dependencies."""
from collections import deque
from copy import deepcopy
import math
import threading
import time
from .recording import Recording

LEGS = ('l1', 'l2', 'l3', 'r1', 'r2', 'r3')
TOPICS = {
    'requested': '/rinbo/monitor/motor_requested',
    'forwarded': '/rinbo/monitor/motor_forwarded',
    'motor': '/motor/state', 'power': '/power/state',
    'output': '/rinbo/motor_output_enabled',
    'ready': '/rinbo/motor_arbiter_ready', 'logs': '/rosout',
}


def json_safe(value):
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class MonitorStore:
    def __init__(self, clock=time.monotonic, wall_clock=time.time, stale_s=0.5,
                 recording_dir='log/monitor_recordings'):
        self.clock, self.wall_clock, self.stale_s = clock, wall_clock, stale_s
        self.lock = threading.Lock()
        self.started = clock()
        self.samples = {}
        self.events = deque(maxlen=300)
        self.history = deque(maxlen=600)  # 60 seconds at 10 Hz
        self.reset_counts = {key: {leg: 0 for leg in LEGS}
                             for key in ('requested', 'forwarded')}
        self.event_id = 0
        self.recording_dir = recording_dir
        self.recording = None

    def _event(self, now, kind, text):
        self.event_id += 1
        event = {'id': self.event_id, 't': now - self.started,
                 'kind': kind, 'text': text}
        self.events.append(event)
        if self.recording:
            self.recording.submit('event', deepcopy(event))

    def start_recording(self):
        with self.lock:
            if self.recording and self.recording.status()['state'] in ('recording', 'finalizing'):
                raise ValueError('已有錄製正在進行或存檔中')
            self.recording = Recording(self.recording_dir, self.clock, self.wall_clock)
            self._record_sample(self._sources(self.clock()), self.clock())
            return self.recording.status()

    def stop_recording(self):
        with self.lock:
            if not self.recording:
                raise ValueError('尚未開始錄製')
            self.recording.stop()
            return self.recording.status()

    def recording_path(self):
        with self.lock:
            if not self.recording or not self.recording.status()['download_ready']:
                raise ValueError('請先停止錄製並等待存檔完成')
            return self.recording.path

    def close(self):
        if self.recording:
            self.recording.close()

    def _record_sample(self, sources, now):
        if self.recording:
            self.recording.submit('sample', deepcopy({
                't_monitor_s': now - self.started,
                'sources': {k: v for k, v in sources.items() if k != 'logs'}}))

    def ingest(self, key, payload):
        payload = json_safe(deepcopy(payload))
        now = self.clock()
        with self.lock:
            previous = self.samples.get(key)
            count = previous['count'] + 1 if previous else 1
            if key in self.reset_counts:
                for leg in LEGS:
                    if payload.get(leg, {}).get('reset_position'):
                        self.reset_counts[key][leg] += 1
                        self._event(now, 'reset', '{} {} reset=true seq={}'.format(
                            key, leg.upper(), payload.get('header', {}).get('seq')))
            if key == 'motor' and previous:
                for leg in LEGS:
                    old = previous['data'].get(leg, {}).get('hall_effect')
                    new = payload.get(leg, {}).get('hall_effect')
                    if old is not None and new is not None and old != new:
                        self._event(now, 'hall', '{} Hall {} → {} (0=原點觸發)'.format(
                            leg.upper(), int(old), int(new)))
            if key in ('output', 'ready'):
                if previous is None or previous['data'] != payload:
                    self._event(now, 'bridge', '{} = {}'.format(key, payload.get('data')))
            if key == 'logs':
                self._event(now, 'error' if payload.get('level', 0) >= 40 else 'log',
                            '[{}] {}'.format(payload.get('name', ''), payload.get('msg', '')))
            self.samples[key] = {'received': now, 'data': payload, 'count': count}

    def _sources(self, now):
        result = {}
        for key, topic in TOPICS.items():
            item = self.samples.get(key)
            age = now - item['received'] if item else None
            stamp = item['data'].get('header', {}).get('stamp', {}) if item else {}
            stamp_s = stamp.get('sec', 0) + stamp.get('nanosec', 0) / 1e9
            result[key] = {
                'topic': topic, 'age_s': age,
                'stale': age is None or age > self.stale_s,
                'source_age_s': self.wall_clock() - stamp_s if stamp_s > 0 else None,
                'count': item['count'] if item else 0,
                'data': item['data'] if item else None,
            }
        return result

    def sample_history(self):
        now = self.clock()
        with self.lock:
            sources = self._sources(now)
            def data(key):
                source = sources[key]
                return source['data'] if not source['stale'] else None
            motor, forwarded = data('motor'), data('forwarded')
            # Compact 10 Hz display trace; reset pulses live in the event stream.
            points = []
            for leg in LEGS:
                main = motor.get(leg, {}) if motor else {}
                servo = motor.get('s' + leg, {}) if motor else {}
                cmd = forwarded.get(leg, {}) if forwarded else {}
                target = forwarded.get('s' + leg, {}) if forwarded else {}
                points.append([main.get('position'), cmd.get('voltage'),
                               target.get('position_encoder') if forwarded and forwarded.get('servo_control_mode') else None,
                               servo.get('position_encoder')])
            self.history.append({'t': now - self.started, 'legs': points})
            self._record_sample(sources, now)

    def snapshot(self, include_history=True):
        now = self.clock()
        with self.lock:
            return deepcopy({'elapsed_s': now - self.started,
                             'recording': self.recording.status() if self.recording else {'state': 'idle'},
                             'stale_s': self.stale_s,
                             'sources': self._sources(now),
                             'reset_counts': self.reset_counts,
                             'events': list(self.events),
                             'history': list(self.history) if include_history else []})
