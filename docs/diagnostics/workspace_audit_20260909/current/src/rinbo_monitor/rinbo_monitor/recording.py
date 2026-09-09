"""Bounded asynchronous recording; disk writes never run in ROS callbacks."""
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import threading
import time
import uuid


class Recording:
    def __init__(self, directory, clock=time.monotonic, wall_clock=time.time,
                 max_seconds=1800, max_bytes=64 * 1024 * 1024, queue_size=1000):
        self.clock = clock
        self.started = clock()
        self.started_unix_s = wall_clock()
        self.ended = None
        self.max_seconds, self.max_bytes = max_seconds, max_bytes
        self.lock = threading.Lock()
        self.queue = queue.Queue(maxsize=queue_size)
        self.stopping = threading.Event()
        self.state = 'recording'
        self.reason = ''
        self.samples = self.events = self.dropped = self.bytes_written = 0
        directory = Path(directory).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.fromtimestamp(self.started_unix_s, timezone.utc).strftime('%Y%m%dT%H%M%S')
        self.path = directory / ('rinbo-{}-{}.json'.format(stamp, uuid.uuid4().hex[:8]))
        self.partial = self.path.with_suffix('.json.partial')
        # Creation errors are returned to the start request, not a ROS callback.
        self.file = self.partial.open('x', encoding='utf-8')
        self.partial.chmod(0o600)
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def status(self):
        with self.lock:
            return {'state': self.state, 'reason': self.reason,
                    'started_unix_s': self.started_unix_s,
                    'duration_s': (self.ended if self.ended is not None else self.clock()) - self.started,
                    'samples': self.samples, 'events': self.events,
                    'dropped_entries': self.dropped, 'bytes_written': self.bytes_written,
                    'filename': self.path.name, 'path': str(self.path),
                    'max_seconds': self.max_seconds, 'max_bytes': self.max_bytes,
                    'download_ready': self.state == 'stopped'}

    def _stop_locked(self, reason, now):
        if self.state != 'recording':
            return
        self.reason = reason
        self.ended = now
        self.state = 'finalizing'
        self.stopping.set()

    def stop(self, reason='user'):
        with self.lock:
            self._stop_locked(reason, self.clock())

    def submit(self, kind, data):
        now = self.clock()
        with self.lock:
            if self.state != 'recording':
                return
            if now - self.started >= self.max_seconds:
                self._stop_locked('duration_limit', now)
                return
            entry = {'kind': kind, 't_recording_s': now - self.started, 'data': data}
            try:
                self.queue.put_nowait(entry)
            except queue.Full:
                self.dropped += 1
                self._stop_locked('writer_queue_full', now)

    def _write(self):
        try:
            metadata = {'schema_version': 2, 'format': 'rinbo_monitor_recording',
                        'sample_hz': 10, 'started_unix_s': self.started_unix_s,
                        'position_units': 'raw encoder counts; no modulo or unwrap',
                        'transport': 'best_effort; received messages only',
                        'entries': []}
            prefix = json.dumps(metadata, ensure_ascii=False)[:-2] + '\n'
            self.file.write(prefix)
            self.bytes_written = len(prefix.encode('utf-8'))
            first = True
            while True:
                try:
                    entry = self.queue.get(timeout=.1)
                except queue.Empty:
                    if self.stopping.is_set():
                        break
                    continue
                text = ('' if first else ',\n') + json.dumps(entry, ensure_ascii=False, allow_nan=False)
                self.file.write(text)
                first = False
                with self.lock:
                    self.bytes_written += len(text.encode('utf-8'))
                    if entry['kind'] == 'sample':
                        self.samples += 1
                    else:
                        self.events += 1
                    if self.bytes_written >= self.max_bytes:
                        self._stop_locked('size_limit', self.clock())
                # Flush at 10 Hz or slower, in this worker only.
                if self.queue.empty():
                    self.file.flush()
            summary = self.status()
            summary['state'] = 'stopped'
            summary['download_ready'] = True
            self.file.write('\n], "summary": ' + json.dumps(summary, ensure_ascii=False) + '}\n')
            self.file.close()
            self.partial.replace(self.path)
            with self.lock:
                self.state = 'stopped'
        except Exception as error:
            self.file.close()
            with self.lock:
                self.state = 'error'
                self.reason = 'write_error: ' + str(error)
                if self.ended is None:
                    self.ended = self.clock()
                self.stopping.set()

    def close(self):
        self.stop('monitor_shutdown')
        self.thread.join(timeout=5)
