import json
import time
import threading

import pytest

from rinbo_monitor.model import MonitorStore
from rinbo_monitor.recording import Recording


def wait_stopped(recording):
    recording.thread.join(timeout=3)
    assert recording.status()['state'] == 'stopped'
    return json.loads(recording.path.read_text())


def test_chosen_interval_has_full_snapshots_and_short_events(tmp_path):
    now = [100.0]
    store = MonitorStore(clock=lambda: now[0], recording_dir=tmp_path)
    store.ingest('motor', {'l3': {'position': 1.0, 'hall_effect': True}})
    store.ingest('logs', {'name': 'rinbo_cali', 'msg': 'before recording'})
    store.start_recording()
    now[0] += .1
    store.ingest('requested', {'l2': {'reset_position': True}})
    store.ingest('requested', {'l2': {'reset_position': False}})
    store.ingest('power', {'v_7': 24.0, 'i_3': .8})
    store.sample_history()
    now[0] += .1
    store.stop_recording()
    store.ingest('logs', {'name': 'rinbo_cali', 'msg': 'after recording'})
    result = wait_stopped(store.recording)
    samples = [e for e in result['entries'] if e['kind'] == 'sample']
    events = [e for e in result['entries'] if e['kind'] == 'event']
    assert len(samples) == 2  # immediate snapshot and 100 ms sample
    assert samples[0]['t_recording_s'] == 0
    assert samples[1]['data']['sources']['power']['data']['i_3'] == .8
    assert samples[1]['data']['sources']['requested']['data']['l2']['reset_position'] is False
    assert len(events) == 1 and 'L2 reset=true' in events[0]['data']['text']
    assert result['summary']['reason'] == 'user'
    assert store.recording_path().exists()


def test_recording_survives_live_ring_rollover_and_keeps_previous_file(tmp_path):
    store = MonitorStore(recording_dir=tmp_path)
    store.start_recording()
    with pytest.raises(ValueError):
        store.start_recording()
    for _ in range(605):
        store.sample_history()
        time.sleep(.0005)
    store.stop_recording()
    first = store.recording
    data = wait_stopped(first)
    assert data['summary']['samples'] == 606
    assert len(store.snapshot()['history']) == 600
    store.start_recording()
    store.close()
    assert store.recording.path != first.path
    assert first.path.exists() and store.recording.path.exists()


def test_download_requires_completed_recording(tmp_path):
    store = MonitorStore(recording_dir=tmp_path)
    with pytest.raises(ValueError):
        store.recording_path()
    store.start_recording()
    with pytest.raises(ValueError):
        store.recording_path()
    store.close()


def test_duration_limit_stops_without_changing_monitor(tmp_path):
    now = [10.0]
    recording = Recording(tmp_path, clock=lambda: now[0], max_seconds=1)
    recording.submit('sample', {'position': -55297.0})
    now[0] = 11.0
    recording.submit('sample', {'position': -1.0})
    result = wait_stopped(recording)
    assert result['summary']['reason'] == 'duration_limit'
    assert len(result['entries']) == 1
    assert result['entries'][0]['data']['position'] == -55297.0


def test_size_limit_and_shutdown_finalize_json(tmp_path):
    recording = Recording(tmp_path, max_bytes=1)
    recording.submit('sample', {'test': True})
    result = wait_stopped(recording)
    assert result['summary']['reason'] == 'size_limit'
    assert not recording.partial.exists()
    recording = Recording(tmp_path)
    recording.close()
    assert json.loads(recording.path.read_text())['summary']['reason'] == 'monitor_shutdown'


def test_stale_source_age_is_preserved_in_recording(tmp_path):
    now = [0.0]
    store = MonitorStore(clock=lambda: now[0], recording_dir=tmp_path)
    store.ingest('motor', {'l3': {'position': 1.0}})
    now[0] = 2.0
    store.start_recording()
    store.close()
    sample = json.loads(store.recording.path.read_text())['entries'][0]
    motor = sample['data']['sources']['motor']
    assert motor['stale'] and motor['age_s'] == 2
    assert motor['data']['l3']['position'] == 1


def test_slow_disk_queue_is_bounded_and_loss_is_reported(tmp_path, monkeypatch):
    release = threading.Event()
    writer = Recording._write
    def delayed(self):
        release.wait(timeout=2)
        writer(self)
    monkeypatch.setattr(Recording, '_write', delayed)
    recording = Recording(tmp_path, queue_size=1)
    recording.submit('sample', {'n': 1})
    recording.submit('sample', {'n': 2})
    assert recording.status()['reason'] == 'writer_queue_full'
    assert recording.status()['dropped_entries'] == 1
    release.set()
    result = wait_stopped(recording)
    assert len(result['entries']) == 1
    assert result['summary']['dropped_entries'] == 1
