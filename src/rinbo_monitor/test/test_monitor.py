import json
import math
from pathlib import Path
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen, build_opener, HTTPCookieProcessor
from http.server import ThreadingHTTPServer

import pytest

from rinbo_monitor.model import LEGS, MonitorStore, TOPICS
from rinbo_monitor.server import make_handler


def command(reset=False):
    return {'header': {'seq': 7, 'stamp': {'sec': 100, 'nanosec': 0}},
            'servo_control_mode': 2,
            **{leg: {'voltage': 20.0, 'reset_position': reset and leg == 'l2'} for leg in LEGS},
            **{'s' + leg: {'position_encoder': 1234} for leg in LEGS}}


def test_single_packet_reset_survives_latest_command_and_snapshot_rate():
    store = MonitorStore()
    store.ingest('requested', command(True))
    store.ingest('forwarded', command(True))
    store.ingest('requested', command(False))
    store.ingest('forwarded', command(False))
    result = store.snapshot()
    assert result['sources']['forwarded']['data']['l2']['reset_position'] is False
    assert result['reset_counts']['forwarded']['l2'] == 1
    assert len([e for e in result['events'] if e['kind'] == 'reset']) == 2
    assert result['reset_counts']['forwarded']['r2'] == 0


def test_requested_reset_does_not_claim_forwarded_or_completed():
    store = MonitorStore()
    store.ingest('requested', command(True))
    assert store.snapshot()['reset_counts']['forwarded']['l2'] == 0
    assert store.snapshot()['sources']['forwarded']['data'] is None


def test_stale_data_gaps_history_but_retains_last_readout():
    now = [10.0]
    store = MonitorStore(clock=lambda: now[0], wall_clock=lambda: 101.5)
    assert all(v['stale'] for v in store.snapshot()['sources'].values())
    store.ingest('forwarded', command())
    store.sample_history()
    assert store.snapshot()['history'][-1]['legs'][1][1] == 20
    assert store.snapshot()['sources']['forwarded']['source_age_s'] == 1.5
    now[0] += .6
    store.sample_history()
    snapshot = store.snapshot()
    assert snapshot['sources']['forwarded']['stale']
    assert snapshot['sources']['forwarded']['data']['l2']['voltage'] == 20
    assert snapshot['history'][-1]['legs'][1][1] is None


def test_nan_and_infinity_are_missing_not_zero():
    store = MonitorStore()
    store.ingest('motor', {'l2': {'position': math.nan}, 'r1': {'position': math.inf}})
    snapshot = store.snapshot()
    assert snapshot['sources']['motor']['data']['l2']['position'] is None
    json.dumps(snapshot, allow_nan=False)


def test_hall_events_use_actual_legs_and_active_low():
    store = MonitorStore()
    store.ingest('motor', {'l3': {'hall_effect': True}})
    store.ingest('motor', {'l3': {'hall_effect': False}})
    assert store.snapshot()['events'][-1]['text'] == 'L3 Hall 1 → 0 (0=原點觸發)'


def test_memory_is_bounded_and_snapshots_are_independent():
    store = MonitorStore()
    for _ in range(650):
        store.ingest('requested', command(True))
        store.sample_history()
    snapshot = store.snapshot()
    assert len(snapshot['events']) == 300
    assert len(snapshot['history']) == 600
    snapshot['reset_counts']['requested']['l2'] = -1
    assert store.snapshot()['reset_counts']['requested']['l2'] == 650


def test_http_export_and_recording_routes(tmp_path):
    store = MonitorStore(recording_dir=tmp_path)
    store.ingest('requested', command(True))
    token = 'test-only-access-token'
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(store, token))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    base = 'http://127.0.0.1:{}'.format(server.server_port)
    def authenticated(path, **kwargs):
        return Request(base + path, headers={'Cookie': 'rinbo_monitor_token=' + token}, **kwargs)
    try:
        with pytest.raises(HTTPError) as error:
            urlopen(base + '/api/export')
        assert error.value.code == 401
        with pytest.raises(HTTPError) as error:
            urlopen(Request(base + '/', headers={'Cookie': 'rinbo_monitor_token=wrong'}))
        assert error.value.code == 401
        opener = build_opener(HTTPCookieProcessor())
        with opener.open(base + '/?token=' + token) as response:
            assert response.geturl() == base + '/'
            assert 'Rinbo 即時監控' in response.read().decode()
        with opener.open(base + '/api/snapshot') as response:
            assert json.load(response)['reset_counts']['requested']['l2'] == 1
        with urlopen(authenticated('/')) as response:
            assert 'Rinbo 即時監控' in response.read().decode()
        with urlopen(authenticated('/api/export')) as response:
            assert 'attachment' in response.headers['Content-Disposition']
            assert json.load(response)['reset_counts']['requested']['l2'] == 1
        with pytest.raises(HTTPError) as error:
            urlopen(authenticated('/api/snapshot', data=b'{}', method='POST'))
        assert error.value.code == 403
        with pytest.raises(HTTPError) as error:
            urlopen(Request(base + '/api/recording/start', method='POST', headers={'X-Rinbo-Recording': '1'}))
        assert error.value.code == 401
        start = authenticated('/api/recording/start', method='POST')
        start.add_header('X-Rinbo-Recording', '1')
        with urlopen(start) as response:
            assert json.load(response)['state'] == 'recording'
        with pytest.raises(HTTPError) as error:
            urlopen(start)
        assert error.value.code == 409
        store.sample_history()
        stop = authenticated('/api/recording/stop', method='POST')
        stop.add_header('X-Rinbo-Recording', '1')
        with urlopen(stop) as response:
            assert json.load(response)['state'] in ('finalizing', 'stopped')
        store.recording.thread.join(timeout=3)
        with urlopen(authenticated('/api/recording/download')) as response:
            recording = json.load(response)
            assert recording['format'] == 'rinbo_monitor_recording'
            assert recording['summary']['samples'] == 2
        with pytest.raises(HTTPError) as error:
            urlopen(authenticated('/../../etc/passwd'))
        assert error.value.code == 404
    finally:
        server.shutdown()
        thread.join()
        server.server_close()


def test_observer_topics_exclude_command_and_control_channels():
    assert '/motor/command' not in TOPICS.values()
    assert '/power/command' not in TOPICS.values()
    assert '/estop' not in TOPICS.values()
    source = Path(__file__).parents[2] / 'rinbo_ros_bridge/src/rinbo_ros_bridge.cpp'
    text = source.read_text()
    function = text.split('void publish_motor_command_to_grpc_locked(')[1].split('void publish_all_disabled_locked')[0]
    assert function.index('grpc_motor_cmd_pub->publish') < function.index('publish_command_monitor')
    assert 'publish_command_monitor(motor_forwarded_monitor_pub, command)' in function
