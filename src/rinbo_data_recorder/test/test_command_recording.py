"""Recorder regression: diagnostic mirrors are data, never hardware commands.

Integration test requires ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=1.
"""
import importlib.util
import csv
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import pytest
import yaml


@pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != '232' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
    reason='requires isolated localhost ROS domain 232')
def test_csv_recorder_observes_both_commands_without_publishing_them(tmp_path):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rinbo_msgs.msg import MotorCmdStamped, PowerCmdStamped

    rclpy.init()
    node = rclpy.create_node('recorder_command_fixture', enable_rosout=False)
    motor = node.create_publisher(MotorCmdStamped, '/motor/command', 10)
    power = node.create_publisher(PowerCmdStamped, '/power/command', 10)
    executable = Path(os.environ.get('RINBO_RECORDER_TEST_BINARY',str(Path(get_package_prefix('rinbo_data_recorder')) / 'lib/rinbo_data_recorder/rinbo_data_recorder')))
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'auto_start:=true',
        '-p', f'output_dir:={tmp_path}', '-p', 'summary_hz:=20.0',
        '-p', 'command_source:=legacy', '-p', f'lock_file:={tmp_path}/recorder.lock',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic() + 8
        saw_csv = False
        while time.monotonic() < deadline:
            assert process.poll() is None, process.stderr.read()
            m = MotorCmdStamped()
            m.l2.enable = True
            m.l2.direction = False
            m.l2.voltage = 12.0
            motor.publish(m)
            power.publish(PowerCmdStamped(digital=True))
            rclpy.spin_once(node, timeout_sec=.05)
            for topic in ('/motor/command', '/power/command'):
                publishers = node.get_publishers_info_by_topic(topic)
                assert all(e.node_name != 'rinbo_data_recorder' for e in publishers)
            path = tmp_path / 'summary.csv'
            if path.exists():
                with path.open() as stream:
                    rows = list(csv.DictReader(stream))
                saw_csv = any(row.get('cmd_voltage_l2') == '12.000000' and
                              row.get('power_cmd_digital') == '1' for row in rows)
                if saw_csv:
                    break
        assert saw_csv, 'recorder must continue collecting both command streams'
        for topic in ('/motor/command', '/power/command'):
            observers = node.get_subscriptions_info_by_topic(topic)
            assert [(e.node_name, e.node_namespace) for e in observers] == [('rinbo_data_recorder', '/')]
    finally:
        # This handle belongs only to our isolated test child, never the live recorder.
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        process.stderr.close()
        node.destroy_node()
        rclpy.shutdown()


def launch_module():
    path = Path(__file__).parents[1] / 'launch/record_experiment.launch.py'
    spec = importlib.util.spec_from_file_location('record_experiment', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_never_reuses_a_previous_csv_or_bag_directory(tmp_path):
    module = launch_module()
    previous = tmp_path / 'experiment'
    previous.mkdir()
    (previous / 'summary.csv').write_text('history\n')
    fresh = Path(module._unique_run_dir(str(tmp_path), 'test', str(previous)))
    assert fresh != previous and fresh.is_dir()
    assert (previous / 'summary.csv').read_text() == 'history\n'


def test_custom_topic_list_is_used_for_both_bag_and_metadata(tmp_path, monkeypatch):
    from launch import LaunchContext
    module = launch_module()
    context = LaunchContext()
    context.launch_configurations.update({
        'output_root': str(tmp_path), 'output_dir': str(tmp_path / 'run'),
        'run_name': 'test', 'profile': 'test', 'config_file': '/unused.yaml',
        'start_paused': 'false', 'record_bag': 'true', 'record_csv': 'true',
        'bag_topics': '/motor/state, /rinbo/monitor/motor_forwarded',
    })
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(module, 'ExecuteProcess', lambda **kwargs: kwargs)
    node, process = module._launch_setup(context)
    expected = ['/motor/state', '/rinbo/monitor/motor_forwarded']
    assert node['parameters'][-1]['bag_topics'] == expected
    assert process['cmd'][-2:] == expected


@pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != '232' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
    reason='requires isolated localhost ROS domain 232')
def test_real_bag_preserves_requested_forwarded_and_encoder_data(tmp_path):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from rinbo_msgs.msg import MotorCmdStamped, MotorStateStamped
    from std_msgs.msg import Bool

    rclpy.init()
    node = rclpy.create_node('recorder_test_fake_telemetry', enable_rosout=False)
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    requested = MotorCmdStamped()
    requested.header.seq = 123
    requested.l1.enable = True
    requested.l1.direction = False
    requested.l1.voltage = 40.0
    # Simulate a Bridge rejection: forwarded output must stay distinguishable.
    forwarded = MotorCmdStamped()
    forwarded.header.seq = 124
    feedback = MotorStateStamped()
    feedback.l1.position = -1234.0
    feedback.l1.tick_count = 4321
    feedback.l1.hall_effect = True
    samples = {
        '/rinbo/monitor/motor_requested': requested,
        '/rinbo/monitor/motor_forwarded': forwarded,
        '/motor/state': feedback,
        '/rinbo/motor_output_enabled': Bool(data=False),
        '/rinbo/motor_arbiter_ready': Bool(data=False),
    }
    pubs = {topic: node.create_publisher(type(msg), topic,
            qos if topic.startswith('/rinbo/monitor/') else 100)
            for topic, msg in samples.items()}
    # No /motor/command, /power/command, /estop publishers; no Bridge launched.
    run_dir = tmp_path / 'recorded'
    process = None
    try:
        with (tmp_path / 'launch.log').open('w') as log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'rinbo_data_recorder', 'record_experiment.launch.py',
                 'output_dir:=' + str(run_dir)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 20
            while not all(pub.get_subscription_count() for pub in pubs.values()):
                assert process.poll() is None, (tmp_path / 'launch.log').read_text()
                assert time.monotonic() < deadline, (tmp_path / 'launch.log').read_text()
                rclpy.spin_once(node, timeout_sec=.05)
            for _ in range(30):
                for topic, msg in samples.items():
                    pubs[topic].publish(msg)
                rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()

    metadata = yaml.safe_load((run_dir / 'metadata.yaml').read_text())
    assert set(samples) <= set(metadata['bag_topics'])
    db_path = next((run_dir / 'raw_bag').glob('*.db3'))
    with sqlite3.connect('file:' + str(db_path) + '?mode=ro', uri=True) as db:
        topics = dict(db.execute('select name,id from topics'))
        for topic, expected in samples.items():
            rows = db.execute('select data from messages where topic_id=?', (topics[topic],)).fetchall()
            assert rows, topic
            actual = deserialize_message(rows[-1][0], type(expected))
            assert actual == expected, topic
