"""Recorder regression: diagnostic mirrors are data, never hardware commands.

Integration test requires ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=1.
"""
import importlib.util
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import pytest
import yaml


def launch_module():
    path = Path(__file__).parents[1] / 'launch/record_experiment.launch.py'
    spec = importlib.util.spec_from_file_location('record_experiment', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
    pubs = {topic: node.create_publisher(type(msg), topic, qos)
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
