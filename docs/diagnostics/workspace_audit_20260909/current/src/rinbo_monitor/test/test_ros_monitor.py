"""Run only with ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=1. No hardware bridge."""
import os
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != '232' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
    reason='requires isolated localhost test domain 232')


def test_passive_subscriber_graph_and_reset_pulse_over_ros():
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rinbo_msgs.msg import MotorCmdStamped, MotorStateStamped
    from rinbo_monitor.model import MonitorStore
    from rinbo_monitor.server import create_monitor_node

    rclpy.init()
    store = MonitorStore()
    monitor = create_monitor_node(store)
    fake = rclpy.create_node('monitor_test_fake_bridge', enable_rosout=False)
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(fake)
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    mirror = fake.create_publisher(MotorCmdStamped, '/rinbo/monitor/motor_forwarded', qos)
    state = fake.create_publisher(MotorStateStamped, '/motor/state', qos)
    # Create the sole bridge command endpoint, but never publish a control command.
    fake.create_subscription(MotorCmdStamped, '/motor/command', lambda _: None, 10)

    def until(predicate, seconds=5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        assert predicate()

    try:
        until(lambda: mirror.get_subscription_count() == 1 and state.get_subscription_count() == 1)
        until(lambda: monitor.count_subscribers('/motor/command') == 1)
        assert monitor.count_publishers('/motor/command') == 0
        assert monitor.count_publishers('/power/command') == 0
        for topic in ('/motor/command', '/power/command', '/estop'):
            assert not any(e.node_name == 'rinbo_monitor' for e in monitor.get_subscriptions_info_by_topic(topic))
        pulse = MotorCmdStamped()
        pulse.l2.reset_position = True
        pulse.l2.enable = False
        pulse.header.seq = 12
        mirror.publish(pulse)
        until(lambda: store.snapshot()['reset_counts']['forwarded']['l2'] == 1)
        pulse.l2.reset_position = False
        mirror.publish(pulse)
        feedback = MotorStateStamped()
        feedback.l2.position = 9876.0
        feedback.l3.hall_effect = True
        state.publish(feedback)
        until(lambda: store.snapshot()['sources']['motor']['data'] is not None)
        until(lambda: store.snapshot()['sources']['forwarded']['count'] == 2)
        snapshot = store.snapshot()
        assert snapshot['reset_counts']['forwarded']['l2'] == 1
        assert snapshot['sources']['motor']['data']['l2']['position'] == 9876.0
        assert snapshot['sources']['motor']['data']['l3']['hall_effect'] is True
        until(lambda: store.snapshot()['sources']['motor']['stale'], 2)
    finally:
        executor.shutdown()
        monitor.destroy_node()
        fake.destroy_node()
        rclpy.shutdown()
