"""Use localhost domain 232; never start the real Bridge or publish commands."""
import os
import time
import pytest


def test_feedback_and_humble_parameter_service():
    # Fail before rclpy.init if an operator accidentally uses the robot domain.
    if os.environ.get('ROS_DOMAIN_ID') != '232' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        pytest.skip('Requires ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=1')
    from rinbo_control.feedback import Feedback
    from rinbo_msgs.msg import MotorStateStamped, PowerStateStamped
    from std_msgs.msg import Bool
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    import rclpy
    import threading
    feedback=Feedback()
    bridge=rclpy.create_node('rinbo_ros2_bridge')
    bridge.declare_parameter('core_ip','192.168.30.254')
    motor=bridge.create_publisher(MotorStateStamped,'/motor/state',1)
    power=bridge.create_publisher(PowerStateStamped,'/power/state',1)
    output=bridge.create_publisher(Bool,'/rinbo/motor_output_enabled',1)
    values={'relay':False,'voltage':24.0,'motor':True}
    sequence=0
    def send():
        nonlocal sequence
        sequence+=1
        m=MotorStateStamped(); m.header.seq=sequence; m.header.stamp=bridge.get_clock().now().to_msg()
        m.l2.position = -27648.0
        m.r2.position = 13824.0
        m.l2.hall_effect = True
        p=PowerStateStamped(); p.header.seq=sequence; p.header.stamp=bridge.get_clock().now().to_msg()
        p.digital=p.signal=True; p.v_7=values['voltage']; p.power=values['relay']
        if values['motor']:
            motor.publish(m)
        power.publish(p)
        output.publish(Bool(data=False))
    timer=bridge.create_timer(.02,send)
    executor=SingleThreadedExecutor(); executor.add_node(bridge)
    thread=threading.Thread(target=executor.spin); thread.start()
    try:
        end=time.monotonic()+3
        while time.monotonic()<end:
            snapshot=feedback.status()
            if snapshot['motor_fresh'] and snapshot['power_fresh']: break
        assert snapshot['bridge_count']==1
        assert snapshot['motor_fresh'] and snapshot['power_fresh']
        assert snapshot['sensors_on'] and snapshot['relay_on'] is False
        assert snapshot['legs']['L2'] == dict(angle_deg=180.0, hall=True)
        assert snapshot['legs']['R2'] == dict(angle_deg=90.0, hall=False)
        assert feedback.bridge_ip()=='192.168.30.254'
        assert not bridge.get_subscriptions_info_by_topic('/motor/command')
        feedback.wait_motor_disabled()
        values['relay']=True
        # Wait for the source to change, then require three subsequent samples.
        feedback.spin(.1)
        feedback.wait_power_ready([])
        values['voltage']=17.0
        with pytest.raises(RuntimeError,match='電源回讀未通過'):
            feedback.wait_power_ready([])
        epoch=feedback.sensor_epoch
        values['motor']=False
        feedback.spin(.5)
        assert feedback.fresh('/power/state') is not None
        assert feedback.fresh('/motor/state') is None
        assert feedback.sensor_epoch > epoch
        values['motor']=True
        end=time.monotonic()+2
        while feedback.fresh('/motor/state') is None and time.monotonic()<end:
            feedback.spin(.02)
        assert feedback.fresh('/motor/state') is not None
        feedback.spin(.05)
        epoch=feedback.sensor_epoch
        bridge.destroy_timer(timer)
        feedback.spin(.65)
        assert feedback.fresh('/motor/state') is None
        assert feedback.fresh('/power/state') is None
        assert feedback.sensor_epoch > epoch
        stale = feedback.status()
        assert stale['legs'] == {} and stale['voltage'] is None
    finally:
        executor.shutdown(); thread.join(timeout=3)
        bridge.destroy_node(); feedback.close()
