"""Read-only ROS view. Never subscribes to /motor/command or publishes commands."""
import math
import os
import threading
import time
from .plans import LEGS


class Feedback:
    def __init__(self):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rinbo_msgs.msg import MotorStateStamped, PowerStateStamped
        from std_msgs.msg import Bool
        self.ros = rclpy
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node(f'rinbo_control_console_{os.getpid()}', use_global_arguments=False)
        self.samples = {}
        self.previous = {}
        self.problem = ''
        self.motor_output = None
        self.sensor_epoch = 0
        self._sensors_were_fresh = False
        self._stopping = threading.Event()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subs = []
        for topic, kind in (('/motor/state', MotorStateStamped), ('/power/state', PowerStateStamped)):
            def callback(msg, topic=topic):
                self.accept(topic, msg)
            self.subs.append(self.node.create_subscription(kind, topic, callback, qos))
        self.subs.append(self.node.create_subscription(
            Bool, '/rinbo/motor_output_enabled', self.accept_motor_output, qos))
        from rclpy.executors import SingleThreadedExecutor
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()

    def _listen(self):
        try:
            while not self._stopping.is_set():
                self.executor.spin_once(timeout_sec=.02)
                p = self.fresh('/power/state')
                # Preserve calibration only while both encoder and power
                # feedback remain continuous, including while input() waits.
                good = bool(p and p.digital and p.signal and self.fresh('/motor/state'))
                if self._sensors_were_fresh and not good:
                    self.sensor_epoch += 1
                self._sensors_were_fresh = good
        except Exception as exc:
            self.problem = str(exc)
            self.samples.clear()
            self.sensor_epoch += 1

    def accept(self, topic, msg):
        observed = time.monotonic()
        peers = self.node.get_publishers_info_by_topic(topic)
        if len(peers) != 1 or peers[0].node_name != 'rinbo_ros2_bridge' or peers[0].node_namespace != '/':
            self.problem = f'{topic} 沒有唯一的 Bridge 來源'
            self.samples.pop(topic, None)
            return
        h = msg.header
        stamp = h.stamp.sec * 10**9 + h.stamp.nanosec
        age_limit = .1 if topic == '/motor/state' else .35
        now = self.node.get_clock().now().nanoseconds
        old = self.previous.get(topic)
        if stamp <= 0 or h.stamp.nanosec >= 10**9 or abs(now-stamp) > age_limit*1e9:
            self.samples.pop(topic, None)
            return
        # This installed Humble rclpy only supplies a message, not MessageInfo.
        # This UI check is advisory (sole graph endpoint + source freshness).
        # The existing C++ controllers retain their per-packet GID validation.
        gid = bytes(peers[0].endpoint_gid)
        if old and old[2] != gid:
            self.sensor_epoch += 1
        if old and old[2] == gid and (stamp <= old[0] or not 0 < ((h.seq-old[1]) & 0xffffffff) < 2**31):
            self.samples.pop(topic, None)
            return
        self.previous[topic] = (stamp, h.seq, gid)
        self.samples[topic] = (msg, observed)

    def spin(self, duration=.1):
        # Background listener also watches power continuity while input() waits.
        self._stopping.wait(duration)

    def accept_motor_output(self, msg):
        peers = self.node.get_publishers_info_by_topic('/rinbo/motor_output_enabled')
        if len(peers) != 1 or peers[0].node_name != 'rinbo_ros2_bridge' or peers[0].node_namespace != '/':
            self.motor_output = None
            return
        # Bool has no header. Require repeated live delivery after entering the
        # gate; the C++ controller still verifies stamped active/rearm ACKs.
        self.motor_output = (bool(msg.data), time.monotonic())

    def wait_motor_disabled(self, cancel=lambda:False):
        entered = time.monotonic()
        while time.monotonic()-entered < 6:
            if cancel():
                raise RuntimeError('已取消上電前檢查')
            output = self.motor_output
            if (output and not output[0] and output[1] > entered and
                    time.monotonic()-output[1] < .2 and not self.writers()):
                return
            self.spin(.02)
        raise RuntimeError('未確認 motor output=false 且 motor command 發布者為零；不開 Relay')

    def wait_power_ready(self, disabled_legs, cancel=lambda:False):
        from redrhex_lowlevel_bridge.rinbo_power_tool import _relay_feedback_violation
        entered = time.monotonic()
        epoch = self.sensor_epoch
        last = None
        count = 0
        while time.monotonic()-entered < 6:
            if cancel():
                raise RuntimeError('已取消電源回讀檢查')
            if self.sensor_epoch != epoch:
                raise RuntimeError('感測器回讀中斷，不能沿用已上電狀態')
            sample = self.samples.get('/power/state')
            p = self.fresh('/power/state')
            if p is not None and sample is not None and sample[1] > entered and sample[1] != last:
                last = sample[1]
                if not (p.digital and p.signal and p.power):
                    raise RuntimeError('原已上電的電源狀態改變，不會自動補送上電')
                violation = _relay_feedback_violation(p, tuple(disabled_legs))
                if violation:
                    raise RuntimeError('電源回讀未通過：'+violation)
                count += 1
                if count >= 3:
                    return
            elif p is None:
                count = 0
            self.spin(.02)
        raise RuntimeError('未收到連續 3 筆有效電源回讀；停止操作')

    def fresh(self, topic):
        sample = self.samples.get(topic)
        if not sample or time.monotonic()-sample[1] > (.25 if topic == '/motor/state' else .5):
            return None
        return sample[0]

    def bridge_count(self):
        return sum(n == 'rinbo_ros2_bridge' and ns == '/' for n, ns in self.node.get_node_names_and_namespaces())

    def writers(self):
        return [p.node_namespace.rstrip('/')+'/'+p.node_name
                for p in self.node.get_publishers_info_by_topic('/motor/command')]

    def bridge_ip(self):
        from rcl_interfaces.srv import GetParameters
        client = self.node.create_client(GetParameters, '/rinbo_ros2_bridge/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=2):
                raise RuntimeError('無法讀取既有 Bridge 的位址；請確認它仍在執行')
            request = GetParameters.Request()
            request.names = ['core_ip']
            future = client.call_async(request)
            end = time.monotonic()+2
            while not future.done() and time.monotonic() < end:
                self.spin(.02)
            if not future.done() or not future.result() or len(future.result().values) != 1:
                raise RuntimeError('讀取 Bridge 位址逾時')
            return future.result().values[0].string_value
        finally:
            self.node.destroy_client(client)

    def status(self):
        self.spin(.15)
        p = self.fresh('/power/state')
        m = self.fresh('/motor/state')
        legs = {}
        if m is not None:
            for name in LEGS:
                leg = getattr(m, name.lower())
                # Match rinbo_fsm/src/manual_motion.hpp::encoder_degrees:
                # position is encoder counts, 55296 per turn; left is reversed.
                angle = leg.position * (-1 if name.startswith('L') else 1) * 360.0 / 55296.0
                legs[name] = dict(angle_deg=angle if math.isfinite(angle) else None,
                                  hall=bool(leg.hall_effect))
        return {'bridge_count': self.bridge_count(), 'writers': self.writers(),
                'motor_fresh': m is not None, 'power_fresh': p is not None,
                'sensors_on': bool(p and p.digital and p.signal),
                'relay_on': bool(p.power) if p else None,
                'voltage': p.v_7 if p and math.isfinite(p.v_7) else None,
                'legs': legs, 'problem': self.problem if m is None or p is None else ''}

    def close(self):
        self._stopping.set()
        self.thread.join(timeout=2)
        self.executor.shutdown()
        self.node.destroy_node()
        self.ros.shutdown()
