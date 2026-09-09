#!/usr/bin/env python3
import os
import sys
import subprocess
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

from std_msgs.msg import Bool, String
from rinbo_msgs.msg import MotorCmdStamped, MotorStateStamped, PowerCmdStamped, PowerStateStamped


class RinboControlPanel(QWidget):
    def __init__(self, node):
        super(RinboControlPanel, self).__init__()
        self.node = node
        self.init_ui()
        self.init_ros()
        self.reset()

    def init_ui(self):

        # --- 共用按鈕樣式 ---
        btn_style = '''QPushButton {background-color: white; text-align: center; border-radius: 5px; color: black;}
                       QPushButton:checked {background-color: lightblue;}
                       QPushButton:hover:!checked {background-color: silver;}
                       QPushButton:hover:pressed {background-color: deepskyblue; border-style: inset;}'''

        # --- 0. ROS Bridge 區塊 (補回遺漏的按鈕) ---
        btn_standard_style = '''QPushButton {background-color: white; text-align: center; border-radius: 5px; color: black;}
                               QPushButton:checked {background-color: lavender; color: black;}
                               QPushButton:hover:!checked {background-color: silver;}
                               QPushButton:hover:pressed {background-color: gray; border-style: inset;}
                               QPushButton:disabled {background-color: lightgray; color: gray;}'''

        self.btn_ros_bridge = QPushButton('Run ROS Bridge', self)
        self.btn_ros_bridge.setStyleSheet(btn_standard_style)
        self.btn_ros_bridge.setCheckable(True)
        self.btn_ros_bridge.clicked.connect(self.ros_bridge_cmd)

        # --- 1. Digital 區塊 ---
        self.label_digital = QLabel('Digital:', self)
        self.label_digital.setStyleSheet('color: white; font-weight: bold;')
        self.btn_group_digital = QButtonGroup(self)
        self.btn_group_digital.setExclusive(True)
        self.btn_digital_on = QPushButton('ON', self)
        self.btn_digital_off = QPushButton('OFF', self)

        for btn in [self.btn_digital_on, self.btn_digital_off]:
            btn.setCheckable(True)
            btn.setStyleSheet(btn_style)
            btn.clicked.connect(self.publish_power_cmd)
            self.btn_group_digital.addButton(btn)
        self.btn_digital_off.setChecked(True)

        # --- 2. Signal 區塊 ---
        self.label_signal = QLabel('Signal:', self)
        self.label_signal.setStyleSheet('color: white; font-weight: bold;')
        self.btn_group_signal = QButtonGroup(self)
        self.btn_group_signal.setExclusive(True)
        self.btn_signal_on = QPushButton('ON', self)
        self.btn_signal_off = QPushButton('OFF', self)

        for btn in [self.btn_signal_on, self.btn_signal_off]:
            btn.setCheckable(True)
            btn.setStyleSheet(btn_style)
            btn.clicked.connect(self.publish_power_cmd)
            self.btn_group_signal.addButton(btn)
        self.btn_signal_off.setChecked(True)

        # --- 3. Power 區塊 ---
        self.label_power = QLabel('Power:', self)
        self.label_power.setStyleSheet('color: white; font-weight: bold;')
        self.btn_group_power = QButtonGroup(self)
        self.btn_group_power.setExclusive(True)
        self.btn_power_on = QPushButton('ON', self)
        self.btn_power_off = QPushButton('OFF', self)

        for btn in [self.btn_power_on, self.btn_power_off]:
            btn.setCheckable(True)
            btn.setStyleSheet(btn_style)
            btn.clicked.connect(self.publish_power_cmd)
            self.btn_group_power.addButton(btn)
        self.btn_power_off.setChecked(True)

        ### Mode Switch Button ###
        self.label_mode = QLabel('Robot Mode:', self)
        self.label_mode.setStyleSheet('color: white; font-weight: bold;')
        
        self.btn_group_mode = QButtonGroup(self)
        self.btn_group_mode.setExclusive(True)
        
        self.btn_calibration = QPushButton('Calibration', self)
        self.btn_standing = QPushButton('Standing', self)
        self.btn_tripod = QPushButton('Tripod Gait', self)
        
        btn_mode_list = [self.btn_calibration, self.btn_standing, self.btn_tripod]
        btn_mode_style = '''QPushButton {background-color: white; color: black; text-align: center; border-radius: 5px;}
                            QPushButton:checked {background-color: palegreen; color: black;}
                            QPushButton:hover:!checked {background-color: silver; color: black;}
                            QPushButton:hover:pressed {background-color: mediumseagreen; border-style: inset;}
                            QPushButton:disabled {background-color: lightgray; color: gray;}'''
        
        for id, btn in enumerate(btn_mode_list):
            btn.setCheckable(True)
            btn.setStyleSheet(btn_mode_style)
            btn.clicked.connect(self.mode_switch_cmd)
            self.btn_group_mode.addButton(btn, id)

        ### Data Output File Name ###
        self.label_output = QLabel('Output File Name:', self)
        self.label_output.setStyleSheet('color: white; font-weight: bold;')
        
        self.edit_output = QLineEdit('', self)
        self.edit_output.setStyleSheet('''background-color: white; color: black;''')
        
        ### Trigger Button ###
        self.btn_trigger = QPushButton('Record', self)
        self.btn_trigger.setCheckable(True)
        self.btn_trigger.setStyleSheet('''QPushButton {background-color: white; color: black; text-align: center; border-radius: 5px;}
                                          QPushButton:checked {background-color: skyblue; color: black;}
                                          QPushButton:hover:!checked {background-color: silver; color: black;}
                                          QPushButton:hover:checked {background-color: cornflowerblue; color: black;}
                                          QPushButton:hover:pressed {background-color: royalblue; border-style: inset;}
                                          QPushButton:disabled {background-color: lightgray; color: gray;}''')
        self.btn_trigger.clicked.connect(self.publish_trigger_cmd)
        
        ### Stop All Button ###
        self.btn_stop_all = QPushButton('STOP ALL', self)
        self.btn_stop_all.setStyleSheet('''QPushButton {background-color: red; color: white; text-align: center; border-radius: 5px; font-weight: bold;}
                                           QPushButton:hover {background-color: darkred;}
                                           QPushButton:pressed {background-color: maroon; border-style: inset;}''')
        self.btn_stop_all.clicked.connect(self.stop_all)

        ### Reset Button ###
        self.btn_reset = QPushButton('Reset', self)
        self.btn_reset.setStyleSheet('''QPushButton {background-color: plum; color: black; text-align: center; border-radius: 5px;}
                                        QPushButton:hover:!checked {background-color: violet; color: black;}
                                        QPushButton:hover:pressed {background-color: darkviolet; border-style: inset;}
                                        QPushButton:disabled {background-color: lightgray; color: gray;}''')
        self.btn_reset.clicked.connect(self.reset)

        ### Status Labels ###
        self.label_status = QLabel('Status:', self)
        self.label_status.setStyleSheet('color: white; font-weight: bold;')
        
        self.label_voltage = QLabel('Voltage: -- V', self)
        self.label_voltage.setStyleSheet('color: gold;')
        
        self.label_current = QLabel('Current: -- A', self)
        self.label_current.setStyleSheet('color: gold;')

        self.label_mode_status = QLabel('Mode: --', self)
        self.label_mode_status.setStyleSheet('color: gold;')

        ### Vertical Line ###
        vline = QWidget()
        vline.setFixedWidth(5)
        vline.setStyleSheet('background-color: black;')

        ### Arrange Elements ###
        layout = QGridLayout()
        layout.setSpacing(15) 
        
        # Left column - Control
        layout.addWidget(self.btn_ros_bridge,   0, 0, 1, 2)

        layout.addWidget(self.label_digital,    1, 0, 1, 2)
        layout.addWidget(self.btn_digital_on,   2, 0, 1, 1)
        layout.addWidget(self.btn_digital_off,  2, 1, 1, 1)

        layout.addWidget(self.label_signal,     3, 0, 1, 2)
        layout.addWidget(self.btn_signal_on,    4, 0, 1, 1)
        layout.addWidget(self.btn_signal_off,   4, 1, 1, 1)
        
        layout.addWidget(self.label_power,      5, 0, 1, 2)
        layout.addWidget(self.btn_power_on,     6, 0, 1, 1)
        layout.addWidget(self.btn_power_off,    6, 1, 1, 1)
        
        layout.addWidget(self.label_mode,       7, 0, 1, 2)
        layout.addWidget(self.btn_calibration,  8, 0, 1, 2)
        layout.addWidget(self.btn_standing,     9, 0, 1, 2)
        layout.addWidget(self.btn_tripod,       10, 0, 1, 2)
        
        layout.addWidget(self.btn_stop_all,     11, 0, 1, 2)
        
        # Vertical line spans all rows (0 to 11)
        layout.addWidget(vline, 0, 2, 12, 1)
        
        # Right column - Data recording & Status
        layout.addWidget(self.label_output,     0, 3, 1, 2)
        layout.addWidget(self.edit_output,      1, 3, 1, 2)
        layout.addWidget(self.btn_trigger,      2, 3, 1, 2)
        layout.addWidget(self.btn_reset,        3, 3, 1, 2)
        
        layout.addWidget(self.label_status,     5, 3, 1, 2)
        layout.addWidget(self.label_voltage,    6, 3, 1, 2)
        layout.addWidget(self.label_current,    7, 3, 1, 2)
        layout.addWidget(self.label_mode_status,8, 3, 1, 2)

        ### General Setting ###
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_update)
        self.timer.start(100)
        
        self.setLayout(layout)
        self.setWindowTitle('Rinbo Control Panel')
        self.setStyleSheet('''background-color: dimgray; font-family: Ubuntu; font-size: 24px; padding: 5px''')
        self.setWindowFlag(Qt.WindowStaysOnTopHint)
        
        self.adjustSize()
        
        x = (QApplication.desktop().screenGeometry().width() - self.width())
        y = (QApplication.desktop().screenGeometry().height() - self.height())
        self.move(x // 3, y // 3)
        
        self.show()

    def init_ros(self):
        qos = QoSProfile(depth=10)
        
        # Publishers
        self.trigger_pub = self.node.create_publisher(Bool, '/trigger', qos)
        self.filename_pub = self.node.create_publisher(String, '/output_filename', qos)
        self.power_cmd_pub = self.node.create_publisher(PowerCmdStamped, '/power/command', qos)
        
        # Subscribers
        self.power_state_sub = self.node.create_subscription(
            PowerStateStamped, '/power/state', self.power_state_cb, qos)
        self.motor_state_sub = self.node.create_subscription(
            MotorStateStamped, '/motor/state', self.motor_state_cb, qos)
        
        # Process handles
        self.process_ros_bridge = None
        self.process_controller = None

    def ros_bridge_cmd(self):
        if self.btn_ros_bridge.isChecked():
            self.btn_ros_bridge.setText('Stop ROS Bridge')
            self.process_ros_bridge = subprocess.Popen(
                ['ros2', 'run', 'rinbo_ros_bridge', 'rinbo_ros_bridge']
            )
        else:
            self.btn_ros_bridge.setText('Run ROS Bridge')
            if self.process_ros_bridge:
                self.process_ros_bridge.send_signal(signal.SIGINT)
                self.process_ros_bridge = None

    def mode_switch_cmd(self):
        # Stop any running controller first
        if self.process_controller:
            self.process_controller.send_signal(signal.SIGINT)
            self.process_controller = None
        
        if self.btn_calibration.isChecked():
            self.label_mode_status.setText('Mode: Calibration')
            self.process_controller = subprocess.Popen(
                ['ros2', 'run', 'rinbo_fsm', 'rinbo_cali'] 
            )
        elif self.btn_standing.isChecked():
            self.label_mode_status.setText('Mode: Standing')
            self.process_controller = subprocess.Popen(
                ['ros2', 'run', 'rinbo_fsm', 'rinbo_standing']
            )
        elif self.btn_tripod.isChecked():
            self.label_mode_status.setText('Mode: Tripod Gait')
            self.process_controller = subprocess.Popen(
                ['ros2', 'run', 'rinbo_fsm', 'rinbo_tripod_rslip']
            )

    def publish_power_cmd(self):
        msg = PowerCmdStamped()
        
        msg.header.stamp = self.node.get_clock().now().to_msg()
        
        # 三個狀態獨立讀取
        msg.digital = self.btn_digital_on.isChecked()
        msg.signal = self.btn_signal_on.isChecked()
        msg.power = self.btn_power_on.isChecked()
        
        msg.clean = False
        msg.trigger = False
        self.power_cmd_pub.publish(msg)

    def publish_trigger_cmd(self):
        # Send filename first
        filename_msg = String()
        filename_msg.data = self.edit_output.text()
        self.filename_pub.publish(filename_msg)
        
        # Send trigger
        trigger_msg = Bool()
        trigger_msg.data = self.btn_trigger.isChecked()
        self.trigger_pub.publish(trigger_msg)
        
        if self.btn_trigger.isChecked():
            self.btn_trigger.setText('Stop Recording')
        else:
            self.btn_trigger.setText('Record')

    def stop_all(self):
        # Stop all controllers
        if self.process_controller:
            self.process_controller.send_signal(signal.SIGINT)
            self.process_controller = None
        
        # Uncheck mode buttons
        self.btn_group_mode.setExclusive(False)
        self.btn_calibration.setChecked(False)
        self.btn_standing.setChecked(False)
        self.btn_tripod.setChecked(False)
        self.btn_group_mode.setExclusive(True)
        
        self.label_mode_status.setText('Mode: STOPPED')

    def reset(self):
        self.stop_all()
        self.btn_digital_off.setChecked(True)
        self.btn_signal_off.setChecked(True)
        self.btn_power_off.setChecked(True)
        self.publish_power_cmd()
        
        self.btn_trigger.setChecked(False)
        self.publish_trigger_cmd()

    def power_state_cb(self, msg):
        self.label_voltage.setText(f'Voltage: {msg.v_0:.1f} V')
        self.label_current.setText(f'Current: {msg.i_0:.1f} A')

    def motor_state_cb(self, msg):
        pass  # Can add motor state display if needed

    def timer_update(self):
        # Spin ROS node
        rclpy.spin_once(self.node, timeout_sec=0)
        
        # Check if processes are still running
        if self.process_controller and self.process_controller.poll() is not None:
            self.btn_group_mode.setExclusive(False)
            self.btn_calibration.setChecked(False)
            self.btn_standing.setChecked(False)
            self.btn_tripod.setChecked(False)
            self.btn_group_mode.setExclusive(True)
            self.label_mode_status.setText('Mode: --')
            self.process_controller = None

    def closeEvent(self, event):
        self.reset()
        
        # Stop bridges gracefully
        if self.process_ros_bridge:
            self.process_ros_bridge.send_signal(signal.SIGINT)
            
        if self.process_controller:
            self.process_controller.send_signal(signal.SIGINT)
            
        super(RinboControlPanel, self).closeEvent(event)


def main():
    rclpy.init()
    node = Node('rinbo_control_panel')
    
    app = QApplication(sys.argv)
    window = RinboControlPanel(node)
    
    ret = app.exec_()
    
    node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(ret)


if __name__ == '__main__':
    main()