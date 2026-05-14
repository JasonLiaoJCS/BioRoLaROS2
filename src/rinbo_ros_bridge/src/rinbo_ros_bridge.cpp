#include <iostream>
#include <mutex>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "NodeHandler.h"
#include "Motor.pb.h"
#include "Power.pb.h"

#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include "rinbo_msgs/msg/motor_state_stamped.hpp"
#include "rinbo_msgs/msg/power_cmd_stamped.hpp"
#include "rinbo_msgs/msg/power_state_stamped.hpp"

std::mutex mutex_ros_motor_state;
std::mutex mutex_ros_power_state;
std::mutex mutex_grpc_motor_cmd;
std::mutex mutex_grpc_power_cmd;

rinbo_msgs::msg::MotorCmdStamped ros_motor_cmd;
rinbo_msgs::msg::MotorStateStamped ros_motor_state;
rinbo_msgs::msg::PowerCmdStamped ros_power_cmd;
rinbo_msgs::msg::PowerStateStamped ros_power_state;

motor_msg::MotorCmdStamped grpc_motor_cmd;
motor_msg::MotorStateStamped grpc_motor_state;
power_msg::PowerCmdStamped grpc_power_cmd;
power_msg::PowerStateStamped grpc_power_state;

core::Publisher<motor_msg::MotorCmdStamped>* grpc_motor_cmd_pub = nullptr;
core::Publisher<power_msg::PowerCmdStamped>* grpc_power_cmd_pub = nullptr;
rclcpp::Publisher<rinbo_msgs::msg::MotorStateStamped>::SharedPtr ros_motor_state_pub = nullptr;
rclcpp::Publisher<rinbo_msgs::msg::PowerStateStamped>::SharedPtr ros_power_state_pub = nullptr;

void ros_motor_cmd_cb(const rinbo_msgs::msg::MotorCmdStamped::SharedPtr cmd) {
    std::lock_guard<std::mutex> lock(mutex_grpc_motor_cmd);

    ros_motor_cmd = *cmd;
    std::vector<motor_msg::LegCmd*> grpc_legs = {
        grpc_motor_cmd.mutable_l1(),
        grpc_motor_cmd.mutable_l2(),
        grpc_motor_cmd.mutable_l3(),
        grpc_motor_cmd.mutable_r1(),
        grpc_motor_cmd.mutable_r2(),
        grpc_motor_cmd.mutable_r3()
    };

    std::vector<rinbo_msgs::msg::LegCmd> ros_legs = {
        ros_motor_cmd.l1,
        ros_motor_cmd.l2,
        ros_motor_cmd.l3,
        ros_motor_cmd.r1,
        ros_motor_cmd.r2,
        ros_motor_cmd.r3
    };

    for (int i = 0; i < 6; ++i) {
        const auto& src = ros_legs[i];
        auto* dst = grpc_legs[i];

        dst->set_enable(src.enable);
        dst->set_direction(src.direction);
        dst->set_voltage(src.voltage);
        dst->set_state(src.state);
        dst->set_reset_position(src.reset_position);
    }
    std::vector<motor_msg::ServoCmd*> grpc_servos = {
        grpc_motor_cmd.mutable_sl1(),
        grpc_motor_cmd.mutable_sl2(),
        grpc_motor_cmd.mutable_sl3(),
        grpc_motor_cmd.mutable_sr1(),
        grpc_motor_cmd.mutable_sr2(),
        grpc_motor_cmd.mutable_sr3()
    };

    std::vector<rinbo_msgs::msg::ServoCmd> ros_servos = {
        ros_motor_cmd.sl1,
        ros_motor_cmd.sl2,
        ros_motor_cmd.sl3,
        ros_motor_cmd.sr1,
        ros_motor_cmd.sr2,
        ros_motor_cmd.sr3
    };

    for (int i = 0; i < 6; ++i) {
        grpc_servos[i]->set_position_encoder(ros_servos[i].position_encoder);
    }

    grpc_motor_cmd.set_servo_control_mode(ros_motor_cmd.servo_control_mode);
    grpc_motor_cmd.mutable_header()->set_seq(ros_motor_cmd.header.seq);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_sec(ros_motor_cmd.header.stamp.sec);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_usec(ros_motor_cmd.header.stamp.nanosec / 1000);
    if (grpc_motor_cmd_pub != nullptr) {
        grpc_motor_cmd_pub->publish(grpc_motor_cmd);
    }
}

void ros_power_cmd_cb(const rinbo_msgs::msg::PowerCmdStamped::SharedPtr cmd) {
    std::lock_guard<std::mutex> lock(mutex_grpc_power_cmd);

    ros_power_cmd = *cmd;

    grpc_power_cmd.set_digital(ros_power_cmd.digital);
    grpc_power_cmd.set_signal(ros_power_cmd.signal);
    grpc_power_cmd.set_power(ros_power_cmd.power);
    grpc_power_cmd.mutable_header()->set_seq(ros_power_cmd.header.seq);
    grpc_power_cmd.mutable_header()->mutable_stamp()->set_sec(ros_power_cmd.header.stamp.sec);
    grpc_power_cmd.mutable_header()->mutable_stamp()->set_usec(ros_power_cmd.header.stamp.nanosec / 1000);

    if (grpc_power_cmd_pub != nullptr) {
        grpc_power_cmd_pub->publish(grpc_power_cmd);
    }
}

void grpc_motor_state_cb(motor_msg::MotorStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_motor_state);

    grpc_motor_state = state;
    std::vector<const motor_msg::LegState*> grpc_legs = {
        &grpc_motor_state.l1(),
        &grpc_motor_state.l2(),
        &grpc_motor_state.l3(),
        &grpc_motor_state.r1(),
        &grpc_motor_state.r2(),
        &grpc_motor_state.r3()
    };

    std::vector<rinbo_msgs::msg::LegState*> ros_legs = {
        &ros_motor_state.l1,
        &ros_motor_state.l2,
        &ros_motor_state.l3,
        &ros_motor_state.r1,
        &ros_motor_state.r2,
        &ros_motor_state.r3
    };

    for (int i = 0; i < 6; ++i) {
        const auto* src = grpc_legs[i];
        auto* dst = ros_legs[i];

        dst->position = src->position();
        dst->tick_count = src->tick_count();
        dst->hall_effect = src->hall_effect();
    }
    std::vector<const motor_msg::ServoState*> grpc_servos = {
        &grpc_motor_state.sl1(),
        &grpc_motor_state.sl2(),
        &grpc_motor_state.sl3(),
        &grpc_motor_state.sr1(),
        &grpc_motor_state.sr2(),
        &grpc_motor_state.sr3()
    };

    std::vector<rinbo_msgs::msg::ServoState*> ros_servos = {
        &ros_motor_state.sl1,
        &ros_motor_state.sl2,
        &ros_motor_state.sl3,
        &ros_motor_state.sr1,
        &ros_motor_state.sr2,
        &ros_motor_state.sr3
    };

    for (int i = 0; i < 6; ++i) {
        ros_servos[i]->position_encoder = grpc_servos[i]->position_encoder();
    }

    ros_motor_state.servo_control_mode = grpc_motor_state.servo_control_mode();
    ros_motor_state.header.seq = grpc_motor_state.header().seq();
    ros_motor_state.header.stamp.sec = grpc_motor_state.header().stamp().sec();
    ros_motor_state.header.stamp.nanosec = grpc_motor_state.header().stamp().usec() * 1000;

    if (ros_motor_state_pub) {
        ros_motor_state_pub->publish(ros_motor_state);
    }
}

void grpc_power_state_cb(power_msg::PowerStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_power_state);

    grpc_power_state = state;

    ros_power_state.digital = grpc_power_state.digital();
    ros_power_state.signal = grpc_power_state.signal();
    ros_power_state.power = grpc_power_state.power();
    ros_power_state.v_0 = grpc_power_state.v_0();
    ros_power_state.i_0 = grpc_power_state.i_0();
    ros_power_state.v_1 = grpc_power_state.v_1();
    ros_power_state.i_1 = grpc_power_state.i_1();
    ros_power_state.v_2 = grpc_power_state.v_2();
    ros_power_state.i_2 = grpc_power_state.i_2();
    ros_power_state.v_3 = grpc_power_state.v_3();
    ros_power_state.i_3 = grpc_power_state.i_3();
    ros_power_state.v_4 = grpc_power_state.v_4();
    ros_power_state.i_4 = grpc_power_state.i_4();
    ros_power_state.v_5 = grpc_power_state.v_5();
    ros_power_state.i_5 = grpc_power_state.i_5();
    ros_power_state.v_6 = grpc_power_state.v_6();
    ros_power_state.i_6 = grpc_power_state.i_6();
    ros_power_state.v_7 = grpc_power_state.v_7();
    ros_power_state.i_7 = grpc_power_state.i_7();
    ros_power_state.header.seq = grpc_power_state.header().seq();
    ros_power_state.header.stamp.sec = grpc_power_state.header().stamp().sec();
    ros_power_state.header.stamp.nanosec = grpc_power_state.header().stamp().usec() * 1000;

    if (ros_power_state_pub) {
        ros_power_state_pub->publish(ros_power_state);
    }
}

int main(int argc, char **argv) {
    setenv("CORE_IP", "192.168.30.12", 1); 
    RCLCPP_INFO(rclcpp::get_logger("rinbo_ros2_bridge"), "Rinbo ROS2 Bridge Starts\n");

    bool debug_mode = false;
    if (argc >= 2 && argv[1] != nullptr) {
        if (strcmp(argv[1], "log") == 0) {
            debug_mode = true;
        }
    }

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
    auto ros_motor_cmd_sub = node->create_subscription<rinbo_msgs::msg::MotorCmdStamped>("motor/command", 1, ros_motor_cmd_cb);
    auto ros_power_cmd_sub = node->create_subscription<rinbo_msgs::msg::PowerCmdStamped>("power/command", 1, ros_power_cmd_cb);
    ros_motor_state_pub = node->create_publisher<rinbo_msgs::msg::MotorStateStamped>("motor/state", 1);
    ros_power_state_pub = node->create_publisher<rinbo_msgs::msg::PowerStateStamped>("power/state", 1);

    core::NodeHandler nh_;
    core::Subscriber<motor_msg::MotorStateStamped> &grpc_motor_state_sub = nh_.subscribe<motor_msg::MotorStateStamped>("motor/state", 1000, grpc_motor_state_cb);
    core::Subscriber<power_msg::PowerStateStamped> &grpc_power_state_sub = nh_.subscribe<power_msg::PowerStateStamped>("power/state", 1000, grpc_power_state_cb);
    grpc_motor_cmd_pub = &(nh_.advertise<motor_msg::MotorCmdStamped>("motor/command"));
    grpc_power_cmd_pub = &(nh_.advertise<power_msg::PowerCmdStamped>("power/command"));

    rclcpp::WallRate rate(1000);

    int loop_counter = 0;
    while (rclcpp::ok()) {
        if (debug_mode) RCLCPP_INFO(rclcpp::get_logger("rinbo_ros2_bridge"), "Loop Count: %d", loop_counter);

        rclcpp::spin_some(node);
        core::spinOnce();

        if (debug_mode) RCLCPP_INFO(rclcpp::get_logger("rinbo_ros2_bridge"), " ");

        loop_counter++;
        rate.sleep();
    }

    RCLCPP_INFO(rclcpp::get_logger("rinbo_ros2_bridge"), "Rinbo ROS2 Bridge is killed");

    rclcpp::shutdown();
    
    return 0;
}