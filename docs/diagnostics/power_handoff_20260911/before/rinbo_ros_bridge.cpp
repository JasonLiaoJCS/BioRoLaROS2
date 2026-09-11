#include <iostream>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <limits>
#include <iomanip>
#include <mutex>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/message_info.hpp"
#include "rmw/types.h"

#include "NodeHandler.h"
#include "Motor.pb.h"
#include "Power.pb.h"
#include "power_command_epoch_guard.hpp"
#include "motor_output_limits.hpp"

#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include "rinbo_msgs/msg/motor_state_stamped.hpp"
#include "rinbo_msgs/msg/header.hpp"
#include "rinbo_msgs/msg/power_cmd_stamped.hpp"
#include "rinbo_msgs/msg/power_state_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

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
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_output_enabled_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_arbiter_ready_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr motor_arbiter_heartbeat_pub = nullptr;
rclcpp::Publisher<rinbo_msgs::msg::Header>::SharedPtr motor_arbiter_epoch_pub = nullptr;
rclcpp::Publisher<rinbo_msgs::msg::Header>::SharedPtr motor_rearm_ack_pub = nullptr;
rclcpp::Publisher<rinbo_msgs::msg::Header>::SharedPtr motor_active_ack_pub = nullptr;
rclcpp::Node::SharedPtr bridge_node = nullptr;
rclcpp::Publisher<rinbo_msgs::msg::MotorCmdStamped>::SharedPtr motor_requested_monitor_pub;
rclcpp::Publisher<rinbo_msgs::msg::MotorCmdStamped>::SharedPtr motor_forwarded_monitor_pub;

// General diagnostics use these mirrors. The motion handshake accepts only
// this Bridge plus the audited passive /rinbo_data_recorder on /motor/command.
// Mirrors are lossy and cannot throw into the control path or affect rearm/ACK.
void publish_command_monitor(
    const rclcpp::Publisher<rinbo_msgs::msg::MotorCmdStamped>::SharedPtr& publisher,
    const rinbo_msgs::msg::MotorCmdStamped& command) noexcept {
    try {
        if (publisher && publisher->get_subscription_count() > 0) publisher->publish(command);
    } catch (...) {
        // Losing a diagnostic sample must not interrupt a motor/stop command.
    }
}

std::chrono::steady_clock::time_point last_ros_motor_cmd_time;
bool ros_motor_cmd_active = false;
bool motor_safety_latched = false;
std::string motor_safety_latch_reason;
int motor_command_timeout_ms = 100;
int motor_command_max_age_ms = 100;
int motor_command_rearm_disabled_samples = 5;
int motor_disable_resend_period_ms = 20;
int motor_arbiter_heartbeat_period_ms = 50;
int motor_output_status_period_ms = 20;
int motor_shutdown_disabled_packets = 8;
int power_shutdown_off_packets = 3;
int power_command_max_age_ms = 200;
double motor_command_max_pwm = rinbo_bridge::kMaxRawMotorCommand;
uint32_t motor_command_min_servo_encoder = 0;
uint32_t motor_command_max_servo_encoder = 65535;
int motor_disabled_rearm_count = 0;
std::chrono::steady_clock::time_point last_disabled_rearm_time;
bool disabled_rearm_time_valid = false;
bool source_header_seen = false;
uint32_t last_source_sequence = 0;
int64_t last_source_stamp_ns = 0;
using PublisherGid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;
PublisherGid last_motor_command_publisher_gid {};
bool motor_command_publisher_seen = false;
uint32_t grpc_motor_output_sequence = 0;
uint32_t grpc_power_output_sequence = 0;
std::array<uint32_t, 6> last_servo_targets {};
bool last_servo_targets_valid = false;
std::chrono::steady_clock::time_point last_disable_publish_time;
bool disable_publish_time_valid = false;
std::chrono::steady_clock::time_point last_arbiter_ready_publish_time;
bool arbiter_ready_publish_time_valid = false;
std::chrono::steady_clock::time_point last_arbiter_heartbeat_publish_time;
bool arbiter_heartbeat_publish_time_valid = false;
std::string bridge_boot_id;
uint32_t motor_latch_generation = 0;
std::atomic<bool> software_estop_asserted{false};
rinbo_ros_bridge::PowerCommandEpochGuard power_command_epoch_guard;
std::chrono::steady_clock::time_point last_motor_output_status_time;
bool motor_output_status_time_valid = false;

constexpr const char * kMotorCommandTopic = "/motor/command";
constexpr const char * kMotorOutputEnabledTopic = "/rinbo/motor_output_enabled";
constexpr const char * kMotorArbiterReadyTopic = "/rinbo/motor_arbiter_ready";
constexpr const char * kMotorArbiterHeartbeatTopic = "/rinbo/motor_arbiter_heartbeat";
constexpr const char * kMotorArbiterEpochTopic = "/rinbo/motor_arbiter_epoch";
constexpr const char * kMotorRearmAckTopic = "/rinbo/motor_rearm_ack";
constexpr const char * kMotorActiveAckTopic = "/rinbo/motor_active_ack";
constexpr const char * kPowerCommandTopic = "/power/command";
constexpr const char * kEstopTopic = "/estop";
constexpr double kHardMotorCommandMaxPwm = rinbo_bridge::kMaxRawMotorCommand;
constexpr int kHardMotorCommandTimeoutMaxMs = 100;
constexpr int kHardMotorCommandMaxAgeMaxMs = 100;
constexpr int kHardMotorCommandRearmMinSamples = 5;
constexpr int kHardMotorDisableResendMaxMs = 20;
constexpr int kHardMotorArbiterHeartbeatMaxMs = 50;
constexpr int kHardMotorOutputStatusMaxMs = 20;
constexpr int kHardMotorShutdownMinPackets = 8;
constexpr int kHardPowerShutdownMinPackets = 3;
constexpr int kHardPowerCommandMaxAgeMaxMs = 200;
constexpr uint32_t kHardServoEncoderMax = 65535U;

volatile std::sig_atomic_t bridge_shutdown_requested = 0;

extern "C" void bridge_signal_handler(int) noexcept {
    bridge_shutdown_requested = 1;
}

std::string make_bridge_boot_id() {
    const auto wall_nonce = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const auto steady_nonce = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    std::random_device entropy;
    std::ostringstream out;
    out << "rinbo_ros2_bridge/boot-" << std::hex
        << static_cast<uint64_t>(wall_nonce) << "-"
        << static_cast<uint64_t>(steady_nonce);
    for (int index = 0; index < 4; ++index) {
        out << "-" << std::setw(8) << std::setfill('0')
            << static_cast<uint32_t>(entropy());
    }
    return out.str();
}

std::string resolve_core_ip_default() {
    const char *core_ip_env = std::getenv("CORE_IP");
    if (core_ip_env != nullptr && core_ip_env[0] != '\0') {
        return core_ip_env;
    }

    // The existing R-Slip launcher/manual exports CORE_MASTER_ADDR with the
    // dynamically discovered sbRIO address. Reuse that host when CORE_IP was
    // not exported so lab-DHCP deployments cannot silently fall back to an
    // unrelated checked-in address.
    const char *master_addr_env = std::getenv("CORE_MASTER_ADDR");
    if (master_addr_env != nullptr && master_addr_env[0] != '\0') {
        const std::string master_addr(master_addr_env);
        if (master_addr.front() == '[') {
            const auto closing_bracket = master_addr.find(']');
            if (closing_bracket != std::string::npos && closing_bracket > 1U) {
                return master_addr.substr(1U, closing_bracket - 1U);
            }
        }
        const auto first_colon = master_addr.find(':');
        const auto last_colon = master_addr.rfind(':');
        if (last_colon != std::string::npos && first_colon == last_colon &&
            last_colon > 0U) {
            return master_addr.substr(0U, last_colon);
        }
        if (first_colon == std::string::npos) {
            return master_addr;
        }
    }
    return "192.168.30.2";
}

struct PublisherGuardResult {
    bool valid = false;
    PublisherGid gid {};
    std::string reason;
};

bool motor_cmd_has_active_output(const rinbo_msgs::msg::MotorCmdStamped &cmd) {
    return cmd.servo_control_mode != 0 || cmd.l1.enable || cmd.l2.enable ||
           cmd.l3.enable || cmd.r1.enable || cmd.r2.enable || cmd.r3.enable;
}

bool frame_id_has_suffix(const std::string &frame_id, const char *suffix) {
    const std::size_t suffix_size = std::strlen(suffix);
    return frame_id.size() >= suffix_size &&
        frame_id.compare(frame_id.size() - suffix_size, suffix_size, suffix) == 0;
}

bool motor_cmd_requests_rearm_ack(const rinbo_msgs::msg::MotorCmdStamped &cmd) {
    return frame_id_has_suffix(cmd.header.frame_id, "/rearm");
}

bool motor_cmd_requests_active_ack(const rinbo_msgs::msg::MotorCmdStamped &cmd) {
    return frame_id_has_suffix(cmd.header.frame_id, "/active-probe");
}

bool sequence_is_newer(uint32_t sequence, uint32_t previous) {
    const uint32_t delta = sequence - previous;
    return delta != 0U && delta < 0x80000000U;
}

PublisherGuardResult inspect_motor_command_publishers() {
    PublisherGuardResult result;
    if (bridge_node == nullptr) {
        result.reason = "bridge node is unavailable for publisher arbitration";
        return result;
    }
    try {
        const auto infos = bridge_node->get_publishers_info_by_topic(kMotorCommandTopic);
        if (infos.size() != 1U) {
            std::ostringstream out;
            out << "expected exactly one publisher on " << kMotorCommandTopic
                << ", got " << infos.size();
            if (!infos.empty()) {
                out << ": ";
                for (std::size_t index = 0; index < infos.size(); ++index) {
                    if (index != 0U) out << ",";
                    out << infos[index].node_namespace() << "/" << infos[index].node_name();
                }
            }
            result.reason = out.str();
            return result;
        }
        result.valid = true;
        result.gid = infos.front().endpoint_gid();
        return result;
    } catch (const std::exception &exc) {
        result.reason = std::string("publisher graph query failed: ") + exc.what();
        return result;
    }
}

rinbo_ros_bridge::PowerCommandEpochGuard::PublisherGraph
inspect_power_command_publishers() {
    rinbo_ros_bridge::PowerCommandEpochGuard::PublisherGraph result;
    if (bridge_node == nullptr) {
        result.query_succeeded = false;
        result.query_error =
            "bridge node is unavailable for power publisher arbitration";
        return result;
    }
    try {
        const auto infos = bridge_node->get_publishers_info_by_topic(kPowerCommandTopic);
        result.publisher_count = infos.size();
        if (infos.size() == 1U) {
            const auto &gid = infos.front().endpoint_gid();
            result.sole_publisher_gid.assign(gid.begin(), gid.end());
            result.sole_node_name = infos.front().node_name();
            result.sole_node_namespace = infos.front().node_namespace();
        }
        return result;
    } catch (const std::exception &exc) {
        result.query_succeeded = false;
        result.query_error =
            std::string("power publisher graph query failed: ") + exc.what();
        return result;
    }
}

std::optional<std::string> validate_motor_command_payload(
    const rinbo_msgs::msg::MotorCmdStamped &cmd) {
    if (cmd.servo_control_mode != 0U && cmd.servo_control_mode != 2U) {
        return "servo_control_mode must be 0 (disabled) or 2 (position control)";
    }
    const std::array<const rinbo_msgs::msg::LegCmd *, 6> legs = {
        &cmd.l1, &cmd.l2, &cmd.l3, &cmd.r1, &cmd.r2, &cmd.r3};
    static constexpr std::array<const char *, 6> leg_names = {
        "L1", "L2", "L3", "R1", "R2", "R3"};
    for (std::size_t index = 0; index < legs.size(); ++index) {
        const double voltage = static_cast<double>(legs[index]->voltage);
        if (const auto reason = rinbo_bridge::validate_motor_output(
                voltage, motor_command_max_pwm, leg_names[index])) return reason;
    }

    // Disabled packets intentionally carry no actionable servo target.  Do
    // not let configured position bounds make the safety rearm impossible.
    if (cmd.servo_control_mode != 0U) {
        const std::array<uint32_t, 6> servo_targets = {
            cmd.sl1.position_encoder, cmd.sl2.position_encoder, cmd.sl3.position_encoder,
            cmd.sr1.position_encoder, cmd.sr2.position_encoder, cmd.sr3.position_encoder};
        static constexpr std::array<const char *, 6> servo_names = {
            "SL1", "SL2", "SL3", "SR1", "SR2", "SR3"};
        for (std::size_t index = 0; index < servo_targets.size(); ++index) {
            if (servo_targets[index] < motor_command_min_servo_encoder ||
                servo_targets[index] > motor_command_max_servo_encoder) {
                std::ostringstream out;
                out << "servo target for " << servo_names[index] << " is "
                    << servo_targets[index] << ", outside ["
                    << motor_command_min_servo_encoder << ","
                    << motor_command_max_servo_encoder << "]";
                return out.str();
            }
        }
    }
    return std::nullopt;
}

std::optional<std::string> validate_motor_command_header_locked(
    const rinbo_msgs::msg::MotorCmdStamped &cmd) {
    if (cmd.header.stamp.nanosec >= 1000000000U) {
        return "motor command stamp.nanosec is outside [0,1e9)";
    }
    const int64_t stamp_ns =
        static_cast<int64_t>(cmd.header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(cmd.header.stamp.nanosec);
    if (stamp_ns <= 0) {
        return "motor command stamp must be nonzero and positive";
    }
    const int64_t now_ns = bridge_node->now().nanoseconds();
    const int64_t max_age_ns = static_cast<int64_t>(motor_command_max_age_ms) * 1000000LL;
    const int64_t age_ns = now_ns - stamp_ns;
    if (age_ns > max_age_ns || age_ns < -max_age_ns) {
        std::ostringstream out;
        out << "motor command stamp age " << static_cast<double>(age_ns) * 1.0e-9
            << "s exceeds +/-" << static_cast<double>(motor_command_max_age_ms) * 1.0e-3
            << "s";
        return out.str();
    }
    if (source_header_seen) {
        if (stamp_ns <= last_source_stamp_ns) {
            return "motor command stamp is duplicate or non-monotonic";
        }
        if (!sequence_is_newer(cmd.header.seq, last_source_sequence)) {
            return "motor command sequence is duplicate or out-of-order";
        }
    }
    return std::nullopt;
}

void publish_motor_output_enabled(bool enabled) {
    if (motor_output_enabled_pub == nullptr) return;
    std_msgs::msg::Bool status;
    status.data = enabled;
    motor_output_enabled_pub->publish(status);
}

void publish_motor_arbiter_ready(bool ready) {
    if (motor_arbiter_ready_pub == nullptr) return;
    std_msgs::msg::Bool status;
    status.data = ready;
    motor_arbiter_ready_pub->publish(status);
    last_arbiter_ready_publish_time = std::chrono::steady_clock::now();
    arbiter_ready_publish_time_valid = true;
}

void publish_motor_arbiter_heartbeat() {
    if (motor_arbiter_heartbeat_pub == nullptr) return;
    motor_arbiter_heartbeat_pub->publish(std_msgs::msg::Empty());
    last_arbiter_heartbeat_publish_time = std::chrono::steady_clock::now();
    arbiter_heartbeat_publish_time_valid = true;
}

void publish_motor_arbiter_epoch() {
    if (motor_arbiter_epoch_pub == nullptr || bridge_node == nullptr) return;
    rinbo_msgs::msg::Header status;
    status.frame_id = bridge_boot_id;
    status.seq = motor_latch_generation;
    const int64_t stamp_ns = bridge_node->now().nanoseconds();
    status.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    status.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    motor_arbiter_epoch_pub->publish(status);
}

void publish_motor_ack(
    const rclcpp::Publisher<rinbo_msgs::msg::Header>::SharedPtr &publisher,
    const rinbo_msgs::msg::Header &header) {
    if (publisher == nullptr) return;
    auto acknowledgement = header;
    acknowledgement.frame_id += "|bridge=" + bridge_boot_id +
        "|latch=" + std::to_string(motor_latch_generation);
    publisher->publish(acknowledgement);
}

void latch_motor_safety_locked(const std::string &reason, bool reset_source_header = true) {
    const bool newly_latched = !motor_safety_latched;
    const bool reason_changed = reason != motor_safety_latch_reason;
    motor_safety_latched = true;
    if (newly_latched) ++motor_latch_generation;
    motor_safety_latch_reason = reason;
    motor_disabled_rearm_count = 0;
    disabled_rearm_time_valid = false;
    ros_motor_cmd_active = false;
    publish_motor_output_enabled(false);
    publish_motor_arbiter_ready(false);
    publish_motor_arbiter_epoch();
    if (reset_source_header) source_header_seen = false;
    if (newly_latched) disable_publish_time_valid = false;
    if ((newly_latched || reason_changed) && bridge_node != nullptr) {
        RCLCPP_ERROR(bridge_node->get_logger(), "MOTOR SAFETY LATCHED: %s", reason.c_str());
    }
}

void capture_servo_targets_locked(const rinbo_msgs::msg::MotorCmdStamped &cmd) {
    last_servo_targets = {
        cmd.sl1.position_encoder, cmd.sl2.position_encoder, cmd.sl3.position_encoder,
        cmd.sr1.position_encoder, cmd.sr2.position_encoder, cmd.sr3.position_encoder};
    last_servo_targets_valid = true;
}

rinbo_msgs::msg::MotorCmdStamped make_all_disabled_command_locked() {
    rinbo_msgs::msg::MotorCmdStamped stop;
    stop.header.frame_id = "rinbo_ros_bridge_safety_stop";
    stop.servo_control_mode = 0;
    if (last_servo_targets_valid) {
        stop.sl1.position_encoder = last_servo_targets[0];
        stop.sl2.position_encoder = last_servo_targets[1];
        stop.sl3.position_encoder = last_servo_targets[2];
        stop.sr1.position_encoder = last_servo_targets[3];
        stop.sr2.position_encoder = last_servo_targets[4];
        stop.sr3.position_encoder = last_servo_targets[5];
    }
    return stop;
}

void publish_motor_command_to_grpc_locked(
    const rinbo_msgs::msg::MotorCmdStamped &command) {
    grpc_motor_cmd.Clear();
    const std::array<motor_msg::LegCmd *, 6> grpc_legs = {
        grpc_motor_cmd.mutable_l1(), grpc_motor_cmd.mutable_l2(),
        grpc_motor_cmd.mutable_l3(), grpc_motor_cmd.mutable_r1(),
        grpc_motor_cmd.mutable_r2(), grpc_motor_cmd.mutable_r3()};
    const std::array<const rinbo_msgs::msg::LegCmd *, 6> ros_legs = {
        &command.l1, &command.l2, &command.l3, &command.r1, &command.r2, &command.r3};
    for (std::size_t index = 0; index < grpc_legs.size(); ++index) {
        grpc_legs[index]->set_enable(ros_legs[index]->enable);
        grpc_legs[index]->set_direction(ros_legs[index]->direction);
        grpc_legs[index]->set_voltage(ros_legs[index]->voltage);
        grpc_legs[index]->set_state(ros_legs[index]->state);
        grpc_legs[index]->set_reset_position(ros_legs[index]->reset_position);
    }

    const std::array<motor_msg::ServoCmd *, 6> grpc_servos = {
        grpc_motor_cmd.mutable_sl1(), grpc_motor_cmd.mutable_sl2(),
        grpc_motor_cmd.mutable_sl3(), grpc_motor_cmd.mutable_sr1(),
        grpc_motor_cmd.mutable_sr2(), grpc_motor_cmd.mutable_sr3()};
    const std::array<const rinbo_msgs::msg::ServoCmd *, 6> ros_servos = {
        &command.sl1, &command.sl2, &command.sl3,
        &command.sr1, &command.sr2, &command.sr3};
    for (std::size_t index = 0; index < grpc_servos.size(); ++index) {
        grpc_servos[index]->set_position_encoder(ros_servos[index]->position_encoder);
    }

    grpc_motor_cmd.set_servo_control_mode(command.servo_control_mode);
    const auto output_stamp = bridge_node->now();
    const int64_t output_stamp_ns = output_stamp.nanoseconds();
    grpc_motor_cmd.mutable_header()->set_seq(++grpc_motor_output_sequence);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_sec(
        output_stamp_ns / 1000000000LL);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_usec(
        (output_stamp_ns % 1000000000LL) / 1000LL);
    if (grpc_motor_cmd_pub != nullptr) {
        grpc_motor_cmd_pub->publish(grpc_motor_cmd);
        // This is a transport handoff, NOT an acknowledgement from the sbRIO.
        // Preserve the source header to correlate with the requested mirror.
        publish_command_monitor(motor_forwarded_monitor_pub, command);
    }
}

void publish_all_disabled_locked(const std::chrono::steady_clock::time_point &now) {
    const auto stop = make_all_disabled_command_locked();
    publish_motor_command_to_grpc_locked(stop);
    ros_motor_cmd = stop;
    ros_motor_cmd_active = false;
    publish_motor_output_enabled(false);
    last_disable_publish_time = now;
    disable_publish_time_valid = true;
}

void ros_motor_cmd_cb(
    const rinbo_msgs::msg::MotorCmdStamped::ConstSharedPtr cmd,
    const rclcpp::MessageInfo &message_info) {
    publish_command_monitor(motor_requested_monitor_pub, *cmd);
    const auto publisher_guard = inspect_motor_command_publishers();
    const auto receive_time = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_grpc_motor_cmd);

    if (software_estop_asserted.load()) {
        latch_motor_safety_locked("software /estop asserted");
        publish_all_disabled_locked(receive_time);
        return;
    }

    if (!publisher_guard.valid) {
        latch_motor_safety_locked(publisher_guard.reason);
        return;
    }

    const auto &received_gid = message_info.get_rmw_message_info().publisher_gid;
    if (std::memcmp(
            received_gid.data, publisher_guard.gid.data(), RMW_GID_STORAGE_SIZE) != 0) {
        latch_motor_safety_locked(
            "received /motor/command source does not match the sole graph publisher");
        return;
    }

    const bool publisher_changed =
        motor_command_publisher_seen && publisher_guard.gid != last_motor_command_publisher_gid;
    if (publisher_changed) {
        latch_motor_safety_locked(
            "the sole /motor/command publisher changed; disabled-command rearm is required");
    }
    last_motor_command_publisher_gid = publisher_guard.gid;
    motor_command_publisher_seen = true;

    if (const auto reason = validate_motor_command_payload(*cmd)) {
        latch_motor_safety_locked(*reason);
        return;
    }
    if (const auto reason = validate_motor_command_header_locked(*cmd)) {
        latch_motor_safety_locked(*reason);
        return;
    }

    const int64_t stamp_ns =
        static_cast<int64_t>(cmd->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(cmd->header.stamp.nanosec);
    source_header_seen = true;
    last_source_sequence = cmd->header.seq;
    last_source_stamp_ns = stamp_ns;

    const bool active = motor_cmd_has_active_output(*cmd);
    if (motor_safety_latched) {
        if (active) {
            motor_disabled_rearm_count = 0;
            disabled_rearm_time_valid = false;
            RCLCPP_WARN_THROTTLE(
                bridge_node->get_logger(), *bridge_node->get_clock(), 1000,
                "Rejecting active motor command while safety latch is set: %s",
                motor_safety_latch_reason.c_str());
            return;
        }

        if (!disabled_rearm_time_valid ||
            receive_time - last_disabled_rearm_time >
                std::chrono::milliseconds(motor_command_timeout_ms)) {
            motor_disabled_rearm_count = 0;
        }
        ++motor_disabled_rearm_count;
        last_disabled_rearm_time = receive_time;
        disabled_rearm_time_valid = true;
        publish_all_disabled_locked(receive_time);
        publish_motor_arbiter_ready(false);
        RCLCPP_INFO(
            bridge_node->get_logger(), "Motor safety rearm disabled sample %d/%d",
            motor_disabled_rearm_count, motor_command_rearm_disabled_samples);
        if (motor_disabled_rearm_count >= motor_command_rearm_disabled_samples) {
            motor_safety_latched = false;
            motor_safety_latch_reason.clear();
            motor_disabled_rearm_count = 0;
            disabled_rearm_time_valid = false;
            publish_motor_arbiter_ready(true);
            if (motor_cmd_requests_rearm_ack(*cmd)) {
                publish_motor_ack(motor_rearm_ack_pub, cmd->header);
            }
            RCLCPP_WARN(
                bridge_node->get_logger(),
                "Motor safety latch cleared after consecutive fresh all-disabled commands");
        }
        return;
    }

    if (active) {
        capture_servo_targets_locked(*cmd);
        ros_motor_cmd = *cmd;
        publish_motor_command_to_grpc_locked(*cmd);
        ros_motor_cmd_active = true;
        last_ros_motor_cmd_time = receive_time;
        publish_motor_output_enabled(true);
        if (motor_cmd_requests_active_ack(*cmd)) {
            publish_motor_ack(motor_active_ack_pub, cmd->header);
        }
    } else {
        publish_all_disabled_locked(receive_time);
        // Marked probes are acknowledged for as long as the FSM keeps sending
        // them; discovery delay can therefore never exhaust a retry counter.
        if (motor_cmd_requests_rearm_ack(*cmd)) {
            publish_motor_ack(motor_rearm_ack_pub, cmd->header);
        }
        if (!arbiter_ready_publish_time_valid ||
            receive_time - last_arbiter_ready_publish_time >=
                std::chrono::milliseconds(motor_disable_resend_period_ms)) {
            publish_motor_arbiter_ready(true);
        }
    }
}

bool publish_shutdown_stop_locked() noexcept {
    bool all_stop_publishes_succeeded = true;
    const bool newly_latched = !motor_safety_latched;
    motor_safety_latched = true;
    if (newly_latched) ++motor_latch_generation;
    motor_safety_latch_reason = "bridge shutdown";
    motor_disabled_rearm_count = 0;
    disabled_rearm_time_valid = false;
    source_header_seen = false;
    ros_motor_cmd_active = false;

    const auto stop = make_all_disabled_command_locked();
    for (int attempt = 0; attempt < motor_shutdown_disabled_packets; ++attempt) {
        try {
            publish_motor_command_to_grpc_locked(stop);
        } catch (const std::exception &exc) {
            all_stop_publishes_succeeded = false;
            if (bridge_node != nullptr) {
                RCLCPP_ERROR(
                    bridge_node->get_logger(),
                    "Failed to send shutdown all-disabled command: %s", exc.what());
            }
        } catch (...) {
            all_stop_publishes_succeeded = false;
            if (bridge_node != nullptr) {
                RCLCPP_ERROR(
                    bridge_node->get_logger(),
                "Failed to send shutdown all-disabled command: unknown exception");
            }
        }
        if (attempt + 1 < motor_shutdown_disabled_packets) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(motor_disable_resend_period_ms));
        }
    }
    ros_motor_cmd = stop;

    // These ROS status publications are best effort during context shutdown;
    // the FSM also faults if the independent arbiter heartbeat disappears.
    try {
        publish_motor_output_enabled(false);
        publish_motor_arbiter_ready(false);
        publish_motor_arbiter_epoch();
    } catch (const std::exception &exc) {
        all_stop_publishes_succeeded = false;
        if (bridge_node != nullptr) {
            RCLCPP_ERROR(
                bridge_node->get_logger(),
                "Failed to publish shutdown motor status: %s", exc.what());
        }
    } catch (...) {
        all_stop_publishes_succeeded = false;
        if (bridge_node != nullptr) {
            RCLCPP_ERROR(
                bridge_node->get_logger(),
                "Failed to publish shutdown motor status: unknown exception");
        }
    }
    return all_stop_publishes_succeeded;
}

void publish_power_command_to_grpc_locked(
    const rinbo_msgs::msg::PowerCmdStamped &command) {
    grpc_power_cmd.Clear();
    grpc_power_cmd.set_digital(command.digital);
    grpc_power_cmd.set_signal(command.signal);
    grpc_power_cmd.set_power(command.power);
    grpc_power_cmd.set_clean(command.clean);
    grpc_power_cmd.set_trigger(command.trigger);
    const int64_t output_stamp_ns = bridge_node->now().nanoseconds();
    grpc_power_cmd.mutable_header()->set_seq(++grpc_power_output_sequence);
    grpc_power_cmd.mutable_header()->mutable_stamp()->set_sec(
        output_stamp_ns / 1000000000LL);
    grpc_power_cmd.mutable_header()->mutable_stamp()->set_usec(
        (output_stamp_ns % 1000000000LL) / 1000LL);
    if (grpc_power_cmd_pub != nullptr) grpc_power_cmd_pub->publish(grpc_power_cmd);
}

void publish_all_power_off_locked() {
    rinbo_msgs::msg::PowerCmdStamped stop;
    stop.power = false;
    stop.digital = false;
    stop.signal = false;
    stop.clean = false;
    stop.trigger = false;
    ros_power_cmd = stop;
    publish_power_command_to_grpc_locked(stop);
}

void reject_power_command_locked(const std::string &reason) {
    if (bridge_node != nullptr) {
        RCLCPP_ERROR(
            bridge_node->get_logger(), "POWER COMMAND REJECTED: %s", reason.c_str());
    }
    // This fail-safe output deliberately does not reset the input source
    // epoch. Only a received true all-off command may authorize that reset;
    // otherwise a rejected publisher could manufacture its own takeover.
    publish_all_power_off_locked();
}

void ros_power_cmd_cb(
    const rinbo_msgs::msg::PowerCmdStamped::ConstSharedPtr cmd,
    const rclcpp::MessageInfo &message_info) {
    const auto &rmw_received_gid =
        message_info.get_rmw_message_info().publisher_gid;
    const rinbo_ros_bridge::PowerCommandEpochGuard::PublisherGid received_gid(
        rmw_received_gid.data,
        rmw_received_gid.data + RMW_GID_STORAGE_SIZE);

    using PowerGuard = rinbo_ros_bridge::PowerCommandEpochGuard;
    const auto classification = PowerGuard::classify(PowerGuard::Payload {
        cmd->digital,
        cmd->signal,
        cmd->power,
        cmd->clean,
        cmd->trigger});

    // Only a true all-off bypasses graph/source/header/E-stop checks and erases
    // the source epoch. Relay-off commands that keep a rail energized are
    // authenticated sequence actions handled by the fail-closed path below.
    if (classification.kind == PowerGuard::CommandKind::kAllOff) {
        std::lock_guard<std::mutex> lock(mutex_grpc_power_cmd);
        auto safe_all_off = *cmd;
        safe_all_off.digital = classification.forward_payload.digital;
        safe_all_off.signal = classification.forward_payload.signal;
        safe_all_off.power = classification.forward_payload.power;
        safe_all_off.clean = classification.forward_payload.clean;
        safe_all_off.trigger = classification.forward_payload.trigger;
        ros_power_cmd = safe_all_off;
        publish_power_command_to_grpc_locked(safe_all_off);
        power_command_epoch_guard.observe_all_off();
        return;
    }

    const auto publisher_guard = inspect_power_command_publishers();
    std::lock_guard<std::mutex> lock(mutex_grpc_power_cmd);
    if (software_estop_asserted.load()) {
        reject_power_command_locked("software /estop is asserted");
        return;
    }
    if (classification.kind == PowerGuard::CommandKind::kInvalid) {
        reject_power_command_locked(
            classification.rejection_reason.value_or(
                "invalid power command payload"));
        return;
    }
    const PowerGuard::Header header {
        cmd->header.seq,
        static_cast<int32_t>(cmd->header.stamp.sec),
        cmd->header.stamp.nanosec};
    // Energizing commands must remain fresh at the serialized commit point.
    // DDS graph discovery and mutex contention both consume the bounded command
    // age; unlike passive state observation, neither may be hidden from the
    // actuation gate.
    const int64_t observed_ns =
        bridge_node == nullptr ? 0 : bridge_node->now().nanoseconds();
    if (const auto reason = power_command_epoch_guard.accept_energizing(
            classification.kind,
            publisher_guard,
            received_gid,
            header,
            observed_ns,
            static_cast<int64_t>(power_command_max_age_ms) * 1000000LL)) {
        reject_power_command_locked(*reason);
        return;
    }

    ros_power_cmd = *cmd;
    publish_power_command_to_grpc_locked(*cmd);
}

void estop_cb(const std_msgs::msg::Bool::SharedPtr msg) {
    // Treat E-stop assertion as process-lifetime sticky.  A plain Bool topic
    // has no reset authority, so accepting an arbitrary/stale false sample
    // would let another publisher silently clear the power-on gate.  Reset by
    // restarting the bridge after the physical hazard has been cleared.
    if (!msg->data) {
        if (software_estop_asserted.load() && bridge_node != nullptr) {
            RCLCPP_WARN_THROTTLE(
                bridge_node->get_logger(), *bridge_node->get_clock(), 5000,
                "Ignoring /estop=false: E-stop is latched until bridge restart");
        }
        return;
    }
    software_estop_asserted.store(true);
    {
        std::lock_guard<std::mutex> lock(mutex_grpc_motor_cmd);
        latch_motor_safety_locked("software /estop asserted");
        publish_all_disabled_locked(std::chrono::steady_clock::now());
    }
    {
        std::lock_guard<std::mutex> lock(mutex_grpc_power_cmd);
        publish_all_power_off_locked();
    }
}

void grpc_motor_state_cb(motor_msg::MotorStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_motor_state);

    // core::Subscriber::spinOnce() replays msgs_queue.back() until a newer TCP
    // packet arrives.  Do not turn that local replay into a fresh ROS sample:
    // downstream guards use the device sequence to detect replay, and their
    // arrival-time watchdog must expire if the sbRIO stream actually stalls.
    static bool source_header_seen = false;
    static int32_t last_source_sequence = 0;
    static int64_t last_source_stamp_sec = 0;
    static int32_t last_source_stamp_usec = 0;
    const auto &source_header = state.header();
    const bool exact_replay =
        source_header_seen &&
        source_header.seq() == last_source_sequence &&
        source_header.stamp().sec() == last_source_stamp_sec &&
        source_header.stamp().usec() == last_source_stamp_usec;
    if (exact_replay) return;
    source_header_seen = true;
    last_source_sequence = source_header.seq();
    last_source_stamp_sec = source_header.stamp().sec();
    last_source_stamp_usec = source_header.stamp().usec();

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

    {
        std::lock_guard<std::mutex> command_lock(mutex_grpc_motor_cmd);
        if (!last_servo_targets_valid) {
            for (std::size_t index = 0; index < last_servo_targets.size(); ++index) {
                last_servo_targets[index] = grpc_servos[index]->position_encoder();
            }
            last_servo_targets_valid = true;
        }
    }

    ros_motor_state.servo_control_mode = grpc_motor_state.servo_control_mode();
    ros_motor_state.header.seq = grpc_motor_state.header().seq();
    // sbRIO and Jetson do not have a guaranteed shared epoch.  The ROS source
    // is this bridge, so use Jetson receipt time for downstream age checks and
    // retain the sbRIO sequence to reject replay/out-of-order device packets.
    if (bridge_node != nullptr) {
        const int64_t receipt_ns = bridge_node->now().nanoseconds();
        ros_motor_state.header.stamp.sec =
            static_cast<int32_t>(receipt_ns / 1000000000LL);
        ros_motor_state.header.stamp.nanosec =
            static_cast<uint32_t>(receipt_ns % 1000000000LL);
    }

    if (ros_motor_state_pub) {
        ros_motor_state_pub->publish(ros_motor_state);
    }
}

void grpc_power_state_cb(power_msg::PowerStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_power_state);

    // See grpc_motor_state_cb(): only a new sbRIO packet may refresh the ROS
    // timestamp.  Exact core queue replays are intentionally suppressed.
    static bool source_header_seen = false;
    static int32_t last_source_sequence = 0;
    static int64_t last_source_stamp_sec = 0;
    static int32_t last_source_stamp_usec = 0;
    const auto &source_header = state.header();
    const bool exact_replay =
        source_header_seen &&
        source_header.seq() == last_source_sequence &&
        source_header.stamp().sec() == last_source_stamp_sec &&
        source_header.stamp().usec() == last_source_stamp_usec;
    if (exact_replay) return;
    source_header_seen = true;
    last_source_sequence = source_header.seq();
    last_source_stamp_sec = source_header.stamp().sec();
    last_source_stamp_usec = source_header.stamp().usec();

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
    if (bridge_node != nullptr) {
        const int64_t receipt_ns = bridge_node->now().nanoseconds();
        ros_power_state.header.stamp.sec =
            static_cast<int32_t>(receipt_ns / 1000000000LL);
        ros_power_state.header.stamp.nanosec =
            static_cast<uint32_t>(receipt_ns % 1000000000LL);
    }

    if (ros_power_state_pub) {
        ros_power_state_pub->publish(ros_power_state);
    }
}

int main(int argc, char **argv) {
    bool debug_mode = false;
    if (argc >= 2 && argv[1] != nullptr) {
        if (strcmp(argv[1], "log") == 0) {
            debug_mode = true;
        }
    }

    rclcpp::init(
        argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, bridge_signal_handler);
    std::signal(SIGTERM, bridge_signal_handler);

    auto node = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
    bridge_node = node;
    bridge_boot_id = make_bridge_boot_id();
    // core::NodeHandler constructs std::string objects directly from these
    // environment variables.  Passing a null pointer there aborts the process
    // before the bridge can report a useful, fail-closed startup error.
    const char *core_master_addr_env = std::getenv("CORE_MASTER_ADDR");
    const char *core_local_ip_env = std::getenv("CORE_LOCAL_IP");
    if (core_master_addr_env == nullptr || core_master_addr_env[0] == '\0' ||
        core_local_ip_env == nullptr || core_local_ip_env[0] == '\0') {
        RCLCPP_FATAL(
            node->get_logger(),
            "Missing required core environment: CORE_MASTER_ADDR and "
            "CORE_LOCAL_IP must both be nonempty");
        bridge_node.reset();
        node.reset();
        rclcpp::shutdown();
        return 2;
    }
    const std::string core_ip_default = resolve_core_ip_default();
    const std::string core_ip =
        node->declare_parameter<std::string>("core_ip", core_ip_default);
    motor_command_timeout_ms =
        node->declare_parameter<int>("motor_command_timeout_ms", 100);
    motor_command_max_age_ms =
        node->declare_parameter<int>("motor_command_max_age_ms", 100);
    motor_command_rearm_disabled_samples =
        node->declare_parameter<int>("motor_command_rearm_disabled_samples", 5);
    motor_disable_resend_period_ms =
        node->declare_parameter<int>("motor_disable_resend_period_ms", 20);
    motor_arbiter_heartbeat_period_ms =
        node->declare_parameter<int>("motor_arbiter_heartbeat_period_ms", 50);
    motor_output_status_period_ms =
        node->declare_parameter<int>("motor_output_status_period_ms", 20);
    motor_shutdown_disabled_packets =
        node->declare_parameter<int>("motor_shutdown_disabled_packets", 8);
    power_shutdown_off_packets =
        node->declare_parameter<int>("power_shutdown_off_packets", 3);
    power_command_max_age_ms =
        node->declare_parameter<int>("power_command_max_age_ms", 200);
    motor_command_max_pwm =
        node->declare_parameter<double>("motor_command_max_pwm", rinbo_bridge::kMaxRawMotorCommand);
    const int64_t min_servo_encoder =
        node->declare_parameter<int64_t>("motor_command_min_servo_encoder", 0);
    const int64_t max_servo_encoder =
        node->declare_parameter<int64_t>("motor_command_max_servo_encoder", 65535);
    const bool invalid_parameters =
        motor_command_timeout_ms <= 0 || motor_command_max_age_ms <= 0 ||
        motor_command_rearm_disabled_samples <= 0 || motor_disable_resend_period_ms <= 0 ||
        motor_arbiter_heartbeat_period_ms <= 0 || motor_output_status_period_ms <= 0 ||
        motor_shutdown_disabled_packets <= 0 || power_shutdown_off_packets <= 0 ||
        power_command_max_age_ms <= 0 || core_ip.empty() ||
        motor_command_timeout_ms > kHardMotorCommandTimeoutMaxMs ||
        motor_command_max_age_ms > kHardMotorCommandMaxAgeMaxMs ||
        motor_command_rearm_disabled_samples < kHardMotorCommandRearmMinSamples ||
        motor_disable_resend_period_ms > kHardMotorDisableResendMaxMs ||
        motor_arbiter_heartbeat_period_ms > kHardMotorArbiterHeartbeatMaxMs ||
        motor_output_status_period_ms > kHardMotorOutputStatusMaxMs ||
        motor_shutdown_disabled_packets < kHardMotorShutdownMinPackets ||
        power_shutdown_off_packets < kHardPowerShutdownMinPackets ||
        power_command_max_age_ms > kHardPowerCommandMaxAgeMaxMs ||
        !std::isfinite(motor_command_max_pwm) || motor_command_max_pwm <= 0.0 ||
        motor_command_max_pwm > kHardMotorCommandMaxPwm ||
        min_servo_encoder < 0 || max_servo_encoder < min_servo_encoder ||
        static_cast<uint64_t>(max_servo_encoder) >
            static_cast<uint64_t>(kHardServoEncoderMax);
    if (invalid_parameters) {
        RCLCPP_FATAL(
            node->get_logger(),
            "Invalid/unsafe bridge parameters: CORE_IP must be nonempty; motor timeout/max-age "
            "must be <=100ms; rearm must be >=5 samples; disable resend/status must be <=20ms; "
            "heartbeat must be <=50ms; shutdown must send >=8 motor and >=3 power packets; "
            "power max-age must be <=200ms; max-PWM must be finite and in (0,3300] raw units; servo encoder "
            "bounds must be ordered within [0,65535]");
        bridge_node.reset();
        node.reset();
        rclcpp::shutdown();
        return 2;
    }
    if (setenv("CORE_IP", core_ip.c_str(), 1) != 0) {
        RCLCPP_FATAL(node->get_logger(), "Failed to set CORE_IP to '%s'", core_ip.c_str());
        bridge_node.reset();
        node.reset();
        rclcpp::shutdown();
        return 2;
    }
    motor_command_min_servo_encoder = static_cast<uint32_t>(min_servo_encoder);
    motor_command_max_servo_encoder = static_cast<uint32_t>(max_servo_encoder);
    RCLCPP_INFO(
        node->get_logger(),
        "Motor arbiter: timeout=%dms max_age=%dms rearm=%d disabled packets "
        "disable_resend=%dms heartbeat=%dms status=%dms max_pwm=%.1f servo=[%u,%u] CORE_IP=%s",
        motor_command_timeout_ms, motor_command_max_age_ms,
        motor_command_rearm_disabled_samples, motor_disable_resend_period_ms,
        motor_arbiter_heartbeat_period_ms, motor_output_status_period_ms,
        motor_command_max_pwm,
        motor_command_min_servo_encoder,
        motor_command_max_servo_encoder, core_ip.c_str());

    auto ros_motor_cmd_sub =
        node->create_subscription<rinbo_msgs::msg::MotorCmdStamped>(
            kMotorCommandTopic,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            [](const rinbo_msgs::msg::MotorCmdStamped::ConstSharedPtr cmd,
               const rclcpp::MessageInfo &message_info) {
                ros_motor_cmd_cb(cmd, message_info);
            });
    auto ros_power_cmd_sub =
        node->create_subscription<rinbo_msgs::msg::PowerCmdStamped>(
            kPowerCommandTopic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            [](const rinbo_msgs::msg::PowerCmdStamped::ConstSharedPtr cmd,
               const rclcpp::MessageInfo &message_info) {
                ros_power_cmd_cb(cmd, message_info);
            });
    ros_motor_state_pub = node->create_publisher<rinbo_msgs::msg::MotorStateStamped>("motor/state", 1);
    ros_power_state_pub = node->create_publisher<rinbo_msgs::msg::PowerStateStamped>("power/state", 1);
    const auto monitor_qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
    motor_requested_monitor_pub = node->create_publisher<rinbo_msgs::msg::MotorCmdStamped>(
        "/rinbo/monitor/motor_requested", monitor_qos);
    motor_forwarded_monitor_pub = node->create_publisher<rinbo_msgs::msg::MotorCmdStamped>(
        "/rinbo/monitor/motor_forwarded", monitor_qos);
    const auto arbiter_status_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    motor_output_enabled_pub =
        node->create_publisher<std_msgs::msg::Bool>(
            kMotorOutputEnabledTopic, arbiter_status_qos);
    motor_arbiter_ready_pub =
        node->create_publisher<std_msgs::msg::Bool>(
            kMotorArbiterReadyTopic, arbiter_status_qos);
    motor_arbiter_heartbeat_pub =
        node->create_publisher<std_msgs::msg::Empty>(
            kMotorArbiterHeartbeatTopic,
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    motor_arbiter_epoch_pub =
        node->create_publisher<rinbo_msgs::msg::Header>(
            kMotorArbiterEpochTopic, arbiter_status_qos);
    motor_rearm_ack_pub =
        node->create_publisher<rinbo_msgs::msg::Header>(
            kMotorRearmAckTopic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    motor_active_ack_pub =
        node->create_publisher<rinbo_msgs::msg::Header>(
            kMotorActiveAckTopic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    auto estop_sub = node->create_subscription<std_msgs::msg::Bool>(
        kEstopTopic, 10, estop_cb);

    core::NodeHandler nh_;
    core::Subscriber<motor_msg::MotorStateStamped> &grpc_motor_state_sub = nh_.subscribe<motor_msg::MotorStateStamped>("motor/state", 1000, grpc_motor_state_cb);
    core::Subscriber<power_msg::PowerStateStamped> &grpc_power_state_sub = nh_.subscribe<power_msg::PowerStateStamped>("power/state", 1000, grpc_power_state_cb);
    grpc_motor_cmd_pub = &(nh_.advertise<motor_msg::MotorCmdStamped>("motor/command"));
    grpc_power_cmd_pub = &(nh_.advertise<power_msg::PowerCmdStamped>("power/command"));

    {
        std::lock_guard<std::mutex> lock(mutex_grpc_motor_cmd);
        latch_motor_safety_locked(
            "bridge startup requires consecutive fresh all-disabled commands");
    }
    rclcpp::WallRate rate(1000);

    int exit_code = 0;
    try {
    int loop_counter = 0;
    while (rclcpp::ok() && bridge_shutdown_requested == 0) {
        if (debug_mode) RCLCPP_INFO(rclcpp::get_logger("rinbo_ros2_bridge"), "Loop Count: %d", loop_counter);

        rclcpp::spin_some(node);
        core::spinOnce();

        const auto loop_time = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(mutex_grpc_motor_cmd);
            if (!motor_safety_latched && ros_motor_cmd_active &&
                loop_time - last_ros_motor_cmd_time >
                    std::chrono::milliseconds(motor_command_timeout_ms)) {
                latch_motor_safety_locked(
                    "active motor command stream exceeded " +
                    std::to_string(motor_command_timeout_ms) + "ms");
            }
            const bool disable_due =
                motor_safety_latched &&
                (!disable_publish_time_valid ||
                 loop_time - last_disable_publish_time >=
                     std::chrono::milliseconds(motor_disable_resend_period_ms));
            if (disable_due) publish_all_disabled_locked(loop_time);
            const bool arbiter_heartbeat_due =
                !arbiter_heartbeat_publish_time_valid ||
                loop_time - last_arbiter_heartbeat_publish_time >=
                    std::chrono::milliseconds(motor_arbiter_heartbeat_period_ms);
            if (arbiter_heartbeat_due) {
                publish_motor_arbiter_heartbeat();
                publish_motor_arbiter_ready(!motor_safety_latched);
                publish_motor_arbiter_epoch();
            }
            const bool output_status_due =
                !motor_output_status_time_valid ||
                loop_time - last_motor_output_status_time >=
                    std::chrono::milliseconds(motor_output_status_period_ms);
            if (output_status_due) {
                publish_motor_output_enabled(
                    !motor_safety_latched && ros_motor_cmd_active &&
                    !software_estop_asserted.load());
                last_motor_output_status_time = loop_time;
                motor_output_status_time_valid = true;
            }
        }

        if (debug_mode) RCLCPP_INFO(rclcpp::get_logger("rinbo_ros2_bridge"), " ");

        loop_counter++;
        rate.sleep();
    }
    } catch (const std::exception &exc) {
        exit_code = 3;
        RCLCPP_FATAL(node->get_logger(), "Bridge loop failed: %s", exc.what());
    } catch (...) {
        exit_code = 3;
        RCLCPP_FATAL(node->get_logger(), "Bridge loop failed: unknown exception");
    }

    bool shutdown_stop_ok = false;
    {
        std::lock_guard<std::mutex> lock(mutex_grpc_motor_cmd);
        shutdown_stop_ok = publish_shutdown_stop_locked();
    }
    core::spinOnce();
    bool shutdown_power_ok = true;
    for (int packet = 0; packet < power_shutdown_off_packets; ++packet) {
        try {
            {
                std::lock_guard<std::mutex> lock(mutex_grpc_power_cmd);
                publish_all_power_off_locked();
            }
            core::spinOnce();
        } catch (const std::exception &exc) {
            shutdown_power_ok = false;
            RCLCPP_ERROR(
                node->get_logger(), "Failed to send shutdown all-off power command: %s",
                exc.what());
        } catch (...) {
            shutdown_power_ok = false;
            RCLCPP_ERROR(
                node->get_logger(),
                "Failed to send shutdown all-off power command: unknown exception");
        }
        if (packet + 1 < power_shutdown_off_packets) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(motor_disable_resend_period_ms));
        }
    }
    if (!shutdown_stop_ok && exit_code == 0) exit_code = 4;
    if (!shutdown_power_ok && exit_code == 0) exit_code = 5;
    if (motor_output_enabled_pub != nullptr) {
        motor_output_enabled_pub->wait_for_all_acked(std::chrono::milliseconds(50));
    }
    if (motor_arbiter_ready_pub != nullptr) {
        motor_arbiter_ready_pub->wait_for_all_acked(std::chrono::milliseconds(50));
    }
    if (motor_arbiter_epoch_pub != nullptr) {
        motor_arbiter_epoch_pub->wait_for_all_acked(std::chrono::milliseconds(50));
    }
    RCLCPP_INFO(
        node->get_logger(),
        "Shutdown safety sequence %s: requested %d all-disabled motor packets "
        "and %d all-off power packets",
        shutdown_stop_ok && shutdown_power_ok ? "completed" : "FAILED",
        motor_shutdown_disabled_packets, power_shutdown_off_packets);
    RCLCPP_INFO(rclcpp::get_logger("rinbo_ros2_bridge"), "Rinbo ROS2 Bridge is killed");

    // Destroy every ROS endpoint while the rcl context is still valid.  In
    // particular, keeping the file-scope publisher shared_ptrs alive until
    // static destruction (after rclcpp::shutdown) makes Fast DDS report
    // publish/data-reader destruction failures and turns a normal Ctrl+C into
    // exit status 1.
    ros_motor_cmd_sub.reset();
    ros_power_cmd_sub.reset();
    estop_sub.reset();
    motor_output_enabled_pub.reset();
    motor_arbiter_ready_pub.reset();
    motor_arbiter_heartbeat_pub.reset();
    motor_arbiter_epoch_pub.reset();
    motor_rearm_ack_pub.reset();
    motor_active_ack_pub.reset();
    {
        // gRPC state callbacks run on the core library's worker threads.
        // Synchronize their last possible ROS publication before releasing
        // the global state publishers and node alias.
        std::scoped_lock state_locks(mutex_ros_motor_state, mutex_ros_power_state);
        ros_motor_state_pub.reset();
        ros_power_state_pub.reset();
        bridge_node.reset();
    }
    node.reset();
    rclcpp::shutdown();

    // The vendored core library creates detached, process-lifetime worker
    // threads that wait forever on core::spin_cv.  It exposes no stop/join API;
    // returning from main would therefore destroy that condition variable
    // while waiters still exist (hang/undefined behaviour).  All commanded
    // safety packets and ROS cleanup are complete at this point, so finish the
    // process without running the unsupported core static destructors.
    std::cout.flush();
    std::cerr.flush();
    std::_Exit(exit_code);
}
