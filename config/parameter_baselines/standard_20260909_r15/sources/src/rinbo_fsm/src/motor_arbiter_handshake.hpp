#pragma once

#include "rclcpp/message_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "safety_invariants.hpp"
#include "rinbo_msgs/msg/header.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rmw/types.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace rinbo_fsm {

using MotorArbiterPublisherGid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

struct MotorArbiterPublisherSnapshot {
    std::size_t endpoint_count = 0;
    std::string node_name;
    std::string node_namespace;
    MotorArbiterPublisherGid endpoint_gid {};
};

inline std::string motor_arbiter_fully_qualified_node_name(
    const std::string& node_namespace,
    const std::string& node_name) {
    if (node_namespace.empty() || node_namespace == "/") {
        return "/" + node_name;
    }
    return node_namespace.back() == '/'
        ? node_namespace + node_name
        : node_namespace + "/" + node_name;
}

inline bool motor_arbiter_is_expected_root_node(
    const std::string& expected_node_name,
    const std::string& actual_node_name,
    const std::string& actual_node_namespace) {
    return actual_node_name == expected_node_name &&
        actual_node_namespace == "/";
}

// Keep the graph-policy decision independent from graph discovery itself so
// zero/multiple endpoints, node identity, and callback GID correlation are
// deterministic unit-test inputs. Unknown or ambiguous graph state is never
// accepted.
inline bool validate_motor_arbiter_publisher_identity(
    const std::string& topic,
    const std::string& expected_node_name,
    const MotorArbiterPublisherGid& callback_gid,
    const MotorArbiterPublisherSnapshot& snapshot,
    std::string& issue) {
    issue.clear();
    if (snapshot.endpoint_count != 1U) {
        std::ostringstream out;
        out << "expected exactly one trusted publisher on " << topic
            << ", got " << snapshot.endpoint_count;
        issue = out.str();
        return false;
    }

    const bool gid_matches = snapshot.endpoint_gid == callback_gid;
    const std::string actual_node = motor_arbiter_fully_qualified_node_name(
        snapshot.node_namespace, snapshot.node_name);
    if (!motor_arbiter_is_expected_root_node(
            expected_node_name, snapshot.node_name, snapshot.node_namespace)) {
        issue = "publisher on " + topic + " is node " + actual_node +
            ", not node /" + expected_node_name +
            "; callback GID matches sole graph endpoint=" +
            (gid_matches ? "yes" : "no");
        return false;
    }
    if (!gid_matches) {
        issue = "callback source GID does not match the sole publisher on " + topic +
            " (graph node " + actual_node + ")";
        return false;
    }
    return true;
}

inline bool validate_motor_arbiter_subscription_identity(
    const std::string& topic,
    const std::string& expected_node_name,
    std::size_t endpoint_count,
    const std::string& node_name,
    const std::string& node_namespace,
    std::string& issue) {
    issue.clear();
    if (endpoint_count != 1U) {
        std::ostringstream out;
        out << "expected exactly one subscriber on " << topic
            << ", got " << endpoint_count;
        issue = out.str();
        return false;
    }
    if (!motor_arbiter_is_expected_root_node(
            expected_node_name, node_name, node_namespace)) {
        issue = "subscriber on " + topic + " is node " +
            motor_arbiter_fully_qualified_node_name(node_namespace, node_name) +
            ", not node /" + expected_node_name;
        return false;
    }
    return true;
}

// A graph endpoint may legitimately be replaced while DDS discovery and an
// uncommitted rearm are converging. Once a correlated rearm ACK commits the
// protocol, however, accepting a different endpoint would mix two sessions.
class MotorArbiterEndpointPinState {
public:
    bool observe(
        const MotorArbiterPublisherGid& endpoint_gid,
        bool protocol_committed) {
        if (!endpoint_gid_) {
            endpoint_gid_ = endpoint_gid;
            return true;
        }
        if (*endpoint_gid_ == endpoint_gid) return true;
        if (protocol_committed) return false;
        endpoint_gid_ = endpoint_gid;
        return true;
    }

private:
    std::optional<MotorArbiterPublisherGid> endpoint_gid_;
};

// The Bridge publishes a boot identifier plus a monotonic latch generation.
// Unlike a depth-one Bool transition, a generation change remains observable
// after the latch has been cleared again.
class MotorArbiterEpochState {
public:
    bool observe(
        const std::string& boot_id,
        uint32_t latch_generation,
        bool terminal_on_change = true) {
        if (!received_) {
            received_ = true;
            boot_id_ = boot_id;
            latch_generation_ = latch_generation;
            return false;
        }
        if (boot_id == boot_id_ && latch_generation == latch_generation_) {
            return false;
        }
        boot_id_ = boot_id;
        latch_generation_ = latch_generation;
        if (terminal_on_change) {
            changed_ = true;
            return true;
        }
        return false;
    }

    bool received() const { return received_; }
    bool changed() const { return changed_; }
    const std::string& boot_id() const { return boot_id_; }
    uint32_t latch_generation() const { return latch_generation_; }

private:
    bool received_ = false;
    bool changed_ = false;
    std::string boot_id_;
    uint32_t latch_generation_ = 0;
};

// Protocol state is separate from ROS graph inspection so the correlated
// rearm/active acknowledgement rules can be unit tested deterministically.
class MotorArbiterHandshakeState {
public:
    void observe_ready_status(bool ready) {
        ++status_generation_;
        status_received_ = true;
        ready_status_ = ready;

        if (armed_) {
            if (!ready) faulted_ = true;
            return;
        }

        if (rearm_ack_received_ && ready &&
            status_generation_ > rearm_ack_status_generation_) {
            armed_ = true;
        }
    }

    bool can_publish_rearm(std::size_t command_subscriber_count) const {
        return !armed_ && !faulted_ && status_received_ &&
            command_subscriber_count == 1U;
    }

    bool start_rearm_request() {
        if (!can_publish_rearm(1U)) return false;
        if (!rearm_requested_) {
            rearm_requested_ = true;
            return true;
        }
        return false;
    }

    bool note_rearm_command(uint32_t sequence) {
        if (!rearm_requested_ || armed_ || faulted_) return false;
        rearm_sequences_.insert(sequence);
        return true;
    }

    bool observe_rearm_ack(uint32_t sequence) {
        if (!rearm_requested_ || armed_ || faulted_ ||
            rearm_sequences_.count(sequence) == 0U) {
            return false;
        }
        if (!rearm_ack_received_) {
            rearm_ack_received_ = true;
            rearm_ack_status_generation_ = status_generation_;
            return true;
        }
        return false;
    }

    bool ready_for_output(std::size_t command_subscriber_count) const {
        return armed_ && !faulted_ && ready_status_ &&
            command_subscriber_count == 1U;
    }

    bool note_active_command(uint32_t sequence) {
        if (!armed_ || faulted_ || active_output_confirmed_) return false;
        active_ack_requested_ = true;
        active_sequences_.insert(sequence);
        return true;
    }

    bool observe_active_ack(uint32_t sequence) {
        if (!active_ack_requested_ || faulted_ ||
            active_sequences_.count(sequence) == 0U) {
            return false;
        }
        const bool newly_confirmed = !active_output_confirmed_;
        active_output_confirmed_ = true;
        return newly_confirmed;
    }

    void force_fault() { faulted_ = true; }

    std::optional<std::string> immediate_violation(
        std::size_t command_subscriber_count) const {
        if (faulted_) {
            return "motor arbiter readiness or endpoint identity was revoked after protocol commitment";
        }
        if (armed_ && command_subscriber_count != 1U) {
            return "expected exactly one /motor/command bridge subscriber after arming";
        }
        return std::nullopt;
    }

    bool status_received() const { return status_received_; }
    bool ready_status() const { return ready_status_; }
    bool rearm_requested() const { return rearm_requested_; }
    bool rearm_ack_received() const { return rearm_ack_received_; }
    bool armed() const { return armed_; }
    bool protocol_committed() const { return rearm_ack_received_ || armed_; }
    bool faulted() const { return faulted_; }
    bool active_ack_requested() const { return active_ack_requested_; }
    bool active_output_confirmed() const { return active_output_confirmed_; }

private:
    bool status_received_ = false;
    bool ready_status_ = false;
    bool rearm_requested_ = false;
    bool rearm_ack_received_ = false;
    bool armed_ = false;
    bool faulted_ = false;
    bool active_ack_requested_ = false;
    bool active_output_confirmed_ = false;
    uint64_t status_generation_ = 0;
    uint64_t rearm_ack_status_generation_ = 0;
    std::unordered_set<uint32_t> rearm_sequences_;
    std::unordered_set<uint32_t> active_sequences_;
};

class MotorArbiterHeartbeatState {
public:
    using TimePoint = std::chrono::steady_clock::time_point;

    void observe(const TimePoint& observed_at) {
        last_observed_at_ = observed_at;
    }

    bool received() const { return last_observed_at_.has_value(); }

    bool fresh(const TimePoint& now, double stale_seconds) const {
        if (!last_observed_at_ || now < *last_observed_at_) return false;
        return std::chrono::duration<double>(now - *last_observed_at_).count() <=
            stale_seconds;
    }

private:
    std::optional<TimePoint> last_observed_at_;
};

// Each FSM uses unique session, rearm, and active-probe frame ids. The bridge
// echoes every marked probe until the FSM advances, so discovery delay cannot
// exhaust a fixed ACK retry budget and an old session cannot satisfy this one.
class MotorArbiterHandshake {
public:
    explicit MotorArbiterHandshake(rclcpp::Node& node)
        : node_(node),
          logger_(node.get_logger()),
          expected_bridge_node_name_(node.declare_parameter<std::string>(
              "safety.motor_arbiter_node_name", "rinbo_ros2_bridge")),
          timeout_seconds_(node.declare_parameter<double>(
              "safety.motor_arbiter_ready_timeout_s", 5.0)),
          heartbeat_stale_seconds_(node.declare_parameter<double>(
              "safety.motor_arbiter_heartbeat_stale_s", 0.25)),
          active_ack_timeout_seconds_(node.declare_parameter<double>(
              "safety.motor_command_ack_timeout_s", 0.25)),
          command_frame_id_(make_command_frame_id(node)),
          rearm_frame_id_(command_frame_id_ + "/rearm"),
          active_probe_frame_id_(command_frame_id_ + "/active-probe"),
          started_at_(std::chrono::steady_clock::now()) {
        validate_motor_arbiter_contract(
            expected_bridge_node_name_, timeout_seconds_,
            heartbeat_stale_seconds_, active_ack_timeout_seconds_);

        const auto ready_qos =
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        ready_sub_ = node.create_subscription<std_msgs::msg::Bool>(
            kReadyTopic, ready_qos,
            [this](const std_msgs::msg::Bool::ConstSharedPtr msg,
                   const rclcpp::MessageInfo& info) {
                std::string issue;
                if (!publisher_is_unique_bridge(kReadyTopic, info, issue)) {
                    note_untrusted_source(issue);
                    return;
                }

                bool first_status = false;
                bool changed = false;
                bool became_armed = false;
                bool became_faulted = false;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    first_status = !state_.status_received();
                    const bool previous_ready = state_.ready_status();
                    const bool previous_armed = state_.armed();
                    const bool previous_faulted = state_.faulted();
                    state_.observe_ready_status(msg->data);
                    changed = first_status || previous_ready != msg->data;
                    became_armed = !previous_armed && state_.armed();
                    became_faulted = !previous_faulted && state_.faulted();
                }

                if (became_faulted) {
                    RCLCPP_ERROR(logger_, "Motor arbiter readiness was revoked after arming");
                } else if (became_armed) {
                    RCLCPP_INFO(
                        logger_,
                        "Motor arbiter rearm ACK and post-ACK ready state confirmed");
                } else if (changed) {
                    RCLCPP_INFO(
                        logger_, "Motor arbiter reports ready=%s",
                        msg->data ? "true" : "false");
                }
            });

        epoch_sub_ = node.create_subscription<rinbo_msgs::msg::Header>(
            kEpochTopic, ready_qos,
            [this](const rinbo_msgs::msg::Header::ConstSharedPtr msg,
                   const rclcpp::MessageInfo& info) {
                if (msg->frame_id.empty()) {
                    note_untrusted_source("motor arbiter epoch has an empty Bridge boot id");
                    return;
                }
                std::string issue;
                if (!publisher_is_unique_bridge(kEpochTopic, info, issue)) {
                    note_untrusted_source(issue);
                    return;
                }

                bool first_epoch = false;
                bool epoch_changed = false;
                bool epoch_rebased = false;
                std::string old_boot;
                std::string epoch_issue;
                uint32_t old_generation = 0;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    first_epoch = !epoch_state_.received();
                    old_boot = epoch_state_.boot_id();
                    old_generation = epoch_state_.latch_generation();
                    const bool values_differ = !first_epoch &&
                        (old_boot != msg->frame_id || old_generation != msg->seq);
                    const bool epoch_is_committed = state_.protocol_committed();
                    epoch_changed = epoch_state_.observe(
                        msg->frame_id, msg->seq, epoch_is_committed);
                    epoch_rebased = values_differ && !epoch_is_committed;
                    if (epoch_changed) {
                        state_.force_fault();
                        std::ostringstream out;
                        out << "motor arbiter Bridge boot/latch epoch changed from "
                            << old_boot << ":" << old_generation << " to "
                            << msg->frame_id << ":" << msg->seq;
                        epoch_issue = out.str();
                        external_fault_reason_ = epoch_issue;
                    }
                }
                if (epoch_changed) {
                    RCLCPP_ERROR(logger_, "%s", epoch_issue.c_str());
                } else if (epoch_rebased) {
                    RCLCPP_WARN(
                        logger_,
                        "Motor arbiter epoch changed during uncommitted rearm; "
                        "using new baseline %s:%u",
                        msg->frame_id.c_str(), msg->seq);
                } else if (first_epoch) {
                    RCLCPP_INFO(
                        logger_, "Motor arbiter epoch confirmed: %s:%u",
                        msg->frame_id.c_str(), msg->seq);
                }
            });

        heartbeat_sub_ = node.create_subscription<std_msgs::msg::Empty>(
            kHeartbeatTopic,
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            [this](const std_msgs::msg::Empty::ConstSharedPtr,
                   const rclcpp::MessageInfo& info) {
                // Timestamp callback entry before either synchronous graph
                // query so their own latency cannot extend heartbeat freshness.
                const auto observed_at = std::chrono::steady_clock::now();
                std::string issue;
                if (!publisher_is_unique_bridge(kHeartbeatTopic, info, issue)) {
                    note_untrusted_source(issue);
                    return;
                }

                std::string command_issue;
                const bool command_valid = command_endpoint_is_unique_bridge(command_issue);
                bool faulted_now = false;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    heartbeat_state_.observe(observed_at);
                    command_bridge_valid_ = command_valid;
                    command_bridge_issue_ = command_valid ? std::string() : command_issue;
                    if (state_.protocol_committed() &&
                        !command_valid && !state_.faulted()) {
                        state_.force_fault();
                        external_fault_reason_ = command_issue;
                        faulted_now = true;
                    }
                }
                if (faulted_now) RCLCPP_ERROR(logger_, "%s", command_issue.c_str());
            });

        rearm_ack_sub_ = node.create_subscription<rinbo_msgs::msg::Header>(
            kRearmAckTopic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [this](const rinbo_msgs::msg::Header::ConstSharedPtr msg,
                   const rclcpp::MessageInfo& info) {
                std::string issue;
                if (!publisher_is_unique_bridge(kRearmAckTopic, info, issue)) {
                    note_untrusted_source(issue);
                    return;
                }

                bool matched = false;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (command_bridge_valid_ &&
                        msg->frame_id == expected_ack_frame_id_locked(rearm_frame_id_)) {
                        matched = state_.observe_rearm_ack(msg->seq);
                    }
                }
                if (matched) {
                    RCLCPP_INFO(
                        logger_, "Bridge acknowledged disabled-command rearm sequence %u",
                        msg->seq);
                }
            });

        active_ack_sub_ = node.create_subscription<rinbo_msgs::msg::Header>(
            kActiveAckTopic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [this](const rinbo_msgs::msg::Header::ConstSharedPtr msg,
                   const rclcpp::MessageInfo& info) {
                std::string issue;
                if (!publisher_is_unique_bridge(kActiveAckTopic, info, issue)) {
                    note_untrusted_source(issue);
                    return;
                }

                bool matched = false;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (command_bridge_valid_ &&
                        msg->frame_id == expected_ack_frame_id_locked(active_probe_frame_id_)) {
                        matched = state_.observe_active_ack(msg->seq);
                    }
                }
                if (matched) {
                    RCLCPP_INFO(
                        logger_, "Bridge acknowledged active command sequence %u", msg->seq);
                }
            });
    }

    const std::string& command_frame_id() const { return command_frame_id_; }
    const std::string& rearm_frame_id() const { return rearm_frame_id_; }
    const std::string& active_probe_frame_id() const { return active_probe_frame_id_; }

    bool ready_for_output(std::size_t command_subscriber_count) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_.ready_for_output(command_subscriber_count) &&
            epoch_state_.received() && !epoch_state_.changed() &&
            command_bridge_valid_ &&
            heartbeat_is_fresh_locked(std::chrono::steady_clock::now());
    }

    bool mark_rearm_command_about_to_publish(
        std::size_t command_subscriber_count) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        if (!state_.can_publish_rearm(command_subscriber_count) ||
            !epoch_state_.received() || epoch_state_.changed() ||
            !command_bridge_valid_ || !heartbeat_is_fresh_locked(now)) {
            return false;
        }
        if (state_.start_rearm_request()) rearm_started_at_ = now;
        return true;
    }

    bool record_rearm_command_published(uint32_t sequence) {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_.note_rearm_command(sequence);
    }

    bool mark_active_command_published(
        std::size_t command_subscriber_count, uint32_t sequence) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        if (!state_.ready_for_output(command_subscriber_count) ||
            !epoch_state_.received() || epoch_state_.changed() ||
            !command_bridge_valid_ || !heartbeat_is_fresh_locked(now)) {
            return false;
        }
        const bool first_request = !state_.active_ack_requested();
        if (!state_.note_active_command(sequence)) return false;
        if (first_request) active_ack_started_at_ = now;
        return true;
    }

    bool active_output_confirmed() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_.active_output_confirmed();
    }

    std::optional<std::string> violation(
        std::size_t command_subscriber_count) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!external_fault_reason_.empty()) return external_fault_reason_;
        if (const auto reason = state_.immediate_violation(command_subscriber_count)) {
            return reason;
        }

        const auto now = std::chrono::steady_clock::now();
        if (state_.armed() && !command_bridge_valid_) {
            return command_bridge_issue_.empty()
                ? "motor command endpoint is not the unique expected bridge subscriber"
                : command_bridge_issue_;
        }
        if (state_.armed() && !heartbeat_is_fresh_locked(now)) {
            return "motor arbiter heartbeat became stale after arming";
        }
        if (state_.active_ack_requested() && !state_.active_output_confirmed()) {
            const double age =
                std::chrono::duration<double>(now - active_ack_started_at_).count();
            if (age > active_ack_timeout_seconds_) {
                return "bridge did not acknowledge a correlated active motor command";
            }
        }
        if (state_.ready_for_output(command_subscriber_count) &&
            command_bridge_valid_ && heartbeat_is_fresh_locked(now)) {
            return std::nullopt;
        }

        const auto reference = state_.rearm_requested() ? rearm_started_at_ : started_at_;
        const double elapsed = std::chrono::duration<double>(now - reference).count();
        if (elapsed <= timeout_seconds_) return std::nullopt;

        if (command_subscriber_count != 1U) {
            return "motor arbiter handshake timeout: /motor/command must have exactly one subscriber";
        }
        if (!heartbeat_state_.received()) {
            return "motor arbiter handshake timeout: no trusted arbiter heartbeat";
        }
        if (!epoch_state_.received()) {
            return "motor arbiter handshake timeout: no trusted Bridge boot/latch epoch";
        }
        if (!heartbeat_is_fresh_locked(now)) {
            return "motor arbiter handshake timeout: trusted arbiter heartbeat is stale";
        }
        if (!command_bridge_valid_) {
            return command_bridge_issue_.empty()
                ? "motor arbiter handshake timeout: command subscriber is not the expected bridge"
                : command_bridge_issue_;
        }
        if (!state_.status_received()) {
            return "motor arbiter handshake timeout: no trusted readiness status";
        }
        if (!state_.rearm_requested()) {
            return "motor arbiter handshake timeout: unable to start disabled-command rearm";
        }
        if (!state_.rearm_ack_received()) {
            return "motor arbiter handshake timeout: no correlated disabled-command ACK";
        }
        return "motor arbiter handshake timeout: no post-ACK ready status";
    }

    std::string waiting_reason(std::size_t command_subscriber_count) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (command_subscriber_count != 1U) {
            return "waiting for exactly one /motor/command subscriber";
        }
        if (!heartbeat_state_.received()) return "waiting for a trusted arbiter heartbeat";
        if (!epoch_state_.received()) return "waiting for the Bridge boot/latch epoch";
        if (!command_bridge_valid_) {
            return command_bridge_issue_.empty()
                ? "waiting for the expected Bridge command subscriber"
                : command_bridge_issue_;
        }
        if (!state_.status_received()) return "waiting for trusted arbiter readiness";
        if (!state_.rearm_requested()) return "waiting to send disabled rearm commands";
        if (!state_.rearm_ack_received()) {
            return "waiting for a correlated disabled-command ACK";
        }
        return "waiting for post-ACK arbiter ready status";
    }

private:
    static constexpr const char* kCommandTopic = "/motor/command";
    static constexpr const char* kReadyTopic = "/rinbo/motor_arbiter_ready";
    static constexpr const char* kHeartbeatTopic = "/rinbo/motor_arbiter_heartbeat";
    static constexpr const char* kEpochTopic = "/rinbo/motor_arbiter_epoch";
    static constexpr const char* kRearmAckTopic = "/rinbo/motor_rearm_ack";
    static constexpr const char* kActiveAckTopic = "/rinbo/motor_active_ack";

    static std::string make_command_frame_id(const rclcpp::Node& node) {
        const auto wall_nonce = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto steady_nonce = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        std::random_device entropy;
        std::ostringstream out;
        out << node.get_name() << "/session-" << std::hex
            << static_cast<uint64_t>(wall_nonce) << "-"
            << static_cast<uint64_t>(steady_nonce);
        for (int index = 0; index < 4; ++index) {
            out << "-" << std::setw(8) << std::setfill('0')
                << static_cast<uint32_t>(entropy());
        }
        return out.str();
    }

    MotorArbiterPublisherGid received_gid(const rclcpp::MessageInfo& info) const {
        MotorArbiterPublisherGid result {};
        const auto& gid = info.get_rmw_message_info().publisher_gid;
        std::copy_n(gid.data, result.size(), result.begin());
        return result;
    }

    bool publisher_is_unique_bridge(
        const std::string& topic,
        const rclcpp::MessageInfo& info,
        std::string& issue) {
        try {
            const auto endpoints = node_.get_publishers_info_by_topic(topic);
            MotorArbiterPublisherSnapshot snapshot;
            snapshot.endpoint_count = endpoints.size();
            if (endpoints.size() == 1U) {
                snapshot.node_name = endpoints.front().node_name();
                snapshot.node_namespace = endpoints.front().node_namespace();
                snapshot.endpoint_gid = endpoints.front().endpoint_gid();
            }
            if (!validate_motor_arbiter_publisher_identity(
                    topic, expected_bridge_node_name_, received_gid(info), snapshot, issue)) {
                return false;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!publisher_endpoint_pins_[topic].observe(
                        snapshot.endpoint_gid, state_.protocol_committed())) {
                    issue = "trusted publisher endpoint GID changed after motor arbiter "
                        "protocol commitment on " + topic;
                    return false;
                }
            }
            return true;
        } catch (const std::exception& exc) {
            issue = "publisher graph query failed for " + topic + ": " + exc.what();
            return false;
        }
    }

    bool command_endpoint_is_unique_bridge(std::string& issue) {
        try {
            const auto endpoints = node_.get_subscriptions_info_by_topic(kCommandTopic);
            const std::string node_name = endpoints.size() == 1U
                ? endpoints.front().node_name() : std::string();
            const std::string node_namespace = endpoints.size() == 1U
                ? endpoints.front().node_namespace() : std::string();
            if (!validate_motor_arbiter_subscription_identity(
                    kCommandTopic, expected_bridge_node_name_, endpoints.size(),
                    node_name, node_namespace, issue)) {
                return false;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!command_subscription_pin_.observe(
                        endpoints.front().endpoint_gid(), state_.protocol_committed())) {
                    issue = "/motor/command subscriber endpoint GID changed after motor "
                        "arbiter protocol commitment";
                    return false;
                }
            }
            return true;
        } catch (const std::exception& exc) {
            issue = std::string("subscription graph query failed for /motor/command: ") +
                exc.what();
            return false;
        }
    }

    void note_untrusted_source(const std::string& issue) {
        bool log_issue = false;
        bool protocol_committed = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            log_issue = issue != last_source_issue_;
            last_source_issue_ = issue;
            protocol_committed = state_.protocol_committed();
            if (protocol_committed) {
                state_.force_fault();
                external_fault_reason_ = issue;
            }
        }
        if (log_issue) {
            if (protocol_committed) {
                RCLCPP_ERROR(logger_, "%s", issue.c_str());
            } else {
                RCLCPP_WARN(logger_, "%s; ignoring sample during handshake", issue.c_str());
            }
        }
    }

    bool heartbeat_is_fresh_locked(
        const std::chrono::steady_clock::time_point& now) const {
        return heartbeat_state_.fresh(now, heartbeat_stale_seconds_);
    }

    std::string expected_ack_frame_id_locked(const std::string& probe_frame_id) const {
        if (!epoch_state_.received()) return std::string();
        return probe_frame_id + "|bridge=" + epoch_state_.boot_id() +
            "|latch=" + std::to_string(epoch_state_.latch_generation());
    }

    rclcpp::Node& node_;
    rclcpp::Logger logger_;
    std::string expected_bridge_node_name_;
    double timeout_seconds_;
    double heartbeat_stale_seconds_;
    double active_ack_timeout_seconds_;
    std::string command_frame_id_;
    std::string rearm_frame_id_;
    std::string active_probe_frame_id_;
    std::chrono::steady_clock::time_point started_at_;
    std::chrono::steady_clock::time_point rearm_started_at_;
    std::chrono::steady_clock::time_point active_ack_started_at_;
    MotorArbiterHeartbeatState heartbeat_state_;
    bool command_bridge_valid_ = false;
    std::string command_bridge_issue_;
    std::string last_source_issue_;
    std::string external_fault_reason_;
    mutable std::mutex mutex_;
    MotorArbiterHandshakeState state_;
    MotorArbiterEpochState epoch_state_;
    std::unordered_map<std::string, MotorArbiterEndpointPinState>
        publisher_endpoint_pins_;
    MotorArbiterEndpointPinState command_subscription_pin_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
    rclcpp::Subscription<rinbo_msgs::msg::Header>::SharedPtr epoch_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
    rclcpp::Subscription<rinbo_msgs::msg::Header>::SharedPtr rearm_ack_sub_;
    rclcpp::Subscription<rinbo_msgs::msg::Header>::SharedPtr active_ack_sub_;
};

}  // namespace rinbo_fsm
