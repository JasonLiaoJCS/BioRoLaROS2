#pragma once

#include "rclcpp/message_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "safety_invariants.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <functional>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace rinbo_fsm {

// Pins a safety-critical input to one ROS publisher and rejects replayed or
// out-of-order source headers. Callbacks must validate before refreshing their
// local arrival-time watchdog.
class RosInputGuard {
public:
    struct Validation {
        int64_t observed_ns = 0;
        int64_t validated_ns = 0;
        std::optional<std::string> violation;

        bool fresh_at_validation(double timeout_s) const {
            return !violation && std::isfinite(timeout_s) && timeout_s > 0.0 &&
                validated_ns >= observed_ns &&
                static_cast<double>(validated_ns - observed_ns) * 1.0e-9 <= timeout_s;
        }
    };

    struct TestHooks {
        // These hooks only make ROS-time and graph-query latency deterministic
        // in tests.  They do not replace the production publisher query.
        std::function<int64_t()> now_ns;
        std::function<void()> before_publisher_query;
    };

    RosInputGuard(rclcpp::Node& node, std::string topic)
        : RosInputGuard(node, std::move(topic), TestHooks {}) {}

    RosInputGuard(rclcpp::Node& node, std::string topic, TestHooks test_hooks)
        : node_(node),
          topic_(std::move(topic)),
          now_ns_(std::move(test_hooks.now_ns)),
          before_publisher_query_(std::move(test_hooks.before_publisher_query)) {
        if (!now_ns_) {
            now_ns_ = [this]() { return node_.now().nanoseconds(); };
        }
        const bool motor_topic = topic_.find("motor") != std::string::npos;
        const std::string parameter = motor_topic
            ? "safety.motor_state_source_max_age_s"
            : "safety.power_state_source_max_age_s";
        const double hard_max_age_s = motor_topic
            ? kHardMaxMotorSourceAgeS
            : kHardMaxPowerSourceAgeS;
        max_age_s_ = node_.declare_parameter<double>(parameter, hard_max_age_s);
        if (!std::isfinite(max_age_s_) || max_age_s_ <= 0.0 ||
            max_age_s_ > hard_max_age_s) {
            throw std::invalid_argument(
                parameter + " must be positive, finite, and no greater than " +
                std::to_string(hard_max_age_s));
        }
    }

    template<typename Header>
    Validation accept(
        const Header& header,
        const rclcpp::MessageInfo& message_info) {
        // Capture ROS time at callback entry.  Publisher graph inspection may
        // block while DDS discovery converges; its own latency is not source
        // transport age and must not make a fresh sample appear stale.
        const int64_t observed_ns = now_ns_();
        if (const auto reason = publisher_violation(received_gid(message_info))) {
            return {observed_ns, observed_ns, reason};
        }
        // A backward ROS-time jump during validation is immediately unsafe.
        // A forward jump is deliberately not used for source age: otherwise
        // graph-query latency would once again be charged to the message.
        const int64_t validated_ns = now_ns_();
        if (validated_ns < observed_ns) {
            return {
                observed_ns, validated_ns,
                topic_ + " safety clock moved backward during input validation"};
        }
        if (header.stamp.nanosec >= 1000000000U) {
            return {
                observed_ns, validated_ns,
                topic_ + " source stamp.nanosec is outside [0,1e9)"};
        }
        const int64_t stamp_ns =
            static_cast<int64_t>(header.stamp.sec) * 1000000000LL +
            static_cast<int64_t>(header.stamp.nanosec);
        if (stamp_ns <= 0) {
            return {
                observed_ns, validated_ns,
                topic_ + " source stamp must be positive"};
        }
        const int64_t age_ns = observed_ns - stamp_ns;
        if (std::llabs(age_ns) > static_cast<int64_t>(max_age_s_ * 1.0e9)) {
            std::ostringstream out;
            out << topic_ << " source stamp age " << static_cast<double>(age_ns) * 1.0e-9
                << "s exceeds +/-" << max_age_s_ << "s";
            return {observed_ns, validated_ns, out.str()};
        }
        if (header_seen_) {
            if (stamp_ns <= last_stamp_ns_) {
                return {
                    observed_ns, validated_ns,
                    topic_ + " source stamp is duplicate or non-monotonic"};
            }
            if (!sequence_is_newer(header.seq, last_sequence_)) {
                return {
                    observed_ns, validated_ns,
                    topic_ + " source sequence is duplicate or out-of-order"};
            }
        }
        header_seen_ = true;
        last_stamp_ns_ = stamp_ns;
        last_sequence_ = header.seq;
        return {observed_ns, validated_ns, std::nullopt};
    }

    std::optional<std::string> publisher_violation() {
        return publisher_violation(std::nullopt);
    }

private:
    using PublisherGid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

    static PublisherGid received_gid(const rclcpp::MessageInfo& message_info) {
        PublisherGid result {};
        const auto& gid = message_info.get_rmw_message_info().publisher_gid;
        std::copy_n(gid.data, result.size(), result.begin());
        return result;
    }

    std::optional<std::string> publisher_violation(
        const std::optional<PublisherGid>& callback_gid) {
        try {
            if (before_publisher_query_) before_publisher_query_();
            const auto publishers = node_.get_publishers_info_by_topic(topic_);
            if (publishers.size() != 1U) {
                std::ostringstream out;
                out << topic_ << " requires exactly one publisher, got " << publishers.size();
                return out.str();
            }
            const auto& publisher = publishers.front();
            if (publisher.node_name() != kExpectedPublisherNodeName ||
                publisher.node_namespace() != kExpectedPublisherNodeNamespace) {
                return topic_ + " sole publisher is " + publisher.node_namespace() +
                    publisher.node_name() + "; expected /" + kExpectedPublisherNodeName;
            }
            const PublisherGid gid = publisher.endpoint_gid();
            if (callback_gid && *callback_gid != gid) {
                return topic_ +
                    " callback publisher GID does not match the sole graph endpoint";
            }
            if (publisher_seen_ && gid != publisher_gid_) {
                return topic_ + " sole publisher changed during an active FSM run";
            }
            publisher_gid_ = gid;
            publisher_seen_ = true;
            return std::nullopt;
        } catch (const std::exception& exc) {
            return topic_ + " publisher graph query failed: " + exc.what();
        }
    }

    static bool sequence_is_newer(uint32_t sequence, uint32_t previous) {
        const uint32_t delta = sequence - previous;
        return delta != 0U && delta < 0x80000000U;
    }

    rclcpp::Node& node_;
    std::string topic_;
    std::function<int64_t()> now_ns_;
    std::function<void()> before_publisher_query_;
    static constexpr const char* kExpectedPublisherNodeName = "rinbo_ros2_bridge";
    static constexpr const char* kExpectedPublisherNodeNamespace = "/";
    bool publisher_seen_ = false;
    PublisherGid publisher_gid_ {};
    bool header_seen_ = false;
    uint32_t last_sequence_ = 0;
    int64_t last_stamp_ns_ = 0;
    double max_age_s_ = 0.10;
};

}  // namespace rinbo_fsm
