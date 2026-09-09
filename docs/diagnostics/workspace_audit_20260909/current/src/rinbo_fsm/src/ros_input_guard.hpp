#pragma once

#include "rclcpp/message_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "safety_invariants.hpp"

#include <algorithm>
#include <array>
#include <chrono>
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

// DDS can deliver a callback before the graph has attributed its endpoint.
// Waiting is allowed only before the first trusted publisher, never during
// an active run, and consumes the existing hard input-startup budget.
inline bool input_startup_wait_allowed(
    bool publisher_already_trusted, double elapsed_seconds) {
    return !publisher_already_trusted && std::isfinite(elapsed_seconds) &&
        elapsed_seconds >= 0.0 &&
        elapsed_seconds < kHardMaxMotorStateRequiredAfterS;
}

inline bool input_publisher_discovery_pending(
    std::size_t endpoint_count,
    const std::string& node_name,
    const std::string& node_namespace,
    bool callback_gid_matches,
    bool publisher_already_trusted,
    double elapsed_seconds) {
    if (!input_startup_wait_allowed(publisher_already_trusted, elapsed_seconds)) {
        return false;
    }
    if (endpoint_count == 0U) return true;
    return endpoint_count == 1U && callback_gid_matches &&
        node_name == "_NODE_NAME_UNKNOWN_" &&
        (node_namespace == "_NODE_NAMESPACE_UNKNOWN_" ||
         node_namespace == "/_NODE_NAMESPACE_UNKNOWN_");
}

// Pins a safety-critical input to one ROS publisher and rejects replayed or
// out-of-order source headers. Callbacks must validate before refreshing their
// local arrival-time watchdog.
class RosInputGuard {
public:
    struct Validation {
        int64_t observed_ns = 0;
        int64_t validated_ns = 0;
        std::optional<std::string> violation;
        bool pending_startup = false;

        bool fresh_at_validation(double timeout_s) const {
            return !pending_startup && !violation &&
                std::isfinite(timeout_s) && timeout_s > 0.0 &&
                validated_ns >= observed_ns &&
                static_cast<double>(validated_ns - observed_ns) * 1.0e-9 <= timeout_s;
        }
    };

    struct TestHooks {
        // These hooks only make ROS/steady time and graph latency deterministic
        // in tests.  They do not replace the production publisher query.
        std::function<int64_t()> now_ns;
        std::function<std::chrono::steady_clock::time_point()> steady_now;
        std::function<void()> before_publisher_query;
    };

    RosInputGuard(rclcpp::Node& node, std::string topic)
        : RosInputGuard(node, std::move(topic), TestHooks {}) {}

    RosInputGuard(rclcpp::Node& node, std::string topic, TestHooks test_hooks)
        : node_(node),
          topic_(std::move(topic)),
          now_ns_(std::move(test_hooks.now_ns)),
          steady_now_(std::move(test_hooks.steady_now)),
          before_publisher_query_(std::move(test_hooks.before_publisher_query)) {
        if (!now_ns_) {
            now_ns_ = [this]() { return node_.now().nanoseconds(); };
        }
        if (!steady_now_) {
            steady_now_ = []() { return std::chrono::steady_clock::now(); };
        }
        discovery_started_at_ = steady_now_();
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
        bool pending_discovery = false;
        PublisherGid validated_gid {};
        if (const auto reason = publisher_violation(
                received_gid(message_info), &pending_discovery, &validated_gid)) {
            return {observed_ns, observed_ns, reason};
        }
        if (pending_discovery) {
            // Do not accept headers, advance sequences, refresh watchdogs, or
            // permit output from an unattributed initial callback.
            return {observed_ns, observed_ns, std::nullopt, true};
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
            if (age_ns > 0 && input_startup_wait_allowed(
                    publisher_seen_, startup_elapsed_seconds())) {
                // A correctly attributed queued sample is not current state.
                // Discard it before first input; never replay it after waiting.
                RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                    "Waiting for fresh initial %s; past-stale sample ignored, output remains disabled",
                    topic_.c_str());
                return {observed_ns, validated_ns, std::nullopt, true};
            }
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
        // Commit publisher identity only together with a fully valid fresh
        // header. Discovery or old queued samples must not establish trust.
        publisher_gid_ = validated_gid;
        publisher_seen_ = true;
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
        const std::optional<PublisherGid>& callback_gid,
        bool* pending_discovery = nullptr,
        PublisherGid* validated_gid = nullptr) {
        try {
            if (before_publisher_query_) before_publisher_query_();
            const auto publishers = node_.get_publishers_info_by_topic(topic_);
            const double discovery_elapsed_s = startup_elapsed_seconds();
            if (pending_discovery && input_publisher_discovery_pending(
                    publishers.size(),
                    publishers.size() == 1U ? publishers.front().node_name() : "",
                    publishers.size() == 1U ? publishers.front().node_namespace() : "",
                    publishers.size() == 1U && callback_gid &&
                        *callback_gid == publishers.front().endpoint_gid(),
                    publisher_seen_, discovery_elapsed_s)) {
                *pending_discovery = true;
                RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                    "Waiting for initial %s publisher discovery; sample ignored, output remains disabled",
                    topic_.c_str());
                return std::nullopt;
            }
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
            if (validated_gid) *validated_gid = gid;
            return std::nullopt;
        } catch (const std::exception& exc) {
            return topic_ + " publisher graph query failed: " + exc.what();
        }
    }

    static bool sequence_is_newer(uint32_t sequence, uint32_t previous) {
        const uint32_t delta = sequence - previous;
        return delta != 0U && delta < 0x80000000U;
    }

    double startup_elapsed_seconds() const {
        return std::chrono::duration<double>(steady_now_() - discovery_started_at_).count();
    }

    rclcpp::Node& node_;
    std::string topic_;
    std::function<int64_t()> now_ns_;
    std::function<std::chrono::steady_clock::time_point()> steady_now_;
    std::function<void()> before_publisher_query_;
    static constexpr const char* kExpectedPublisherNodeName = "rinbo_ros2_bridge";
    static constexpr const char* kExpectedPublisherNodeNamespace = "/";
    bool publisher_seen_ = false;
    PublisherGid publisher_gid_ {};
    bool header_seen_ = false;
    uint32_t last_sequence_ = 0;
    int64_t last_stamp_ns_ = 0;
    double max_age_s_ = 0.10;
    std::chrono::steady_clock::time_point discovery_started_at_;
};

}  // namespace rinbo_fsm
