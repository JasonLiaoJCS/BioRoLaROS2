#pragma once

#include "rclcpp/rclcpp.hpp"

#include <array>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>

namespace rinbo_fsm {

enum class BridgeInputDiscovery { Pending, Ready };

// Only incomplete attribution can wait. A known wrong name/namespace or a
// second publisher is an error, even during preparation. This grants no trust
// to a callback: RosInputGuard still validates its GID and fresh source header.
inline BridgeInputDiscovery bridge_input_discovery(
    std::size_t count, const std::string& name, const std::string& ns,
    const std::string& topic) {
    if (count == 0) return BridgeInputDiscovery::Pending;
    if (count != 1) {
        throw std::runtime_error(topic + " requires exactly one publisher, got " +
                                 std::to_string(count));
    }
    const bool unknown_name = name == "_NODE_NAME_UNKNOWN_";
    const bool unknown_ns = ns == "_NODE_NAMESPACE_UNKNOWN_" ||
                            ns == "/_NODE_NAMESPACE_UNKNOWN_";
    if ((!unknown_name && name != "rinbo_ros2_bridge") ||
        (!unknown_ns && ns != "/")) {
        throw std::runtime_error(topic + " sole publisher is " + ns + name +
                                 "; expected /rinbo_ros2_bridge");
    }
    return unknown_name || unknown_ns ? BridgeInputDiscovery::Pending
                                      : BridgeInputDiscovery::Ready;
}

// Call on the actual FSM node BEFORE creating command publishers, watchdog
// clocks, or the arbiter handshake. DDS graph discovery has its own bounded
// preparation phase; it must not consume a running motor's 2 s startup budget.
// No subscriptions, command packets, receipt completion or parameter changes.
inline void wait_for_bridge_inputs(
    rclcpp::Node& node, const std::function<bool()>& cancelled = [] { return false; },
    std::chrono::milliseconds budget = std::chrono::seconds(8)) {
    using Clock = std::chrono::steady_clock;
    if (budget <= std::chrono::milliseconds::zero() || budget > std::chrono::seconds(8))
        throw std::invalid_argument("Bridge discovery budget must be in (0,8s]");
    const auto deadline = Clock::now() + budget;
    auto next_notice = Clock::now();
    std::string pending;
    while (rclcpp::ok(node.get_node_base_interface()->get_context()) && !cancelled()) {
        bool ready = true;
        pending.clear();
        for (const char* topic : {"/motor/state", "/power/state"}) {
            const auto endpoints = node.get_publishers_info_by_topic(topic);
            const auto state = bridge_input_discovery(
                endpoints.size(), endpoints.size() == 1 ? endpoints.front().node_name() : "",
                endpoints.size() == 1 ? endpoints.front().node_namespace() : "", topic);
            if (state == BridgeInputDiscovery::Pending) {
                ready = false;
                pending += std::string(topic) + " ";
            }
        }
        if (Clock::now() >= deadline) break;
        if (ready) {
            RCLCPP_INFO(node.get_logger(),
                "DISCOVERY READY: Bridge input identities resolved; fresh-input and arbiter checks follow");
            return;
        }
        if (Clock::now() >= next_notice) {
            RCLCPP_INFO(node.get_logger(),
                "DISCOVERY WAIT: waiting for %s identity; no motor command publisher yet",
                pending.c_str());
            next_notice = Clock::now() + std::chrono::seconds(1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!rclcpp::ok(node.get_node_base_interface()->get_context()) || cancelled())
        throw std::runtime_error("Bridge discovery cancelled before motor commands");
    throw std::runtime_error("Bridge discovery timed out before motor commands: " + pending);
}

}  // namespace rinbo_fsm
