#pragma once

#include "rclcpp/rclcpp.hpp"

namespace rinbo_fsm {

// Motor and power state are latest-value safety telemetry.  A stalled executor
// must resume from the newest sample instead of replaying an obsolete backlog.
inline rclcpp::QoS latest_state_qos() {
    return rclcpp::QoS(rclcpp::KeepLast(1))
        .reliable()
        .durability_volatile();
}

}  // namespace rinbo_fsm
