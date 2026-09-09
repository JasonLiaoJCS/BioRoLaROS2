#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rinbo_fsm {

class DisabledLegs {
public:
    static constexpr std::array<const char*, 6> kLegNames = {
        "L1", "L2", "L3", "R1", "R2", "R3"
    };

    static DisabledLegs load(rclcpp::Node& node) {
        DisabledLegs result;
        rcl_interfaces::msg::ParameterDescriptor mask_descriptor;
        mask_descriptor.description =
            "Physical legs disabled for this process. Startup-only; stop the "
            "controller, update the site YAML, and restart to change it.";
        mask_descriptor.read_only = true;
        const auto requested = node.declare_parameter<std::vector<std::string>>(
            "hardware.disabled_legs", std::vector<std::string>{}, mask_descriptor);

        rcl_interfaces::msg::ParameterDescriptor limit_descriptor;
        limit_descriptor.description =
            "Maximum supported disabled legs. Fixed at one for hardware bringup.";
        limit_descriptor.read_only = true;
        const int max_disabled = node.declare_parameter<int>(
            "hardware.max_disabled_legs", 1, limit_descriptor);

        if (max_disabled != 1) {
            throw std::invalid_argument(
                "hardware.max_disabled_legs must be exactly 1 for hardware bringup"
            );
        }

        for (const auto& raw_name : requested) {
            const std::string name = canonical_name(raw_name);
            const auto it = std::find_if(
                kLegNames.begin(), kLegNames.end(),
                [&](const char* candidate) { return name == candidate; });
            if (it == kLegNames.end()) {
                throw std::invalid_argument(
                    "Unknown hardware.disabled_legs entry '" + raw_name +
                    "'; expected one of L1,L2,L3,R1,R2,R3");
            }
            const auto index = static_cast<std::size_t>(std::distance(kLegNames.begin(), it));
            if (result.mask_[index]) {
                throw std::invalid_argument(
                    "Duplicate hardware.disabled_legs entry '" + name + "'");
            }
            result.mask_[index] = true;
            result.names_.push_back(name);
        }

        if (static_cast<int>(result.names_.size()) > max_disabled) {
            std::ostringstream reason;
            reason << "hardware.disabled_legs has " << result.names_.size()
                   << " entries, exceeding hardware.max_disabled_legs=" << max_disabled;
            throw std::invalid_argument(reason.str());
        }
        return result;
    }

    bool contains(int index) const {
        return index >= 0 && index < 6 && mask_[static_cast<std::size_t>(index)];
    }

    int count() const {
        return static_cast<int>(names_.size());
    }

    int healthy_count() const {
        return 6 - count();
    }

    bool enabled() const {
        return !names_.empty();
    }

    const std::array<bool, 6>& mask() const {
        return mask_;
    }

    std::string summary() const {
        if (names_.empty()) {
            return "none (strict six-leg mode)";
        }
        std::ostringstream out;
        for (std::size_t i = 0; i < names_.size(); ++i) {
            if (i > 0) out << ",";
            out << names_[i];
        }
        return out.str();
    }

private:
    static std::string canonical_name(std::string value) {
        value.erase(
            std::remove_if(value.begin(), value.end(),
                           [](unsigned char ch) { return std::isspace(ch) != 0; }),
            value.end());
        std::transform(
            value.begin(), value.end(), value.begin(),
            [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
        return value;
    }

    std::array<bool, 6> mask_ {};
    std::vector<std::string> names_;
};

template<typename LegCommand>
inline void force_main_drive_disabled(LegCommand& leg) {
    leg.enable = false;
    leg.direction = false;
    leg.voltage = 0.0f;
    leg.state = 0;
    leg.reset_position = false;
}

// Final publish-boundary mask shared by every FSM.  Upstream state logic also
// avoids populating disabled-leg commands, but this last gate prevents a future
// branch or partially initialized message from bypassing the hardware mask.
template<typename MotorCommand>
inline void enforce_disabled_leg_commands(
    MotorCommand& command,
    const DisabledLegs& disabled_legs) {
    auto apply = [&](int index, auto& leg) {
        if (disabled_legs.contains(index)) force_main_drive_disabled(leg);
    };
    apply(0, command.l1);
    apply(1, command.l2);
    apply(2, command.l3);
    apply(3, command.r1);
    apply(4, command.r2);
    apply(5, command.r3);
}

}  // namespace rinbo_fsm
