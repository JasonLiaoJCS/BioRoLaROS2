#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rinbo_msgs/msg/power_state_stamped.hpp"
#include "safety_invariants.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rinbo_fsm {

// Power-board safety shared by calibration and standing. Counters are advanced
// only by distinct /power/state callbacks, not by repeated motor callbacks.
class RinboPowerGuard {
public:
    explicit RinboPowerGuard(rclcpp::Node& node)
        : RinboPowerGuard(node, std::array<bool, 6>{}) {}

    RinboPowerGuard(
        rclcpp::Node& node,
        const std::array<bool, 6>& disabled_legs)
        : disabled_legs_(disabled_legs) {
        enabled_ = node.declare_parameter<bool>("safety.power_guard_enabled", true);
        stop_on_stale_ = node.declare_parameter<bool>("safety.stop_on_power_stale", true);
        stop_on_voltage_ = node.declare_parameter<bool>("safety.stop_on_voltage", true);
        stop_on_leg_current_ = node.declare_parameter<bool>("safety.stop_on_current_limit", true);
        stop_on_bus_current_ = node.declare_parameter<bool>("safety.stop_on_bus_current_limit", false);
        require_power_relay_ = node.declare_parameter<bool>("safety.require_power_relay", true);
        bus_channel_ = node.declare_parameter<int>("safety.power_bus_voltage_channel", 7);
        min_bus_voltage_ = node.declare_parameter<double>("safety.min_bus_voltage", 18.0);
        max_bus_voltage_ = node.declare_parameter<double>("safety.max_bus_voltage", 30.0);
        max_leg_current_ = node.declare_parameter<double>("safety.max_current", 3.0);
        max_bus_current_ = node.declare_parameter<double>("safety.max_bus_current", 30.0);
        stale_seconds_ = node.declare_parameter<double>("safety.power_stale_seconds", 0.5);
        required_after_seconds_ = node.declare_parameter<double>("safety.power_required_after_seconds", 2.0);
        voltage_trip_samples_ = node.declare_parameter<int>("safety.voltage_trip_samples", 5);
        current_trip_samples_ = node.declare_parameter<int>("safety.current_trip_samples", 10);
        const auto channels = node.declare_parameter<std::vector<int64_t>>(
            "safety.leg_current_channels", std::vector<int64_t>{1, 2, 3, 4, 5, 6});

        if (bus_channel_ < 0 || bus_channel_ > 7 || channels.size() != 6 ||
            !std::isfinite(min_bus_voltage_) ||
            !std::isfinite(max_bus_voltage_) ||
            !std::isfinite(max_leg_current_) ||
            !std::isfinite(max_bus_current_) ||
            !std::isfinite(stale_seconds_) ||
            !std::isfinite(required_after_seconds_) ||
            min_bus_voltage_ <= 0.0 || max_bus_voltage_ <= min_bus_voltage_ ||
            max_leg_current_ <= 0.0 || max_bus_current_ <= 0.0 || stale_seconds_ <= 0.0 ||
            required_after_seconds_ <= 0.0 || voltage_trip_samples_ <= 0 ||
            current_trip_samples_ <= 0) {
            throw std::invalid_argument("Invalid Rinbo power safety parameter range");
        }
        for (std::size_t i = 0; i < channels.size(); ++i) {
            if (channels[i] < 0 || channels[i] > 7) {
                throw std::invalid_argument(
                    "safety.leg_current_channels entries must be in [0, 7]");
            }
            if (channels[i] == bus_channel_) {
                throw std::invalid_argument(
                    "safety.leg_current_channels must not include the bus channel");
            }
            for (std::size_t j = 0; j < i; ++j) {
                if (channels[i] == channels[j]) {
                    throw std::invalid_argument(
                        "safety.leg_current_channels entries must be unique");
                }
            }
            leg_current_channels_[i] = static_cast<int>(channels[i]);
        }

        const PowerSafetyContract contract {
            enabled_,
            stop_on_stale_,
            stop_on_voltage_,
            stop_on_voltage_,
            stop_on_leg_current_,
            require_power_relay_,
            bus_channel_,
            leg_current_channels_,
            min_bus_voltage_,
            max_bus_voltage_,
            max_leg_current_,
            max_bus_current_,
            stale_seconds_,
            required_after_seconds_,
            voltage_trip_samples_,
            current_trip_samples_};
        validate_power_safety_contract(contract, "Calibration/standing");
    }

    void update(const rinbo_msgs::msg::PowerStateStamped& msg, double now_s) {
        if (received_ && now_s < last_update_s_) clock_rollback_detected_ = true;
        received_ = true;
        last_update_s_ = now_s;
        const std::array<double, 8> voltages = {
            msg.v_0, msg.v_1, msg.v_2, msg.v_3,
            msg.v_4, msg.v_5, msg.v_6, msg.v_7
        };
        const std::array<double, 8> currents = {
            msg.i_0, msg.i_1, msg.i_2, msg.i_3,
            msg.i_4, msg.i_5, msg.i_6, msg.i_7
        };
        relay_power_on_ = msg.power;
        valid_ = std::isfinite(voltages[bus_channel_]);
        for (std::size_t leg = 0; leg < leg_current_channels_.size(); ++leg) {
            if (disabled_legs_[leg]) continue;
            valid_ = valid_ && std::isfinite(currents[leg_current_channels_[leg]]);
        }
        if (stop_on_bus_current_) {
            valid_ = valid_ && std::isfinite(currents[bus_channel_]);
        }
        bus_voltage_ = voltages[bus_channel_];
        bus_current_ = std::fabs(currents[bus_channel_]);
        max_leg_current_observed_ = 0.0;
        for (std::size_t leg = 0; leg < leg_current_channels_.size(); ++leg) {
            if (disabled_legs_[leg]) continue;
            const int channel = leg_current_channels_[leg];
            max_leg_current_observed_ = std::max(
                max_leg_current_observed_, std::fabs(currents[channel]));
        }

        // Fresh, valid relay-off telemetry is a safe pre-start WAIT state.
        // Voltage/current trips become meaningful when the relay is on, and
        // remain armed after output has once reached a fully safe snapshot.
        const bool evaluate_limits =
            valid_ && (!require_power_relay_ || relay_power_on_ || armed_once_);
        undervoltage_count_ = evaluate_limits && bus_voltage_ < min_bus_voltage_
            ? undervoltage_count_ + 1 : 0;
        overvoltage_count_ = evaluate_limits && bus_voltage_ > max_bus_voltage_
            ? overvoltage_count_ + 1 : 0;
        leg_current_count_ = evaluate_limits &&
            max_leg_current_observed_ > max_leg_current_
            ? leg_current_count_ + 1 : 0;
        bus_current_count_ = evaluate_limits && bus_current_ > max_bus_current_
            ? bus_current_count_ + 1 : 0;

        const bool raw_safe = valid_ && (!require_power_relay_ || relay_power_on_) &&
            (!stop_on_voltage_ ||
                (bus_voltage_ >= min_bus_voltage_ && bus_voltage_ <= max_bus_voltage_)) &&
            (!stop_on_leg_current_ || max_leg_current_observed_ <= max_leg_current_) &&
            (!stop_on_bus_current_ || bus_current_ <= max_bus_current_);
        armed_once_ = armed_once_ || raw_safe;
    }

    std::optional<std::string> violation(double now_s, double node_start_s) const {
        if (!enabled_) return std::nullopt;
        if (clock_rollback_detected_ || now_s < node_start_s ||
            (received_ && now_s < last_update_s_)) {
            return "power safety clock moved backward";
        }
        if (!received_) {
            if (stop_on_stale_ && now_s - node_start_s > required_after_seconds_) {
                return "no /power/state received";
            }
            return std::nullopt;
        }
        if (stop_on_stale_ && now_s - last_update_s_ > stale_seconds_) {
            std::ostringstream out;
            out << "/power/state stale for " << now_s - last_update_s_ << "s";
            return out.str();
        }
        if (require_power_relay_ && !relay_power_on_ && armed_once_) {
            return "power relay is off after output was armed";
        }
        if (!valid_) return "invalid NaN/Inf in /power/state";
        if (stop_on_voltage_ && undervoltage_count_ >= voltage_trip_samples_) {
            std::ostringstream out;
            out << "bus undervoltage ch" << bus_channel_ << "=" << bus_voltage_
                << "V threshold=" << min_bus_voltage_ << "V";
            return out.str();
        }
        if (stop_on_voltage_ && overvoltage_count_ >= voltage_trip_samples_) {
            std::ostringstream out;
            out << "bus overvoltage ch" << bus_channel_ << "=" << bus_voltage_
                << "V threshold=" << max_bus_voltage_ << "V";
            return out.str();
        }
        if (stop_on_leg_current_ && leg_current_count_ >= current_trip_samples_) {
            std::ostringstream out;
            out << "leg current=" << max_leg_current_observed_
                << "A threshold=" << max_leg_current_ << "A";
            return out.str();
        }
        if (stop_on_bus_current_ && bus_current_count_ >= current_trip_samples_) {
            std::ostringstream out;
            out << "bus current=" << bus_current_
                << "A threshold=" << max_bus_current_ << "A";
            return out.str();
        }
        return std::nullopt;
    }

    bool ready_for_output(double now_s) const {
        if (!enabled_) return true;
        if (clock_rollback_detected_ || (received_ && now_s < last_update_s_)) {
            return false;
        }
        const bool needs_snapshot =
            require_power_relay_ || stop_on_stale_ || stop_on_voltage_ ||
            stop_on_leg_current_ || stop_on_bus_current_;
        if (!needs_snapshot) return true;
        if (!received_ || !valid_) return false;
        if (require_power_relay_ && !relay_power_on_) return false;
        if (stop_on_stale_ && now_s - last_update_s_ > stale_seconds_) return false;
        if (stop_on_voltage_ &&
            (bus_voltage_ < min_bus_voltage_ || bus_voltage_ > max_bus_voltage_)) {
            return false;
        }
        if (stop_on_leg_current_ && max_leg_current_observed_ > max_leg_current_) {
            return false;
        }
        if (stop_on_bus_current_ && bus_current_ > max_bus_current_) return false;
        return true;
    }

private:
    bool enabled_ = true;
    bool stop_on_stale_ = true;
    bool stop_on_voltage_ = true;
    bool stop_on_leg_current_ = true;
    bool stop_on_bus_current_ = false;
    bool require_power_relay_ = true;
    bool received_ = false;
    bool valid_ = false;
    bool relay_power_on_ = false;
    bool armed_once_ = false;
    bool clock_rollback_detected_ = false;
    int bus_channel_ = 7;
    std::array<int, 6> leg_current_channels_ {1, 2, 3, 4, 5, 6};
    std::array<bool, 6> disabled_legs_ {};
    double min_bus_voltage_ = 18.0;
    double max_bus_voltage_ = 30.0;
    double max_leg_current_ = 3.0;
    double max_bus_current_ = 30.0;
    double stale_seconds_ = 0.5;
    double required_after_seconds_ = 2.0;
    int voltage_trip_samples_ = 5;
    int current_trip_samples_ = 10;
    int undervoltage_count_ = 0;
    int overvoltage_count_ = 0;
    int leg_current_count_ = 0;
    int bus_current_count_ = 0;
    double last_update_s_ = 0.0;
    double bus_voltage_ = 0.0;
    double bus_current_ = 0.0;
    double max_leg_current_observed_ = 0.0;
};

}  // namespace rinbo_fsm
