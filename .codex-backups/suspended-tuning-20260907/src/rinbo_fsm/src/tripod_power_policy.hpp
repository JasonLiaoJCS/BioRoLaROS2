#pragma once

#include <cstdint>
#include <optional>
#include <sstream>
#include <string>

namespace rinbo_fsm {

// Pure state machine for Tripod's power-sample counters and arrival watchdog.
// ROS publisher/header authentication remains the caller's responsibility.
struct TripodPowerPolicyConfig {
    bool enabled = true;
    bool stop_on_voltage_sag = true;
    bool stop_on_over_voltage = true;
    bool stop_on_current_limit = true;
    bool stop_on_bus_current_limit = false;
    bool require_power_relay = true;
    bool stop_on_power_stale = true;
    double min_bus_voltage = 18.0;
    double max_bus_voltage = 30.0;
    double max_leg_current = 3.0;
    double max_bus_current = 30.0;
    double stale_seconds = 0.5;
    double required_after_seconds = 2.0;
    int bus_voltage_channel = 7;
    int voltage_trip_samples = 5;
    int current_trip_samples = 10;
};

struct TripodPowerObservation {
    int64_t observed_ns = 0;
    bool relay_power_on = false;
    bool valid = false;
    double bus_voltage = 0.0;
    double max_leg_current = 0.0;
    double bus_current = 0.0;
};

class TripodPowerPolicy {
public:
    explicit TripodPowerPolicy(TripodPowerPolicyConfig config)
        : config_(config) {}

    void observe(const TripodPowerObservation& sample) {
        if (received_ && sample.observed_ns < last_observed_ns_) {
            clock_rollback_detected_ = true;
        }
        received_ = true;
        last_observed_ns_ = sample.observed_ns;
        relay_power_on_ = sample.relay_power_on;
        valid_ = sample.valid;
        bus_voltage_ = sample.bus_voltage;
        max_leg_current_ = sample.max_leg_current;
        bus_current_ = sample.bus_current;

        const bool evaluate_limits = valid_ &&
            (!config_.require_power_relay || relay_power_on_ || output_armed_);
        undervoltage_count_ = evaluate_limits &&
            bus_voltage_ < config_.min_bus_voltage ? undervoltage_count_ + 1 : 0;
        overvoltage_count_ = evaluate_limits &&
            bus_voltage_ > config_.max_bus_voltage ? overvoltage_count_ + 1 : 0;
        leg_current_trip_count_ = evaluate_limits &&
            max_leg_current_ > config_.max_leg_current
            ? leg_current_trip_count_ + 1 : 0;
        bus_current_trip_count_ = evaluate_limits &&
            bus_current_ > config_.max_bus_current
            ? bus_current_trip_count_ + 1 : 0;
    }

    std::optional<std::string> violation(
        int64_t now_ns,
        int64_t node_start_ns) const {
        if (!config_.enabled) return std::nullopt;
        if (clock_rollback_detected_ || now_ns < node_start_ns ||
            (received_ && now_ns < last_observed_ns_)) {
            return "power safety clock moved backward";
        }
        if (!received_) {
            if (config_.stop_on_power_stale &&
                seconds_between(now_ns, node_start_ns) >
                    config_.required_after_seconds) {
                return "no /power/state received";
            }
            return std::nullopt;
        }

        const double age = seconds_between(now_ns, last_observed_ns_);
        if (config_.stop_on_power_stale && age > config_.stale_seconds) {
            std::ostringstream reason;
            reason << "/power/state stale for " << age << "s";
            return reason.str();
        }
        if (config_.require_power_relay && !relay_power_on_ && output_armed_) {
            return "power relay is off";
        }
        if (!valid_) return "invalid NaN/Inf in /power/state";
        if (config_.stop_on_voltage_sag &&
            undervoltage_count_ >= config_.voltage_trip_samples) {
            std::ostringstream reason;
            reason << "bus undervoltage ch" << config_.bus_voltage_channel << "="
                   << bus_voltage_
                   << "V threshold=" << config_.min_bus_voltage << "V";
            return reason.str();
        }
        if (config_.stop_on_over_voltage &&
            overvoltage_count_ >= config_.voltage_trip_samples) {
            std::ostringstream reason;
            reason << "bus overvoltage ch" << config_.bus_voltage_channel << "="
                   << bus_voltage_
                   << "V threshold=" << config_.max_bus_voltage << "V";
            return reason.str();
        }
        if (config_.stop_on_current_limit &&
            leg_current_trip_count_ >= config_.current_trip_samples) {
            std::ostringstream reason;
            reason << "leg current limit max=" << max_leg_current_
                   << "A threshold=" << config_.max_leg_current << "A";
            return reason.str();
        }
        if (config_.stop_on_bus_current_limit &&
            bus_current_trip_count_ >= config_.current_trip_samples) {
            std::ostringstream reason;
            reason << "bus current limit=" << bus_current_
                   << "A threshold=" << config_.max_bus_current << "A";
            return reason.str();
        }
        return std::nullopt;
    }

    bool ready_for_output(int64_t now_ns) {
        if (!config_.enabled) return true;
        if (clock_rollback_detected_ ||
            (received_ && now_ns < last_observed_ns_)) {
            return false;
        }
        const bool needs_snapshot = config_.require_power_relay ||
            config_.stop_on_power_stale || config_.stop_on_voltage_sag ||
            config_.stop_on_over_voltage || config_.stop_on_current_limit ||
            config_.stop_on_bus_current_limit;
        if (!needs_snapshot) return true;
        if (!received_ || !valid_) return false;
        if (config_.require_power_relay && !relay_power_on_) return false;
        if (config_.stop_on_power_stale &&
            seconds_between(now_ns, last_observed_ns_) > config_.stale_seconds) {
            return false;
        }
        if (config_.stop_on_voltage_sag &&
            bus_voltage_ < config_.min_bus_voltage) {
            return false;
        }
        if (config_.stop_on_over_voltage &&
            bus_voltage_ > config_.max_bus_voltage) {
            return false;
        }
        if (config_.stop_on_current_limit &&
            max_leg_current_ > config_.max_leg_current) {
            return false;
        }
        if (config_.stop_on_bus_current_limit &&
            bus_current_ > config_.max_bus_current) {
            return false;
        }
        output_armed_ = true;
        return true;
    }

    int leg_current_trip_count() const { return leg_current_trip_count_; }

private:
    static double seconds_between(int64_t later_ns, int64_t earlier_ns) {
        return static_cast<double>(later_ns - earlier_ns) * 1.0e-9;
    }

    TripodPowerPolicyConfig config_;
    bool received_ = false;
    bool relay_power_on_ = false;
    bool valid_ = false;
    bool output_armed_ = false;
    bool clock_rollback_detected_ = false;
    int64_t last_observed_ns_ = 0;
    double bus_voltage_ = 0.0;
    double max_leg_current_ = 0.0;
    double bus_current_ = 0.0;
    int undervoltage_count_ = 0;
    int overvoltage_count_ = 0;
    int leg_current_trip_count_ = 0;
    int bus_current_trip_count_ = 0;
};

}  // namespace rinbo_fsm
