#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace rinbo_fsm {

// These are hard deployment limits, not tunable defaults.  Site YAML may make
// a guard stricter, but it may not disable it, delay it, or remap telemetry so
// that a healthy leg is no longer supervised.
inline constexpr double kHardMinBusVoltageV = 18.0;
inline constexpr double kHardMaxBusVoltageV = 42.0;
inline constexpr double kHardMaxLegCurrentA = 10.0;
inline constexpr double kHardMaxBusCurrentA = 30.0;
inline constexpr double kHardMaxPowerStaleS = 0.5;
inline constexpr double kHardMaxPowerRequiredAfterS = 2.0;
inline constexpr int kHardMaxVoltageTripSamples = 5;
inline constexpr int kHardMaxCurrentTripSamples = 100;
inline constexpr int kPowerBusVoltageChannel = 7;
inline constexpr std::array<int, 6> kLegCurrentChannels = {1, 2, 3, 4, 5, 6};

inline constexpr double kHardMaxMotorStateStaleS = 0.25;
inline constexpr double kHardMaxMotorStateRequiredAfterS = 2.0;
inline constexpr double kHardMaxMotorSourceAgeS = 0.10;
inline constexpr double kHardMaxPowerSourceAgeS = 0.35;
inline constexpr double kHardMaxPwm = 80.0;
inline constexpr double kHardMaxMotorArbiterReadyTimeoutS = 5.0;
inline constexpr double kHardMaxMotorArbiterHeartbeatStaleS = 0.25;
inline constexpr double kHardMaxMotorCommandAckTimeoutS = 0.25;

inline constexpr double kHardMaxPositionErrorCounts = 12000.0;
inline constexpr double kHardMaxPwmSlewRatePerS = 250.0;
inline constexpr int kHardMaxPositionErrorTripSamples = 10;

struct PowerSafetyContract {
    bool enabled = false;
    bool stop_on_power_stale = false;
    bool stop_on_voltage_sag = false;
    bool stop_on_over_voltage = false;
    bool stop_on_current_limit = false;
    bool require_power_relay = false;
    int bus_voltage_channel = -1;
    std::array<int, 6> leg_current_channels {};
    double min_bus_voltage = 0.0;
    double max_bus_voltage = 0.0;
    double max_leg_current = 0.0;
    double max_bus_current = 0.0;
    double stale_seconds = 0.0;
    double required_after_seconds = 0.0;
    int voltage_trip_samples = 0;
    int current_trip_samples = 0;
};

inline void validate_power_safety_contract(
    const PowerSafetyContract& safety,
    const std::string& owner) {
    if (!safety.enabled || !safety.stop_on_power_stale ||
        !safety.stop_on_voltage_sag || !safety.stop_on_over_voltage ||
        !safety.stop_on_current_limit || !safety.require_power_relay) {
        throw std::invalid_argument(
            owner + " core power guards must remain enabled (stale, voltage, "
            "leg current, and relay acknowledgement)");
    }
    if (safety.bus_voltage_channel != kPowerBusVoltageChannel ||
        safety.leg_current_channels != kLegCurrentChannels) {
        throw std::invalid_argument(
            owner + " power telemetry mapping is fixed at bus=v7 and "
            "leg currents=[i1,i2,i3,i4,i5,i6]");
    }
    if (!std::isfinite(safety.min_bus_voltage) ||
        !std::isfinite(safety.max_bus_voltage) ||
        !std::isfinite(safety.max_leg_current) ||
        !std::isfinite(safety.max_bus_current) ||
        !std::isfinite(safety.stale_seconds) ||
        !std::isfinite(safety.required_after_seconds) ||
        safety.min_bus_voltage < kHardMinBusVoltageV ||
        safety.max_bus_voltage > kHardMaxBusVoltageV ||
        safety.max_bus_voltage <= safety.min_bus_voltage ||
        safety.max_leg_current <= 0.0 ||
        safety.max_leg_current > kHardMaxLegCurrentA ||
        safety.max_bus_current <= 0.0 ||
        safety.max_bus_current > kHardMaxBusCurrentA ||
        safety.stale_seconds <= 0.0 ||
        safety.stale_seconds > kHardMaxPowerStaleS ||
        safety.required_after_seconds <= 0.0 ||
        safety.required_after_seconds > kHardMaxPowerRequiredAfterS ||
        safety.voltage_trip_samples <= 0 ||
        safety.voltage_trip_samples > kHardMaxVoltageTripSamples ||
        safety.current_trip_samples <= 0 ||
        safety.current_trip_samples > kHardMaxCurrentTripSamples) {
        throw std::invalid_argument(
            owner + " exceeds hard power safety bounds: minV>=18, maxV<=42, "
            "leg current<=10A, bus current<=30A, stale<=0.5s, required<=2s, "
            "voltage samples<=5, current samples<=100");
    }
}

inline void validate_motor_state_watchdog(
    double stale_seconds,
    double required_after_seconds,
    const std::string& owner) {
    if (!std::isfinite(stale_seconds) ||
        !std::isfinite(required_after_seconds) ||
        stale_seconds <= 0.0 || stale_seconds > kHardMaxMotorStateStaleS ||
        required_after_seconds <= 0.0 ||
        required_after_seconds > kHardMaxMotorStateRequiredAfterS) {
        throw std::invalid_argument(
            owner + " motor-state watchdog must satisfy stale<=0.25s and "
            "required-after<=2s");
    }
}

inline void validate_pwm(double max_pwm, const std::string& owner) {
    if (!std::isfinite(max_pwm) || max_pwm <= 0.0 || max_pwm > kHardMaxPwm) {
        throw std::invalid_argument(owner + " max_pwm must be finite and in (0,80]");
    }
}

inline void validate_motor_arbiter_contract(
    const std::string& expected_node_name,
    double ready_timeout_seconds,
    double heartbeat_stale_seconds,
    double command_ack_timeout_seconds) {
    if (expected_node_name != "rinbo_ros2_bridge") {
        throw std::invalid_argument(
            "safety.motor_arbiter_node_name is fixed at rinbo_ros2_bridge");
    }
    if (!std::isfinite(ready_timeout_seconds) ||
        !std::isfinite(heartbeat_stale_seconds) ||
        !std::isfinite(command_ack_timeout_seconds) ||
        ready_timeout_seconds <= 0.0 ||
        ready_timeout_seconds > kHardMaxMotorArbiterReadyTimeoutS ||
        heartbeat_stale_seconds <= 0.0 ||
        heartbeat_stale_seconds > kHardMaxMotorArbiterHeartbeatStaleS ||
        command_ack_timeout_seconds <= 0.0 ||
        command_ack_timeout_seconds > kHardMaxMotorCommandAckTimeoutS) {
        throw std::invalid_argument(
            "motor arbiter requires ready timeout<=5s, heartbeat stale<=0.25s, "
            "and command ACK timeout<=0.25s");
    }
}

inline void validate_bounded_timeout(
    double value,
    double hard_maximum,
    const std::string& parameter) {
    if (!std::isfinite(value) || value <= 0.0 || value > hard_maximum) {
        throw std::invalid_argument(
            parameter + " must be finite and in (0," +
            std::to_string(hard_maximum) + "]");
    }
}

struct TripodMotionSafetyContract {
    bool stop_on_position_error = false;
    bool enable_pwm_slew_limit = false;
    double max_position_error_counts = 0.0;
    double pwm_slew_rate_per_second = 0.0;
    double motor_state_stale_seconds = 0.0;
    double motor_state_required_after_seconds = 0.0;
    int position_error_trip_samples = 0;
    double position_error_trip_seconds = 0.5;
};

inline void validate_tripod_motion_safety_contract(
    const TripodMotionSafetyContract& safety) {
    if (!safety.stop_on_position_error || !safety.enable_pwm_slew_limit) {
        throw std::invalid_argument(
            "Tripod position-error and PWM-slew guards must remain enabled");
    }
    validate_bounded_timeout(safety.position_error_trip_seconds, 2.0,
                             "safety.position_error_trip_seconds");
    validate_motor_state_watchdog(
        safety.motor_state_stale_seconds,
        safety.motor_state_required_after_seconds,
        "Tripod");
    if (!std::isfinite(safety.max_position_error_counts) ||
        !std::isfinite(safety.pwm_slew_rate_per_second) ||
        safety.max_position_error_counts <= 0.0 ||
        safety.max_position_error_counts > kHardMaxPositionErrorCounts ||
        safety.pwm_slew_rate_per_second <= 0.0 ||
        safety.pwm_slew_rate_per_second > kHardMaxPwmSlewRatePerS ||
        safety.position_error_trip_samples <= 0 ||
        safety.position_error_trip_samples > kHardMaxPositionErrorTripSamples) {
        throw std::invalid_argument(
            "Tripod hard motion bounds require position error<=12000 counts, "
            "PWM slew<=250/s, and position-error samples<=10");
    }
}

}  // namespace rinbo_fsm
