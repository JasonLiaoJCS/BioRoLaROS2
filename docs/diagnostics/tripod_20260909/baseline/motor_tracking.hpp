#pragma once

#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace rinbo_fsm {
// Calibration/standing positive travel is decreasing raw counts on the left.
// This detects travel in the wrong direction; it does not guess whether the
// motor wiring or the encoder polarity is responsible.
inline double forward_travel(int leg, double origin, double raw) {
    return (leg < 3 ? -1.0 : 1.0) * (raw - origin);
}
inline constexpr double kWrongWayTravelCounts = 500.0;

struct ForwardReference { double position; double velocity; };
// Gradually start searching; keep the existing cruise speed and PWM cap.
inline ForwardReference search_reference(double elapsed, double speed) {
    const double t = std::max(0.0, elapsed);
    constexpr double ramp = 0.5;
    if (t < ramp) return {0.5 * speed * t*t/ramp, speed*t/ramp};
    return {speed*(t-ramp/2), speed};
}
inline double standing_duration(double distance, double speed) {
    return distance/speed + std::min(0.5, distance/speed);
}
// Accelerate, cruise, decelerate: no velocity step to zero at the endpoint.
inline ForwardReference standing_reference(double elapsed, double distance, double speed) {
    const double ramp = std::min(0.5, distance/speed);
    const double end = standing_duration(distance,speed);
    const double t = std::max(0.0, elapsed);
    if (t >= end) return {distance, 0};
    if (t < ramp) return {0.5*speed*t*t/ramp, speed*t/ramp};
    if (t <= end-ramp) return {speed*(t-ramp/2), speed};
    const double remaining = end-t;
    return {distance-0.5*speed*remaining*remaining/ramp, speed*remaining/ramp};
}

inline std::string tracking_trace(const std::array<float, 6>& positions,
                                 const std::array<float, 6>& targets,
                                 const std::array<float, 6>& velocities,
                                 const rinbo_msgs::msg::MotorCmdStamped& cmd) {
    static constexpr const char* names[] = {"L1", "L2", "L3", "R1", "R2", "R3"};
    const std::array<const rinbo_msgs::msg::LegCmd*, 6> legs{
        &cmd.l1, &cmd.l2, &cmd.l3, &cmd.r1, &cmd.r2, &cmd.r3};
    std::ostringstream out;
    out << std::fixed << std::setprecision(0);
    for (int i = 0; i < 6; ++i) {
        const auto& leg = *legs[i];
        const double signed_pwm = (leg.direction == (i >= 3) ? 1.0 : -1.0) * leg.voltage;
        out << (i ? " | " : "") << names[i] << ": raw=" << positions[i]
            << " target_raw=" << targets[i] << " velocity=" << velocities[i]
            << " pwm=" << signed_pwm << " enable=" << leg.enable << " dir=" << leg.direction;
    }
    return out.str();
}
}  // namespace rinbo_fsm
