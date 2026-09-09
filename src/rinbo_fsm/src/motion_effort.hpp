#pragma once

#include <cmath>
#include <stdexcept>

namespace rinbo_fsm {
// Units: encoder counts, counts/s, and signed PWM command units.
// This is a feedforward + PD controller; it never enables a motor or bypasses
// the caller's final PWM cap, slew limit, or immediate disabled/stop path.
struct MotionEffort {
    double kp = 0.35;
    double kd = 0.002;
    double k_ff = 0.02;
    double friction_pwm = 0.0;
    double friction_velocity_counts_s = 153.6;  // About 1 deg/s.

    void validate() const {
        if (!std::isfinite(kp) || !std::isfinite(kd) || !std::isfinite(k_ff) ||
            !std::isfinite(friction_pwm) || !std::isfinite(friction_velocity_counts_s) ||
            kp < 0 || kp > 1 || kd < 0 || kd > 0.1 || k_ff < 0 || k_ff > 0.1 ||
            friction_pwm < 0 || friction_pwm > 80 || friction_velocity_counts_s <= 0) {
            throw std::invalid_argument(
                "Invalid motion effort: kp in [0,1], kd/k_ff in [0,0.1], "
                "friction_pwm in [0,80], friction_velocity_counts_s > 0; all finite");
        }
    }

    double feedforward(double target_velocity) const {
        // Use the reference direction, never the noisy measured velocity or
        // position error. The compensation fades continuously to zero at rest
        // and during reversals, with no fixed minimum-output clamp.
        return k_ff * target_velocity + friction_pwm *
            std::tanh(target_velocity / friction_velocity_counts_s);
    }

    double command(double position_error, double target_velocity,
                   double measured_control_velocity) const {
        return kp * position_error + kd * (target_velocity - measured_control_velocity)
            + feedforward(target_velocity);
    }
};

template<class Node>
MotionEffort load_motion_effort(Node& node, MotionEffort fallback = {}) {
    MotionEffort out{
        node.template declare_parameter<double>("kp", fallback.kp),
        node.template declare_parameter<double>("kd", fallback.kd),
        node.template declare_parameter<double>("k_ff", fallback.k_ff),
        node.template declare_parameter<double>("friction_pwm", fallback.friction_pwm),
        node.template declare_parameter<double>("friction_velocity_counts_s", fallback.friction_velocity_counts_s)};
    out.validate();
    return out;
}
}  // namespace rinbo_fsm
