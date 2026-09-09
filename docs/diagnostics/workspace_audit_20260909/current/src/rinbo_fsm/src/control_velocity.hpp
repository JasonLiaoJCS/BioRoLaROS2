#pragma once

#include <cmath>
#include <stdexcept>

namespace rinbo_fsm {
// Smooth only the measured-velocity term used for damping. Raw encoder values
// and raw velocity MUST remain available for stop/overspeed/settling checks.
// A time-based coefficient avoids changing the filter bandwidth with DDS rate.
class ControlVelocity {
public:
    // Existing controllers retain 20 ms. Tripod explicitly selects its own
    // constant; zero gives the historical raw derivative for comparison.
    double update(double raw, double dt, double time_constant_s = 0.02) {
        if (!std::isfinite(time_constant_s) || time_constant_s < 0.0)
            throw std::invalid_argument("velocity_filter_time_constant_s must be finite and >= 0");
        if (!std::isfinite(raw) || !std::isfinite(dt) || dt <= 0.0) {
            reset();
            return raw;  // Do not conceal invalid input from the caller.
        }
        if (!initialized_ || dt > 0.25 || time_constant_s == 0.0) {
            value_ = raw;
            initialized_ = true;
        } else {
            value_ += dt / (time_constant_s + dt) * (raw - value_);
        }
        return value_;
    }
    void reset() { initialized_ = false; value_ = 0.0; }
private:
    double value_ = 0.0;
    bool initialized_ = false;
};
}  // namespace rinbo_fsm
