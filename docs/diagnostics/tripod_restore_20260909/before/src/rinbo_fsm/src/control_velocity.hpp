#pragma once

#include <cmath>

namespace rinbo_fsm {
// Smooth only the measured-velocity term used for damping. Raw encoder values
// and raw velocity MUST remain available for stop/overspeed/settling checks.
// A time-based coefficient avoids changing the filter bandwidth with DDS rate.
class ControlVelocity {
public:
    double update(double raw, double dt) {
        if (!std::isfinite(raw) || !std::isfinite(dt) || dt <= 0.0) {
            reset();
            return raw;  // Do not conceal invalid input from the caller.
        }
        if (!initialized_ || dt > 0.25) {
            value_ = raw;
            initialized_ = true;
        } else {
            constexpr double time_constant_s = 0.02;
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
