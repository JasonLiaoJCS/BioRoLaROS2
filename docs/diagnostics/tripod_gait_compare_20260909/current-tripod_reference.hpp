#pragma once

#include <algorithm>
#include <cmath>

namespace rinbo_fsm::tripod {
// Historical Tripod coordinates, deliberately NOT shared with Standing.
// Physical direction and counts/revolution still require site verification.
inline double position(unsigned leg, double raw) { return leg < 3 ? raw : -raw; }
inline bool direction(unsigned leg, double pwm) { return leg < 3 ? pwm >= 0 : pwm < 0; }
constexpr double kCountsPerRevolution = 54984.83;
constexpr float kHardPositionError = 18000.0f;

struct Reference { double position; double velocity; };

// Rest-to-rest, with zero acceleration at both ends. Keeps unwrapped travel.
inline Reference startup(double t, double duration, double origin, double travel) {
    const double x = std::clamp(t / duration, 0.0, 1.0);
    const double s = x*x*x*(10 + x*(-15 + 6*x));
    const double ds = 30*x*x*(1-x)*(1-x);
    return {origin + travel*s, travel*ds/duration};
}

// Smoothly enter the existing phase trajectory from rest, then recover its
// exact phase (including all revolutions). Derivative is continuous at joins.
inline Reference launch_phase(double progress, double blend) {
    if (progress <= 0) return {0, 0};
    if (progress >= blend) return {progress, 1};
    const double x = progress/blend;
    return {blend*x*x*x*(6+x*(-8+3*x)), x*x*(18+x*(-32+15*x))};
}

// Stop from the last commanded reference, including when interrupted during
// STARTUP. A slower-than-ratio-10 gait must never accelerate during shutdown.
inline Reference brake(double t, double duration, Reference initial) {
    const double x = std::clamp(t/duration, 0.0, 1.0);
    return {initial.position + initial.velocity*duration*(x-x*x/2),
            initial.velocity*(1-x)};
}
}  // namespace rinbo_fsm::tripod
