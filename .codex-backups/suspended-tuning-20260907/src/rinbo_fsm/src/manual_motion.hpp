#pragma once

// Hardware-independent reference generator. All public angles are degrees;
// normalized positive rotation follows rinbo_cali (left encoder sign reversed).
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>

namespace rinbo_manual {
inline constexpr std::array<const char*, 6> names{"L1", "L2", "L3", "R1", "R2", "R3"};
inline constexpr double counts_per_degree = 55296.0 / 360.0;
enum class Mode { Off, Position, Velocity, Cycle, Relative };
struct Leg {
    Mode mode = Mode::Off;
    double angle = 0.0;  // position angle or initial cycle phase, modulo 360
    double speed = 0.0;
};
struct Plan {
    double duration = 5.0;
    double max_pwm = 20.0;
    double max_speed = 30.0;
    double acceleration = 30.0;
    std::array<Leg, 6> legs{};
};

inline void keys(const YAML::Node& n, const std::set<std::string>& allowed,
                 const std::set<std::string>& required, const std::string& label) {
    if (!n.IsMap()) throw std::runtime_error(label + " must be a mapping");
    std::set<std::string> seen;
    for (const auto& entry : n) {
        if (!entry.first.IsScalar()) throw std::runtime_error(label + ": invalid key");
        const auto key = entry.first.Scalar();
        if (!allowed.count(key) || !seen.insert(key).second)
            throw std::runtime_error(label + ": unknown or duplicate key " + key);
    }
    for (const auto& key : required)
        if (!seen.count(key)) throw std::runtime_error(label + ": missing " + key);
}
inline double number(const YAML::Node& n, double lo, double hi, const std::string& label) {
    if (!n.IsScalar() || n.Tag() == "!" || n.Tag() == "tag:yaml.org,2002:str")
        throw std::runtime_error(label + " must be a number, not a string");
    const double v = n.as<double>();
    if (!std::isfinite(v) || v < lo || v > hi)
        throw std::runtime_error(label + " outside [" + std::to_string(lo) + "," + std::to_string(hi) + "]");
    return v;
}
inline Plan parse(const std::string& contents) {
    if (contents.size() > 65536) throw std::runtime_error("Plan exceeds 64 KiB");
    const auto docs = YAML::LoadAll(contents);
    if (docs.size() != 1) throw std::runtime_error("Plan must have exactly one YAML document");
    const auto root = docs.front();
    keys(root, {"duration_s", "max_pwm", "max_speed_deg_s", "acceleration_deg_s2", "legs"},
         {"duration_s", "legs"}, "plan");
    Plan p;
    p.duration = number(root["duration_s"], 1.0, 60.0, "duration_s");
    if (root["max_pwm"]) p.max_pwm = number(root["max_pwm"], 1.0, 80.0, "max_pwm");
    if (root["max_speed_deg_s"]) p.max_speed = number(root["max_speed_deg_s"], 1.0, 90.0, "max_speed_deg_s");
    if (root["acceleration_deg_s2"]) p.acceleration = number(root["acceleration_deg_s2"], 1.0, 90.0, "acceleration_deg_s2");
    keys(root["legs"], {names.begin(), names.end()}, {}, "legs");
    bool active = false;
    for (size_t i = 0; i < 6; ++i) {
        const auto n = root["legs"][names[i]];
        if (!n) continue;  // omitted legs are always disabled, never held
        if (!n.IsMap() || !n["mode"] || !n["mode"].IsScalar())
            throw std::runtime_error(std::string(names[i]) + ": missing mode");
        const auto mode = n["mode"].as<std::string>();
        auto& leg = p.legs[i];
        if (mode == "off") {
            keys(n, {"mode"}, {"mode"}, names[i]);
        } else if (mode == "relative") {
            keys(n, {"mode", "move_deg"}, {"mode", "move_deg"}, names[i]);
            leg.mode = Mode::Relative;
            leg.angle = number(n["move_deg"], -30.0, 30.0, "move_deg");
        } else if (mode == "position") {
            keys(n, {"mode", "angle_deg"}, {"mode", "angle_deg"}, names[i]);
            leg.mode = Mode::Position;
            leg.angle = number(n["angle_deg"], -360.0, 360.0, "angle_deg");
        } else if (mode == "velocity" || mode == "cycle") {
            const std::set<std::string> fields = mode == "cycle"
                ? std::set<std::string>{"mode", "speed_deg_s", "phase_deg"}
                : std::set<std::string>{"mode", "speed_deg_s"};
            keys(n, fields, fields, names[i]);
            leg.mode = mode == "cycle" ? Mode::Cycle : Mode::Velocity;
            leg.speed = number(n["speed_deg_s"], -p.max_speed, p.max_speed, "speed_deg_s");
            if (mode == "cycle") leg.angle = number(n["phase_deg"], -360.0, 360.0, "phase_deg");
        } else throw std::runtime_error(std::string(names[i]) + ": mode must be off, relative, position, velocity or cycle");
        active = active || leg.mode != Mode::Off;
    }
    if (!active) throw std::runtime_error("Plan selects no motors");
    return p;
}
inline Plan load(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("Cannot open plan: " + path);
    std::string bytes;
    char c;
    while (in.get(c)) {
        bytes += c;
        if (bytes.size() > 65536) throw std::runtime_error("Plan exceeds 64 KiB");
    }
    if (!in.eof()) throw std::runtime_error("Cannot read plan: " + path);
    return parse(bytes);
}
inline void validate_mask(const Plan& p, const std::array<bool, 6>& disabled) {
    for (size_t i = 0; i < 6; ++i)
        if (disabled[i] && p.legs[i].mode != Mode::Off)
            throw std::runtime_error(std::string(names[i]) + " is disabled in the site configuration; remove it from this plan");
}
inline double encoder_degrees(size_t i, double raw) {
    return (i < 3 ? -raw : raw) / counts_per_degree;
}
inline double nearest_angle(double current, double phase) {
    // Exactly 180 degrees chooses the positive direction deterministically.
    double d = std::fmod(phase - current, 360.0);
    if (d <= -180.0) d += 360.0;
    if (d > 180.0) d -= 360.0;
    return current + d;
}
struct Sample {
    std::array<double, 6> position{}, velocity{};
    bool done = false;
};
class Trajectory {
public:
    Trajectory(const Plan& plan, const std::array<double, 6>& initial)
        : p_(plan), initial_(initial), aligned_(initial) {
        for (size_t i = 0; i < 6; ++i) {
            if (p_.legs[i].mode == Mode::Off) continue;
            if (!std::isfinite(initial[i])) throw std::runtime_error("Invalid initial position");
            if (p_.legs[i].mode == Mode::Position || p_.legs[i].mode == Mode::Cycle)
                aligned_[i] = nearest_angle(initial[i], p_.legs[i].angle);
            if (p_.legs[i].mode == Mode::Relative)
                aligned_[i] = initial[i] + p_.legs[i].angle;
            const double distance = std::abs(aligned_[i] - initial[i]);
            // Quintic smoothstep has max derivative 1.875 and max second
            // derivative 10/sqrt(3). Both bounds apply to the whole alignment.
            align_s_ = std::max({align_s_, 1.875 * distance / p_.max_speed,
                std::sqrt((10.0 / std::sqrt(3.0)) * distance / p_.acceleration)});
            ramp_s_ = std::max(ramp_s_, std::abs(p_.legs[i].speed) / p_.acceleration);
        }
    }
    double alignment_seconds() const { return align_s_; }
    double total_seconds() const { return align_s_ + ramp_s_ + p_.duration + ramp_s_ + 1.0; }
    Sample sample(double elapsed) const {
        if (!std::isfinite(elapsed) || elapsed < 0) throw std::runtime_error("Invalid elapsed time");
        Sample out;
        out.done = elapsed >= total_seconds();
        const double u = std::clamp(elapsed / align_s_, 0.0, 1.0);
        const double blend = u*u*u*(10.0 + u*(-15.0 + 6.0*u));
        const double blend_rate = 30.0*u*u*(1.0-u)*(1.0-u)/align_s_;
        const double t = std::max(0.0, elapsed - align_s_);
        double travel = 0.0, rate = 0.0;
        if (t < ramp_s_) { rate = t/ramp_s_; travel = t*t/(2.0*ramp_s_); }
        else if (t < ramp_s_ + p_.duration) { rate = 1.0; travel = t - ramp_s_/2.0; }
        else {
            const double decel = std::min(t - ramp_s_ - p_.duration, ramp_s_);
            rate = 1.0 - decel/ramp_s_;
            travel = ramp_s_/2.0 + p_.duration + decel - decel*decel/(2.0*ramp_s_);
        }
        for (size_t i = 0; i < 6; ++i) {
            const double delta = aligned_[i] - initial_[i];
            out.position[i] = initial_[i] + delta*blend;
            out.velocity[i] = delta*blend_rate;
            if (elapsed >= align_s_ && (p_.legs[i].mode == Mode::Velocity || p_.legs[i].mode == Mode::Cycle)) {
                out.position[i] += p_.legs[i].speed*travel;
                out.velocity[i] = p_.legs[i].speed*rate;
            }
        }
        return out;
    }
private:
    Plan p_;
    std::array<double, 6> initial_, aligned_;
    double align_s_ = 1.0, ramp_s_ = 1.0;
};

// Last output boundary: neither omitted nor masked motors can be enabled.
template<class Command>
inline void set_outputs(Command& cmd, const Plan& p, const std::array<bool, 6>& disabled,
                        const std::array<double, 6>& pwm, double cap, bool active) {
    cmd = Command{};
    const std::array<decltype(&cmd.l1), 6> legs{&cmd.l1, &cmd.l2, &cmd.l3, &cmd.r1, &cmd.r2, &cmd.r3};
    for (size_t i = 0; i < 6; ++i) {
        if (!active || disabled[i] || p.legs[i].mode == Mode::Off) continue;
        if (!std::isfinite(pwm[i]) || !std::isfinite(cap) || cap <= 0 || cap > 80)
            throw std::runtime_error("Invalid PWM output");
        auto& leg = *legs[i];
        const double value = std::clamp(pwm[i], -cap, cap);
        leg.enable = true;
        leg.direction = i < 3 ? value < 0 : value >= 0;
        leg.voltage = std::abs(value);
        leg.state = 1;
    }
    // No encoder resets and no servo commands. Global servo mode stays zero.
}
}  // namespace rinbo_manual
