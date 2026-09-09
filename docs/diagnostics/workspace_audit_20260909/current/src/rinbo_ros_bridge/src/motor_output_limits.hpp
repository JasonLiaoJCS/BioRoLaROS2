#pragma once
#include <cmath>
#include <optional>
#include <sstream>
#include <string>

namespace rinbo_bridge {
// Transport envelope in raw U16 FPGA command units, not a percentage. Each
// controller still applies its own cap: Tripod 3300, Cali/Standing/Manual 80.
inline constexpr double kMaxRawMotorCommand = 3300.0;
inline bool valid_motor_output_cap(double value) {
    return std::isfinite(value) && value > 0 && value <= kMaxRawMotorCommand;
}
inline std::optional<std::string> validate_motor_output(double value, double cap,
                                                      const char* leg) {
    if (!valid_motor_output_cap(cap)) return "invalid main motor output cap";
    if (!std::isfinite(value)) return std::string("non-finite main voltage for ") + leg;
    if (value < 0 || value > cap) {
        std::ostringstream out;
        out << "main voltage for " << leg << " is " << value << ", outside [0," << cap << "]";
        return out.str();
    }
    return std::nullopt;
}
} // namespace rinbo_bridge
