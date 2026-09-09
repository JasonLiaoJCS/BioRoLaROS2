#include "motion_effort.hpp"
#include "robot_config.hpp"
#include "safety_invariants.hpp"

#include <openssl/sha.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace rinbo_config {
namespace {
const std::array<std::string, 6> kLegs{{"L1", "L2", "L3", "R1", "R2", "R3"}};
const std::array<std::string, 3> kNodes{{"rinbo_cali", "rinbo_standing", "rinbo_tripod_rslip"}};
using ParamMap = std::map<std::string, rclcpp::Parameter>;

[[noreturn]] void fail(const std::string& message) {
    throw std::runtime_error(message);
}
std::string sys_error(const std::string& what) {
    return what + ": " + std::strerror(errno);
}
std::string read_file(const std::string& path) {
    const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) fail(sys_error("Cannot read effective configuration/state " + path));
    std::string result;
    try {
        struct stat info{};
        if (fstat(fd, &info) < 0) fail(sys_error("Cannot inspect " + path));
        if (!S_ISREG(info.st_mode)) fail("Configuration/state must be a regular file: " + path);
        if (info.st_size > 1024 * 1024) fail("Configuration/state exceeds 1 MiB: " + path);
        std::array<char, 8192> buffer;
        for (;;) {
            const ssize_t n = read(fd, buffer.data(), buffer.size());
            if (n < 0 && errno == EINTR) continue;
            if (n < 0) fail(sys_error("Read failed: " + path));
            if (n == 0) break;
            result.append(buffer.data(), static_cast<size_t>(n));
            if (result.size() > 1024 * 1024) fail("Configuration/state exceeds 1 MiB: " + path);
        }
    } catch (...) { close(fd); throw; }
    close(fd);
    return result;
}
std::string sha256(const std::string& bytes) {
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size(), digest);
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (auto byte : digest) out << std::setw(2) << static_cast<unsigned>(byte);
    return out.str();
}
void validate_tree(const YAML::Node& node, const std::string& location, int depth = 0) {
    if (depth > 20) fail("YAML nesting/alias recursion too deep at " + location);
    if (node.IsMap()) {
        std::set<std::string> keys;
        for (const auto& item : node) {
            if (!item.first.IsScalar()) fail("Non-scalar YAML key at " + location);
            const auto key = item.first.Scalar();
            if (key.empty() || key == "<<") fail("Empty/merge YAML key unsupported at " + location);
            if (!keys.insert(key).second) fail("Duplicate YAML key: " + location + "." + key);
            validate_tree(item.second, location + "." + key, depth + 1);
        }
    } else if (node.IsSequence()) {
        for (const auto& item : node) validate_tree(item, location + "[]", depth + 1);
    } else if (!node.IsScalar()) {
        fail("Null or missing YAML value at " + location);
    }
}
bool implicit_scalar(const YAML::Node& node) {
    return node.IsScalar() && node.Tag() != "!" && node.Tag() != "tag:yaml.org,2002:str";
}
int64_t strict_int(const YAML::Node& node, const std::string& label) {
    static const std::regex integer("[-+]?[0-9]+");
    if (!implicit_scalar(node) || !std::regex_match(node.Scalar(), integer))
        fail(label + " must be an integer (not a quoted string)");
    try { return node.as<int64_t>(); }
    catch (const YAML::Exception&) { fail(label + " integer is out of range"); }
}
double strict_double(const YAML::Node& node, const std::string& label) {
    static const std::regex number("[-+]?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][-+]?[0-9]+)?");
    if (!implicit_scalar(node) || !std::regex_match(node.Scalar(), number))
        fail(label + " must be a finite number (not a quoted string)");
    double value;
    try { value = node.as<double>(); }
    catch (const YAML::Exception&) { fail(label + " number is out of range"); }
    if (!std::isfinite(value)) fail(label + " must be finite");
    return value;
}
bool strict_bool(const YAML::Node& node, const std::string& label) {
    if (!implicit_scalar(node)) fail(label + " must be a boolean true/false");
    auto value = node.Scalar();
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return std::tolower(c); });
    if (value == "true") return true;
    if (value == "false") return false;
    fail(label + " must be a boolean true/false");
}
void require_keys(const YAML::Node& node, const std::set<std::string>& expected,
                  const std::string& label) {
    if (!node.IsMap()) fail(label + " must be a YAML mapping");
    for (const auto& key : expected)
        if (!node[key]) fail(label + " missing required key: " + key);
    for (const auto& item : node)
        if (!expected.count(item.first.Scalar())) fail(label + " unknown key: " + item.first.Scalar());
}

// Schema v1 defaults mirror the declared controller defaults, including shared
// handshake and source-age parameters. Every supplied parameter is checked
// against this table, so misspellings cannot silently fall back to a default.
ParamMap defaults(const std::string& node) {
    ParamMap out;
    auto put = [&](const std::string& name, auto value) { out.emplace(name, rclcpp::Parameter(name, value)); };
    put("max_pwm", node == "rinbo_tripod_rslip" ? 3300.0 : 80.0);
    // Legacy configurations retain their old gains until explicitly tuned.
    put("kp", node == "rinbo_tripod_rslip" ? 0.38 : 0.35);
    put("kd", node == "rinbo_tripod_rslip" ? 0.003 : 0.002);
    put("k_ff", node == "rinbo_tripod_rslip" ? 0.005 : 0.02);
    put("friction_pwm", 0.0);
    put("friction_velocity_counts_s", 153.6);
    put("safety.motor_state_source_max_age_s", 0.10);
    put("safety.power_state_source_max_age_s", 0.35);
    put("safety.motor_arbiter_node_name", std::string("rinbo_ros2_bridge"));
    put("safety.motor_arbiter_ready_timeout_s", 5.0);
    put("safety.motor_arbiter_heartbeat_stale_s", 0.25);
    put("safety.motor_command_ack_timeout_s", 0.25);
    put("safety.stop_on_power_stale", true);
    put("safety.stop_on_current_limit", true);
    put("safety.stop_on_bus_current_limit", false);
    put("safety.require_power_relay", true);
    put("safety.power_bus_voltage_channel", int64_t(7));
    put("safety.leg_current_channels", std::vector<int64_t>{1,2,3,4,5,6});
    put("safety.min_bus_voltage", 18.0);
    put("safety.max_bus_voltage", 42.0);
    put("safety.max_current", 5.0);
    put("safety.max_bus_current", 30.0);
    put("safety.power_stale_seconds", 0.5);
    put("safety.power_required_after_seconds", 2.0);
    put("safety.voltage_trip_samples", int64_t(5));
    put("safety.current_trip_samples", int64_t(25));
    if (node == "rinbo_tripod_rslip") {
        put("startup_duration", 8.0); put("start_ratio", 8.0);
        put("target_ratio", 1.0); put("ratio_step", -0.0002);
        put("velocity_filter_time_constant_s", 0.005);
        put("slowdown_step", 0.002); put("stop_servo_control_mode", int64_t(0));
        put("shutdown.slowdown_duration_s", 2.0); put("shutdown.timeout_s", 5.0);
        put("safety.enabled", true); put("safety.stop_on_voltage_sag", true);
        put("safety.stop_on_over_voltage", true); put("safety.stop_on_position_error", true);
        put("safety.enable_pwm_slew_limit", false);
        put("safety.max_position_error_counts", 9000.0);
        put("safety.position_error_trip_seconds", 0.5);
        put("safety.pwm_slew_rate_per_sec", 250.0);
        put("safety.motor_state_stale_seconds", 0.25);
        put("safety.motor_state_required_after_seconds", 2.0);
        put("safety.position_error_trip_samples", int64_t(10));
    } else {
        put("safety.power_guard_enabled", true); put("safety.stop_on_voltage", true);
        put("safety.hall_search_timeout_s", 30.0);
        put("safety.motor_state_timeout_s", 0.25);
        put("safety.motor_state_required_after_s", 2.0);
        if (node == "rinbo_cali") {
            put("safety.servo_homing_timeout_s", 20.0); put("safety.stop_timeout_s", 5.0);
        } else {
            put("safety.rotate_timeout_s", 20.0);
            put("safety.position_tolerance_counts", 200.0);
            put("safety.settle_velocity_counts_s", 500.0);
            put("safety.settle_time_s", 0.3);
            put("safety.hold_error_counts", 12000.0);
        }
    }
    return out;
}
void flatten(const YAML::Node& node, const std::string& prefix,
             std::map<std::string, YAML::Node>& out) {
    if (!node.IsMap()) fail("parameters must be mappings at " + prefix);
    for (const auto& item : node) {
        const std::string key = prefix.empty() ? item.first.Scalar() : prefix + "." + item.first.Scalar();
        if (item.second.IsMap()) flatten(item.second, key, out);
        else if (!out.emplace(key, item.second).second) fail("Duplicate flattened parameter: " + key);
    }
}
rclcpp::Parameter convert(const std::string& name, const YAML::Node& node,
                          rclcpp::ParameterType type) {
    switch (type) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
        return rclcpp::Parameter(name, strict_bool(node, name));
    case rclcpp::ParameterType::PARAMETER_INTEGER:
        return rclcpp::Parameter(name, strict_int(node, name));
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
        return rclcpp::Parameter(name, strict_double(node, name));
    case rclcpp::ParameterType::PARAMETER_STRING:
        if (!node.IsScalar() || node.Scalar().empty()) fail(name + " must be a nonempty string");
        return rclcpp::Parameter(name, node.Scalar());
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
        if (!node.IsSequence()) fail(name + " must be an integer array");
        std::vector<int64_t> values;
        for (const auto& item : node) values.push_back(strict_int(item, name));
        if (values.size() != 6) fail(name + " must contain exactly six channel indices");
        return rclcpp::Parameter(name, values);
    }
    default: fail("Unsupported schema parameter type: " + name);
    }
}
void validate_parameters(const std::string& node, const ParamMap& p) {
    const bool tripod = node == "rinbo_tripod_rslip";
    auto d = [&](const std::string& name) { return p.at(name).as_double(); };
    auto b = [&](const std::string& name) { return p.at(name).as_bool(); };
    auto i = [&](const std::string& name) {
        const int64_t value = p.at(name).as_int();
        if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max())
            fail(node + ": integer parameter out of range: " + name);
        return static_cast<int>(value);
    };
    std::array<int, 6> channels{};
    const auto values = p.at("safety.leg_current_channels").as_integer_array();
    for (size_t n = 0; n < channels.size(); ++n) {
        if (values[n] < 0 || values[n] > 7) fail(node + ": leg current channels must be in [0,7]");
        channels[n] = static_cast<int>(values[n]);
    }
    const rinbo_fsm::PowerSafetyContract power{
        b(tripod ? "safety.enabled" : "safety.power_guard_enabled"),
        b("safety.stop_on_power_stale"),
        b(tripod ? "safety.stop_on_voltage_sag" : "safety.stop_on_voltage"),
        b(tripod ? "safety.stop_on_over_voltage" : "safety.stop_on_voltage"),
        b("safety.stop_on_current_limit"), b("safety.require_power_relay"),
        i("safety.power_bus_voltage_channel"), channels,
        d("safety.min_bus_voltage"), d("safety.max_bus_voltage"),
        d("safety.max_current"), d("safety.max_bus_current"),
        d("safety.power_stale_seconds"), d("safety.power_required_after_seconds"),
        i("safety.voltage_trip_samples"), i("safety.current_trip_samples")};
    rinbo_fsm::validate_power_safety_contract(power, node);
    if (tripod) rinbo_fsm::validate_tripod_pwm(d("max_pwm"));
    else rinbo_fsm::validate_pwm(d("max_pwm"), node);
    rinbo_fsm::MotionEffort{d("kp"), d("kd"), d("k_ff"), d("friction_pwm"),
        d("friction_velocity_counts_s")}.validate();
    rinbo_fsm::validate_bounded_timeout(d("safety.motor_state_source_max_age_s"),
        rinbo_fsm::kHardMaxMotorSourceAgeS, "safety.motor_state_source_max_age_s");
    rinbo_fsm::validate_bounded_timeout(d("safety.power_state_source_max_age_s"),
        rinbo_fsm::kHardMaxPowerSourceAgeS, "safety.power_state_source_max_age_s");
    rinbo_fsm::validate_motor_arbiter_contract(p.at("safety.motor_arbiter_node_name").as_string(),
        d("safety.motor_arbiter_ready_timeout_s"), d("safety.motor_arbiter_heartbeat_stale_s"),
        d("safety.motor_command_ack_timeout_s"));
    if (tripod) {
        const rinbo_fsm::TripodMotionSafetyContract motion{
            b("safety.stop_on_position_error"), b("safety.enable_pwm_slew_limit"),
            d("safety.max_position_error_counts"), d("safety.pwm_slew_rate_per_sec"),
            d("safety.motor_state_stale_seconds"), d("safety.motor_state_required_after_seconds"),
            i("safety.position_error_trip_samples"), d("safety.position_error_trip_seconds")};
        rinbo_fsm::validate_tripod_motion_safety_contract(motion);
        if (d("velocity_filter_time_constant_s") < 0.0)
            fail("velocity_filter_time_constant_s must be finite and >= 0 (0 = raw velocity)");
        if (d("startup_duration") <= 0 || d("start_ratio") <= 0 || d("target_ratio") <= 0 ||
            d("target_ratio") > d("start_ratio") || d("ratio_step") >= 0 ||
            d("slowdown_step") <= 0 || d("kp") < 0 || d("kd") < 0 || d("k_ff") < 0)
            fail("Tripod trajectory parameters require positive duration/ratios/slowdown, target<=start, negative ratio_step and nonnegative gains");
        const auto slowdown = d("shutdown.slowdown_duration_s");
        const auto timeout = d("shutdown.timeout_s");
        if (slowdown <= 0 || timeout < slowdown || timeout > 5.0)
            fail("Tripod shutdown must satisfy 0 < slowdown_duration <= timeout <= 5s");
        if (i("stop_servo_control_mode") != 0 && i("stop_servo_control_mode") != 2)
            fail("stop_servo_control_mode must be 0 or 2");
    } else {
        rinbo_fsm::validate_motor_state_watchdog(d("safety.motor_state_timeout_s"),
            d("safety.motor_state_required_after_s"), node);
        rinbo_fsm::validate_bounded_timeout(d("safety.hall_search_timeout_s"), 600.0, "safety.hall_search_timeout_s");
        if (node == "rinbo_cali") {
            rinbo_fsm::validate_bounded_timeout(d("safety.servo_homing_timeout_s"), 600.0, "safety.servo_homing_timeout_s");
            rinbo_fsm::validate_bounded_timeout(d("safety.stop_timeout_s"), 600.0, "safety.stop_timeout_s");
        } else {
            rinbo_fsm::validate_bounded_timeout(d("safety.rotate_timeout_s"), 600.0, "safety.rotate_timeout_s");
            rinbo_fsm::validate_bounded_timeout(d("safety.position_tolerance_counts"), 12000.0, "safety.position_tolerance_counts");
            rinbo_fsm::validate_bounded_timeout(d("safety.settle_velocity_counts_s"), 5000.0, "safety.settle_velocity_counts_s");
            rinbo_fsm::validate_bounded_timeout(d("safety.settle_time_s"), 10.0, "safety.settle_time_s");
            rinbo_fsm::validate_bounded_timeout(d("safety.hold_error_counts"), 55296.0, "safety.hold_error_counts");
            if (d("safety.hold_error_counts") < d("safety.position_tolerance_counts"))
                fail("Standing hold error must be >= arrival tolerance");
        }
    }
}
std::string quote_json(const std::string& text) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : text) {
        switch (c) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (c < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
            else out << c;
        }
    }
    out << '"'; return out.str();
}
std::string json_array(const std::vector<std::string>& values) {
    std::ostringstream out; out << '[';
    for (size_t i = 0; i < values.size(); ++i) { if (i) out << ','; out << quote_json(values[i]); }
    out << ']'; return out.str();
}
std::string param_json(const rclcpp::Parameter& p) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) return quote_json(p.as_string());
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) return json_array(p.as_string_array());
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        std::ostringstream out; out << '['; bool comma = false;
        for (const auto v : p.as_integer_array()) { if (comma) out << ','; comma = true; out << v; }
        out << ']'; return out.str();
    }
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        std::ostringstream out; out << std::setprecision(17) << p.as_double(); return out.str();
    }
    return p.value_to_string();
}
void sync_directory(const std::string& path) {
    const auto parent = std::filesystem::path(path).parent_path().string();
    const int fd = open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (fd < 0) fail(sys_error("Cannot open directory for durable write " + parent));
    const int result = fsync(fd); const int saved = errno; close(fd); errno = saved;
    if (result < 0) fail(sys_error("Cannot sync directory " + parent));
}
void atomic_write(const std::string& path, const std::string& contents) {
    std::string temporary = path + ".tmp.XXXXXX";
    std::vector<char> pattern(temporary.begin(), temporary.end()); pattern.push_back('\0');
    int fd = mkstemp(pattern.data());
    if (fd < 0) fail(sys_error("Cannot create atomic temporary file for " + path));
    temporary = pattern.data();
    try {
        struct stat existing{};
        if (lstat(path.c_str(), &existing) == 0) {
            if (!S_ISREG(existing.st_mode)) fail("Refusing to replace nonregular file: " + path);
            if (fchmod(fd, existing.st_mode & 0777) < 0) fail(sys_error("Cannot preserve mode " + path));
        } else if (errno != ENOENT) fail(sys_error("Cannot inspect " + path));
        size_t written = 0;
        while (written < contents.size()) {
            const ssize_t n = write(fd, contents.data() + written, contents.size() - written);
            if (n < 0 && errno == EINTR) continue;
            if (n <= 0) fail(sys_error("Cannot write " + temporary));
            written += static_cast<size_t>(n);
        }
        if (fsync(fd) < 0) fail(sys_error("Cannot sync " + temporary));
        if (close(fd) < 0) { fd = -1; fail(sys_error("Cannot close " + temporary)); }
        fd = -1;
        if (rename(temporary.c_str(), path.c_str()) < 0) fail(sys_error("Cannot replace " + path));
        sync_directory(path);
    } catch (...) {
        if (fd >= 0) close(fd);
        unlink(temporary.c_str()); throw;
    }
}
std::string receipt_path(const RobotConfig& config, Stage stage) {
    return config.path() + (stage == Stage::Calibration ? ".calibration.json" : ".standing.json");
}
void remove_receipt(const RobotConfig& config, Stage stage) {
    const auto path = receipt_path(config, stage);
    if (unlink(path.c_str()) < 0 && errno != ENOENT) fail(sys_error("Cannot invalidate result " + path));
}
void invalidate_all(const RobotConfig& config) {
    remove_receipt(config, Stage::Calibration); remove_receipt(config, Stage::Standing);
    sync_directory(config.path());
}
std::vector<std::string> motion_legs(const RobotConfig& config,
                                    const std::vector<std::string>& selected) {
    const auto enabled = config.enabled_legs();
    const auto legs = selected.empty() ? enabled : normalize_legs(selected);
    if (legs.empty()) fail("沒有可測試腿 / no testable legs");
    for (const auto& leg : legs)
        if (std::find(enabled.begin(), enabled.end(), leg) == enabled.end())
            fail(leg + " is disabled in the site configuration");
    return legs;
}
void require_receipt(const RobotConfig& config, Stage stage,
                     const std::vector<std::string>& selected = {}) {
    const std::string label = stage == Stage::Calibration ? "Calibration" : "Standing";
    const auto path = receipt_path(config, stage);
    try {
        const auto docs = YAML::LoadAll(read_file(path));
        if (docs.size() != 1) fail("result has multiple documents");
        const auto root = docs.front(); validate_tree(root, path);
        const auto version = strict_int(root["schema_version"], path);
        const bool scoped = stage == Stage::Calibration && version == 2;
        if (scoped)
            require_keys(root, {"schema_version", "revision", "hash", "stage", "boot_id", "legs"}, path);
        else
            require_keys(root, {"schema_version", "revision", "hash", "stage", "boot_id"}, path);
        const std::string boot = read_file("/proc/sys/kernel/random/boot_id");
        if ((!scoped && version != 1) ||
            strict_int(root["revision"], path) != config.revision() ||
            root["hash"].as<std::string>() != config.hash() ||
            root["stage"].as<std::string>() != label ||
            root["boot_id"].as<std::string>() != boot)
            fail("result does not match current configuration/boot");
        if (scoped) {
            if (!root["legs"].IsSequence() || root["legs"].size() == 0)
                fail("calibration legs must be a nonempty list");
            const auto covered = motion_legs(config, root["legs"].as<std::vector<std::string>>());
            for (const auto& leg : motion_legs(config, selected))
                if (std::find(covered.begin(), covered.end(), leg) == covered.end())
                    fail("calibration does not cover " + leg);
        }
    } catch (const std::exception& e) {
        fail(label + " result missing/invalid for current config revision/hash: " + e.what() +
             ". Run Calibration -> Standing again before this action.");
    }
}
void validate_cli(int argc, char** argv, bool allow_check) {
    bool ros_args = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--check-config" && allow_check) continue;
        if (arg == "--ros-args") { ros_args = true; continue; }
        if (arg == "--") { ros_args = false; continue; }
        // Logging does not change hardware configuration. Remaps are refused:
        // changing the node name could bypass the node-keyed parameter source.
        if (ros_args && (arg == "--log-level" || arg == "--log-config-file")) {
            if (++i >= argc) fail("Missing value after " + arg);
            continue;
        }
        if (ros_args && (arg == "--enable-rosout-logs" || arg == "--disable-rosout-logs" ||
                        arg == "--enable-stdout-logs" || arg == "--disable-stdout-logs" ||
                        arg == "--enable-external-lib-logs" || arg == "--disable-external-lib-logs")) continue;
        fail("Configuration conflict/unsupported argument '" + arg + "': FSM configuration is fixed at " +
             kConfigPath + "; --params-file, -p/--param, remaps and alternate config paths are not allowed. Use rinbo_legs.");
    }
}
}  // namespace

PathLock::PathLock(const std::string& path, bool exclusive) {
    fd_ = open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd_ < 0) fail(sys_error("Cannot lock configuration; stop actions and check permissions: " + path));
    struct stat info{};
    if (fstat(fd_, &info) < 0 || !S_ISREG(info.st_mode)) {
        close(fd_); fd_ = -1;
        fail("Configuration lock must be a readable regular file: " + path);
    }
    if (flock(fd_, (exclusive ? LOCK_EX : LOCK_SH) | LOCK_NB) < 0) {
        const auto error = sys_error("Configuration/action is busy; stop active motion/power tools before changing it: " + path);
        close(fd_); fd_ = -1; fail(error);
    }
}
PathLock::~PathLock() { if (fd_ >= 0) { flock(fd_, LOCK_UN); close(fd_); } }

std::vector<std::string> normalize_legs(const std::vector<std::string>& names) {
    std::set<std::string> selected;
    for (const auto& raw : names) {
        auto name = raw;
        const auto first = name.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) name.clear();
        else name = name.substr(first, name.find_last_not_of(" \t\r\n") - first + 1);
        std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::toupper(c); });
        if (std::find(kLegs.begin(), kLegs.end(), name) == kLegs.end())
            fail("Unknown leg '" + raw + "'; expected L1,L2,L3,R1,R2,R3 (case-insensitive)");
        if (!selected.insert(name).second) fail("Duplicate leg '" + name + "'");
    }
    std::vector<std::string> result;
    for (const auto& name : kLegs) if (selected.count(name)) result.push_back(name);
    return result;
}

RobotConfig RobotConfig::load(const std::string& path) {
    RobotConfig result;
    if (!std::filesystem::path(path).is_absolute()) fail("Configuration path must be absolute: " + path);
    result.path_ = path;
    const auto bytes = read_file(path);
    try {
        const auto docs = YAML::LoadAll(bytes);
        if (docs.size() != 1) fail("Configuration must contain exactly one YAML document");
        result.document_ = docs.front();
        validate_tree(result.document_, path);
        const auto& root = result.document_;
        require_keys(root, {"schema_version", "revision", "disabled_legs", "test_mode", "parameters"}, path);
        if (strict_int(root["schema_version"], "schema_version") != 1) fail("Unsupported schema_version; supported version is 1");
        result.revision_ = strict_int(root["revision"], "revision");
        if (result.revision_ < 1) fail("revision must be positive");
        if (!root["test_mode"].IsScalar() || root["test_mode"].as<std::string>() != "supported_leg_test")
            fail("Unsupported test_mode; expected supported_leg_test (robot supported/suspended)");
        if (!root["disabled_legs"].IsSequence()) fail("disabled_legs must be a YAML list, including [] for explicitly enabled-all");
        std::vector<std::string> raw;
        for (const auto& item : root["disabled_legs"]) {
            if (!item.IsScalar()) fail("Every disabled_legs entry must be a leg name string");
            raw.push_back(item.Scalar());
        }
        result.disabled_ = normalize_legs(raw);
        require_keys(root["parameters"], std::set<std::string>(kNodes.begin(), kNodes.end()), "parameters");
        for (const auto& node : kNodes) {
            auto effective = defaults(node);
            std::map<std::string, YAML::Node> supplied;
            flatten(root["parameters"][node], "", supplied);
            std::set<std::string> optional_defaults{
                "safety.motor_arbiter_node_name", "safety.motor_arbiter_ready_timeout_s",
                "safety.motor_arbiter_heartbeat_stale_s", "safety.motor_command_ack_timeout_s",
                "stop_servo_control_mode", "safety.position_error_trip_seconds",
                "friction_pwm", "friction_velocity_counts_s"};
            if (node != "rinbo_tripod_rslip") {
                optional_defaults.insert({"kp", "kd", "k_ff"});
            } else {
                optional_defaults.insert("velocity_filter_time_constant_s");
                // A pre-existing site file had a fixed 20 ms filter. Keep it
                // until explicit tuning writes the new Tripod-only setting.
                if (!supplied.count("velocity_filter_time_constant_s"))
                    effective.at("velocity_filter_time_constant_s") =
                        rclcpp::Parameter("velocity_filter_time_constant_s", 0.02);
            }
            // These used to be source constants, so an existing v1 file has
            // no keys for them. Missing keys preserve exactly the old behavior.
            if (node == "rinbo_standing") {
                optional_defaults.insert({"safety.position_tolerance_counts",
                    "safety.settle_velocity_counts_s", "safety.settle_time_s",
                    "safety.hold_error_counts"});
            }
            for (const auto& item : effective)
                if (!supplied.count(item.first) && !optional_defaults.count(item.first))
                    fail(node + ": missing required parameter '" + item.first + "'; restore the site setting explicitly");
            for (const auto& entry : supplied) {
                const auto found = effective.find(entry.first);
                if (found == effective.end()) fail(node + ": unknown/conflicting parameter '" + entry.first +
                    "'; hardware.disabled_legs belongs only at the shared top level");
                found->second = convert(entry.first, entry.second, found->second.get_type());
            }
            validate_parameters(node, effective);
            effective.emplace("hardware.disabled_legs", rclcpp::Parameter("hardware.disabled_legs", result.disabled_));
            effective.emplace("hardware.max_disabled_legs", rclcpp::Parameter("hardware.max_disabled_legs", int64_t(6)));
            for (const auto& entry : effective) result.parameters_[node].push_back(entry.second);
        }
    } catch (const YAML::Exception& e) { fail("Invalid configuration " + path + ": " + e.what()); }
    result.hash_ = sha256(bytes);
    return result;
}
std::vector<std::string> RobotConfig::enabled_legs() const {
    std::vector<std::string> result;
    for (const auto& name : kLegs)
        if (std::find(disabled_.begin(), disabled_.end(), name) == disabled_.end()) result.push_back(name);
    return result;
}
const std::vector<rclcpp::Parameter>& RobotConfig::parameters(const std::string& node) const {
    const auto found = parameters_.find(node);
    if (found == parameters_.end()) fail("Unknown FSM node name: " + node);
    return found->second;
}
rclcpp::NodeOptions RobotConfig::node_options(const std::string& node) const {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.parameter_overrides(parameters(node));
    return options;
}
std::string RobotConfig::json_status() const {
    std::ostringstream out;
    out << "{\"path\":" << quote_json(path_) << ",\"schema_version\":1,\"revision\":" << revision_
        << ",\"hash\":" << quote_json(hash_) << ",\"test_mode\":\"supported_leg_test\",\"disabled_legs\":"
        << json_array(disabled_) << ",\"enabled_legs\":" << json_array(enabled_legs()) << ",\"parameters\":{";
    bool node_comma = false;
    for (const auto& node : parameters_) {
        if (node_comma) out << ','; node_comma = true;
        out << quote_json(node.first) << ":{"; bool comma = false;
        for (const auto& p : node.second) {
            if (comma) out << ','; comma = true;
            out << quote_json(p.get_name()) << ':' << param_json(p);
        }
        out << '}';
    }
    out << "}}"; return out.str();
}
void RobotConfig::print_status(std::ostream& out, const std::string& node) const {
    out << "Effective configuration: " << path_ << "\nSchema version: 1; revision: " << revision_
        << "; SHA256: " << hash_ << "\nDisabled legs: " << json_array(disabled_)
        << "\nEnabled legs: " << json_array(enabled_legs())
        << "\nNext test: supported_leg_test; " << enabled_legs().size() << " enabled legs.\n";
    if (enabled_legs().empty()) out << "没有可測試腿 / no testable legs: all motion entrypoints will refuse.\n";
    if (!node.empty()) {
        out << "Effective parameters for " << node << ":\n";
        for (const auto& p : parameters(node)) out << "  " << p.get_name() << ": " << param_json(p) << '\n';
    }
}

void assert_no_motion_processes(const std::string& proc_root) {
    const std::set<std::string> writers{
        "rinbo_cali", "rinbo_standing", "rinbo_tripod", "rinbo_sin_sweep", "rinbo_manual",
        "redrhex_rl_controller", "rl_controller_node", "lowlevel_bridge_node", "motor_command_tool", "rinbo_motor_command",
        "biorola_servo_probe", "biorola_bringup_plan", "biorola_fault_diag", "rinbo_bringup_check",
        "rinbo_power_tool", "biorola_power_tool", "rinbo_leg_mask"};
    const auto is_writer_name = [&](const std::string& value) {
        std::string base = std::filesystem::path(value).filename().string();
        const std::string deleted = " (deleted)";
        if (base.size() >= deleted.size() && base.compare(base.size() - deleted.size(), deleted.size(), deleted) == 0)
            base.resize(base.size() - deleted.size());
        if (base.size() > 3 && base.substr(base.size() - 3) == ".py") base.resize(base.size() - 3);
        const auto module = base.find_last_of('.');
        if (module != std::string::npos) base = base.substr(module + 1);
        return writers.count(base) != 0;
    };
    try {
        for (const auto& directory : std::filesystem::directory_iterator(proc_root)) {
            const std::string pid = directory.path().filename().string();
            if (pid.empty() || !std::all_of(pid.begin(), pid.end(), [](unsigned char c) { return std::isdigit(c); })) continue;
            if (pid == std::to_string(getpid()) && proc_root == "/proc") continue;
            const auto command_path = directory.path() / "cmdline";
            std::ifstream in(command_path, std::ios::binary);
            if (!in) {
                if (!std::filesystem::exists(directory.path())) continue; // Process exited during scan.
                fail("Cannot inspect process " + pid + "; unable to confirm motion is stopped. Stop actions first.");
            }
            std::ostringstream bytes; bytes << in.rdbuf();
            if (in.bad()) fail("Cannot read process " + pid + "; unable to confirm motion is stopped");
            std::vector<std::string> argv;
            std::istringstream args(bytes.str()); std::string arg;
            while (std::getline(args, arg, '\0')) argv.push_back(arg);
            bool writer = false;
            for (const auto& value : argv) {
                if (is_writer_name(value)) { writer = true; break; }
            }
            // argv[0] can be renamed by exec -a. Inspect the actual binary and
            // Linux task name too, including an old executable replaced on disk.
            // Some unrelated root-owned processes expose cmdline but deny exe;
            // that extra probe is optional once cmdline has been read successfully.
            std::error_code exe_error;
            const auto executable = std::filesystem::read_symlink(directory.path() / "exe", exe_error);
            if (!exe_error && is_writer_name(executable.string())) writer = true;
            std::ifstream comm_file(directory.path() / "comm");
            std::string comm;
            if (std::getline(comm_file, comm)) {
                if (is_writer_name(comm)) writer = true;
                // Linux comm is limited to 15 characters for longer task names.
                for (const auto& name : writers)
                    if (name.size() > 15 && comm == name.substr(0, 15)) writer = true;
            }
            if (writer) {
                // Zombies cannot issue commands. Empty argv alone is not proof:
                // a live process can clear it while its actual executable remains.
                if (argv.empty()) {
                    std::ifstream status_file(directory.path() / "status");
                    std::string line;
                    bool zombie = false;
                    while (std::getline(status_file, line)) {
                        if (line.rfind("State:", 0) != 0) continue;
                        const auto state = line.find_first_not_of(" \t", 6);
                        zombie = state != std::string::npos && line[state] == 'Z';
                        break;
                    }
                    if (zombie) continue;
                }
                // Do not trust a --check-config token attached to an arbitrary
                // legacy tool. A short-lived read-only checker can be retried.
                fail("Motor action/power/mask process detected (PID " + pid + "): stop it before changing configuration");
            }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        fail(std::string("Cannot inspect running actions; refusing configuration change: ") + e.what());
    }
}

bool update_motion_effort(const std::string& path, const rinbo_fsm::MotionEffort& effort) {
    effort.validate();
    PathLock lock(path + ".lock", true);
    assert_no_motion_processes();
    const auto config = RobotConfig::load(path);
    const std::map<std::string, double> values{
        {"kp", effort.kp}, {"kd", effort.kd}, {"k_ff", effort.k_ff},
        {"friction_pwm", effort.friction_pwm},
        {"friction_velocity_counts_s", effort.friction_velocity_counts_s}};
    bool changed = false;
    auto document = YAML::Clone(config.document_);
    for (const auto& node : kNodes) {
        for (const auto& value : values) {
            const auto& parameters = config.parameters(node);
            const auto old = std::find_if(parameters.begin(), parameters.end(),
                [&](const auto& p) { return p.get_name() == value.first; });
            changed = changed || old == parameters.end() || old->as_double() != value.second;
            document["parameters"][node][value.first] = value.second;
        }
    }
    if (!changed) return false;
    if (config.revision() == std::numeric_limits<int64_t>::max()) fail("Configuration revision exhausted");
    document["revision"] = config.revision() + 1;
    YAML::Emitter emit; emit << document;
    if (!emit.good()) fail("Cannot serialize validated motion effort");
    invalidate_all(config);
    atomic_write(path, std::string(emit.c_str()) + "\n");
    return true;
}


std::string tune_limits(const std::string& path, const std::string& stage,
                        const std::map<std::string, double>& updates,
                        bool dry_run, int64_t expected_revision,
                        const std::string& proc_root) {
    const std::map<std::string, std::set<std::string>> allowed{
        {"calibration", {"servo_homing_timeout_s", "hall_search_timeout_s", "stop_timeout_s"}},
        {"standing", {"hall_search_timeout_s", "rotate_timeout_s", "position_tolerance_counts",
                      "settle_velocity_counts_s", "settle_time_s", "hold_error_counts"}},
        {"tripod", {"max_pwm", "max_position_error_counts", "position_error_trip_samples",
                    "position_error_trip_seconds", "stop_on_position_error",
                    "enable_pwm_slew_limit", "pwm_slew_rate_per_sec"}}};
    const std::map<std::string, std::string> nodes{
        {"calibration", "rinbo_cali"}, {"standing", "rinbo_standing"}, {"tripod", "rinbo_tripod_rslip"}};
    if (!allowed.count(stage) || updates.empty()) fail("tune-limits requires calibration|standing|tripod and KEY=VALUE");
    std::unique_ptr<PathLock> lock;
    if (!dry_run) {
        lock = std::make_unique<PathLock>(path + ".lock", true);
        assert_no_motion_processes(proc_root);
    }
    const auto config = RobotConfig::load(path);
    if (expected_revision >= 0 && expected_revision != config.revision())
        fail("設定版本已改變，請重新整理後再儲存 (configuration revision changed)");
    const auto& node = nodes.at(stage);
    ParamMap parameters;
    for (const auto& p : config.parameters(node)) parameters.emplace(p.get_name(), p);
    bool changed = false;
    std::ostringstream changes; bool comma = false;
    for (const auto& [key, value] : updates) {
        if (!allowed.at(stage).count(key) || !std::isfinite(value)) fail("Unsupported/nonfinite motion limit: " + key);
        const auto name = key == "max_pwm" ? key : "safety." + key;
        const auto before = parameters.at(name);
        rclcpp::Parameter after(name, value);
        if (before.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
            if (value != 0 && value != 1) fail(key + " must be 0 (off) or 1 (on)");
            after = rclcpp::Parameter(name, value == 1);
        }
        if (before.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            if (value < 1 || value > 1000000 || std::floor(value) != value) fail(key + " must be a positive integer");
            after = rclcpp::Parameter(name, static_cast<int64_t>(value));
        }
        changed = changed || before != after;
        parameters[name] = after;
        if (comma) changes << ','; comma = true;
        changes << quote_json(key) << ":{\"before\":" << param_json(before) << ",\"after\":" << param_json(after) << '}';
    }
    validate_parameters(node, parameters);
    std::map<Stage, YAML::Node> receipts;
    // Only prerequisites of an unchanged stage may carry forward. A changed
    // Standing result itself is always invalidated; never manufacture success.
    if (stage != "calibration") for (const auto prerequisite : {Stage::Calibration, Stage::Standing}) {
        if (prerequisite == Stage::Standing && (stage != "tripod" || !receipts.count(Stage::Calibration))) continue;
        try {
            require_receipt(config, prerequisite);
            receipts.emplace(prerequisite, YAML::Load(read_file(receipt_path(config, prerequisite))));
        } catch (const std::exception&) {}
    }
    std::string result_hash = config.hash();
    if (changed && config.revision() == std::numeric_limits<int64_t>::max()) fail("Configuration revision exhausted");
    const auto next_revision = config.revision() + (changed ? 1 : 0);
    if (changed) {
        if (config.revision() == std::numeric_limits<int64_t>::max()) fail("Configuration revision exhausted");
        const auto current = read_file(path);
        if (sha256(current) != config.hash()) fail("Configuration changed during limit editing; retry");
        auto document = YAML::Load(current);
        for (const auto& [key, value] : updates) {
            if (key == "max_pwm") { document["parameters"][node][key] = value; continue; }
            if (parameters.at("safety." + key).get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
                document["parameters"][node]["safety"][key] = value == 1;
            else if (parameters.at("safety." + key).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                document["parameters"][node]["safety"][key] = static_cast<int64_t>(value);
            else document["parameters"][node]["safety"][key] = value;
        }
        document["revision"] = next_revision;
        YAML::Emitter emit; emit << document;
        if (!emit.good()) fail("Cannot serialize motion limits");
        const auto bytes = std::string(emit.c_str()) + "\n";
        result_hash = sha256(bytes);
        if (!dry_run) {
            invalidate_all(config);
            atomic_write(path, bytes);
            for (auto& [prerequisite, receipt] : receipts) {
                receipt["revision"] = next_revision; receipt["hash"] = result_hash;
                YAML::Emitter result; result << receipt;
                if (!result.good()) fail("Cannot preserve prerequisite result");
                atomic_write(receipt_path(config, prerequisite), std::string(result.c_str()) + "\n");
            }
        }
    }
    std::ostringstream out;
    out << "{\"dry_run\":" << (dry_run ? "true" : "false")
        << ",\"changed\":" << (changed ? "true" : "false")
        << ",\"stage\":" << quote_json(stage) << ",\"previous_revision\":" << config.revision()
        << ",\"revision\":" << next_revision << ",\"hash\":" << quote_json(result_hash)
        << ",\"changes\":{" << changes.str() << "},\"restart_required\":" << (changed ? "true" : "false") << '}';
    return out.str();
}

static std::string apply_tripod_tuning(const std::string& path,
                       const std::map<std::string, double>& updates,
                       bool dry_run, const std::string& proc_root,
                       std::optional<bool> stop_on_position_error) {
    const std::set<std::string> allowed{
        "startup_duration", "start_ratio", "target_ratio", "ratio_step",
        "kp", "kd", "k_ff", "friction_pwm", "friction_velocity_counts_s",
        "velocity_filter_time_constant_s"};
    if (updates.empty() && !stop_on_position_error.has_value())
        fail("tune-tripod requires KEY=VALUE; use status --json to inspect");
    for (const auto& [key, value] : updates)
        if (!allowed.count(key) || !std::isfinite(value))
            fail("Unsupported/nonfinite Tripod tuning value: " + key);
    // Preview remains available while an action is running. Applying takes the
    // same exclusive configuration lock as existing configuration operations.
    std::unique_ptr<PathLock> lock;
    if (!dry_run) {
        lock = std::make_unique<PathLock>(path + ".lock", true);
        assert_no_motion_processes(proc_root);
    }
    const auto config = RobotConfig::load(path);
    std::map<std::string, rclcpp::Parameter> parameters;
    for (const auto& p : config.parameters("rinbo_tripod_rslip")) parameters.emplace(p.get_name(), p);
    bool changed = false;
    for (const auto& [key, value] : updates) {
        changed = changed || parameters.at(key).as_double() != value;
        parameters[key] = rclcpp::Parameter(key, value);
    }
    if (stop_on_position_error.has_value()) {
        const std::string key = "safety.stop_on_position_error";
        changed = changed || parameters.at(key).as_bool() != *stop_on_position_error;
        parameters[key] = rclcpp::Parameter(key, *stop_on_position_error);
    }
    validate_parameters("rinbo_tripod_rslip", parameters);
    std::map<Stage, YAML::Node> receipts;
    for (const Stage stage : {Stage::Calibration, Stage::Standing}) {
        if (stage == Stage::Standing && !receipts.count(Stage::Calibration)) continue;
        try {
            require_receipt(config, stage);
            receipts.emplace(stage, YAML::Load(read_file(receipt_path(config, stage))));
        } catch (const std::exception&) { /* A missing/invalid result is never recreated. */ }
    }
    if (!dry_run && changed) {
        if (config.revision() == std::numeric_limits<int64_t>::max()) fail("Configuration revision exhausted");
        const auto current_bytes = read_file(path);
        // Abort an uncooperative external edit instead of rebinding its receipts.
        if (sha256(current_bytes) != config.hash()) fail("Configuration changed during tuning; retry");
        auto document = YAML::Load(current_bytes);
        for (const auto& [key, value] : updates) document["parameters"]["rinbo_tripod_rslip"][key] = value;
        if (stop_on_position_error.has_value())
            document["parameters"]["rinbo_tripod_rslip"]["safety"]["stop_on_position_error"] = *stop_on_position_error;
        document["revision"] = config.revision() + 1;
        YAML::Emitter emit; emit << document;
        if (!emit.good()) fail("Cannot serialize Tripod tuning");
        const std::string bytes = std::string(emit.c_str()) + "\n";
        const std::string hash = sha256(bytes);
        // Remove old bindings before publishing the new file. A crash at any
        // intermediate step can lose readiness, never approve stale settings.
        invalidate_all(config);
        atomic_write(path, bytes);
        for (auto& [stage, receipt] : receipts) {
            receipt["revision"] = config.revision() + 1;
            receipt["hash"] = hash;
            YAML::Emitter result; result << receipt;
            if (!result.good()) fail("Cannot preserve prerequisite result");
            atomic_write(receipt_path(config, stage), std::string(result.c_str()) + "\n");
        }
    }
    std::ostringstream result;
    result << "{\"dry_run\":" << (dry_run ? "true" : "false")
           << ",\"changed\":" << (changed ? "true" : "false")
           << ",\"calibration_valid\":" << (receipts.count(Stage::Calibration) ? "true" : "false")
           << ",\"standing_valid\":" << (receipts.count(Stage::Standing) ? "true" : "false")
           << ",\"position_error_policy\":" << quote_json(
                parameters.at("safety.stop_on_position_error").as_bool() ? "stop" : "warn_only")
           << ",\"parameters\":{";
    bool comma = false;
    for (const auto& key : allowed) {
        if (comma) result << ','; comma = true;
        result << quote_json(key) << ':' << param_json(parameters.at(key));
    }
    result << "}}";
    return result.str();
}

std::string tune_tripod(const std::string& path,
                       const std::map<std::string, double>& updates,
                       bool dry_run, const std::string& proc_root) {
    return apply_tripod_tuning(path, updates, dry_run, proc_root, std::nullopt);
}

std::string set_tripod_position_policy(const std::string& path, bool stop_on_error,
                                      bool dry_run, const std::string& proc_root) {
    return apply_tripod_tuning(path, {}, dry_run, proc_root, stop_on_error);
}

bool update_disabled_legs(const std::string& path, const std::string& operation,
                         const std::vector<std::string>& names) {
    const auto requested = normalize_legs(names); // Validate the entire request before taking any action.
    if (operation != "set" && operation != "disable" && operation != "enable" && operation != "enable-all")
        fail("Unknown operation: " + operation);
    if (operation == "enable-all" && !requested.empty()) fail("enable-all takes no leg names");
    if (operation != "enable-all" && requested.empty()) fail(operation + " requires leg names; use explicit enable-all to clear the mask");
    PathLock lock(path + ".lock", true);
    assert_no_motion_processes();
    const auto config = RobotConfig::load(path);
    auto target = config.disabled_legs();
    if (operation == "set") target = requested;
    else if (operation == "enable-all") target.clear();
    else {
        for (const auto& name : requested) {
            const auto found = std::find(target.begin(), target.end(), name);
            if (operation == "disable" && found == target.end()) target.push_back(name);
            else if (operation == "enable" && found != target.end()) target.erase(found);
        }
        target = normalize_legs(target);
    }
    if (target == config.disabled_legs()) return false;
    if (config.revision() == std::numeric_limits<int64_t>::max()) fail("Configuration revision exhausted");
    auto document = YAML::Clone(config.document_);
    document["disabled_legs"] = target;
    document["disabled_legs"].SetStyle(YAML::EmitterStyle::Flow);
    document["revision"] = config.revision() + 1;
    YAML::Emitter emit; emit << document;
    if (!emit.good()) fail("Cannot serialize validated configuration");
    // Remove receipts first: even a failed/uncertain update cannot leave stale
    // approval data behind. Hash checks also reject direct external file edits.
    invalidate_all(config);
    atomic_write(path, std::string(emit.c_str()) + "\n");
    return true;
}

bool check_config_cli(int argc, char** argv, const std::string& node_name) {
    validate_cli(argc, argv, true);
    bool check = false;
    for (int i = 1; i < argc; ++i) if (std::string(argv[i]) == "--check-config") check = true;
    if (!check) return false;
    const auto config = RobotConfig::load();
    config.print_status(std::cout, node_name);
    return true;
}
MotionSession::MotionSession(Stage stage, int argc, char** argv, const std::string& path)
    : MotionSession(stage, [&]() { validate_cli(argc, argv, false); return path; }()) {}
void check_manual_readiness(const RobotConfig& config, const std::vector<std::string>& selected) {
    if (config.enabled_legs().empty()) fail("沒有可測試腿 / no testable legs");
    require_receipt(config, Stage::Calibration, motion_legs(config, selected));
}
MotionSession::MotionSession(Stage stage, const std::string& path,
                             const std::vector<std::string>& selected)
    : config_lock_(path + ".lock", false), action_lock_(path + ".motion.lock", true),
      config_(RobotConfig::load(path)), stage_(stage) {
    if (config_.enabled_legs().empty()) fail("没有可測試腿 / no testable legs: all six legs are disabled; refusing motion");
    if (!selected.empty() && stage_ != Stage::Calibration && stage_ != Stage::Manual)
        fail("Selected legs are supported only for calibration and manual control");
    active_legs_ = motion_legs(config_, selected);
    scoped_ = !selected.empty();
    if (stage_ != Stage::Calibration) require_receipt(config_, Stage::Calibration, active_legs_);
    if (stage_ == Stage::Tripod) require_receipt(config_, Stage::Standing);
    if (stage_ == Stage::Calibration) invalidate_all(config_);
    else if (stage_ == Stage::Standing || stage_ == Stage::Manual) {
        remove_receipt(config_, Stage::Standing); sync_directory(config_.path());
    }
}
void MotionSession::complete() {
    // Never approve a process whose source was manually edited during runtime.
    const auto current = RobotConfig::load(config_.path());
    if (current.hash() != config_.hash() || current.revision() != config_.revision()) {
        invalidate(); fail("Effective configuration changed during motion; result rejected. Repeat Calibration -> Standing");
    }
    if (stage_ == Stage::Tripod || stage_ == Stage::Manual) return;
    const std::string label = stage_ == Stage::Calibration ? "Calibration" : "Standing";
    std::ostringstream receipt;
    receipt << "{\"schema_version\":" << (stage_ == Stage::Calibration ? 2 : 1)
            << ",\"revision\":" << config_.revision()
            << ",\"hash\":" << quote_json(config_.hash()) << ",\"stage\":" << quote_json(label)
            << ",\"boot_id\":" << quote_json(read_file("/proc/sys/kernel/random/boot_id"));
    if (stage_ == Stage::Calibration) receipt << ",\"legs\":" << json_array(active_legs_);
    receipt << "}\n";
    atomic_write(receipt_path(config_, stage_), receipt.str());
}
void MotionSession::invalidate() { invalidate_all(config_); }
}  // namespace rinbo_config
