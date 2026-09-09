#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <iosfwd>
#include <map>
#include <string>
#include <vector>

namespace rinbo_fsm { struct MotionEffort; }

// The production entrypoints deliberately have no path argument or environment
// override. Library path arguments exist for isolated, hardware-free tests.
namespace rinbo_config {
inline constexpr const char* kConfigPath =
    "/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml";
enum class Stage { Calibration, Standing, Tripod, Manual };

class PathLock {
public:
    explicit PathLock(const std::string& lock_path, bool exclusive = true);
    ~PathLock();
    PathLock(const PathLock&) = delete;
    PathLock& operator=(const PathLock&) = delete;
private:
    int fd_ = -1;
};

class RobotConfig {
public:
    static RobotConfig load(const std::string& path = kConfigPath);
    rclcpp::NodeOptions node_options(const std::string& node_name) const;
    void print_status(std::ostream& out, const std::string& node_name = "") const;
    std::string json_status() const;
    const std::string& path() const { return path_; }
    const std::string& hash() const { return hash_; }
    int64_t revision() const { return revision_; }
    const std::vector<std::string>& disabled_legs() const { return disabled_; }
    std::vector<std::string> enabled_legs() const;
    const std::vector<rclcpp::Parameter>& parameters(const std::string& node_name) const;
private:
    friend bool update_disabled_legs(const std::string&, const std::string&,
                                    const std::vector<std::string>&);
    friend bool update_motion_effort(const std::string&, const rinbo_fsm::MotionEffort&);
    YAML::Node document_;
    std::string path_, hash_;
    int64_t revision_ = 0;
    std::vector<std::string> disabled_;
    std::map<std::string, std::vector<rclcpp::Parameter>> parameters_;
};

std::vector<std::string> normalize_legs(const std::vector<std::string>& names);
void assert_no_motion_processes(const std::string& proc_root = "/proc");
bool update_disabled_legs(const std::string& path, const std::string& operation,
                         const std::vector<std::string>& names);
bool update_motion_effort(const std::string& path, const rinbo_fsm::MotionEffort& effort);
// Tripod-only operator tuning. Preview never writes or requires motion to stop.
// Applying carries forward ONLY receipts valid for the unchanged Calibration /
// Standing configuration and current boot. proc_root injection is for offline tests.
std::string tune_tripod(const std::string& path,
                       const std::map<std::string, double>& updates,
                       bool dry_run = false, const std::string& proc_root = "/proc");
// Only finite Tripod tracking errors change action. Other guards are unchanged.
std::string set_tripod_position_policy(const std::string& path, bool stop_on_error,
                                      bool dry_run = false,
                                      const std::string& proc_root = "/proc");
// Pure read-only check, called before rclcpp::init; also rejects competing ROS
// parameter inputs in normal startup. Throws on malformed/unsupported options.
bool check_config_cli(int argc, char** argv, const std::string& node_name);
// Read-only prerequisite check; never creates, removes, or approves receipts.
void check_manual_readiness(const RobotConfig& config,
                            const std::vector<std::string>& selected = {});

class MotionSession {
public:
    MotionSession(Stage stage, int argc, char** argv,
                  const std::string& path = kConfigPath);
    explicit MotionSession(Stage stage, const std::string& path = kConfigPath,
                           const std::vector<std::string>& selected = {});
    const RobotConfig& config() const { return config_; }
    const std::vector<std::string>& active_legs() const { return active_legs_; }
    bool scoped() const { return scoped_; }
    void complete();
    void invalidate();
private:
    PathLock config_lock_;
    PathLock action_lock_;
    RobotConfig config_;
    Stage stage_;
    std::vector<std::string> active_legs_;
    bool scoped_ = false;
};
}  // namespace rinbo_config
