#include "robot_config.hpp"
#include "motion_effort.hpp"
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    try {
        if (argc < 2) throw std::runtime_error(
            "Usage: rinbo_legs status [--json] | set LEG... | disable LEG... | enable LEG... | enable-all | tripod-position-policy warn|stop [--dry-run] | tune-tripod [--dry-run] KEY=VALUE... | tune-motion KP KD K_FF FRICTION_PWM FADE_COUNTS_S");
        const std::string operation = argv[1];
        if (operation == "status") {
            if (argc > 3 || (argc == 3 && std::string(argv[2]) != "--json"))
                throw std::runtime_error("Usage: rinbo_legs status [--json]");
            const auto config = rinbo_config::RobotConfig::load();
            if (argc == 3) std::cout << config.json_status() << '\n';
            else config.print_status(std::cout);
            return 0;
        }
        if (operation == "tripod-position-policy") {
            if (argc < 3 || argc > 4 ||
                (std::string(argv[2]) != "warn" && std::string(argv[2]) != "stop") ||
                (argc == 4 && std::string(argv[3]) != "--dry-run"))
                throw std::runtime_error("Usage: rinbo_legs tripod-position-policy warn|stop [--dry-run]");
            std::cout << rinbo_config::set_tripod_position_policy(
                rinbo_config::kConfigPath, std::string(argv[2]) == "stop", argc == 4) << '\n';
            return 0;
        }
        if (operation == "tune-tripod") {
            bool preview = false;
            std::map<std::string, double> updates;
            for (int i = 2; i < argc; ++i) {
                const std::string token = argv[i];
                if (token == "--dry-run") { preview = true; continue; }
                const auto equal = token.find('=');
                if (equal == std::string::npos) throw std::runtime_error("Expected KEY=VALUE: " + token);
                const auto key = token.substr(0, equal), value_text = token.substr(equal+1);
                size_t used = 0;
                const double value = std::stod(value_text, &used);
                if (used != value_text.size() || !updates.emplace(key, value).second)
                    throw std::runtime_error("Invalid or duplicate Tripod setting: " + token);
            }
            std::cout << rinbo_config::tune_tripod(rinbo_config::kConfigPath, updates, preview) << '\n';
            return 0;
        }
        if (operation == "tune-motion") {
            if (argc != 7) throw std::runtime_error("Usage: rinbo_legs tune-motion KP KD K_FF FRICTION_PWM FADE_COUNTS_S");
            auto number = [&](int index) {
                size_t used = 0;
                const std::string text = argv[index];
                const double value = std::stod(text, &used);
                if (used != text.size()) throw std::runtime_error("Invalid motion effort number: " + text);
                return value;
            };
            const rinbo_fsm::MotionEffort effort{number(2), number(3), number(4), number(5), number(6)};
            const bool changed = rinbo_config::update_motion_effort(rinbo_config::kConfigPath, effort);
            std::cout << (changed ? "Motion effort updated for Calibration, Standing, Manual and Tripod. Repeat calibration.\n"
                                  : "Motion effort unchanged.\n");
            rinbo_config::RobotConfig::load().print_status(std::cout);
            return 0;
        }
        std::vector<std::string> names;
        for (int i = 2; i < argc; ++i) names.emplace_back(argv[i]);
        const bool changed = rinbo_config::update_disabled_legs(rinbo_config::kConfigPath, operation, names);
        const auto config = rinbo_config::RobotConfig::load();
        std::cout << (changed ? "Configuration updated. Previous Calibration/Standing results invalidated.\n"
                              : "No change (idempotent); revision and existing results unchanged.\n");
        config.print_status(std::cout);
        if (changed) std::cout << "Next: Calibration -> Standing -> Tripod; no rebuild is required.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[FATAL] " << error.what() << '\n';
        return 2;
    }
}
