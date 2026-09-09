#include "robot_config.hpp"
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    try {
        if (argc < 2) throw std::runtime_error(
            "Usage: rinbo_legs status [--json] | set LEG... | disable LEG... | enable LEG... | enable-all");
        const std::string operation = argv[1];
        if (operation == "status") {
            if (argc > 3 || (argc == 3 && std::string(argv[2]) != "--json"))
                throw std::runtime_error("Usage: rinbo_legs status [--json]");
            const auto config = rinbo_config::RobotConfig::load();
            if (argc == 3) std::cout << config.json_status() << '\n';
            else config.print_status(std::cout);
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
