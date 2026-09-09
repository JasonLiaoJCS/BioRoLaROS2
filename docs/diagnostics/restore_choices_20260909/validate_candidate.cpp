// Diagnostic-only: parse a private candidate using production validation.
// Never initializes ROS or starts a controller.
#include "robot_config.hpp"
#include <iostream>
int main(int argc,char** argv) {
    try {
        if(argc != 2) throw std::runtime_error("Expected candidate YAML path");
        rinbo_config::assert_no_motion_processes();
        std::cout << rinbo_config::RobotConfig::load(argv[1]).json_status() << '\n';
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
