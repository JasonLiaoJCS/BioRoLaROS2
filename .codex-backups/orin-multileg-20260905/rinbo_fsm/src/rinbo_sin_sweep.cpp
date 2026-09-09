#include <iostream>

int main() {
    std::cerr
        << "ERROR: rinbo_sin_sweep is retired. No ROS node or motor-command "
           "publisher was created. The legacy binary bypassed the current "
           "power, source, arbiter, and disabled-leg safety contracts. Follow "
           "/home/jetson/rinbo_ros_ws/docs/redrhex_sim2real_sbrio.md.\n";
    return 2;
}
