#ifndef FPGA_SERVER_HPP
#define FPGA_SERVER_HPP

#include "fpga_handler.hpp"
#include "console.hpp"

#include <NodeHandler.h>
#include <sys/time.h>
#include <fstream>
#include <yaml.h>
#include <string>
#include <vector>
#include <mutex>

#ifndef CONFIG_PATH
#define CONFIG_PATH "/home/admin/rinbo_sbRIO_ws/rinbo_fpga_driver/config/config.yaml"
#endif

void inthand(int signum);
bool is_sys_stop();


void inthand(int signum);
bool is_sys_stop();

class Rinbo{
    public:
        Rinbo();
        ~Rinbo();

        FpgaHandler fpga_;
        YAML::Node yaml_node_;

        /* console */
        std::mutex main_mtx_;
        Console console_;

        /* header msg */
        struct timeval t_stamp;
        uint32_t seq;

        /* powerboard state */
        std::vector<bool> powerboard_state_;
        bool digital_switch_;
        bool signal_switch_;
        bool power_switch_;

        uint32_t servo_control_mode_;

        void load_config_();

        void interruptHandler(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
            core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
            core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
            core::Publisher<motor_msg::MotorStateStamped>& state_pub_);

        void powerboardPack(power_msg::PowerStateStamped &power_fb_msg);

        void motorPack(motor_msg::MotorStateStamped& motor_state_reply);

        void processMotorCommands();

        void mainLoop_(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
            core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
            core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
            core::Publisher<motor_msg::MotorStateStamped>& state_pub_);

};

#endif