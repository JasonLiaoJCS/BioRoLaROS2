#include "fpga_server.hpp"
#include "driver_lifecycle.hpp"

namespace lifecycle {
volatile sig_atomic_t stop_signal = 0;
std::atomic<bool> terminal_lost{false};
bool headless = false;
}

volatile int motor_message_updated = 0;
volatile int fpga_message_updated = 0;

std::mutex mutex_;
int log_counter = 0;
motor_msg::MotorCmdStamped motor_cmd_data;
void motor_data_cb(motor_msg::MotorCmdStamped motor_msg)
{
    mutex_.lock();
    if (lifecycle::stopping()) { mutex_.unlock(); return; }
    motor_message_updated = 1;
    motor_cmd_data = motor_msg;
    mutex_.unlock();
}

power_msg::PowerCmdStamped power_cmd_data;
void power_data_cb(power_msg::PowerCmdStamped power_msg)
{
    mutex_.lock();
    if (lifecycle::stopping()) { mutex_.unlock(); return; }
    fpga_message_updated = 1;
    power_cmd_data = power_msg;
    mutex_.unlock();
}

void inthand(int signum)
{
    lifecycle::signal_stop(signum);
}

bool is_sys_stop()
{
    return lifecycle::stopping();
}

Rinbo::Rinbo()
{
    if (NiFpga_IsError(fpga_.status_) || lifecycle::stopping())
        throw std::runtime_error("FPGA startup failed or interrupted");
    seq = 0;
    digital_switch_ = false;
    signal_switch_ = false;
    power_switch_ = false;

    servo_control_mode_ = 1;
    
    powerboard_state_.push_back(digital_switch_);
    powerboard_state_.push_back(signal_switch_);
    powerboard_state_.push_back(power_switch_);

    // std::cout << "[Debug] Attempting to write CM = 2..." << std::endl;
    fpga_.moduleIO.write_cm_(1);
    
    usleep(10000); 

    uint16_t immediate_check = fpga_.moduleIO.read_cm_();
    std::cout << "[Debug] Immediate Readback: " << immediate_check << std::endl;

    bool stop_status = fpga_.moduleIO.read_stop_();
    std::cout << "[Debug] FPGA Stop Status: " << stop_status << std::endl;

    load_config_();
    console_.init(&fpga_, &powerboard_state_, &main_mtx_);
}


Rinbo::~Rinbo()
{
    console_.stop();
    fpga_.safe_shutdown();
}

void Rinbo::load_config_()
{
    yaml_node_ = YAML::LoadFile(CONFIG_PATH);

    YAML::Node Factors_node_ = yaml_node_["Powerboard_Scaling_Factor"];
    int idx_ = 0;

    std::cout << "PowerBoard Scaling Factor" << std::endl;
    for (auto f : Factors_node_)
    {
        fpga_.powerboard_Ifactor[idx_] = f["Current_Factor"].as<double>();
        fpga_.powerboard_Ioffset[idx_] = f["Current_Offset"].as<double>();
        fpga_.powerboard_Vfactor[idx_] = f["Voltage_Factor"].as<double>();
        fpga_.powerboard_Voffset[idx_] = f["Voltage_Offset"].as<double>();
        std::cout   << "Index " << idx_ 
                    << " Current Factor: " << fpga_.powerboard_Ifactor[idx_]
                    << ", Current Offset: " << fpga_.powerboard_Ioffset[idx_]
                    << std::endl
                    << " Voltage Factor: " << fpga_.powerboard_Vfactor[idx_]
                    << ", Voltage Offset: " << fpga_.powerboard_Voffset[idx_]
                    << std::endl;
        idx_++;
    }
}


void Rinbo::interruptHandler(
				core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
				core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
				core::Publisher<motor_msg::MotorStateStamped>& state_pub_)
{
    while (!lifecycle::stopping())
    {
        {
            std::lock_guard<std::mutex> device_lock(main_mtx_);
            if (NiFpga_IsError(fpga_.status_) || NiFpga_IsError(fpga_.moduleIO.status_)) break;
        }
        mainLoop_(cmd_pb_sub_, state_pb_pub_, cmd_sub_, state_pub_);
        if (NiFpga_IsError(fpga_.status_))
        {
            std::cout << red << "[FPGA Server] Error! LabVIEW code: " << fpga_.status_ << reset << std::endl;
        }
        usleep(1000); 
    }
}


void Rinbo::powerboardPack(power_msg::PowerStateStamped&power_dashboard_reply)
{   
    
    mutex_.lock();
    gettimeofday(&t_stamp, NULL);
    power_dashboard_reply.mutable_header()->set_seq(seq++);
    power_dashboard_reply.mutable_header()->mutable_stamp()->set_sec(t_stamp.tv_sec);
    power_dashboard_reply.mutable_header()->mutable_stamp()->set_usec(t_stamp.tv_usec);

    power_dashboard_reply.set_digital(powerboard_state_.at(0));
    power_dashboard_reply.set_signal(powerboard_state_.at(1));
    power_dashboard_reply.set_power(powerboard_state_.at(2));

    power_dashboard_reply.set_v_0(fpga_.powerboard_V_list_[0]);
    power_dashboard_reply.set_i_0(fpga_.powerboard_I_list_[0]);

    power_dashboard_reply.set_v_1(fpga_.powerboard_V_list_[1]);
    power_dashboard_reply.set_i_1(fpga_.powerboard_I_list_[1]);

    power_dashboard_reply.set_v_2(fpga_.powerboard_V_list_[2]);
    power_dashboard_reply.set_i_2(fpga_.powerboard_I_list_[2]);

    power_dashboard_reply.set_v_3(fpga_.powerboard_V_list_[3]);
    power_dashboard_reply.set_i_3(fpga_.powerboard_I_list_[3]);

    power_dashboard_reply.set_v_4(fpga_.powerboard_V_list_[4]);
    power_dashboard_reply.set_i_4(fpga_.powerboard_I_list_[4]);

    power_dashboard_reply.set_v_5(fpga_.powerboard_V_list_[5]);
    power_dashboard_reply.set_i_5(fpga_.powerboard_I_list_[5]);

    power_dashboard_reply.set_v_6(fpga_.powerboard_V_list_[6]);
    power_dashboard_reply.set_i_6(fpga_.powerboard_I_list_[6]);

    power_dashboard_reply.set_v_7(fpga_.powerboard_V_list_[7]);
    power_dashboard_reply.set_i_7(fpga_.powerboard_I_list_[7]);
    mutex_.unlock();
}

void Rinbo::motorPack(motor_msg::MotorStateStamped& motor_state_reply)
{   
    mutex_.lock();
    
    gettimeofday(&t_stamp, NULL);
    motor_state_reply.mutable_header()->set_seq(seq); 
    motor_state_reply.mutable_header()->mutable_stamp()->set_sec(t_stamp.tv_sec);
    motor_state_reply.mutable_header()->mutable_stamp()->set_usec(t_stamp.tv_usec);

    motor_msg::LegState* legs[6] = {
        motor_state_reply.mutable_l1(),
        motor_state_reply.mutable_l2(),
        motor_state_reply.mutable_l3(),
        motor_state_reply.mutable_r1(),
        motor_state_reply.mutable_r2(),
        motor_state_reply.mutable_r3()
    };
    for(int i = 0; i < 6; i++) {
        auto* motor = legs[i];

        float current_position = (float)fpga_.moduleIO.read_ep_(i);
        uint32_t current_tick  = fpga_.moduleIO.read_tc_(i);
        bool hall_effect       = fpga_.moduleIO.read_he_(i);

        motor->set_position(current_position);
        motor->set_tick_count(current_tick);
        motor->set_hall_effect(hall_effect);

    }

    motor_msg::ServoState* servos[6] = {
        motor_state_reply.mutable_sl1(),
        motor_state_reply.mutable_sl2(),
        motor_state_reply.mutable_sl3(),
        motor_state_reply.mutable_sr1(),
        motor_state_reply.mutable_sr2(),
        motor_state_reply.mutable_sr3()
    };
    
    for(int i = 0; i < 6; i++) {
        uint16_t encoder_value = fpga_.moduleIO.read_position_encoder_(i);
        servos[i]->set_position_encoder(encoder_value);
    }
    if (seq % 1000 == 0) {
    std::cout << "[motorPack] Set encoder: " 
              << motor_state_reply.sl1().position_encoder() << ", "
              << motor_state_reply.sl2().position_encoder() << ", "
              << motor_state_reply.sl3().position_encoder() << ", "
              << motor_state_reply.sr1().position_encoder() << ", "
              << motor_state_reply.sr2().position_encoder() << ", "
              << motor_state_reply.sr3().position_encoder() << std::endl;
    }
    
    motor_state_reply.set_servo_control_mode(servo_control_mode_);

    mutex_.unlock();
}

void Rinbo::processMotorCommands()
{
    if (lifecycle::stopping() || !motor_message_updated) return;
    
    const motor_msg::LegCmd* cmds[6] = {
        &motor_cmd_data.l1(),
        &motor_cmd_data.l2(),
        &motor_cmd_data.l3(),
        &motor_cmd_data.r1(),
        &motor_cmd_data.r2(),
        &motor_cmd_data.r3()
    };
    
    for(int i = 0; i < 6; i++) {
        const auto* cmd = cmds[i]; 

        
        fpga_.moduleIO.write_en_(i, cmd->enable());
        fpga_.moduleIO.write_dir_(i, cmd->direction());
        float v = cmd->voltage();
	if (v < 0.0f || v > 4096.0f)
	      printf("[WARN] leg %d voltage clamped: %f\n", i, v);
        if (v < 0.0f) v = 0.0f;    
	if (v > 4096.0f) v = 4096.0f;
	fpga_.moduleIO.write_iv_(i, (uint16_t)v); 
        fpga_.moduleIO.write_state_(i, NiFpga_True);
        
        if(cmd->reset_position()) {
            fpga_.moduleIO.write_rp_(i, NiFpga_True);
            fpga_.moduleIO.write_rp_(i, NiFpga_False);
            
        }
    }

    const motor_msg::ServoCmd* servo_cmds[6] = {
        &motor_cmd_data.sl1(),
        &motor_cmd_data.sl2(),
        &motor_cmd_data.sl3(),
        &motor_cmd_data.sr1(),
        &motor_cmd_data.sr2(),
        &motor_cmd_data.sr3()
    };
    
    for(int i = 0; i < 6; i++) {
        uint32_t target_position = servo_cmds[i]->position_encoder();
        fpga_.moduleIO.write_position_bus_(i, target_position);
    }
    
    servo_control_mode_ = motor_cmd_data.servo_control_mode();
    fpga_.moduleIO.write_cm_(servo_control_mode_);
    
    motor_message_updated = 0;
}


void Rinbo::mainLoop_(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
                      core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                      core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
				      core::Publisher<motor_msg::MotorStateStamped>& state_pub_)
{
    core::spinOnce();
    std::lock_guard<std::mutex> device_lock(main_mtx_);
    if (lifecycle::stopping() || NiFpga_IsError(fpga_.status_) ||
        NiFpga_IsError(fpga_.moduleIO.status_)) return;
    mutex_.lock();
    processMotorCommands();
    if (fpga_message_updated)
    {
        powerboard_state_.at(0) = power_cmd_data.digital();
        powerboard_state_.at(1) = power_cmd_data.signal();
        powerboard_state_.at(2) = power_cmd_data.power();
        fpga_message_updated = 0;
    }
    mutex_.unlock();
    fpga_.write_powerboard_(&powerboard_state_);
    fpga_.read_powerboard_data_();
    power_msg::PowerStateStamped power_fb_msg;
    motor_msg::MotorStateStamped motor_fb_msg;
    powerboardPack(power_fb_msg);
    motorPack(motor_fb_msg);
    state_pb_pub_.publish(power_fb_msg);
    state_pub_.publish(motor_fb_msg);

}


int main(int argc, char* argv[])
{
    if (!getenv("CORE_MASTER_ADDR")) {
        setenv("CORE_MASTER_ADDR", "192.168.30.164:50051", 1);
    }
    
    if (!getenv("CORE_LOCAL_IP")) {
        setenv("CORE_LOCAL_IP", "192.168.30.12", 1);     
    }
    // Parse and validate before any FpgaHandler construction / NI initialization.
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "fpga_driver [--headless|--interactive]\n"
                     "No args: legacy auto-TTY mode. Monitoring must use logs/ROS.\n";
        return 0;
    }
    if (argc > 2 || (argc == 2 && std::string(argv[1]) != "--headless" &&
                     std::string(argv[1]) != "--interactive")) return 64;
    lifecycle::headless = argc == 2 && std::string(argv[1]) == "--headless";
    if (argc == 2 && std::string(argv[1]) == "--interactive" &&
        (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO))) return 64;
    try {
    lifecycle::install_signals();
    lifecycle::DriverLock driver_lock;
    lifecycle::event(lifecycle::headless ? "start headless" : "start auto_or_interactive");
    if (lifecycle::stopping()) return 2;

    important_message("[FPGA Server] : Launched");
    
    Rinbo rinbo;
    
    std::cout << "[Debug] Rinbo init done. Entering loop..." << std::endl;
    /* gRPC Topic */
    core::NodeHandler nh;

    core::Publisher<power_msg::PowerStateStamped>& power_pub = nh.advertise<power_msg::PowerStateStamped>("power/state");
    core::Subscriber<power_msg::PowerCmdStamped>& power_sub = nh.subscribe<power_msg::PowerCmdStamped>("power/command", 1000, power_data_cb);

    core::Publisher<motor_msg::MotorStateStamped>& motor_pub = nh.advertise<motor_msg::MotorStateStamped>("motor/state");
    core::Subscriber<motor_msg::MotorCmdStamped>& motor_sub = nh.subscribe<motor_msg::MotorCmdStamped>("motor/command", 1000, motor_data_cb);

    rinbo.interruptHandler(power_sub, power_pub, motor_sub, motor_pub);

    // Stop/join UI before off writes. No command callback is consumed afterward.
    rinbo.console_.stop();
    bool fault = NiFpga_IsError(rinbo.fpga_.status_) ||
                 NiFpga_IsError(rinbo.fpga_.moduleIO.status_);
    bool off_ok = rinbo.fpga_.safe_shutdown();
    const int code = !off_ok ? 3 : (fault || lifecycle::terminal_lost ||
                     lifecycle::stop_signal == SIGHUP || lifecycle::stop_signal == SIGPIPE ? 2 : 0);
    const std::string result = "exit reason_signal=" + std::to_string(lifecycle::stop_signal) +
        " terminal_lost=" + std::to_string(lifecycle::terminal_lost.load()) +
        " code=" + std::to_string(code);
    lifecycle::event(result.c_str());
    return code;
    } catch (const std::exception& error) {
        // Rinbo / FpgaHandler destructors attempt off writes during unwinding.
        lifecycle::event(error.what());
        lifecycle::event("exit exception code=3 PowerUnknown");
        return 3;
    }
}
