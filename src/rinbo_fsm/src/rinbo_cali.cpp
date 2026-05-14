#include "rclcpp/rclcpp.hpp"
#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include "rinbo_msgs/msg/motor_state_stamped.hpp"
#include <cmath>
#include <algorithm>
#include <array>

enum class CalibState {
    SERVO_HOMING,
    WAIT_SERVO,
    DC_SPINNING, 
    DONE
};

enum class LegState {
    SPINNING, 
    STOPPING, 
    DONE
};

class CalibrationFSM : public rclcpp::Node {
public:
    CalibrationFSM() : Node("rinbo_cali") {
        kp_ = 0.35f;
        kd_ = 0.002f;
        k_ff_ = 0.02f;
        
        rad_to_counts_ = 55296.0 / (2.0 * M_PI);
        
        target_vel_rad_ = 0.2 * M_PI;
        target_vel_counts_ = target_vel_rad_ * rad_to_counts_;

        servo_targets_[0] = 740; 
        servo_targets_[1] = 2565;
        servo_targets_[2] = 3283;
        servo_targets_[3] = 1944;
        servo_targets_[4] = 2071;
        servo_targets_[5] = 989; 
        
        servo_tolerance_ = 100;
        
        state_ = CalibState::SERVO_HOMING; 
        initialized_ = false;
        
        for (int i = 0; i < 6; i++) {
            leg_states_[i] = LegState::SPINNING;
            start_positions_[i] = 0.0f;
            prev_positions_[i] = 0.0f;
            stop_start_times_[i] = this->now();
        }
        
        prev_time_ = this->now();
        start_time_ = this->now();
        state_start_time_ = this->now(); 
        
        max_pwm_ = 500.0f;
        stop_velocity_threshold_ = 500.0f;
        
        cmd_pub_ = this->create_publisher<rinbo_msgs::msg::MotorCmdStamped>(
            "/motor/command", 10);
        
        state_sub_ = this->create_subscription<rinbo_msgs::msg::MotorStateStamped>(
            "/motor/state", 10,
            std::bind(&CalibrationFSM::state_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "=== 6-Leg Calibration FSM Started ===");
        RCLCPP_INFO(this->get_logger(), "Servo targets: sl1=%u, sl2=%u, sl3=%u, sr1=%u, sr2=%u, sr3=%u",
                    servo_targets_[0], servo_targets_[1], servo_targets_[2],
                    servo_targets_[3], servo_targets_[4], servo_targets_[5]);
        RCLCPP_INFO(this->get_logger(), "DC target velocity: %.2f rad/s", target_vel_rad_);
        RCLCPP_INFO(this->get_logger(), "State: SERVO_HOMING");
    }

private:
    bool is_left_leg(int idx) {
        return (idx == 0 || idx == 1 || idx == 2);
    }

    const char* leg_name(int idx) {
        static const char* names[] = {"L1", "L2", "L3", "R1", "R2", "R3"};
        return names[idx];
    }
    
    void state_callback(const rinbo_msgs::msg::MotorStateStamped::SharedPtr msg) {
        auto now_time = this->now();
        float dt = (now_time - prev_time_).seconds();
        if (dt <= 0) dt = 0.001;
        
        std::array<float, 6> positions = {
            msg->l1.position, msg->l2.position, msg->l3.position,
            msg->r1.position, msg->r2.position, msg->r3.position
        };
        
        std::array<bool, 6> hall_effects = {
            msg->l1.hall_effect, msg->l2.hall_effect, msg->l3.hall_effect,
            msg->r1.hall_effect, msg->r2.hall_effect, msg->r3.hall_effect
        };

        std::array<uint32_t, 6> servo_positions = {
            msg->sl1.position_encoder, msg->sl2.position_encoder, msg->sl3.position_encoder,
            msg->sr1.position_encoder, msg->sr2.position_encoder, msg->sr3.position_encoder
        };
        
        std::array<float, 6> velocities;
        for (int i = 0; i < 6; i++) {
            float raw_vel = (positions[i] - prev_positions_[i]) / dt;
            velocities[i] = is_left_leg(i) ? -raw_vel : raw_vel;
        }
        
        auto cmd = rinbo_msgs::msg::MotorCmdStamped();
        disable_all_legs(cmd);
        set_servos(cmd);
        cmd.servo_control_mode = 2;
        
        switch (state_) {
            case CalibState::SERVO_HOMING:
                handle_servo_homing(cmd, servo_positions, now_time);
                break;
                
            case CalibState::WAIT_SERVO:
                handle_wait_servo(cmd, servo_positions, now_time);
                break;

            case CalibState::DC_SPINNING:
                handle_dc_spinning(cmd, positions, velocities, hall_effects, now_time);
                break;
                
            case CalibState::DONE:
                handle_done(cmd, positions);
                break;
        }
        
        cmd_pub_->publish(cmd);
        
        for (int i = 0; i < 6; i++) {
            prev_positions_[i] = positions[i];
        }
        prev_time_ = now_time;
    }
    
    void disable_all_legs(rinbo_msgs::msg::MotorCmdStamped& cmd) {
        cmd.l1.enable = false; cmd.l1.voltage = 0; cmd.l1.state = 0; cmd.l1.reset_position = false;
        cmd.l2.enable = false; cmd.l2.voltage = 0; cmd.l2.state = 0; cmd.l2.reset_position = false;
        cmd.l3.enable = false; cmd.l3.voltage = 0; cmd.l3.state = 0; cmd.l3.reset_position = false;
        cmd.r1.enable = false; cmd.r1.voltage = 0; cmd.r1.state = 0; cmd.r1.reset_position = false;
        cmd.r2.enable = false; cmd.r2.voltage = 0; cmd.r2.state = 0; cmd.r2.reset_position = false;
        cmd.r3.enable = false; cmd.r3.voltage = 0; cmd.r3.state = 0; cmd.r3.reset_position = false;
    }

    void set_servos(rinbo_msgs::msg::MotorCmdStamped& cmd) {
        cmd.sl1.position_encoder = servo_targets_[0];
        cmd.sl2.position_encoder = servo_targets_[1];
        cmd.sl3.position_encoder = servo_targets_[2];
        cmd.sr1.position_encoder = servo_targets_[3];
        cmd.sr2.position_encoder = servo_targets_[4];
        cmd.sr3.position_encoder = servo_targets_[5];
    }
    
    void set_leg_cmd(rinbo_msgs::msg::MotorCmdStamped& cmd, int leg_idx, 
                     bool enable, float pwm, bool reset_position) {
        auto set_leg = [&](auto& leg, bool invert_dir) {
            leg.enable = enable;
            leg.direction = invert_dir ? (pwm < 0) : (pwm >= 0);
            leg.voltage = std::fabs(pwm);
            leg.state = enable ? 1 : 0;
            leg.reset_position = reset_position;
        };
        
        switch (leg_idx) {
            case 0: set_leg(cmd.l1, true); break;
            case 1: set_leg(cmd.l2, true); break;
            case 2: set_leg(cmd.l3, true); break;
            case 3: set_leg(cmd.r1, false); break;
            case 4: set_leg(cmd.r2, false); break;
            case 5: set_leg(cmd.r3, false); break; 
        }
    }

    void handle_servo_homing(rinbo_msgs::msg::MotorCmdStamped& cmd,
                             const std::array<uint32_t, 6>& servo_positions,
                             rclcpp::Time now_time) {
        RCLCPP_INFO(this->get_logger(), "SERVO_HOMING | Servos: [%u %u %u %u %u %u]",
                    servo_positions[0], servo_positions[1], servo_positions[2],
                    servo_positions[3], servo_positions[4], servo_positions[5]);
        
        state_ = CalibState::WAIT_SERVO;
        state_start_time_ = now_time;
        RCLCPP_INFO(this->get_logger(), "State: WAIT_SERVO");
    }
    
    void handle_wait_servo(rinbo_msgs::msg::MotorCmdStamped& cmd,
                           const std::array<uint32_t, 6>& servo_positions,
                           rclcpp::Time now_time) {
        bool all_homed = true;
        for (int i = 0; i < 6; i++) {
            int32_t error = static_cast<int32_t>(servo_targets_[i]) - 
                           static_cast<int32_t>(servo_positions[i]);
            if (std::abs(error) > static_cast<int32_t>(servo_tolerance_)) {
                all_homed = false;
            }
        }
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "WAIT_SERVO | Servos: [%u %u %u %u %u %u] | All homed: %s",
            servo_positions[0], servo_positions[1], servo_positions[2],
            servo_positions[3], servo_positions[4], servo_positions[5],
            all_homed ? "YES" : "NO");
        
        if (all_homed) {
            if ((now_time - state_start_time_).seconds() > 0.5) {
                state_ = CalibState::DC_SPINNING;
                start_time_ = now_time;
                initialized_ = false;
                RCLCPP_INFO(this->get_logger(), "All servos homed! State: DC_SPINNING");
            }
        } else {
            state_start_time_ = now_time; 
        }
    }
    
    void handle_dc_spinning(rinbo_msgs::msg::MotorCmdStamped& cmd,
                            const std::array<float, 6>& positions,
                            const std::array<float, 6>& velocities,
                            const std::array<bool, 6>& hall_effects,
                            rclcpp::Time now_time) {
        if (!initialized_) {
            start_time_ = now_time;
            for (int i = 0; i < 6; i++) {
                start_positions_[i] = positions[i];
            }
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "DC motors initialized!");
        }
        
        double elapsed = (now_time - start_time_).seconds();
        int done_count = 0;
        
        for (int i = 0; i < 6; i++) {
            switch (leg_states_[i]) {
                case LegState::SPINNING: {
                    if (!hall_effects[i]) {
                        leg_states_[i] = LegState::STOPPING;
                        stop_start_times_[i] = now_time;
                        RCLCPP_INFO(this->get_logger(), "%s: Hall detected! Stopping...", leg_name(i));
                    } else {
                        float target_pos = is_left_leg(i) ? 
                            start_positions_[i] - target_vel_counts_ * elapsed :
                            start_positions_[i] + target_vel_counts_ * elapsed;
                        float target_vel = target_vel_counts_;
                        
                        float pos_error = is_left_leg(i) ? 
                            -(target_pos - positions[i]) : (target_pos - positions[i]);
                        float vel_error = target_vel - velocities[i];
                        
                        float pwm = kp_ * pos_error + kd_ * vel_error + k_ff_ * target_vel;
                        pwm = std::clamp(pwm, -max_pwm_, max_pwm_);
                        
                        set_leg_cmd(cmd, i, true, pwm, false);
                    }
                    break;
                }
                
                case LegState::STOPPING: {
                    set_leg_cmd(cmd, i, true, 0, false);
                    
                    if (std::fabs(velocities[i]) < stop_velocity_threshold_) {
                        if ((now_time - stop_start_times_[i]).seconds() > 0.3) {
                            set_leg_cmd(cmd, i, false, 0, true);
                            leg_states_[i] = LegState::DONE;
                            RCLCPP_INFO(this->get_logger(), "%s: DONE! Position reset.", leg_name(i));
                        }
                    } else {
                        stop_start_times_[i] = now_time;
                    }
                    break;
                }
                
                case LegState::DONE: {
                    set_leg_cmd(cmd, i, false, 0, false);
                    done_count++;
                    break;
                }
            }
        }
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "DC_SPINNING | t: %.2f | Done: %d/6 | Hall: [%d %d %d %d %d %d]",
            elapsed, done_count,
            hall_effects[0], hall_effects[1], hall_effects[2],
            hall_effects[3], hall_effects[4], hall_effects[5]);
        
        if (done_count == 6) {
            state_ = CalibState::DONE;
            RCLCPP_INFO(this->get_logger(), "=== ALL LEGS CALIBRATED ===");
            RCLCPP_INFO(this->get_logger(), "State: DONE (Press Ctrl+C to continue)");
        }
    }
    
    void handle_done(rinbo_msgs::msg::MotorCmdStamped& cmd,
                     const std::array<float, 6>& positions) {
        disable_all_legs(cmd);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "DONE | Positions: L1=%.0f, L2=%.0f, L3=%.0f, R1=%.0f, R2=%.0f, R3=%.0f",
            positions[0], positions[1], positions[2],
            positions[3], positions[4], positions[5]);
    }

    float kp_, kd_, k_ff_;
    float max_pwm_;
    float stop_velocity_threshold_;
    
    double rad_to_counts_;
    double target_vel_rad_;
    double target_vel_counts_;
    
    std::array<uint32_t, 6> servo_targets_;
    uint32_t servo_tolerance_;
    
    CalibState state_;
    bool initialized_;
    
    std::array<LegState, 6> leg_states_;
    std::array<float, 6> start_positions_;
    std::array<float, 6> prev_positions_;
    std::array<rclcpp::Time, 6> stop_start_times_;
    rclcpp::Time prev_time_;
    rclcpp::Time start_time_;
    rclcpp::Time state_start_time_; 
    
    rclcpp::Publisher<rinbo_msgs::msg::MotorCmdStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<rinbo_msgs::msg::MotorStateStamped>::SharedPtr state_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalibrationFSM>());
    rclcpp::shutdown();
    return 0;
}
