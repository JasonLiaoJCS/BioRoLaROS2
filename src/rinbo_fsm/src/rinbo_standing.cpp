#include "rclcpp/rclcpp.hpp"
#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include "rinbo_msgs/msg/motor_state_stamped.hpp"
#include <cmath>
#include <algorithm>
#include <array>

enum class LegState {
    FIND_HALL,
    ROTATE_180,
    DONE
};

class StandingController : public rclcpp::Node {
public:
    StandingController() : Node("rinbo_standing") {
        kp_ = 0.35f;
        kd_ = 0.002f;
        k_ff_ = 0.02f;
        
        rad_to_counts_ = 55296.0 / (2.0 * M_PI);
        
        target_vel_rad_ = 0.2 * M_PI;
        target_vel_counts_ = target_vel_rad_ * rad_to_counts_;
        
        rotate_180_counts_ = M_PI * rad_to_counts_;
        
        max_pwm_ = 500.0f;
        position_tolerance_ = 200.0f;
        
        servo_targets_[0] = 740; 
        servo_targets_[1] = 2565; 
        servo_targets_[2] = 3283;
        servo_targets_[3] = 1944; 
        servo_targets_[4] = 2071; 
        servo_targets_[5] = 989;
        
        for (int i = 0; i < 6; i++) {
            leg_states_[i] = LegState::FIND_HALL;
            hall_positions_[i] = 0.0f;
            target_positions_[i] = 0.0f;
            start_positions_[i] = 0.0f;
            prev_positions_[i] = 0.0f;
            rotate_start_times_[i] = this->now();
        }
        
        prev_time_ = this->now();
        start_time_ = this->now();
        first_msg_ = true;
        
        cmd_pub_ = this->create_publisher<rinbo_msgs::msg::MotorCmdStamped>(
            "/motor/command", 10);
        
        state_sub_ = this->create_subscription<rinbo_msgs::msg::MotorStateStamped>(
            "/motor/state", 10,
            std::bind(&StandingController::state_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "=== Standing Controller ===");
        RCLCPP_INFO(this->get_logger(), "Step 1: Find Hall (PID trajectory tracking)");
        RCLCPP_INFO(this->get_logger(), "Step 2: Rotate 180 degrees (PID)");
        RCLCPP_INFO(this->get_logger(), "Velocity: %.2f rad/s", target_vel_rad_);
    }

private:
    bool is_left_leg(int idx) {
        return (idx == 0 || idx == 1 || idx == 2);
    }
    
    const char* leg_name(int idx) {
        static const char* names[] = {"L1", "L2", "L3", "R1", "R2", "R3"};
        return names[idx];
    }
    
    void set_servos(rinbo_msgs::msg::MotorCmdStamped& cmd) {
        cmd.sl1.position_encoder = servo_targets_[0];
        cmd.sl2.position_encoder = servo_targets_[1];
        cmd.sl3.position_encoder = servo_targets_[2];
        cmd.sr1.position_encoder = servo_targets_[3];
        cmd.sr2.position_encoder = servo_targets_[4];
        cmd.sr3.position_encoder = servo_targets_[5];
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
        
        if (first_msg_) {
            start_time_ = now_time;
            for (int i = 0; i < 6; i++) {
                prev_positions_[i] = positions[i];
                start_positions_[i] = positions[i];
                rotate_start_times_[i] = now_time;
            }
            first_msg_ = false;
            prev_time_ = now_time;
            RCLCPP_INFO(this->get_logger(), "Started! All legs enabled.");
            RCLCPP_INFO(this->get_logger(), "Start pos: [%.0f %.0f %.0f %.0f %.0f %.0f]",
                        start_positions_[0], start_positions_[1], start_positions_[2],
                        start_positions_[3], start_positions_[4], start_positions_[5]);
            return;
        }
        
        std::array<float, 6> velocities;
        for (int i = 0; i < 6; i++) {
            float raw_vel = (positions[i] - prev_positions_[i]) / dt;
            velocities[i] = is_left_leg(i) ? -raw_vel : raw_vel;
        }
        
        auto cmd = rinbo_msgs::msg::MotorCmdStamped();
        disable_all_legs(cmd);
        set_servos(cmd);       
        cmd.servo_control_mode = 2; 
        
        int done_count = 0;
        double elapsed = (now_time - start_time_).seconds();
        
        for (int i = 0; i < 6; i++) {
            switch (leg_states_[i]) {
                case LegState::FIND_HALL: {
                    if (!hall_effects[i]) {
                        hall_positions_[i] = positions[i];
                        if (is_left_leg(i)) {
                            target_positions_[i] = hall_positions_[i] - rotate_180_counts_;
                        } else {
                            target_positions_[i] = hall_positions_[i] + rotate_180_counts_;
                        }
                        rotate_start_times_[i] = now_time;
                        leg_states_[i] = LegState::ROTATE_180;
                        RCLCPP_INFO(this->get_logger(), "%s: Hall at %.0f, target: %.0f", 
                                    leg_name(i), hall_positions_[i], target_positions_[i]);
                    } else {
                        // PID trajectory tracking
                        float target_pos = is_left_leg(i) ? 
                            start_positions_[i] - target_vel_counts_ * elapsed :
                            start_positions_[i] + target_vel_counts_ * elapsed;
                        float target_vel = target_vel_counts_;
                        
                        float pos_error = is_left_leg(i) ? 
                            -(target_pos - positions[i]) : (target_pos - positions[i]);
                        float vel_error = target_vel - velocities[i];
                        
                        float pwm = kp_ * pos_error + kd_ * vel_error + k_ff_ * target_vel;
                        pwm = std::clamp(pwm, -max_pwm_, max_pwm_);
                        set_leg_cmd(cmd, i, pwm);
                    }
                    break;
                }
                
                case LegState::ROTATE_180: {
                    double rotate_elapsed = (now_time - rotate_start_times_[i]).seconds();
                    double rotate_duration = M_PI / target_vel_rad_;
                    
                    float target_pos, target_vel;
                    
                    if (rotate_elapsed < rotate_duration) {
                        if (is_left_leg(i)) {
                            target_pos = hall_positions_[i] - target_vel_counts_ * rotate_elapsed;
                        } else {
                            target_pos = hall_positions_[i] + target_vel_counts_ * rotate_elapsed;
                        }
                        target_vel = target_vel_counts_;
                    } else {
                        target_pos = target_positions_[i];
                        target_vel = 0.0f;
                    }
                    
                    float pos_error = is_left_leg(i) ? 
                        -(target_pos - positions[i]) : (target_pos - positions[i]);
                    float vel_error = target_vel - velocities[i];
                    
                    float pwm = kp_ * pos_error + kd_ * vel_error + k_ff_ * target_vel;
                    pwm = std::clamp(pwm, -max_pwm_, max_pwm_);
                    set_leg_cmd(cmd, i, pwm);
                    
                    float final_error = std::fabs(target_positions_[i] - positions[i]);
                    if (rotate_elapsed >= rotate_duration && final_error < position_tolerance_) {
                        leg_states_[i] = LegState::DONE;
                        RCLCPP_INFO(this->get_logger(), "%s: DONE! pos: %.0f, err: %.0f", 
                                    leg_name(i), positions[i], final_error);
                    }
                    break;
                }
                
                case LegState::DONE: {
                    float pos_error = is_left_leg(i) ? 
                        -(target_positions_[i] - positions[i]) : (target_positions_[i] - positions[i]);
                    float pwm = 0.1f * pos_error;
                    pwm = std::clamp(pwm, -300.0f, 300.0f);
                    set_leg_cmd(cmd, i, pwm);
                    done_count++;
                    break;
                }
            }
        }
        
        cmd_pub_->publish(cmd);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Done: %d/6 | Hall: [%d %d %d %d %d %d]",
            done_count,
            hall_effects[0], hall_effects[1], hall_effects[2],
            hall_effects[3], hall_effects[4], hall_effects[5]);
        
        if (done_count == 6) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "=== ALL LEGS STANDING ===");
        }
        
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
    
    void set_leg_cmd(rinbo_msgs::msg::MotorCmdStamped& cmd, int leg_idx, float pwm) {
        auto set_leg = [&](auto& leg, bool invert_dir) {
            leg.enable = true;
            leg.direction = invert_dir ? (pwm < 0) : (pwm >= 0);
            leg.voltage = std::fabs(pwm);
            leg.state = 1;
            leg.reset_position = false;
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

    float kp_, kd_, k_ff_;
    float max_pwm_;
    float position_tolerance_;
    
    double rad_to_counts_;
    double target_vel_rad_;
    double target_vel_counts_;
    double rotate_180_counts_;
    
    bool first_msg_;
    
    std::array<uint32_t, 6> servo_targets_;
    
    std::array<LegState, 6> leg_states_;
    std::array<float, 6> hall_positions_;
    std::array<float, 6> target_positions_;
    std::array<float, 6> start_positions_;
    std::array<float, 6> prev_positions_;
    std::array<rclcpp::Time, 6> rotate_start_times_;
    rclcpp::Time prev_time_;
    rclcpp::Time start_time_;
    
    rclcpp::Publisher<rinbo_msgs::msg::MotorCmdStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<rinbo_msgs::msg::MotorStateStamped>::SharedPtr state_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandingController>());
    rclcpp::shutdown();
    return 0;
}