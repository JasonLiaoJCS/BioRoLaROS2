#include "rclcpp/rclcpp.hpp"
#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include "rinbo_msgs/msg/motor_state_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cmath>
#include <algorithm>

class PIDController : public rclcpp::Node {
public:
    PIDController() : Node("rinbo_pid_test") {
        // PD gain
        kp_ = 0.35f;
        ki_ = 0.0f;
        kd_ = 0.002f;
        
        // Feedforward gain
        k_ff_ = 0.02f;
        
        amplitude_ = 5000.0f;  
        frequency_ = 0.2f; 
        
        prev_error_ = 0.0f;
        integral_ = 0.0f;
        prev_position_ = 0.0f;
        
        max_pwm_ = 500.0f;
        integral_limit_ = 50000.0f;
        
        initialized_ = false;
        offset_ = 0.0f;

        cmd_pub_ = this->create_publisher<rinbo_msgs::msg::MotorCmdStamped>(
            "/motor/command", 10);
        
        pid_data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/pid/data", 10);
        
        state_sub_ = this->create_subscription<rinbo_msgs::msg::MotorStateStamped>(
            "/motor/state", 10,
            std::bind(&PIDController::state_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "PID Controller started (Feedforward test mode)");
        RCLCPP_INFO(this->get_logger(), "k_ff: %.4f, kp: %.4f, kd: %.4f", k_ff_, kp_, kd_);
    }

private:
    void state_callback(const rinbo_msgs::msg::MotorStateStamped::SharedPtr msg) {
        float current_position = msg->r1.position;
        auto now_time = this->now();
        
        if (!initialized_) {
            offset_ = current_position;
            start_time_ = now_time;
            prev_time_ = now_time;
            prev_position_ = current_position;
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Initialized! offset: %.1f", offset_);
            RCLCPP_INFO(this->get_logger(), "Tracking sin wave: %.0f + %.0f * sin(2pi * %.2f * t)", 
                        offset_, amplitude_, frequency_);
            return;
        }
        
        double elapsed = (now_time - start_time_).seconds();
        double dt = (now_time - prev_time_).seconds();
        if (dt <= 0) dt = 0.001;
        
        float target = offset_ + amplitude_ * std::sin(2.0 * M_PI * frequency_ * elapsed);
        float target_vel = amplitude_ * 2.0 * M_PI * frequency_ * std::cos(2.0 * M_PI * frequency_ * elapsed);
        float actual_vel = (current_position - prev_position_) / dt;
        
        float pos_error = target - current_position;
        float vel_error = target_vel - actual_vel;
        
        integral_ += pos_error;
        integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
        
        // PWM = kp * pos_error + kd * vel_error + k_ff * target_vel
        float pwm_fb = kp_ * pos_error + kd_ * vel_error;  // feedback
        float pwm_ff = k_ff_ * target_vel;                  // feedforward
        float pwm = pwm_fb + pwm_ff;
        pwm = std::clamp(pwm, -max_pwm_, max_pwm_);
        
        auto cmd = rinbo_msgs::msg::MotorCmdStamped();
        
        cmd.r1.enable = true;
        cmd.r1.direction = (pwm >= 0);
        cmd.r1.voltage = std::fabs(pwm);
        cmd.r1.state = 1;
        cmd.r1.reset_position = false;
        
        cmd.l1.enable = false;
        cmd.l2.enable = false;
        cmd.l3.enable = false;
        cmd.r2.enable = false;
        cmd.r3.enable = false;
        
        cmd.servo_control_mode = 2;
        
        cmd_pub_->publish(cmd);
        
        // PID data: [time, target_pos, actual_pos, target_vel, actual_vel, pwm]
        auto pid_data = std_msgs::msg::Float32MultiArray();
        pid_data.data = {
            static_cast<float>(elapsed),
            target,
            current_position,
            target_vel,
            actual_vel,
            pwm
        };
        pid_data_pub_->publish(pid_data);
        
        prev_error_ = pos_error;
        prev_position_ = current_position;
        prev_time_ = now_time;
        
        RCLCPP_INFO(this->get_logger(), 
            "tgt_v: %.1f, act_v: %.1f, pwm_ff: %.1f, pwm: %.1f, pos_err: %.1f", 
            target_vel, actual_vel, pwm_ff, pwm, pos_error);
    }

    float kp_, ki_, kd_;
    float k_ff_;  // feedforward gain
    float prev_error_, integral_;
    float max_pwm_, integral_limit_;
    
    float amplitude_, frequency_, offset_;
    float prev_position_;
    rclcpp::Time start_time_;
    rclcpp::Time prev_time_;
    
    bool initialized_;
    
    rclcpp::Publisher<rinbo_msgs::msg::MotorCmdStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pid_data_pub_;
    rclcpp::Subscription<rinbo_msgs::msg::MotorStateStamped>::SharedPtr state_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDController>());
    rclcpp::shutdown();
    return 0;
}