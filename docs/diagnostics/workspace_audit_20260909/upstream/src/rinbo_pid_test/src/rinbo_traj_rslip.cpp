#include "rclcpp/rclcpp.hpp"
#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include "rinbo_msgs/msg/motor_state_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <array>
#include <csignal>

std::atomic<bool> g_shutdown_requested{false};

void signal_handler(int signum) {
    (void)signum;
    g_shutdown_requested = true;
}

class PIDController : public rclcpp::Node {
public:
    PIDController() : Node("rinbo_tripod_rslip") {
        kp_ = 0.38f;
        kd_ = 0.003f;
        k_ff_ = 0.005f;
        
        poly_matlab_ = {413358.149203744, -95507.5174225401, 334.637824332724, 
                        -2090.88289174918, 1001.29584712994, -48.8132486352745};
        
        poly_.assign(poly_matlab_.rbegin(), poly_matlab_.rend());
        
        t_stance_ = 0.176352417699870;
        t_flight_ = 0.217200939592260;
        period_ = t_stance_ + t_flight_;
        
        br_ = 0.3;
        
        theta_LO_ = eval_poly_rad(t_stance_);
        theta_dot_LO_ = eval_poly_derivative_rad(t_stance_);
        
        theta_start_rslip_ = eval_poly_rad(0.0);
        theta_TD_ = theta_start_rslip_ + 2.0 * M_PI;
        theta_dot_TD_ = eval_poly_derivative_rad(0.0);
        
        compute_trapezoid_params();
        
        rad_to_counts_ = 54984.83 / (2.0 * M_PI);
        
        theta_home_ = M_PI;
        startup_duration_ = 3.0;
        
        servo_targets_[0] = 740; 
        servo_targets_[1] = 2565; 
        servo_targets_[2] = 3283;
        servo_targets_[3] = 1944; 
        servo_targets_[4] = 2071; 
        servo_targets_[5] = 989;
        
        max_pwm_ = 2000.0f;
        
        initialized_ = false;
        cycle_count_ = 0;
        
        tau_ = 0.0;
        startup_time_ = 0.0;
        
        start_ratio_ = 8.0;
        current_ratio_ = 5.0;
        target_ratio_ = 1.0;  // 先用 2.0 測試
        ratio_step_ = -0.0002;
        slowdown_step_ = 0.002;
        
        state_ = State::STARTUP;
        fully_stopped_ = false;
        
        prev_pos_ = 0.0f;
        initial_pos_ = 0.0f;
        home_offset_ = 0.0f;
        
        // 誤差積分（可選）
        integral_error_ = 0.0f;
        ki_ = 0.0f;  // 先不用 I 項

        cmd_pub_ = this->create_publisher<rinbo_msgs::msg::MotorCmdStamped>(
            "/motor/command", 10);
        
        pid_data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/pid/data", 10);
        
        state_sub_ = this->create_subscription<rinbo_msgs::msg::MotorStateStamped>(
            "/motor/state", 10,
            std::bind(&PIDController::state_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "=== RSLIP Single Leg R1 Test ===");
        RCLCPP_INFO(this->get_logger(), "kp: %.3f | kd: %.4f | k_ff: %.4f | target_ratio: %.1f",
                    kp_, kd_, k_ff_, target_ratio_);
        RCLCPP_INFO(this->get_logger(), "theta_start: %.1f deg | theta_LO: %.1f deg",
                    theta_start_rslip_ * 180.0 / M_PI, theta_LO_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "w_top: %.2f rad/s | period: %.4f s",
                    w_top_, period_);
    }

private:
    enum class State {
        STARTUP,
        RUNNING,
        STOPPING
    };
    
    double eval_poly_rad(double t) {
        double result = 0.0;
        double t_pow = 1.0;
        for (size_t i = 0; i < poly_.size(); i++) {
            result += poly_[i] * t_pow;
            t_pow *= t;
        }
        return result * M_PI / 180.0;
    }
    
    double eval_poly_derivative_rad(double t) {
        double result = 0.0;
        double t_pow = 1.0;
        for (size_t i = 1; i < poly_.size(); i++) {
            result += i * poly_[i] * t_pow;
            t_pow *= t;
        }
        return result * M_PI / 180.0;
    }
    
    void compute_trapezoid_params() {
        double area = theta_TD_ - theta_LO_; 
        w_top_ = (area - (theta_dot_LO_ + theta_dot_TD_) * br_ * t_flight_ / 2.0) 
                 / ((1.0 - br_) * t_flight_);
        a1_ = (w_top_ - theta_dot_LO_) / (br_ * t_flight_);
        a2_ = (theta_dot_TD_ - w_top_) / (br_ * t_flight_);
    }
    
    void eval_trapezoid(double t, double& theta, double& theta_dot) {
        double t_f = t_flight_;
        double t_br = br_ * t_f;
        double t_1_br = (1.0 - br_) * t_f;
        
        if (t <= t_br) {
            theta = theta_LO_ + theta_dot_LO_ * t + 0.5 * a1_ * t * t;
            theta_dot = theta_dot_LO_ + a1_ * t;
        } else if (t <= t_1_br) {
            double theta_at_br = theta_LO_ + theta_dot_LO_ * t_br + 0.5 * a1_ * t_br * t_br;
            theta = theta_at_br + w_top_ * (t - t_br);
            theta_dot = w_top_;
        } else {
            double theta_at_br = theta_LO_ + theta_dot_LO_ * t_br + 0.5 * a1_ * t_br * t_br;
            double theta_at_1_br = theta_at_br + w_top_ * (t_1_br - t_br);
            double dt = t - t_1_br;
            theta = theta_at_1_br + w_top_ * dt + 0.5 * a2_ * dt * dt;
            theta_dot = w_top_ + a2_ * dt;
        }
    }
    
    void eval_cubic(double t, double T, double p0, double v0, double p1, double v1,
                    double& pos, double& vel) {
        if (t <= 0) { pos = p0; vel = v0; return; }
        if (t >= T) { pos = p1; vel = v1; return; }
        
        double a0 = p0;
        double a1 = v0;
        double a2 = (3.0 * (p1 - p0) / (T * T)) - (2.0 * v0 / T) - (v1 / T);
        double a3 = (-2.0 * (p1 - p0) / (T * T * T)) + ((v1 + v0) / (T * T));
        
        pos = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
        vel = a1 + 2.0 * a2 * t + 3.0 * a3 * t * t;
    }
    
    void compute_trajectory(double tau_local, double& target_rad, double& target_vel_rad, bool& stance_phase) {
        int n = static_cast<int>(std::floor(tau_local / period_));
        double t_local = std::fmod(tau_local, period_);
        
        if (t_local < 0) {
            t_local += period_;
            n -= 1;
        }
        
        if (t_local < t_stance_) {
            stance_phase = true;
            target_rad = eval_poly_rad(t_local);
            target_vel_rad = eval_poly_derivative_rad(t_local);
        } else {
            stance_phase = false;
            double t_in_flight = t_local - t_stance_;
            eval_trapezoid(t_in_flight, target_rad, target_vel_rad);
        }
        
        target_rad += n * 2.0 * M_PI;
        target_vel_rad /= current_ratio_;
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
        
        if (g_shutdown_requested && state_ != State::STOPPING) {
            state_ = State::STOPPING;
            RCLCPP_INFO(this->get_logger(), "=== Shutdown requested, slowing down... ===");
        }
        
        if (fully_stopped_) {
            rclcpp::shutdown();
            return;
        }
        
        // R1 encoder (取負號讓方向一致)
        float pos = -msg->r1.position;
        
        if (!initialized_) {
            start_time_ = now_time;
            prev_time_ = now_time;
            cycle_count_ = 0;
            tau_ = 0.0;
            startup_time_ = 0.0;
            prev_pos_ = pos;
            initial_pos_ = pos;
            integral_error_ = 0.0f;
            
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Initialized! R1=%.0f", pos);
            return;
        }
        
        double dt = (now_time - prev_time_).seconds();
        if (dt <= 0) dt = 0.001;
        
        float pwm = 0.0f;
        float pos_error = 0.0f;
        float target_vel = 0.0f;
        bool stance = false;
        
        switch (state_) {
            case State::STARTUP: {
                startup_time_ += dt;
                
                double rslip_start_vel = eval_poly_derivative_rad(0.0) / start_ratio_;
                
                double p0 = initial_pos_;
                double v0 = 0.0;
                double p1 = initial_pos_ + theta_home_ * rad_to_counts_;
                double v1 = rslip_start_vel * rad_to_counts_;
                
                double tgt_pos, tgt_vel;
                eval_cubic(startup_time_, startup_duration_, p0, v0, p1, v1, tgt_pos, tgt_vel);
                
                float actual_vel = (pos - prev_pos_) / dt;
                pos_error = tgt_pos - pos;
                target_vel = tgt_vel;
                float vel_error = tgt_vel - actual_vel;
                
                pwm = kp_ * pos_error + kd_ * vel_error + k_ff_ * target_vel;
                pwm = std::clamp(pwm, -max_pwm_, max_pwm_);
                
                if (startup_time_ >= startup_duration_) {
                    state_ = State::RUNNING;
                    tau_ = 0.0;
                    current_ratio_ = start_ratio_;
                    home_offset_ = pos - theta_start_rslip_ * rad_to_counts_;
                    integral_error_ = 0.0f;
                    
                    RCLCPP_INFO(this->get_logger(), "=== Startup done, entering RUNNING ===");
                    RCLCPP_INFO(this->get_logger(), "home_offset: %.0f", home_offset_);
                }
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                    "[STARTUP] %.2f/%.2f s | pwm:%.0f err:%.0f", 
                    startup_time_, startup_duration_, pwm, pos_error);
                break;
            }
            
            case State::RUNNING: {
                // Ratio 緩降
                if (std::fabs(current_ratio_ - target_ratio_) > std::fabs(ratio_step_)) {
                    current_ratio_ += ratio_step_;
                } else {
                    current_ratio_ = target_ratio_;
                }
                if (current_ratio_ < target_ratio_) {
                    current_ratio_ = target_ratio_;
                }
                
                double d_tau = dt / current_ratio_;
                tau_ += d_tau;
                
                int n = static_cast<int>(std::floor(tau_ / period_));
                if (n > cycle_count_) {
                    cycle_count_ = n;
                    RCLCPP_INFO(this->get_logger(), "=== Cycle %d (ratio: %.2f) ===", 
                                cycle_count_, current_ratio_);
                }
                
                double target_rad, target_vel_rad;
                compute_trajectory(tau_, target_rad, target_vel_rad, stance);
                
                float tgt = home_offset_ + target_rad * rad_to_counts_;
                target_vel = target_vel_rad * rad_to_counts_;
                float actual_vel = (pos - prev_pos_) / dt;
                
                pos_error = tgt - pos;
                float vel_error = target_vel - actual_vel;
                
                // PID 計算
                pwm = kp_ * pos_error + kd_ * vel_error + k_ff_ * target_vel;
                pwm = std::clamp(pwm, -max_pwm_, max_pwm_);
                
                // 發布 PID 數據供分析
                auto pid_msg = std_msgs::msg::Float32MultiArray();
                pid_msg.data = {
                    static_cast<float>(tau_),
                    static_cast<float>(tgt),
                    pos,
                    pos_error,
                    target_vel,
                    actual_vel,
                    pwm,
                    static_cast<float>(stance ? 1.0 : 0.0)
                };
                pid_data_pub_->publish(pid_msg);
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                    "tau:%.3f r:%.2f %s | err:%.0f tv:%.0f av:%.0f pwm:%.0f", 
                    tau_, current_ratio_,
                    stance ? "ST" : "FL",
                    pos_error, target_vel, actual_vel, pwm);
                break;
            }
            
            case State::STOPPING: {
                current_ratio_ += slowdown_step_;
                
                if (current_ratio_ >= 10.0) {
                    RCLCPP_INFO(this->get_logger(), "=== Fully stopped ===");
                    
                    auto cmd = rinbo_msgs::msg::MotorCmdStamped();
                    cmd.l1.enable = false; cmd.l2.enable = false; cmd.l3.enable = false;
                    cmd.r1.enable = false; cmd.r2.enable = false; cmd.r3.enable = false;
                    set_servos(cmd);
                    cmd.servo_control_mode = 2;
                    cmd_pub_->publish(cmd);
                    
                    fully_stopped_ = true;
                    return;
                }
                
                double d_tau = dt / current_ratio_;
                tau_ += d_tau;
                
                double target_rad, target_vel_rad;
                compute_trajectory(tau_, target_rad, target_vel_rad, stance);
                
                float tgt = home_offset_ + target_rad * rad_to_counts_;
                target_vel = target_vel_rad * rad_to_counts_;
                float actual_vel = (pos - prev_pos_) / dt;
                
                pos_error = tgt - pos;
                float vel_error = target_vel - actual_vel;
                
                pwm = kp_ * pos_error + kd_ * vel_error + k_ff_ * target_vel;
                pwm = std::clamp(pwm, -max_pwm_, max_pwm_);
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                    "[STOPPING] r:%.2f | pwm:%.0f err:%.0f", 
                    current_ratio_, pwm, pos_error);
                break;
            }
        }
        
        // 只驅動 R1
        auto cmd = rinbo_msgs::msg::MotorCmdStamped();
        
        cmd.l1.enable = false; cmd.l1.voltage = 0; cmd.l1.state = 1; cmd.l1.reset_position = false;
        cmd.l2.enable = false; cmd.l2.voltage = 0; cmd.l2.state = 1; cmd.l2.reset_position = false;
        cmd.l3.enable = false; cmd.l3.voltage = 0; cmd.l3.state = 1; cmd.l3.reset_position = false;
        
        cmd.r1.enable = true;
        cmd.r1.direction = (pwm < 0);
        cmd.r1.voltage = std::fabs(pwm);
        cmd.r1.state = 1;
        cmd.r1.reset_position = false;
        
        cmd.r2.enable = false; cmd.r2.voltage = 0; cmd.r2.state = 1; cmd.r2.reset_position = false;
        cmd.r3.enable = false; cmd.r3.voltage = 0; cmd.r3.state = 1; cmd.r3.reset_position = false;
        
        set_servos(cmd);
        cmd.servo_control_mode = 2;
        
        cmd_pub_->publish(cmd);
        
        prev_pos_ = pos;
        prev_time_ = now_time;
    }

    float kp_, kd_, k_ff_, ki_;
    float max_pwm_;
    float integral_error_;
    
    std::vector<double> poly_matlab_;  
    std::vector<double> poly_; 
    
    double t_stance_, t_flight_, period_;
    double br_, rad_to_counts_;
    
    double theta_LO_, theta_TD_;
    double theta_dot_LO_, theta_dot_TD_;
    double w_top_, a1_, a2_;
    
    double theta_start_rslip_;
    double theta_home_;
    double startup_duration_;
    double startup_time_;
    
    std::array<uint32_t, 6> servo_targets_;
    
    double tau_;
    double current_ratio_, target_ratio_;
    double ratio_step_, slowdown_step_, start_ratio_;
    
    State state_;
    bool fully_stopped_;
    
    float initial_pos_;
    float home_offset_;
    float prev_pos_;
    int cycle_count_;
    rclcpp::Time start_time_;
    rclcpp::Time prev_time_;
    
    bool initialized_;
    
    rclcpp::Publisher<rinbo_msgs::msg::MotorCmdStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pid_data_pub_;
    rclcpp::Subscription<rinbo_msgs::msg::MotorStateStamped>::SharedPtr state_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    auto node = std::make_shared<PIDController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}