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
        
        poly_matlab_ = {499226.851105064, -124169.690772846, 2572.43821028774, 
                        -2180.08626086051, 1013.75966324102, -48.7991629707687};
        poly_.assign(poly_matlab_.rbegin(), poly_matlab_.rend());
        
        t_stance_ = 0.175887413151118;
        t_flight_ = 0.227629442432964;
        period_ = t_stance_ + t_flight_;
        
        phase_offset_B_ = period_ / 2.0;
        
        br_ = 0.3;
        
        // 找到 poly(t) = 0 的時間點（Stance 中心 = Standing）
        t_center_ = find_poly_zero();
        
        theta_LO_ = eval_poly_rad(t_stance_);
        theta_dot_LO_ = eval_poly_derivative_rad(t_stance_);
        
        // Standing 位置對應 poly = 0
        theta_start_rslip_ = 0.0;  // Standing 時 poly = 0
        
        // TD 點 = Standing + 2π（一圈後回到 TD）
        theta_TD_ = eval_poly_rad(0.0) + 2.0 * M_PI;  // TD 位置 + 2π
        theta_dot_TD_ = eval_poly_derivative_rad(0.0);
        
        // Standing 時的速度
        theta_dot_center_ = eval_poly_derivative_rad(t_center_);
        
        compute_trapezoid_params();
        
        rad_to_counts_ = 54984.83 / (2.0 * M_PI);
        
        theta_home_ = 2 * M_PI;
        
        // 啟動階段時間
        startup_duration_ = 8.0;
        
        servo_targets_[0] = 740; 
        servo_targets_[1] = 2565; 
        servo_targets_[2] = 3283;
        servo_targets_[3] = 1944; 
        servo_targets_[4] = 2071; 
        servo_targets_[5] = 989;
        
        max_pwm_ = 3300.0f;
        
        initialized_ = false;
        cycle_count_ = 0;
        
        tau_ = 0.0;
        startup_time_ = 0.0;
        
        start_ratio_ = 8.0;
        current_ratio_ = 5.0;
        target_ratio_ = 2.0;
        ratio_step_ = -0.0002;
        slowdown_step_ = 0.002;
        
        state_ = State::STARTUP;
        group_b_started_ = false;
        fully_stopped_ = false;
        
        for (int i = 0; i < 6; i++) {
            prev_positions_[i] = 0.0f;
            initial_positions_[i] = 0.0f;
            home_offsets_[i] = 0.0f;
        }

        cmd_pub_ = this->create_publisher<rinbo_msgs::msg::MotorCmdStamped>(
            "/motor/command", 10);
        
        pid_data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/pid/data", 10);
        
        state_sub_ = this->create_subscription<rinbo_msgs::msg::MotorStateStamped>(
            "/motor/state", 10,
            std::bind(&PIDController::state_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "=== RSLIP Tripod Controller (6 legs) ===");
        RCLCPP_INFO(this->get_logger(), "Startup: Move to %.1f deg in %.1f s", 
                    theta_home_ * 180.0 / M_PI, startup_duration_);
        RCLCPP_INFO(this->get_logger(), "t_center (Standing): %.6f s", t_center_);
        RCLCPP_INFO(this->get_logger(), "theta_dot at Standing: %.2f rad/s", theta_dot_center_);
        RCLCPP_INFO(this->get_logger(), "Group A (phase 0): R1, L2, R3");
        RCLCPP_INFO(this->get_logger(), "Group B (phase %.4f): L1, R2, L3", phase_offset_B_);
        RCLCPP_INFO(this->get_logger(), "t_stance: %.4f s, t_flight: %.4f s, period: %.4f s", 
                    t_stance_, t_flight_, period_);
        RCLCPP_INFO(this->get_logger(), "w_top: %.2f rad/s", w_top_);
    }

private:
    enum class State {
        STARTUP,
        RUNNING,
        STOPPING
    };
    
    double eval_poly_deg(double t) {
        double result = 0.0;
        double t_pow = 1.0;
        for (size_t i = 0; i < poly_.size(); i++) {
            result += poly_[i] * t_pow;
            t_pow *= t;
        }
        return result;
    }
    
    double eval_poly_rad(double t) {
        return eval_poly_deg(t) * M_PI / 180.0;
    }
    
    double eval_poly_derivative_deg(double t) {
        double result = 0.0;
        double t_pow = 1.0;
        for (size_t i = 1; i < poly_.size(); i++) {
            result += i * poly_[i] * t_pow;
            t_pow *= t;
        }
        return result;
    }
    
    double eval_poly_derivative_rad(double t) {
        return eval_poly_derivative_deg(t) * M_PI / 180.0;
    }
    
    // 用牛頓法找 poly(t) = 0 的點
    double find_poly_zero() {
        double t = t_stance_ / 2.0;  // 初始猜測：Stance 中間
        
        for (int i = 0; i < 20; i++) {
            double f = eval_poly_deg(t);
            double df = eval_poly_derivative_deg(t);
            
            if (std::fabs(df) < 1e-10) break;
            
            double t_new = t - f / df;
            
            // 確保在 [0, t_stance] 範圍內
            t_new = std::clamp(t_new, 0.0, t_stance_);
            
            if (std::fabs(t_new - t) < 1e-10) break;
            t = t_new;
        }
        
        return t;
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
    
    // Cubic polynomial: 從 (p0, v0) 到 (p1, v1)，時間 T
    void eval_cubic(double t, double T, double p0, double v0, double p1, double v1,
                    double& pos, double& vel) {
        if (t <= 0) {
            pos = p0;
            vel = v0;
            return;
        }
        if (t >= T) {
            pos = p1;
            vel = v1;
            return;
        }
        
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
        
        // 檢查 shutdown 信號
        if (g_shutdown_requested && state_ != State::STOPPING) {
            state_ = State::STOPPING;
            RCLCPP_INFO(this->get_logger(), "=== Shutdown requested, slowing down... ===");
        }
        
        if (fully_stopped_) {
            rclcpp::shutdown();
            return;
        }
        
        // 6 隻腳的 encoder
        std::array<float, 6> positions = {
            msg->l1.position,
            msg->l2.position,
            msg->l3.position,
            -msg->r1.position,
            -msg->r2.position,
            -msg->r3.position
        };
        
        std::array<bool, 6> is_group_b = {true, false, true, false, true, false};
        
        if (!initialized_) {
            start_time_ = now_time;
            prev_time_ = now_time;
            cycle_count_ = 0;
            tau_ = 0.0;
            startup_time_ = 0.0;
            
            for (int i = 0; i < 6; i++) {
                prev_positions_[i] = positions[i];
                initial_positions_[i] = positions[i];
            }
            
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Initialized!");
            RCLCPP_INFO(this->get_logger(), "  L1-L3: [%.0f, %.0f, %.0f]", 
                        positions[0], positions[1], positions[2]);
            RCLCPP_INFO(this->get_logger(), "  R1-R3: [%.0f, %.0f, %.0f]", 
                        positions[3], positions[4], positions[5]);
            return;
        }
        
        double dt = (now_time - prev_time_).seconds();
        if (dt <= 0) dt = 0.001;
        
        std::array<float, 6> pwms;
        bool group_b_active = true;
        
        switch (state_) {
            case State::STARTUP: {
                startup_time_ += dt;
                
                // RSLIP 起始速度 = Standing 時的速度（用 start_ratio_ 縮放）
                double rslip_start_vel = theta_dot_center_ / start_ratio_;
                
                for (int i = 0; i < 6; i++) {
                    double p0 = initial_positions_[i];
                    double v0 = 0.0;
                    double p1 = initial_positions_[i] + theta_home_ * rad_to_counts_;
                    double v1 = rslip_start_vel * rad_to_counts_;
                    
                    double target_pos, target_vel;
                    eval_cubic(startup_time_, startup_duration_, p0, v0, p1, v1, target_pos, target_vel);
                    
                    float actual_vel = (positions[i] - prev_positions_[i]) / dt;
                    float pos_error = target_pos - positions[i];
                    float vel_error = target_vel - actual_vel;
                    
                    float pwm = kp_ * pos_error + kd_ * vel_error + k_ff_ * target_vel;
                    pwms[i] = std::clamp(pwm, -max_pwm_, max_pwm_);
                }
                
                // 啟動完成，進入 RUNNING
                if (startup_time_ >= startup_duration_) {
                    state_ = State::RUNNING;
                    tau_ = t_center_;  // 從 Standing（Stance 中心）開始！
                    current_ratio_ = start_ratio_;
                    
                    for (int i = 0; i < 6; i++) {
                        // Standing 時 poly = 0，所以 home_offset 對應 theta = 0
                        home_offsets_[i] = positions[i] - theta_start_rslip_ * rad_to_counts_;
                        // theta_start_rslip_ = 0，所以 home_offsets_[i] = positions[i]
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "=== Startup done, entering RUNNING ===");
                    RCLCPP_INFO(this->get_logger(), "Starting from tau = %.6f (Standing)", tau_);
                }
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                    "[STARTUP] %.2f/%.2f s | PWM: [%.0f, %.0f, %.0f, %.0f, %.0f, %.0f]", 
                    startup_time_, startup_duration_,
                    pwms[0], pwms[1], pwms[2], pwms[3], pwms[4], pwms[5]);
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
                
                // Group B 啟動
                group_b_active = (tau_ >= t_center_ + phase_offset_B_);
                
                if (group_b_active && !group_b_started_) {
                    home_offsets_[0] = positions[0] - theta_start_rslip_ * rad_to_counts_;
                    home_offsets_[2] = positions[2] - theta_start_rslip_ * rad_to_counts_;
                    home_offsets_[4] = positions[4] - theta_start_rslip_ * rad_to_counts_;
                    group_b_started_ = true;
                    RCLCPP_INFO(this->get_logger(), "=== Group B started! ===");
                }
                
                if (n > cycle_count_) {
                    cycle_count_ = n;
                    RCLCPP_INFO(this->get_logger(), "=== New cycle: %d (ratio: %.2f) ===", 
                                cycle_count_, current_ratio_);
                }
                std::array<float, 6> target_positions;
                std::array<float, 6> target_velocities;
                
                // 計算 PWM
                for (int i = 0; i < 6; i++) {
                    if (is_group_b[i] && !group_b_active) {
                        pwms[i] = 0.0f;
                        continue;
                    }
                    
                    double tau_for_calc = is_group_b[i] ? (tau_ - phase_offset_B_) : tau_;
                    
                    double target_rad, target_vel_rad;
                    bool stance_phase;
                    compute_trajectory(tau_for_calc, target_rad, target_vel_rad, stance_phase);
                    
                    float target = home_offsets_[i] + target_rad * rad_to_counts_;
                    float target_vel = target_vel_rad * rad_to_counts_;
                    float actual_vel = (positions[i] - prev_positions_[i]) / dt;
                    target_positions[i] = target;
                    target_velocities[i] = target_vel;
                    float pos_error = target - positions[i];
                    float vel_error = target_vel - actual_vel;
                    
                    float pwm = kp_ * pos_error + kd_ * vel_error + k_ff_ * target_vel;
                    pwms[i] = std::clamp(pwm, -max_pwm_, max_pwm_);
                }
                auto pid_msg = std_msgs::msg::Float32MultiArray();
                pid_msg.data.resize(20);
                
                pid_msg.data[0] = static_cast<float>(tau_);
                pid_msg.data[1] = static_cast<float>(current_ratio_);
                
                for (int i = 0; i < 6; i++) {
                    pid_msg.data[2 + i] = target_positions[i];
                    pid_msg.data[8 + i] = target_velocities[i];
                    pid_msg.data[14 + i] = pwms[i];
                }
                
                pid_data_pub_->publish(pid_msg);
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                    "tau: %.3f | ratio: %.2f | A: [%.0f, %.0f, %.0f] | B: [%.0f, %.0f, %.0f]%s", 
                    tau_, current_ratio_, 
                    pwms[3], pwms[1], pwms[5],
                    pwms[0], pwms[4], pwms[2],
                    group_b_active ? "" : " [B:wait]");
                break;
            }
            
            case State::STOPPING: {
                current_ratio_ += slowdown_step_;
                
                if (current_ratio_ >= 10.0) {
                    RCLCPP_INFO(this->get_logger(), "=== Fully stopped ===");
                    
                    auto cmd = rinbo_msgs::msg::MotorCmdStamped();
                    cmd.l1.enable = false;
                    cmd.l2.enable = false;
                    cmd.l3.enable = false;
                    cmd.r1.enable = false;
                    cmd.r2.enable = false;
                    cmd.r3.enable = false;
                    set_servos(cmd);
                    cmd.servo_control_mode = 2;
                    cmd_pub_->publish(cmd);
                    
                    fully_stopped_ = true;
                    return;
                }
                
                double d_tau = dt / current_ratio_;
                tau_ += d_tau;
                
                group_b_active = (tau_ >= t_center_ + phase_offset_B_);
                
                for (int i = 0; i < 6; i++) {
                    if (is_group_b[i] && !group_b_active) {
                        pwms[i] = 0.0f;
                        continue;
                    }
                    
                    double tau_for_calc = is_group_b[i] ? (tau_ - phase_offset_B_) : tau_;
                    
                    double target_rad, target_vel_rad;
                    bool stance_phase;
                    compute_trajectory(tau_for_calc, target_rad, target_vel_rad, stance_phase);
                    
                    float target = home_offsets_[i] + target_rad * rad_to_counts_;
                    float target_vel = target_vel_rad * rad_to_counts_;
                    float actual_vel = (positions[i] - prev_positions_[i]) / dt;
                    
                    float pos_error = target - positions[i];
                    float vel_error = target_vel - actual_vel;
                    
                    float pwm = kp_ * pos_error + kd_ * vel_error + k_ff_ * target_vel;
                    pwms[i] = std::clamp(pwm, -max_pwm_, max_pwm_);
                }
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                    "[STOPPING] ratio: %.2f | A: [%.0f, %.0f, %.0f] | B: [%.0f, %.0f, %.0f]", 
                    current_ratio_, 
                    pwms[3], pwms[1], pwms[5],
                    pwms[0], pwms[4], pwms[2]);
                break;
            }
        }
        
        // 發送命令
        auto cmd = rinbo_msgs::msg::MotorCmdStamped();
        
        bool all_enabled = (state_ == State::STARTUP);
        
        cmd.l1.enable = all_enabled || group_b_active;
        cmd.l1.direction = (pwms[0] >= 0);
        cmd.l1.voltage = std::fabs(pwms[0]);
        cmd.l1.state = 1;
        cmd.l1.reset_position = false;
        
        cmd.l2.enable = true;
        cmd.l2.direction = (pwms[1] >= 0);
        cmd.l2.voltage = std::fabs(pwms[1]);
        cmd.l2.state = 1;
        cmd.l2.reset_position = false;
        
        cmd.l3.enable = all_enabled || group_b_active;
        cmd.l3.direction = (pwms[2] >= 0);
        cmd.l3.voltage = std::fabs(pwms[2]);
        cmd.l3.state = 1;
        cmd.l3.reset_position = false;
        
        cmd.r1.enable = true;
        cmd.r1.direction = (pwms[3] < 0);
        cmd.r1.voltage = std::fabs(pwms[3]);
        cmd.r1.state = 1;
        cmd.r1.reset_position = false;
        
        cmd.r2.enable = all_enabled || group_b_active;
        cmd.r2.direction = (pwms[4] < 0);
        cmd.r2.voltage = std::fabs(pwms[4]);
        cmd.r2.state = 1;
        cmd.r2.reset_position = false;
        
        cmd.r3.enable = true;
        cmd.r3.direction = (pwms[5] < 0);
        cmd.r3.voltage = std::fabs(pwms[5]);
        cmd.r3.state = 1;
        cmd.r3.reset_position = false;
        
        set_servos(cmd);
        cmd.servo_control_mode = 2;
        
        cmd_pub_->publish(cmd);
        
        for (int i = 0; i < 6; i++) {
            prev_positions_[i] = positions[i];
        }
        prev_time_ = now_time;
    }

    float kp_, kd_, k_ff_;
    float max_pwm_;
    
    std::vector<double> poly_matlab_;  
    std::vector<double> poly_; 
    
    double t_stance_;
    double t_flight_;
    double period_;
    double phase_offset_B_;
    double br_;
    double rad_to_counts_;
    
    double t_center_;  // poly(t) = 0 的時間點
    double theta_dot_center_;  // Standing 時的速度
    
    double theta_LO_, theta_TD_;
    double theta_dot_LO_, theta_dot_TD_;
    double w_top_, a1_, a2_;
    
    double theta_start_rslip_;
    double theta_home_;
    double startup_duration_;
    double startup_time_;
    
    std::array<uint32_t, 6> servo_targets_;
    
    double tau_;
    double current_ratio_;
    double target_ratio_;
    double ratio_step_;
    double slowdown_step_;
    double start_ratio_;
    
    State state_;
    bool group_b_started_;
    bool fully_stopped_;
    
    std::array<float, 6> initial_positions_;
    std::array<float, 6> home_offsets_;
    std::array<float, 6> prev_positions_;
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
