#include "rclcpp/rclcpp.hpp"
#include "disabled_legs.hpp"
#include "robot_config.hpp"
#include <iostream>
#include <cstdio>
#include "latest_state_qos.hpp"
#include "motor_arbiter_handshake.hpp"
#include "ros_input_guard.hpp"
#include "bridge_input_discovery.hpp"
#include "safety_invariants.hpp"
#include "tripod_power_policy.hpp"
#include "rinbo_msgs/msg/controller_debug_stamped.hpp"
#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include "rinbo_msgs/msg/motor_state_stamped.hpp"
#include "rinbo_msgs/msg/power_state_stamped.hpp"
#include "rinbo_msgs/msg/safety_event_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <csignal>
#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

std::atomic<bool> g_shutdown_requested{false};

void signal_handler(int signum) {
    (void)signum;
    g_shutdown_requested = true;
}

class PIDController : public rclcpp::Node {
    friend struct TripodOfflineTestAccess;
public:
    explicit PIDController(rinbo_config::MotionSession& session)
        : Node("rinbo_tripod_rslip", session.config().node_options("rinbo_tripod_rslip")),
          motion_session_(session) {
#ifndef RINBO_FSM_OFFLINE_TEST
        rinbo_fsm::wait_for_bridge_inputs(*this, [] { return g_shutdown_requested != 0; });
#endif
        disabled_legs_ = rinbo_fsm::DisabledLegs::load(*this);
        if (disabled_legs_.healthy_count() == 0)
            throw std::runtime_error("没有可測試腿: all six main drives are disabled");
        kp_ = static_cast<float>(this->declare_parameter<double>("kp", 0.38));
        kd_ = static_cast<float>(this->declare_parameter<double>("kd", 0.003));
        k_ff_ = static_cast<float>(this->declare_parameter<double>("k_ff", 0.005));
        
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
        startup_duration_ = this->declare_parameter<double>("startup_duration", 8.0);
        
        servo_targets_[0] = 740; 
        servo_targets_[1] = 2565; 
        servo_targets_[2] = 3283;
        servo_targets_[3] = 1944; 
        servo_targets_[4] = 2071; 
        servo_targets_[5] = 989;
        
        max_pwm_ = static_cast<float>(this->declare_parameter<double>("max_pwm", 80.0));
        
        initialized_ = false;
        cycle_count_ = 0;
        
        tau_ = 0.0;
        startup_time_ = 0.0;
        
        start_ratio_ = this->declare_parameter<double>("start_ratio", 8.0);
        current_ratio_ = start_ratio_;
        target_ratio_ = this->declare_parameter<double>("target_ratio", 4.0);
        ratio_step_ = this->declare_parameter<double>("ratio_step", -0.00005);
        slowdown_step_ = this->declare_parameter<double>("slowdown_step", 0.002);
        shutdown_slowdown_duration_s_ =
            this->declare_parameter<double>("shutdown.slowdown_duration_s", 2.0);
        shutdown_timeout_s_ =
            this->declare_parameter<double>("shutdown.timeout_s", 5.0);
        stop_servo_control_mode_ = this->declare_parameter<int>("stop_servo_control_mode", 0);
        if (stop_servo_control_mode_ != 0 && stop_servo_control_mode_ != 2) {
            throw std::invalid_argument("stop_servo_control_mode must be 0 or 2");
        }
        rinbo_fsm::validate_pwm(max_pwm_, "Tripod");
        if (!std::isfinite(shutdown_slowdown_duration_s_) ||
            !std::isfinite(shutdown_timeout_s_) ||
            shutdown_slowdown_duration_s_ <= 0.0 ||
            shutdown_timeout_s_ < shutdown_slowdown_duration_s_ ||
            shutdown_timeout_s_ > 5.0) {
            throw std::invalid_argument(
                "Tripod shutdown must satisfy 0 < slowdown_duration <= timeout <= 5s");
        }
        load_safety_parameters();
        power_policy_ = std::make_unique<rinbo_fsm::TripodPowerPolicy>(
            rinbo_fsm::TripodPowerPolicyConfig {
                safety_.enabled,
                safety_.stop_on_voltage_sag,
                safety_.stop_on_over_voltage,
                safety_.stop_on_current_limit,
                safety_.stop_on_bus_current_limit,
                safety_.require_power_relay,
                safety_.stop_on_power_stale,
                safety_.min_bus_voltage,
                safety_.max_bus_voltage,
                safety_.max_current,
                safety_.max_bus_current,
                safety_.power_stale_seconds,
                safety_.power_required_after_seconds,
                safety_.power_bus_voltage_channel,
                safety_.voltage_trip_samples,
                safety_.current_trip_samples});
        motor_state_guard_ = std::make_unique<rinbo_fsm::RosInputGuard>(
            *this, "/motor/state");
        power_state_guard_ = std::make_unique<rinbo_fsm::RosInputGuard>(
            *this, "/power/state");
        
        state_ = State::STARTUP;
        group_b_started_ = false;
        fully_stopped_ = false;
        
        for (int i = 0; i < 6; i++) {
            prev_positions_[i] = 0.0f;
            initial_positions_[i] = 0.0f;
            home_offsets_[i] = 0.0f;
            prev_command_pwms_[i] = 0.0f;
        }

        cmd_pub_ = this->create_publisher<rinbo_msgs::msg::MotorCmdStamped>(
            "/motor/command", 10);
        motor_handshake_ =
            std::make_unique<rinbo_fsm::MotorArbiterHandshake>(*this);
        
        pid_data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/pid/data", 10);

        controller_debug_pub_ = this->create_publisher<rinbo_msgs::msg::ControllerDebugStamped>(
            "/rinbo/controller_debug", 10);

        safety_event_pub_ = this->create_publisher<rinbo_msgs::msg::SafetyEventStamped>(
            "/rinbo/safety_event", 10);
        
        state_sub_ = this->create_subscription<rinbo_msgs::msg::MotorStateStamped>(
            "/motor/state", rinbo_fsm::latest_state_qos(),
            std::bind(
                &PIDController::state_callback, this,
                std::placeholders::_1, std::placeholders::_2));

        power_state_sub_ = this->create_subscription<rinbo_msgs::msg::PowerStateStamped>(
            "/power/state", rinbo_fsm::latest_state_qos(),
            std::bind(
                &PIDController::power_state_callback, this,
                std::placeholders::_1, std::placeholders::_2));
        estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/estop", 10,
            std::bind(&PIDController::estop_callback, this, std::placeholders::_1));

        node_start_time_ = this->now();
        last_motor_state_time_ = node_start_time_;
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&PIDController::watchdog_callback, this));
        handshake_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&PIDController::handshake_timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "=== RSLIP Tripod Controller (%d healthy legs) ===",
                    disabled_legs_.healthy_count());
        for (int i = 0; i < 6; ++i)
            if (disabled_legs_.contains(i))
                RCLCPP_INFO(this->get_logger(), "%s: SKIPPED (main drive disabled)", leg_name(i));
        RCLCPP_INFO(this->get_logger(), "Startup: Move to %.1f deg in %.1f s", 
                    theta_home_ * 180.0 / M_PI, startup_duration_);
        RCLCPP_INFO(this->get_logger(), "t_center (Standing): %.6f s", t_center_);
        RCLCPP_INFO(this->get_logger(), "theta_dot at Standing: %.2f rad/s", theta_dot_center_);
        RCLCPP_INFO(this->get_logger(), "Group A (phase 0): R1, L2, R3");
        RCLCPP_INFO(this->get_logger(), "Group B (phase %.4f): L1, R2, L3", phase_offset_B_);
        RCLCPP_INFO(this->get_logger(), "t_stance: %.4f s, t_flight: %.4f s, period: %.4f s", 
                    t_stance_, t_flight_, period_);
        RCLCPP_INFO(this->get_logger(), "w_top: %.2f rad/s", w_top_);
        RCLCPP_INFO(this->get_logger(),
                    "Safety limits | max_pwm=%.0f start_ratio=%.2f target_ratio=%.2f | bus=ch%d range=[%.1f,%.1f]V samples=%d | leg_current=%s %.1fA samples=%d | position=%s %.0f counts samples=%d | pwm_slew=%s %.0f/s",
                    max_pwm_, start_ratio_, target_ratio_,
                    safety_.power_bus_voltage_channel, safety_.min_bus_voltage, safety_.max_bus_voltage,
                    safety_.voltage_trip_samples,
                    safety_.stop_on_current_limit ? "ON" : "OFF", safety_.max_current,
                    safety_.current_trip_samples,
                    safety_.stop_on_position_error ? "ON" : "OFF", safety_.max_position_error_counts,
                    safety_.position_error_trip_samples,
                    safety_.enable_pwm_slew_limit ? "ON" : "OFF", safety_.pwm_slew_rate_per_sec);
        if (disabled_legs_.enabled()) {
            RCLCPP_WARN(this->get_logger(),
                        "supported_leg_test: hardware.disabled_legs=[%s]. Remaining legs follow their existing independent phase references; robot support/suspension is required, with no ground stability guarantee. Mask disables main drives only; global servo control still commands every connected servo, requiring physical isolation where needed.",
                        disabled_legs_.summary().c_str());
        }
    }

    void fail_closed(const std::string& reason) { trigger_safety_stop(reason); }

private:
    enum class State {
        STARTUP,
        RUNNING,
        STOPPING,
        SAFETY_STOP
    };

    struct SafetyConfig {
        bool enabled = true;
        bool stop_on_voltage_sag = true;
        bool stop_on_over_voltage = true;
        bool stop_on_current_limit = true;
        bool stop_on_bus_current_limit = false;
        bool require_power_relay = true;
        bool stop_on_power_stale = true;
        bool stop_on_position_error = true;
        bool enable_pwm_slew_limit = true;
        float min_bus_voltage = 18.0f;
        float max_bus_voltage = 42.0f;
        float max_current = 5.0f;
        float max_bus_current = 30.0f;
        float max_position_error_counts = 9000.0f;
        double position_error_trip_seconds = 0.5;
        float pwm_slew_rate_per_sec = 250.0f;
        double power_stale_seconds = 0.5;
        double power_required_after_seconds = 2.0;
        double motor_state_stale_seconds = 0.25;
        double motor_state_required_after_seconds = 2.0;
        int power_bus_voltage_channel = 7;
        std::array<int, 6> leg_current_channels {1, 2, 3, 4, 5, 6};
        int voltage_trip_samples = 5;
        int current_trip_samples = 25;
        int position_error_trip_samples = 10;
    };

    struct PowerSnapshot {
        bool received = false;
        bool relay_power_on = false;
        rclcpp::Time stamp;
        std::array<float, 8> voltages {};
        std::array<float, 8> currents {};
        bool valid = false;
        float bus_voltage = 0.0f;
        float max_current = 0.0f;
        float bus_current = 0.0f;
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
        // With a high terminal speed, a cubic can initially run backwards
        // even though p1 > p0. Use a monotone power curve in that case while
        // preserving the requested endpoints, duration and terminal speed.
        if (v0 == 0.0 && p1 > p0 && v1 * T > 3.0 * (p1 - p0)) {
            const double exponent = v1 * T / (p1 - p0);
            const double phase = t / T;
            pos = p0 + (p1 - p0) * std::pow(phase, exponent);
            vel = v1 * std::pow(phase, exponent - 1.0);
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
            // floor() already accounts for the previous cycle.
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
    
    void load_safety_parameters() {
        safety_.enabled = this->declare_parameter<bool>("safety.enabled", true);
        safety_.stop_on_voltage_sag = this->declare_parameter<bool>("safety.stop_on_voltage_sag", true);
        safety_.stop_on_over_voltage = this->declare_parameter<bool>("safety.stop_on_over_voltage", true);
        safety_.stop_on_current_limit = this->declare_parameter<bool>("safety.stop_on_current_limit", true);
        safety_.stop_on_bus_current_limit = this->declare_parameter<bool>("safety.stop_on_bus_current_limit", false);
        safety_.require_power_relay = this->declare_parameter<bool>("safety.require_power_relay", true);
        safety_.stop_on_power_stale = this->declare_parameter<bool>("safety.stop_on_power_stale", true);
        safety_.stop_on_position_error = this->declare_parameter<bool>("safety.stop_on_position_error", true);
        safety_.enable_pwm_slew_limit = this->declare_parameter<bool>("safety.enable_pwm_slew_limit", true);
        safety_.min_bus_voltage = static_cast<float>(this->declare_parameter<double>("safety.min_bus_voltage", 18.0));
        safety_.max_bus_voltage = static_cast<float>(this->declare_parameter<double>("safety.max_bus_voltage", 42.0));
        safety_.max_current = static_cast<float>(this->declare_parameter<double>("safety.max_current", 5.0));
        safety_.max_bus_current = static_cast<float>(this->declare_parameter<double>("safety.max_bus_current", 30.0));
        safety_.max_position_error_counts = static_cast<float>(this->declare_parameter<double>("safety.max_position_error_counts", 9000.0));
        safety_.pwm_slew_rate_per_sec = static_cast<float>(this->declare_parameter<double>("safety.pwm_slew_rate_per_sec", 250.0));
        safety_.power_stale_seconds = this->declare_parameter<double>("safety.power_stale_seconds", 0.5);
        safety_.power_required_after_seconds = this->declare_parameter<double>("safety.power_required_after_seconds", 2.0);
        safety_.motor_state_stale_seconds = this->declare_parameter<double>("safety.motor_state_stale_seconds", 0.25);
        safety_.motor_state_required_after_seconds = this->declare_parameter<double>("safety.motor_state_required_after_seconds", 2.0);
        safety_.power_bus_voltage_channel = this->declare_parameter<int>("safety.power_bus_voltage_channel", 7);
        const auto current_channels = this->declare_parameter<std::vector<int64_t>>(
            "safety.leg_current_channels", std::vector<int64_t>{1, 2, 3, 4, 5, 6});
        if (current_channels.size() != 6) {
            throw std::invalid_argument("safety.leg_current_channels must contain six channel indices");
        }
        for (std::size_t i = 0; i < current_channels.size(); ++i) {
            if (current_channels[i] < 0 || current_channels[i] > 7) {
                throw std::invalid_argument("safety.leg_current_channels entries must be in [0, 7]");
            }
            if (current_channels[i] == safety_.power_bus_voltage_channel) {
                throw std::invalid_argument("safety.leg_current_channels must not include the bus channel");
            }
            for (std::size_t j = 0; j < i; ++j) {
                if (current_channels[i] == current_channels[j]) {
                    throw std::invalid_argument("safety.leg_current_channels entries must be unique");
                }
            }
            safety_.leg_current_channels[i] = static_cast<int>(current_channels[i]);
        }
        safety_.voltage_trip_samples = this->declare_parameter<int>("safety.voltage_trip_samples", 5);
        safety_.current_trip_samples = this->declare_parameter<int>("safety.current_trip_samples", 25);
        safety_.position_error_trip_samples = this->declare_parameter<int>("safety.position_error_trip_samples", 10);

        safety_.position_error_trip_seconds = this->declare_parameter<double>(
            "safety.position_error_trip_seconds", 0.5);

        const rinbo_fsm::PowerSafetyContract power_contract {
            safety_.enabled,
            safety_.stop_on_power_stale,
            safety_.stop_on_voltage_sag,
            safety_.stop_on_over_voltage,
            safety_.stop_on_current_limit,
            safety_.require_power_relay,
            safety_.power_bus_voltage_channel,
            safety_.leg_current_channels,
            safety_.min_bus_voltage,
            safety_.max_bus_voltage,
            safety_.max_current,
            safety_.max_bus_current,
            safety_.power_stale_seconds,
            safety_.power_required_after_seconds,
            safety_.voltage_trip_samples,
            safety_.current_trip_samples};
        rinbo_fsm::validate_power_safety_contract(power_contract, "Tripod");

        const rinbo_fsm::TripodMotionSafetyContract motion_contract {
            safety_.stop_on_position_error,
            safety_.enable_pwm_slew_limit,
            safety_.max_position_error_counts,
            safety_.pwm_slew_rate_per_sec,
            safety_.motor_state_stale_seconds,
            safety_.motor_state_required_after_seconds,
            safety_.position_error_trip_samples, safety_.position_error_trip_seconds};
        rinbo_fsm::validate_tripod_motion_safety_contract(motion_contract);
    }

    void power_state_callback(
        const rinbo_msgs::msg::PowerStateStamped::SharedPtr msg,
        const rclcpp::MessageInfo& message_info) {
        const auto input = power_state_guard_->accept(msg->header, message_info);
        if (input.pending_startup) return;
        if (input.violation) {
            trigger_safety_stop(*input.violation);
            return;
        }
        const auto clock_type = this->get_clock()->get_clock_type();
        const rclcpp::Time arrival_time(input.observed_ns, clock_type);
        const rclcpp::Time validated_time(input.validated_ns, clock_type);
        power_snapshot_.received = true;
        power_snapshot_.relay_power_on = msg->power;
        power_snapshot_.stamp = arrival_time;
        power_snapshot_.voltages = {
            static_cast<float>(msg->v_0), static_cast<float>(msg->v_1),
            static_cast<float>(msg->v_2), static_cast<float>(msg->v_3),
            static_cast<float>(msg->v_4), static_cast<float>(msg->v_5),
            static_cast<float>(msg->v_6), static_cast<float>(msg->v_7)
        };
        power_snapshot_.currents = {
            static_cast<float>(msg->i_0), static_cast<float>(msg->i_1),
            static_cast<float>(msg->i_2), static_cast<float>(msg->i_3),
            static_cast<float>(msg->i_4), static_cast<float>(msg->i_5),
            static_cast<float>(msg->i_6), static_cast<float>(msg->i_7)
        };

        power_snapshot_.valid = std::isfinite(
            power_snapshot_.voltages[safety_.power_bus_voltage_channel]);
        for (std::size_t leg = 0; leg < safety_.leg_current_channels.size(); ++leg) {
            if (disabled_legs_.contains(static_cast<int>(leg))) continue;
            const int channel = safety_.leg_current_channels[leg];
            power_snapshot_.valid = power_snapshot_.valid &&
                std::isfinite(power_snapshot_.currents[channel]);
        }
        if (safety_.stop_on_bus_current_limit) {
            power_snapshot_.valid = power_snapshot_.valid &&
                std::isfinite(power_snapshot_.currents[safety_.power_bus_voltage_channel]);
        }
        power_snapshot_.bus_voltage = power_snapshot_.voltages[safety_.power_bus_voltage_channel];
        power_snapshot_.bus_current = std::fabs(power_snapshot_.currents[safety_.power_bus_voltage_channel]);
        power_snapshot_.max_current = 0.0f;
        for (std::size_t leg = 0; leg < safety_.leg_current_channels.size(); ++leg) {
            if (disabled_legs_.contains(static_cast<int>(leg))) continue;
            const int channel = safety_.leg_current_channels[leg];
            power_snapshot_.max_current = std::max(
                power_snapshot_.max_current, std::fabs(power_snapshot_.currents[channel]));
        }

        power_policy_->observe(rinbo_fsm::TripodPowerObservation {
            input.observed_ns,
            power_snapshot_.relay_power_on,
            power_snapshot_.valid,
            power_snapshot_.bus_voltage,
            power_snapshot_.max_current,
            power_snapshot_.bus_current});

        if (state_ != State::SAFETY_STOP) {
            check_power_safety(validated_time);
        }
    }

    const char* state_name(State state) const {
        switch (state) {
            case State::STARTUP: return "STARTUP";
            case State::RUNNING: return "RUNNING";
            case State::STOPPING: return "STOPPING";
            case State::SAFETY_STOP: return "SAFETY_STOP";
        }
        return "UNKNOWN";
    }

    const char* leg_name(int index) const {
        static constexpr std::array<const char*, 6> names = {"L1", "L2", "L3", "R1", "R2", "R3"};
        return names[std::clamp(index, 0, 5)];
    }

    std::string format_float_array(const std::array<float, 6>& values, int precision = 0) const {
        std::ostringstream out;
        out.setf(std::ios::fixed);
        out.precision(precision);
        out << "[";
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i > 0) out << " ";
            out << values[i];
        }
        out << "]";
        return out.str();
    }

    uint32_t publish_motor_command(
        rinbo_msgs::msg::MotorCmdStamped& cmd,
        const std::string& frame_id = std::string()) {
        if (state_ == State::SAFETY_STOP || !motor_handshake_->ready_for_output(cmd_pub_->get_subscription_count())) {
            const std::array<rinbo_msgs::msg::LegCmd *, 6> legs = {
                &cmd.l1, &cmd.l2, &cmd.l3, &cmd.r1, &cmd.r2, &cmd.r3};
            for (auto *leg : legs) {
                leg->enable = false;
                leg->direction = false;
                leg->voltage = 0.0f;
                leg->state = 0;
                leg->reset_position = false;
            }
            cmd.servo_control_mode = 0;
        }
        rinbo_fsm::enforce_disabled_leg_commands(cmd, disabled_legs_);
        int64_t stamp_ns = this->now().nanoseconds();
        if (stamp_ns <= last_motor_command_stamp_ns_) {
            stamp_ns = last_motor_command_stamp_ns_ + 1;
        }
        last_motor_command_stamp_ns_ = stamp_ns;
        cmd.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
        cmd.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
        cmd.header.frame_id = frame_id.empty()
            ? motor_handshake_->command_frame_id() : frame_id;
        cmd.header.seq = ++motor_command_seq_;
        cmd_pub_->publish(cmd);
        return cmd.header.seq;
    }

    uint32_t publish_stop_command(const std::string& frame_id = std::string()) {
#ifdef RINBO_FSM_OFFLINE_TEST
        if (simulate_stop_publication_failure_)
            throw std::runtime_error("injected stop publication failure");
#endif
        auto cmd = rinbo_msgs::msg::MotorCmdStamped();
        // A safety/rearm stop must never interpret zero-initialized servo
        // targets as actionable position commands.
        cmd.servo_control_mode = 0;
        const uint32_t sequence = publish_motor_command(cmd, frame_id);
        prev_command_pwms_.fill(0.0f);
        return sequence;
    }

    void fill_header(rinbo_msgs::msg::Header& header, rclcpp::Time stamp, uint32_t seq) {
        const int64_t nanoseconds = stamp.nanoseconds();
        header.stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
        header.stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
        header.frame_id = "rinbo_tripod_rslip";
        header.seq = seq;
    }

    void publish_safety_event(rclcpp::Time now_time, const std::string& reason) {
#ifdef RINBO_FSM_OFFLINE_TEST
        if (simulate_event_publication_failure_)
            throw std::runtime_error("injected safety event publication failure");
#endif
        auto event = rinbo_msgs::msg::SafetyEventStamped();
        fill_header(event.header, now_time, safety_event_seq_++);
        event.source = "rinbo_tripod_rslip";
        event.severity = "ERROR";
        event.reason = reason;
        event.min_bus_voltage = power_snapshot_.received ? power_snapshot_.bus_voltage : 0.0f;
        event.max_current = power_snapshot_.received ? power_snapshot_.max_current : 0.0f;
        event.position_error = last_position_errors_;
        event.tau = tau_;
        event.ratio = current_ratio_;
        event.cycle_count = static_cast<uint32_t>(std::max(cycle_count_, 0));
        safety_event_pub_->publish(event);
    }

    void publish_controller_debug(
        rclcpp::Time now_time,
        const std::array<float, 6>& positions,
        const std::array<float, 6>& target_positions,
        const std::array<float, 6>& target_velocities,
        const std::array<float, 6>& actual_velocities,
        const std::array<float, 6>& pos_errors,
        const std::array<float, 6>& raw_pwms,
        const std::array<float, 6>& limited_pwms,
        bool group_b_active,
        const std::array<bool, 6>& group_b_leg) {
        auto debug = rinbo_msgs::msg::ControllerDebugStamped();
        fill_header(debug.header, now_time, controller_debug_seq_++);
        debug.controller_state = state_name(state_);
        debug.tau = tau_;
        debug.ratio = current_ratio_;
        debug.cycle_count = static_cast<uint32_t>(std::max(cycle_count_, 0));
        debug.group_b_active = group_b_active;
        debug.safety_stopped = (state_ == State::SAFETY_STOP);
        debug.group_b_leg = group_b_leg;
        debug.min_bus_voltage = power_snapshot_.received ? power_snapshot_.bus_voltage : 0.0f;
        debug.max_current = power_snapshot_.received ? power_snapshot_.max_current : 0.0f;
        debug.target_position = target_positions;
        debug.actual_position = positions;
        debug.target_velocity = target_velocities;
        debug.actual_velocity = actual_velocities;
        debug.position_error = pos_errors;
        debug.raw_pwm = raw_pwms;
        debug.limited_pwm = limited_pwms;
        debug.safety_reason = safety_stop_reason_;
        controller_debug_pub_->publish(debug);
    }

    bool trigger_safety_stop(const std::string& reason) {
        const bool first_failure = state_ != State::SAFETY_STOP;
        state_ = State::SAFETY_STOP;
        if (first_failure) safety_stop_reason_ = reason;
        // ROS may already be shutting down. Publication failure must not skip
        // invalidating a previously completed calibration/standing receipt.
        try {
            publish_stop_command();
        } catch (const std::exception& error) {
            std::fprintf(stderr, "[FATAL] stop command publication failed: %s\n", error.what());
        } catch (...) {
            std::fprintf(stderr, "[FATAL] stop command publication failed\n");
        }
        if (first_failure) {
            std::fprintf(stderr, "TRIPOD SAFETY STOP: %s\n", safety_stop_reason_.c_str());
            try {
                motion_session_.invalidate();
            } catch (const std::exception& error) {
                std::fprintf(stderr, "[FATAL] cannot invalidate stage receipts: %s\n", error.what());
            } catch (...) {
                std::fprintf(stderr, "[FATAL] cannot invalidate stage receipts\n");
            }
            try {
                publish_safety_event(this->now(), safety_stop_reason_);
            } catch (const std::exception& error) {
                std::fprintf(stderr, "[FATAL] safety event publication failed: %s\n", error.what());
            } catch (...) {
                std::fprintf(stderr, "[FATAL] safety event publication failed\n");
            }
        }
        return true;
    }

    bool check_power_safety(rclcpp::Time now_time) {
        if (const auto reason = power_policy_->violation(
                now_time.nanoseconds(), node_start_time_.nanoseconds())) {
            return trigger_safety_stop(*reason);
        }
        return false;
    }

    bool power_ready_for_output(rclcpp::Time now_time) {
        return power_policy_->ready_for_output(now_time.nanoseconds());
    }

    bool check_position_error_safety(const std::array<float, 6>& pos_errors, double now_s) {
        if (!safety_.enabled || !safety_.stop_on_position_error) {
            return false;
        }

        int worst_idx = -1;
        float worst_abs_error = 0.0f;
        for (int i = 0; i < 6; ++i) {
            if (disabled_legs_.contains(i)) {
                position_error_trip_counts_[i] = 0;
                continue;
            }
            const float error = std::fabs(pos_errors[i]);
            // Large divergence / invalid feedback bypass the transient allowance.
            if (!std::isfinite(error) || error > 18000.0f) {
                return trigger_safety_stop(std::string("hard position error: ") + leg_name(i) +
                    " counts=" + std::to_string(error) + " hard_limit=18000");
            }
            if (error > safety_.max_position_error_counts) {
                if (position_error_trip_counts_[i] == 0) position_error_since_s_[i] = now_s;
                position_error_trip_counts_[i] = std::min(
                    position_error_trip_counts_[i] + 1, safety_.position_error_trip_samples);
            } else {
                position_error_trip_counts_[i] = 0;
            }
            if (position_error_trip_counts_[i] >= safety_.position_error_trip_samples &&
                now_s - position_error_since_s_[i] >= safety_.position_error_trip_seconds &&
                error > worst_abs_error) {
                worst_abs_error = error;
                worst_idx = i;
            }
        }

        if (worst_idx >= 0) {
            std::ostringstream reason;
            reason << "position error " << leg_name(worst_idx) << "=" << worst_abs_error
                   << " counts threshold=" << safety_.max_position_error_counts
                   << " duration=" << now_s - position_error_since_s_[worst_idx]
                   << "s required=" << safety_.position_error_trip_seconds
                   << "s errors=" << format_float_array(pos_errors);
            return trigger_safety_stop(reason.str());
        }

        return false;
    }

    void apply_pwm_slew_limit(std::array<float, 6>& pwms, double dt) {
        for (int i = 0; i < 6; ++i) {
            if (disabled_legs_.contains(i)) {
                pwms[i] = 0.0f;
                prev_command_pwms_[i] = 0.0f;
            }
        }
        if (!safety_.enabled || !safety_.enable_pwm_slew_limit) {
            prev_command_pwms_ = pwms;
            return;
        }

        const float step = std::max(1.0f, safety_.pwm_slew_rate_per_sec * static_cast<float>(std::max(dt, 0.001)));
        for (int i = 0; i < 6; ++i) {
            if (disabled_legs_.contains(i)) {
                pwms[i] = 0.0f;
                prev_command_pwms_[i] = 0.0f;
                continue;
            }
            const float delta = pwms[i] - prev_command_pwms_[i];
            if (std::fabs(delta) > step) {
                pwms[i] = prev_command_pwms_[i] + std::copysign(step, delta);
            }
        }
        prev_command_pwms_ = pwms;
    }

    void update_previous_samples(const std::array<float, 6>& positions, rclcpp::Time now_time) {
        for (int i = 0; i < 6; i++) {
            prev_positions_[i] = positions[i];
        }
        prev_time_ = now_time;
    }

    void state_callback(
        const rinbo_msgs::msg::MotorStateStamped::SharedPtr msg,
        const rclcpp::MessageInfo& message_info) {
        const auto input = motor_state_guard_->accept(msg->header, message_info);
        if (input.pending_startup) return;
        if (input.violation) {
            trigger_safety_stop(*input.violation);
            return;
        }
        const auto clock_type = this->get_clock()->get_clock_type();
        const rclcpp::Time arrival_time(input.observed_ns, clock_type);
        const rclcpp::Time now_time(input.validated_ns, clock_type);
        if (!input.fresh_at_validation(safety_.motor_state_stale_seconds)) {
            trigger_safety_stop(
                "/motor/state became stale during publisher validation");
            return;
        }
        if (motor_state_received_ && arrival_time < last_motor_state_time_) {
            motor_state_clock_rollback_detected_ = true;
        }
        last_motor_state_time_ = arrival_time;
        motor_state_received_ = true;
        
        // 檢查 shutdown 信號
        if (g_shutdown_requested && state_ != State::STOPPING && state_ != State::SAFETY_STOP) {
            state_ = State::STOPPING;
            stopping_start_time_ = now_time;
            stopping_start_ratio_ = current_ratio_;
            stopping_started_ = true;
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

        if (state_ == State::SAFETY_STOP) {
            publish_stop_command();
            std::array<float, 6> zero_values {};
            publish_controller_debug(now_time, positions, positions, zero_values, zero_values,
                                     zero_values, zero_values, zero_values, false, is_group_b);
            update_previous_samples(positions, now_time);
            return;
        }

        for (int i = 0; i < 6; ++i) {
            if (!disabled_legs_.contains(i) && !std::isfinite(positions[i])) {
                trigger_safety_stop(std::string("invalid motor position: ") + leg_name(i));
                update_previous_samples(positions, now_time);
                return;
            }
        }

        const auto command_subscriber_count = cmd_pub_->get_subscription_count();
        if (const auto reason = motor_handshake_->violation(command_subscriber_count)) {
            trigger_safety_stop(*reason);
            update_previous_samples(positions, now_time);
            return;
        }
        if (!motor_handshake_->ready_for_output(command_subscriber_count)) {
            const auto reason = motor_handshake_->waiting_reason(command_subscriber_count);
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Motor output remains disabled: %s", reason.c_str());
            update_previous_samples(positions, now_time);
            return;
        }

        if (check_power_safety(now_time)) {
            update_previous_samples(positions, now_time);
            return;
        }

        if (!power_ready_for_output(now_time)) {
            publish_stop_command();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Waiting for valid in-range /power/state; output remains disabled");
            update_previous_samples(positions, now_time);
            return;
        }

        if (!motor_handshake_->active_output_confirmed()) {
            auto probe = rinbo_msgs::msg::MotorCmdStamped();
            set_servo_hold_targets(probe, *msg);
            probe.servo_control_mode = 2;
            const uint32_t probe_sequence = publish_motor_command(
                probe, motor_handshake_->active_probe_frame_id());
            if (!motor_handshake_->mark_active_command_published(
                    command_subscriber_count, probe_sequence)) {
                trigger_safety_stop("motor arbiter refused the first active hold command");
                update_previous_samples(positions, now_time);
                return;
            }
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Motion remains paused: waiting for correlated /rinbo/motor_active_ack");
            update_previous_samples(positions, now_time);
            return;
        }
        
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
        
        handle_motion_sample(msg, positions, now_time);
    }

    // Called only after production input and arbiter guards pass. Tests invoke
    // this body directly with samples in an isolated localhost ROS domain.
    void handle_motion_sample(const rinbo_msgs::msg::MotorStateStamped::SharedPtr& msg,
                              const std::array<float, 6>& positions,
                              rclcpp::Time now_time) {
        const std::array<bool, 6> is_group_b = {true, false, true, false, true, false};
        double dt = (now_time - prev_time_).seconds();
        if (dt <= 0) dt = 0.001;

        std::array<float, 6> actual_velocities {};
        for (int i = 0; i < 6; i++) {
            actual_velocities[i] = (positions[i] - prev_positions_[i]) / dt;
            if (disabled_legs_.contains(i)) actual_velocities[i] = 0.0f;
        }
        
        std::array<float, 6> pwms {};
        std::array<float, 6> target_positions = positions;
        std::array<float, 6> target_velocities {};
        std::array<float, 6> pos_errors {};
        bool group_b_active = true;
        bool startup_completed = false;
        
        switch (state_) {
            case State::STARTUP: {
                startup_time_ += dt;
                
                // RSLIP 起始速度 = Standing 時的速度（用 start_ratio_ 縮放）
                double rslip_start_vel = theta_dot_center_ / start_ratio_;
                
                for (int i = 0; i < 6; i++) {
                    if (disabled_legs_.contains(i)) {
                        target_positions[i] = positions[i];
                        target_velocities[i] = 0.0f;
                        pos_errors[i] = 0.0f;
                        pwms[i] = 0.0f;
                        continue;
                    }
                    double p0 = initial_positions_[i];
                    double v0 = 0.0;
                    double p1 = initial_positions_[i] + theta_home_ * rad_to_counts_;
                    double v1 = rslip_start_vel * rad_to_counts_;
                    
                    double target_pos, target_vel;
                    eval_cubic(startup_time_, startup_duration_, p0, v0, p1, v1, target_pos, target_vel);
                    
                    float pos_error = target_pos - positions[i];
                    float vel_error = target_vel - actual_velocities[i];
                    target_positions[i] = static_cast<float>(target_pos);
                    target_velocities[i] = static_cast<float>(target_vel);
                    pos_errors[i] = pos_error;
                    
                    float pwm = kp_ * pos_error + kd_ * vel_error + k_ff_ * target_vel;
                    pwms[i] = std::clamp(pwm, -max_pwm_, max_pwm_);
                }
                
                // 啟動完成，進入 RUNNING
                bool startup_aligned = true;
                for (int i = 0; i < 6; ++i) {
                    if (disabled_legs_.contains(i)) continue;
                    startup_aligned = startup_aligned && std::isfinite(pos_errors[i]) &&
                        std::fabs(pos_errors[i]) <= safety_.max_position_error_counts;
                }
                // The timed guard allows a brief endpoint tracking delay.
                if (startup_time_ >= startup_duration_ && startup_aligned) {
                    state_ = State::RUNNING;
                    startup_completed = true;
                    tau_ = t_center_;  // 從 Standing（Stance 中心）開始！
                    current_ratio_ = start_ratio_;
                    
                    for (int i = 0; i < 6; i++) {
                        if (disabled_legs_.contains(i)) continue;
                        // Standing 時 poly = 0，所以 home_offset 對應 theta = 0
                        home_offsets_[i] = positions[i] - theta_start_rslip_ * rad_to_counts_;
                        // theta_start_rslip_ = 0，所以 home_offsets_[i] = positions[i]
                    }
                    
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
                    for (int i : {0, 2, 4})
                        if (!disabled_legs_.contains(i))
                            home_offsets_[i] = positions[i] - theta_start_rslip_ * rad_to_counts_;
                    group_b_started_ = true;
                    RCLCPP_INFO(this->get_logger(), "=== Group B started! ===");
                }
                
                if (n > cycle_count_) {
                    cycle_count_ = n;
                    RCLCPP_INFO(this->get_logger(), "=== New cycle: %d (ratio: %.2f) ===", 
                                cycle_count_, current_ratio_);
                }
                
                // 計算 PWM
                for (int i = 0; i < 6; i++) {
                    if (disabled_legs_.contains(i)) {
                        pwms[i] = 0.0f;
                        target_positions[i] = positions[i];
                        target_velocities[i] = 0.0f;
                        pos_errors[i] = 0.0f;
                        continue;
                    }
                    if (is_group_b[i] && !group_b_active) {
                        pwms[i] = 0.0f;
                        target_positions[i] = positions[i];
                        target_velocities[i] = 0.0f;
                        pos_errors[i] = 0.0f;
                        continue;
                    }
                    
                    double tau_for_calc = is_group_b[i] ? (tau_ - phase_offset_B_) : tau_;
                    
                    double target_rad, target_vel_rad;
                    bool stance_phase;
                    compute_trajectory(tau_for_calc, target_rad, target_vel_rad, stance_phase);
                    
                    float target = home_offsets_[i] + target_rad * rad_to_counts_;
                    float target_vel = target_vel_rad * rad_to_counts_;
                    target_positions[i] = target;
                    target_velocities[i] = target_vel;
                    float pos_error = target - positions[i];
                    float vel_error = target_vel - actual_velocities[i];
                    pos_errors[i] = pos_error;
                    
                    float pwm = kp_ * pos_error + kd_ * vel_error + k_ff_ * target_vel;
                    pwms[i] = std::clamp(pwm, -max_pwm_, max_pwm_);
                }
                auto pid_msg = std_msgs::msg::Float32MultiArray();
                pid_msg.data.resize(20);
                
                pid_msg.data[0] = static_cast<float>(tau_);
                pid_msg.data[1] = static_cast<float>(current_ratio_);
                
                for (int i = 0; i < 6; i++) {
                    if (disabled_legs_.contains(i)) {
                        pwms[i] = 0.0f;
                        target_positions[i] = positions[i];
                        target_velocities[i] = 0.0f;
                        pos_errors[i] = 0.0f;
                        continue;
                    }
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
                if (!stopping_started_) {
                    stopping_start_time_ = now_time;
                    stopping_start_ratio_ = current_ratio_;
                    stopping_started_ = true;
                }
                const double stop_elapsed =
                    (now_time - stopping_start_time_).seconds();
                if (stop_elapsed > shutdown_timeout_s_) {
                    trigger_safety_stop("controlled shutdown timeout");
                    update_previous_samples(positions, now_time);
                    return;
                }

                if (stop_elapsed >= shutdown_slowdown_duration_s_) {
                    
                    auto cmd = rinbo_msgs::msg::MotorCmdStamped();
                    cmd.l1.enable = false;
                    cmd.l2.enable = false;
                    cmd.l3.enable = false;
                    cmd.r1.enable = false;
                    cmd.r2.enable = false;
                    cmd.r3.enable = false;
                    cmd.servo_control_mode =
                        static_cast<uint32_t>(stop_servo_control_mode_);
                    if (cmd.servo_control_mode == 2U) {
                        set_servo_hold_targets(cmd, *msg);
                    }
                    publish_motor_command(cmd);
                    prev_command_pwms_.fill(0.0f);
                    
                    fully_stopped_ = true;
                    RCLCPP_INFO(this->get_logger(), "=== Fully stopped ===");
                    rclcpp::shutdown();
                    return;
                }

                const double stop_progress = std::clamp(
                    stop_elapsed / shutdown_slowdown_duration_s_, 0.0, 1.0);
                current_ratio_ = stopping_start_ratio_ +
                    (10.0 - stopping_start_ratio_) * stop_progress;
                
                double d_tau = dt / current_ratio_;
                tau_ += d_tau;
                
                group_b_active = (tau_ >= t_center_ + phase_offset_B_);
                
                for (int i = 0; i < 6; i++) {
                    if (disabled_legs_.contains(i)) {
                        pwms[i] = 0.0f;
                        target_positions[i] = positions[i];
                        target_velocities[i] = 0.0f;
                        pos_errors[i] = 0.0f;
                        continue;
                    }
                    if (is_group_b[i] && !group_b_active) {
                        pwms[i] = 0.0f;
                        target_positions[i] = positions[i];
                        target_velocities[i] = 0.0f;
                        pos_errors[i] = 0.0f;
                        continue;
                    }
                    
                    double tau_for_calc = is_group_b[i] ? (tau_ - phase_offset_B_) : tau_;
                    
                    double target_rad, target_vel_rad;
                    bool stance_phase;
                    compute_trajectory(tau_for_calc, target_rad, target_vel_rad, stance_phase);
                    
                    float target = home_offsets_[i] + target_rad * rad_to_counts_;
                    float target_vel = target_vel_rad * rad_to_counts_;
                    
                    float pos_error = target - positions[i];
                    float vel_error = target_vel - actual_velocities[i];
                    target_positions[i] = target;
                    target_velocities[i] = target_vel;
                    pos_errors[i] = pos_error;
                    
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

            case State::SAFETY_STOP: {
                publish_stop_command();
                update_previous_samples(positions, now_time);
                return;
            }
        }

        last_position_errors_ = pos_errors;
        if (check_position_error_safety(pos_errors, now_time.seconds())) {
            update_previous_samples(positions, now_time);
            return;
        }
        for (int i = 0; i < 6; ++i) {
            if (disabled_legs_.contains(i)) continue;
            if (!std::isfinite(pwms[i]) || !std::isfinite(actual_velocities[i]) ||
                !std::isfinite(target_positions[i]) || !std::isfinite(target_velocities[i])) {
                trigger_safety_stop(std::string("invalid trajectory or velocity: ") + leg_name(i));
                update_previous_samples(positions, now_time);
                return;
            }
        }

        std::array<float, 6> raw_pwms = pwms;
        apply_pwm_slew_limit(pwms, dt);
        publish_controller_debug(now_time, positions, target_positions, target_velocities,
                                 actual_velocities, pos_errors, raw_pwms, pwms,
                                 group_b_active, is_group_b);
        
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
        
        set_servos(cmd, *msg);
        cmd.servo_control_mode = 2;
        
        publish_motor_command(cmd);
        if (startup_completed) {
            RCLCPP_INFO(this->get_logger(), "=== Startup done, entering RUNNING ===");
            RCLCPP_INFO(this->get_logger(), "Starting from tau = %.6f (Standing)", tau_);
        }
        
        update_previous_samples(positions, now_time);
    }

    void set_servos(rinbo_msgs::msg::MotorCmdStamped& cmd,
                    const rinbo_msgs::msg::MotorStateStamped& state) {
        (void)state;
        const auto target = [&](int index) {
            return servo_targets_[index];
        };
        cmd.sl1.position_encoder = target(0);
        cmd.sl2.position_encoder = target(1);
        cmd.sl3.position_encoder = target(2);
        cmd.sr1.position_encoder = target(3);
        cmd.sr2.position_encoder = target(4);
        cmd.sr3.position_encoder = target(5);
    }

    void set_servo_hold_targets(
        rinbo_msgs::msg::MotorCmdStamped& cmd,
        const rinbo_msgs::msg::MotorStateStamped& state) {
        const std::array<uint32_t, 6> current_positions = {
            state.sl1.position_encoder, state.sl2.position_encoder,
            state.sl3.position_encoder, state.sr1.position_encoder,
            state.sr2.position_encoder, state.sr3.position_encoder};
        const auto target = [&](int index) {
            // A disconnected encoder can report UINT32 sentinels. This valid
            // neutral target is still actionable on a connected servo: the
            // protocol has no per-servo disable. Isolation must be physical.
            return disabled_legs_.contains(index)
                ? servo_targets_[index] : current_positions[index];
        };
        cmd.sl1.position_encoder = target(0);
        cmd.sl2.position_encoder = target(1);
        cmd.sl3.position_encoder = target(2);
        cmd.sr1.position_encoder = target(3);
        cmd.sr2.position_encoder = target(4);
        cmd.sr3.position_encoder = target(5);
    }

    void handshake_timer_callback() {
        if (fully_stopped_ || state_ == State::SAFETY_STOP) return;
        const auto command_subscriber_count = cmd_pub_->get_subscription_count();
        if (const auto reason = motor_handshake_->violation(command_subscriber_count)) {
            trigger_safety_stop(*reason);
            return;
        }
        if (motor_handshake_->mark_rearm_command_about_to_publish(
                command_subscriber_count)) {
            const uint32_t sequence = publish_stop_command(
                motor_handshake_->rearm_frame_id());
            if (!motor_handshake_->record_rearm_command_published(sequence)) {
                trigger_safety_stop("failed to record disabled rearm command sequence");
            }
        }
    }

    void watchdog_callback() {
        if (fully_stopped_) {
            publish_stop_command();
            rclcpp::shutdown();
            return;
        }
        if (state_ == State::SAFETY_STOP) {
            publish_stop_command();
            if (g_shutdown_requested) {
                // A failed action stays failed, but must honor the operator's
                // exit request. No motor-state callback is needed to exit.
                fully_stopped_ = true;
                RCLCPP_INFO(this->get_logger(), "Safety-stopped Tripod: shutdown requested, exiting failed action");
                rclcpp::shutdown();
            }
            return;
        }
        const auto command_subscriber_count = cmd_pub_->get_subscription_count();
        if (const auto reason = motor_handshake_->violation(command_subscriber_count)) {
            trigger_safety_stop(*reason);
            return;
        }
        const auto now_time = this->now();
        if (!motor_handshake_->ready_for_output(command_subscriber_count)) {
            if (g_shutdown_requested) {
                publish_stop_command();
                fully_stopped_ = true;
                rclcpp::shutdown();
            }
            return;
        }
        if (motor_state_received_) {
            if (const auto reason = motor_state_guard_->publisher_violation()) {
                trigger_safety_stop(*reason);
                return;
            }
        }
        if (power_snapshot_.received) {
            if (const auto reason = power_state_guard_->publisher_violation()) {
                trigger_safety_stop(*reason);
                return;
            }
        }
        if (g_shutdown_requested && state_ != State::STOPPING) {
            state_ = State::STOPPING;
            stopping_start_time_ = now_time;
            stopping_start_ratio_ = current_ratio_;
            stopping_started_ = true;
            RCLCPP_INFO(this->get_logger(), "=== Shutdown requested, slowing down... ===");
        }
        if (state_ == State::STOPPING && stopping_started_ &&
            (now_time - stopping_start_time_).seconds() > shutdown_timeout_s_) {
            trigger_safety_stop("controlled shutdown timeout");
            return;
        }
        if (motor_state_clock_rollback_detected_ || now_time < node_start_time_ ||
            (motor_state_received_ && now_time < last_motor_state_time_)) {
            trigger_safety_stop("motor-state safety clock moved backward");
            return;
        }
        if (check_power_safety(now_time)) {
            return;
        }
        if (!motor_state_received_) {
            if ((now_time - node_start_time_).seconds() > safety_.motor_state_required_after_seconds) {
                trigger_safety_stop("no /motor/state received");
            }
            return;
        }
        const double age = (now_time - last_motor_state_time_).seconds();
        if (age > safety_.motor_state_stale_seconds) {
            trigger_safety_stop("/motor/state stale for " + std::to_string(age) + "s");
        }
    }

    void estop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            trigger_safety_stop("software /estop asserted");
        }
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
    double shutdown_slowdown_duration_s_;
    double shutdown_timeout_s_;
    double stopping_start_ratio_ = 0.0;
    double start_ratio_;
    int stop_servo_control_mode_;
    
    State state_;
    bool group_b_started_;
    bool fully_stopped_;
    bool stopping_started_ = false;
    
    std::array<float, 6> initial_positions_;
    std::array<float, 6> home_offsets_;
    std::array<float, 6> prev_positions_;
    std::array<float, 6> prev_command_pwms_;
    SafetyConfig safety_;
    PowerSnapshot power_snapshot_;
    std::unique_ptr<rinbo_fsm::TripodPowerPolicy> power_policy_;
    std::array<int, 6> position_error_trip_counts_ {};
    std::array<double, 6> position_error_since_s_ {};
    rinbo_config::MotionSession& motion_session_;
    rinbo_fsm::DisabledLegs disabled_legs_;
    std::string safety_stop_reason_;
#ifdef RINBO_FSM_OFFLINE_TEST
    bool simulate_stop_publication_failure_ = false;
    bool simulate_event_publication_failure_ = false;
#endif
    int cycle_count_;
    rclcpp::Time start_time_;
    rclcpp::Time prev_time_;
    rclcpp::Time node_start_time_;
    rclcpp::Time last_motor_state_time_;
    rclcpp::Time stopping_start_time_;
    bool motor_state_received_ = false;
    bool motor_state_clock_rollback_detected_ = false;
    uint32_t motor_command_seq_ = 0;
    int64_t last_motor_command_stamp_ns_ = 0;
    
    bool initialized_;
    
    rclcpp::Publisher<rinbo_msgs::msg::MotorCmdStamped>::SharedPtr cmd_pub_;
    std::unique_ptr<rinbo_fsm::MotorArbiterHandshake> motor_handshake_;
    std::unique_ptr<rinbo_fsm::RosInputGuard> motor_state_guard_;
    std::unique_ptr<rinbo_fsm::RosInputGuard> power_state_guard_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pid_data_pub_;
    rclcpp::Publisher<rinbo_msgs::msg::ControllerDebugStamped>::SharedPtr controller_debug_pub_;
    rclcpp::Publisher<rinbo_msgs::msg::SafetyEventStamped>::SharedPtr safety_event_pub_;
    rclcpp::Subscription<rinbo_msgs::msg::MotorStateStamped>::SharedPtr state_sub_;
    rclcpp::Subscription<rinbo_msgs::msg::PowerStateStamped>::SharedPtr power_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr handshake_timer_;
    std::array<float, 6> last_position_errors_ {};
    uint32_t controller_debug_seq_ = 0;
    uint32_t safety_event_seq_ = 0;
};

#ifndef RINBO_FSM_OFFLINE_TEST
int main(int argc, char* argv[]) {
    try {
        if (rinbo_config::check_config_cli(argc, argv, "rinbo_tripod_rslip")) return 0;
        rinbo_config::MotionSession session(rinbo_config::Stage::Tripod, argc, argv);
        session.config().print_status(std::cout, "rinbo_tripod_rslip");
        rclcpp::init(argc, argv);
        signal(SIGINT, signal_handler);
        auto node = std::make_shared<PIDController>(session);
        try {
            rclcpp::spin(node);
        } catch (const std::exception& error) {
            node->fail_closed(std::string("unhandled controller error: ") + error.what());
            throw;
        }
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[FATAL] Tripod: " << error.what() << std::endl;
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
}
#endif
