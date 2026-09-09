#include "motion_effort.hpp"
#include "control_velocity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "disabled_legs.hpp"
#include "robot_config.hpp"
#include "motor_tracking.hpp"
#include "manual_motion.hpp"
#include <iostream>
#include <cstdio>
#include "latest_state_qos.hpp"
#include "motor_arbiter_handshake.hpp"
#include "rinbo_power_guard.hpp"
#include "ros_input_guard.hpp"
#include "bridge_input_discovery.hpp"
#include "safety_invariants.hpp"
#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include "rinbo_msgs/msg/motor_state_stamped.hpp"
#include "rinbo_msgs/msg/power_state_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>
#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <csignal>
#include <thread>

namespace rinbo_cali_lifecycle {
volatile std::sig_atomic_t stop_requested = 0;
void signal_handler(int) { stop_requested = 1; }
void install_signal_handlers() {
    stop_requested = 0;
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
}
}

enum class CalibState {
    SERVO_HOMING,
    WAIT_SERVO,
    DC_SPINNING, 
    DONE
};

enum class LegState {
    SPINNING, 
    STOPPING,
    RESETTING,
    DONE,
    SKIPPED
};

class CalibrationFSM : public rclcpp::Node {
    friend struct CalibrationOfflineTestAccess;
public:
    explicit CalibrationFSM(rinbo_config::MotionSession& session)
        : Node("rinbo_cali", session.config().node_options("rinbo_cali")),
          motion_session_(session) {
#ifndef RINBO_FSM_OFFLINE_TEST
        rinbo_fsm::wait_for_bridge_inputs(*this, [] {
            return rinbo_cali_lifecycle::stop_requested != 0;
        });
#endif
        const auto site_disabled = rinbo_fsm::DisabledLegs::load(*this);
        std::vector<std::string> excluded;
        for (const auto* name : rinbo_fsm::DisabledLegs::kLegNames)
            if (std::find(session.active_legs().begin(), session.active_legs().end(), name)
                    == session.active_legs().end()) excluded.emplace_back(name);
        disabled_legs_ = rinbo_fsm::DisabledLegs::from_names(excluded);
        if (disabled_legs_.healthy_count() == 0)
            throw std::runtime_error("没有可測試腿: all six main drives are disabled");
        effort_ = rinbo_fsm::load_motion_effort(*this);
        velocity_filter_s_ = declare_parameter<double>("velocity_filter_time_constant_s", 0.02);
        if (!std::isfinite(velocity_filter_s_) || velocity_filter_s_ < 0.0)
            throw std::invalid_argument("velocity_filter_time_constant_s must be finite and >= 0");
        
        RCLCPP_INFO(get_logger(), "Motion effort: kp=%.4f kd=%.4f k_ff=%.4f friction_pwm=%.1f fade_counts_s=%.1f",
                    effort_.kp, effort_.kd, effort_.k_ff, effort_.friction_pwm, effort_.friction_velocity_counts_s);
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
        node_start_time_ = this->now();
        last_motor_state_time_ = node_start_time_;
        
        max_pwm_ = static_cast<float>(this->declare_parameter<double>("max_pwm", 80.0));
        stop_velocity_threshold_ = 500.0f;
        servo_homing_timeout_s_ = this->declare_parameter<double>("safety.servo_homing_timeout_s", 20.0);
        hall_search_timeout_s_ = this->declare_parameter<double>("safety.hall_search_timeout_s", 30.0);
        stop_timeout_s_ = this->declare_parameter<double>("safety.stop_timeout_s", 5.0);
        motor_state_timeout_s_ = this->declare_parameter<double>("safety.motor_state_timeout_s", 0.25);
        motor_state_required_after_s_ = this->declare_parameter<double>("safety.motor_state_required_after_s", 2.0);
        rinbo_fsm::validate_pwm(max_pwm_, "Calibration");
        rinbo_fsm::validate_bounded_timeout(
            servo_homing_timeout_s_, 600.0, "safety.servo_homing_timeout_s");
        rinbo_fsm::validate_bounded_timeout(
            hall_search_timeout_s_, 600.0, "safety.hall_search_timeout_s");
        rinbo_fsm::validate_bounded_timeout(
            stop_timeout_s_, 600.0, "safety.stop_timeout_s");
        rinbo_fsm::validate_motor_state_watchdog(
            motor_state_timeout_s_, motor_state_required_after_s_, "Calibration");
        power_guard_ = std::make_unique<rinbo_fsm::RinboPowerGuard>(
            *this, site_disabled.mask());
        motor_state_guard_ = std::make_unique<rinbo_fsm::RosInputGuard>(
            *this, "/motor/state");
        power_state_guard_ = std::make_unique<rinbo_fsm::RosInputGuard>(
            *this, "/power/state");

        for (int i = 0; i < 6; ++i) {
            if (disabled_legs_.contains(i)) {
                leg_states_[i] = LegState::SKIPPED;
                RCLCPP_INFO(this->get_logger(), "%s: SKIPPED (%s)", leg_name(i),
                    site_disabled.contains(i) ? "main drive disabled" : "not selected for this calibration");
            }
        }
        
        cmd_pub_ = this->create_publisher<rinbo_msgs::msg::MotorCmdStamped>(
            "/motor/command", 10);
        motor_handshake_ =
            std::make_unique<rinbo_fsm::MotorArbiterHandshake>(*this);
        
        state_sub_ = this->create_subscription<rinbo_msgs::msg::MotorStateStamped>(
            "/motor/state", rinbo_fsm::latest_state_qos(),
            std::bind(
                &CalibrationFSM::state_callback, this,
                std::placeholders::_1, std::placeholders::_2));
        power_state_sub_ = this->create_subscription<rinbo_msgs::msg::PowerStateStamped>(
            "/power/state", rinbo_fsm::latest_state_qos(),
            std::bind(
                &CalibrationFSM::power_state_callback, this,
                std::placeholders::_1, std::placeholders::_2));
        estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/estop", 10,
            std::bind(&CalibrationFSM::estop_callback, this, std::placeholders::_1));

        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&CalibrationFSM::watchdog_callback, this));
        handshake_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&CalibrationFSM::handshake_timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "=== Calibration FSM Started: %d healthy legs ===",
                    disabled_legs_.healthy_count());
        RCLCPP_INFO(this->get_logger(), "Servo targets: sl1=%u, sl2=%u, sl3=%u, sr1=%u, sr2=%u, sr3=%u",
                    servo_targets_[0], servo_targets_[1], servo_targets_[2],
                    servo_targets_[3], servo_targets_[4], servo_targets_[5]);
        RCLCPP_INFO(this->get_logger(), "DC target velocity: %.2f rad/s", target_vel_rad_);
        RCLCPP_INFO(this->get_logger(), "Main-drive PWM cap: %.1f", max_pwm_);
        RCLCPP_INFO(this->get_logger(), "State: SERVO_HOMING");
        if (session.scoped()) {
            RCLCPP_INFO(this->get_logger(),
                "Selected-leg calibration: %d legs. Other main drives stay disabled; unselected servos hold feedback positions (global servo control remains shared).",
                disabled_legs_.healthy_count());
        } else if (disabled_legs_.enabled()) {
            RCLCPP_WARN(this->get_logger(),
                        "supported_leg_test: hardware.disabled_legs=[%s]. Remaining legs follow independent test references; robot support/suspension is required, with no ground stability guarantee. Mask disables main drives only; global servo control still commands every connected servo, requiring physical isolation where needed.",
                        disabled_legs_.summary().c_str());
        }
    }

    void fail_closed(const std::string& reason) { trigger_safety_stop(reason); }

    // Called only after the single-threaded executor stops dispatching callbacks,
    // while ROS is still valid. A normal handoff preserves the completed receipt;
    // interruption before completion and real failures must invalidate it.
    bool finish_shutdown() {
        watchdog_timer_->cancel();
        handshake_timer_->cancel();
        if (state_ != CalibState::DONE && !safety_stopped_)
            trigger_safety_stop("calibration interrupted before completion");
        for (int i = 0; i < 8; ++i) {
            publish_stop_command();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return state_ == CalibState::DONE && !safety_stopped_;
    }

private:
    uint32_t publish_motor_command(
        rinbo_msgs::msg::MotorCmdStamped& cmd,
        const std::string& frame_id = std::string()) {
        if (safety_stopped_ || !motor_handshake_->ready_for_output(cmd_pub_->get_subscription_count())) {
            disable_all_legs(cmd);
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

    bool is_left_leg(int idx) {
        return (idx == 0 || idx == 1 || idx == 2);
    }

    const char* leg_name(int idx) {
        static const char* names[] = {"L1", "L2", "L3", "R1", "R2", "R3"};
        return names[idx];
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
        if (!input.fresh_at_validation(motor_state_timeout_s_)) {
            trigger_safety_stop(
                "/motor/state became stale during publisher validation");
            return;
        }
        if (motor_state_received_ && arrival_time < last_motor_state_time_) {
            motor_state_clock_rollback_detected_ = true;
        }
        last_motor_state_time_ = arrival_time;
        motor_state_received_ = true;
        float dt = (now_time - prev_time_).seconds();
        if (dt <= 0) dt = 0.001;
        
        std::array<float, 6> positions = {
            msg->l1.position, msg->l2.position, msg->l3.position,
            msg->r1.position, msg->r2.position, msg->r3.position
        };
        for (int i = 0; i < 6; ++i) {
            if (!disabled_legs_.contains(i) && !std::isfinite(positions[i])) {
                trigger_safety_stop(std::string("invalid motor position: ") + leg_name(i));
                update_previous_samples(positions, now_time);
                return;
            }
        }
        
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
            control_velocity_[i] = velocity_filter_[i].update(velocities[i], dt, velocity_filter_s_);
        }
        
        auto cmd = rinbo_msgs::msg::MotorCmdStamped();
        disable_all_legs(cmd);
        set_servos(cmd, servo_positions);
        cmd.servo_control_mode = 2;

        if (safety_stopped_) {
            publish_stop_command();
            update_previous_samples(positions, now_time);
            return;
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

        if (!power_guard_->ready_for_output(now_time.seconds())) {
            disable_all_legs(cmd);
            cmd.servo_control_mode = 0;
            publish_motor_command(cmd);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Waiting for valid in-range /power/state; motor and servo output remain disabled");
            update_previous_samples(positions, now_time);
            return;
        }

        if (!motor_handshake_->active_output_confirmed()) {
            disable_all_legs(cmd);
            set_servo_hold_targets(cmd, servo_positions);
            cmd.servo_control_mode = 2;
            const uint32_t probe_sequence = publish_motor_command(
                cmd, motor_handshake_->active_probe_frame_id());
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
        
        trace_targets_ = positions;
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
        
        if (safety_stopped_) {
            disable_all_legs(cmd);
            cmd.servo_control_mode = 0;
        }
        publish_motor_command(cmd);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "CALIBRATION TRACE (raw counts, normalized velocity/PWM): %s",
            rinbo_fsm::tracking_trace(positions, trace_targets_, velocities, cmd).c_str());
        
        for (int i = 0; i < 6; i++) {
            prev_positions_[i] = positions[i];
        }
        prev_time_ = now_time;
    }
    
    void disable_all_legs(rinbo_msgs::msg::MotorCmdStamped& cmd) {
        rinbo_fsm::force_main_drive_disabled(cmd.l1);
        rinbo_fsm::force_main_drive_disabled(cmd.l2);
        rinbo_fsm::force_main_drive_disabled(cmd.l3);
        rinbo_fsm::force_main_drive_disabled(cmd.r1);
        rinbo_fsm::force_main_drive_disabled(cmd.r2);
        rinbo_fsm::force_main_drive_disabled(cmd.r3);
    }

    void set_servos(rinbo_msgs::msg::MotorCmdStamped& cmd,
                    const std::array<uint32_t, 6>& current_positions) {
        const auto target = [&](int index) {
            if (motion_session_.scoped() && disabled_legs_.contains(index))
                return current_positions[index];
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
        const std::array<uint32_t, 6>& current_positions) {
        const auto target = [&](int index) {
            if (motion_session_.scoped()) return current_positions[index];
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
    
    void set_leg_cmd(rinbo_msgs::msg::MotorCmdStamped& cmd, int leg_idx, 
                     bool enable, float pwm, bool reset_position) {
        if (disabled_legs_.contains(leg_idx)) {
            return;
        }
        if (!std::isfinite(pwm)) {
            trigger_safety_stop(std::string("invalid main-drive PWM: ") + leg_name(leg_idx));
            return;
        }
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
        servo_wait_start_time_ = now_time;
        RCLCPP_INFO(this->get_logger(), "State: WAIT_SERVO");
    }
    
    void handle_wait_servo(rinbo_msgs::msg::MotorCmdStamped& cmd,
                           const std::array<uint32_t, 6>& servo_positions,
                           rclcpp::Time now_time) {
        std::string waiting_legs;
        for (int i = 0; i < 6; i++) {
            if (disabled_legs_.contains(i)) {
                continue;
            }
            const int64_t error = static_cast<int64_t>(servo_targets_[i]) -
                                  static_cast<int64_t>(servo_positions[i]);
            if (std::abs(error) > static_cast<int64_t>(servo_tolerance_)) {
                if (!waiting_legs.empty()) waiting_legs += ",";
                waiting_legs += leg_name(i);
            }
        }
        const bool all_homed = disabled_legs_.all_healthy([&](int i) {
            return std::abs(static_cast<int64_t>(servo_targets_[i]) -
                            static_cast<int64_t>(servo_positions[i])) <= servo_tolerance_;
        });

        if ((now_time - servo_wait_start_time_).seconds() > servo_homing_timeout_s_) {
            trigger_safety_stop("healthy servo homing timeout: " +
                (waiting_legs.empty() ? std::string("position stability not confirmed") : waiting_legs));
            return;
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
                RCLCPP_INFO(this->get_logger(), "All %d healthy servos homed! State: DC_SPINNING",
                            disabled_legs_.healthy_count());
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
        trace_targets_ = positions;
        int done_count = 0;
        
        for (int i = 0; i < 6; i++) {
            if (safety_stopped_) break;
            if (disabled_legs_.contains(i)) {
                continue;
            }
            switch (leg_states_[i]) {
                case LegState::SPINNING: {
                    if (rinbo_fsm::forward_travel(i, start_positions_[i], positions[i]) <
                        -rinbo_fsm::kWrongWayTravelCounts) {
                        trigger_safety_stop(std::string("opposite encoder travel: ") + leg_name(i) +
                            " start=" + std::to_string(start_positions_[i]) +
                            " actual=" + std::to_string(positions[i]) +
                            " expected=" + (i < 3 ? "decreasing counts" : "increasing counts"));
                        break;
                    }
                    if (!hall_effects[i]) {
                        leg_states_[i] = LegState::STOPPING;
                        stop_start_times_[i] = now_time;
                        stopping_entry_times_[i] = now_time;
                        RCLCPP_INFO(this->get_logger(), "%s: Hall detected! Stopping...", leg_name(i));
                    } else {
                        if (elapsed > hall_search_timeout_s_) {
                            trigger_safety_stop(std::string("hall search timeout: ") + leg_name(i));
                            break;
                        }
                        const auto reference = rinbo_fsm::search_reference(elapsed, target_vel_counts_);
                        float target_pos = start_positions_[i] +
                            (is_left_leg(i) ? -reference.position : reference.position);
                        float target_vel = reference.velocity;
                        trace_targets_[i] = target_pos;
                        
                        float pos_error = is_left_leg(i) ? 
                            -(target_pos - positions[i]) : (target_pos - positions[i]);

                        
                        float pwm = effort_.command(pos_error, target_vel, control_velocity_[i]);
                        pwm = std::clamp(pwm, -max_pwm_, max_pwm_);
                        
                        set_leg_cmd(cmd, i, true, pwm, false);
                    }
                    break;
                }
                
                case LegState::STOPPING: {
                    set_leg_cmd(cmd, i, true, 0, false);

                    if ((now_time - stopping_entry_times_[i]).seconds() > stop_timeout_s_) {
                        trigger_safety_stop(std::string("motor stop timeout: ") + leg_name(i));
                        break;
                    }
                    
                    if (std::fabs(velocities[i]) < stop_velocity_threshold_) {
                        if ((now_time - stop_start_times_[i]).seconds() > 0.3) {
                            set_leg_cmd(cmd, i, false, 0, true);
                            leg_states_[i] = LegState::RESETTING;
                            stopping_entry_times_[i] = now_time;
                            reset_last_sent_times_[i] = now_time;
                            RCLCPP_INFO(this->get_logger(), "%s: RESETTING (waiting for zero feedback)", leg_name(i));
                        }
                    } else {
                        stop_start_times_[i] = now_time;
                    }
                    break;
                }
                
                case LegState::RESETTING: {
                    set_leg_cmd(cmd, i, false, 0, false);
                    if (std::fabs(positions[i]) <= 100.0f &&
                        std::fabs(velocities[i]) < stop_velocity_threshold_) {
                        leg_states_[i] = LegState::DONE;
                        RCLCPP_INFO(this->get_logger(), "%s: DONE! Zero feedback confirmed.", leg_name(i));
                    } else if ((now_time - stopping_entry_times_[i]).seconds() > stop_timeout_s_) {
                        trigger_safety_stop(std::string("position reset timeout: ") + leg_name(i) +
                            " pos=" + std::to_string(positions[i]) +
                            " velocity=" + std::to_string(velocities[i]) +
                            " elapsed=" + std::to_string((now_time - stopping_entry_times_[i]).seconds()));
                    } else if (std::fabs(velocities[i]) < stop_velocity_threshold_ &&
                               (now_time - reset_last_sent_times_[i]).seconds() >= 0.1) {
                        // Resend a missed one-cycle reset only while stationary,
                        // with drive disabled, inside the original deadline.
                        // Zero feedback, never transmission, completes calibration.
                        set_leg_cmd(cmd, i, false, 0, true);
                        reset_last_sent_times_[i] = now_time;
                        RCLCPP_INFO(this->get_logger(), "%s: reset retry | pos=%.0f velocity=%.0f elapsed=%.2f",
                            leg_name(i), positions[i], velocities[i],
                            (now_time - stopping_entry_times_[i]).seconds());
                    }
                    break;
                }
                case LegState::DONE: {
                    set_leg_cmd(cmd, i, false, 0, false);
                    done_count++;
                    break;
                }
                case LegState::SKIPPED:
                    break;
            }
        }
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "DC_SPINNING | t: %.2f | Healthy complete: %d/%d | SKIPPED: %d | Hall: [%d %d %d %d %d %d]",
            elapsed, done_count, disabled_legs_.healthy_count(), disabled_legs_.count(),
            hall_effects[0], hall_effects[1], hall_effects[2],
            hall_effects[3], hall_effects[4], hall_effects[5]);
        
        if (!safety_stopped_ && disabled_legs_.healthy_complete(done_count)) {
            try {
                motion_session_.complete();
            } catch (const std::exception& error) {
                trigger_safety_stop(std::string("cannot persist calibration receipt: ") + error.what());
                return;
            }
            state_ = CalibState::DONE;
            RCLCPP_INFO(this->get_logger(), "=== ALL %d HEALTHY LEGS CALIBRATED (%d disabled) ===",
                        disabled_legs_.healthy_count(), disabled_legs_.count());
            RCLCPP_INFO(this->get_logger(), "State: DONE (Press Ctrl+C to continue)");
        }
    }
    
    void handle_done(rinbo_msgs::msg::MotorCmdStamped& cmd,
                     const std::array<float, 6>& positions) {
        disable_all_legs(cmd);
        cmd.servo_control_mode = 0;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "DONE | Positions: L1=%.0f, L2=%.0f, L3=%.0f, R1=%.0f, R2=%.0f, R3=%.0f",
            positions[0], positions[1], positions[2],
            positions[3], positions[4], positions[5]);
    }

    void update_previous_samples(const std::array<float, 6>& positions, rclcpp::Time now_time) {
        prev_positions_ = positions;
        prev_time_ = now_time;
    }

    uint32_t publish_stop_command(const std::string& frame_id = std::string()) {
        auto cmd = rinbo_msgs::msg::MotorCmdStamped();
        disable_all_legs(cmd);
        cmd.servo_control_mode = 0;
        return publish_motor_command(cmd, frame_id);
    }

    void trigger_safety_stop(const std::string& reason) {
        const bool first_failure = !safety_stopped_;
        safety_stopped_ = true;
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
            RCLCPP_ERROR(this->get_logger(), "CALIBRATION SAFETY STOP: %s", safety_stop_reason_.c_str());
            std::fprintf(stderr, "CALIBRATION SAFETY STOP: %s\n", safety_stop_reason_.c_str());
            try {
                motion_session_.invalidate();
            } catch (const std::exception& error) {
                std::fprintf(stderr, "[FATAL] cannot invalidate stage receipts: %s\n", error.what());
            } catch (...) {
                std::fprintf(stderr, "[FATAL] cannot invalidate stage receipts\n");
            }
        }
    }

    void handshake_timer_callback() {
        if (safety_stopped_ || state_ == CalibState::DONE) return;
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
        if (safety_stopped_ || state_ == CalibState::DONE) {
            publish_stop_command();
            return;
        }
        const auto command_subscriber_count = cmd_pub_->get_subscription_count();
        if (const auto reason = motor_handshake_->violation(command_subscriber_count)) {
            trigger_safety_stop(*reason);
            return;
        }
        if (!motor_handshake_->ready_for_output(command_subscriber_count)) return;
        const auto now_time = this->now();
        if (motor_state_received_) {
            if (const auto reason = motor_state_guard_->publisher_violation()) {
                trigger_safety_stop(*reason);
                return;
            }
        }
        if (power_state_received_) {
            if (const auto reason = power_state_guard_->publisher_violation()) {
                trigger_safety_stop(*reason);
                return;
            }
        }
        if (motor_state_clock_rollback_detected_ || now_time < node_start_time_ ||
            (motor_state_received_ && now_time < last_motor_state_time_)) {
            trigger_safety_stop("motor-state safety clock moved backward");
            return;
        }
        if (const auto reason = power_guard_->violation(now_time.seconds(), node_start_time_.seconds())) {
            trigger_safety_stop(*reason);
            return;
        }
        if (!motor_state_received_) {
            if ((now_time - node_start_time_).seconds() > motor_state_required_after_s_) {
                trigger_safety_stop("no /motor/state received");
            }
            return;
        }
        const double age = (now_time - last_motor_state_time_).seconds();
        if (age > motor_state_timeout_s_) {
            trigger_safety_stop("/motor/state stale for " + std::to_string(age) + "s");
        }
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
        const double arrival_s = rclcpp::Time(input.observed_ns, clock_type).seconds();
        const double validated_s = rclcpp::Time(input.validated_ns, clock_type).seconds();
        power_state_received_ = true;
        power_guard_->update(*msg, arrival_s);
        if (const auto reason = power_guard_->violation(
                validated_s, node_start_time_.seconds())) {
            trigger_safety_stop(*reason);
        }
    }

    void estop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            trigger_safety_stop("software /estop asserted");
        }
    }

    std::array<rinbo_fsm::ControlVelocity, 6> velocity_filter_{};
    std::array<float, 6> control_velocity_{};
    rinbo_fsm::MotionEffort effort_;
    float max_pwm_;
    double velocity_filter_s_ = 0.02;
    std::array<float, 6> trace_targets_{};
    float stop_velocity_threshold_;
    double servo_homing_timeout_s_;
    double hall_search_timeout_s_;
    std::array<rclcpp::Time, 6> reset_last_sent_times_;
    double stop_timeout_s_;
    double motor_state_timeout_s_;
    double motor_state_required_after_s_;
    
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
    std::array<rclcpp::Time, 6> stopping_entry_times_;
    rclcpp::Time prev_time_;
    rclcpp::Time start_time_;
    rclcpp::Time state_start_time_; 
    rclcpp::Time servo_wait_start_time_;
    rclcpp::Time node_start_time_;
    rclcpp::Time last_motor_state_time_;
    bool motor_state_received_ = false;
    bool power_state_received_ = false;
    bool motor_state_clock_rollback_detected_ = false;
    bool safety_stopped_ = false;
    uint32_t motor_command_seq_ = 0;
    int64_t last_motor_command_stamp_ns_ = 0;
    std::string safety_stop_reason_;
    rinbo_config::MotionSession& motion_session_;
    rinbo_fsm::DisabledLegs disabled_legs_;
    std::unique_ptr<rinbo_fsm::RinboPowerGuard> power_guard_;
    std::unique_ptr<rinbo_fsm::MotorArbiterHandshake> motor_handshake_;
    std::unique_ptr<rinbo_fsm::RosInputGuard> motor_state_guard_;
    std::unique_ptr<rinbo_fsm::RosInputGuard> power_state_guard_;
    
    rclcpp::Publisher<rinbo_msgs::msg::MotorCmdStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<rinbo_msgs::msg::MotorStateStamped>::SharedPtr state_sub_;
    rclcpp::Subscription<rinbo_msgs::msg::PowerStateStamped>::SharedPtr power_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr handshake_timer_;
};

int spin_calibration_until_stop(const std::shared_ptr<CalibrationFSM>& node) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    try {
        while (rclcpp::ok() && !rinbo_cali_lifecycle::stop_requested) {
            executor.spin_some(std::chrono::milliseconds(5));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        executor.remove_node(node);
        const bool completed = node->finish_shutdown();
        if (completed)
            RCLCPP_INFO(node->get_logger(), "CALIBRATION HANDOFF READY: callbacks stopped, motor output disabled, receipt preserved.");
        return completed ? 0 : 1;
    } catch (const std::exception& error) {
        node->fail_closed(std::string("unhandled controller error: ") + error.what());
        throw;
    }
}

#ifndef RINBO_FSM_OFFLINE_TEST
int main(int argc, char* argv[]) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") {
            std::cout << "rinbo_cali [--plan FILE [--check-config]]\n"
                         "--plan: calibrate only legs selected in a validated manual plan.\n"
                         "Without --plan: calibrate all site-enabled legs.\n";
            return 0;
        }
        const bool selected_plan = argc > 1 && std::string(argv[1]) == "--plan";
        std::unique_ptr<rinbo_config::MotionSession> session;
        if (selected_plan) {
            if (argc != 3 && !(argc == 4 && std::string(argv[3]) == "--check-config"))
                throw std::runtime_error("Use rinbo_cali --plan FILE [--check-config]");
            const auto plan = rinbo_manual::load(argv[2]);
            const auto config = rinbo_config::RobotConfig::load();
            rinbo_manual::validate_mask(plan, rinbo_fsm::DisabledLegs::from_names(config.disabled_legs()).mask());
            const auto legs = rinbo_manual::selected_legs(plan);
            std::cout << "Calibration selected legs:";
            for (const auto& leg : legs) std::cout << ' ' << leg;
            std::cout << '\n';
            if (argc == 4) {
                config.print_status(std::cout, "rinbo_cali");
                std::cout << "CHECK ONLY: no ROS initialized, no commands sent.\n";
                return 0;
            }
            session = std::make_unique<rinbo_config::MotionSession>(
                rinbo_config::Stage::Calibration, rinbo_config::kConfigPath, legs);
        } else {
            if (rinbo_config::check_config_cli(argc, argv, "rinbo_cali")) return 0;
            session = std::make_unique<rinbo_config::MotionSession>(rinbo_config::Stage::Calibration, argc, argv);
        }
        session->config().print_status(std::cout, "rinbo_cali");
        rclcpp::init(selected_plan ? 0 : argc, selected_plan ? nullptr : argv,
                     rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
        rinbo_cali_lifecycle::install_signal_handlers();
        auto node = std::make_shared<CalibrationFSM>(*session);
        const int result = spin_calibration_until_stop(node);
        node.reset();
        rclcpp::shutdown();
        return result;
    } catch (const std::exception& error) {
        std::cerr << "[FATAL] Calibration: " << error.what() << std::endl;
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
}
#endif
