#include "rclcpp/rclcpp.hpp"
#include "disabled_legs.hpp"
#include "latest_state_qos.hpp"
#include "motor_arbiter_handshake.hpp"
#include "rinbo_power_guard.hpp"
#include "ros_input_guard.hpp"
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
        disabled_legs_ = rinbo_fsm::DisabledLegs::load(*this);
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
        node_start_time_ = this->now();
        last_motor_state_time_ = node_start_time_;
        
        max_pwm_ = static_cast<float>(this->declare_parameter<double>("max_pwm", 80.0));
        stop_velocity_threshold_ = 500.0f;
        servo_homing_timeout_s_ = this->declare_parameter<double>("safety.servo_homing_timeout_s", 10.0);
        hall_search_timeout_s_ = this->declare_parameter<double>("safety.hall_search_timeout_s", 12.0);
        stop_timeout_s_ = this->declare_parameter<double>("safety.stop_timeout_s", 2.0);
        motor_state_timeout_s_ = this->declare_parameter<double>("safety.motor_state_timeout_s", 0.25);
        motor_state_required_after_s_ = this->declare_parameter<double>("safety.motor_state_required_after_s", 2.0);
        rinbo_fsm::validate_pwm(max_pwm_, "Calibration");
        rinbo_fsm::validate_bounded_timeout(
            servo_homing_timeout_s_, 10.0, "safety.servo_homing_timeout_s");
        rinbo_fsm::validate_bounded_timeout(
            hall_search_timeout_s_, 12.0, "safety.hall_search_timeout_s");
        rinbo_fsm::validate_bounded_timeout(
            stop_timeout_s_, 2.0, "safety.stop_timeout_s");
        rinbo_fsm::validate_motor_state_watchdog(
            motor_state_timeout_s_, motor_state_required_after_s_, "Calibration");
        power_guard_ = std::make_unique<rinbo_fsm::RinboPowerGuard>(
            *this, disabled_legs_.mask());
        motor_state_guard_ = std::make_unique<rinbo_fsm::RosInputGuard>(
            *this, "/motor/state");
        power_state_guard_ = std::make_unique<rinbo_fsm::RosInputGuard>(
            *this, "/power/state");

        for (int i = 0; i < 6; ++i) {
            if (disabled_legs_.contains(i)) {
                leg_states_[i] = LegState::DONE;
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
        
        RCLCPP_INFO(this->get_logger(), "=== 6-Leg Calibration FSM Started ===");
        RCLCPP_INFO(this->get_logger(), "Servo targets: sl1=%u, sl2=%u, sl3=%u, sr1=%u, sr2=%u, sr3=%u",
                    servo_targets_[0], servo_targets_[1], servo_targets_[2],
                    servo_targets_[3], servo_targets_[4], servo_targets_[5]);
        RCLCPP_INFO(this->get_logger(), "DC target velocity: %.2f rad/s", target_vel_rad_);
        RCLCPP_INFO(this->get_logger(), "Main-drive PWM cap: %.1f", max_pwm_);
        RCLCPP_INFO(this->get_logger(), "State: SERVO_HOMING");
        if (disabled_legs_.enabled()) {
            RCLCPP_WARN(this->get_logger(),
                        "DEGRADED MODE: hardware.disabled_legs=[%s]. Those main drives are forced disabled; physical servo isolation is still required.",
                        disabled_legs_.summary().c_str());
        }
    }

private:
    uint32_t publish_motor_command(
        rinbo_msgs::msg::MotorCmdStamped& cmd,
        const std::string& frame_id = std::string()) {
        if (!motor_handshake_->ready_for_output(cmd_pub_->get_subscription_count())) {
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

    void set_servos(rinbo_msgs::msg::MotorCmdStamped& cmd,
                    const std::array<uint32_t, 6>& current_positions) {
        const auto target = [&](int index) {
            (void)current_positions;
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
        bool all_homed = true;
        for (int i = 0; i < 6; i++) {
            if (disabled_legs_.contains(i)) {
                continue;
            }
            int32_t error = static_cast<int32_t>(servo_targets_[i]) - 
                           static_cast<int32_t>(servo_positions[i]);
            if (std::abs(error) > static_cast<int32_t>(servo_tolerance_)) {
                all_homed = false;
            }
        }

        if ((now_time - servo_wait_start_time_).seconds() > servo_homing_timeout_s_) {
            trigger_safety_stop("healthy servo homing timeout");
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
            if (disabled_legs_.contains(i)) {
                done_count++;
                continue;
            }
            switch (leg_states_[i]) {
                case LegState::SPINNING: {
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

                    if ((now_time - stopping_entry_times_[i]).seconds() > stop_timeout_s_) {
                        trigger_safety_stop(std::string("motor stop timeout: ") + leg_name(i));
                        break;
                    }
                    
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
            "DC_SPINNING | t: %.2f | Complete/skipped: %d/6 | Hall: [%d %d %d %d %d %d]",
            elapsed, done_count,
            hall_effects[0], hall_effects[1], hall_effects[2],
            hall_effects[3], hall_effects[4], hall_effects[5]);
        
        if (done_count == 6) {
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
        if (!safety_stopped_) {
            safety_stopped_ = true;
            safety_stop_reason_ = reason;
            RCLCPP_ERROR(this->get_logger(), "CALIBRATION SAFETY STOP: %s", reason.c_str());
        }
        publish_stop_command();
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

    float kp_, kd_, k_ff_;
    float max_pwm_;
    float stop_velocity_threshold_;
    double servo_homing_timeout_s_;
    double hall_search_timeout_s_;
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

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalibrationFSM>());
    rclcpp::shutdown();
    return 0;
}
