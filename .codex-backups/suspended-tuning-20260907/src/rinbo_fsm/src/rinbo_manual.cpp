#include "manual_motion.hpp"
#include "robot_config.hpp"
#include "disabled_legs.hpp"
#include "latest_state_qos.hpp"
#include "motor_arbiter_handshake.hpp"
#include "rinbo_power_guard.hpp"
#include "ros_input_guard.hpp"
#include "bridge_input_discovery.hpp"
#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include "rinbo_msgs/msg/motor_state_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double>(a-b).count();
}
volatile std::sig_atomic_t interrupted = 0;
void signal_handler(int) { interrupted = 1; }
}

class ManualController : public rclcpp::Node {
    friend struct ManualOfflineTestAccess;
public:
    ManualController(rinbo_config::MotionSession& session, const rinbo_manual::Plan& plan)
        // Reuse the authoritative site's Standing safety settings and mask.
        // Motion parameters live in the separately validated manual plan.
        : Node("rinbo_manual", session.config().node_options("rinbo_standing")),
          session_(session), plan_(plan), disabled_(rinbo_fsm::DisabledLegs::load(*this)) {
#ifndef RINBO_FSM_OFFLINE_TEST
        rinbo_fsm::wait_for_bridge_inputs(*this, [] { return interrupted != 0; });
#endif
        rinbo_manual::validate_mask(plan_, disabled_.mask());
        cap_ = std::min(plan_.max_pwm, declare_parameter<double>("max_pwm", 80.0));
        state_timeout_ = declare_parameter<double>("safety.motor_state_timeout_s", 0.25);
        required_after_ = declare_parameter<double>("safety.motor_state_required_after_s", 2.0);
        rinbo_fsm::validate_pwm(cap_, "Manual");
        rinbo_fsm::validate_motor_state_watchdog(state_timeout_, required_after_, "Manual");
        start_ros_ = now().seconds();
        previous_ros_ = start_ros_;
        power_ = std::make_unique<rinbo_fsm::RinboPowerGuard>(*this, disabled_.mask());
        motor_input_ = std::make_unique<rinbo_fsm::RosInputGuard>(*this, "/motor/state");
        power_input_ = std::make_unique<rinbo_fsm::RosInputGuard>(*this, "/power/state");
        command_ = create_publisher<rinbo_msgs::msg::MotorCmdStamped>("/motor/command", 10);
        status_ = create_publisher<std_msgs::msg::String>("/rinbo/manual/status", 1);
        handshake_ = std::make_unique<rinbo_fsm::MotorArbiterHandshake>(*this);
        motor_sub_ = create_subscription<rinbo_msgs::msg::MotorStateStamped>(
            "/motor/state", rinbo_fsm::latest_state_qos(),
            [this](rinbo_msgs::msg::MotorStateStamped::SharedPtr m, const rclcpp::MessageInfo& info) {
                motor_callback(*m, info);
            });
        power_sub_ = create_subscription<rinbo_msgs::msg::PowerStateStamped>(
            "/power/state", rinbo_fsm::latest_state_qos(),
            [this](rinbo_msgs::msg::PowerStateStamped::SharedPtr m, const rclcpp::MessageInfo& info) {
                const auto input = power_input_->accept(m->header, info);
                if (input.pending_startup) return;
                if (input.violation) { stop(*input.violation, true); return; }
                power_received_ = true;
                power_->update(*m, input.observed_ns / 1e9);
                if (const auto why = power_->violation(input.validated_ns / 1e9, start_ros_)) stop(*why, true);
            });
        estop_sub_ = create_subscription<std_msgs::msg::Bool>("/estop", 10,
            [this](std_msgs::msg::Bool::SharedPtr m) { if (m->data) stop("/estop asserted", true); });
        timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { tick(); });
        RCLCPP_INFO(get_logger(), "WAIT: manual plan loaded; PWM cap %.1f; servo mode 0. Waiting for Bridge and fresh feedback.", cap_);
        last_tick_ = Clock::now();
    }
    bool finished() const { return stopping_ && seconds(Clock::now(), stop_at_) >= 0.3; }
    bool failed() const { return failed_; }
    void stop(const std::string& reason, bool fault) {
        if (!stopping_) {
            stopping_ = true;
            failed_ = fault;
            stop_at_ = Clock::now();
            reason_ = reason;
            // A failed publication must not skip receipt invalidation.
            try { publish_disabled(); }
            catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "Disable publication failed: %s", e.what()); }
            if (fault) {
                RCLCPP_ERROR(get_logger(), "MANUAL SAFETY STOP: %s", reason.c_str());
                try { session_.invalidate(); }
                catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "Cannot invalidate receipts: %s", e.what()); }
            } else RCLCPP_INFO(get_logger(), "STOP: %s; disabling all main drives", reason.c_str());
        }
        publish_disabled();
    }
private:
    uint32_t publish(rinbo_msgs::msg::MotorCmdStamped cmd, const std::string& frame = "") {
        if (stopping_ || !handshake_->ready_for_output(command_->get_subscription_count()))
            cmd = rinbo_msgs::msg::MotorCmdStamped{};
        rinbo_fsm::enforce_disabled_leg_commands(cmd, disabled_);
        const bool active = cmd.l1.enable || cmd.l2.enable || cmd.l3.enable ||
                            cmd.r1.enable || cmd.r2.enable || cmd.r3.enable;
        // Graph queries can block. Recheck age at the actual output boundary,
        // rather than extending feedback freshness by time spent validating it.
        const auto at_output = Clock::now();
        if (active && (seconds(at_output, last_feedback_) > state_timeout_ ||
                       seconds(at_output, last_tick_) > 0.1 ||
                       !power_->ready_for_output(now().seconds()))) {
            stop("feedback/power/control timing expired before publication", true);
            return seq_;
        }
        const auto stamp = std::max(now().nanoseconds(), last_stamp_ + 1);
        last_stamp_ = stamp;
        cmd.header.stamp.sec = static_cast<int32_t>(stamp / 1000000000LL);
        cmd.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000000000LL);
        cmd.header.seq = ++seq_;
        cmd.header.frame_id = frame.empty() ? handshake_->command_frame_id() : frame;
        command_->publish(cmd);
        return seq_;
    }
    uint32_t publish_disabled(const std::string& frame = "") {
        return publish(rinbo_msgs::msg::MotorCmdStamped{}, frame);
    }
    void motor_callback(const rinbo_msgs::msg::MotorStateStamped& msg, const rclcpp::MessageInfo& info) {
        const auto received = Clock::now();
        const auto input = motor_input_->accept(msg.header, info);
        if (input.pending_startup) return;
        if (input.violation) { stop(*input.violation, true); return; }
        if (!input.fresh_at_validation(state_timeout_)) { stop("motor feedback stale during validation", true); return; }
        const std::array<double, 6> raw{msg.l1.position, msg.l2.position, msg.l3.position,
                                      msg.r1.position, msg.r2.position, msg.r3.position};
        const double dt = seconds(received, last_feedback_);
        for (size_t i = 0; i < 6; ++i) {
            if (disabled_.contains(i)) continue;
            if (!std::isfinite(raw[i])) { stop(std::string("invalid encoder: ") + rinbo_manual::names[i], true); return; }
            const double q = rinbo_manual::encoder_degrees(i, raw[i]);
            if (motor_received_) {
                if (dt <= 0 || dt > state_timeout_) { stop("motor feedback gap", true); return; }
                const double v = (q - position_[i]) / dt;
                // Covers unexpected reset/jump and gross overspeed on selected motors.
                if (plan_.legs[i].mode != rinbo_manual::Mode::Off && std::abs(v) > 360.0) {
                    stop(std::string("encoder jump/overspeed: ") + rinbo_manual::names[i], true); return;
                }
                velocity_[i] = v;
            }
            position_[i] = q;
        }
        have_velocity_ = motor_received_;
        motor_received_ = true;
        last_feedback_ = received;
    }
    void report(double elapsed) {
        if (seconds(Clock::now(), last_report_) < 0.5) return;
        last_report_ = Clock::now();
        std::ostringstream out;
        out << std::fixed << std::setprecision(1) << "t=" << elapsed << "s";
        for (size_t i = 0; i < 6; ++i) {
            out << " | " << rinbo_manual::names[i] << ": ";
            if (disabled_.contains(i)) out << "MASKED";
            else if (plan_.legs[i].mode == rinbo_manual::Mode::Off) out << "OFF";
            else out << position_[i] << "deg " << velocity_[i] << "deg/s PWM=" << last_pwm_[i];
        }
        std_msgs::msg::String msg; msg.data = out.str(); status_->publish(msg);
        RCLCPP_INFO(get_logger(), "%s", msg.data.c_str());
    }
    void tick() {
        const auto t = Clock::now();
        const double dt = seconds(t, last_tick_);
        last_tick_ = t;
        if (stopping_) { publish_disabled(); return; }
        if (interrupted) { stop("Ctrl+C / termination requested", false); return; }
        if (dt <= 0 || dt > 0.1) { stop("control loop delayed >100ms", true); return; }
        const double ros_now = now().seconds();
        if (ros_now < previous_ros_) { stop("ROS clock moved backward", true); return; }
        previous_ros_ = ros_now;
        const auto count = command_->get_subscription_count();
        if (const auto why = handshake_->violation(count)) { stop(*why, true); return; }
        if (motor_received_) {
            if (const auto why = motor_input_->publisher_violation()) { stop(*why, true); return; }
        }
        if (power_received_) {
            if (const auto why = power_input_->publisher_violation()) { stop(*why, true); return; }
        }
        if ((motor_received_ && seconds(t, last_feedback_) > state_timeout_) ||
            (!motor_received_ && seconds(t, started_) > required_after_)) {
            stop("missing/stale /motor/state", true); return;
        }
        if (const auto why = power_->violation(ros_now, start_ros_)) { stop(*why, true); return; }
        if (!handshake_->ready_for_output(count)) {
            if (handshake_->mark_rearm_command_about_to_publish(count)) {
                const auto seq = publish_disabled(handshake_->rearm_frame_id());
                if (!handshake_->record_rearm_command_published(seq)) stop("cannot record rearm ACK request", true);
            }
            return;
        }
        if (!power_->ready_for_output(ros_now) || !have_velocity_) {
            publish_disabled();
            if (trajectory_) stop("power/feedback readiness lost during motion", true);
            else if (seconds(t, started_) > 30.0) stop("power/feedback readiness timeout (30s)", true);
            return;
        }
        if (!handshake_->active_output_confirmed()) {
            // Bridge only acknowledges an active payload. Enable selected main
            // drives at ZERO PWM; never enable global servos just for the probe.
            rinbo_msgs::msg::MotorCmdStamped probe;
            rinbo_manual::set_outputs(probe, plan_, disabled_.mask(), {}, cap_, true);
            const auto seq = publish(probe, handshake_->active_probe_frame_id());
            if (!handshake_->mark_active_command_published(count, seq)) stop("cannot record active probe", true);
            return;
        }
        if (!trajectory_) {
            for (size_t i = 0; i < 6; ++i)
                if (plan_.legs[i].mode != rinbo_manual::Mode::Off && std::abs(velocity_[i]) > 5.0) {
                    stop("selected motors must be stationary before starting", true); return;
                }
            trajectory_ = std::make_unique<rinbo_manual::Trajectory>(plan_, position_);
            motion_start_ = t;
            RCLCPP_INFO(get_logger(), "RUN: align %.2fs, total %.2fs including speed ramps and settling", trajectory_->alignment_seconds(), trajectory_->total_seconds());
        }
        const double elapsed = seconds(t, motion_start_);
        const auto target = trajectory_->sample(elapsed);
        if ((!alignment_checked_ && elapsed >= trajectory_->alignment_seconds()) || target.done) {
            for (size_t i = 0; i < 6; ++i) {
                if (plan_.legs[i].mode == rinbo_manual::Mode::Off) continue;
                if (std::abs(target.position[i]-position_[i]) > 2.0 || std::abs(velocity_[i]) > 5.0) {
                    stop(std::string(target.done ? "final pose not reached: " : "alignment not reached: ") +
                         rinbo_manual::names[i], true); return;
                }
            }
            alignment_checked_ = true;
        }
        if (target.done) { stop("plan completed", false); return; }
        std::array<double, 6> pwm{};
        for (size_t i = 0; i < 6; ++i) {
            if (plan_.legs[i].mode == rinbo_manual::Mode::Off || disabled_.contains(i)) continue;
            const double error = (target.position[i] - position_[i]) * rinbo_manual::counts_per_degree;
            if (!std::isfinite(error) || std::abs(error) > rinbo_fsm::kHardMaxPositionErrorCounts) {
                stop(std::string("tracking error >5000 counts: ") + rinbo_manual::names[i], true); return;
            }
            // Existing calibration gains use counts and counts/s, NOT degrees.
            const double raw = 0.35*error + 0.002*(target.velocity[i] - velocity_[i])*rinbo_manual::counts_per_degree
                + 0.02*target.velocity[i]*rinbo_manual::counts_per_degree;
            const double limited = std::clamp(raw, -cap_, cap_);
            pwm[i] = std::clamp(limited, last_pwm_[i]-100.0*dt, last_pwm_[i]+100.0*dt);
        }
        rinbo_msgs::msg::MotorCmdStamped cmd;
        rinbo_manual::set_outputs(cmd, plan_, disabled_.mask(), pwm, cap_, true);
        publish(cmd);
        if (stopping_) return;
        last_pwm_ = pwm;
        report(elapsed);
    }
    rinbo_config::MotionSession& session_;
    rinbo_manual::Plan plan_;
    rinbo_fsm::DisabledLegs disabled_;
    double cap_, state_timeout_, required_after_, start_ros_, previous_ros_;
    bool motor_received_ = false, power_received_ = false, have_velocity_ = false;
    bool stopping_ = false, failed_ = false;
    bool alignment_checked_ = false;
    uint32_t seq_ = 0;
    int64_t last_stamp_ = 0;
    std::string reason_;
    std::array<double, 6> position_{}, velocity_{}, last_pwm_{};
    Clock::time_point started_ = Clock::now(), last_tick_ = started_, last_feedback_ = started_;
    Clock::time_point motion_start_{}, stop_at_{}, last_report_{};
    std::unique_ptr<rinbo_manual::Trajectory> trajectory_;
    std::unique_ptr<rinbo_fsm::RinboPowerGuard> power_;
    std::unique_ptr<rinbo_fsm::RosInputGuard> motor_input_, power_input_;
    std::unique_ptr<rinbo_fsm::MotorArbiterHandshake> handshake_;
    rclcpp::Publisher<rinbo_msgs::msg::MotorCmdStamped>::SharedPtr command_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
    rclcpp::Subscription<rinbo_msgs::msg::MotorStateStamped>::SharedPtr motor_sub_;
    rclcpp::Subscription<rinbo_msgs::msg::PowerStateStamped>::SharedPtr power_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#ifndef RINBO_FSM_OFFLINE_TEST
int main(int argc, char** argv) {
    try {
        std::string path, preview;
        bool execute = false, check_ready = false;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help") {
                std::cout << "rinbo_manual --plan FILE [--preview CSV | --check-ready | --execute]\n"
                    "Default: validate only, without ROS. --preview assumes all initial angles are zero.\n"
                    "--check-ready: also check calibration receipt, without ROS or changing any receipt.\n"
                    "--execute: suspended robot, calibrated motors, existing Bridge/power required.\n";
                return 0;
            }
            if (arg == "--plan" && path.empty() && i+1 < argc) path = argv[++i];
            else if (arg == "--preview" && preview.empty() && i+1 < argc) preview = argv[++i];
            else if (arg == "--execute" && !execute) execute = true;
            else if (arg == "--check-ready" && !check_ready) check_ready = true;
            else throw std::runtime_error("Unknown/repeated argument: " + arg);
        }
        if (path.empty() || (int(execute) + int(check_ready) + int(!preview.empty()) > 1))
            throw std::runtime_error("Use --plan FILE [--preview CSV | --check-ready | --execute]");
        const auto plan = rinbo_manual::load(path);
        const auto config = rinbo_config::RobotConfig::load();
        rinbo_manual::validate_mask(plan, rinbo_fsm::DisabledLegs::from_names(config.disabled_legs()).mask());
        config.print_status(std::cout);
        std::cout << "Valid plan: " << path << "; cruise/hold duration " << plan.duration << "s.\n"
                  << "Safety parameters inherited from site rinbo_standing; PWM cap=min(plan,site).\n";
        if (!execute) {
            if (check_ready) {
                rinbo_config::check_manual_readiness(config);
                std::cout << "CALIBRATION READY: receipt matches current configuration and boot.\n";
            }
            if (!preview.empty()) {
                if (std::filesystem::exists(preview)) throw std::runtime_error("Preview path already exists; choose a new file");
                std::ofstream csv(preview);
                if (!csv) throw std::runtime_error("Cannot create preview CSV");
                rinbo_manual::Trajectory trajectory(plan, {});
                csv << "time_s";
                for (auto name : rinbo_manual::names) csv << ',' << name << "_deg," << name << "_deg_s";
                csv << '\n';
                for (double t = 0; t <= trajectory.total_seconds()+0.02; t += 0.02) {
                    const auto q = trajectory.sample(t);
                    csv << t;
                    for (size_t i = 0; i < 6; ++i) csv << ',' << q.position[i] << ',' << q.velocity[i];
                    csv << '\n';
                }
                csv.close();
                if (!csv) throw std::runtime_error("Cannot finish preview CSV");
                std::cout << "Preview saved: " << preview << " (reference only; assumes zero initial angles).\n";
            }
            std::cout << "CHECK ONLY: no ROS initialized, no motor/power commands sent.\n";
            return 0;
        }
        rinbo_config::MotionSession session(rinbo_config::Stage::Manual);
        rinbo_manual::validate_mask(plan, rinbo_fsm::DisabledLegs::from_names(session.config().disabled_legs()).mask());
        rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);
        auto node = std::make_shared<ManualController>(session, plan);
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        try {
            while (rclcpp::ok() && !node->finished()) {
                executor.spin_some(std::chrono::milliseconds(5));
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } catch (const std::exception& e) {
            node->stop(std::string("unhandled controller error: ") + e.what(), true);
            // Keep the ROS context alive for disabled packets to reach Bridge.
            for (int i = 0; i < 15; ++i) {
                node->stop("exception", true);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            throw;
        }
        const bool failed = node->failed();
        rclcpp::shutdown();
        return failed ? 1 : 0;
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] Manual: " << e.what() << '\n';
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
}
#endif
