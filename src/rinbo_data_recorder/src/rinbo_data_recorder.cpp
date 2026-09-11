#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>
#include <deque>
#include <map>
#include <sys/file.h>
#include <fcntl.h>
#include "recorder_support.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "rinbo_msgs/msg/controller_debug_stamped.hpp"
#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include "rinbo_msgs/msg/motor_state_stamped.hpp"
#include "rinbo_msgs/msg/power_cmd_stamped.hpp"
#include "rinbo_msgs/msg/power_state_stamped.hpp"
#include "rinbo_msgs/msg/safety_event_stamped.hpp"

class RinboDataRecorder : public rclcpp::Node {
public:
    RinboDataRecorder() : Node("rinbo_data_recorder") {
        load_parameters();
        const auto lock_path = expand_user(this->declare_parameter<std::string>("lock_file",
            home_dir()+"/.local/state/rinbo-recorder/recorder-"+
            std::string(std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : "0")+".lock"));
        std::filesystem::create_directories(std::filesystem::path(lock_path).parent_path());
        lock_fd_ = ::open(lock_path.c_str(), O_CREAT|O_RDWR|O_CLOEXEC, 0600);
        if(lock_fd_<0 || flock(lock_fd_, LOCK_EX|LOCK_NB)) throw std::runtime_error("recorder_already_owned");
        control_srv_ = this->create_service<rcl_interfaces::srv::SetParametersAtomically>(
            "/rinbo/recorder/control", std::bind(&RinboDataRecorder::control_cb, this,
                std::placeholders::_1, std::placeholders::_2));

        trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "trigger", 10, std::bind(&RinboDataRecorder::trigger_cb, this, std::placeholders::_1));
        filename_sub_ = this->create_subscription<std_msgs::msg::String>(
            "output_filename", 10, std::bind(&RinboDataRecorder::filename_cb, this, std::placeholders::_1));

        recording_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "set_recording",
            std::bind(&RinboDataRecorder::set_recording_cb, this, std::placeholders::_1, std::placeholders::_2));

        motor_state_sub_ = this->create_subscription<rinbo_msgs::msg::MotorStateStamped>(
            "/motor/state", 50, std::bind(&RinboDataRecorder::motor_state_cb, this, std::placeholders::_1));
        if(command_source_ == "legacy") {
            motor_cmd_sub_ = this->create_subscription<rinbo_msgs::msg::MotorCmdStamped>(
                "/motor/command", 50, std::bind(&RinboDataRecorder::motor_cmd_cb, this, std::placeholders::_1));
        } else {
            const auto mirror_qos = rclcpp::QoS(100).best_effort();
            motor_cmd_sub_ = this->create_subscription<rinbo_msgs::msg::MotorCmdStamped>(
                "/rinbo/monitor/motor_requested", mirror_qos,
                std::bind(&RinboDataRecorder::motor_cmd_cb, this, std::placeholders::_1));
            forwarded_sub_ = this->create_subscription<rinbo_msgs::msg::MotorCmdStamped>(
                "/rinbo/monitor/motor_forwarded", mirror_qos,
                std::bind(&RinboDataRecorder::forwarded_cb, this, std::placeholders::_1));
        }
        power_state_sub_ = this->create_subscription<rinbo_msgs::msg::PowerStateStamped>(
            "/power/state", 50, std::bind(&RinboDataRecorder::power_state_cb, this, std::placeholders::_1));
        if(command_source_ == "legacy") power_cmd_sub_ = this->create_subscription<rinbo_msgs::msg::PowerCmdStamped>(
            "/power/command", 10, std::bind(&RinboDataRecorder::power_cmd_cb, this, std::placeholders::_1));
        pid_data_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pid/data", 50, std::bind(&RinboDataRecorder::pid_data_cb, this, std::placeholders::_1));
        controller_debug_sub_ = this->create_subscription<rinbo_msgs::msg::ControllerDebugStamped>(
            "/rinbo/controller_debug", 50, std::bind(&RinboDataRecorder::controller_debug_cb, this, std::placeholders::_1));
        safety_event_sub_ = this->create_subscription<rinbo_msgs::msg::SafetyEventStamped>(
            "/rinbo/safety_event", 20, std::bind(&RinboDataRecorder::safety_event_cb, this, std::placeholders::_1));

        safety_detail_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/rinbo/safety_detail", 100, std::bind(&RinboDataRecorder::safety_detail_cb, this, std::placeholders::_1));

        const int period_ms = std::max(1, static_cast<int>(std::round(1000.0 / std::max(summary_hz_, 1.0))));
        summary_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&RinboDataRecorder::summary_timer_cb, this));

        if (auto_start_) {
            start_recording(run_name_);
        }

        RCLCPP_INFO(this->get_logger(), "Rinbo data recorder ready | output_root=%s profile=%s auto_start=%s",
                    output_root_.c_str(), profile_.c_str(), auto_start_ ? "true" : "false");
    }

    ~RinboDataRecorder() override {
        close_files();
        if(lock_fd_>=0) ::close(lock_fd_);
    }

private:
    static constexpr std::array<const char*, 6> kLegNames = {"l1", "l2", "l3", "r1", "r2", "r3"};

    void load_parameters() {
        const std::string default_root = home_dir() + "/rinbo_logs";
        output_root_ = expand_user(this->declare_parameter<std::string>("output_root", default_root));
        output_dir_param_ = expand_user(this->declare_parameter<std::string>("output_dir", ""));
        workspace_root_ = expand_user(this->declare_parameter<std::string>("workspace_root", home_dir() + "/rinbo_ros_ws"));
        run_name_ = sanitize_name(this->declare_parameter<std::string>("run_name", "rinbo_run"));
        command_source_ = this->declare_parameter<std::string>("command_source", "diagnostic");
        if(command_source_!="diagnostic" && command_source_!="legacy") throw std::invalid_argument("command_source must be diagnostic or legacy");
        profile_ = sanitize_name(this->declare_parameter<std::string>("profile", "tripod_safety"));
        auto_start_ = this->declare_parameter<bool>("auto_start", true);
        record_csv_ = this->declare_parameter<bool>("record_csv", true);
        summary_hz_ = this->declare_parameter<double>("summary_hz", 100.0);
        flush_every_n_rows_ = std::max(1, static_cast<int>(this->declare_parameter<int>("csv_flush_every_n_rows", 1)));

        log_time_ = this->declare_parameter<bool>("log.time", true);
        log_motor_state_ = this->declare_parameter<bool>("log.motor_state", true);
        log_motor_command_ = this->declare_parameter<bool>("log.motor_command", true);
        log_power_state_ = this->declare_parameter<bool>("log.power_state", true);
        log_power_command_ = this->declare_parameter<bool>("log.power_command", true);
        log_pid_data_ = this->declare_parameter<bool>("log.pid_data", true);
        log_controller_debug_ = this->declare_parameter<bool>("log.controller_debug", true);
        log_safety_ = this->declare_parameter<bool>("log.safety", true);
        bag_topics_ = this->declare_parameter<std::vector<std::string>>(
            "bag_topics",
            std::vector<std::string>{
                "/motor/state", "/power/state",
                "/rinbo/monitor/motor_requested", "/rinbo/monitor/motor_forwarded",
                "/rinbo/motor_output_enabled", "/rinbo/motor_arbiter_ready",
                "/pid/data", "/rinbo/controller_debug", "/rinbo/safety_event", "/rinbo/safety_detail", "/rosout"});
        if(command_source_=="diagnostic" && std::find(bag_topics_.begin(),bag_topics_.end(),"/motor/command")!=bag_topics_.end())
            throw std::invalid_argument("diagnostic bag_topics must not include /motor/command");
    }

    static std::string home_dir() {
        const char* home = std::getenv("HOME");
        return home ? std::string(home) : std::string("/tmp");
    }

    static std::string expand_user(const std::string& path) {
        if (path.empty()) return path;
        if (path == "~") return home_dir();
        if (path.rfind("~/", 0) == 0) return home_dir() + path.substr(1);
        return path;
    }

    static std::string sanitize_name(const std::string& input) {
        std::string out;
        for (char c : input) {
            if (static_cast<unsigned char>(c)>=128 || std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') {
                out.push_back(c);
            } else if (c == ' ' || c == '/' || c == '.') {
                out.push_back('_');
            }
        }
        return out.empty() ? "rinbo_run" : out;
    }

    static std::string wall_time_string(const char* format) {
        const auto now = std::chrono::system_clock::now();
        const std::time_t tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm {};
        localtime_r(&tt, &tm);
        char buf[128];
        std::strftime(buf, sizeof(buf), format, &tm);
        return std::string(buf);
    }

    static std::string shell_line(const std::string& command) {
        std::array<char, 256> buffer {};
        std::string result;
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) return "";
        while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
            result += buffer.data();
        }
        pclose(pipe);
        while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
            result.pop_back();
        }
        return result;
    }

    static std::string csv_escape(const std::string& value) {
        if (value.find_first_of(",\"\n\r") == std::string::npos) return value;
        std::string out = "\"";
        for (char c : value) {
            if (c == '"') out += "\"\"";
            else out.push_back(c);
        }
        out += "\"";
        return out;
    }

    static std::string num(double value) {
        if (!std::isfinite(value)) return "";
        std::ostringstream out;
        out << std::fixed << std::setprecision(6) << value;
        return out.str();
    }

    static std::string boolean(bool value) {
        return value ? "1" : "0";
    }

    std::filesystem::path choose_run_dir(const std::string& requested_name) const {
        if (!output_dir_param_.empty()) {
            return std::filesystem::path(output_dir_param_);
        }

        const std::string suffix = sanitize_name(requested_name.empty() ? run_name_ : requested_name);
        std::filesystem::path base = std::filesystem::path(output_root_) /
            (wall_time_string("%Y%m%d_%H%M%S") + "_" + suffix + "_" + std::to_string(getpid()));
        std::filesystem::path candidate = base;
        int index = 1;
        while (std::filesystem::exists(candidate)) {
            candidate = std::filesystem::path(base.string() + "_" + std::to_string(index++));
        }
        return candidate;
    }

    void io_failed(const std::string& reason) {
        if(disk_error_.empty()) disk_error_=reason;
        recording_=false;
        RCLCPP_ERROR(get_logger(), "RECORDER DISK ERROR: %s", disk_error_.c_str());
    }
    bool streams_ok() {
        if (!summary_file_ || !events_file_ || !commands_file_ || !power_file_ || !controller_file_) {
            io_failed("write_or_flush_failed: " + active_run_dir_.string()); return false;
        }
        return true;
    }
    void flush_files() {
        for(auto* f:{&summary_file_,&events_file_,&commands_file_,&power_file_,&controller_file_}) if(f->is_open()) f->flush();
        if(summary_file_.is_open() && summary_file_) summary_flushed_rows_=summary_rows_;
        if(!active_run_dir_.empty()) streams_ok();
    }
    void start_recording(const std::string& requested_name) {
        if(recording_) return;
        if(!disk_error_.empty()) return;
        if(!record_csv_) { io_failed("CSV recording is disabled");return; }
        try {
            actual_name_=sanitize_name(requested_name.empty()?run_name_:requested_name);
            active_run_dir_=choose_run_dir(actual_name_);
            if(!output_dir_param_.empty() && (std::filesystem::exists(active_run_dir_/"metadata.yaml") || std::filesystem::exists(active_run_dir_/"summary.csv"))) {
                const auto base=active_run_dir_; unsigned index=1;
                do { active_run_dir_=base.string()+"_"+std::to_string(index++); } while(std::filesystem::exists(active_run_dir_));
            }
            std::filesystem::create_directories(active_run_dir_);
            // No previous CSV is ever appended with a potentially different schema.
            for(const auto* name:{"summary.csv","events.csv","commands.csv","power_samples.csv","controller_samples.csv"})
                if(std::filesystem::exists(active_run_dir_/name)) throw std::runtime_error("run_directory_contains_history");
            summary_file_.open(active_run_dir_/"summary.csv"); events_file_.open(active_run_dir_/"events.csv");
            commands_file_.open(active_run_dir_/"commands.csv"); power_file_.open(active_run_dir_/"power_samples.csv");
            controller_file_.open(active_run_dir_/"controller_samples.csv");
            if(!streams_ok()) { close_files(); return; }
            summary_rows_=summary_flushed_rows_=event_rows_=command_rows_=power_rows_=controller_rows_=0;
            write_summary_header();write_events_header();write_raw_headers();write_metadata(actual_name_);
            if(!disk_error_.empty()) {close_files();return;}
            recording_=true; rows_since_flush_=0;
            for(const auto& item:power_buffer_) if(recorder::monotonic()-item.first<=2.0) {
                write_csv_line(power_file_,item.second);power_file_.flush();
                if(streams_ok()) ++power_rows_; else break;
            }
            flush_files();
            RCLCPP_INFO(get_logger(),"Recording CSV logs to: %s",active_run_dir_.c_str());
        } catch(const std::exception& e) { io_failed(e.what());close_files(); }
    }
    void stop_recording() {
        if(recording_) flush_files();
        recording_=false;close_files();
    }
    void close_files() {
        for(auto* f:{&summary_file_,&events_file_,&commands_file_,&power_file_,&controller_file_}) {
            if(f->is_open()) {f->flush();if(!*f) io_failed("flush_failed");f->close();if(f->fail()) io_failed("close_failed");}
        }
    }

    void write_metadata(const std::string& requested_name) {
        std::ofstream metadata(active_run_dir_ / "metadata.yaml");
        if (!metadata.is_open()) { io_failed("metadata_open_failed");return; }
        metadata << "schema_version: 2\nrecorder_version: " << recorder::version << "\n";
        metadata << "command_source: " << command_source_ << "\n";
        metadata << "requested_topic: " << motor_cmd_sub_->get_topic_name() << "\n";
        metadata << "forwarded_topic: " << (forwarded_sub_ ? forwarded_sub_->get_topic_name() : "unavailable") << "\n";
        metadata << "command_qos: " << (command_source_=="legacy" ? "reliable depth50" : "best_effort depth100") << "\n";
        metadata << "state_qos: reliable depth50\nlegacy_cmd_columns: requested_only_not_applied\n";
        metadata << "forwarded_semantics: bridge_forwarded_not_hardware_ack\npre_record_power_buffer_s: 2\n";

        char host[256] {};
        gethostname(host, sizeof(host) - 1);

        metadata << "run_name: " << recorder::quote(requested_name.empty() ? run_name_ : requested_name) << "\n";
        metadata << "profile: " << recorder::quote(profile_) << "\n";
        metadata << "start_wall_time: " << wall_time_string("%Y-%m-%dT%H:%M:%S%z") << "\n";
        metadata << "output_dir: " << recorder::quote(active_run_dir_.string()) << "\n";
        metadata << "raw_bag_dir: " << recorder::quote((active_run_dir_ / "raw_bag").string()) << "\n";
        metadata << "hostname: " << host << "\n";
        metadata << "workspace_root: " << workspace_root_ << "\n";
        metadata << "executable: " << std::filesystem::read_symlink("/proc/self/exe").string() << "\n";
        metadata << "record_csv: " << (record_csv_ ? "true" : "false") << "\n";
        metadata << "summary_hz: " << summary_hz_ << "\n";
        metadata << "csv_flush_every_n_rows: " << flush_every_n_rows_ << "\n";
        metadata << "bag_topics:\n";
        for (const auto& topic : bag_topics_) {
            metadata << "  - " << topic << "\n";
        }
        metadata.flush(); if(!metadata) io_failed("metadata_write_failed");
    }

    void write_summary_header() {
        std::vector<std::string> cols;
        if (log_time_) {
            append(cols, {"ros_time_s", "wall_time", "row",
                          "age_motor_state_s", "age_motor_cmd_s", "age_power_state_s",
                          "age_power_cmd_s", "age_pid_data_s", "age_controller_debug_s", "age_safety_event_s"});
        }
        if (log_motor_state_) append_motor_state_header(cols);
        if (log_motor_command_) append_motor_cmd_header(cols);
        if (log_power_state_) append_power_state_header(cols);
        if (log_power_command_) append(cols, {"power_cmd_digital", "power_cmd_signal", "power_cmd_power", "power_cmd_clean", "power_cmd_trigger"});
        if (log_pid_data_) append_pid_header(cols);
        if (log_controller_debug_) append_controller_debug_header(cols);
        if (log_safety_) append(cols, {"safety_last_source", "safety_last_severity", "safety_last_reason"});
        append(cols,{"age_requested_s","age_forwarded_s","requested_seq","forwarded_seq"});
        for(const auto* prefix:{"requested_","forwarded_"}) {
            std::vector<std::string> fields;append_motor_cmd_header(fields);
            for(const auto& field:fields) cols.push_back(std::string(prefix)+field);
        }
        write_csv_line(summary_file_, cols);
    }

    void write_events_header() {
        std::vector<std::string> cols = {
            "ros_time_s", "wall_time", "source", "severity", "reason",
            "min_bus_voltage", "max_current", "tau", "ratio", "cycle_count"
        };
        for (const auto* leg : kLegNames) cols.push_back(std::string("pos_error_") + leg);
        append(cols,{"schema_version","event_kind","event_stamp_ns","event_seq","detail_json","controller_state"});
        write_csv_line(events_file_, cols);
    }

    static void append(std::vector<std::string>& dst, std::initializer_list<std::string> values) {
        dst.insert(dst.end(), values.begin(), values.end());
    }

    static void write_csv_line(std::ofstream& file, const std::vector<std::string>& values) {
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i > 0) file << ",";
            file << csv_escape(values[i]);
        }
        file << "\n";
    }

    double age(bool has_msg, const rclcpp::Time& stamp, const rclcpp::Time& now) const {
        return has_msg ? (now - stamp).seconds() : -1.0;
    }

    void append_motor_state_header(std::vector<std::string>& cols) {
        for (const auto* leg : kLegNames) cols.push_back(std::string("motor_pos_raw_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("motor_pos_ctrl_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("motor_ticks_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("motor_hall_") + leg);
        append(cols, {"servo_pos_sl1", "servo_pos_sl2", "servo_pos_sl3", "servo_pos_sr1", "servo_pos_sr2", "servo_pos_sr3", "servo_mode_state"});
    }

    void append_motor_cmd_header(std::vector<std::string>& cols) {
        for (const auto* leg : kLegNames) cols.push_back(std::string("cmd_enable_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("cmd_dir_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("cmd_voltage_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("cmd_state_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("cmd_reset_") + leg);
        append(cols, {"cmd_servo_sl1", "cmd_servo_sl2", "cmd_servo_sl3", "cmd_servo_sr1", "cmd_servo_sr2", "cmd_servo_sr3", "cmd_servo_mode"});
    }

    void append_power_state_header(std::vector<std::string>& cols) {
        append(cols, {"power_state_digital", "power_state_signal", "power_state_power", "power_state_clean"});
        for (int i = 0; i < 8; ++i) cols.push_back("v_" + std::to_string(i));
        for (int i = 0; i < 8; ++i) cols.push_back("i_" + std::to_string(i));
        append(cols, {"min_bus_voltage", "max_abs_current", "estimated_total_power_w"});
    }

    void append_pid_header(std::vector<std::string>& cols) {
        append(cols, {"pid_tau", "pid_ratio"});
        for (const auto* leg : kLegNames) cols.push_back(std::string("pid_target_pos_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("pid_target_vel_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("pid_pwm_") + leg);
    }

    void append_controller_debug_header(std::vector<std::string>& cols) {
        append(cols, {"ctrl_state", "ctrl_tau", "ctrl_ratio", "ctrl_cycle_count", "ctrl_group_b_active", "ctrl_safety_stopped",
                      "ctrl_min_bus_voltage", "ctrl_max_current"});
        for (const auto* leg : kLegNames) cols.push_back(std::string("ctrl_target_pos_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("ctrl_actual_pos_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("ctrl_target_vel_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("ctrl_actual_vel_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("ctrl_pos_error_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("ctrl_raw_pwm_") + leg);
        for (const auto* leg : kLegNames) cols.push_back(std::string("ctrl_limited_pwm_") + leg);
        cols.push_back("ctrl_safety_reason");
    }

    void summary_timer_cb() {
        if (!recording_ || !summary_file_.is_open()) return;

        const auto now = this->now();
        std::vector<std::string> row;
        if (log_time_) {
            append(row, {num(now.seconds()), wall_time_string("%Y-%m-%dT%H:%M:%S%z"), std::to_string(summary_rows_),
                         num(age(has_motor_state_, motor_state_time_, now)),
                         num(age(has_motor_cmd_, motor_cmd_time_, now)),
                         num(age(has_power_state_, power_state_time_, now)),
                         num(age(has_power_cmd_, power_cmd_time_, now)),
                         num(age(has_pid_data_, pid_data_time_, now)),
                         num(age(has_controller_debug_, controller_debug_time_, now)),
                         num(age(has_safety_event_, safety_event_time_, now))});
        }
        if (log_motor_state_) append_motor_state_row(row);
        if (log_motor_command_) append_motor_cmd_row(row);
        if (log_power_state_) append_power_state_row(row);
        if (log_power_command_) append_power_cmd_row(row);
        if (log_pid_data_) append_pid_row(row);
        if (log_controller_debug_) append_controller_debug_row(row);
        if (log_safety_) append_safety_row(row);

        append(row,{topic_age("requested"),topic_age("forwarded"),has_motor_cmd_?std::to_string(motor_cmd_.header.seq):"",has_forwarded_?std::to_string(forwarded_.header.seq):""});
        append_motor_cmd_row(row);
        const auto saved=motor_cmd_;const bool had=has_motor_cmd_;
        motor_cmd_=forwarded_;has_motor_cmd_=has_forwarded_;append_motor_cmd_row(row);motor_cmd_=saved;has_motor_cmd_=had;
        write_csv_line(summary_file_, row);
        if(!streams_ok()) return;
        summary_rows_++;
        rows_since_flush_++;
        if (rows_since_flush_ >= static_cast<std::size_t>(flush_every_n_rows_)) {
            flush_files();
            rows_since_flush_ = 0;
        }
    }

    const rinbo_msgs::msg::LegState& leg_state(int index) const {
        switch (index) {
            case 0: return motor_state_.l1;
            case 1: return motor_state_.l2;
            case 2: return motor_state_.l3;
            case 3: return motor_state_.r1;
            case 4: return motor_state_.r2;
            default: return motor_state_.r3;
        }
    }

    const rinbo_msgs::msg::LegCmd& leg_cmd(int index) const {
        switch (index) {
            case 0: return motor_cmd_.l1;
            case 1: return motor_cmd_.l2;
            case 2: return motor_cmd_.l3;
            case 3: return motor_cmd_.r1;
            case 4: return motor_cmd_.r2;
            default: return motor_cmd_.r3;
        }
    }

    void append_motor_state_row(std::vector<std::string>& row) {
        if (!has_motor_state_) {
            row.insert(row.end(), 31, "");
            return;
        }
        for (int i = 0; i < 6; ++i) row.push_back(num(leg_state(i).position));
        for (int i = 0; i < 6; ++i) row.push_back(num(i < 3 ? leg_state(i).position : -leg_state(i).position));
        for (int i = 0; i < 6; ++i) row.push_back(std::to_string(leg_state(i).tick_count));
        for (int i = 0; i < 6; ++i) row.push_back(boolean(leg_state(i).hall_effect));
        append(row, {std::to_string(motor_state_.sl1.position_encoder), std::to_string(motor_state_.sl2.position_encoder),
                     std::to_string(motor_state_.sl3.position_encoder), std::to_string(motor_state_.sr1.position_encoder),
                     std::to_string(motor_state_.sr2.position_encoder), std::to_string(motor_state_.sr3.position_encoder),
                     std::to_string(motor_state_.servo_control_mode)});
    }

    void append_motor_cmd_row(std::vector<std::string>& row) {
        if (!has_motor_cmd_) {
            row.insert(row.end(), 37, "");
            return;
        }
        for (int i = 0; i < 6; ++i) row.push_back(boolean(leg_cmd(i).enable));
        for (int i = 0; i < 6; ++i) row.push_back(boolean(leg_cmd(i).direction));
        for (int i = 0; i < 6; ++i) row.push_back(num(leg_cmd(i).voltage));
        for (int i = 0; i < 6; ++i) row.push_back(std::to_string(leg_cmd(i).state));
        for (int i = 0; i < 6; ++i) row.push_back(boolean(leg_cmd(i).reset_position));
        append(row, {std::to_string(motor_cmd_.sl1.position_encoder), std::to_string(motor_cmd_.sl2.position_encoder),
                     std::to_string(motor_cmd_.sl3.position_encoder), std::to_string(motor_cmd_.sr1.position_encoder),
                     std::to_string(motor_cmd_.sr2.position_encoder), std::to_string(motor_cmd_.sr3.position_encoder),
                     std::to_string(motor_cmd_.servo_control_mode)});
    }

    std::array<double, 8> power_voltages() const {
        return {power_state_.v_0, power_state_.v_1, power_state_.v_2, power_state_.v_3,
                power_state_.v_4, power_state_.v_5, power_state_.v_6, power_state_.v_7};
    }

    std::array<double, 8> power_currents() const {
        return {power_state_.i_0, power_state_.i_1, power_state_.i_2, power_state_.i_3,
                power_state_.i_4, power_state_.i_5, power_state_.i_6, power_state_.i_7};
    }

    void append_power_state_row(std::vector<std::string>& row) {
        if (!has_power_state_) {
            row.insert(row.end(), 23, "");
            return;
        }
        append(row, {boolean(power_state_.digital), boolean(power_state_.signal),
                     boolean(power_state_.power), boolean(power_state_.clean)});
        const auto voltages = power_voltages();
        const auto currents = power_currents();
        double min_voltage = voltages[0];
        double max_abs_current = std::fabs(currents[0]);
        double total_power = 0.0;
        for (int i = 0; i < 8; ++i) {
            row.push_back(num(voltages[i]));
            min_voltage = std::min(min_voltage, voltages[i]);
            max_abs_current = std::max(max_abs_current, std::fabs(currents[i]));
            total_power += voltages[i] * currents[i];
        }
        for (int i = 0; i < 8; ++i) row.push_back(num(currents[i]));
        append(row, {num(min_voltage), num(max_abs_current), num(total_power)});
    }

    void append_power_cmd_row(std::vector<std::string>& row) {
        if (!has_power_cmd_) {
            row.insert(row.end(), 5, "");
            return;
        }
        append(row, {boolean(power_cmd_.digital), boolean(power_cmd_.signal), boolean(power_cmd_.power),
                     boolean(power_cmd_.clean), boolean(power_cmd_.trigger)});
    }

    void append_pid_row(std::vector<std::string>& row) {
        if (!has_pid_data_ || pid_data_.data.size() < 20) {
            row.insert(row.end(), 20, "");
            return;
        }
        row.push_back(num(pid_data_.data[0]));
        row.push_back(num(pid_data_.data[1]));
        for (int i = 0; i < 6; ++i) row.push_back(num(pid_data_.data[2 + i]));
        for (int i = 0; i < 6; ++i) row.push_back(num(pid_data_.data[8 + i]));
        for (int i = 0; i < 6; ++i) row.push_back(num(pid_data_.data[14 + i]));
    }

    void append_controller_debug_row(std::vector<std::string>& row) {
        if (!has_controller_debug_) {
            row.insert(row.end(), 51, "");
            return;
        }
        append(row, {controller_debug_.controller_state, num(controller_debug_.tau), num(controller_debug_.ratio),
                     std::to_string(controller_debug_.cycle_count), boolean(controller_debug_.group_b_active),
                     boolean(controller_debug_.safety_stopped), num(controller_debug_.min_bus_voltage),
                     num(controller_debug_.max_current)});
        for (float value : controller_debug_.target_position) row.push_back(num(value));
        for (float value : controller_debug_.actual_position) row.push_back(num(value));
        for (float value : controller_debug_.target_velocity) row.push_back(num(value));
        for (float value : controller_debug_.actual_velocity) row.push_back(num(value));
        for (float value : controller_debug_.position_error) row.push_back(num(value));
        for (float value : controller_debug_.raw_pwm) row.push_back(num(value));
        for (float value : controller_debug_.limited_pwm) row.push_back(num(value));
        row.push_back(controller_debug_.safety_reason);
    }

    void append_safety_row(std::vector<std::string>& row) {
        if (!has_safety_event_) {
            row.insert(row.end(), 3, "");
            return;
        }
        append(row, {safety_event_.source, safety_event_.severity, safety_event_.reason});
    }

    void write_safety_event_row(const rinbo_msgs::msg::SafetyEventStamped& msg) {
        if (!recording_ || !events_file_.is_open()) return;
        std::vector<std::string> row = {
            num(this->now().seconds()), wall_time_string("%Y-%m-%dT%H:%M:%S%z"), msg.source,
            msg.severity, msg.reason, num(msg.min_bus_voltage), num(msg.max_current),
            num(msg.tau), num(msg.ratio), std::to_string(msg.cycle_count)
        };
        for (float value : msg.position_error) row.push_back(num(value));
        append(row,{"2","safety_event",std::to_string(stamp_ns(msg.header)),std::to_string(msg.header.seq),"",has_controller_debug_?controller_debug_.controller_state:""});
        write_csv_line(events_file_, row);
        events_file_.flush();
        if(streams_ok()) event_rows_++;
    }

    void set_recording_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (request->data) {
            start_recording(output_filename_.empty() ? run_name_ : output_filename_);
            response->success = recording_;
            response->message = recording_ ? active_run_dir_.string() : "failed to start recording";
        } else {
            stop_recording();
            response->success = disk_error_.empty();
            response->message = disk_error_.empty()?"recording stopped":disk_error_;
        }
    }

    void trigger_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            start_recording(output_filename_.empty() ? run_name_ : output_filename_);
        } else {
            stop_recording();
        }
    }

    void filename_cb(const std_msgs::msg::String::SharedPtr msg) {
        output_filename_ = sanitize_name(msg->data);
    }

    void motor_state_cb(const rinbo_msgs::msg::MotorStateStamped::SharedPtr msg) {
        motor_state_ = *msg;
        seen_["motor"]=recorder::monotonic();
        source_stamp_["motor"]=stamp_ns(msg->header);source_seq_["motor"]=msg->header.seq;
        motor_state_time_ = this->now();
        has_motor_state_ = true;
    }

    void motor_cmd_cb(const rinbo_msgs::msg::MotorCmdStamped::SharedPtr msg) {
        motor_cmd_ = *msg;
        seen_["requested"]=recorder::monotonic();
        source_stamp_["requested"]=stamp_ns(msg->header);source_seq_["requested"]=msg->header.seq;
        motor_cmd_time_ = this->now();
        has_motor_cmd_ = true;
        write_command_sample(*msg,"requested");
    }

    void power_state_cb(const rinbo_msgs::msg::PowerStateStamped::SharedPtr msg) {
        power_state_ = *msg;
        seen_["power"]=recorder::monotonic();
        source_stamp_["power"]=stamp_ns(msg->header);source_seq_["power"]=msg->header.seq;
        power_state_time_ = this->now();
        has_power_state_ = true;
        write_power_sample(*msg);
    }

    void power_cmd_cb(const rinbo_msgs::msg::PowerCmdStamped::SharedPtr msg) {
        power_cmd_ = *msg;
        seen_["power_command"]=recorder::monotonic();
        power_cmd_time_ = this->now();
        has_power_cmd_ = true;
    }

    void pid_data_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        pid_data_ = *msg;
        seen_["pid"]=recorder::monotonic();
        pid_data_time_ = this->now();
        has_pid_data_ = true;
    }

    void controller_debug_cb(const rinbo_msgs::msg::ControllerDebugStamped::SharedPtr msg) {
        controller_debug_ = *msg;
        seen_["controller"]=recorder::monotonic();
        source_stamp_["controller"]=stamp_ns(msg->header);source_seq_["controller"]=msg->header.seq;
        controller_debug_time_ = this->now();
        has_controller_debug_ = true;
        if(recording_) {
            std::vector<std::string> row={num(this->now().seconds()),std::to_string(stamp_ns(msg->header)),std::to_string(msg->header.seq)};
            append_controller_debug_row(row);write_csv_line(controller_file_,row);controller_file_.flush();
            if(streams_ok()) ++controller_rows_;
        }
    }

    void safety_event_cb(const rinbo_msgs::msg::SafetyEventStamped::SharedPtr msg) {
        safety_event_ = *msg;
        seen_["safety"]=recorder::monotonic();
        source_stamp_["safety"]=stamp_ns(msg->header);source_seq_["safety"]=msg->header.seq;
        safety_event_time_ = this->now();
        has_safety_event_ = true;
        write_safety_event_row(*msg);
    }

    // API and raw diagnostics are serialized by the same ROS executor as CSV writes.
    #include "recorder_methods.inc"
    int lock_fd_=-1;
    std::string command_source_,actual_name_,disk_error_;
    std::map<std::string,double> seen_;
    std::map<std::string,int64_t> source_stamp_;
    std::map<std::string,uint32_t> source_seq_;
    std::ofstream commands_file_,power_file_,controller_file_;
    std::size_t command_rows_=0,power_rows_=0,controller_rows_=0;
    std::deque<std::pair<double,std::vector<std::string>>> power_buffer_;
    rinbo_msgs::msg::MotorCmdStamped forwarded_;
    bool has_forwarded_=false;
    rclcpp::Subscription<rinbo_msgs::msg::MotorCmdStamped>::SharedPtr forwarded_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_detail_sub_;
    rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr control_srv_;
    std::string output_root_;
    std::string output_dir_param_;
    std::string workspace_root_;
    std::string run_name_;
    std::string profile_;
    std::string output_filename_;
    std::vector<std::string> bag_topics_;

    bool auto_start_ = true;
    bool record_csv_ = true;
    double summary_hz_ = 100.0;
    int flush_every_n_rows_ = 1;
    bool log_time_ = true;
    bool log_motor_state_ = true;
    bool log_motor_command_ = true;
    bool log_power_state_ = true;
    bool log_power_command_ = true;
    bool log_pid_data_ = true;
    bool log_controller_debug_ = true;
    bool log_safety_ = true;

    bool recording_ = false;
    std::filesystem::path active_run_dir_;
    std::ofstream summary_file_;
    std::ofstream events_file_;
    std::size_t summary_rows_ = 0;
    std::size_t summary_flushed_rows_ = 0;
    std::size_t event_rows_ = 0;
    std::size_t rows_since_flush_ = 0;

    rinbo_msgs::msg::MotorStateStamped motor_state_;
    rinbo_msgs::msg::MotorCmdStamped motor_cmd_;
    rinbo_msgs::msg::PowerStateStamped power_state_;
    rinbo_msgs::msg::PowerCmdStamped power_cmd_;
    std_msgs::msg::Float32MultiArray pid_data_;
    rinbo_msgs::msg::ControllerDebugStamped controller_debug_;
    rinbo_msgs::msg::SafetyEventStamped safety_event_;

    bool has_motor_state_ = false;
    bool has_motor_cmd_ = false;
    bool has_power_state_ = false;
    bool has_power_cmd_ = false;
    bool has_pid_data_ = false;
    bool has_controller_debug_ = false;
    bool has_safety_event_ = false;
    rclcpp::Time motor_state_time_;
    rclcpp::Time motor_cmd_time_;
    rclcpp::Time power_state_time_;
    rclcpp::Time power_cmd_time_;
    rclcpp::Time pid_data_time_;
    rclcpp::Time controller_debug_time_;
    rclcpp::Time safety_event_time_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr filename_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr recording_srv_;
    rclcpp::Subscription<rinbo_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<rinbo_msgs::msg::MotorCmdStamped>::SharedPtr motor_cmd_sub_;
    rclcpp::Subscription<rinbo_msgs::msg::PowerStateStamped>::SharedPtr power_state_sub_;
    rclcpp::Subscription<rinbo_msgs::msg::PowerCmdStamped>::SharedPtr power_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pid_data_sub_;
    rclcpp::Subscription<rinbo_msgs::msg::ControllerDebugStamped>::SharedPtr controller_debug_sub_;
    rclcpp::Subscription<rinbo_msgs::msg::SafetyEventStamped>::SharedPtr safety_event_sub_;
    rclcpp::TimerBase::SharedPtr summary_timer_;
};

int main(int argc, char** argv) {
    if(argc==2 && std::string(argv[1])=="--version") {std::printf("%s schema=2 control_api=1 observer=diagnostic-mirrors\n",recorder::version);return 0;}
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RinboDataRecorder>());
    rclcpp::shutdown();
    return 0;
}
