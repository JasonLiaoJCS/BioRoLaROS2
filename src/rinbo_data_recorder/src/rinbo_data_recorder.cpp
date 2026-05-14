#include <iostream>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <chrono>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "rinbo_msgs/msg/motor_state_stamped.hpp"

class RinboDataRecorder : public rclcpp::Node {
public:
    RinboDataRecorder() : Node("rinbo_data_recorder"), trigger_(false), record_count_(0) {
        trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "trigger", 10, std::bind(&RinboDataRecorder::trigger_cb, this, std::placeholders::_1));
        
        filename_sub_ = this->create_subscription<std_msgs::msg::String>(
            "output_filename", 10, std::bind(&RinboDataRecorder::filename_cb, this, std::placeholders::_1));
        
        pid_data_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pid/data", 10, std::bind(&RinboDataRecorder::pid_data_cb, this, std::placeholders::_1));
        
        motor_state_sub_ = this->create_subscription<rinbo_msgs::msg::MotorStateStamped>(
            "/motor/state", 10, std::bind(&RinboDataRecorder::motor_state_cb, this, std::placeholders::_1));

        // 初始化
        for (int i = 0; i < 6; i++) {
            actual_positions_[i] = 0.0f;
        }

        RCLCPP_INFO(this->get_logger(), "Rinbo 6-Leg Data Recorder Started");
    }

    ~RinboDataRecorder() {
        if (output_file_.is_open()) {
            output_file_.close();
            RCLCPP_INFO(this->get_logger(), "File closed. Total: %ld", record_count_);
        }
    }

private:
    bool file_exists(const std::string &filename) {
        struct stat buffer;
        return (stat(filename.c_str(), &buffer) == 0);
    }

    void trigger_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        bool new_trigger = msg->data;
        
        if (new_trigger && !trigger_) {
            std::string home_dir = getenv("HOME");
            std::string output_path = home_dir + "/rinbo_ros_ws/output_data/";
            system(("mkdir -p " + output_path).c_str());
            
            std::string filename;
            if (output_filename_.empty()) {
                std::time_t now = std::time(nullptr);
                std::tm* t = std::localtime(&now);
                char buf[64];
                std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", t);
                filename = output_path + "tripod_data_" + std::string(buf) + ".csv";
            } else {
                std::string base_path = output_path + output_filename_;
                filename = base_path + ".csv";
                int index = 1;
                while (file_exists(filename)) {
                    filename = base_path + "_" + std::to_string(index) + ".csv";
                    index++;
                }
            }
            
            output_file_.open(filename);
            if (output_file_.is_open()) {
                record_count_ = 0;
                // CSV Header
                output_file_ << "tau,ratio,"
                             << "tgt_pos_L1,tgt_pos_L2,tgt_pos_L3,tgt_pos_R1,tgt_pos_R2,tgt_pos_R3,"
                             << "act_pos_L1,act_pos_L2,act_pos_L3,act_pos_R1,act_pos_R2,act_pos_R3,"
                             << "tgt_vel_L1,tgt_vel_L2,tgt_vel_L3,tgt_vel_R1,tgt_vel_R2,tgt_vel_R3,"
                             << "pwm_L1,pwm_L2,pwm_L3,pwm_R1,pwm_R2,pwm_R3\n";
                output_file_.flush();
                RCLCPP_INFO(this->get_logger(), "Recording to: %s", filename.c_str());
            }
        } else if (!new_trigger && trigger_) {
            if (output_file_.is_open()) {
                output_file_.close();
                RCLCPP_INFO(this->get_logger(), "Stopped. Total: %ld", record_count_);
            }
        }
        trigger_ = new_trigger;
    }

    void filename_cb(const std_msgs::msg::String::SharedPtr msg) {
        output_filename_ = msg->data;
    }

    void motor_state_cb(const rinbo_msgs::msg::MotorStateStamped::SharedPtr msg) {
        // 存儲實際位置（注意 R 側取負號）
        actual_positions_[0] = msg->l1.position;
        actual_positions_[1] = msg->l2.position;
        actual_positions_[2] = msg->l3.position;
        actual_positions_[3] = -msg->r1.position;
        actual_positions_[4] = -msg->r2.position;
        actual_positions_[5] = -msg->r3.position;
    }

    void pid_data_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (!trigger_ || !output_file_.is_open()) return;
        
        // 預期格式: [tau, ratio, tgt_pos x6, tgt_vel x6, pwm x6]
        // 總共 2 + 6 + 6 + 6 = 20 個元素
        if (msg->data.size() < 20) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "PID data size mismatch: %zu (expected 20)", msg->data.size());
            return;
        }
        
        // tau, ratio
        output_file_ << msg->data[0] << ","   // tau
                     << msg->data[1] << ",";  // ratio
        
        // target positions (6)
        for (int i = 0; i < 6; i++) {
            output_file_ << msg->data[2 + i] << ",";
        }
        
        // actual positions (6)
        for (int i = 0; i < 6; i++) {
            output_file_ << actual_positions_[i] << ",";
        }
        
        // target velocities (6)
        for (int i = 0; i < 6; i++) {
            output_file_ << msg->data[8 + i] << ",";
        }
        
        // PWMs (6)
        for (int i = 0; i < 6; i++) {
            output_file_ << msg->data[14 + i];
            if (i < 5) output_file_ << ",";
        }
        
        output_file_ << "\n";
        output_file_.flush();
        record_count_++;
    }

    bool trigger_;
    std::string output_filename_;
    std::ofstream output_file_;
    size_t record_count_;

    // 儲存 motor state
    std::array<float, 6> actual_positions_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr filename_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pid_data_sub_;
    rclcpp::Subscription<rinbo_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RinboDataRecorder>());
    rclcpp::shutdown();
    return 0;
}