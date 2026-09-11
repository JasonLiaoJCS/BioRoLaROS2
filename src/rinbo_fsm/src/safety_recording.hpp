#pragma once
// Observation only: no decisions, command publications or parameter changes.
#include <array>
#include "safety_invariants.hpp"
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rinbo_msgs/msg/power_state_stamped.hpp"
#include "rinbo_msgs/msg/safety_event_stamped.hpp"

namespace rinbo_fsm {
inline std::string safety_json_quote(const std::string& text) {
    std::ostringstream out;out<<'"';
    for(unsigned char c:text) {
        if(c=='"'||c=='\\')out<<'\\'<<c;
        else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
        else out<<c;
    }
    return out.str()+'"';
}
inline std::string safety_json_number(double value) {
    if(!std::isfinite(value))return "null";
    std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();
}
class SafetyRecording {
public:
    SafetyRecording(rclcpp::Node& node, std::array<bool,6> disabled, bool legacy=true)
        : node_(node),disabled_(disabled),legacy_(legacy) {
        detail_=node.create_publisher<std_msgs::msg::String>("/rinbo/safety_detail",100);
        if(legacy_) event_=node.create_publisher<rinbo_msgs::msg::SafetyEventStamped>("/rinbo/safety_event",20);
    }
    void observe(const rinbo_msgs::msg::PowerStateStamped& msg) {power_=msg;received_=true;}
    std::string snapshot(const std::string& reason,const std::string& phase,double tau,double ratio,uint32_t cycle,uint32_t seq) const {
        const auto channels=node_.get_parameter("safety.leg_current_channels").as_integer_array();
        const int bus=node_.get_parameter("safety.power_bus_voltage_channel").as_int();
        const double leg_limit=node_.get_parameter("safety.max_current").as_double();
        const double bus_limit=node_.get_parameter("safety.max_bus_current").as_double();
        const double min_v=node_.get_parameter("safety.min_bus_voltage").as_double();
        const double max_v=node_.get_parameter("safety.max_bus_voltage").as_double();
        const std::array<double,8> currents={power_.i_0,power_.i_1,power_.i_2,power_.i_3,power_.i_4,power_.i_5,power_.i_6,power_.i_7};
        const std::array<double,8> volts={power_.v_0,power_.v_1,power_.v_2,power_.v_3,power_.v_4,power_.v_5,power_.v_6,power_.v_7};
        int peak=-1;double value=-1;
        for(size_t i=0;i<6;++i)if(!disabled_[i]&&std::isfinite(currents[channels[i]])&&std::fabs(currents[channels[i]])>value){peak=channels[i];value=std::fabs(currents[peak]);}
        std::string quantity="unclassified";int channel=-1;double measured=0,threshold=0;
        bool classified=true;
        if(reason.find("hard leg current")!=std::string::npos){quantity="hard_leg_current";channel=peak;measured=value;threshold=kHardMaxLegCurrentA;}
        else if(reason.find("leg current")!=std::string::npos){quantity="leg_current";channel=peak;measured=value;threshold=leg_limit;}
        else if(reason.find("bus current")!=std::string::npos){quantity="bus_current";channel=bus;measured=std::fabs(currents[bus]);threshold=bus_limit;}
        else if(reason.find("undervoltage")!=std::string::npos){quantity="bus_voltage";channel=bus;measured=volts[bus];threshold=min_v;}
        else if(reason.find("overvoltage")!=std::string::npos){quantity="bus_voltage";channel=bus;measured=volts[bus];threshold=max_v;}
        else classified=false;
        std::ostringstream out;
        out<<"{\"schema_version\":1,\"source\":"<<safety_json_quote(node_.get_name())
           <<",\"event_seq\":"<<seq<<",\"event_stamp_ns\":"<<node_.now().nanoseconds()
           <<",\"reason\":"<<safety_json_quote(reason)<<",\"controller_state\":"<<safety_json_quote(phase)
           <<",\"tau\":"<<safety_json_number(tau)<<",\"ratio\":"<<safety_json_number(ratio)<<",\"cycle_count\":"<<cycle
           <<",\"power_received\":"<<(received_?"true":"false")
           <<",\"power_seq\":"<<(received_?std::to_string(power_.header.seq):"null")
           <<",\"power_stamp_ns\":"<<(received_?std::to_string(int64_t(power_.header.stamp.sec)*1000000000LL+power_.header.stamp.nanosec):"null")
           <<",\"quantity\":"<<safety_json_quote(quantity)<<",\"channel\":"<<(classified&&received_?std::to_string(channel):"null")
           <<",\"measured\":"<<(classified&&received_?safety_json_number(measured):"null")
           <<",\"threshold\":"<<(classified?safety_json_number(threshold):"null")
           <<",\"current_trip_samples\":"<<node_.get_parameter("safety.current_trip_samples").as_int()
           <<",\"currents\":[";
        for(int i=0;i<8;++i){if(i)out<<',';out<<(received_?safety_json_number(currents[i]):"null");}
        out<<"],\"voltages\":[";for(int i=0;i<8;++i){if(i)out<<',';out<<(received_?safety_json_number(volts[i]):"null");}
        out<<"],\"disabled_legs\":[";for(int i=0;i<6;++i){if(i)out<<',';out<<(disabled_[i]?"true":"false");}
        return out.str()+"]}";
    }
    void emit(const std::string& reason,const std::string& phase,double tau=0,double ratio=0,uint32_t cycle=0,uint32_t seq=0) noexcept {
        try {
            if(legacy_)seq=sequence_++;
            std_msgs::msg::String detail;detail.data=snapshot(reason,phase,tau,ratio,cycle,seq);detail_->publish(detail);
            if(legacy_) {
                rinbo_msgs::msg::SafetyEventStamped event;
                const auto now=node_.now().nanoseconds();event.header.seq=seq;
                event.header.stamp.sec=now/1000000000LL;event.header.stamp.nanosec=now%1000000000LL;
                event.header.frame_id=node_.get_name();
                event.source=node_.get_name();event.reason=reason;event.severity="ERROR";
                event.tau=tau;event.ratio=ratio;event.cycle_count=cycle;
                event.position_error.fill(std::numeric_limits<float>::quiet_NaN());
                event.min_bus_voltage=event.max_current=std::numeric_limits<float>::quiet_NaN();
                if(received_) {
                    const std::array<double,8> volts={power_.v_0,power_.v_1,power_.v_2,power_.v_3,power_.v_4,power_.v_5,power_.v_6,power_.v_7};
                    const std::array<double,8> amps={power_.i_0,power_.i_1,power_.i_2,power_.i_3,power_.i_4,power_.i_5,power_.i_6,power_.i_7};
                    const auto channels=node_.get_parameter("safety.leg_current_channels").as_integer_array();
                    event.min_bus_voltage=volts[node_.get_parameter("safety.power_bus_voltage_channel").as_int()];
                    event.max_current=0;
                    for(size_t i=0;i<6;++i)if(!disabled_[i])event.max_current=std::max<double>(event.max_current,std::fabs(amps[channels[i]]));
                }
                event_->publish(event);
            }
        }catch(const std::exception& e){RCLCPP_ERROR(node_.get_logger(),"safety recording publication failed: %s",e.what());}
    }
private:
    rclcpp::Node& node_;std::array<bool,6> disabled_;bool legacy_,received_=false;uint32_t sequence_=0;
    rinbo_msgs::msg::PowerStateStamped power_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detail_;
    rclcpp::Publisher<rinbo_msgs::msg::SafetyEventStamped>::SharedPtr event_;
};
}
