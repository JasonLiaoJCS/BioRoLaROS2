// Offline integration publisher: runs the real protection and recording helper,
// never constructs a controller, bridge, motor or power command publisher.
#include "rclcpp/rclcpp.hpp"
#include "rinbo_power_guard.hpp"
#include "safety_recording.hpp"
#include <chrono>
#include <thread>
#include <cstdlib>
int main(int argc,char** argv) {
    if(!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID"))!="232" ||
       !std::getenv("ROS_LOCALHOST_ONLY") || std::string(std::getenv("ROS_LOCALHOST_ONLY"))!="1")return 90;
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("rinbo_standing");
    std::array<bool,6> disabled{false,false,true,false,false,false};
    rinbo_fsm::RinboPowerGuard guard(*node,disabled);
    rinbo_fsm::SafetyRecording recorder(*node,disabled);
    auto pub=node->create_publisher<rinbo_msgs::msg::PowerStateStamped>("/power/state",50);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while(pub->get_subscription_count()==0 && std::chrono::steady_clock::now()<deadline){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(20));}
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    rinbo_msgs::msg::PowerStateStamped sample;sample.digital=sample.signal=sample.power=true;sample.v_7=24;
    sample.i_3=99; // Masked L3 must not become the reported trigger channel.
    for(int i=0;i<30;++i) {
        sample.header.seq=100+i;sample.header.stamp=node->now();sample.i_2=i<5?.2:6.25;
        pub->publish(sample);recorder.observe(sample);guard.update(sample,1+i*.01);
        const auto reason=guard.violation(1+i*.01,0);
        if(i<29 && reason)return 91;
        if(i==29) {
            if(!reason)return 92;
            recorder.emit(*reason,"Standing: ROTATE_180");
        }
        rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    for(int i=0;i<30;++i){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    rclcpp::shutdown();return 0;
}
