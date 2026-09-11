// Test-only transport: production Bridge callbacks, zero NodeHandler instances.
// Never connects to Core, never constructs a gRPC publisher.
#define main unused_production_bridge_main
#include "../src/rinbo_ros_bridge.cpp"
#undef main
int main(int argc,char** argv) {
    if (!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID"))!="232" ||
        !std::getenv("ROS_LOCALHOST_ONLY") || std::string(std::getenv("ROS_LOCALHOST_ONLY"))!="1") return 90;
    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node=std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");bridge_node=node;
    power_operation.epoch=make_bridge_boot_id();
    node->declare_parameter<bool>("feedback",true);
    node->declare_parameter<bool>("ack",true);
    node->declare_parameter<double>("current",0.2);
    node->declare_parameter<bool>("fault",false);
    node->declare_parameter<bool>("replay",false);
    auto sub=node->create_subscription<rinbo_msgs::msg::PowerCmdStamped>("/power/command",10,
      [](rinbo_msgs::msg::PowerCmdStamped::ConstSharedPtr m,const rclcpp::MessageInfo& i){ros_power_cmd_cb(m,i);});
    auto estop=node->create_subscription<std_msgs::msg::Bool>("/estop",10,estop_cb);
    power_operation_status_pub=node->create_publisher<std_msgs::msg::String>("/rinbo/power/operation_status",10);
    ros_power_state_pub=node->create_publisher<rinbo_msgs::msg::PowerStateStamped>("/power/state",10);
    uint32_t seq=0;int mask=0;
    auto timer=node->create_wall_timer(std::chrono::milliseconds(20),[&](){
      if (node->get_parameter("fault").as_bool()) software_estop_asserted.store(true);
      if (node->get_parameter("feedback").as_bool()) {
        ++seq;
        if(node->get_parameter("ack").as_bool()) mask=(ros_power_cmd.digital?1:0)|(ros_power_cmd.signal?2:0)|(ros_power_cmd.power?4:0);
        power_msg::PowerStateStamped msg;msg.set_digital(mask&1);msg.set_signal(mask&2);msg.set_power(mask&4);
        msg.set_v_7(24);msg.set_i_1(node->get_parameter("current").as_double());
        msg.mutable_header()->set_seq(seq);msg.mutable_header()->mutable_stamp()->set_sec(seq);
        if (node->get_parameter("replay").as_bool()) {
            msg.mutable_header()->set_seq(1);msg.mutable_header()->mutable_stamp()->set_sec(1);
        }
        grpc_power_state_cb(msg);
        {std::lock_guard<std::mutex> lock(mutex_grpc_power_cmd);power_operation.motor_time=steady_ns();}
      }
      publish_power_operation_status();
    });
    rclcpp::spin(node);
    timer.reset();sub.reset();power_operation_status_pub.reset();ros_power_state_pub.reset();bridge_node.reset();node.reset();
    if(rclcpp::ok())rclcpp::shutdown();return 0;
}
