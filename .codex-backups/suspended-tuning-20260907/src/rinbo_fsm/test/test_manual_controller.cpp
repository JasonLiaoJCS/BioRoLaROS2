#define RINBO_FSM_OFFLINE_TEST
#include "rinbo_manual.cpp"
#include <gtest/gtest.h>
#include <limits>
#include <unistd.h>

struct ManualOfflineTestAccess {
    static void tick(ManualController& node) { node.tick(); }
    static void expire_feedback(ManualController& node) {
        node.motor_received_ = true;
        node.last_feedback_ = Clock::now()-std::chrono::seconds(1);
    }
    static void expire_startup(ManualController& node) {
        node.started_ = Clock::now()-std::chrono::seconds(3);
    }
    static bool stopped(const ManualController& node) { return node.stopping_; }
    static bool running(const ManualController& node) { return bool(node.trajectory_); }
    static std::array<double,6> ideal_feedback(const ManualController& node) {
        return node.trajectory_ ? node.trajectory_->sample(seconds(Clock::now(),node.motion_start_)).position
                                : std::array<double,6>{};
    }
};

class ManualControllerTest : public ::testing::Test {
protected:
    std::filesystem::path dir, path;
    void SetUp() override {
        ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "231");
        ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
        if (!rclcpp::ok()) rclcpp::init(0,nullptr);
        char pattern[] = "/tmp/rinbo-manual-test-XXXXXX";
        dir = mkdtemp(pattern); path = dir/"robot.yaml";
        std::filesystem::copy_file(std::filesystem::path(RINBO_FSM_SOURCE_DIR)/"test/fixtures/robot_test.yaml", path);
        rinbo_config::MotionSession c(rinbo_config::Stage::Calibration,path);
        c.complete(); // isolated prerequisite fixture; never the live site
    }
    void TearDown() override { std::filesystem::remove_all(dir); rclcpp::shutdown(); }
    rinbo_manual::Plan plan() { return rinbo_manual::parse("duration_s: 3\nlegs: {L2: {mode: position, angle_deg: 5}}\n"); }
};
TEST_F(ManualControllerTest, MissingFeedbackStopsAndInvalidatesCalibration) {
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Manual,path);
        ManualController node(session,plan());
        ManualOfflineTestAccess::expire_startup(node);
        ManualOfflineTestAccess::tick(node);
        EXPECT_TRUE(node.failed());
        EXPECT_TRUE(ManualOfflineTestAccess::stopped(node));
    }
    EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Manual,path),std::exception);
}
TEST_F(ManualControllerTest, StopIsLatchedAndNormalStopPreservesCalibration) {
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Manual,path);
        ManualController node(session,plan());
        node.stop("operator stop",false);
        ManualOfflineTestAccess::tick(node);
        EXPECT_TRUE(ManualOfflineTestAccess::stopped(node));
        EXPECT_FALSE(node.failed());
    }
    EXPECT_NO_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Manual,path));
}

// A ROS-only mock implements the actual Bridge rule: active ACKs require an
// enabled main drive (or servo mode). No gRPC, power tool, or hardware is used.
enum class Fault { None, MotorStale, PowerStale, Estop, Epoch, Heartbeat, Nan, Stuck };
class ManualWireTest : public ManualControllerTest, public ::testing::WithParamInterface<Fault> {};
TEST_P(ManualWireTest, RealRosHandshakeMotionAndFailClosedOutput) {
    rinbo_config::MotionSession session(rinbo_config::Stage::Manual,path);
    auto p = plan(); p.duration = 1.0;
    auto node = std::make_shared<ManualController>(session,p);
    auto bridge = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
    const auto durable = rclcpp::QoS(1).reliable().transient_local();
    auto ready = bridge->create_publisher<std_msgs::msg::Bool>("/rinbo/motor_arbiter_ready",durable);
    auto epoch = bridge->create_publisher<rinbo_msgs::msg::Header>("/rinbo/motor_arbiter_epoch",durable);
    auto rearm = bridge->create_publisher<rinbo_msgs::msg::Header>("/rinbo/motor_rearm_ack",durable);
    auto active = bridge->create_publisher<rinbo_msgs::msg::Header>("/rinbo/motor_active_ack",durable);
    auto heartbeat = bridge->create_publisher<std_msgs::msg::Empty>("/rinbo/motor_arbiter_heartbeat",rclcpp::QoS(1).best_effort());
    auto motors = bridge->create_publisher<rinbo_msgs::msg::MotorStateStamped>("/motor/state",1);
    auto power = bridge->create_publisher<rinbo_msgs::msg::PowerStateStamped>("/power/state",1);
    auto estop = bridge->create_publisher<std_msgs::msg::Bool>("/estop",10);
    rinbo_msgs::msg::MotorCmdStamped last;
    bool saw_probe = false, saw_pwm = false, command_seen = false;
    auto commands = bridge->create_subscription<rinbo_msgs::msg::MotorCmdStamped>("/motor/command",1,
        [&](rinbo_msgs::msg::MotorCmdStamped::SharedPtr cmd) {
            last = *cmd; command_seen = true;
            EXPECT_FALSE(cmd->l1.enable); EXPECT_FALSE(cmd->l3.enable);
            EXPECT_FALSE(cmd->r1.enable); EXPECT_FALSE(cmd->r2.enable); EXPECT_FALSE(cmd->r3.enable);
            EXPECT_EQ(cmd->servo_control_mode,0U); EXPECT_FALSE(cmd->l2.reset_position);
            EXPECT_LE(cmd->l2.voltage,20.0);
            const auto& frame = cmd->header.frame_id;
            auto acknowledgement = cmd->header;
            acknowledgement.frame_id += "|bridge=test-boot|latch=1";
            if (frame.find("/rearm") != std::string::npos) {
                EXPECT_FALSE(cmd->l2.enable);
                rearm->publish(acknowledgement);
            }
            if (cmd->l2.enable && frame.find("/active-probe") != std::string::npos) {
                saw_probe = true; EXPECT_EQ(cmd->l2.voltage,0);
                active->publish(acknowledgement);
            }
            saw_pwm = saw_pwm || cmd->l2.voltage > 0;
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(bridge); executor.add_node(node);
    auto start = Clock::now();
    auto next = start;
    uint32_t seq = 0;
    bool injected = false;
    while (!node->finished() && seconds(Clock::now(),start) < 9.0) {
        if (Clock::now() >= next) {
            next = Clock::now()+std::chrono::milliseconds(10);
            ++seq;
            injected = injected || (saw_pwm && ManualOfflineTestAccess::running(*node));
            const auto fault = injected ? GetParam() : Fault::None;
            if (fault != Fault::Heartbeat) heartbeat->publish(std_msgs::msg::Empty{});
            std_msgs::msg::Bool r; r.data = true; ready->publish(r);
            rinbo_msgs::msg::Header h; h.stamp=bridge->now(); h.seq=1;
            h.frame_id = fault == Fault::Epoch ? "other-boot" : "test-boot"; epoch->publish(h);
            if (fault != Fault::MotorStale) {
                rinbo_msgs::msg::MotorStateStamped m;
                m.header.stamp=bridge->now(); m.header.seq=seq;
                const auto q = fault == Fault::Stuck ? std::array<double,6>{} : ManualOfflineTestAccess::ideal_feedback(*node);
                m.l2.position = -q[1]*rinbo_manual::counts_per_degree;
                if (fault == Fault::Nan) m.l2.position = std::numeric_limits<float>::quiet_NaN();
                motors->publish(m);
            }
            if (fault != Fault::PowerStale) {
                rinbo_msgs::msg::PowerStateStamped s;
                s.header.stamp=bridge->now(); s.header.seq=seq; s.power=true; s.v_7=24;
                power->publish(s);
            }
            if (fault == Fault::Estop) estop->publish(r);
        }
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_TRUE(command_seen); EXPECT_TRUE(saw_probe); EXPECT_TRUE(saw_pwm);
    EXPECT_TRUE(node->finished());
    EXPECT_EQ(node->failed(),GetParam()!=Fault::None);
    EXPECT_FALSE(last.l2.enable); EXPECT_EQ(last.l2.voltage,0);
}
INSTANTIATE_TEST_SUITE_P(IsolatedDomain,ManualWireTest,::testing::Values(
    Fault::None,Fault::MotorStale,Fault::PowerStale,Fault::Estop,
    Fault::Epoch,Fault::Heartbeat,Fault::Nan,Fault::Stuck));
