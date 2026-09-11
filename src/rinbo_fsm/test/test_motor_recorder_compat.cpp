#include "motor_arbiter_handshake.hpp"
#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include "rinbo_msgs/msg/power_cmd_stamped.hpp"
#include <gtest/gtest.h>
#include <cstdlib>
#include <functional>
#include <thread>

namespace {
using namespace rinbo_fsm;
using namespace std::chrono_literals;
using Command = rinbo_msgs::msg::MotorCmdStamped;
using Header = rinbo_msgs::msg::Header;

MotorCommandEndpoint endpoint(const std::string& name, uint8_t marker = 1) {
    MotorCommandEndpoint result{name, "/", "rinbo_msgs/msg/MotorCmdStamped", {}};
    result.gid[0] = marker;
    return result;
}

TEST(MotorCommandGraph, AllowsOnlyUniqueBridgeAndAuditedPassiveRecorder) {
    const auto bridge = endpoint("rinbo_ros2_bridge");
    const auto recorder = endpoint("rinbo_data_recorder", 2);
    const auto controller = endpoint("rinbo_cali", 3);
    MotorArbiterPublisherGid gid;
    std::string issue;
    auto valid = [&](std::vector<MotorCommandEndpoint> subscribers,
                     std::vector<MotorCommandEndpoint> publishers,
                     std::vector<MotorCommandEndpoint> power = {}) {
        return validate_motor_command_graph(subscribers, publishers, power,
            "rinbo_ros2_bridge", "rinbo_cali", gid, issue);
    };
    ASSERT_TRUE(valid({bridge, recorder}, {controller}));
    EXPECT_EQ(gid, bridge.gid);
    EXPECT_TRUE(valid({recorder, bridge}, {controller}));
    EXPECT_EQ(gid, bridge.gid);  // recorder ordering/GID never replaces Bridge pin
    EXPECT_TRUE(valid({bridge}, {controller}));
    EXPECT_FALSE(valid({recorder}, {controller}));
    EXPECT_NE(issue.find("got 0"), std::string::npos);
    EXPECT_FALSE(valid({bridge, endpoint("rinbo_ros2_bridge", 4), recorder}, {controller}));
    EXPECT_NE(issue.find("got 2"), std::string::npos);
    EXPECT_FALSE(valid({bridge, endpoint("unknown")}, {controller}));
    for (bool wrong_bridge : {false, true}) {
        auto bad = wrong_bridge ? bridge : recorder;
        bad.node_namespace = "/unexpected";
        EXPECT_FALSE(valid(wrong_bridge ? std::vector<MotorCommandEndpoint>{bad, recorder}
                                       : std::vector<MotorCommandEndpoint>{bridge, bad}, {controller}));
        bad.node_namespace = "/";
        bad.topic_type = "std_msgs/msg/Bool";
        EXPECT_FALSE(valid(wrong_bridge ? std::vector<MotorCommandEndpoint>{bad, recorder}
                                       : std::vector<MotorCommandEndpoint>{bridge, bad}, {controller}));
    }
    EXPECT_FALSE(valid({bridge, recorder}, {}));
    EXPECT_FALSE(valid({bridge, recorder}, {controller, recorder}));
    EXPECT_FALSE(valid({bridge, recorder}, {recorder}));
    auto bad_controller = controller;
    bad_controller.node_namespace = "/unexpected";
    EXPECT_FALSE(valid({bridge, recorder}, {bad_controller}));
    bad_controller = controller;
    bad_controller.topic_type = "std_msgs/msg/Bool";
    EXPECT_FALSE(valid({bridge, recorder}, {bad_controller}));
    auto power = endpoint("redrhex_rinbo_power_tool");
    power.topic_type = "rinbo_msgs/msg/PowerCmdStamped";
    EXPECT_TRUE(valid({bridge, recorder}, {controller}, {power}));
    EXPECT_FALSE(valid({bridge, recorder}, {controller}, {power, power}));
    power.node_name = "rinbo_data_recorder";
    EXPECT_FALSE(valid({bridge, recorder}, {controller}, {power}));
    power.node_name = "redrhex_rinbo_power_tool";
    power.topic_type = "std_msgs/msg/Bool";
    EXPECT_FALSE(valid({bridge, recorder}, {controller}, {power}));
}

// Fake Bridge only: this suite must run in the localhost-only test domain.
// No hardware-facing executable, sockets or power operations are used.
class RecorderHandshake : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "231");
        ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
        if (!rclcpp::ok()) { int argc = 0; rclcpp::init(argc, nullptr); }
    }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
    std::shared_ptr<rclcpp::Node> bridge, controller, recorder;
    std::unique_ptr<MotorArbiterHandshake> handshake;
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::Publisher<Command>::SharedPtr command;
    rclcpp::Subscription<Command>::SharedPtr bridge_sub, recorder_sub, extra_sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat;
    rclcpp::Publisher<Header>::SharedPtr epoch, rearm_ack, active_ack;
    uint32_t generation = 7;
    uint32_t sequence = 0;
    std::string ack_suffix;
    bool send_ack = true;

    void SetUp() override {
        bridge = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
        recorder = std::make_shared<rclcpp::Node>("rinbo_data_recorder");
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("safety.motor_arbiter_ready_timeout_s", 1.0)});
        controller = std::make_shared<rclcpp::Node>("rinbo_cali", options);
        command = controller->create_publisher<Command>("/motor/command", 10);
        const auto status_qos = rclcpp::QoS(1).reliable().transient_local();
        ready = bridge->create_publisher<std_msgs::msg::Bool>("/rinbo/motor_arbiter_ready", status_qos);
        epoch = bridge->create_publisher<Header>("/rinbo/motor_arbiter_epoch", status_qos);
        heartbeat = bridge->create_publisher<std_msgs::msg::Empty>(
            "/rinbo/motor_arbiter_heartbeat", rclcpp::QoS(1).best_effort());
        rearm_ack = bridge->create_publisher<Header>("/rinbo/motor_rearm_ack", 10);
        active_ack = bridge->create_publisher<Header>("/rinbo/motor_active_ack", 10);
        bridge_sub = bridge->create_subscription<Command>("/motor/command", 10,
            [this](Command::ConstSharedPtr msg) {
                if (!send_ack) return;
                Header ack = msg->header;
                ack.frame_id += ack_suffix.empty()
                    ? "|bridge=test-boot|latch=" + std::to_string(generation) : ack_suffix;
                if (msg->header.frame_id == handshake->rearm_frame_id()) rearm_ack->publish(ack);
                if (msg->header.frame_id == handshake->active_probe_frame_id()) active_ack->publish(ack);
            });
        add_recorder();
        handshake = std::make_unique<MotorArbiterHandshake>(*controller);
        executor.add_node(bridge); executor.add_node(controller); executor.add_node(recorder);
    }
    void TearDown() override {
        executor.remove_node(controller); executor.remove_node(bridge); executor.remove_node(recorder);
        handshake.reset(); extra_sub.reset(); recorder_sub.reset(); bridge_sub.reset();
        command.reset(); ready.reset(); heartbeat.reset(); epoch.reset(); rearm_ack.reset(); active_ack.reset();
        controller.reset(); bridge.reset(); recorder.reset();
    }
    void add_recorder() {
        recorder_sub = recorder->create_subscription<Command>("/motor/command", 10,
            [](Command::ConstSharedPtr) {});
    }
    void tick(bool rearm = false) {
        Header e; e.frame_id = "test-boot"; e.seq = generation; epoch->publish(e);
        std_msgs::msg::Bool r; r.data = true; ready->publish(r);
        heartbeat->publish(std_msgs::msg::Empty{});
        executor.spin_some();
        if (rearm && handshake->mark_rearm_command_about_to_publish()) {
            Command cmd; cmd.header.frame_id = handshake->rearm_frame_id(); cmd.header.seq = ++sequence;
            command->publish(cmd);
            EXPECT_TRUE(handshake->record_rearm_command_published(sequence));
        }
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }
    bool until(const std::function<bool()>& predicate, double seconds = 2.0, bool rearm = false) {
        const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < end) { tick(rearm); if (predicate()) return true; }
        return predicate();
    }
    void arm() { ASSERT_TRUE(until([&] { return handshake->ready_for_output(); }, 2, true)); }
};

TEST_F(RecorderHandshake, RecorderJoinLeaveAndRearmKeepBridgePinned) {
    arm();
    for (int change = 0; change < 4; ++change) {
        if (change % 2 == 0) recorder_sub.reset(); else add_recorder();
        for (int i = 0; i < 15; ++i) { tick(); EXPECT_TRUE(handshake->ready_for_output()); EXPECT_FALSE(handshake->violation()); }
    }
    Command cmd; cmd.header.frame_id = handshake->active_probe_frame_id(); cmd.header.seq = ++sequence;
    command->publish(cmd);
    ASSERT_TRUE(handshake->mark_active_command_published(sequence));
    EXPECT_TRUE(until([&] { return handshake->active_output_confirmed(); }, .2));
}

TEST_F(RecorderHandshake, RecorderAloneCannotRearmAndTimeoutNamesMissingBridge) {
    bridge_sub.reset();
    ASSERT_TRUE(until([&] { return handshake->violation().has_value(); }, 2, true));
    EXPECT_FALSE(handshake->ready_for_output());
    EXPECT_EQ(sequence, 0U);
    EXPECT_NE(handshake->violation()->find("got 0"), std::string::npos);
}

TEST_F(RecorderHandshake, SecondBridgeAfterArmingIsTerminal) {
    arm();
    extra_sub = bridge->create_subscription<Command>("/motor/command", 10, [](Command::ConstSharedPtr) {});
    ASSERT_TRUE(until([&] { return handshake->violation().has_value(); }));
    EXPECT_NE(handshake->violation()->find("got 2"), std::string::npos);
    extra_sub.reset();
    for (int i = 0; i < 20; ++i) tick();
    EXPECT_FALSE(handshake->ready_for_output());
}

TEST_F(RecorderHandshake, RecorderMotorPublisherCannotUseObserverException) {
    arm();
    auto forbidden = recorder->create_publisher<Command>("/motor/command", 10);
    ASSERT_TRUE(until([&] { return handshake->violation().has_value(); }));
    EXPECT_NE(handshake->violation()->find("controller publisher"), std::string::npos);
    EXPECT_FALSE(handshake->ready_for_output());
}

TEST_F(RecorderHandshake, RecorderPowerPublisherCannotUseObserverException) {
    arm();
    auto forbidden = recorder->create_publisher<rinbo_msgs::msg::PowerCmdStamped>("/power/command", 10);
    ASSERT_TRUE(until([&] { return handshake->violation().has_value(); }));
    EXPECT_NE(handshake->violation()->find("recorder publisher"), std::string::npos);
}

TEST_F(RecorderHandshake, OldEpochAckCannotArmAndNewEpochRequiresMatchingAck) {
    ack_suffix = "|bridge=test-boot|latch=6";
    EXPECT_FALSE(until([&] { return handshake->ready_for_output(); }, .3, true));
    generation = 8;  // uncommitted rearm may adopt the new epoch
    EXPECT_FALSE(until([&] { return handshake->ready_for_output(); }, .3, true));
    ack_suffix.clear();
    arm();
    ++generation;  // after commitment, epoch changes remain terminal
    ASSERT_TRUE(until([&] { return handshake->violation().has_value(); }));
    EXPECT_FALSE(handshake->ready_for_output());
}

TEST_F(RecorderHandshake, OldSessionAndUnpublishedSequenceAcksStayRejected) {
    send_ack = false;
    ASSERT_TRUE(until([&] { return sequence > 0; }, .5, true));
    Header ack;
    ack.seq = sequence;
    ack.frame_id = "old-session/rearm|bridge=test-boot|latch=7";
    rearm_ack->publish(ack);
    EXPECT_FALSE(until([&] { return handshake->ready_for_output(); }, .05));
    ack.frame_id = handshake->rearm_frame_id() + "|bridge=test-boot|latch=7";
    ack.seq = sequence + 1000;
    rearm_ack->publish(ack);
    EXPECT_FALSE(until([&] { return handshake->ready_for_output(); }, .05));
    send_ack = true;
    arm();
    send_ack = false;
    Command cmd;
    cmd.header.seq = ++sequence;
    cmd.header.frame_id = handshake->active_probe_frame_id();
    command->publish(cmd);
    ASSERT_TRUE(handshake->mark_active_command_published(sequence));
    ack = cmd.header;
    ack.frame_id += "|bridge=test-boot|latch=6";
    active_ack->publish(ack);
    EXPECT_FALSE(until([&] { return handshake->active_output_confirmed(); }, .04));
    ack.frame_id = cmd.header.frame_id + "|bridge=test-boot|latch=7";
    active_ack->publish(ack);
    EXPECT_TRUE(until([&] { return handshake->active_output_confirmed(); }, .15));
}
}  // namespace
