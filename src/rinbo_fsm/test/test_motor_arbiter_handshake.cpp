#include "motor_arbiter_handshake.hpp"
#include "rinbo_power_guard.hpp"
#include "ros_input_guard.hpp"
#include "safety_invariants.hpp"
#include "rinbo_msgs/msg/motor_state_stamped.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

using rinbo_fsm::MotorArbiterHandshakeState;
using rinbo_fsm::MotorArbiterHeartbeatState;
using rinbo_fsm::MotorArbiterEndpointPinState;
using rinbo_fsm::MotorArbiterEpochState;
using rinbo_fsm::MotorArbiterPublisherGid;
using rinbo_fsm::MotorArbiterPublisherSnapshot;

MotorArbiterPublisherGid publisher_gid(uint8_t marker) {
    MotorArbiterPublisherGid gid {};
    gid[0] = marker;
    return gid;
}

MotorArbiterPublisherSnapshot publisher_snapshot(
    std::size_t count,
    const std::string& node_name = std::string(),
    const MotorArbiterPublisherGid& gid = MotorArbiterPublisherGid {},
    const std::string& node_namespace = "/") {
    MotorArbiterPublisherSnapshot snapshot;
    snapshot.endpoint_count = count;
    snapshot.node_name = node_name;
    snapshot.node_namespace = node_namespace;
    snapshot.endpoint_gid = gid;
    return snapshot;
}

TEST(MotorArbiterPublisherIdentityTest, SoleExpectedPublisherAndCallbackGidIsTrusted) {
    const auto gid = publisher_gid(1U);
    std::string issue = "stale issue";

    EXPECT_TRUE(rinbo_fsm::validate_motor_arbiter_publisher_identity(
        "/rinbo/motor_arbiter_ready", "rinbo_ros2_bridge", gid,
        publisher_snapshot(1U, "rinbo_ros2_bridge", gid), issue));
    EXPECT_TRUE(issue.empty());
}

TEST(MotorArbiterPublisherIdentityTest, NoPublisherIsRejected) {
    std::string issue;
    EXPECT_FALSE(rinbo_fsm::validate_motor_arbiter_publisher_identity(
        "/rinbo/motor_arbiter_ready", "rinbo_ros2_bridge", publisher_gid(1U),
        publisher_snapshot(0U), issue));
    EXPECT_NE(issue.find("got 0"), std::string::npos);
}

TEST(MotorArbiterPublisherIdentityTest, MultiplePublishersAreRejected) {
    const auto gid = publisher_gid(1U);
    std::string issue;
    EXPECT_FALSE(rinbo_fsm::validate_motor_arbiter_publisher_identity(
        "/rinbo/motor_arbiter_ready", "rinbo_ros2_bridge", gid,
        publisher_snapshot(2U, "rinbo_ros2_bridge", gid), issue));
    EXPECT_NE(issue.find("got 2"), std::string::npos);
}

TEST(MotorArbiterPublisherIdentityTest, WrongNodeIsRejectedEvenWhenGidMatches) {
    const auto gid = publisher_gid(1U);
    std::string issue;
    EXPECT_FALSE(rinbo_fsm::validate_motor_arbiter_publisher_identity(
        "/rinbo/motor_arbiter_ready", "rinbo_ros2_bridge", gid,
        publisher_snapshot(1U, "not_the_bridge", gid, "/unexpected"), issue));
    EXPECT_NE(issue.find("/unexpected/not_the_bridge"), std::string::npos);
    EXPECT_NE(issue.find("GID matches sole graph endpoint=yes"), std::string::npos);
}

TEST(MotorArbiterPublisherIdentityTest, SameNameInWrongNamespaceIsRejected) {
    const auto gid = publisher_gid(1U);
    std::string issue;
    EXPECT_FALSE(rinbo_fsm::validate_motor_arbiter_publisher_identity(
        "/rinbo/motor_arbiter_ready", "rinbo_ros2_bridge", gid,
        publisher_snapshot(
            1U, "rinbo_ros2_bridge", gid, "/unexpected"),
        issue));
    EXPECT_NE(issue.find("/unexpected/rinbo_ros2_bridge"), std::string::npos);
    EXPECT_NE(issue.find("not node /rinbo_ros2_bridge"), std::string::npos);
}

TEST(MotorArbiterPublisherIdentityTest, WrongCallbackGidIsRejected) {
    std::string issue;
    EXPECT_FALSE(rinbo_fsm::validate_motor_arbiter_publisher_identity(
        "/rinbo/motor_arbiter_ready", "rinbo_ros2_bridge", publisher_gid(1U),
        publisher_snapshot(1U, "rinbo_ros2_bridge", publisher_gid(2U)), issue));
    EXPECT_NE(issue.find("callback source GID does not match"), std::string::npos);
}

TEST(MotorArbiterSubscriptionIdentityTest, RequiresExpectedNodeInRootNamespace) {
    std::string issue;
    EXPECT_TRUE(rinbo_fsm::validate_motor_arbiter_subscription_identity(
        "/motor/command", "rinbo_ros2_bridge", 1U,
        "rinbo_ros2_bridge", "/", issue));
    EXPECT_TRUE(issue.empty());

    EXPECT_FALSE(rinbo_fsm::validate_motor_arbiter_subscription_identity(
        "/motor/command", "rinbo_ros2_bridge", 1U,
        "rinbo_ros2_bridge", "/unexpected", issue));
    EXPECT_NE(issue.find("/unexpected/rinbo_ros2_bridge"), std::string::npos);
    EXPECT_NE(issue.find("not node /rinbo_ros2_bridge"), std::string::npos);
}

TEST(MotorArbiterEndpointPinStateTest, FirstAndSameEndpointAreAccepted) {
    MotorArbiterEndpointPinState pin;
    const auto gid = publisher_gid(1U);

    EXPECT_TRUE(pin.observe(gid, false));
    EXPECT_TRUE(pin.observe(gid, false));
    EXPECT_TRUE(pin.observe(gid, true));
}

TEST(MotorArbiterEndpointPinStateTest, UncommittedEndpointChangeRebasesPin) {
    MotorArbiterEndpointPinState pin;

    EXPECT_TRUE(pin.observe(publisher_gid(1U), false));
    EXPECT_TRUE(pin.observe(publisher_gid(2U), false));
    EXPECT_TRUE(pin.observe(publisher_gid(2U), true));
}

TEST(MotorArbiterEndpointPinStateTest, CommittedStatusEndpointChangeIsRejected) {
    MotorArbiterEndpointPinState ready_pin;

    EXPECT_TRUE(ready_pin.observe(publisher_gid(1U), false));
    EXPECT_FALSE(ready_pin.observe(publisher_gid(2U), true));
    EXPECT_TRUE(ready_pin.observe(publisher_gid(1U), true));
}

TEST(MotorArbiterEndpointPinStateTest, CommandSubscriptionGidChangeIsRejected) {
    MotorArbiterEndpointPinState command_pin;

    EXPECT_TRUE(command_pin.observe(publisher_gid(3U), false));
    EXPECT_FALSE(command_pin.observe(publisher_gid(4U), true));
    EXPECT_TRUE(command_pin.observe(publisher_gid(3U), true));
}

TEST(MotorArbiterHeartbeatStateTest, GraphQueryDelayCannotExtendCallbackEntryFreshness) {
    using namespace std::chrono_literals;
    MotorArbiterHeartbeatState heartbeat;
    const MotorArbiterHeartbeatState::TimePoint callback_entry {};
    const auto graph_query_finished = callback_entry + 300ms;

    EXPECT_FALSE(heartbeat.fresh(callback_entry, 0.25));
    heartbeat.observe(callback_entry);
    EXPECT_TRUE(heartbeat.fresh(callback_entry + 250ms, 0.25));
    EXPECT_FALSE(heartbeat.fresh(graph_query_finished, 0.25));
}

TEST(MotorArbiterEpochStateTest, SameEpochIsStable) {
    MotorArbiterEpochState epoch;
    EXPECT_FALSE(epoch.observe("bridge-boot-a", 7U));
    EXPECT_TRUE(epoch.received());
    EXPECT_FALSE(epoch.changed());
    EXPECT_FALSE(epoch.observe("bridge-boot-a", 7U));
    EXPECT_FALSE(epoch.changed());
}

TEST(MotorArbiterEpochStateTest, LatchGenerationChangeIsSticky) {
    MotorArbiterEpochState epoch;
    ASSERT_FALSE(epoch.observe("bridge-boot-a", 7U));
    EXPECT_TRUE(epoch.observe("bridge-boot-a", 8U));
    EXPECT_TRUE(epoch.changed());
    EXPECT_FALSE(epoch.observe("bridge-boot-a", 8U));
    EXPECT_TRUE(epoch.changed());
}

TEST(MotorArbiterEpochStateTest, BridgeRestartIsDetected) {
    MotorArbiterEpochState epoch;
    ASSERT_FALSE(epoch.observe("bridge-boot-a", 7U));
    EXPECT_TRUE(epoch.observe("bridge-boot-b", 1U));
    EXPECT_TRUE(epoch.changed());
}

TEST(MotorArbiterEpochStateTest, UncommittedRearmCanAdoptNewEpochBaseline) {
    MotorArbiterEpochState epoch;
    ASSERT_FALSE(epoch.observe("bridge-boot-a", 7U));
    EXPECT_FALSE(epoch.observe("bridge-boot-a", 8U, false));
    EXPECT_FALSE(epoch.changed());
    EXPECT_EQ(epoch.latch_generation(), 8U);

    EXPECT_TRUE(epoch.observe("bridge-boot-a", 9U, true));
    EXPECT_TRUE(epoch.changed());
}

void arm_with_correlated_rearm(
    MotorArbiterHandshakeState& state, uint32_t sequence = 10U) {
    state.observe_ready_status(false);
    ASSERT_TRUE(state.start_rearm_request());
    ASSERT_TRUE(state.note_rearm_command(sequence));
    ASSERT_TRUE(state.observe_rearm_ack(sequence));
    EXPECT_FALSE(state.ready_for_output(1U));
    state.observe_ready_status(true);
    ASSERT_TRUE(state.ready_for_output(1U));
}

TEST(MotorArbiterHandshakeStateTest, HistoricalReadyCannotReplaceRearmAck) {
    MotorArbiterHandshakeState state;
    state.observe_ready_status(true);

    ASSERT_TRUE(state.start_rearm_request());
    ASSERT_TRUE(state.note_rearm_command(42U));
    state.observe_ready_status(true);
    EXPECT_FALSE(state.ready_for_output(1U));

    EXPECT_FALSE(state.observe_rearm_ack(41U));
    state.observe_ready_status(true);
    EXPECT_FALSE(state.ready_for_output(1U));

    EXPECT_TRUE(state.observe_rearm_ack(42U));
    EXPECT_FALSE(state.ready_for_output(1U));
    state.observe_ready_status(true);
    EXPECT_TRUE(state.ready_for_output(1U));
}

TEST(MotorArbiterHandshakeStateTest, RearmRequiresExactlyOneCommandSubscriber) {
    MotorArbiterHandshakeState state;
    state.observe_ready_status(false);

    EXPECT_FALSE(state.can_publish_rearm(0U));
    EXPECT_TRUE(state.can_publish_rearm(1U));
    EXPECT_FALSE(state.can_publish_rearm(2U));
}

TEST(MotorArbiterHandshakeStateTest, AckForUnpublishedSequenceIsIgnored) {
    MotorArbiterHandshakeState state;
    state.observe_ready_status(false);
    ASSERT_TRUE(state.start_rearm_request());
    ASSERT_TRUE(state.note_rearm_command(7U));

    EXPECT_FALSE(state.observe_rearm_ack(8U));
    state.observe_ready_status(true);
    EXPECT_FALSE(state.ready_for_output(1U));
}

TEST(MotorArbiterHandshakeStateTest, LateRearmAckMatchesBeyondOldFivePacketBudget) {
    MotorArbiterHandshakeState state;
    state.observe_ready_status(false);
    ASSERT_TRUE(state.start_rearm_request());
    for (uint32_t sequence = 1U; sequence <= 20U; ++sequence) {
        ASSERT_TRUE(state.note_rearm_command(sequence));
    }

    EXPECT_TRUE(state.observe_rearm_ack(20U));
    EXPECT_FALSE(state.ready_for_output(1U));
    state.observe_ready_status(true);
    EXPECT_TRUE(state.ready_for_output(1U));
}

TEST(MotorArbiterHandshakeStateTest, DelayedReadyAfterAckCompletesRearm) {
    MotorArbiterHandshakeState state;
    state.observe_ready_status(true);
    ASSERT_TRUE(state.start_rearm_request());
    ASSERT_TRUE(state.note_rearm_command(11U));
    ASSERT_TRUE(state.observe_rearm_ack(11U));
    EXPECT_TRUE(state.protocol_committed());
    EXPECT_FALSE(state.armed());

    state.observe_ready_status(false);
    EXPECT_FALSE(state.ready_for_output(1U));
    state.observe_ready_status(true);
    EXPECT_TRUE(state.ready_for_output(1U));
}

TEST(MotorArbiterHandshakeStateTest, EndpointFaultAfterRearmAckIsStickyBeforeArming) {
    MotorArbiterHandshakeState state;
    state.observe_ready_status(false);
    ASSERT_TRUE(state.start_rearm_request());
    ASSERT_TRUE(state.note_rearm_command(12U));
    ASSERT_TRUE(state.observe_rearm_ack(12U));
    ASSERT_TRUE(state.protocol_committed());
    ASSERT_FALSE(state.armed());

    state.force_fault();
    state.observe_ready_status(true);
    EXPECT_FALSE(state.ready_for_output(1U));
    EXPECT_TRUE(state.immediate_violation(1U).has_value());
}

TEST(MotorArbiterHandshakeStateTest, RelatchAfterArmingIsTerminal) {
    MotorArbiterHandshakeState state;
    arm_with_correlated_rearm(state);

    state.observe_ready_status(false);
    EXPECT_FALSE(state.ready_for_output(1U));
    EXPECT_TRUE(state.immediate_violation(1U).has_value());

    state.observe_ready_status(true);
    EXPECT_FALSE(state.ready_for_output(1U));
}

TEST(MotorArbiterHandshakeStateTest, DuplicateSubscriberAfterArmingIsViolation) {
    MotorArbiterHandshakeState state;
    arm_with_correlated_rearm(state);

    EXPECT_TRUE(state.immediate_violation(0U).has_value());
    EXPECT_TRUE(state.immediate_violation(2U).has_value());
}

TEST(MotorArbiterHandshakeStateTest, ActiveAckMustMatchPublishedSequence) {
    MotorArbiterHandshakeState state;
    arm_with_correlated_rearm(state);

    ASSERT_TRUE(state.note_active_command(100U));
    EXPECT_FALSE(state.observe_active_ack(99U));
    EXPECT_FALSE(state.active_output_confirmed());
    EXPECT_TRUE(state.observe_active_ack(100U));
    EXPECT_TRUE(state.active_output_confirmed());
}

TEST(MotorArbiterHandshakeStateTest, AnyRecordedProbeCanBeAcknowledged) {
    MotorArbiterHandshakeState state;
    arm_with_correlated_rearm(state);

    ASSERT_TRUE(state.note_active_command(100U));
    ASSERT_TRUE(state.note_active_command(101U));
    EXPECT_TRUE(state.observe_active_ack(101U));
    EXPECT_TRUE(state.active_output_confirmed());
}

TEST(MotorArbiterHandshakeStateTest, LateActiveAckMatchesBeyondOldFivePacketBudget) {
    MotorArbiterHandshakeState state;
    arm_with_correlated_rearm(state);
    for (uint32_t sequence = 100U; sequence <= 119U; ++sequence) {
        ASSERT_TRUE(state.note_active_command(sequence));
    }

    EXPECT_TRUE(state.observe_active_ack(119U));
    EXPECT_TRUE(state.active_output_confirmed());
}

class RinboPowerGuardTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) rclcpp::shutdown();
    }

    static rinbo_msgs::msg::PowerStateStamped power_sample(
        bool relay_on, double bus_voltage) {
        rinbo_msgs::msg::PowerStateStamped msg;
        msg.power = relay_on;
        msg.v_7 = bus_voltage;
        return msg;
    }
};

TEST_F(RinboPowerGuardTest, FreshInitialRelayOffRemainsSafeWait) {
    auto node = std::make_shared<rclcpp::Node>("power_guard_initial_wait_test");
    rinbo_fsm::RinboPowerGuard guard(*node);

    for (int index = 0; index < 20; ++index) {
        const double now = 3.0 + static_cast<double>(index) * 0.01;
        guard.update(power_sample(false, 0.0), now);
        EXPECT_FALSE(guard.violation(now, 0.0).has_value());
        EXPECT_FALSE(guard.ready_for_output(now));
    }
}

TEST_F(RinboPowerGuardTest, RelayDropAfterSafePowerIsViolation) {
    auto node = std::make_shared<rclcpp::Node>("power_guard_relay_drop_test");
    rinbo_fsm::RinboPowerGuard guard(*node);

    guard.update(power_sample(true, 24.0), 1.0);
    ASSERT_TRUE(guard.ready_for_output(1.0));
    EXPECT_FALSE(guard.violation(1.0, 0.0).has_value());

    guard.update(power_sample(false, 0.0), 1.1);
    const auto reason = guard.violation(1.1, 0.0);
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find("relay is off"), std::string::npos);
}

TEST_F(RinboPowerGuardTest, MissingTelemetryStillFaultsAfterDeadline) {
    auto node = std::make_shared<rclcpp::Node>("power_guard_missing_state_test");
    rinbo_fsm::RinboPowerGuard guard(*node);

    const auto reason = guard.violation(3.0, 0.0);
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find("no /power/state"), std::string::npos);
}

TEST_F(RinboPowerGuardTest, InvalidTelemetryFaultsWhileRelayOff) {
    auto node = std::make_shared<rclcpp::Node>("power_guard_invalid_state_test");
    rinbo_fsm::RinboPowerGuard guard(*node);

    guard.update(
        power_sample(false, std::numeric_limits<double>::quiet_NaN()), 1.0);
    const auto reason = guard.violation(1.0, 0.0);
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find("NaN/Inf"), std::string::npos);
}

TEST_F(RinboPowerGuardTest, NonFiniteStaleTimeoutIsRejected) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter(
        "safety.power_stale_seconds",
        std::numeric_limits<double>::quiet_NaN())});
    auto node = std::make_shared<rclcpp::Node>(
        "power_guard_nan_timeout_test", options);
    EXPECT_THROW(rinbo_fsm::RinboPowerGuard guard(*node), std::invalid_argument);
}

TEST_F(RinboPowerGuardTest, BackwardSafetyClockIsFailSafe) {
    auto node = std::make_shared<rclcpp::Node>("power_guard_clock_rollback_test");
    rinbo_fsm::RinboPowerGuard guard(*node);
    guard.update(power_sample(true, 24.0), 10.0);
    ASSERT_TRUE(guard.ready_for_output(10.0));

    guard.update(power_sample(true, 24.0), 9.0);
    EXPECT_FALSE(guard.ready_for_output(9.0));
    const auto reason = guard.violation(9.0, 0.0);
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find("clock moved backward"), std::string::npos);
}

TEST_F(RinboPowerGuardTest, DisabledLegCurrentChannelIsTheOnlyPerLegExemption) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("safety.current_trip_samples", 1)});
    auto node = std::make_shared<rclcpp::Node>(
        "power_guard_disabled_leg_test", options);
    std::array<bool, 6> disabled {};
    disabled[0] = true;  // L1 maps to configured current channel 1.
    rinbo_fsm::RinboPowerGuard guard(*node, disabled);

    auto sample = power_sample(true, 24.0);
    sample.i_1 = std::numeric_limits<double>::quiet_NaN();
    guard.update(sample, 1.0);
    EXPECT_TRUE(guard.ready_for_output(1.0));
    EXPECT_FALSE(guard.violation(1.0, 0.0).has_value());

    sample.i_2 = 6.0;
    guard.update(sample, 1.1);
    EXPECT_FALSE(guard.ready_for_output(1.1));
    const auto reason = guard.violation(1.1, 0.0);
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find("leg current"), std::string::npos);
}

TEST_F(RinboPowerGuardTest, RosInputGuardRejectsZeroAndReplayedHeaders) {
    auto node = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
    auto publisher = node->create_publisher<rinbo_msgs::msg::MotorStateStamped>(
        "/guard_test/motor/state", 1);
    rinbo_fsm::RosInputGuard guard(*node, "/guard_test/motor/state");
    const auto endpoints = node->get_publishers_info_by_topic(
        "/guard_test/motor/state");
    ASSERT_EQ(endpoints.size(), 1U);
    rmw_message_info_t raw_info {};
    const auto& endpoint_gid = endpoints.front().endpoint_gid();
    std::copy(endpoint_gid.begin(), endpoint_gid.end(), raw_info.publisher_gid.data);
    const rclcpp::MessageInfo message_info(raw_info);

    rinbo_msgs::msg::Header header;
    header.seq = 1U;
    const auto zero_reason = guard.accept(header, message_info).violation;
    ASSERT_TRUE(zero_reason.has_value());
    EXPECT_NE(zero_reason->find("positive"), std::string::npos);

    const int64_t now_ns = node->now().nanoseconds();
    header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
    header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
    EXPECT_FALSE(guard.accept(header, message_info).violation.has_value());

    const auto replay_reason = guard.accept(header, message_info).violation;
    ASSERT_TRUE(replay_reason.has_value());
    EXPECT_NE(replay_reason->find("duplicate"), std::string::npos);
    (void)publisher;
}

rinbo_fsm::PowerSafetyContract valid_power_contract() {
    return rinbo_fsm::PowerSafetyContract {
        true,
        true,
        true,
        true,
        true,
        true,
        7,
        {1, 2, 3, 4, 5, 6},
        18.0,
        30.0,
        3.0,
        30.0,
        0.5,
        2.0,
        5,
        10};
}

TEST(SafetyInvariantsTest, CorePowerGuardsCannotBeDisabled) {
    for (int field = 0; field < 6; ++field) {
        auto contract = valid_power_contract();
        switch (field) {
            case 0: contract.enabled = false; break;
            case 1: contract.stop_on_power_stale = false; break;
            case 2: contract.stop_on_voltage_sag = false; break;
            case 3: contract.stop_on_over_voltage = false; break;
            case 4: contract.stop_on_current_limit = false; break;
            case 5: contract.require_power_relay = false; break;
        }
        EXPECT_THROW(
            rinbo_fsm::validate_power_safety_contract(contract, "test"),
            std::invalid_argument);
    }
}

TEST(SafetyInvariantsTest, PowerEnvelopeAndMappingRemainBounded) {
    auto expect_rejected = [](const rinbo_fsm::PowerSafetyContract& contract) {
        EXPECT_THROW(
            rinbo_fsm::validate_power_safety_contract(contract, "test"),
            std::invalid_argument);
    };

    auto contract = valid_power_contract();
    EXPECT_NO_THROW(rinbo_fsm::validate_power_safety_contract(contract, "test"));
    contract.min_bus_voltage = 17.999;
    expect_rejected(contract);
    contract = valid_power_contract();
    contract.max_bus_voltage = 42.001;
    expect_rejected(contract);
    contract = valid_power_contract();
    contract.max_leg_current = 10.001;
    expect_rejected(contract);
    contract = valid_power_contract();
    contract.max_bus_current = 30.001;
    expect_rejected(contract);
    contract = valid_power_contract();
    contract.stale_seconds = 0.501;
    expect_rejected(contract);
    contract = valid_power_contract();
    contract.required_after_seconds = 2.001;
    expect_rejected(contract);
    contract = valid_power_contract();
    contract.voltage_trip_samples = 6;
    expect_rejected(contract);
    contract = valid_power_contract();
    contract.current_trip_samples = 101;
    expect_rejected(contract);
    contract = valid_power_contract();
    contract.bus_voltage_channel = 0;
    expect_rejected(contract);
    contract = valid_power_contract();
    contract.leg_current_channels = {6, 5, 4, 3, 2, 1};
    expect_rejected(contract);
}

TEST(SafetyInvariantsTest, TripodOptionalSlewPreservesFeedbackAndPositionValidation) {
    rinbo_fsm::TripodMotionSafetyContract contract {
        true, true, 5000.0, 250.0, 0.25, 2.0, 10};
    EXPECT_NO_THROW(
        rinbo_fsm::validate_tripod_motion_safety_contract(contract));

    contract.stop_on_position_error = false;
    EXPECT_NO_THROW(
        rinbo_fsm::validate_tripod_motion_safety_contract(contract));
    contract = {true, false, 5000.0, 250.0, 0.25, 2.0, 10};
    EXPECT_NO_THROW(rinbo_fsm::validate_tripod_motion_safety_contract(contract));
    contract = {true, true, 12000.1, 250.0, 0.25, 2.0, 10};
    EXPECT_THROW(
        rinbo_fsm::validate_tripod_motion_safety_contract(contract),
        std::invalid_argument);
    contract = {true, true, 5000.0, 250.1, 0.25, 2.0, 10};
    EXPECT_NO_THROW(rinbo_fsm::validate_tripod_motion_safety_contract(contract));
    for (double rate : {0., -1., 1e100, std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::quiet_NaN()}) {
        contract.pwm_slew_rate_per_second = rate;
        EXPECT_THROW(rinbo_fsm::validate_tripod_motion_safety_contract(contract), std::invalid_argument);
    }
    contract = {true, true, 5000.0, 250.0, 0.251, 2.0, 10};
    EXPECT_THROW(
        rinbo_fsm::validate_tripod_motion_safety_contract(contract),
        std::invalid_argument);
    contract = {true, true, 5000.0, 250.0, 0.25, 2.001, 10};
    EXPECT_THROW(
        rinbo_fsm::validate_tripod_motion_safety_contract(contract),
        std::invalid_argument);
    contract = {true, true, 5000.0, 250.0, 0.25, 2.0, 11};
    EXPECT_THROW(
        rinbo_fsm::validate_tripod_motion_safety_contract(contract),
        std::invalid_argument);
}

TEST_F(RinboPowerGuardTest, ConstructorRejectsDisabledCorePowerFlag) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("safety.power_guard_enabled", false)});
    auto node = std::make_shared<rclcpp::Node>(
        "power_guard_disabled_bypass_test", options);
    EXPECT_THROW(rinbo_fsm::RinboPowerGuard guard(*node), std::invalid_argument);
}

TEST_F(RinboPowerGuardTest, SourceAgeOverridesCanOnlyTighten) {
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter(
            "safety.motor_state_source_max_age_s", 0.1001)});
        auto node = std::make_shared<rclcpp::Node>(
            "motor_source_age_bypass_test", options);
        EXPECT_THROW(
            rinbo_fsm::RosInputGuard guard(*node, "/motor/state"),
            std::invalid_argument);
    }
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter(
            "safety.power_state_source_max_age_s", 0.3501)});
        auto node = std::make_shared<rclcpp::Node>(
            "power_source_age_bypass_test", options);
        EXPECT_THROW(
            rinbo_fsm::RosInputGuard guard(*node, "/power/state"),
            std::invalid_argument);
    }
}

TEST(SafetyInvariantsTest, MotorArbiterIdentityAndTimeoutsAreHardBounded) {
    EXPECT_NO_THROW(rinbo_fsm::validate_motor_arbiter_contract(
        "rinbo_ros2_bridge", 5.0, 0.25, 0.25));
    EXPECT_THROW(
        rinbo_fsm::validate_motor_arbiter_contract(
            "fake_bridge", 5.0, 0.25, 0.25),
        std::invalid_argument);
    EXPECT_THROW(
        rinbo_fsm::validate_motor_arbiter_contract(
            "rinbo_ros2_bridge", 5.001, 0.25, 0.25),
        std::invalid_argument);
    EXPECT_THROW(
        rinbo_fsm::validate_motor_arbiter_contract(
            "rinbo_ros2_bridge", 5.0, 0.251, 0.25),
        std::invalid_argument);
    EXPECT_THROW(
        rinbo_fsm::validate_motor_arbiter_contract(
            "rinbo_ros2_bridge", 5.0, 0.25, 0.251),
        std::invalid_argument);
}

}  // namespace

TEST_F(RinboPowerGuardTest, RunningOutputToleratesTransientButPersistentCurrentStillTrips) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("safety.current_trip_samples", 3)});
    auto node = std::make_shared<rclcpp::Node>("power_transient_test", options);
    rinbo_fsm::RinboPowerGuard guard(*node);
    auto sample = power_sample(true, 36.0);
    guard.update(sample, 1.0);
    ASSERT_TRUE(guard.ready_for_output(1.0));
    sample.i_2 = 6.0;
    guard.update(sample, 1.01);
    EXPECT_TRUE(guard.ready_for_output(1.01));
    guard.update(sample, 1.02);
    EXPECT_TRUE(guard.ready_for_output(1.02));
    sample.i_2 = 0;
    guard.update(sample, 1.03);
    EXPECT_TRUE(guard.ready_for_output(1.03));
    sample.i_2 = 6.0;
    for (int n = 0; n < 3; ++n) guard.update(sample, 1.04 + n * .01);
    EXPECT_FALSE(guard.ready_for_output(1.06));
    EXPECT_TRUE(guard.violation(1.06, 0).has_value());
}
