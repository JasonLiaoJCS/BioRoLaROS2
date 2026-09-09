#include "disabled_legs.hpp"
#include "latest_state_qos.hpp"
#include "rinbo_power_guard.hpp"
#include "ros_input_guard.hpp"
#include "tripod_power_policy.hpp"

#include "rinbo_msgs/msg/header.hpp"
#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include "rinbo_msgs/msg/motor_state_stamped.hpp"
#include "rinbo_msgs/msg/power_state_stamped.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using namespace std::chrono_literals;

bool wait_until(
    const std::function<bool()>& predicate,
    std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(5ms);
    }
    return predicate();
}

bool spin_until(
    rclcpp::executors::SingleThreadedExecutor& executor,
    const std::function<bool()>& predicate,
    std::chrono::milliseconds timeout) {
    return wait_until([&]() {
        executor.spin_some();
        return predicate();
    }, timeout);
}

void spin_for(
    rclcpp::executors::SingleThreadedExecutor& executor,
    std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(2ms);
    }
}

void set_stamp(rinbo_msgs::msg::Header& header, int64_t stamp_ns) {
    header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
}

rinbo_msgs::msg::PowerStateStamped power_sample(
    uint32_t sequence,
    const rclcpp::Time& stamp,
    bool overcurrent) {
    rinbo_msgs::msg::PowerStateStamped msg;
    msg.header.seq = sequence;
    msg.header.stamp = stamp;
    msg.power = true;
    msg.v_7 = 24.0;
    msg.i_1 = overcurrent ? 4.0 : 0.0;
    return msg;
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot read source contract file: " + path.string());
    }
    return std::string(
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>());
}

std::string without_whitespace(std::string value) {
    value.erase(
        std::remove_if(value.begin(), value.end(), [](unsigned char ch) {
            return std::isspace(ch) != 0;
        }),
        value.end());
    return value;
}

std::size_t occurrence_count(
    const std::string& text,
    const std::string& needle) {
    std::size_t count = 0;
    std::size_t position = 0;
    while ((position = text.find(needle, position)) != std::string::npos) {
        ++count;
        position += needle.size();
    }
    return count;
}

using PublisherGid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

PublisherGid sole_publisher_gid(
    rclcpp::Node& node,
    const std::string& topic) {
    const auto publishers = node.get_publishers_info_by_topic(topic);
    if (publishers.size() != 1U) {
        throw std::runtime_error(
            "expected one publisher while preparing MessageInfo for " + topic);
    }
    return publishers.front().endpoint_gid();
}

rclcpp::MessageInfo message_info_with_gid(const PublisherGid& gid) {
    auto raw_info = rmw_get_zero_initialized_message_info();
    std::copy(gid.begin(), gid.end(), raw_info.publisher_gid.data);
    return rclcpp::MessageInfo(raw_info);
}

void expect_rejection_contains(
    const std::optional<std::string>& reason,
    const std::string& expected) {
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find(expected), std::string::npos);
}

class RosSafetyTest : public ::testing::Test {
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

    void SetUp() override {
        const char* domain = std::getenv("ROS_DOMAIN_ID");
        ASSERT_NE(domain, nullptr);
        EXPECT_STREQ(domain, "231");
    }
};

TEST(LatestStateQosTest, RmwProfileIsReliableVolatileKeepLastOne) {
    const auto profile = rinbo_fsm::latest_state_qos().get_rmw_qos_profile();
    EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    EXPECT_EQ(profile.depth, 1U);
    EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(TripodPowerPolicyTest, MissingAndStaleTelemetryFailClosed) {
    const rinbo_fsm::TripodPowerPolicyConfig config;

    rinbo_fsm::TripodPowerPolicy missing_policy(config);
    EXPECT_FALSE(missing_policy.violation(2000000000LL, 0).has_value());
    const auto missing_reason = missing_policy.violation(2000001000LL, 0);
    ASSERT_TRUE(missing_reason.has_value());
    EXPECT_NE(missing_reason->find("no /power/state"), std::string::npos);
    EXPECT_FALSE(missing_policy.ready_for_output(2000001000LL));

    rinbo_fsm::TripodPowerPolicy stale_policy(config);
    stale_policy.observe(rinbo_fsm::TripodPowerObservation {
        1000000000LL, true, true, 24.0, 0.0, 0.0});
    EXPECT_TRUE(stale_policy.ready_for_output(1000000000LL));
    EXPECT_FALSE(stale_policy.violation(1500000000LL, 0).has_value());
    const auto stale_reason = stale_policy.violation(1500001000LL, 0);
    ASSERT_TRUE(stale_reason.has_value());
    EXPECT_NE(stale_reason->find("stale"), std::string::npos);
    EXPECT_FALSE(stale_policy.ready_for_output(1500001000LL));
}

TEST_F(RosSafetyTest, FortyHertzMotorBacklogResumesAtNewestSampleOnly) {
    auto publisher_node = std::make_shared<rclcpp::Node>(
        "latest_motor_state_publisher_test");
    auto subscriber_node = std::make_shared<rclcpp::Node>(
        "latest_motor_state_subscriber_test");
    const std::string topic = "/rinbo_fsm_safety_test/latest_motor_state";
    auto publisher = publisher_node->create_publisher<
        rinbo_msgs::msg::MotorStateStamped>(
        topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile());

    std::vector<uint32_t> received_sequences;
    auto subscription = subscriber_node->create_subscription<
        rinbo_msgs::msg::MotorStateStamped>(
        topic, rinbo_fsm::latest_state_qos(),
        [&](rinbo_msgs::msg::MotorStateStamped::SharedPtr msg) {
            received_sequences.push_back(msg->header.seq);
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(subscriber_node);

    ASSERT_TRUE(wait_until(
        [&]() { return publisher->get_subscription_count() == 1U; }, 3s));

    // DDS may receive throughout this loop, but the subscription executor is
    // intentionally not spun for about 300 ms.  At 40 Hz this is 12 samples.
    for (uint32_t sequence = 1U; sequence <= 12U; ++sequence) {
        rinbo_msgs::msg::MotorStateStamped msg;
        msg.header.seq = sequence;
        msg.header.stamp = publisher_node->now();
        publisher->publish(msg);
        std::this_thread::sleep_for(25ms);
    }
    std::this_thread::sleep_for(50ms);

    ASSERT_TRUE(spin_until(
        executor, [&]() { return !received_sequences.empty(); }, 2s));
    spin_for(executor, 100ms);
    ASSERT_EQ(received_sequences.size(), 1U);
    EXPECT_EQ(received_sequences.front(), 12U);
    (void)subscription;
}

TEST_F(RosSafetyTest, PowerLatestSafeDropsBacklogThenPersistentFaultTripsAndStales) {
    rclcpp::NodeOptions subscriber_options;
    subscriber_options.parameter_overrides({
        rclcpp::Parameter("safety.current_trip_samples", 3)});
    auto publisher_node = std::make_shared<rclcpp::Node>(
        "latest_power_state_publisher_test");
    auto subscriber_node = std::make_shared<rclcpp::Node>(
        "latest_power_state_subscriber_test", subscriber_options);
    rinbo_fsm::RinboPowerGuard guard(*subscriber_node);
    rinbo_fsm::TripodPowerPolicyConfig tripod_config;
    tripod_config.current_trip_samples = 3;
    rinbo_fsm::TripodPowerPolicy tripod_policy(tripod_config);
    const auto node_start_time = subscriber_node->now();
    const double node_start_s = node_start_time.seconds();
    const int64_t node_start_ns = node_start_time.nanoseconds();
    const std::string topic = "/rinbo_fsm_safety_test/latest_power_state";
    auto publisher = publisher_node->create_publisher<
        rinbo_msgs::msg::PowerStateStamped>(
        topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile());

    std::vector<uint32_t> received_sequences;
    double last_callback_s = node_start_s;
    int64_t last_callback_ns = subscriber_node->now().nanoseconds();
    auto subscription = subscriber_node->create_subscription<
        rinbo_msgs::msg::PowerStateStamped>(
        topic, rinbo_fsm::latest_state_qos(),
        [&](rinbo_msgs::msg::PowerStateStamped::SharedPtr msg) {
            last_callback_ns = subscriber_node->now().nanoseconds();
            last_callback_s = static_cast<double>(last_callback_ns) * 1.0e-9;
            guard.update(*msg, last_callback_s);
            tripod_policy.observe(rinbo_fsm::TripodPowerObservation {
                last_callback_ns,
                msg->power,
                true,
                msg->v_7,
                std::fabs(msg->i_1),
                std::fabs(msg->i_7)});
            received_sequences.push_back(msg->header.seq);
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(subscriber_node);

    ASSERT_TRUE(wait_until(
        [&]() { return publisher->get_subscription_count() == 1U; }, 3s));

    // Eleven unsafe samples followed by a safe latest sample must not replay
    // an unsafe queue into the sample-counting power guard.
    for (uint32_t sequence = 1U; sequence <= 11U; ++sequence) {
        publisher->publish(power_sample(sequence, publisher_node->now(), true));
        std::this_thread::sleep_for(25ms);
    }
    publisher->publish(power_sample(12U, publisher_node->now(), false));
    std::this_thread::sleep_for(25ms);
    std::this_thread::sleep_for(50ms);

    ASSERT_TRUE(spin_until(
        executor, [&]() { return !received_sequences.empty(); }, 2s));
    spin_for(executor, 100ms);
    ASSERT_EQ(received_sequences.size(), 1U);
    EXPECT_EQ(received_sequences.front(), 12U);
    EXPECT_TRUE(guard.ready_for_output(last_callback_s));
    EXPECT_FALSE(guard.violation(last_callback_s, node_start_s).has_value());
    EXPECT_EQ(tripod_policy.leg_current_trip_count(), 0);
    EXPECT_TRUE(tripod_policy.ready_for_output(last_callback_ns));
    EXPECT_FALSE(
        tripod_policy.violation(last_callback_ns, node_start_ns).has_value());

    // Latest-only history does not weaken sample accumulation after spinning
    // resumes: three distinct persistent overcurrent callbacks still trip.
    for (uint32_t sequence = 13U; sequence <= 15U; ++sequence) {
        const std::size_t expected_count = received_sequences.size() + 1U;
        publisher->publish(power_sample(sequence, publisher_node->now(), true));
        ASSERT_TRUE(spin_until(
            executor,
            [&]() { return received_sequences.size() >= expected_count; },
            2s));
        const auto reason = guard.violation(last_callback_s, node_start_s);
        const auto tripod_reason =
            tripod_policy.violation(last_callback_ns, node_start_ns);
        EXPECT_EQ(
            tripod_policy.leg_current_trip_count(),
            static_cast<int>(sequence - 12U));
        if (sequence < 15U) {
            EXPECT_FALSE(reason.has_value());
            EXPECT_FALSE(tripod_reason.has_value());
        } else {
            ASSERT_TRUE(reason.has_value());
            EXPECT_NE(reason->find("leg current"), std::string::npos);
            ASSERT_TRUE(tripod_reason.has_value());
            EXPECT_NE(tripod_reason->find("leg current"), std::string::npos);
        }
        std::this_thread::sleep_for(25ms);
    }

    const auto stale_reason = guard.violation(
        last_callback_s + 0.6, node_start_s);
    ASSERT_TRUE(stale_reason.has_value());
    EXPECT_NE(stale_reason->find("stale"), std::string::npos);
    const auto tripod_stale_reason = tripod_policy.violation(
        last_callback_ns + 600000000LL, node_start_ns);
    ASSERT_TRUE(tripod_stale_reason.has_value());
    EXPECT_NE(tripod_stale_reason->find("stale"), std::string::npos);
    (void)subscription;
}

TEST_F(RosSafetyTest, ActualBridgeMessageInfoMatchesSoleGraphEndpoint) {
    auto publisher_node = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
    auto guard_node = std::make_shared<rclcpp::Node>("guard_actual_message_info_test");
    const std::string topic = "/rinbo_fsm_safety_test/guard_motor_actual_info";
    auto publisher = publisher_node->create_publisher<
        rinbo_msgs::msg::MotorStateStamped>(topic, 1);
    rinbo_fsm::RosInputGuard guard(*guard_node, topic);

    bool callback_received = false;
    std::optional<std::string> callback_reason;
    auto subscription = guard_node->create_subscription<
        rinbo_msgs::msg::MotorStateStamped>(
        topic, rinbo_fsm::latest_state_qos(),
        [&](rinbo_msgs::msg::MotorStateStamped::SharedPtr msg,
            const rclcpp::MessageInfo& message_info) {
            callback_reason = guard.accept(msg->header, message_info).violation;
            callback_received = true;
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(guard_node);
    ASSERT_TRUE(wait_until(
        [&]() { return publisher->get_subscription_count() == 1U; }, 3s));
    ASSERT_TRUE(wait_until([&]() {
        const auto endpoints = guard_node->get_publishers_info_by_topic(topic);
        return endpoints.size() == 1U &&
            endpoints.front().node_name() == "rinbo_ros2_bridge" &&
            endpoints.front().node_namespace() == "/";
    }, 3s));

    rinbo_msgs::msg::MotorStateStamped msg;
    msg.header.seq = 1U;
    msg.header.stamp = publisher_node->now();
    publisher->publish(msg);
    ASSERT_TRUE(spin_until(executor, [&]() { return callback_received; }, 2s));
    EXPECT_FALSE(callback_reason.has_value())
        << (callback_reason ? *callback_reason : std::string());
    (void)subscription;
}

TEST_F(RosSafetyTest, SolePublisherWithWrongNodeNameIsRejected) {
    auto wrong_node = std::make_shared<rclcpp::Node>("not_the_bridge");
    auto guard_node = std::make_shared<rclcpp::Node>("guard_wrong_node_test");
    const std::string topic = "/rinbo_fsm_safety_test/guard_motor_wrong_node";
    auto publisher = wrong_node->create_publisher<
        rinbo_msgs::msg::MotorStateStamped>(topic, 1);
    ASSERT_TRUE(wait_until([&]() {
        return guard_node->get_publishers_info_by_topic(topic).size() == 1U;
    }, 2s));
    const auto message_info = message_info_with_gid(
        sole_publisher_gid(*guard_node, topic));
    rinbo_fsm::RosInputGuard guard(*guard_node, topic);
    rinbo_msgs::msg::Header header;
    header.seq = 1U;
    header.stamp = guard_node->now();

    const auto reason = guard.accept(header, message_info).violation;
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find("expected /rinbo_ros2_bridge"), std::string::npos);
    (void)publisher;
}

TEST_F(RosSafetyTest, SoleBridgeNameOutsideRootNamespaceIsRejected) {
    auto wrong_namespace_node = std::make_shared<rclcpp::Node>(
        "rinbo_ros2_bridge", "/unexpected");
    auto guard_node = std::make_shared<rclcpp::Node>("guard_wrong_namespace_test");
    const std::string topic = "/rinbo_fsm_safety_test/guard_motor_wrong_namespace";
    auto publisher = wrong_namespace_node->create_publisher<
        rinbo_msgs::msg::MotorStateStamped>(topic, 1);
    ASSERT_TRUE(wait_until([&]() {
        return guard_node->get_publishers_info_by_topic(topic).size() == 1U;
    }, 2s));
    const auto message_info = message_info_with_gid(
        sole_publisher_gid(*guard_node, topic));
    rinbo_fsm::RosInputGuard guard(*guard_node, topic);
    rinbo_msgs::msg::Header header;
    header.seq = 1U;
    header.stamp = guard_node->now();

    const auto reason = guard.accept(header, message_info).violation;
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find("expected /rinbo_ros2_bridge"), std::string::npos);
    (void)publisher;
}

TEST_F(RosSafetyTest, CallbackGidMustMatchSoleBridgeGraphEndpoint) {
    auto publisher_node = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
    auto guard_node = std::make_shared<rclcpp::Node>("guard_wrong_gid_test");
    const std::string topic = "/rinbo_fsm_safety_test/guard_motor_wrong_gid";
    auto publisher = publisher_node->create_publisher<
        rinbo_msgs::msg::MotorStateStamped>(topic, 1);
    ASSERT_TRUE(wait_until([&]() {
        return guard_node->get_publishers_info_by_topic(topic).size() == 1U;
    }, 2s));
    auto wrong_gid = sole_publisher_gid(*guard_node, topic);
    wrong_gid[0] ^= 0xffU;
    const auto message_info = message_info_with_gid(wrong_gid);
    rinbo_fsm::RosInputGuard guard(*guard_node, topic);
    rinbo_msgs::msg::Header header;
    header.seq = 1U;
    header.stamp = guard_node->now();

    const auto reason = guard.accept(header, message_info).violation;
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find("callback publisher GID"), std::string::npos);
    (void)publisher;
}

TEST_F(RosSafetyTest, SecondPublisherIsRejectedBeforeHeaderAcceptance) {
    auto bridge_node = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
    auto other_node = std::make_shared<rclcpp::Node>("unexpected_state_source");
    auto guard_node = std::make_shared<rclcpp::Node>("guard_two_publishers_test");
    const std::string topic = "/rinbo_fsm_safety_test/guard_motor_two_publishers";
    auto bridge_publisher = bridge_node->create_publisher<
        rinbo_msgs::msg::MotorStateStamped>(topic, 1);
    auto other_publisher = other_node->create_publisher<
        rinbo_msgs::msg::MotorStateStamped>(topic, 1);
    ASSERT_TRUE(wait_until([&]() {
        return guard_node->get_publishers_info_by_topic(topic).size() == 2U;
    }, 3s));
    const auto message_info = message_info_with_gid(PublisherGid {});
    rinbo_fsm::RosInputGuard guard(*guard_node, topic);
    rinbo_msgs::msg::Header header;
    header.seq = 1U;
    header.stamp = guard_node->now();

    const auto reason = guard.accept(header, message_info).violation;
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find("exactly one publisher, got 2"), std::string::npos);
    (void)bridge_publisher;
    (void)other_publisher;
}

TEST_F(RosSafetyTest, PublisherGidRemainsPinnedForCallbacksAndWatchdog) {
    auto publisher_node = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
    auto guard_node = std::make_shared<rclcpp::Node>("guard_gid_pin_test");
    const std::string topic = "/rinbo_fsm_safety_test/guard_motor_gid_pin";
    auto publisher = publisher_node->create_publisher<
        rinbo_msgs::msg::MotorStateStamped>(topic, 1);
    ASSERT_TRUE(wait_until([&]() {
        return guard_node->get_publishers_info_by_topic(topic).size() == 1U;
    }, 2s));
    const auto first_gid = sole_publisher_gid(*guard_node, topic);
    rinbo_fsm::RosInputGuard guard(*guard_node, topic);
    rinbo_msgs::msg::Header header;
    header.seq = 1U;
    header.stamp = guard_node->now();
    ASSERT_FALSE(guard.accept(
        header, message_info_with_gid(first_gid)).violation.has_value());

    publisher.reset();
    publisher_node.reset();
    ASSERT_TRUE(wait_until([&]() {
        return guard_node->get_publishers_info_by_topic(topic).empty();
    }, 3s));
    publisher_node = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
    publisher = publisher_node->create_publisher<
        rinbo_msgs::msg::MotorStateStamped>(topic, 1);
    ASSERT_TRUE(wait_until([&]() {
        return guard_node->get_publishers_info_by_topic(topic).size() == 1U;
    }, 3s));
    const auto second_gid = sole_publisher_gid(*guard_node, topic);
    ASSERT_NE(first_gid, second_gid);

    const auto watchdog_reason = guard.publisher_violation();
    ASSERT_TRUE(watchdog_reason.has_value());
    EXPECT_NE(watchdog_reason->find("publisher changed"), std::string::npos);

    header.seq = 2U;
    header.stamp = guard_node->now();
    const auto callback_reason = guard.accept(
        header, message_info_with_gid(second_gid)).violation;
    ASSERT_TRUE(callback_reason.has_value());
    EXPECT_NE(callback_reason->find("publisher changed"), std::string::npos);
}

TEST_F(RosSafetyTest, GraphDelayDoesNotTurnFreshAtEntryHeaderStale) {
    auto publisher_node = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
    auto node = std::make_shared<rclcpp::Node>("guard_graph_delay_fresh_test");
    const std::string topic = "/rinbo_fsm_safety_test/guard_motor_delay_fresh";
    auto publisher = publisher_node->create_publisher<
        rinbo_msgs::msg::MotorStateStamped>(topic, 1);
    ASSERT_TRUE(wait_until([&]() {
        return node->get_publishers_info_by_topic(topic).size() == 1U;
    }, 2s));
    const auto message_info = message_info_with_gid(sole_publisher_gid(*node, topic));

    int64_t fake_now_ns = 10000000000LL;
    rinbo_fsm::RosInputGuard::TestHooks hooks;
    hooks.now_ns = [&]() { return fake_now_ns; };
    hooks.before_publisher_query = [&]() {
        std::this_thread::sleep_for(125ms);
        fake_now_ns += 125000000LL;
    };
    rinbo_fsm::RosInputGuard guard(*node, topic, std::move(hooks));
    rinbo_msgs::msg::Header header;
    header.seq = 1U;
    set_stamp(header, 10000000000LL);

    const auto started = std::chrono::steady_clock::now();
    const auto validation = guard.accept(header, message_info);
    const auto elapsed = std::chrono::steady_clock::now() - started;
    EXPECT_GE(
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(),
        100);
    EXPECT_FALSE(validation.violation.has_value())
        << (validation.violation ? *validation.violation : std::string());
    EXPECT_EQ(validation.observed_ns, 10000000000LL);
    EXPECT_EQ(validation.validated_ns, 10125000000LL);
    EXPECT_TRUE(validation.fresh_at_validation(0.25));
    (void)publisher;
}

TEST(RosInputGuardValidationTest, GraphDelayCannotExtendArrivalWatchdogFreshness) {
    const rinbo_fsm::RosInputGuard::Validation delayed {
        10000000000LL, 10300000000LL, std::nullopt};

    EXPECT_FALSE(delayed.violation.has_value());
    EXPECT_FALSE(delayed.fresh_at_validation(0.25));
}

TEST_F(RosSafetyTest, GraphDelayCannotRescueHeaderAlreadyStaleAtEntry) {
    auto publisher_node = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
    auto node = std::make_shared<rclcpp::Node>("guard_graph_delay_stale_test");
    const std::string topic = "/rinbo_fsm_safety_test/guard_motor_delay_stale";
    auto publisher = publisher_node->create_publisher<
        rinbo_msgs::msg::MotorStateStamped>(topic, 1);
    ASSERT_TRUE(wait_until([&]() {
        return node->get_publishers_info_by_topic(topic).size() == 1U;
    }, 2s));
    const auto message_info = message_info_with_gid(sole_publisher_gid(*node, topic));

    int64_t fake_now_ns = 10000000000LL;
    rinbo_fsm::RosInputGuard::TestHooks hooks;
    hooks.now_ns = [&]() { return fake_now_ns; };
    hooks.before_publisher_query = [&]() {
        std::this_thread::sleep_for(125ms);
        fake_now_ns += 200000000LL;
    };
    rinbo_fsm::RosInputGuard guard(*node, topic, std::move(hooks));
    rinbo_msgs::msg::Header header;
    header.seq = 1U;
    set_stamp(header, 10000000000LL - 101000000LL);

    const auto reason = guard.accept(header, message_info).violation;
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find("source stamp age"), std::string::npos);
    (void)publisher;
}

TEST_F(RosSafetyTest, BackwardRosTimeJumpDuringGraphQueryFailsClosed) {
    auto publisher_node = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
    auto node = std::make_shared<rclcpp::Node>("guard_backward_clock_test");
    const std::string topic = "/rinbo_fsm_safety_test/guard_motor_backward_clock";
    auto publisher = publisher_node->create_publisher<
        rinbo_msgs::msg::MotorStateStamped>(topic, 1);
    ASSERT_TRUE(wait_until([&]() {
        return node->get_publishers_info_by_topic(topic).size() == 1U;
    }, 2s));
    const auto message_info = message_info_with_gid(sole_publisher_gid(*node, topic));

    int64_t fake_now_ns = 10000000000LL;
    rinbo_fsm::RosInputGuard::TestHooks hooks;
    hooks.now_ns = [&]() { return fake_now_ns; };
    hooks.before_publisher_query = [&]() { fake_now_ns -= 1; };
    rinbo_fsm::RosInputGuard guard(*node, topic, std::move(hooks));
    rinbo_msgs::msg::Header header;
    header.seq = 1U;
    set_stamp(header, 10000000000LL);

    const auto reason = guard.accept(header, message_info).violation;
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find("clock moved backward"), std::string::npos);
    (void)publisher;
}

TEST_F(RosSafetyTest, InvalidFutureReplaySequenceAndStampRemainRejected) {
    auto publisher_node = std::make_shared<rclcpp::Node>("rinbo_ros2_bridge");
    auto node = std::make_shared<rclcpp::Node>("guard_header_contract_test");
    const std::string topic = "/rinbo_fsm_safety_test/guard_motor_header_contract";
    auto publisher = publisher_node->create_publisher<
        rinbo_msgs::msg::MotorStateStamped>(topic, 1);
    ASSERT_TRUE(wait_until([&]() {
        return node->get_publishers_info_by_topic(topic).size() == 1U;
    }, 2s));
    const auto message_info = message_info_with_gid(sole_publisher_gid(*node, topic));

    constexpr int64_t now_ns = 10000000000LL;
    rinbo_fsm::RosInputGuard::TestHooks hooks;
    hooks.now_ns = [=]() { return now_ns; };
    rinbo_fsm::RosInputGuard guard(*node, topic, std::move(hooks));
    const auto validate = [&](const rinbo_msgs::msg::Header& candidate) {
        return guard.accept(candidate, message_info).violation;
    };
    rinbo_msgs::msg::Header header;
    header.seq = 1U;

    expect_rejection_contains(validate(header), "positive");
    set_stamp(header, now_ns + 101000000LL);
    expect_rejection_contains(validate(header), "source stamp age");
    set_stamp(header, now_ns - 101000000LL);
    expect_rejection_contains(validate(header), "source stamp age");
    header.stamp.sec = 10;
    header.stamp.nanosec = 1000000000U;
    expect_rejection_contains(validate(header), "nanosec");

    header.seq = 10U;
    set_stamp(header, now_ns);
    EXPECT_FALSE(validate(header).has_value());

    header.seq = 10U;
    set_stamp(header, now_ns + 1);
    expect_rejection_contains(validate(header), "sequence");
    header.seq = 9U;
    set_stamp(header, now_ns + 2);
    expect_rejection_contains(validate(header), "sequence");

    header.seq = 11U;
    set_stamp(header, now_ns + 3);
    EXPECT_FALSE(validate(header).has_value());
    header.seq = 12U;
    expect_rejection_contains(validate(header), "stamp");
    set_stamp(header, now_ns + 2);
    expect_rejection_contains(validate(header), "stamp");
    (void)publisher;
}

TEST_F(RosSafetyTest, DisabledLegsAcceptOneCanonicalLegAndRejectUnsafeSets) {
    rclcpp::NodeOptions valid_options;
    valid_options.parameter_overrides({rclcpp::Parameter(
        "hardware.disabled_legs", std::vector<std::string>{" l1 "})});
    auto valid_node = std::make_shared<rclcpp::Node>(
        "disabled_leg_valid_test", valid_options);
    const auto valid = rinbo_fsm::DisabledLegs::load(*valid_node);
    EXPECT_EQ(valid.count(), 1);
    EXPECT_TRUE(valid.contains(0));
    EXPECT_EQ(valid.summary(), "L1");

    rclcpp::NodeOptions unknown_options;
    unknown_options.parameter_overrides({rclcpp::Parameter(
        "hardware.disabled_legs", std::vector<std::string>{"X1"})});
    auto unknown_node = std::make_shared<rclcpp::Node>(
        "disabled_leg_unknown_test", unknown_options);
    EXPECT_THROW(
        rinbo_fsm::DisabledLegs::load(*unknown_node), std::invalid_argument);

    rclcpp::NodeOptions duplicate_options;
    duplicate_options.parameter_overrides({rclcpp::Parameter(
        "hardware.disabled_legs", std::vector<std::string>{"L1", " l1 "})});
    auto duplicate_node = std::make_shared<rclcpp::Node>(
        "disabled_leg_duplicate_test", duplicate_options);
    EXPECT_THROW(
        rinbo_fsm::DisabledLegs::load(*duplicate_node), std::invalid_argument);

    rclcpp::NodeOptions too_many_options;
    too_many_options.parameter_overrides({rclcpp::Parameter(
        "hardware.disabled_legs", std::vector<std::string>{"L1", "R2"})});
    auto too_many_node = std::make_shared<rclcpp::Node>(
        "disabled_leg_too_many_test", too_many_options);
    EXPECT_THROW(
        rinbo_fsm::DisabledLegs::load(*too_many_node), std::invalid_argument);

    rclcpp::NodeOptions loosened_max_options;
    loosened_max_options.parameter_overrides({
        rclcpp::Parameter(
            "hardware.disabled_legs", std::vector<std::string>{"L1"}),
        rclcpp::Parameter("hardware.max_disabled_legs", 2)});
    auto loosened_max_node = std::make_shared<rclcpp::Node>(
        "disabled_leg_loosened_max_test", loosened_max_options);
    EXPECT_THROW(
        rinbo_fsm::DisabledLegs::load(*loosened_max_node),
        std::invalid_argument);
}

TEST_F(RosSafetyTest, PublishBoundaryMaskDisablesEachConfiguredMainDrive) {
    ASSERT_EQ(rinbo_fsm::DisabledLegs::kLegNames.size(), 6U);
    for (std::size_t disabled_index = 0;
         disabled_index < rinbo_fsm::DisabledLegs::kLegNames.size();
         ++disabled_index) {
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter(
            "hardware.disabled_legs",
            std::vector<std::string> {
                rinbo_fsm::DisabledLegs::kLegNames[disabled_index]})});
        auto node = std::make_shared<rclcpp::Node>(
            "disabled_leg_output_mask_test_" + std::to_string(disabled_index),
            options);
        const auto disabled_legs = rinbo_fsm::DisabledLegs::load(*node);

        rinbo_msgs::msg::MotorCmdStamped command;
        const std::array<rinbo_msgs::msg::LegCmd*, 6> legs = {
            &command.l1, &command.l2, &command.l3,
            &command.r1, &command.r2, &command.r3};
        for (auto* leg : legs) {
            leg->enable = true;
            leg->direction = true;
            leg->voltage = 42.0f;
            leg->state = 1U;
            leg->reset_position = true;
        }

        rinbo_fsm::enforce_disabled_leg_commands(command, disabled_legs);

        for (std::size_t index = 0; index < legs.size(); ++index) {
            SCOPED_TRACE(rinbo_fsm::DisabledLegs::kLegNames[disabled_index]);
            if (index == disabled_index) {
                EXPECT_FALSE(legs[index]->enable);
                EXPECT_FALSE(legs[index]->direction);
                EXPECT_FLOAT_EQ(legs[index]->voltage, 0.0f);
                EXPECT_EQ(legs[index]->state, 0U);
                EXPECT_FALSE(legs[index]->reset_position);
            } else {
                EXPECT_TRUE(legs[index]->enable);
                EXPECT_TRUE(legs[index]->direction);
                EXPECT_FLOAT_EQ(legs[index]->voltage, 42.0f);
                EXPECT_EQ(legs[index]->state, 1U);
                EXPECT_TRUE(legs[index]->reset_position);
            }
        }
    }
}

TEST_F(RosSafetyTest, DisabledLegParametersAreStartupOnlyAndRejectRuntimeUpdates) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter(
        "hardware.disabled_legs", std::vector<std::string>{"L1"})});
    auto node = std::make_shared<rclcpp::Node>(
        "disabled_leg_read_only_test", options);
    const auto disabled_legs = rinbo_fsm::DisabledLegs::load(*node);

    const auto mask_descriptor =
        node->describe_parameter("hardware.disabled_legs");
    const auto limit_descriptor =
        node->describe_parameter("hardware.max_disabled_legs");
    EXPECT_TRUE(mask_descriptor.read_only);
    EXPECT_TRUE(limit_descriptor.read_only);

    const auto mask_result = node->set_parameter(rclcpp::Parameter(
        "hardware.disabled_legs", std::vector<std::string>{"R2"}));
    EXPECT_FALSE(mask_result.successful);
    EXPECT_NE(mask_result.reason.find("read-only"), std::string::npos)
        << mask_result.reason;

    const auto limit_result = node->set_parameter(
        rclcpp::Parameter("hardware.max_disabled_legs", 2));
    EXPECT_FALSE(limit_result.successful);
    EXPECT_NE(limit_result.reason.find("read-only"), std::string::npos)
        << limit_result.reason;

    EXPECT_EQ(
        node->get_parameter("hardware.disabled_legs").as_string_array(),
        std::vector<std::string>({"L1"}));
    EXPECT_EQ(node->get_parameter("hardware.max_disabled_legs").as_int(), 1);
    EXPECT_TRUE(disabled_legs.contains(0));
    for (int index = 1; index < 6; ++index) {
        EXPECT_FALSE(disabled_legs.contains(index));
    }
}

TEST(FsmSourceContractTest, AllFsmsShareLatestQosAndDisabledLegSafetyContract) {
    const std::filesystem::path source_root(RINBO_FSM_SOURCE_DIR);
    const std::vector<std::string> files = {
        "rinbo_cali.cpp", "rinbo_standing.cpp", "rinbo_tripod.cpp"};
    for (const auto& file : files) {
        const std::string source = read_file(source_root / "src" / file);
        const std::string compact = without_whitespace(source);
        EXPECT_EQ(occurrence_count(compact, "rinbo_fsm::latest_state_qos()"), 2U)
            << file;
        EXPECT_NE(
            compact.find("\"/motor/state\",rinbo_fsm::latest_state_qos()"),
            std::string::npos) << file;
        EXPECT_NE(
            compact.find("\"/power/state\",rinbo_fsm::latest_state_qos()"),
            std::string::npos) << file;
        EXPECT_EQ(occurrence_count(compact, "std::placeholders::_2"), 2U)
            << file;
        EXPECT_EQ(occurrence_count(compact, "constrclcpp::MessageInfo&"), 2U)
            << file;
        EXPECT_EQ(
            occurrence_count(compact, "accept(msg->header,message_info)"),
            2U) << file;
        EXPECT_NE(
            compact.find("last_motor_state_time_=arrival_time;"),
            std::string::npos) << file;
        EXPECT_NE(compact.find("fresh_at_validation("), std::string::npos)
            << file;
        EXPECT_EQ(
            occurrence_count(
                compact,
                "rinbo_fsm::enforce_disabled_leg_commands(cmd,disabled_legs_)"),
            1U) << file;
        EXPECT_EQ(
            occurrence_count(compact, "rinbo_fsm::DisabledLegs::load(*this)"),
            1U) << file;
        EXPECT_NE(source.find("DEGRADED MODE"), std::string::npos) << file;
    }

    for (const auto& file : {"rinbo_cali.cpp", "rinbo_standing.cpp"}) {
        const std::string compact = without_whitespace(
            read_file(source_root / "src" / file));
        EXPECT_NE(
            compact.find("if(disabled_legs_.contains(leg_idx)){return;}"),
            std::string::npos) << file;
    }
    const std::string tripod = without_whitespace(
        read_file(source_root / "src" / "rinbo_tripod.cpp"));
    EXPECT_NE(
        tripod.find("enforce_disabled_leg_commands(cmd);"),
        std::string::npos);
    EXPECT_EQ(occurrence_count(tripod, "force_leg_disabled(cmd."), 6U);

    const std::string degraded_yaml = read_file(
        source_root / "config" / "l1_degraded_test.yaml");
    EXPECT_NE(degraded_yaml.find("rinbo_cali:"), std::string::npos);
    EXPECT_NE(degraded_yaml.find("rinbo_standing:"), std::string::npos);
    EXPECT_NE(degraded_yaml.find("rinbo_tripod_rslip:"), std::string::npos);
    EXPECT_EQ(
        occurrence_count(degraded_yaml, "disabled_legs: [\"L1\"]"), 3U);
    EXPECT_EQ(
        occurrence_count(degraded_yaml, "max_disabled_legs: 1"), 3U);
}

}  // namespace
