#define RINBO_FSM_OFFLINE_TEST
#include "rinbo_standing.cpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <limits>
#include <unistd.h>

// Direct production transitions with no executor/bridge and no real motion
// entrypoint. Safety stop messages stay in localhost-only domain 231.
struct StandingOfflineTestAccess {
    static rclcpp::Time time(double seconds) {
        return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
    }
    static void initialize(StandingController& f) {
        f.start_time_ = time(10); f.prev_time_ = time(10); f.first_msg_ = false;
    }
    static void tick(StandingController& f, double now, unsigned missing, bool at_target) {
        std::array<float, 6> positions {};
        std::array<bool, 6> halls {};
        for (int i = 0; i < 6; ++i) {
            halls[i] = (missing & (1U << i)) != 0;
            positions[i] = halls[i] ? 1000.0f :
                (at_target ? (i < 3 ? -f.rotate_180_counts_ : f.rotate_180_counts_) : 0.0f);
            if (f.disabled_legs_.contains(i))
                positions[i] = std::numeric_limits<float>::quiet_NaN();
        }
        const float dt = std::max(0.001, (time(now) - f.prev_time_).seconds());
        f.handle_standing_sample(positions, halls, f.servo_targets_, time(now), dt);
        for (int i = 0; i < 6; ++i) {
            if (f.disabled_legs_.contains(i)) {
                EXPECT_EQ(f.leg_states_[i], LegState::SKIPPED);
                EXPECT_EQ(f.target_positions_[i], 0.0f);
            }
        }
    }
    static bool complete(const StandingController& f) { return f.completion_recorded_; }
    static bool stopped(const StandingController& f) { return f.safety_stopped_; }
    static std::string reason(const StandingController& f) { return f.safety_stop_reason_; }
    static LegState leg(const StandingController& f, int i) { return f.leg_states_[i]; }
    static void publish_stop(StandingController& f) { f.publish_stop_command(); }
    static void fail_publication(StandingController& f) { f.simulate_stop_publication_failure_ = true; }
    static void command_branches(StandingController& f) {
        for (bool stop : {false, true}) {
            rinbo_msgs::msg::MotorCmdStamped cmd;
            for (int i = 0; i < 6; ++i) f.set_leg_cmd(cmd, i, 50.0f);
            f.set_servo_hold_targets(cmd, f.servo_targets_);
            if (stop) f.disable_all_legs(cmd);
            rinbo_fsm::enforce_disabled_leg_commands(cmd, f.disabled_legs_);
            const std::array<const rinbo_msgs::msg::LegCmd*, 6> legs = {
                &cmd.l1, &cmd.l2, &cmd.l3, &cmd.r1, &cmd.r2, &cmd.r3};
            for (int i = 0; i < 6; ++i) {
                if (stop || f.disabled_legs_.contains(i)) {
                    EXPECT_FALSE(legs[i]->enable); EXPECT_FALSE(legs[i]->direction);
                    EXPECT_EQ(legs[i]->voltage, 0.0f); EXPECT_EQ(legs[i]->state, 0U);
                    EXPECT_FALSE(legs[i]->reset_position);
                } else {
                    EXPECT_TRUE(legs[i]->enable); EXPECT_EQ(legs[i]->voltage, 50.0f);
                }
            }
        }
    }
};
namespace {
using Access = StandingOfflineTestAccess;
class StandingMultiLegTest : public ::testing::Test {
protected:
    std::filesystem::path dir, path;
    static void SetUpTestSuite() { if (!rclcpp::ok()) { int argc = 0; rclcpp::init(argc, nullptr); } }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
    void SetUp() override {
        ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
        ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "231");
        if (!rclcpp::ok()) { int argc = 0; rclcpp::init(argc, nullptr); }
        char pattern[] = "/tmp/rinbo-standing-test-XXXXXX";
        dir = mkdtemp(pattern); path = dir / "robot.yaml";
        std::filesystem::copy_file(std::filesystem::path(RINBO_FSM_SOURCE_DIR) /
                                  "test/fixtures/robot_test.yaml", path);
    }
    void TearDown() override { std::filesystem::remove_all(dir); }
    void mask(unsigned bits) {
        std::vector<std::string> names;
        for (int i = 0; i < 6; ++i)
            if (bits & (1U << i)) names.emplace_back(rinbo_fsm::DisabledLegs::kLegNames[i]);
        rinbo_config::update_disabled_legs(path, names.empty() ? "enable-all" : "set", names);
        if (bits != 63) {
            rinbo_config::MotionSession calibration(rinbo_config::Stage::Calibration, path);
            calibration.complete(); // prerequisite fixture, never a mechanical claim
        }
    }
};
TEST_F(StandingMultiLegTest, All64MasksRequireActualHealthyStandingTransitions) {
    for (unsigned bits = 0; bits < 64; ++bits) {
        SCOPED_TRACE(bits); mask(bits);
        if (bits == 63) {
            EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Standing, path), std::exception);
            continue;
        }
        {
            rinbo_config::MotionSession session(rinbo_config::Stage::Standing, path);
            StandingController f(session); Access::initialize(f);
            Access::command_branches(f);
            Access::tick(f, 10.0, bits, false); EXPECT_FALSE(Access::complete(f));
            Access::tick(f, 15.1, bits, true); EXPECT_FALSE(Access::complete(f));
            Access::tick(f, 15.2, bits, true);
            EXPECT_TRUE(Access::complete(f)); EXPECT_FALSE(Access::stopped(f));
        }
        EXPECT_NO_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Tripod, path));
    }
}
TEST_F(StandingMultiLegTest, ReenabledL3IsRequiredAndItsHallTimeoutInvalidatesReceipts) {
    mask(1);
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Standing, path);
        StandingController f(session); Access::initialize(f);
        EXPECT_EQ(Access::leg(f, 2), LegState::FIND_HALL);
        Access::tick(f, 10.0, 5, false);
        Access::tick(f, 15.1, 5, true);
        Access::tick(f, 23.0, 5, true);
        EXPECT_FALSE(Access::complete(f)); EXPECT_TRUE(Access::stopped(f));
        EXPECT_NE(Access::reason(f).find("hall search timeout: L3"), std::string::npos);
    }
    EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Tripod, path), std::exception);
}
TEST_F(StandingMultiLegTest, ShutdownPublicationFailureStillInvalidatesCompletedReceipt) {
    mask(5);
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Standing, path);
        StandingController f(session); Access::initialize(f);
        Access::tick(f, 10.0, 5, false);
        Access::tick(f, 15.1, 5, true);
        Access::tick(f, 15.2, 5, true);
        ASSERT_TRUE(Access::complete(f));
        rclcpp::shutdown();
        Access::fail_publication(f); // force the throwing path; some RMWs silently discard after shutdown
        EXPECT_ANY_THROW(Access::publish_stop(f));
        EXPECT_NO_THROW(f.fail_closed("test: ROS context already stopped"));
        EXPECT_TRUE(Access::stopped(f));
    }
    EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Tripod, path), std::exception);
}
} // namespace
