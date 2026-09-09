#define RINBO_FSM_OFFLINE_TEST
#include "rinbo_tripod.cpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <unistd.h>

// Direct production trajectory/guard transitions with no executor or bridge.
struct TripodOfflineTestAccess {
    static rclcpp::Time time(double seconds) {
        return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
    }
    static void initialize(PIDController& f) {
        f.initialized_ = true; f.prev_time_ = time(10); f.start_time_ = time(10);
        f.initial_positions_.fill(0); f.prev_positions_.fill(0);
    }
    static std::array<float, 6> startup_positions(PIDController& f, bool at_target = true) {
        std::array<float, 6> p;
        for (int i = 0; i < 6; ++i)
            p[i] = f.disabled_legs_.contains(i) ? std::numeric_limits<float>::quiet_NaN() :
                (at_target ? f.theta_home_ * f.rad_to_counts_ : 0.0f);
        return p;
    }
    static void tick(PIDController& f, double now, const std::array<float, 6>& p) {
        auto msg = std::make_shared<rinbo_msgs::msg::MotorStateStamped>();
        f.handle_motion_sample(msg, p, time(now));
        for (int i = 0; i < 6; ++i) {
            if (f.disabled_legs_.contains(i)) {
                EXPECT_EQ(f.prev_command_pwms_[i], 0.0f);
                EXPECT_EQ(f.last_position_errors_[i], 0.0f);
                EXPECT_EQ(f.home_offsets_[i], 0.0f);
            } else {
                EXPECT_TRUE(std::isfinite(f.prev_command_pwms_[i]));
            }
        }
    }
    static bool running(const PIDController& f) { return f.state_ == PIDController::State::RUNNING; }
    static bool stopped(const PIDController& f) { return f.state_ == PIDController::State::SAFETY_STOP; }
    static bool fully_stopped(const PIDController& f) { return f.fully_stopped_; }
    static std::string reason(const PIDController& f) { return f.safety_stop_reason_; }
    static void publish_stop(PIDController& f) { f.publish_stop_command(); }
    static void fail_publication(PIDController& f) {
        f.simulate_stop_publication_failure_ = true;
        f.simulate_event_publication_failure_ = true;
    }
    static bool group_b_started(const PIDController& f) { return f.group_b_started_; }
    static void expect_waiting_group_b_excluded(const PIDController& f) {
        for (int i : {0, 2, 4}) EXPECT_EQ(f.last_position_errors_[i], 0.0f);
    }
    static void begin_stop(PIDController& f, double now) {
        f.state_ = PIDController::State::STOPPING; f.stopping_started_ = true;
        f.stopping_start_time_ = time(now); f.stopping_start_ratio_ = f.current_ratio_;
    }
    static bool position_guard(PIDController& f, unsigned bad_bits) {
        std::array<float, 6> errors {};
        for (int i = 0; i < 6; ++i) if (bad_bits & (1U << i)) errors[i] = 100000.0f;
        bool failed = false;
        for (int n = 0; n < f.safety_.position_error_trip_samples; ++n)
            failed = f.check_position_error_safety(errors) || failed;
        return failed;
    }
};
namespace {
using Access = TripodOfflineTestAccess;
class TripodMultiLegTest : public ::testing::Test {
protected:
    std::filesystem::path dir, path;
    static void TearDownTestSuite() { if (rclcpp::ok()) rclcpp::shutdown(); }
    void SetUp() override {
        ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
        ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "231");
        if (!rclcpp::ok()) { int argc = 0; rclcpp::init(argc, nullptr); }
        g_shutdown_requested = false;
        char pattern[] = "/tmp/rinbo-tripod-test-XXXXXX";
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
            { rinbo_config::MotionSession calibration(rinbo_config::Stage::Calibration, path); calibration.complete(); }
            { rinbo_config::MotionSession standing(rinbo_config::Stage::Standing, path); standing.complete(); }
        } // prerequisite fixtures, never claims about mechanical actions
    }
};
TEST_F(TripodMultiLegTest, All64MasksCoverStartupBothPhaseGroupsAndControlledStop) {
    for (unsigned bits = 0; bits < 64; ++bits) {
        SCOPED_TRACE(bits); mask(bits);
        if (bits == 63) {
            EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Tripod, path), std::exception);
            continue;
        }
        rinbo_config::MotionSession session(rinbo_config::Stage::Tripod, path);
        PIDController f(session); Access::initialize(f);
        auto p = Access::startup_positions(f);
        Access::tick(f, 18.0, p);
        ASSERT_TRUE(Access::running(f)); EXPECT_FALSE(Access::stopped(f));
        Access::tick(f, 18.01, p);
        EXPECT_FALSE(Access::group_b_started(f)); Access::expect_waiting_group_b_excluded(f);
        Access::tick(f, 20.0, p);
        EXPECT_TRUE(Access::group_b_started(f)); EXPECT_FALSE(Access::stopped(f));
        Access::begin_stop(f, 20.0);
        Access::tick(f, 20.01, p);
        EXPECT_FALSE(Access::fully_stopped(f));
    }
}
TEST_F(TripodMultiLegTest, HealthyStartupFailureCannotEnterRunningOrRetainReceipts) {
    mask(5);
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Tripod, path);
        PIDController f(session); Access::initialize(f);
        Access::tick(f, 18.0, Access::startup_positions(f, false));
        EXPECT_TRUE(Access::stopped(f)); EXPECT_FALSE(Access::running(f));
        EXPECT_NE(Access::reason(f).find("startup position error: L2"), std::string::npos);
    }
    EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Tripod, path), std::exception);
}
TEST_F(TripodMultiLegTest, PositionProtectionIgnoresExactlyTheMaskedLegs) {
    mask(5);
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Tripod, path);
        PIDController f(session);
        EXPECT_FALSE(Access::position_guard(f, 5)); EXPECT_FALSE(Access::stopped(f));
    }
    mask(1);
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Tripod, path);
        PIDController f(session);
        EXPECT_TRUE(Access::position_guard(f, 5)); EXPECT_TRUE(Access::stopped(f));
        EXPECT_NE(Access::reason(f).find("position error L3"), std::string::npos);
    }
}
TEST_F(TripodMultiLegTest, NormalStopPublishesDisabledCommandThenEnds) {
    mask(5);
    rinbo_config::MotionSession session(rinbo_config::Stage::Tripod, path);
    PIDController f(session); Access::initialize(f);
    auto p = Access::startup_positions(f);
    Access::tick(f, 18.0, p); ASSERT_TRUE(Access::running(f));
    Access::begin_stop(f, 18.0); Access::tick(f, 20.1, p);
    EXPECT_TRUE(Access::fully_stopped(f)); EXPECT_FALSE(Access::stopped(f));
}
TEST_F(TripodMultiLegTest, ShutdownPublishFailuresCannotPreservePrerequisiteReceipts) {
    mask(5);
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Tripod, path);
        PIDController f(session);
        rclcpp::shutdown();
        Access::fail_publication(f); // deterministically cover both catch paths for every RMW
        EXPECT_ANY_THROW(Access::publish_stop(f));
        EXPECT_NO_THROW(f.fail_closed("test: ROS context already stopped"));
        EXPECT_TRUE(Access::stopped(f));
    }
    EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Tripod, path), std::exception);
}
} // namespace
