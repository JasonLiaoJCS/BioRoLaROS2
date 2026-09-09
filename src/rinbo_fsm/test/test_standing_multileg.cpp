#define RINBO_FSM_OFFLINE_TEST
#include "rinbo_standing.cpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
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
            positions[i] = halls[i] ? (i < 3 ? -1000.0f : 1000.0f) :
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
    static void sample_r2(StandingController& f, double now, float raw) {
        std::array<float,6> p{}; p[4] = raw;
        std::array<bool,6> hall{};
        f.handle_standing_sample(p,hall,f.servo_targets_,time(now),std::max(0.001,now-f.prev_time_.seconds()));
    }
    static bool complete(const StandingController& f) { return f.completion_recorded_; }
    static void sample(StandingController& f, double now, float l1) {
        std::array<float, 6> p{}; p[0] = l1;
        std::array<bool, 6> hall{};
        f.handle_standing_sample(p, hall, f.servo_targets_, time(now),
            std::max(0.001, now - f.prev_time_.seconds()));
    }
    static rinbo_msgs::msg::MotorCmdStamped command(StandingController& f, int i, float pwm) {
        rinbo_msgs::msg::MotorCmdStamped cmd; f.set_leg_cmd(cmd, i, pwm); return cmd;
    }
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
        // Private fixture only; do not run the live process-management gate.
        auto document = YAML::LoadFile(path);
        document["disabled_legs"] = names;
        document["revision"] = document["revision"].as<int64_t>() + 1;
        { std::ofstream out(path); out << document; ASSERT_TRUE(out.good()); }
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
            EXPECT_FALSE(Access::complete(f));
            Access::tick(f, 15.6, bits, true);
            Access::tick(f, 16.0, bits, true);
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
        Access::tick(f, 15.6, 5, true);
        Access::tick(f, 16.0, 5, true);
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
        Access::tick(f, 15.6, 5, true);
        Access::tick(f, 16.0, 5, true);
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

TEST_F(StandingMultiLegTest, RelaxedRotateDeadlineAllowsLateArrivalButStillRequiresTarget) {
    auto doc = YAML::LoadFile(path);
    doc["parameters"]["rinbo_standing"]["safety"]["rotate_timeout_s"] = 20.0;
    { std::ofstream out(path); out << doc; }
    mask(5);
    rinbo_config::MotionSession session(rinbo_config::Stage::Standing, path);
    StandingController f(session); Access::initialize(f);
    Access::tick(f, 10.0, 5, false);
    Access::tick(f, 18.0, 5, false); // original 7s deadline would stop here
    EXPECT_FALSE(Access::complete(f)); EXPECT_FALSE(Access::stopped(f));
    Access::tick(f, 20.0, 5, true);
    Access::tick(f, 20.1, 5, true);
    Access::tick(f, 20.5, 5, true);
    EXPECT_TRUE(Access::complete(f)); EXPECT_FALSE(Access::stopped(f));
}

TEST_F(StandingMultiLegTest, OppositeL1FeedbackStopsBeforeItCanChaseAnotherRevolution) {
    mask(62); // L1 only, same sign mismatch as the reported positive count drift.
    rinbo_config::MotionSession session(rinbo_config::Stage::Standing, path);
    StandingController f(session); Access::initialize(f);
    Access::sample(f, 10.0, 0);
    Access::sample(f, 10.2, 501);
    EXPECT_TRUE(Access::stopped(f));
    EXPECT_FALSE(Access::complete(f));
    EXPECT_NE(Access::reason(f).find("opposite encoder travel: L1"), std::string::npos);
}

TEST_F(StandingMultiLegTest, PassingTargetAtSpeedCannotCompleteAndHoldDivergenceInvalidates) {
    mask(62);
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Standing, path);
        StandingController f(session); Access::initialize(f);
        Access::sample(f, 10.0, 0);
        Access::sample(f, 15.1, -27648);
        EXPECT_EQ(Access::leg(f, 0), LegState::ROTATE_180);
        Access::sample(f, 15.2, -27648);
        EXPECT_FALSE(Access::complete(f));
        Access::sample(f, 15.6, -27648);
        EXPECT_FALSE(Access::complete(f));
        Access::sample(f, 16.0, -27648);
        EXPECT_TRUE(Access::complete(f));
        Access::sample(f, 16.4, -42000);
        EXPECT_TRUE(Access::stopped(f));
        EXPECT_NE(Access::reason(f).find("standing hold position lost: L1"), std::string::npos);
    }
    EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Tripod, path), std::exception);
}

TEST_F(StandingMultiLegTest, CommandDirectionsMatchCalibrationConventionForBothSigns) {
    mask(0);
    rinbo_config::MotionSession session(rinbo_config::Stage::Standing, path);
    StandingController f(session);
    for (int i = 0; i < 6; ++i) for (float pwm : {-40.0f, 40.0f}) {
        const auto cmd = Access::command(f, i, pwm);
        const std::array<const rinbo_msgs::msg::LegCmd*,6> legs{
            &cmd.l1,&cmd.l2,&cmd.l3,&cmd.r1,&cmd.r2,&cmd.r3};
        EXPECT_EQ(legs[i]->direction, i < 3 ? pwm < 0 : pwm >= 0);
        EXPECT_TRUE(legs[i]->enable);
        EXPECT_FLOAT_EQ(legs[i]->voltage, 40);
        EXPECT_FALSE(legs[i]->reset_position);
    }
}

TEST(StandingReference, SmoothStartAndStopNeverAskForReverseOrExceedCruiseSpeed) {
    const double speed = 55296.0/10.0;
    const double distance = 27648;
    const double end = rinbo_fsm::standing_duration(distance,speed);
    EXPECT_NEAR(end,5.5,1e-9);
    double previous = 0;
    for (int i=0; i<=6000; ++i) {
        const auto ref = rinbo_fsm::standing_reference(i*0.001,distance,speed);
        EXPECT_GE(ref.position,previous);
        EXPECT_LE(ref.position,distance);
        EXPECT_GE(ref.velocity,0);
        EXPECT_LE(ref.velocity,speed);
        previous=ref.position;
    }
    EXPECT_DOUBLE_EQ(rinbo_fsm::standing_reference(0,distance,speed).velocity,0);
    EXPECT_DOUBLE_EQ(rinbo_fsm::standing_reference(end,distance,speed).velocity,0);
    EXPECT_LT(rinbo_fsm::standing_reference(end-1e-6,distance,speed).velocity,0.1);
    EXPECT_DOUBLE_EQ(rinbo_fsm::search_reference(0,speed).velocity,0);
    EXPECT_NEAR(rinbo_fsm::search_reference(.5,speed).velocity,speed,1e-9);
}

TEST_F(StandingMultiLegTest, ReportedR2Error218FailsOldToleranceAndPassesConfiguredTolerance) {
    for (double tolerance : {200.0, 1000.0}) {
        auto doc = YAML::LoadFile(path);
        doc["parameters"]["rinbo_standing"]["safety"]["position_tolerance_counts"] = tolerance;
        doc["parameters"]["rinbo_standing"]["safety"]["rotate_timeout_s"] = 20.0;
        {std::ofstream out(path); out << doc;}
        mask(47); // only R2; L3 stays masked
        rinbo_config::MotionSession session(rinbo_config::Stage::Standing,path);
        StandingController f(session);Access::initialize(f);
        Access::sample_r2(f,10,0);
        Access::sample_r2(f,15.6,27866);
        EXPECT_FALSE(Access::complete(f)); // still moving on first arrival sample
        Access::sample_r2(f,16,27866);
        EXPECT_FALSE(Access::complete(f)); // has not maintained low speed yet
        Access::sample_r2(f,16.4,27866);
        if (tolerance == 1000) {
            EXPECT_TRUE(Access::complete(f)); EXPECT_FALSE(Access::stopped(f));
        } else {
            EXPECT_FALSE(Access::complete(f));
            Access::sample_r2(f,30.001,27866);
            EXPECT_TRUE(Access::stopped(f));
            const auto first=Access::reason(f);
            EXPECT_NE(first.find("standing position timeout: R2"),std::string::npos);
            EXPECT_NE(first.find("error=218"),std::string::npos);
            EXPECT_NE(first.find("tolerance_counts=200"),std::string::npos);
            f.fail_closed("later failure"); EXPECT_EQ(Access::reason(f),first);
        }
    }
}
TEST_F(StandingMultiLegTest, WiderArrivalToleranceStillRejectsPersistentErrorOutsideBand) {
    auto doc=YAML::LoadFile(path);
    doc["parameters"]["rinbo_standing"]["safety"]["position_tolerance_counts"]=1000.0;
    doc["parameters"]["rinbo_standing"]["safety"]["rotate_timeout_s"]=60.0;
    {std::ofstream out(path);out<<doc;}
    mask(47);
    rinbo_config::MotionSession session(rinbo_config::Stage::Standing,path);
    StandingController f(session);Access::initialize(f);
    Access::sample_r2(f,10,0);
    Access::sample_r2(f,16,28649);
    Access::sample_r2(f,17,28649);
    Access::sample_r2(f,31,28649);
    EXPECT_FALSE(Access::stopped(f)); EXPECT_FALSE(Access::complete(f));
    Access::sample_r2(f,70.001,28649);
    EXPECT_TRUE(Access::stopped(f)); EXPECT_FALSE(Access::complete(f));
}

TEST_F(StandingMultiLegTest, PositionOnlyChoiceCompletesOnFirstArrivalButKeepsHoldGuard) {
    auto doc=YAML::LoadFile(path);
    auto s=doc["parameters"]["rinbo_standing"];
    s["max_pwm"]=500.;s["kp"]=.35;s["kd"]=.002;s["k_ff"]=.02;
    s["friction_pwm"]=0.;s["velocity_filter_time_constant_s"]=.005;
    s["safety"]["settle_time_s"]=0.;s["safety"]["position_tolerance_counts"]=1000.;
    {std::ofstream out(path);out<<doc;}
    mask(47); // only R2, including L3 mask
    rinbo_config::MotionSession session(rinbo_config::Stage::Standing,path);
    StandingController f(session);Access::initialize(f);
    Access::sample_r2(f,10,0);
    Access::sample_r2(f,15.4,27866); // before reference ends
    EXPECT_FALSE(Access::complete(f));
    Access::sample_r2(f,15.6,27648); // speed exceeds old settling limit; position-only may complete
    EXPECT_TRUE(Access::complete(f));EXPECT_FALSE(Access::stopped(f));
    Access::sample_r2(f,16,42000);
    EXPECT_TRUE(Access::stopped(f));
    EXPECT_NE(Access::reason(f).find("standing hold position lost: R2"),std::string::npos);
}
