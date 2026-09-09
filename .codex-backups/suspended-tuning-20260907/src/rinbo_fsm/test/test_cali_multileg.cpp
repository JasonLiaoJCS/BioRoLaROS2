#define RINBO_FSM_OFFLINE_TEST
#include "rinbo_cali.cpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <unistd.h>

// Exercises production transition handlers with synthetic samples. No executor
// is spun, no bridge is present; any safety-stop publication stays in localhost
// test domain 231. This does not execute a production motion entrypoint.
struct CalibrationOfflineTestAccess {
    static rclcpp::Time time(double seconds) { return rclcpp::Time(static_cast<int64_t>(seconds*1e9),RCL_ROS_TIME); }
    static void home(CalibrationFSM& f, unsigned missing) {
        f.state_=CalibState::WAIT_SERVO;
        f.servo_wait_start_time_=time(10);f.state_start_time_=time(10);
        auto positions=f.servo_targets_;
        for(int i=0;i<6;++i) if(missing&(1U<<i)) positions[i]=0;
        rinbo_msgs::msg::MotorCmdStamped cmd;
        f.handle_wait_servo(cmd,positions,time(10.6));
    }
    static void tick(CalibrationFSM& f,double now,unsigned missing,bool zero_feedback) {
        std::array<float,6> positions,velocities;
        std::array<bool,6> halls;
        for(int i=0;i<6;++i) {
            positions[i]=(missing&(1U<<i)) ?
                (f.disabled_legs_.contains(i)?std::numeric_limits<float>::quiet_NaN():1000.0f) :
                (zero_feedback?0.0f:1000.0f);
            velocities[i]=((missing&(1U<<i)) && f.disabled_legs_.contains(i))?std::numeric_limits<float>::quiet_NaN():0.0f;
            halls[i]=(missing&(1U<<i))!=0;
        }
        rinbo_msgs::msg::MotorCmdStamped cmd;
        f.handle_dc_spinning(cmd,positions,velocities,halls,time(now));
        rinbo_fsm::enforce_disabled_leg_commands(cmd,f.disabled_legs_);
        const std::array<const rinbo_msgs::msg::LegCmd*,6> legs={&cmd.l1,&cmd.l2,&cmd.l3,&cmd.r1,&cmd.r2,&cmd.r3};
        for(int i=0;i<6;++i) if(f.disabled_legs_.contains(i)) {
            EXPECT_FALSE(legs[i]->enable);EXPECT_EQ(legs[i]->voltage,0.0f);
            EXPECT_FALSE(legs[i]->reset_position);
            EXPECT_EQ(f.leg_states_[i],LegState::SKIPPED);
        }
    }
    static rinbo_msgs::msg::MotorCmdStamped reset_sample(CalibrationFSM& f, double now, float pos, float vel) {
        std::array<float,6> positions, velocities;
        std::array<bool,6> halls {};
        positions.fill(pos); velocities.fill(vel);
        rinbo_msgs::msg::MotorCmdStamped cmd;
        f.handle_dc_spinning(cmd,positions,velocities,halls,time(now));
        rinbo_fsm::enforce_disabled_leg_commands(cmd,f.disabled_legs_);
        return cmd;
    }
    static bool done(const CalibrationFSM& f){return f.state_==CalibState::DONE;}
    static bool stopped(const CalibrationFSM& f){return f.safety_stopped_;}
    static std::string reason(const CalibrationFSM& f){return f.safety_stop_reason_;}
    static LegState leg(const CalibrationFSM& f,int i){return f.leg_states_[i];}
    static void skip_home(CalibrationFSM& f){f.state_=CalibState::DC_SPINNING;}
};
namespace {
using Access=CalibrationOfflineTestAccess;
class CaliMultiLegTest:public ::testing::Test {
protected:
    std::filesystem::path dir,path;
    static void SetUpTestSuite(){if(!rclcpp::ok()){int argc=0;rclcpp::init(argc,nullptr);}}
    static void TearDownTestSuite(){rclcpp::shutdown();}
    void SetUp()override {
        ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"),"1");
        ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"231");
        char pattern[]="/tmp/rinbo-cali-test-XXXXXX";dir=mkdtemp(pattern);path=dir/"robot.yaml";
        std::filesystem::copy_file(std::filesystem::path(RINBO_FSM_SOURCE_DIR)/"test/fixtures/robot_test.yaml",path);
    }
    void TearDown()override{std::filesystem::remove_all(dir);}
    void mask(unsigned bits){std::vector<std::string> names;for(int i=0;i<6;++i)if(bits&(1U<<i))names.emplace_back(rinbo_fsm::DisabledLegs::kLegNames[i]);rinbo_config::update_disabled_legs(path,names.empty()?"enable-all":"set",names);}
};
TEST_F(CaliMultiLegTest, All64MasksUseActualHealthyCalibrationTransitions) {
    for(unsigned bits=0;bits<64;++bits){
        SCOPED_TRACE(bits);mask(bits);
        if(bits==63){EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Calibration,path),std::exception);continue;}
        rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path);
        CalibrationFSM f(session);
        Access::home(f,bits);EXPECT_FALSE(Access::stopped(f));
        Access::tick(f,11.0,bits,false);Access::tick(f,11.4,bits,false);
        EXPECT_FALSE(Access::done(f)); // reset request alone is not completion
        Access::tick(f,11.5,bits,true);Access::tick(f,11.6,bits,true);
        EXPECT_TRUE(Access::done(f));EXPECT_FALSE(Access::stopped(f));
    }
}
TEST_F(CaliMultiLegTest, L1AndL3MissingOnlyPassWhenBothExplicitlyMasked) {
    mask(5);
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path);
        CalibrationFSM f(session);Access::home(f,5);
        Access::tick(f,11,5,false);Access::tick(f,11.4,5,false);
        Access::tick(f,11.5,5,true);Access::tick(f,11.6,5,true);
        EXPECT_TRUE(Access::done(f));
    }
    mask(1);
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path);
        CalibrationFSM f(session);
        EXPECT_EQ(Access::leg(f,2),LegState::SPINNING); // reenabled L3 never inherits SKIPPED
        Access::skip_home(f); // specifically exercise Hall search failure separately
        Access::tick(f,11,5,false);Access::tick(f,11.4,5,false);
        Access::tick(f,11.5,5,true);Access::tick(f,24,5,true);
        EXPECT_FALSE(Access::done(f));EXPECT_TRUE(Access::stopped(f));
        EXPECT_NE(Access::reason(f).find("hall search timeout: L3"),std::string::npos);
    }
    EXPECT_EQ(rinbo_config::RobotConfig::load(path).disabled_legs(),std::vector<std::string>{"L1"});
}
TEST_F(CaliMultiLegTest, ResetWithoutZeroFeedbackFailsAndCannotCreateReceipt) {
    mask(5);
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path);
        CalibrationFSM f(session);Access::home(f,5);
        Access::tick(f,11,5,false);Access::tick(f,11.4,5,false);
        Access::tick(f,14,5,false);
        EXPECT_TRUE(Access::stopped(f));EXPECT_FALSE(Access::done(f));
        EXPECT_NE(Access::reason(f).find("position reset timeout: L2"),std::string::npos);
    }
    EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Standing,path),std::exception);
}
TEST_F(CaliMultiLegTest, ReenabledServoMustHomeAgain) {
    mask(1);
    rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path);
    CalibrationFSM f(session);Access::home(f,5);
    EXPECT_FALSE(Access::done(f));EXPECT_EQ(Access::leg(f,2),LegState::SPINNING);
}
TEST_F(CaliMultiLegTest, LostResetRetriesOnlyWhileStationaryAndNeverCompletesWithoutFeedback) {
    mask(5);
    rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path);
    CalibrationFSM f(session); Access::home(f,5);
    Access::tick(f,11,5,false); Access::tick(f,11.4,5,false);
    auto cmd=Access::reset_sample(f,11.45,1000,0);
    EXPECT_FALSE(cmd.l2.reset_position);
    cmd=Access::reset_sample(f,11.51,1000,0);
    EXPECT_TRUE(cmd.l2.reset_position); EXPECT_FALSE(cmd.l2.enable); EXPECT_EQ(cmd.l2.voltage,0);
    EXPECT_FALSE(cmd.l1.reset_position); EXPECT_FALSE(cmd.l3.reset_position);
    EXPECT_FALSE(Access::done(f));
    cmd=Access::reset_sample(f,11.56,1000,0);
    EXPECT_FALSE(cmd.l2.reset_position);
    cmd=Access::reset_sample(f,11.75,1000,600);
    EXPECT_FALSE(cmd.l2.reset_position); // moving legs cannot be reset
    cmd=Access::reset_sample(f,11.85,0,0);
    EXPECT_FALSE(cmd.l2.reset_position); // feedback ends retries immediately
    Access::reset_sample(f,11.9,0,0);
    EXPECT_TRUE(Access::done(f)); EXPECT_FALSE(Access::stopped(f));
}
TEST_F(CaliMultiLegTest, LostResetStillTimesOutAtOriginalDeadlineWithDiagnosticValues) {
    mask(5);
    rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path);
    CalibrationFSM f(session); Access::home(f,5);
    Access::tick(f,11,5,false); Access::tick(f,11.4,5,false);
    Access::reset_sample(f,12,1000,0);
    auto cmd=Access::reset_sample(f,13.41,1000,0);
    EXPECT_TRUE(Access::stopped(f)); EXPECT_FALSE(Access::done(f));
    EXPECT_FALSE(cmd.l2.reset_position); EXPECT_FALSE(cmd.l2.enable);
    EXPECT_NE(Access::reason(f).find("pos=1000"),std::string::npos);
    EXPECT_NE(Access::reason(f).find("velocity=0"),std::string::npos);
}
} // namespace
