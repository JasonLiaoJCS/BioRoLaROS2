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
    static double search_speed(const CalibrationFSM& f){return f.target_vel_counts_;}
    static void opposite_l1(CalibrationFSM& f, bool hall_triggered) {
        std::array<float,6> p{}, v{};
        std::array<bool,6> halls{}; halls.fill(true);
        rinbo_msgs::msg::MotorCmdStamped cmd;
        f.handle_dc_spinning(cmd,p,v,halls,time(10));
        p[0] = 501;
        halls[0] = !hall_triggered;
        f.handle_dc_spinning(cmd,p,v,halls,time(10.1));
    }
    static bool stopped(const CalibrationFSM& f){return f.safety_stopped_;}
    static std::string reason(const CalibrationFSM& f){return f.safety_stop_reason_;}
    static LegState leg(const CalibrationFSM& f,int i){return f.leg_states_[i];}
    static void skip_home(CalibrationFSM& f){f.state_=CalibState::DC_SPINNING;}
    static rinbo_msgs::msg::MotorCmdStamped servos(CalibrationFSM& f,
            const std::array<uint32_t,6>& current, bool hold=false) {
        rinbo_msgs::msg::MotorCmdStamped cmd;
        if (hold) f.set_servo_hold_targets(cmd,current); else f.set_servos(cmd,current);
        return cmd;
    }
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
    void mask(unsigned bits) {
        // This suite exercises FSM transitions, not the management CLI. Prepare
        // only its private fixture; unrelated live controllers must not block
        // offline tests (production update_disabled_legs keeps its process gate).
        std::vector<std::string> names;
        for (int i=0; i<6; ++i)
            if (bits & (1U<<i)) names.emplace_back(rinbo_fsm::DisabledLegs::kLegNames[i]);
        auto document = YAML::LoadFile(path);
        document["disabled_legs"] = names;
        document["revision"] = document["revision"].as<int64_t>() + 1;
        std::ofstream output(path);
        output << document;
        output.close();
        ASSERT_TRUE(output.good());
    }
};
TEST_F(CaliMultiLegTest, WrongDirectionCannotBeHiddenByFindingHall) {
    mask(62);
    for (bool hall : {false, true}) {
        rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path,{"L1"});
        CalibrationFSM f(session);
        Access::opposite_l1(f, hall);
        EXPECT_TRUE(Access::stopped(f));
        EXPECT_FALSE(Access::done(f));
        EXPECT_NE(Access::reason(f).find("opposite encoder travel: L1"),std::string::npos);
    }
}

TEST_F(CaliMultiLegTest, ManualPlanValuesOnlySelectLegsAndDoNotOverrideCalibrationParameters) {
    mask(0);
    const auto before = rinbo_config::RobotConfig::load(path).hash();
    for (int pwm : {20,80}) {
        const auto plan = rinbo_manual::parse("duration_s: 3\nmax_pwm: " + std::to_string(pwm) +
            "\nmax_speed_deg_s: 90\nlegs: {L2: {mode: velocity, speed_deg_s: -70}}\n");
        rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path,
            rinbo_manual::selected_legs(plan));
        CalibrationFSM f(session);
        EXPECT_DOUBLE_EQ(f.get_parameter("max_pwm").as_double(),80.0);
        EXPECT_NEAR(Access::search_speed(f),55296.0/10.0,0.01);
        EXPECT_EQ(session.active_legs(),std::vector<std::string>{"L2"});
        EXPECT_EQ(rinbo_config::RobotConfig::load(path).hash(),before);
    }
}
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
TEST_F(CaliMultiLegTest, SelectedL2IgnoresOtherHallsAndOnlyCertifiesL2) {
    mask(1); // L3 remains enabled in the shared site config.
    const auto original_hash = rinbo_config::RobotConfig::load(path).hash();
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path,{"L2"});
        CalibrationFSM f(session);
        Access::home(f,61); // every unselected servo is away from its homing target
        Access::tick(f,11,61,false); Access::tick(f,11.4,61,false);
        EXPECT_FALSE(Access::done(f));
        Access::tick(f,11.5,61,true); Access::tick(f,11.6,61,true);
        EXPECT_TRUE(Access::done(f)); EXPECT_FALSE(Access::stopped(f));
        const std::array<uint32_t,6> current{501,502,503,504,505,506};
        const auto cmd = Access::servos(f,current);
        EXPECT_EQ(cmd.sl2.position_encoder,2565U);
        EXPECT_EQ(cmd.sl1.position_encoder,501U); EXPECT_EQ(cmd.sl3.position_encoder,503U);
        EXPECT_EQ(cmd.sr1.position_encoder,504U); EXPECT_EQ(cmd.sr2.position_encoder,505U);
        EXPECT_EQ(cmd.sr3.position_encoder,506U);
        EXPECT_EQ(Access::servos(f,current,true).sl2.position_encoder,502U);
    }
    const auto config = rinbo_config::RobotConfig::load(path);
    EXPECT_EQ(config.hash(),original_hash);
    EXPECT_NO_THROW(rinbo_config::check_manual_readiness(config,{"L2"}));
    EXPECT_THROW(rinbo_config::check_manual_readiness(config,{"L3"}),std::exception);
    EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Standing,path),std::exception);
}
TEST_F(CaliMultiLegTest, EverySingleLegCompletesWithoutWaitingForFiveUnselectedLegs) {
    mask(0); // All six are available; only the action selection limits scope.
    for (unsigned selected=0; selected<6; ++selected) {
        SCOPED_TRACE(selected);
        const std::string name = rinbo_fsm::DisabledLegs::kLegNames[selected];
        const unsigned others = 63U ^ (1U << selected);
        {
            rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path,{name});
            CalibrationFSM f(session);
            Access::home(f,others); // Other servos never reach their targets.
            Access::tick(f,11,others,false); // Other encoders invalid, Halls never trigger.
            Access::tick(f,11.4,others,false);
            EXPECT_FALSE(Access::done(f)); // Merely sending reset is insufficient.
            Access::tick(f,11.5,others,true);
            Access::tick(f,11.6,others,true);
            EXPECT_TRUE(Access::done(f));
            EXPECT_FALSE(Access::stopped(f));
        }
        const auto config = rinbo_config::RobotConfig::load(path);
        EXPECT_NO_THROW(rinbo_config::check_manual_readiness(config,{name}));
        for (unsigned other=0; other<6; ++other)
            if (other != selected)
                EXPECT_THROW(rinbo_config::check_manual_readiness(
                    config,{rinbo_fsm::DisabledLegs::kLegNames[other]}),std::exception);
    }
}
TEST_F(CaliMultiLegTest, StopSignalsAfterDoneKeepContextAliveUntilCallbacksEndAndPreserveReceipt) {
    mask(0);
    for (const int signal : {SIGINT, SIGTERM}) {
        SCOPED_TRACE(signal);
        rinbo_cali_lifecycle::install_signal_handlers();
        {
            rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path,{"L2"});
            auto node = std::make_shared<CalibrationFSM>(session);
            Access::home(*node,61);
            Access::tick(*node,11,61,false); Access::tick(*node,11.4,61,false);
            Access::tick(*node,11.5,61,true); Access::tick(*node,11.6,61,true);
            ASSERT_TRUE(Access::done(*node));
            bool queried_after_signal = false;
            auto signal_timer = node->create_wall_timer(std::chrono::milliseconds(1), [&] {
                std::raise(signal);
                // Reproduce a callback still querying the graph after SIGINT.
                // The old default ROS handler invalidated the context here.
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                EXPECT_TRUE(rclcpp::ok());
                EXPECT_NO_THROW(node->get_publishers_info_by_topic("/motor/state"));
                EXPECT_NO_THROW(node->get_publishers_info_by_topic("/power/state"));
                queried_after_signal = true;
            });
            EXPECT_EQ(spin_calibration_until_stop(node),0);
            EXPECT_TRUE(queried_after_signal);
            EXPECT_FALSE(Access::stopped(*node));
            EXPECT_TRUE(rclcpp::ok());
        }
        const auto config = rinbo_config::RobotConfig::load(path);
        EXPECT_NO_THROW(rinbo_config::check_manual_readiness(config,{"L2"}));
        EXPECT_THROW(rinbo_config::check_manual_readiness(config,{"R2"}),std::exception);
    }
}
TEST_F(CaliMultiLegTest, StopBeforeCompletionCannotLeaveAnApprovedReceipt) {
    mask(0);
    rinbo_cali_lifecycle::install_signal_handlers();
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path,{"L2"});
        auto node = std::make_shared<CalibrationFSM>(session);
        auto signal_timer = node->create_wall_timer(std::chrono::milliseconds(1), [] { std::raise(SIGINT); });
        EXPECT_EQ(spin_calibration_until_stop(node),1);
        EXPECT_TRUE(Access::stopped(*node));
        EXPECT_FALSE(Access::done(*node));
    }
    EXPECT_THROW(rinbo_config::check_manual_readiness(rinbo_config::RobotConfig::load(path),{"L2"}),std::exception);
}
TEST_F(CaliMultiLegTest, RealFailureAfterDoneIsNotHiddenByNormalShutdown) {
    mask(0);
    rinbo_cali_lifecycle::install_signal_handlers();
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path,{"L2"});
        auto node = std::make_shared<CalibrationFSM>(session);
        Access::home(*node,61);
        Access::tick(*node,11,61,false); Access::tick(*node,11.4,61,false);
        Access::tick(*node,11.5,61,true); Access::tick(*node,11.6,61,true);
        ASSERT_TRUE(Access::done(*node));
        node->fail_closed("real input failure after completion");
        rinbo_cali_lifecycle::stop_requested = 1;
        EXPECT_EQ(spin_calibration_until_stop(node),1);
    }
    EXPECT_THROW(rinbo_config::check_manual_readiness(rinbo_config::RobotConfig::load(path),{"L2"}),std::exception);
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

TEST_F(CaliMultiLegTest, RelaxedResetWaitStillRequiresActualZeroFeedback) {
    auto doc = YAML::LoadFile(path);
    doc["parameters"]["rinbo_cali"]["safety"]["stop_timeout_s"] = 5.0;
    { std::ofstream out(path); out << doc; }
    mask(5);
    rinbo_config::MotionSession session(rinbo_config::Stage::Calibration, path);
    CalibrationFSM f(session); Access::home(f,5);
    Access::tick(f,11,5,false); Access::tick(f,11.4,5,false);
    Access::tick(f,14,5,false); // old 2s deadline expired
    EXPECT_FALSE(Access::stopped(f)); EXPECT_FALSE(Access::done(f));
    Access::tick(f,15,5,true); Access::tick(f,15.1,5,true);
    EXPECT_TRUE(Access::done(f));
}

TEST_F(CaliMultiLegTest, LongerHallWaitStillRequiresRealHallAndCannotApproveCalibration) {
    auto doc=YAML::LoadFile(path);
    doc["parameters"]["rinbo_cali"]["safety"]["hall_search_timeout_s"]=60.0;
    doc["parameters"]["rinbo_cali"]["safety"]["servo_homing_timeout_s"]=60.0;
    doc["parameters"]["rinbo_cali"]["safety"]["stop_timeout_s"]=15.0;
    {std::ofstream out(path);out<<doc;}
    mask(47); // only R2, L3 remains masked
    {
        rinbo_config::MotionSession session(rinbo_config::Stage::Calibration,path);
        CalibrationFSM f(session);Access::home(f,47);
        Access::tick(f,11,63,false);
        Access::tick(f,45,63,false);
        EXPECT_FALSE(Access::stopped(f));EXPECT_FALSE(Access::done(f));
        Access::tick(f,72,63,false);
        EXPECT_TRUE(Access::stopped(f));EXPECT_FALSE(Access::done(f));
        EXPECT_NE(Access::reason(f).find("hall search timeout: R2"),std::string::npos);
    }
    EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Standing,path),std::exception);
}
