#define RINBO_FSM_OFFLINE_TEST
#include "rinbo_tripod.cpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <unistd.h>

// Calls production control/guards with simulated feedback, no executor/Bridge.
struct TripodOfflineTestAccess {
    static rclcpp::Time time(double s) {
        return rclcpp::Time(static_cast<int64_t>(s*1e9), RCL_ROS_TIME);
    }
    static void initialize(PIDController& f) {
        f.initialized_ = true; f.prev_time_ = time(10); f.start_time_ = time(10);
        f.initial_positions_.fill(0); f.prev_positions_.fill(0);
    }
    static std::array<float,6> endpoint(PIDController& f) {
        std::array<float,6> p;
        for (int i=0;i<6;++i) p[i] = f.disabled_legs_.contains(i) ?
            std::numeric_limits<float>::quiet_NaN() : f.theta_home_*f.rad_to_counts_;
        return p;
    }
    static void tick(PIDController& f, double now, const std::array<float,6>& p) {
        auto msg=std::make_shared<rinbo_msgs::msg::MotorStateStamped>();
        msg->header.stamp=time(now); msg->header.seq=++sequence;
        msg->l1.position=p[0]; msg->l2.position=p[1]; msg->l3.position=p[2];
        msg->r1.position=-p[3]; msg->r2.position=-p[4]; msg->r3.position=-p[5];
        f.last_motor_state_time_=time(now);
        f.handle_motion_sample(msg,p,time(now));
        for(int i=0;i<6;++i) {
            if(f.disabled_legs_.contains(i)) {
                EXPECT_EQ(f.prev_command_pwms_[i],0);
                EXPECT_EQ(f.last_position_errors_[i],0);
                EXPECT_EQ(f.home_offsets_[i],0);
            } else EXPECT_TRUE(std::isfinite(f.prev_command_pwms_[i]));
        }
    }
    static void boundary(PIDController& f, float error=0) {
        initialize(f);
        auto p=endpoint(f); p[1]-=error;
        f.prev_positions_=p; f.startup_time_=f.startup_duration_;
        tick(f,10.1,p);
        // Preserve the boundary frame; tests advance from this exact instant.
    }
    static std::array<float,6> next_reference(PIDController& f, double dt) {
        std::array<float,6> p=endpoint(f);
        const double ratio=std::max(f.target_ratio_,f.current_ratio_+f.ratio_step_*dt/.001);
        const double saved=f.current_ratio_; f.current_ratio_=ratio;
        for(int i=0;i<6;++i) {
            if(f.disabled_legs_.contains(i)) continue;
            double rad,v;
            f.compute_group_reference(f.tau_+dt/ratio-(i%2==0?f.phase_offset_B_:0),rad,v);
            p[i]=f.home_offsets_[i]+rad*f.rad_to_counts_;
        }
        f.current_ratio_=saved;
        return p;
    }
    static void advance(PIDController& f,double dt) {
        tick(f,f.prev_time_.seconds()+dt,next_reference(f,dt));
    }
    static bool running(const PIDController& f) { return f.state_==PIDController::State::RUNNING; }
    static bool stopped(const PIDController& f) { return f.state_==PIDController::State::SAFETY_STOP; }
    static bool fully_stopped(const PIDController& f) { return f.fully_stopped_; }
    static bool group_b(const PIDController& f) { return f.group_b_started_; }
    static auto debug(const PIDController& f) { return f.trace_debug_; }
    static std::array<double,2> budget(const PIDController& f) { return {f.reference_peak_pwm_, f.reference_peak_slew_}; }
    static std::string reason(const PIDController& f) { return f.safety_stop_reason_; }
    static void begin_stop(PIDController& f) {
        f.capture_stopping_reference(); f.state_=PIDController::State::STOPPING;
        f.stopping_started_=true; f.stopping_start_time_=f.prev_time_;
    }
    static void watchdog(PIDController& f) { f.watchdog_callback(); }
    static void publish_stop(PIDController& f) { f.publish_stop_command(); }
    static void fail_publication(PIDController& f) {
        f.simulate_stop_publication_failure_=true; f.simulate_event_publication_failure_=true;
    }
    static bool guard(PIDController& f,double t,int leg,float error) {
        std::array<float,6> errors{};errors[leg]=error;
        return f.check_position_error_safety(errors,t);
    }
    static void periodicity(PIDController& f) {
        for(double x:{-3.01,-1.8,-1.0,-.99,-.5,-.01,0.,.01,.5,.99,3.01}) {
            double p0,v0,p1,v1;bool a,b;
            f.compute_trajectory(x*f.period_,p0,v0,a);
            f.compute_trajectory((x+1)*f.period_,p1,v1,b);
            EXPECT_NEAR(p1-p0,2*M_PI,1e-7);EXPECT_NEAR(v1,v0,1e-7);
        }
        // Position and velocity must meet on both sides of LO/TD and wrap.
        for(double t:{f.t_stance_,f.period_}) {
            double p0,v0,p1,v1;bool a,b;
            f.compute_trajectory(t-1e-9,p0,v0,a);f.compute_trajectory(t+1e-9,p1,v1,b);
            EXPECT_NEAR(p1,p0,1e-6);EXPECT_NEAR(v1,v0,1e-6);
        }
    }
    static void startup_tracking(PIDController& f) {
        initialize(f);
        for(int n=1;n<=3000;++n) {
            const double t=n*.01;
            const auto reference=rinbo_fsm::tripod::startup(t,30,0,rinbo_fsm::tripod::kCountsPerRevolution);
            auto p=endpoint(f);for(int i=0;i<6;++i) if(!f.disabled_legs_.contains(i)) p[i]=reference.position;
            tick(f,10+t,p);ASSERT_FALSE(stopped(f));
        }
        auto p=endpoint(f);for(int n=1;n<=40;++n) tick(f,40+n*.01,p);
        ASSERT_TRUE(running(f));
    }
    static void check_slew(PIDController& f) {
        for(double dt:{.0005,.001,.005,.02}) {
            f.prev_command_pwms_.fill(0);std::array<float,6> p;p.fill(80);
            f.apply_pwm_slew_limit(p,dt);
            for(int i=0;i<6;++i) EXPECT_NEAR(p[i],f.disabled_legs_.contains(i)?0:250*dt,1e-6);
        }
        f.publish_stop_command();for(float p:f.prev_command_pwms_) EXPECT_EQ(p,0);
    }
    static void check_ratio_rate(PIDController& f) {
        f.current_ratio_=81; f.target_ratio_=80;
        for(int n=0;n<100;++n) advance(f,.001);
        EXPECT_NEAR(f.current_ratio_,80.995,1e-8);
        f.current_ratio_=81;
        for(int n=0;n<10;++n) advance(f,.01);
        EXPECT_NEAR(f.current_ratio_,80.995,1e-8);
    }
    static void continuous_running(PIDController& f) {
        boundary(f);
        f.current_ratio_=81; f.target_ratio_=80;
        // 250 simulated seconds: crosses target ratio, startup duration and
        // both shutdown limits; none is a RUNNING lifetime limit.
        for(int n=0;n<5000;++n) {
            advance(f,.05);
            ASSERT_TRUE(running(f)); ASSERT_FALSE(fully_stopped(f));
            if(n>=401) EXPECT_DOUBLE_EQ(f.current_ratio_,80);
        }
        EXPECT_GT(f.cycle_count_,5);
        const auto last=f.trace_debug_;
        begin_stop(f);
        tick(f,f.prev_time_.seconds()+2.1,last.actual_position);
        EXPECT_TRUE(fully_stopped(f));EXPECT_FALSE(stopped(f));
    }
    static void fault_sample(PIDController& f) {
        initialize(f); f.startup_time_=f.startup_duration_;
        auto p=endpoint(f);p[1]-=18008.929688f;f.prev_positions_=p;
        tick(f,10.001,p);
    }
    static void stall_waiting_b(PIDController& f) {
        auto p=next_reference(f,.01);p[0]-=18001;
        tick(f,f.prev_time_.seconds()+.01,p);
    }
    static void endpoint_timeout(PIDController& f) {
        initialize(f);f.startup_time_=f.startup_duration_+2;
        auto p=endpoint(f);p[1]-=300;f.prev_positions_=p;
        tick(f,10.01,p);
    }
    static void blocked_startup(PIDController& f) {
        initialize(f);f.startup_time_=f.startup_duration_;
        auto p=endpoint(f);p[1]=0;f.prev_positions_=p;tick(f,10.01,p);
    }
    static void invalid_dt(PIDController& f) { initialize(f);tick(f,10,endpoint(f)); }
    static inline uint32_t sequence=0;
};
namespace {
using Access=TripodOfflineTestAccess;
class TripodMultiLegTest:public ::testing::Test {
protected:
    std::filesystem::path dir,path;
    void SetUp() override {
        ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"),"1");
        ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"231");
        if(!rclcpp::ok()){int argc=0;rclcpp::init(argc,nullptr);}
        ASSERT_EQ(rcutils_logging_set_logger_level("rinbo_tripod_rslip",RCUTILS_LOG_SEVERITY_ERROR),RCUTILS_RET_OK);
        g_shutdown_requested=false;
        char pattern[]="/tmp/rinbo-tripod-test-XXXXXX";dir=mkdtemp(pattern);path=dir/"robot.yaml";
        std::filesystem::copy_file(std::filesystem::path(RINBO_FSM_SOURCE_DIR)/"test/fixtures/robot_test.yaml",path);
    }
    void TearDown() override { std::filesystem::remove_all(dir); }
    static void TearDownTestSuite(){if(rclcpp::ok())rclcpp::shutdown();}
    void fixture(unsigned bits,bool original=false) {
        auto d=YAML::LoadFile(path);
        std::vector<std::string> names;
        for(int i=0;i<6;++i)if(bits&(1U<<i))names.emplace_back(rinbo_fsm::DisabledLegs::kLegNames[i]);
        d["disabled_legs"]=names;d["revision"]=d["revision"].as<int64_t>()+1;
        auto t=d["parameters"]["rinbo_tripod_rslip"];
        t["kp"]=.08;t["kd"]=.006;t["k_ff"]=.005;t["friction_pwm"]=40.;
        t["startup_duration"]=original?8.:30.;t["start_ratio"]=original?8.:80.;t["target_ratio"]=original?4.:80.;
        t["safety"]["max_position_error_counts"]=9000.;
        {std::ofstream out(path);out<<d;ASSERT_TRUE(out.good());}
        if(bits!=63){
            {rinbo_config::MotionSession c(rinbo_config::Stage::Calibration,path);c.complete();}
            {rinbo_config::MotionSession s(rinbo_config::Stage::Standing,path);s.complete();}
        }
    }
};

TEST_F(TripodMultiLegTest, OriginalSiteModelWarningDoesNotAddAnOperationInterlock) {
    fixture(4,true);rinbo_config::MotionSession session(rinbo_config::Stage::Tripod,path);
    PIDController f(session);
    EXPECT_GT(Access::budget(f)[0],300);EXPECT_GT(Access::budget(f)[1],250);
}
TEST_F(TripodMultiLegTest, Suggested20SecondRatio40ProfileFitsNominalBudget) {
    fixture(4);
    auto d=YAML::LoadFile(path);auto t=d["parameters"]["rinbo_tripod_rslip"];
    t["startup_duration"]=20.;t["start_ratio"]=40.;t["target_ratio"]=40.;
    {std::ofstream out(path);out<<d;}
    {rinbo_config::MotionSession c(rinbo_config::Stage::Calibration,path);c.complete();}
    {rinbo_config::MotionSession standing(rinbo_config::Stage::Standing,path);standing.complete();}
    rinbo_config::MotionSession session(rinbo_config::Stage::Tripod,path);PIDController f(session);
    EXPECT_NEAR(Access::budget(f)[0],67.4928,.01);
    EXPECT_NEAR(Access::budget(f)[1],190.234,.1);
}
TEST_F(TripodMultiLegTest, RestToRestStartupIsMonotoneWithContinuousEndpoints) {
    double previous=0;
    for(int n=0;n<=1000;++n){
        auto r=rinbo_fsm::tripod::startup(n*.03,30,0,54984.83);
        EXPECT_GE(r.position+1e-7,previous);EXPECT_GE(r.velocity,0);previous=r.position;
        if(n==0||n==1000)EXPECT_NEAR(r.velocity,0,1e-8);
    }
    EXPECT_NEAR(previous,54984.83,1e-7);
}
TEST_F(TripodMultiLegTest, PhaseLaunchIsContinuousAndKeepsTheUnwrappedPhase) {
    const double h=.1,e=1e-8;
    for(double t:{0.,h}){
        auto a=rinbo_fsm::tripod::launch_phase(t-e,h),b=rinbo_fsm::tripod::launch_phase(t+e,h);
        EXPECT_NEAR(a.position,b.position,3*e);EXPECT_NEAR(a.velocity,b.velocity,1e-6);
    }
    EXPECT_EQ(rinbo_fsm::tripod::launch_phase(-.01,h).velocity,0);
    EXPECT_DOUBLE_EQ(rinbo_fsm::tripod::launch_phase(123.4,h).position,123.4);
}
TEST_F(TripodMultiLegTest, NegativePhaseAndGaitJoinsPreserveMultipleRevolutions) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);Access::periodicity(f);
}
TEST_F(TripodMultiLegTest, HistoricalPolarityAndRawCountsRemainExplicit) {
    for(unsigned i=0;i<6;++i){
        EXPECT_EQ(rinbo_fsm::tripod::position(i,60000),i<3?60000:-60000);
        EXPECT_EQ(rinbo_fsm::tripod::position(i,-60000),i<3?-60000:60000);
        EXPECT_EQ(rinbo_fsm::tripod::direction(i,10),i<3);
        EXPECT_EQ(rinbo_fsm::tripod::direction(i,-10),i>=3);
        const double delta=rinbo_fsm::tripod::position(i,-2147483648.)-rinbo_fsm::tripod::position(i,2147483647.);
        EXPECT_GT(std::fabs(delta),4e9); // Do not silently wrap a feedback reset into a small angle.
    }
}
TEST_F(TripodMultiLegTest, All64MasksCoverStartupBothGroupsAndControlledStop) {
    for(unsigned bits=0;bits<64;++bits){
        SCOPED_TRACE(bits);fixture(bits);
        if(bits==63){EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Tripod,path),std::exception);continue;}
        rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);
        Access::boundary(f);ASSERT_TRUE(Access::running(f));
        Access::advance(f,.01);EXPECT_FALSE(Access::group_b(f));
        for(int n=0;n<170;++n)Access::advance(f,.1);
        EXPECT_TRUE(Access::group_b(f));ASSERT_FALSE(Access::stopped(f));
        Access::begin_stop(f);auto d=Access::debug(f);
        Access::tick(f,d.header.stamp.sec+d.header.stamp.nanosec*1e-9+.001,d.actual_position);
        EXPECT_FALSE(Access::stopped(f));EXPECT_FALSE(Access::fully_stopped(f));
    }
}
TEST_F(TripodMultiLegTest, NormalTrackingAcrossStartupAndGroupBLaunch) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);
    Access::startup_tracking(f);
    auto previous=Access::debug(f);
    for(int n=0;n<1700;++n){
        Access::advance(f,.01);auto d=Access::debug(f);ASSERT_FALSE(Access::stopped(f));
        for(int i:{0,1,3,4,5}){
            EXPECT_LT(std::fabs(d.target_position[i]-previous.target_position[i]),50);
            EXPECT_LT(std::fabs(d.target_velocity[i]-previous.target_velocity[i]),80);
            EXPECT_LE(std::fabs(d.limited_pwm[i]-previous.limited_pwm[i]),2.501);
        }
        previous=d;
    }
    EXPECT_TRUE(Access::group_b(f));
}
TEST_F(TripodMultiLegTest, StartupErrorIsNotErasedByRebasingAndWaitingBRemainsProtected) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);
    Access::boundary(f,100);ASSERT_TRUE(Access::running(f));auto before=Access::debug(f);
    Access::tick(f,10.11,before.actual_position);auto after=Access::debug(f);
    EXPECT_NEAR(after.target_position[1],before.target_position[1],1);
    EXPECT_NEAR(after.position_error[1],100,1);
    EXPECT_FALSE(after.group_b_leg[1]);EXPECT_TRUE(after.group_b_leg[0]);
    Access::stall_waiting_b(f);EXPECT_TRUE(Access::stopped(f));
    EXPECT_NE(Access::reason(f).find("hard position error: L1"),std::string::npos);
}
TEST_F(TripodMultiLegTest, HealthyBlockedStartupCannotEnterRunningOrRetainReceipts) {
    fixture(4);
    {rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);Access::blocked_startup(f);
     EXPECT_TRUE(Access::stopped(f));EXPECT_FALSE(Access::running(f));}
    EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Tripod,path),std::exception);
}
TEST_F(TripodMultiLegTest, StartupKeepsExistingAlignmentToleranceWithoutAnExtraInterlock) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);Access::endpoint_timeout(f);
    EXPECT_FALSE(Access::stopped(f));EXPECT_TRUE(Access::running(f));
}
TEST_F(TripodMultiLegTest, TransientLagRecoversButContinuousLagTripsAfterSamplesAndTime) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);
    for(int n=0;n<400;++n)EXPECT_FALSE(Access::guard(f,10+n*.001,1,10000));
    EXPECT_FALSE(Access::guard(f,10.4,1,0));
    for(int n=0;n<400;++n)EXPECT_FALSE(Access::guard(f,11+n*.001,1,-10000));
    EXPECT_TRUE(Access::guard(f,11.51,1,-10000));
}
TEST_F(TripodMultiLegTest, SoftGuardRequiresBothTenSamplesAndHalfASecond) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);
    for(int n=0;n<9;++n)EXPECT_FALSE(Access::guard(f,10+n*.1,1,9001));
    EXPECT_TRUE(Access::guard(f,10.9,1,9001));
}
TEST_F(TripodMultiLegTest, HardBoundaryAndInvalidFeedbackBypassTransientAllowance) {
    for(float value:{18000.01f,-18000.01f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}){
        fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);
        EXPECT_FALSE(Access::guard(f,10,1,18000));EXPECT_TRUE(Access::guard(f,10.001,1,value));
    }
}
TEST_F(TripodMultiLegTest, PositionProtectionIgnoresExactlyL3Mask) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);
    EXPECT_FALSE(Access::guard(f,10,2,std::numeric_limits<float>::quiet_NaN()));
    EXPECT_TRUE(Access::guard(f,10,1,18001));
}
TEST_F(TripodMultiLegTest, TriggerFrameRetainsRawSignedDataAndFirstReason) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);
    testing::internal::CaptureStderr();
    Access::fault_sample(f);
    const auto fault_log = testing::internal::GetCapturedStderr();
    ASSERT_TRUE(Access::stopped(f));auto d=Access::debug(f);
    for(const auto* token : {"TRIPOD_FAULT_FRAME", "L2 group=A", "raw_counts=", "target_counts=",
                            "signed_error=", "pwm_raw=", "hard=18000", "source_seq="})
        EXPECT_NE(fault_log.find(token),std::string::npos) << token;
    EXPECT_TRUE(d.safety_stopped);EXPECT_GT(d.position_error[1],18000);
    EXPECT_GT(d.raw_pwm[1],80);EXPECT_EQ(d.limited_pwm[1],0);
    EXPECT_NEAR(d.target_position[1]-d.actual_position[1],d.position_error[1],.01);
    EXPECT_EQ(d.controller_state,"SAFETY_STOP");EXPECT_FALSE(d.group_b_leg[1]);
    auto reason=Access::reason(f);f.fail_closed("secondary stale feedback");
    EXPECT_EQ(Access::reason(f),reason);EXPECT_EQ(Access::debug(f).header.seq,d.header.seq);
    EXPECT_EQ(Access::debug(f).position_error[1],d.position_error[1]);
}
TEST_F(TripodMultiLegTest, SlewUsesRealElapsedTimeAndImmediateStopBypassesIt) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);Access::check_slew(f);
}
TEST_F(TripodMultiLegTest, RatioRampIsIndependentOfCallbackRate) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);Access::boundary(f);Access::check_ratio_rate(f);
}
TEST_F(TripodMultiLegTest, TargetRatioAndElapsedTimeNeverAutoCompleteRunning) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);
    PIDController f(s);Access::continuous_running(f);
}
TEST_F(TripodMultiLegTest, ShutdownBrakesFromCurrentReferenceIncludingStartup) {
    for(double v:{-3000.,0.,3000.}) {
        auto a=rinbo_fsm::tripod::brake(0,2,{12345,v}),b=rinbo_fsm::tripod::brake(2,2,{12345,v});
        EXPECT_DOUBLE_EQ(a.position,12345);EXPECT_DOUBLE_EQ(a.velocity,v);
        EXPECT_DOUBLE_EQ(b.velocity,0);EXPECT_DOUBLE_EQ(b.position,12345+v);
    }
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);Access::initialize(f);
    std::array<float,6> p{};Access::tick(f,10.01,p);auto before=Access::debug(f);
    Access::begin_stop(f);Access::tick(f,10.011,p);auto after=Access::debug(f);
    EXPECT_NEAR(after.target_position[1],before.target_position[1],.01);
    EXPECT_LE(std::fabs(after.target_velocity[1]),std::fabs(before.target_velocity[1]));
}
TEST_F(TripodMultiLegTest, NormalStopPublishesDisabledCommandThenEnds) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);Access::boundary(f);
    Access::begin_stop(f);Access::tick(f,12.2,Access::endpoint(f));
    EXPECT_TRUE(Access::fully_stopped(f));EXPECT_FALSE(Access::stopped(f));
}
TEST_F(TripodMultiLegTest, NonPositiveSampleTimeFailsClosed) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);Access::invalid_dt(f);EXPECT_TRUE(Access::stopped(f));
}
TEST_F(TripodMultiLegTest, ShutdownPublishFailuresCannotPreservePrerequisiteReceipts) {
    fixture(4);
    {rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);rclcpp::shutdown();
     Access::fail_publication(f);EXPECT_ANY_THROW(Access::publish_stop(f));EXPECT_NO_THROW(f.fail_closed("ROS context stopped"));EXPECT_TRUE(Access::stopped(f));}
    EXPECT_THROW(rinbo_config::MotionSession(rinbo_config::Stage::Tripod,path),std::exception);
}
TEST_F(TripodMultiLegTest, SafetyStoppedActionHonorsShutdownWithoutFeedback) {
    fixture(4);rinbo_config::MotionSession s(rinbo_config::Stage::Tripod,path);PIDController f(s);
    f.fail_closed("first position error");Access::watchdog(f);EXPECT_TRUE(rclcpp::ok());
    g_shutdown_requested=true;Access::watchdog(f);EXPECT_FALSE(rclcpp::ok());
    EXPECT_TRUE(Access::fully_stopped(f));EXPECT_EQ(Access::reason(f),"first position error");
}
} // namespace
