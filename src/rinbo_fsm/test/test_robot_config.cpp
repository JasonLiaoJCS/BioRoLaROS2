#include "robot_config.hpp"
#include "motion_effort.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>

namespace {
namespace fs = std::filesystem;
using namespace rinbo_config;
class ConfigTest : public ::testing::Test {
protected:
    fs::path dir, path;
    void SetUp() override {
        char pattern[] = "/tmp/rinbo-config-test-XXXXXX";
        dir = ::mkdtemp(pattern);
        path = dir / "robot.yaml";
        fs::copy_file(fs::path(RINBO_FSM_SOURCE_DIR) / "test/fixtures/robot_test.yaml", path);
    }
    void TearDown() override { fs::remove_all(dir); }
    std::string bytes() { std::ifstream f(path); return {std::istreambuf_iterator<char>(f), {}}; }
    void write(const YAML::Node& doc) { std::ofstream f(path); f << doc; }
    void change(const std::function<void(YAML::Node&)>& fn) {
        auto doc = YAML::LoadFile(path); fn(doc); write(doc);
    }
};
TEST_F(ConfigTest, EveryMaskRoundTripsAndPreservesSafetyParameters) {
    const auto baseline = RobotConfig::load(path);
    for (unsigned bits = 0; bits < 64; ++bits) {
        SCOPED_TRACE(bits);
        std::vector<std::string> legs;
        for (int i=0;i<6;++i) if (bits & (1U<<i)) legs.push_back(std::string(i<3?"L":"R")+std::to_string(i%3+1));
        update_disabled_legs(path, legs.empty()?"enable-all":"set", legs);
        const auto loaded = RobotConfig::load(path);
        EXPECT_EQ(loaded.disabled_legs(), legs);
        EXPECT_EQ(loaded.enabled_legs().size(), 6-legs.size());
        for (const auto& name : {"rinbo_cali", "rinbo_standing", "rinbo_tripod_rslip"}) {
            for (const auto& before : baseline.parameters(name)) {
                if (before.get_name().find("hardware.") == 0) continue;
                bool found = false;
                for (const auto& after : loaded.parameters(name)) {
                    if (before.get_name() != after.get_name()) continue;
                    EXPECT_EQ(before, after); found=true;
                }
                EXPECT_TRUE(found) << before.get_name();
            }
        }
    }
}
TEST_F(ConfigTest, ManagementOperationsNormalizeAreAtomicAndIdempotent) {
    EXPECT_TRUE(update_disabled_legs(path,"set",{" l1 ","r3"}));
    EXPECT_EQ(RobotConfig::load(path).disabled_legs(), (std::vector<std::string>{"L1","R3"}));
    const auto saved=bytes();
    EXPECT_FALSE(update_disabled_legs(path,"disable",{"L1"}));
    EXPECT_EQ(bytes(),saved);
    EXPECT_THROW(update_disabled_legs(path,"set",{}),std::exception);
    EXPECT_THROW(update_disabled_legs(path,"set",{"L1","l1"}),std::exception);
    EXPECT_THROW(update_disabled_legs(path,"disable",{"L2","typo"}),std::exception);
    EXPECT_EQ(bytes(),saved);
    EXPECT_TRUE(update_disabled_legs(path,"disable",{"R2"}));
    EXPECT_TRUE(update_disabled_legs(path,"enable",{"L1"}));
    EXPECT_EQ(RobotConfig::load(path).disabled_legs(),(std::vector<std::string>{"R2","R3"}));
    EXPECT_TRUE(update_disabled_legs(path,"enable-all",{}));
    EXPECT_TRUE(RobotConfig::load(path).disabled_legs().empty());
}
TEST_F(ConfigTest, MotionTuningUpdatesAllModesPreservesLimitsAndInvalidatesReceipts) {
    const rinbo_fsm::MotionEffort profile{.08,.006,.005,40,153.6};
    update_disabled_legs(path,"set",{"L3"});
    const auto before=RobotConfig::load(path);
    { MotionSession c(Stage::Calibration,path); c.complete(); }
    EXPECT_TRUE(update_motion_effort(path,profile));
    const auto after=RobotConfig::load(path);
    EXPECT_EQ(after.revision(),before.revision()+1);
    EXPECT_EQ(after.disabled_legs(),before.disabled_legs());
    EXPECT_THROW(MotionSession(Stage::Standing,path),std::exception);
    for (const auto& node : {"rinbo_cali","rinbo_standing","rinbo_tripod_rslip"}) {
        for (const auto& parameter : after.parameters(node)) {
            if (parameter.get_name()=="kp") EXPECT_DOUBLE_EQ(parameter.as_double(),.08);
            else if (parameter.get_name()=="kd") EXPECT_DOUBLE_EQ(parameter.as_double(),.006);
            else if (parameter.get_name()=="k_ff") EXPECT_DOUBLE_EQ(parameter.as_double(),.005);
            else if (parameter.get_name()=="friction_pwm") EXPECT_DOUBLE_EQ(parameter.as_double(),40);
            else if (parameter.get_name()=="friction_velocity_counts_s") EXPECT_DOUBLE_EQ(parameter.as_double(),153.6);
            else {
                const auto& old=before.parameters(node);
                auto found=std::find_if(old.begin(),old.end(),[&](const auto& p){return p.get_name()==parameter.get_name();});
                ASSERT_NE(found,old.end()); EXPECT_EQ(*found,parameter);
            }
        }
    }
    const auto saved=bytes();
    EXPECT_FALSE(update_motion_effort(path,profile));
    EXPECT_EQ(bytes(),saved);
    { MotionSession c(Stage::Calibration,path);
      EXPECT_THROW(update_motion_effort(path,profile),std::exception); }
    auto bad=profile; bad.friction_pwm=81;
    EXPECT_THROW(update_motion_effort(path,bad),std::exception);
    EXPECT_EQ(bytes(),saved);
    change([](auto& d){d["parameters"]["rinbo_cali"]["friction_velocity_counts_s"]=0;});
    EXPECT_THROW(RobotConfig::load(path),std::exception);
}
TEST_F(ConfigTest, CorruptMissingUnsupportedAndConflictingConfigFailClosed) {
    const auto original=bytes();
    const std::vector<std::function<void(YAML::Node&)>> bad = {
        [](auto& d){d["schema_version"]=2;}, [](auto& d){d["revision"]=0;},
        [](auto& d){d.remove("disabled_legs");}, [](auto& d){d["disabled_legs"]="L1";},
        [](auto& d){d["disabled_legs"]=std::vector<std::string>{"X1"};},
        [](auto& d){d["disabled_legs"]=std::vector<std::string>{"L1","l1"};},
        [](auto& d){d["test_mode"]="ground_walk";},
        [](auto& d){d["parameters"].remove("rinbo_standing");},
        [](auto& d){d["parameters"]["rinbo_cali"].remove("max_pwm");},
        [](auto& d){d["parameters"]["rinbo_cali"]["max_pwm"]=501.0;},
        [](auto& d){d["parameters"]["rinbo_cali"]["safety"]["max_current"]=10.1;},
        [](auto& d){d["parameters"]["rinbo_standing"]["safety"]["stop_on_current_limit"]=false;},
        [](auto& d){d["parameters"]["rinbo_tripod_rslip"]["startup_duration"]=0.0;},
        [](auto& d){d["parameters"]["rinbo_cali"]["hardware"]["disabled_legs"]=std::vector<std::string>{"L1"};},
        [](auto& d){d["parameters"]["rinbo_cali"]["safety.max_current"]=2.0;},
        [](auto& d){d["parameters"]["rinbo_cali"]["max_pwm"]="oops";},
        [](auto& d){d["parameters"]["rinbo_cali"]["max_pwmm"]=80.0;}
    };
    for(size_t i=0;i<bad.size();++i) {
        SCOPED_TRACE(i); auto doc=YAML::Load(original);bad[i](doc);write(doc);
        EXPECT_THROW(RobotConfig::load(path),std::exception);
    }
    {std::ofstream f(path); f << original << "\nrevision: 123\n";}
    EXPECT_THROW(RobotConfig::load(path),std::exception);
    {std::ofstream f(path); f << "[broken";}
    EXPECT_THROW(RobotConfig::load(path),std::exception);
    fs::remove(path);
    EXPECT_THROW(RobotConfig::load(path),std::exception);
}
TEST_F(ConfigTest, UnreadableConfigurationFailsWithoutReinitialization) {
    if (geteuid() == 0) GTEST_SKIP() << "permission test needs unprivileged user";
    const auto saved=bytes();
    fs::permissions(path, fs::perms::none);
    EXPECT_THROW(RobotConfig::load(path),std::exception);
    EXPECT_THROW(update_disabled_legs(path,"set",{"L1"}),std::exception);
    fs::permissions(path,fs::perms::owner_read|fs::perms::owner_write);
    EXPECT_EQ(bytes(),saved);
}
TEST_F(ConfigTest, LocksRejectChangesAndConcurrentMotionUntilStopped) {
    {
        MotionSession session(Stage::Calibration,path);
        const auto saved=bytes();
        EXPECT_THROW(update_disabled_legs(path,"set",{"L1"}),std::exception);
        EXPECT_EQ(bytes(),saved);
        EXPECT_THROW(MotionSession(Stage::Calibration,path),std::exception);
    }
    EXPECT_TRUE(update_disabled_legs(path,"set",{"L1"}));
}
TEST_F(ConfigTest, IndependentWriterLockAndReaderSnapshot) {
    int ready[2], release[2]; ASSERT_EQ(pipe(ready),0);ASSERT_EQ(pipe(release),0);
    const pid_t child=fork(); ASSERT_GE(child,0);
    if(child==0) {
        close(ready[0]);close(release[1]);
        try {PathLock lock(path.string()+".lock");char signal='x';::write(ready[1],&signal,1);read(release[0],&signal,1);_exit(0);}
        catch(...) {_exit(2);}
    }
    close(ready[1]);close(release[0]);char signal; ASSERT_EQ(read(ready[0],&signal,1),1);
    EXPECT_THROW(update_disabled_legs(path,"set",{"L1"}),std::exception);
    EXPECT_TRUE(RobotConfig::load(path).disabled_legs().empty());
    ASSERT_EQ(::write(release[1],"x",1),1);int status=0;waitpid(child,&status,0);
    EXPECT_EQ(status,0);close(ready[0]);close(release[1]);
}
TEST_F(ConfigTest, ConfigurationChangesAndNewAttemptsInvalidateStageReceipts) {
    EXPECT_THROW(MotionSession(Stage::Standing,path),std::exception);
    EXPECT_THROW(MotionSession(Stage::Tripod,path),std::exception);
    {MotionSession c(Stage::Calibration,path);c.complete();}
    {MotionSession s(Stage::Standing,path);s.complete();}
    {MotionSession t(Stage::Tripod,path);}
    update_disabled_legs(path,"set",{"L1","L3"});
    EXPECT_THROW(MotionSession(Stage::Standing,path),std::exception);
    EXPECT_THROW(MotionSession(Stage::Tripod,path),std::exception);
    {MotionSession c(Stage::Calibration,path);c.complete();}
    {MotionSession s(Stage::Standing,path);s.complete();}
    {MotionSession c(Stage::Calibration,path);}
    EXPECT_THROW(MotionSession(Stage::Standing,path),std::exception);
}
TEST_F(ConfigTest, ManualEditsCannotReuseCalibrationAtSameRevision) {
    {MotionSession c(Stage::Calibration,path);c.complete();}
    change([](auto& d){d["parameters"]["rinbo_cali"]["max_pwm"]=70.0;});
    EXPECT_THROW(MotionSession(Stage::Standing,path),std::exception);
}
TEST_F(ConfigTest, AllDisabledCanBeSavedButNoMotionStageCanStart) {
    update_disabled_legs(path,"set",{"L1","L2","L3","R1","R2","R3"});
    EXPECT_TRUE(RobotConfig::load(path).enabled_legs().empty());
    EXPECT_THROW(MotionSession(Stage::Calibration,path),std::exception);
    EXPECT_THROW(MotionSession(Stage::Standing,path),std::exception);
    EXPECT_THROW(MotionSession(Stage::Tripod,path),std::exception);
    EXPECT_THROW(MotionSession(Stage::Manual,path),std::exception);
}
TEST_F(ConfigTest, ManualRequiresCalibrationAndInvalidatesStandingWithoutApprovingIt) {
    EXPECT_THROW(MotionSession(Stage::Manual,path),std::exception);
    {MotionSession c(Stage::Calibration,path);c.complete();}
    {MotionSession s(Stage::Standing,path);s.complete();}
    {MotionSession m(Stage::Manual,path);m.complete();}
    EXPECT_THROW(MotionSession(Stage::Tripod,path),std::exception);
    EXPECT_NO_THROW(MotionSession(Stage::Manual,path));
}
TEST_F(ConfigTest, ScopedReceiptsCoverOnlySelectedLegsAndNeverApproveWholeRobot) {
    update_disabled_legs(path,"set",{"L1"});
    { MotionSession c(Stage::Calibration,path,{"L2","R2"}); c.complete(); }
    const auto config = RobotConfig::load(path);
    EXPECT_NO_THROW(check_manual_readiness(config,{"L2"}));
    EXPECT_NO_THROW(check_manual_readiness(config,{"R2","L2"}));
    EXPECT_THROW(check_manual_readiness(config,{"L3"}),std::exception);
    EXPECT_THROW(check_manual_readiness(config,{"L1"}),std::exception);
    EXPECT_THROW(check_manual_readiness(config),std::exception);
    EXPECT_NO_THROW(MotionSession(Stage::Manual,path,{"L2"}));
    EXPECT_THROW(MotionSession(Stage::Standing,path),std::exception);
    EXPECT_THROW(MotionSession(Stage::Tripod,path),std::exception);
    EXPECT_THROW(MotionSession(Stage::Calibration,path,{"L1"}),std::exception);
    EXPECT_THROW(MotionSession(Stage::Calibration,path,{"L2","L2"}),std::exception);
    EXPECT_THROW(MotionSession(Stage::Standing,path,{"L2"}),std::exception);
    { MotionSession c(Stage::Calibration,path,{"R1"}); }
    EXPECT_THROW(check_manual_readiness(config,{"L2"}),std::exception);
    EXPECT_THROW(check_manual_readiness(config,{"R1"}),std::exception);
}
TEST_F(ConfigTest, ReadinessCheckDoesNotChangeOrCreateStageReceipts) {
    EXPECT_THROW(check_manual_readiness(RobotConfig::load(path)),std::exception);
    EXPECT_FALSE(fs::exists(path.string()+".calibration.json"));
    {MotionSession c(Stage::Calibration,path);c.complete();}
    {MotionSession s(Stage::Standing,path);s.complete();}
    const auto before = fs::last_write_time(path.string()+".calibration.json");
    EXPECT_NO_THROW(check_manual_readiness(RobotConfig::load(path)));
    EXPECT_EQ(fs::last_write_time(path.string()+".calibration.json"),before);
    EXPECT_NO_THROW(MotionSession(Stage::Tripod,path));
    change([](auto& d){d["revision"]=99;});
    EXPECT_THROW(check_manual_readiness(RobotConfig::load(path)),std::exception);
}
TEST_F(ConfigTest, ProcessInspectionDistinguishesMotionFromCommunication) {
    const auto proc=dir/"proc";
    fs::create_directories(proc/"123");
    {std::ofstream f(proc/"123"/"cmdline",std::ios::binary);f << "/bin/rinbo_ros2_bridge" << '\0';}
    EXPECT_NO_THROW(assert_no_motion_processes(proc));
    {std::ofstream f(proc/"123"/"cmdline",std::ios::binary);f << "/old/build/rinbo_cali" << '\0';}
    EXPECT_THROW(assert_no_motion_processes(proc),std::exception);
    {std::ofstream f(proc/"123"/"cmdline",std::ios::binary);f << "/bin/renamed" << '\0';}
    fs::create_symlink("/old/build/rinbo_cali (deleted)",proc/"123"/"exe");
    EXPECT_THROW(assert_no_motion_processes(proc),std::exception);
    fs::remove(proc/"123"/"exe");
    {std::ofstream f(proc/"123"/"comm");f << "rinbo_tripod\n";}
    EXPECT_THROW(assert_no_motion_processes(proc),std::exception);
    fs::remove(proc/"123"/"comm");
    {std::ofstream f(proc/"123"/"cmdline",std::ios::binary);f << "biorola_power_tool" << '\0' << "--check-config" << '\0';}
    EXPECT_THROW(assert_no_motion_processes(proc),std::exception);
    fs::remove(proc/"123"/"cmdline");
    EXPECT_THROW(assert_no_motion_processes(proc),std::exception);
}
} // namespace

namespace {
TEST_F(ConfigTest, TripodTuningPreservesOnlyValidUnchangedPrerequisites) {
    const auto proc=dir/"empty-proc";fs::create_directory(proc);
    {MotionSession c(Stage::Calibration,path);c.complete();}
    {MotionSession s(Stage::Standing,path);s.complete();}
    const auto before=RobotConfig::load(path);
    const auto result=tune_tripod(path,{{"startup_duration",30},{"start_ratio",80},{"target_ratio",80},{"kp",.08}},false,proc);
    EXPECT_NE(result.find("\"calibration_valid\":true"),std::string::npos);
    const auto after=RobotConfig::load(path);
    EXPECT_EQ(after.revision(),before.revision()+1);
    EXPECT_EQ(after.parameters("rinbo_cali"),before.parameters("rinbo_cali"));
    EXPECT_EQ(after.parameters("rinbo_standing"),before.parameters("rinbo_standing"));
    EXPECT_EQ(after.disabled_legs(),before.disabled_legs());
    EXPECT_NO_THROW(MotionSession t(Stage::Tripod,path));
    for(const auto& p:before.parameters("rinbo_tripod_rslip")){
        if(p.get_name().find("safety.")!=0 && p.get_name()!="max_pwm")continue;
        const auto& a=after.parameters("rinbo_tripod_rslip");
        auto found=std::find_if(a.begin(),a.end(),[&](const auto& v){return v.get_name()==p.get_name();});
        ASSERT_NE(found,a.end());EXPECT_EQ(*found,p);
    }
    const auto unchanged=bytes();tune_tripod(path,{{"kp",.08}},false,proc);EXPECT_EQ(bytes(),unchanged);
}
TEST_F(ConfigTest, TripodTuningPreviewIsReadOnlyEvenDuringMotion) {
    const auto proc=dir/"empty-proc";fs::create_directory(proc);
    {MotionSession c(Stage::Calibration,path);c.complete();}
    {MotionSession s(Stage::Standing,path);s.complete();}
    MotionSession active(Stage::Tripod,path);
    const auto before=bytes();
    EXPECT_NO_THROW(tune_tripod(path,{{"startup_duration",20}},true,proc));
    EXPECT_EQ(bytes(),before);
    EXPECT_THROW(tune_tripod(path,{{"startup_duration",20}},false,proc),std::exception);
}
TEST_F(ConfigTest, TripodTuningNeverCreatesMissingOrInvalidReceipts) {
    const auto proc=dir/"empty-proc";fs::create_directory(proc);
    EXPECT_NE(tune_tripod(path,{{"startup_duration",20}},false,proc).find("\"calibration_valid\":false"),std::string::npos);
    EXPECT_THROW(MotionSession t(Stage::Tripod,path),std::exception);
    {MotionSession c(Stage::Calibration,path);c.complete();}
    EXPECT_NE(tune_tripod(path,{{"startup_duration",21}},false,proc).find("\"standing_valid\":false"),std::string::npos);
    EXPECT_NO_THROW(check_manual_readiness(RobotConfig::load(path)));
    EXPECT_THROW(MotionSession t(Stage::Tripod,path),std::exception);
    {MotionSession s(Stage::Standing,path);s.complete();}
    {auto r=YAML::LoadFile(path.string()+".calibration.json");r["boot_id"]="other-boot";
     std::ofstream out(path.string()+".calibration.json");out<<r;}
    tune_tripod(path,{{"startup_duration",22}},false,proc);
    EXPECT_THROW(MotionSession t(Stage::Tripod,path),std::exception);
}
TEST_F(ConfigTest, TripodTuningRejectsProtectedKeysInvalidNumbersAndActiveWriters) {
    const auto proc=dir/"proc";fs::create_directories(proc/"123");
    {std::ofstream out(proc/"123"/"cmdline",std::ios::binary);out<<"/bin/rinbo_tripod"<<'\0';}
    const auto before=bytes();
    EXPECT_THROW(tune_tripod(path,{{"startup_duration",20}},false,proc),std::exception);
    for(const auto& key:{"max_pwm","safety.max_position_error_counts","hardware.disabled_legs","unknown"})
        EXPECT_THROW(tune_tripod(path,{{key,80}},true,proc),std::exception);
    EXPECT_THROW(tune_tripod(path,{{"startup_duration",0}},true,proc),std::exception);
    EXPECT_THROW(tune_tripod(path,{{"start_ratio",2},{"target_ratio",8}},true,proc),std::exception);
    EXPECT_THROW(tune_tripod(path,{{"kp",std::numeric_limits<double>::quiet_NaN()}},true,proc),std::exception);
    EXPECT_EQ(bytes(),before);
}
TEST_F(ConfigTest, TripodFilterAndRestoredGainsKeepOtherStagesAndRejectInvalidFilter) {
    const auto proc=dir/"empty_proc";fs::create_directories(proc);
    const auto before=YAML::Load(bytes());
    const auto legacy=RobotConfig::load(path);
    bool legacy_filter_found=false;
    for(const auto& p:legacy.parameters("rinbo_tripod_rslip"))
        if(p.get_name()=="velocity_filter_time_constant_s") {
            legacy_filter_found=true;EXPECT_DOUBLE_EQ(p.as_double(),.02);
        }
    EXPECT_TRUE(legacy_filter_found);
    tune_tripod(path,{{"kp",.38},{"kd",.003},{"friction_pwm",0},
        {"target_ratio",1},{"ratio_step",-.0002},{"velocity_filter_time_constant_s",.005}},false,proc);
    const auto after=YAML::Load(bytes());
    for(const auto* node:{"rinbo_cali","rinbo_standing"})
        EXPECT_EQ(YAML::Dump(before["parameters"][node]),YAML::Dump(after["parameters"][node]));
    EXPECT_EQ(YAML::Dump(before["disabled_legs"]),YAML::Dump(after["disabled_legs"]));
    EXPECT_DOUBLE_EQ(after["parameters"]["rinbo_tripod_rslip"]["kd"].as<double>(),.003);
    EXPECT_NO_THROW(tune_tripod(path,{{"velocity_filter_time_constant_s",0}},true,proc));
    for(double value:{-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
        EXPECT_THROW(tune_tripod(path,{{"velocity_filter_time_constant_s",value}},true,proc),std::exception);
}
} // namespace

namespace {
TEST_F(ConfigTest, TripodPositionPolicyIsExplicitReversibleAndKeepsOtherControllers) {
    const auto proc=dir/"empty-proc";fs::create_directory(proc);
    {MotionSession c(Stage::Calibration,path);c.complete();}
    {MotionSession s(Stage::Standing,path);s.complete();}
    const auto before=RobotConfig::load(path);const auto original_bytes=bytes();
    EXPECT_NE(set_tripod_position_policy(path,false,true,proc).find("warn_only"),std::string::npos);
    EXPECT_EQ(bytes(),original_bytes);
    EXPECT_NE(set_tripod_position_policy(path,false,false,proc).find("warn_only"),std::string::npos);
    auto after=RobotConfig::load(path);
    EXPECT_EQ(after.parameters("rinbo_cali"),before.parameters("rinbo_cali"));
    EXPECT_EQ(after.parameters("rinbo_standing"),before.parameters("rinbo_standing"));
    EXPECT_EQ(after.disabled_legs(),before.disabled_legs());
    EXPECT_NO_THROW(MotionSession t(Stage::Tripod,path));
    const auto unchanged=bytes();set_tripod_position_policy(path,false,false,proc);EXPECT_EQ(bytes(),unchanged);
    set_tripod_position_policy(path,true,false,proc);
    EXPECT_NO_THROW(MotionSession t(Stage::Tripod,path));
    const auto restored=RobotConfig::load(path);
    EXPECT_EQ(restored.parameters("rinbo_tripod_rslip"),before.parameters("rinbo_tripod_rslip"));
}
TEST_F(ConfigTest, TripodPositionPolicyCannotBypassBusyOrRecreateInvalidReceipts) {
    const auto proc=dir/"empty-proc";fs::create_directory(proc);
    {
        MotionSession c(Stage::Calibration,path);
        EXPECT_THROW(set_tripod_position_policy(path,false,false,proc),std::exception);
    }
    EXPECT_NE(set_tripod_position_policy(path,false,false,proc).find("\"calibration_valid\":false"),std::string::npos);
    EXPECT_THROW(MotionSession t(Stage::Tripod,path),std::exception);
}
} // namespace

TEST_F(ConfigTest, MotionLimitsAreAtomicTypedRevisionCheckedAndPreserveUnrelatedSettings) {
    const auto proc = dir/"proc"; fs::create_directory(proc);
    const auto before = RobotConfig::load(path);
    const auto original = bytes();
    const auto rev = before.revision();
    EXPECT_NO_THROW(tune_limits(path,"standing",{{"position_tolerance_counts",1000},{"rotate_timeout_s",60}},true,rev,proc));
    EXPECT_EQ(bytes(),original);
    EXPECT_THROW(tune_limits(path,"standing",{{"rotate_timeout_s",60}},false,rev+1,proc),std::exception);
    EXPECT_EQ(bytes(),original);
    tune_limits(path,"standing",{{"position_tolerance_counts",1000},{"rotate_timeout_s",60}},false,rev,proc);
    const auto after = RobotConfig::load(path);
    EXPECT_EQ(after.revision(),rev+1);
    EXPECT_EQ(after.parameters("rinbo_cali"),before.parameters("rinbo_cali"));
    EXPECT_EQ(after.parameters("rinbo_tripod_rslip"),before.parameters("rinbo_tripod_rslip"));
    EXPECT_EQ(after.disabled_legs(),before.disabled_legs());
    const auto unchanged = bytes();
    tune_limits(path,"standing",{{"position_tolerance_counts",1000}},false,rev+1,proc);
    EXPECT_EQ(bytes(),unchanged);
    for (const auto& entry : std::map<std::string,double>{{"max_current",10},{"rotate_timeout_s",601},{"position_tolerance_counts",0},{"settle_time_s",-1},{"hold_error_counts",500}})
        EXPECT_THROW(tune_limits(path,"standing",{{entry.first,entry.second}},false,-1,proc),std::exception);
    EXPECT_THROW(tune_limits(path,"standing",{{"rotate_timeout_s",std::numeric_limits<double>::quiet_NaN()}},true,-1,proc),std::exception);
    EXPECT_EQ(bytes(),unchanged);
    EXPECT_NO_THROW(tune_limits(path,"calibration",{{"stop_timeout_s",600}},true,-1,proc));
}
TEST_F(ConfigTest, MotionLimitsCarryOnlyUnchangedPrerequisitesAndCannotEditWhileBusy) {
    const auto proc = dir/"proc"; fs::create_directory(proc);
    { MotionSession cal(Stage::Calibration,path);cal.complete(); }
    { MotionSession stand(Stage::Standing,path);stand.complete(); }
    tune_limits(path,"standing",{{"position_tolerance_counts",1000}},false,-1,proc);
    EXPECT_NO_THROW(MotionSession(Stage::Standing,path));
    EXPECT_THROW(MotionSession(Stage::Tripod,path),std::exception);
    { MotionSession stand(Stage::Standing,path);stand.complete(); }
    tune_limits(path,"tripod",{{"stop_on_position_error",0},{"position_error_trip_samples",5}},false,-1,proc);
    EXPECT_NO_THROW(MotionSession(Stage::Tripod,path));
    EXPECT_THROW(tune_limits(path,"tripod",{{"stop_on_position_error",.5}},true,-1,proc),std::exception);
    EXPECT_THROW(tune_limits(path,"tripod",{{"position_error_trip_samples",1.5}},true,-1,proc),std::exception);
    {
        MotionSession active(Stage::Standing,path);
        EXPECT_NO_THROW(tune_limits(path,"standing",{{"rotate_timeout_s",60}},true,-1,proc));
        EXPECT_THROW(tune_limits(path,"standing",{{"rotate_timeout_s",60}},false,-1,proc),std::exception);
    }
    fs::create_directory(proc/"123");
    {std::ofstream out(proc/"123"/"cmdline",std::ios::binary);out<<"/bin/rinbo_tripod"<<'\0';}
    EXPECT_THROW(tune_limits(path,"standing",{{"rotate_timeout_s",60}},false,-1,proc),std::exception);
    fs::remove_all(proc/"123");
    tune_limits(path,"calibration",{{"stop_timeout_s",15}},false,-1,proc);
    EXPECT_THROW(MotionSession(Stage::Standing,path),std::exception);
}

TEST_F(ConfigTest, TripodRawPwm3300IsTypedAtomicAndDoesNotRaiseOtherControllerCaps) {
    const auto proc=dir/"proc";fs::create_directory(proc);
    {MotionSession c(Stage::Calibration,path);c.complete();}
    {MotionSession s(Stage::Standing,path);s.complete();}
    const auto before=RobotConfig::load(path);const auto initial=bytes();
    EXPECT_NO_THROW(tune_limits(path,"tripod",{{"max_pwm",3300}},true,before.revision(),proc));
    EXPECT_EQ(bytes(),initial);
    tune_limits(path,"tripod",{{"max_pwm",3300}},false,before.revision(),proc);
    const auto after=RobotConfig::load(path);
    EXPECT_EQ(after.parameters("rinbo_cali"),before.parameters("rinbo_cali"));
    EXPECT_EQ(after.parameters("rinbo_standing"),before.parameters("rinbo_standing"));
    EXPECT_NO_THROW(MotionSession(Stage::Tripod,path));
    bool found=false;
    for(const auto& p:after.parameters("rinbo_tripod_rslip")) if(p.get_name()=="max_pwm") {EXPECT_EQ(p.as_double(),3300);found=true;}
    EXPECT_TRUE(found);
    const auto saved=bytes();
    for(double v:{0.,3300.1,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
        EXPECT_THROW(tune_limits(path,"tripod",{{"max_pwm",v}},false,-1,proc),std::exception);
    EXPECT_THROW(tune_limits(path,"standing",{{"max_pwm",3300}},false,-1,proc),std::exception);
    EXPECT_EQ(bytes(),saved);
    change([](auto& d){d["parameters"]["rinbo_standing"]["max_pwm"]=3300.0;});
    EXPECT_THROW(RobotConfig::load(path),std::exception);
}

TEST_F(ConfigTest, TripodSlewIsOptionalAndOnlyTripodSettingsChange) {
    const auto proc=dir/"proc";fs::create_directory(proc);
    const auto before=RobotConfig::load(path);
    const auto old=YAML::LoadFile(path);
    tune_limits(path,"tripod",{{"enable_pwm_slew_limit",0}},false,before.revision(),proc);
    const auto after=YAML::LoadFile(path);
    EXPECT_FALSE(after["parameters"]["rinbo_tripod_rslip"]["safety"]["enable_pwm_slew_limit"].as<bool>());
    EXPECT_EQ(YAML::Dump(old["parameters"]["rinbo_cali"]),YAML::Dump(after["parameters"]["rinbo_cali"]));
    EXPECT_EQ(YAML::Dump(old["parameters"]["rinbo_standing"]),YAML::Dump(after["parameters"]["rinbo_standing"]));
    EXPECT_EQ(YAML::Dump(old["disabled_legs"]),YAML::Dump(after["disabled_legs"]));
    EXPECT_NO_THROW(tune_limits(path,"tripod",{{"enable_pwm_slew_limit",1},{"pwm_slew_rate_per_sec",2000}},true,-1,proc));
    for(double v:{-.1,.5,2.}) EXPECT_THROW(tune_limits(path,"tripod",{{"enable_pwm_slew_limit",v}},true,-1,proc),std::exception);
    EXPECT_THROW(tune_limits(path,"tripod",{{"pwm_slew_rate_per_sec",0}},true,-1,proc),std::exception);
    EXPECT_THROW(tune_limits(path,"standing",{{"enable_pwm_slew_limit",0}},true,-1,proc),std::exception);
    EXPECT_THROW(tune_limits(path,"tripod",{{"stop_on_power_stale",0}},true,-1,proc),std::exception);
}

TEST_F(ConfigTest, AcceptedChoicesKeepManualControlIndependentAndSharedGuardsIntact) {
    fs::copy_file(fs::path(RINBO_FSM_SOURCE_DIR)/"test/fixtures/robot_restore_choices.yaml",path,fs::copy_options::overwrite_existing);
    const auto before=RobotConfig::load(path);
    auto number=[](const RobotConfig& cfg,const std::string& node,const std::string& key) {
        for (const auto& p:cfg.parameters(node)) if(p.get_name()==key) return p.as_double();
        throw std::runtime_error("missing parameter");
    };
    for (const auto& node:{"rinbo_cali","rinbo_standing","rinbo_manual"}) {
        EXPECT_DOUBLE_EQ(number(before,node,"max_pwm"),500);
        EXPECT_DOUBLE_EQ(number(before,node,"kp"),.35);
        EXPECT_DOUBLE_EQ(number(before,node,"kd"),.002);
        EXPECT_DOUBLE_EQ(number(before,node,"k_ff"),.02);
        EXPECT_DOUBLE_EQ(number(before,node,"friction_pwm"),0);
        EXPECT_DOUBLE_EQ(number(before,node,"velocity_filter_time_constant_s"),.005);
    }
    EXPECT_EQ(before.disabled_legs(),(std::vector<std::string>{"L3"}));
    EXPECT_DOUBLE_EQ(number(before,"rinbo_standing","safety.settle_time_s"),0);
    change([](auto& d){d["parameters"]["rinbo_standing"]["kp"]=.12;d["parameters"]["rinbo_standing"]["max_pwm"]=100.;});
    const auto after=RobotConfig::load(path);
    EXPECT_EQ(before.parameters("rinbo_manual"),after.parameters("rinbo_manual"));
    EXPECT_EQ(before.parameters("rinbo_tripod_rslip"),after.parameters("rinbo_tripod_rslip"));
    EXPECT_DOUBLE_EQ(number(after,"rinbo_standing","kp"),.12);
    change([](auto& d){d["parameters"]["rinbo_manual"]["max_pwm"]=501.;});
    EXPECT_THROW(RobotConfig::load(path),std::exception);
}
TEST_F(ConfigTest, LegacyManualInheritanceAndNewBlockValidation) {
    const auto legacy=RobotConfig::load(path);
    EXPECT_EQ(legacy.parameters("rinbo_manual"),legacy.parameters("rinbo_standing"));
    change([](auto& d){d["parameters"]["rinbo_manual"]["kp"]=.35;});
    EXPECT_THROW(RobotConfig::load(path),std::exception); // partial blocks cannot silently inherit control fields
}
