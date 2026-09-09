#include "robot_config.hpp"
#include <gtest/gtest.h>
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
        [](auto& d){d["parameters"]["rinbo_cali"]["max_pwm"]=81.0;},
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
