#include "motor_output_limits.hpp"
#include <gtest/gtest.h>
#include <limits>
TEST(MotorOutputLimits, Raw3300PassesWithoutScalingAndHigherValuesFail) {
    for (const char* leg : {"L1","L2","L3","R1","R2","R3"}) {
        EXPECT_FALSE(rinbo_bridge::validate_motor_output(3300,3300,leg));
        EXPECT_FALSE(rinbo_bridge::validate_motor_output(0,3300,leg));
        EXPECT_TRUE(rinbo_bridge::validate_motor_output(3300.01,3300,leg));
        EXPECT_TRUE(rinbo_bridge::validate_motor_output(-1,3300,leg));
        EXPECT_TRUE(rinbo_bridge::validate_motor_output(std::numeric_limits<double>::quiet_NaN(),3300,leg));
        EXPECT_TRUE(rinbo_bridge::validate_motor_output(std::numeric_limits<double>::infinity(),3300,leg));
    }
}
TEST(MotorOutputLimits, ExplicitOlderTransportCapStillEnforcesItsOwnValue) {
    EXPECT_FALSE(rinbo_bridge::validate_motor_output(80,80,"L2"));
    EXPECT_TRUE(rinbo_bridge::validate_motor_output(81,80,"L2"));
    EXPECT_TRUE(rinbo_bridge::validate_motor_output(3300,80,"L2"));
}
TEST(MotorOutputLimits, InvalidTransportConfigurationNeverApprovesOutput) {
    for (double cap : {0.,-1.,3301.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_FALSE(rinbo_bridge::valid_motor_output_cap(cap));
        EXPECT_TRUE(rinbo_bridge::validate_motor_output(0,cap,"L2"));
    }
    EXPECT_TRUE(rinbo_bridge::valid_motor_output_cap(3300));
}
