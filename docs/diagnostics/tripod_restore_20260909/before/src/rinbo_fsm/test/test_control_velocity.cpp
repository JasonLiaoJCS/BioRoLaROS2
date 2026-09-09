#include "control_velocity.hpp"
#include <gtest/gtest.h>
#include <limits>

TEST(ControlVelocity, ReducesEncoderPacketStepsWithoutLosingMeanSpeed) {
    rinbo_fsm::ControlVelocity filter;
    double raw_variance = 0, filtered_variance = 0, sum = 0;
    // A steady 5 deg/s encoder delivered in 5 ms steps to a 1 kHz consumer.
    // The raw derivative alternates 0/25 despite the constant mean speed.
    for (int i = 0; i < 2000; ++i) {
        const double raw = i % 5 == 0 ? 25.0 : 0.0;
        const double v = filter.update(raw, .001);
        if (i >= 1000) {
            raw_variance += (raw-5)*(raw-5);
            filtered_variance += (v-5)*(v-5);
            sum += v;
        }
    }
    EXPECT_NEAR(sum/1000, 5, .01);
    EXPECT_LT(filtered_variance, raw_variance * .02);
}

TEST(ControlVelocity, TracksStoppingAndReversingAtDifferentFeedbackRates) {
    for (double dt : {.001, .005, .01}) {
        rinbo_fsm::ControlVelocity filter;
        filter.update(36, dt);
        double v = 36;
        for (int i=0; i<static_cast<int>(.1/dt); ++i) v=filter.update(0, dt);
        EXPECT_LT(std::abs(v), 1);
        for (int i=0; i<static_cast<int>(.1/dt); ++i) v=filter.update(-36, dt);
        EXPECT_NEAR(v, -36, 1);
    }
}

TEST(ControlVelocity, DoesNotHideInvalidFeedbackOrCarryHistoryAcrossGaps) {
    rinbo_fsm::ControlVelocity filter;
    filter.update(100, .01);
    EXPECT_TRUE(std::isnan(filter.update(std::numeric_limits<double>::quiet_NaN(), .01)));
    EXPECT_DOUBLE_EQ(filter.update(0, .01), 0);
    filter.update(100, .01);
    EXPECT_DOUBLE_EQ(filter.update(-10, .3), -10);
}
