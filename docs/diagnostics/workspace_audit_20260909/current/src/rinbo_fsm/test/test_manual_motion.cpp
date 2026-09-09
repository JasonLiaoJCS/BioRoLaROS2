#include "manual_motion.hpp"
#include "rinbo_msgs/msg/motor_cmd_stamped.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace rinbo_manual;

TEST(ManualPlan, DefaultsAndOmittedLegsAreOff) {
    auto p = parse("duration_s: 3.0\nlegs: {L2: {mode: position, angle_deg: 5.0}}\n");
    EXPECT_EQ(p.legs[0].mode, Mode::Off);
    EXPECT_EQ(p.legs[1].mode, Mode::Position);
    EXPECT_EQ(p.max_pwm, 80);
}
TEST(ManualPlan, RejectsMalformedAmbiguousAndUnsafeInput) {
    for (const auto& text : {
        "duration_s: 0\nlegs: {L2: {mode: position, angle_deg: 5}}",
        "duration_s: 61\nlegs: {L2: {mode: position, angle_deg: 5}}",
        "duration_s: 3\nmax_pwm: .nan\nlegs: {L2: {mode: position, angle_deg: 5}}",
        "duration_s: 3\nmax_pwm: 81\nlegs: {L2: {mode: position, angle_deg: 5}}",
        "duration_s: 3\nlegs: {L2: {mode: velocity, speed_deg_s: 31}}",
        "duration_s: 3\nlegs: {L2: {mode: velocity, speed_deg_s: -31}}",
        "duration_s: 3\nlegs: {L2: {mode: cycle, speed_deg_s: 10}}",
        "duration_s: 3\nlegs: {L2: {mode: off, angle_deg: 5}}",
        "duration_s: 3\nlegs: {L2: {mode: position, angle_deg: .inf}}",
        "duration_s: 3\nlegs: {L2: {mode: position, angle_deg: '5'}}",
        "duration_s: 3\nlegs: {L2: {mode: position, angle_deg: 5, angle_deg: 6}}",
        "duration_s: 3\nlegs: {L2: {mode: velocity, speed_deg_s: 5, phase_deg: 0}}",
        "duration_s: 3\nlegs: {X1: {mode: off}}",
        "duration_s: 3\nlegs: {L2: {mode: off}}",
        "duration_s: 3\nlegs: {}",
        "duration_s: 3\nduration_s: 4\nlegs: {}",
        "duration_s: 3\nlegs: {L2: {mode: off}, L2: {mode: off}}",
        "---\nduration_s: 3\nlegs: {}\n---\nlegs: {}"}) {
        SCOPED_TRACE(text);
        EXPECT_THROW(parse(text), std::exception);
    }
}
TEST(ManualTrajectory, AngleWrapAndEncoderSignMatchCalibration) {
    EXPECT_DOUBLE_EQ(nearest_angle(359, 1), 361);
    EXPECT_DOUBLE_EQ(nearest_angle(-359, -1), -361);
    EXPECT_DOUBLE_EQ(nearest_angle(0, -180), 180);
    EXPECT_DOUBLE_EQ(encoder_degrees(0, -55296), 360);
    EXPECT_DOUBLE_EQ(encoder_degrees(3, 55296), 360);
}
TEST(ManualTrajectory, RelativeMoveIsSmallEvenFarFromTheCalibrationZero) {
    auto p = parse("duration_s: 3\nlegs: {L2: {mode: relative, move_deg: 5}}\n");
    std::array<double,6> initial{}; initial[1] = 178.0;
    Trajectory t(p, initial);
    EXPECT_DOUBLE_EQ(t.sample(0).position[1], 178.0);
    EXPECT_DOUBLE_EQ(t.sample(t.total_seconds()).position[1], 183.0);
    p.legs[1].angle = -5;
    Trajectory back(p, initial);
    EXPECT_DOUBLE_EQ(back.sample(back.total_seconds()).position[1], 173.0);
    EXPECT_THROW(parse("duration_s: 3\nlegs: {L2: {mode: relative, move_deg: 31}}"),std::exception);
    EXPECT_THROW(parse("duration_s: 3\nlegs: {L2: {mode: relative, angle_deg: 5}}"),std::exception);
}
TEST(ManualTrajectory, AlignmentAndRampsRespectVelocityAccelerationAndContinuity) {
    auto p = parse("duration_s: 3\nlegs: {L2: {mode: cycle, speed_deg_s: -30, phase_deg: 180}}\n");
    Trajectory t(p, {});
    const double h = 0.001;
    auto previous = t.sample(0);
    EXPECT_EQ(previous.position[1], 0);
    EXPECT_EQ(previous.velocity[1], 0);
    for (double now = h; now < t.total_seconds(); now += h) {
        const auto q = t.sample(now);
        EXPECT_LE(std::abs(q.velocity[1]), p.max_speed + 1e-6);
        EXPECT_LE(std::abs(q.velocity[1]-previous.velocity[1])/h, p.acceleration + 1e-3);
        EXPECT_NEAR((q.position[1]-previous.position[1])/h,
                    (q.velocity[1]+previous.velocity[1])/2, 0.001);
        previous = q;
    }
    const auto end = t.sample(t.total_seconds());
    EXPECT_TRUE(end.done);
    EXPECT_NEAR(end.velocity[1], 0, 1e-9);
    EXPECT_NEAR(end.position[1], 180.0 - 30.0*4.0, 1e-9);
}
TEST(ManualTrajectory, SharedClockPreservesPhaseAndVelocityModeStartsFromFeedback) {
    auto p = parse("duration_s: 5\nlegs:\n  L2: {mode: cycle, speed_deg_s: 20, phase_deg: 0}\n  R2: {mode: cycle, speed_deg_s: 20, phase_deg: 180}\n  R3: {mode: velocity, speed_deg_s: -10}\n");
    std::array<double,6> initial{0, 359, 0, 0, 182, 200};
    Trajectory t(p, initial);
    EXPECT_DOUBLE_EQ(t.sample(0).position[5], 200);
    for (double now = t.alignment_seconds(); now < t.total_seconds(); now += 0.01) {
        const auto q = t.sample(now);
        EXPECT_NEAR(std::abs(std::remainder(q.position[4]-q.position[1],360)), 180, 1e-9);
        EXPECT_NEAR(q.velocity[4], q.velocity[1], 1e-9);
    }
    EXPECT_THROW(t.sample(-1), std::exception);
    EXPECT_THROW(t.sample(std::numeric_limits<double>::quiet_NaN()), std::exception);
}
TEST(ManualOutput, EveryMaskAndStopDisablesAtFinalBoundary) {
    Plan p;
    for (auto& leg : p.legs) leg.mode = Mode::Velocity;
    for (unsigned bits = 0; bits < 64; ++bits) {
        std::array<bool,6> mask{};
        for (size_t i=0;i<6;++i) mask[i] = bits & (1U << i);
        if (bits) EXPECT_THROW(validate_mask(p, mask),std::exception);
        else EXPECT_NO_THROW(validate_mask(p, mask));
        for (bool active : {false,true}) {
            rinbo_msgs::msg::MotorCmdStamped cmd;
            set_outputs(cmd,p,mask,{100,-100,0,100,-100,0},20,active);
            const std::array<rinbo_msgs::msg::LegCmd,6> legs{cmd.l1,cmd.l2,cmd.l3,cmd.r1,cmd.r2,cmd.r3};
            EXPECT_EQ(cmd.servo_control_mode,0U);
            for(size_t i=0;i<6;++i) {
                EXPECT_FALSE(legs[i].reset_position);
                EXPECT_LE(legs[i].voltage,20);
                if(mask[i] || !active) {
                    EXPECT_FALSE(legs[i].enable); EXPECT_FALSE(legs[i].direction);
                    EXPECT_EQ(legs[i].voltage,0); EXPECT_EQ(legs[i].state,0U);
                } else EXPECT_TRUE(legs[i].enable);
            }
        }
    }
    rinbo_msgs::msg::MotorCmdStamped cmd;
    set_outputs(cmd,p,{}, {10,-10,0,10,-10,0},20,true);
    EXPECT_FALSE(cmd.l1.direction); EXPECT_TRUE(cmd.l2.direction);
    EXPECT_TRUE(cmd.r1.direction); EXPECT_FALSE(cmd.r2.direction);
    p.legs[0].mode = Mode::Off;
    set_outputs(cmd,p,{}, {10,0,0,0,0,0},20,true);
    EXPECT_FALSE(cmd.l1.enable);
}
