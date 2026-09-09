#include "motion_effort.hpp"
#include "control_velocity.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <deque>
#include <limits>

namespace {
const rinbo_fsm::MotionEffort suspended{0.08, 0.006, 0.005, 40.0, 153.6};
constexpr double counts_per_degree = 153.6;

struct Result { double ripple; double mean_speed; double stopped_fraction; double max_error; };
// An illustrative delayed plant with static/kinetic friction, NOT a hardware
// identification result. Vary delay and friction to check a failure mechanism,
// rather than declaring the real robot smooth from ideal trajectory feedback.
Result simulate(const rinbo_fsm::MotionEffort& profile, int delay_ms,
                double kinetic_friction = 40, double speed = 5) {
    double p=0, v=0, previous_q=0, output=0, target=0, sum=0, sum_sq=0, error=0;
    int count=0, stopped=0;
    constexpr double dt=.001;
    std::deque<double> delayed(delay_ms, 0.0);
    rinbo_fsm::ControlVelocity filter;
    for (int i=0; i<10000; ++i) {
        const double t=i*dt;
        const double reference_v=std::min(t,1.0)*speed;
        target += reference_v*dt;
        const double q=std::round(p*counts_per_degree)/counts_per_degree;
        const double measured=filter.update((q-previous_q)/dt,dt);
        previous_q=q;
        if (i%10==0) {
            const double requested=profile.command((target-q)*counts_per_degree,
                reference_v*counts_per_degree, measured*counts_per_degree);
            output=std::clamp(std::clamp(requested,-80.0,80.0),output-2.5,output+2.5);
        }
        delayed.push_back(output);
        const double applied=delayed.front(); delayed.pop_front();
        const double breakaway=kinetic_friction+8;
        if (std::abs(v)<.01 && std::abs(applied)<=breakaway) v=0;
        else {
            const double friction=std::copysign(kinetic_friction,std::abs(v)>=.01?v:applied);
            double next=v+dt*(applied-friction-.768*v)/.08;
            if (v*next<0 && std::abs(applied)<=breakaway) next=0;
            v=next;
        }
        p += v*dt;
        if (t>3) {
            ++count; sum+=v; sum_sq+=v*v;
            if (std::abs(v)<.01) ++stopped;
            error=std::max(error,std::abs(target-p));
        }
    }
    const double mean=sum/count;
    return {std::sqrt(std::max(0.0,sum_sq/count-mean*mean)),mean,double(stopped)/count,error};
}

TEST(MotionEffort, CompensationIsContinuousSymmetricAndZeroAtRest) {
    suspended.validate();
    EXPECT_DOUBLE_EQ(suspended.command(0,0,0),0);
    for (double v : {.001,1.0,153.6,768.0,5529.6}) {
        EXPECT_NEAR(suspended.feedforward(-v),-suspended.feedforward(v),1e-10);
        EXPECT_LE(std::abs(suspended.feedforward(v)-suspended.k_ff*v),40.0);
    }
    EXPECT_LT(std::abs(suspended.feedforward(.001)),.001);
    EXPECT_LT(suspended.command(0,0,100),0);  // Damping can still brake.
    EXPECT_LT(suspended.command(-1000,768,768),0); // Compensation is not a PWM floor.
}

TEST(MotionEffort, RejectsMalformedProfiles) {
    for (int field=0; field<5; ++field) {
        auto p=suspended;
        double* values[]{&p.kp,&p.kd,&p.k_ff,&p.friction_pwm,&p.friction_velocity_counts_s};
        *values[field]=std::numeric_limits<double>::quiet_NaN();
        EXPECT_THROW(p.validate(),std::invalid_argument);
    }
    auto p=suspended; p.friction_pwm=81; EXPECT_THROW(p.validate(),std::invalid_argument);
    p=suspended; p.friction_velocity_counts_s=0; EXPECT_THROW(p.validate(),std::invalid_argument);
}

TEST(MotionEffort, ReducesStopStartCyclesInDelayedFrictionModel) {
    for (int delay_ms : {20,30,40}) {
        const auto old=simulate({},delay_ms);
        const auto tuned=simulate(suspended,delay_ms);
        EXPECT_GT(old.stopped_fraction,.2);
        EXPECT_LT(tuned.stopped_fraction,.01);
        EXPECT_LT(tuned.ripple,old.ripple*.15);
        EXPECT_NEAR(tuned.mean_speed,5,.1);
    }
}

TEST(MotionEffort, TracksBothDirectionsAcrossFrictionAndSpeedVariations) {
    for (double friction : {36.0,40.0,44.0}) {
        for (double speed : {-36.0,-5.0,5.0,36.0}) {
            const auto tuned=simulate(suspended,30,friction,speed);
            EXPECT_NEAR(tuned.mean_speed,speed,.2);
            EXPECT_LT(tuned.ripple,.5);
            EXPECT_LT(tuned.max_error,2.0);
            EXPECT_LT(tuned.stopped_fraction,.01);
        }
    }
}
} // namespace
