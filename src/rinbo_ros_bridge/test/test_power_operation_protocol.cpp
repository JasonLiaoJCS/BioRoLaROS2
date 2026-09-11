#include <gtest/gtest.h>
#include "power_operation_protocol.hpp"
using P=rinbo_ros_bridge::PowerOperationProtocol;
TEST(PowerOperation, ParseRequiresEpochGenerationAndRequest) {
    EXPECT_FALSE(P::parse("redrhex_power")); EXPECT_FALSE(P::parse("P2|b|-1|r"));
    EXPECT_FALSE(P::parse("P2|b|0|bad id"));
    auto r=P::parse("P2|b|2|request-1"); ASSERT_TRUE(r); EXPECT_EQ(r->generation,2U);
}
TEST(PowerOperation, OffPreemptsAllOldInflightStagesAndRetries) {
    P p;p.epoch="b";P::Request old{"b","on",0},off{"b","off",0};
    p.accepted(old,3,1);p.off(off,1);EXPECT_EQ(p.generation,1U);
    ASSERT_TRUE(p.fence(old));p.off(off,2);EXPECT_EQ(p.generation,1U);
    P::Request newer{"b","new",1};EXPECT_FALSE(p.fence(newer));
    p.accepted(newer,1,3);p.off(off,3);EXPECT_EQ(p.generation,2U);
}
TEST(PowerOperation, AckRequiresPostCommandMatchingFreshBackendPacket) {
    P p;p.epoch="b";p.feedback(3,100);p.accepted({"b","r",0},1,3);
    EXPECT_FALSE(p.acknowledged(100));p.feedback(1,110);EXPECT_FALSE(p.acknowledged(110));
    p.feedback(3,120);EXPECT_TRUE(p.acknowledged(120));EXPECT_FALSE(p.acknowledged(400000121));
}
TEST(PowerOperation, OldEpochNeverResumesAfterBridgeRestart) {
    P p;p.epoch="new";EXPECT_TRUE(p.fence(P::Request{"old","r",0}));
    p.off(P::Request{"old","off",0},1);EXPECT_EQ(p.target_mask,0);
}
TEST(PowerOperation, RejectionDoesNotEraseOffReceiptAndOffDoesNotClearEstop) {
    P p;p.epoch="b";p.off(P::Request{"b","off",0},1);
    p.rejected(P::Request{"b","old",0},"off_preempted",false);
    p.feedback(0,10);EXPECT_TRUE(p.acknowledged(10));EXPECT_EQ(p.accepted_id,"off");
    p.rejected(P::Request{"b","bad",1},"unauthorized");EXPECT_EQ(p.readiness(10,false),"rejected");
    p.off(P::Request{"b","off2",1},1);EXPECT_TRUE(p.fault.empty());
    EXPECT_EQ(p.readiness(10,true),"rejected");
}
TEST(PowerOperation, BackendReadinessRequiresBothStreams) {
    P p;EXPECT_EQ(p.readiness(1,false),"starting");p.feedback(0,10);
    EXPECT_EQ(p.readiness(20,false),"starting");p.motor_time=20;
    EXPECT_EQ(p.readiness(30,false),"ready");EXPECT_EQ(p.readiness(400000030,false),"backend_unavailable");
}

TEST(PowerOperation, NonAdjacentBackendReplayCannotBecomeFreshAck) {
    rinbo_ros_bridge::DeviceFeedbackOrder order;
    EXPECT_TRUE(order.accept(10,100,1));EXPECT_TRUE(order.accept(11,100,2));
    EXPECT_FALSE(order.accept(10,100,1));EXPECT_FALSE(order.accept(12,100,1));
    EXPECT_FALSE(order.accept(11,100,3));EXPECT_TRUE(order.accept(13,100,4));
    EXPECT_FALSE(order.accept(1,101,0)); // Backend reset is not silently trusted.
    EXPECT_FALSE(order.accept(14,101,1000000));
}
