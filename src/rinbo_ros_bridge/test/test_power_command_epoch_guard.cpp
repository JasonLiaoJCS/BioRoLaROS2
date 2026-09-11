#include "power_command_epoch_guard.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace {

using Guard = rinbo_ros_bridge::PowerCommandEpochGuard;
using Kind = Guard::CommandKind;

constexpr int64_t kNowNs = 10000000000LL;
constexpr int64_t kMaxAgeNs = 200000000LL;

Guard::PublisherGid gid(uint8_t value) {
    return Guard::PublisherGid(24U, value);
}

Guard::PublisherGraph sole_publisher(const Guard::PublisherGid &publisher_gid) {
    Guard::PublisherGraph graph;
    graph.publisher_count = 1U;
    graph.sole_publisher_gid = publisher_gid;
    graph.sole_node_name = "redrhex_rinbo_power_tool";
    graph.sole_node_namespace = "/";
    return graph;
}

Guard::Header header(uint32_t sequence, int64_t stamp_ns) {
    Guard::Header result;
    result.sequence = sequence;
    result.stamp_sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    result.stamp_nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    return result;
}

std::optional<std::string> accept(
    Guard &guard,
    Kind kind,
    const Guard::PublisherGraph &graph,
    const Guard::PublisherGid &publisher_gid,
    uint32_t sequence,
    int64_t stamp_ns) {
    return guard.accept_energizing(
        kind, graph, publisher_gid, header(sequence, stamp_ns),
        kNowNs, kMaxAgeNs);
}

std::optional<std::string> accept(
    Guard &guard,
    Kind kind,
    const Guard::PublisherGid &publisher_gid,
    uint32_t sequence,
    int64_t stamp_ns) {
    return accept(
        guard, kind, sole_publisher(publisher_gid), publisher_gid,
        sequence, stamp_ns);
}

void expect_rejection_contains(
    const std::optional<std::string> &reason,
    const std::string &expected) {
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find(expected), std::string::npos) << *reason;
}

TEST(PowerCommandEpochGuard, ClassifiesAndSanitizesTrueAllOff) {
    Guard::Payload payload;
    payload.clean = true;
    payload.trigger = true;

    const auto classification = Guard::classify(payload);
    EXPECT_EQ(classification.kind, Kind::kAllOff);
    EXPECT_FALSE(classification.rejection_reason.has_value());
    EXPECT_FALSE(classification.forward_payload.digital);
    EXPECT_FALSE(classification.forward_payload.signal);
    EXPECT_FALSE(classification.forward_payload.power);
    EXPECT_FALSE(classification.forward_payload.clean);
    EXPECT_FALSE(classification.forward_payload.trigger);
}

TEST(PowerCommandEpochGuard, ClassifiesOnlyValidEnergizingPayloads) {
    {
        Guard::Payload payload;
        payload.digital = true;
        EXPECT_EQ(Guard::classify(payload).kind, Kind::kRelayOffSequence);
    }
    {
        Guard::Payload payload;
        payload.digital = true;
        payload.signal = true;
        EXPECT_EQ(Guard::classify(payload).kind, Kind::kRelayOffSequence);
    }
    {
        Guard::Payload payload;
        payload.digital = true;
        payload.signal = true;
        payload.power = true;
        EXPECT_EQ(Guard::classify(payload).kind, Kind::kPowerOn);
    }
    {
        Guard::Payload payload;
        payload.signal = true;
        const auto classification = Guard::classify(payload);
        EXPECT_EQ(classification.kind, Kind::kInvalid);
        expect_rejection_contains(
            classification.rejection_reason, "signal=true requires digital=true");
    }
    {
        Guard::Payload payload;
        payload.digital = true;
        payload.power = true;
        const auto classification = Guard::classify(payload);
        EXPECT_EQ(classification.kind, Kind::kInvalid);
        expect_rejection_contains(
            classification.rejection_reason,
            "power=true requires digital=true and signal=true");
    }
    for (const bool use_clean : std::array<bool, 2>{false, true}) {
        Guard::Payload payload;
        payload.digital = true;
        payload.clean = use_clean;
        payload.trigger = !use_clean;
        const auto classification = Guard::classify(payload);
        EXPECT_EQ(classification.kind, Kind::kInvalid);
        expect_rejection_contains(
            classification.rejection_reason, "legacy clean/trigger");
    }
}

TEST(PowerCommandEpochGuard, DifferentProcessOffSequenceRelayIsAccepted) {
    Guard guard;
    const auto sequence_gid = gid(0xB2U);
    const auto relay_gid = gid(0xC3U);

    // A true all-off from process/GID A needs no publisher identity at all.
    // Process/GID B then performs the authenticated relay-off sequence, which
    // opens a safe new-source boundary for relay process/GID C.
    guard.observe_all_off();
    EXPECT_FALSE(accept(
        guard, Kind::kRelayOffSequence, sequence_gid,
        1U, kNowNs - 40000000LL));
    EXPECT_FALSE(accept(
        guard, Kind::kRelayOffSequence, sequence_gid,
        2U, kNowNs - 30000000LL));
    EXPECT_FALSE(accept(
        guard, Kind::kPowerOn, relay_gid,
        1U, kNowNs - 20000000LL));
    EXPECT_FALSE(accept(
        guard, Kind::kPowerOn, relay_gid,
        2U, kNowNs - 10000000LL));
}

TEST(PowerCommandEpochGuard, SameSourceRelayOffOpensSafePublisherHandoff) {
    Guard guard;
    const auto first_gid = gid(0xC1U);
    const auto replacement_gid = gid(0xD1U);

    ASSERT_FALSE(accept(
        guard, Kind::kPowerOn, first_gid, 1U, kNowNs - 30000000LL));
    ASSERT_FALSE(accept(
        guard, Kind::kRelayOffSequence, first_gid,
        2U, kNowNs - 20000000LL));
    EXPECT_FALSE(accept(
        guard, Kind::kPowerOn, replacement_gid,
        1U, kNowNs - 10000000LL));
}

TEST(PowerCommandEpochGuard, PublisherChangeWithoutBoundaryRejectsAndDoesNotTakeOver) {
    Guard guard;
    const auto first_gid = gid(0x11U);
    const auto replacement_gid = gid(0x22U);

    ASSERT_FALSE(accept(
        guard, Kind::kPowerOn, first_gid, 7U, kNowNs - 30000000LL));
    expect_rejection_contains(
        accept(
            guard, Kind::kPowerOn, replacement_gid,
            1U, kNowNs - 20000000LL),
        "changed without an authenticated relay-off boundary");

    // A rejected replacement cannot steal or clear the original epoch.
    EXPECT_FALSE(accept(
        guard, Kind::kPowerOn, first_gid, 8U, kNowNs - 10000000LL));
}

TEST(PowerCommandEpochGuard, TrueAllOffAllowsAReplacementPublisher) {
    Guard guard;
    const auto first_gid = gid(0x31U);
    const auto replacement_gid = gid(0x32U);

    ASSERT_FALSE(accept(
        guard, Kind::kPowerOn, first_gid, 7U, kNowNs - 30000000LL));
    expect_rejection_contains(
        accept(
            guard, Kind::kPowerOn, replacement_gid,
            1U, kNowNs - 20000000LL),
        "changed without an authenticated relay-off boundary");
    guard.observe_all_off();
    EXPECT_FALSE(accept(
        guard, Kind::kPowerOn, replacement_gid,
        19U, kNowNs - 10000000LL));
}

TEST(PowerCommandEpochGuard, UnauthenticatedSequenceCannotOpenHandoffBoundary) {
    for (const bool wrong_node : std::array<bool, 2>{true, false}) {
        SCOPED_TRACE(wrong_node ? "wrong node" : "multiple publishers");
        Guard guard;
        const auto first_gid = gid(0x41U);
        const auto replacement_gid = gid(0x42U);
        ASSERT_FALSE(accept(
            guard, Kind::kPowerOn, first_gid,
            1U, kNowNs - 40000000LL));

        auto graph = sole_publisher(first_gid);
        if (wrong_node) {
            graph.sole_node_name = "raw_power_publisher";
        } else {
            graph.publisher_count = 2U;
        }
        ASSERT_TRUE(accept(
            guard, Kind::kRelayOffSequence, graph, first_gid,
            2U, kNowNs - 30000000LL));

        // If the rejected sequence had established a boundary, this different
        // publisher would be accepted. It must remain rejected and unforwarded.
        expect_rejection_contains(
            accept(
                guard, Kind::kPowerOn, replacement_gid,
                1U, kNowNs - 20000000LL),
            "changed without an authenticated relay-off boundary");
        EXPECT_FALSE(accept(
            guard, Kind::kPowerOn, first_gid,
            2U, kNowNs - 10000000LL));
    }
}

TEST(PowerCommandEpochGuard, SkippedSamePublisherSequenceRejectsWithoutAdvancing) {
    Guard guard;
    const auto publisher_gid = gid(0x51U);
    ASSERT_FALSE(accept(
        guard, Kind::kRelayOffSequence, publisher_gid,
        1U, kNowNs - 30000000LL));
    expect_rejection_contains(
        accept(
            guard, Kind::kRelayOffSequence, publisher_gid,
            3U, kNowNs - 20000000LL),
        "sequence is duplicate, out-of-order, or skipped");
    EXPECT_FALSE(accept(
        guard, Kind::kRelayOffSequence, publisher_gid,
        2U, kNowNs - 10000000LL));
}

TEST(PowerCommandEpochGuard, RequiresExactlyOnePublisher) {
    for (const std::size_t publisher_count : std::array<std::size_t, 2>{0U, 2U}) {
        SCOPED_TRACE(publisher_count);
        Guard guard;
        Guard::PublisherGraph graph;
        graph.publisher_count = publisher_count;
        graph.sole_publisher_gid = gid(0x61U);
        expect_rejection_contains(
            accept(
                guard, Kind::kRelayOffSequence, graph, gid(0x61U),
                1U, kNowNs),
            "expected exactly one publisher");
    }
}

TEST(PowerCommandEpochGuard, RejectsPublisherGraphQueryFailure) {
    Guard guard;
    Guard::PublisherGraph graph;
    graph.query_succeeded = false;
    graph.query_error = "power publisher graph query failed: injected";

    expect_rejection_contains(
        accept(
            guard, Kind::kRelayOffSequence, graph, gid(0x71U),
            1U, kNowNs),
        "graph query failed");
}

TEST(PowerCommandEpochGuard, CallbackGidMustMatchTheSoleGraphPublisher) {
    Guard guard;
    expect_rejection_contains(
        accept(
            guard, Kind::kRelayOffSequence,
            sole_publisher(gid(0x81U)), gid(0x82U), 1U, kNowNs),
        "received source does not match");
}

TEST(PowerCommandEpochGuard, RejectsWrongPublisherNodeOrNamespace) {
    for (const bool wrong_name : std::array<bool, 2>{true, false}) {
        SCOPED_TRACE(wrong_name ? "wrong name" : "wrong namespace");
        Guard guard;
        const auto publisher_gid = gid(0x91U);
        auto graph = sole_publisher(publisher_gid);
        if (wrong_name) {
            graph.sole_node_name = "raw_power_publisher";
        } else {
            graph.sole_node_namespace = "/unexpected";
        }

        expect_rejection_contains(
            accept(
                guard, Kind::kRelayOffSequence, graph, publisher_gid,
                1U, kNowNs),
            "must be exact node /redrhex_rinbo_power_tool");
    }
}

TEST(PowerCommandEpochGuard, RejectsZeroDuplicateBackwardAndSkippedSequences) {
    {
        Guard guard;
        const auto publisher_gid = gid(0xA1U);
        expect_rejection_contains(
            accept(
                guard, Kind::kPowerOn, publisher_gid, 0U, kNowNs),
            "sequence must be nonzero");
    }

    for (const uint32_t rejected_sequence : std::array<uint32_t, 3>{10U, 9U, 12U}) {
        SCOPED_TRACE(rejected_sequence);
        Guard guard;
        const auto publisher_gid = gid(0xA2U);
        ASSERT_FALSE(accept(
            guard, Kind::kPowerOn, publisher_gid,
            10U, kNowNs - 20000000LL));
        expect_rejection_contains(
            accept(
                guard, Kind::kPowerOn, publisher_gid,
                rejected_sequence, kNowNs - 10000000LL),
            "sequence is duplicate, out-of-order, or skipped");
    }
}

TEST(PowerCommandEpochGuard, RejectsMalformedStaleAndFutureStamps) {
    const auto publisher_gid = gid(0xB1U);

    {
        Guard guard;
        auto invalid_nanosec = header(1U, kNowNs);
        invalid_nanosec.stamp_nanosec = 1000000000U;
        expect_rejection_contains(
            guard.accept_energizing(
                Kind::kPowerOn, sole_publisher(publisher_gid), publisher_gid,
                invalid_nanosec, kNowNs, kMaxAgeNs),
            "nanosec is outside");
    }
    {
        Guard guard;
        expect_rejection_contains(
            accept(guard, Kind::kPowerOn, publisher_gid, 1U, 0),
            "stamp must be nonzero");
    }
    for (const int64_t rejected_stamp : std::array<int64_t, 2>{
             kNowNs - kMaxAgeNs - 1,
             kNowNs + kMaxAgeNs + 1}) {
        SCOPED_TRACE(rejected_stamp);
        Guard guard;
        expect_rejection_contains(
            accept(
                guard, Kind::kPowerOn, publisher_gid,
                1U, rejected_stamp),
            "stamp age");
    }
}

TEST(PowerCommandEpochGuard, RejectsDuplicateOrBackwardStamp) {
    for (const int64_t rejected_stamp : std::array<int64_t, 2>{
             kNowNs - 20000000LL,
             kNowNs - 30000000LL}) {
        SCOPED_TRACE(rejected_stamp);
        Guard guard;
        const auto publisher_gid = gid(0xC1U);
        ASSERT_FALSE(accept(
            guard, Kind::kPowerOn, publisher_gid,
            1U, kNowNs - 20000000LL));
        expect_rejection_contains(
            accept(
                guard, Kind::kPowerOn, publisher_gid,
                2U, rejected_stamp),
            "stamp is duplicate or non-monotonic");
    }
}

TEST(PowerCommandEpochGuard, RejectsNonEnergizingKindAtAuthenticatedGate) {
    Guard guard;
    const auto publisher_gid = gid(0xD1U);
    expect_rejection_contains(
        accept(guard, Kind::kAllOff, publisher_gid, 1U, kNowNs),
        "not a valid energizing command");
    expect_rejection_contains(
        accept(guard, Kind::kInvalid, publisher_gid, 1U, kNowNs),
        "not a valid energizing command");
}

}  // namespace

TEST(PowerCommandEpochGuard, ExplicitRelayReleaseStillAuthenticatesAndNeverAllowsPowerOn) {
    Guard guard;
    ASSERT_FALSE(accept(guard, Kind::kPowerOn, gid(1), 1, kNowNs));
    EXPECT_TRUE(guard.accept_energizing(Kind::kPowerOn, sole_publisher(gid(2)), gid(2),
        header(1,kNowNs), kNowNs, kMaxAgeNs, true));
    auto foreign = sole_publisher(gid(2)); foreign.sole_node_name = "foreign";
    EXPECT_TRUE(guard.accept_energizing(Kind::kRelayOffSequence, foreign, gid(2),
        header(1,kNowNs), kNowNs, kMaxAgeNs, true));
    EXPECT_TRUE(guard.accept_energizing(Kind::kRelayOffSequence, sole_publisher(gid(2)), gid(2),
        header(1,kNowNs-2*kMaxAgeNs), kNowNs, kMaxAgeNs, true));
    EXPECT_FALSE(guard.accept_energizing(Kind::kRelayOffSequence, sole_publisher(gid(2)), gid(2),
        header(1,kNowNs), kNowNs, kMaxAgeNs, true));
    EXPECT_TRUE(guard.can_handoff());
}
