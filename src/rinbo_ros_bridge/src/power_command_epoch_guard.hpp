#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace rinbo_ros_bridge {

// Pure state machine for the safety-critical /power/command source epoch.
// The caller serializes access and performs the actual all-off/power publish.
class PowerCommandEpochGuard {
public:
    using PublisherGid = std::vector<uint8_t>;

    struct PublisherGraph {
        bool query_succeeded = true;
        std::size_t publisher_count = 0;
        PublisherGid sole_publisher_gid;
        std::string sole_node_name;
        std::string sole_node_namespace;
        std::string query_error;
    };

    struct Header {
        uint32_t sequence = 0;
        int32_t stamp_sec = 0;
        uint32_t stamp_nanosec = 0;
    };

    struct Payload {
        bool digital = false;
        bool signal = false;
        bool power = false;
        bool clean = false;
        bool trigger = false;
    };

    enum class CommandKind {
        kInvalid,
        kAllOff,
        kRelayOffSequence,
        kPowerOn,
    };

    struct Classification {
        CommandKind kind = CommandKind::kInvalid;
        Payload forward_payload;
        std::optional<std::string> rejection_reason;
    };

    // A true all-off is identified only by the three deployed power rails.
    // Legacy clean/trigger bits cannot turn it into an actuation request, but
    // they are forcibly cleared before the caller forwards the command.
    static Classification classify(const Payload &payload) {
        Classification result;
        result.forward_payload = payload;
        if (!payload.power && !payload.digital && !payload.signal) {
            result.kind = CommandKind::kAllOff;
            result.forward_payload.clean = false;
            result.forward_payload.trigger = false;
            return result;
        }

        if (payload.signal && !payload.digital) {
            result.rejection_reason =
                "signal=true requires digital=true";
            return result;
        }
        if (payload.power && (!payload.digital || !payload.signal)) {
            result.rejection_reason =
                "power=true requires digital=true and signal=true";
            return result;
        }
        if (payload.clean || payload.trigger) {
            result.rejection_reason =
                "legacy clean/trigger fields are forbidden for energizing commands";
            return result;
        }

        result.kind = payload.power
            ? CommandKind::kPowerOn
            : CommandKind::kRelayOffSequence;
        return result;
    }

    // A received true all-off is the only unauthenticated command allowed to
    // erase the source epoch. This must remain usable during graph ambiguity,
    // source loss, malformed headers, or a sticky software E-stop.
    void observe_all_off() noexcept {
        last_publisher_gid_.clear();
        publisher_seen_ = false;
        header_seen_ = false;
        next_publisher_handoff_allowed_ = false;
        last_sequence_ = 0;
        last_stamp_ns_ = 0;
    }

    std::optional<std::string> accept_energizing(
        CommandKind kind,
        const PublisherGraph &graph,
        const PublisherGid &received_gid,
        const Header &header,
        int64_t observed_ns,
        int64_t max_age_ns) {
        if (kind != CommandKind::kRelayOffSequence &&
            kind != CommandKind::kPowerOn) {
            return "power command is not a valid energizing command";
        }
        if (!graph.query_succeeded) {
            return graph.query_error.empty()
                ? "power publisher graph query failed"
                : graph.query_error;
        }
        if (graph.publisher_count != 1U) {
            std::ostringstream out;
            out << "expected exactly one publisher on /power/command, got "
                << graph.publisher_count;
            return out.str();
        }
        if (graph.sole_node_name != kExpectedPublisherNodeName ||
            graph.sole_node_namespace != kExpectedPublisherNodeNamespace) {
            std::ostringstream out;
            out << "the sole /power/command publisher must be exact node "
                << kExpectedPublisherNodeNamespace << kExpectedPublisherNodeName
                << ", got namespace='" << graph.sole_node_namespace
                << "' name='" << graph.sole_node_name << "'";
            return out.str();
        }
        if (received_gid != graph.sole_publisher_gid) {
            return "received source does not match the sole /power/command publisher";
        }

        const bool publisher_changed =
            publisher_seen_ && graph.sole_publisher_gid != last_publisher_gid_;
        if (publisher_changed && !next_publisher_handoff_allowed_) {
            return "the sole /power/command publisher changed without an authenticated relay-off boundary";
        }
        if (header.stamp_nanosec >= 1000000000U) {
            return "power command stamp.nanosec is outside [0,1e9)";
        }
        const int64_t stamp_ns =
            header.stamp_sec * 1000000000LL +
            static_cast<int64_t>(header.stamp_nanosec);
        if (stamp_ns <= 0) {
            return "power command stamp must be nonzero and positive";
        }
        if (max_age_ns <= 0) {
            return "power command max age must be positive";
        }
        const int64_t age_ns = observed_ns - stamp_ns;
        if (age_ns > max_age_ns || age_ns < -max_age_ns) {
            std::ostringstream out;
            out << "power command stamp age "
                << static_cast<double>(age_ns) * 1.0e-9
                << "s exceeds +/-"
                << static_cast<double>(max_age_ns) * 1.0e-9 << "s";
            return out.str();
        }
        if (header.sequence == 0U) {
            return "power command sequence must be nonzero and positive";
        }

        // A handoff starts a new per-publisher header epoch. Commands from the
        // same publisher must still be exactly-next and strictly monotonic.
        if (header_seen_ && !publisher_changed) {
            if (stamp_ns <= last_stamp_ns_) {
                return "power command stamp is duplicate or non-monotonic";
            }
            const uint32_t sequence_delta = header.sequence - last_sequence_;
            if (sequence_delta != 1U) {
                return "power command sequence is duplicate, out-of-order, or skipped";
            }
        }

        last_publisher_gid_ = graph.sole_publisher_gid;
        publisher_seen_ = true;
        header_seen_ = true;
        last_sequence_ = header.sequence;
        last_stamp_ns_ = stamp_ns;
        next_publisher_handoff_allowed_ =
            kind == CommandKind::kRelayOffSequence;
        return std::nullopt;
    }

private:
    static constexpr const char *kExpectedPublisherNodeName =
        "redrhex_rinbo_power_tool";
    static constexpr const char *kExpectedPublisherNodeNamespace = "/";

    PublisherGid last_publisher_gid_;
    bool publisher_seen_ = false;
    bool header_seen_ = false;
    bool next_publisher_handoff_allowed_ = false;
    uint32_t last_sequence_ = 0;
    int64_t last_stamp_ns_ = 0;
};

}  // namespace rinbo_ros_bridge
