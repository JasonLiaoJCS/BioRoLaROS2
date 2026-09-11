#pragma once
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <cctype>

namespace rinbo_ros_bridge {
// Backend timestamps are compared only with that backend's earlier packet,
// never against Orin wall time. Reject non-adjacent replay as well as duplicates.
struct DeviceFeedbackOrder {
    bool seen=false; uint32_t sequence=0; int64_t seconds=0; int32_t usec=0;
    bool accept(uint32_t seq,int64_t sec,int32_t us) {
        if (seq==0 || sec<0 || us<0 || us>=1000000) return false;
        const uint32_t delta=seq-sequence;
        if (seen && (delta==0 || delta>=0x80000000U || sec<seconds ||
                     (sec==seconds && us<=usec))) return false;
        seen=true;sequence=seq;seconds=sec;usec=us;return true;
    }
};
// Operation fencing supplements (never replaces) exact publisher/GID checks.
// All timestamps below are monotonic nanoseconds on the Bridge host.
struct PowerOperationProtocol {
    struct Request { std::string epoch, id; uint64_t generation = 0; };
    std::string epoch;
    uint64_t generation = 0, feedback_serial = 0, accepted_at_serial = 0;
    uint32_t accepted_sequence = 0;
    int feedback_mask = -1, target_mask = -1;
    int64_t power_time = 0, motor_time = 0;
    std::string accepted_id, rejected_id, rejection_reason, fault;

    static std::optional<Request> parse(const std::string& text) {
        std::vector<std::string> parts; std::istringstream in(text); std::string part;
        while (std::getline(in, part, '|')) parts.push_back(part);
        if (parts.size()!=4 || parts[0]!="P2" || parts[1].empty() || parts[1].size()>200 ||
            parts[2].empty() || parts[3].empty() || parts[3].size()>80) return std::nullopt;
        for (char c: parts[2]) if (!std::isdigit(static_cast<unsigned char>(c))) return std::nullopt;
        for (char c: parts[3]) if (!std::isalnum(static_cast<unsigned char>(c)) && c!='-' && c!='_') return std::nullopt;
        try { return Request{parts[1],parts[3],std::stoull(parts[2])}; }
        catch (...) { return std::nullopt; }
    }
    bool fresh(int64_t now) const {
        return power_time>0 && motor_time>0 && now>=power_time && now>=motor_time &&
            now-power_time<=350000000 && now-motor_time<=250000000;
    }
    bool acknowledged(int64_t now) const {
        return power_time>0 && now>=power_time && now-power_time<=350000000 &&
            feedback_serial>accepted_at_serial && feedback_mask==target_mask && !accepted_id.empty();
    }
    std::string readiness(int64_t now, bool estop) const {
        if (estop || !fault.empty()) return "rejected";
        if (fresh(now)) return "ready";
        return power_time==0 || motor_time==0 ? "starting" : "backend_unavailable";
    }
    std::optional<std::string> fence(const std::optional<Request>& r) const {
        if (!r) return "protocol_v2_required: upgrade Orin power tool and Bridge together";
        if (r->epoch!=epoch) return "bridge_epoch_changed: obtain fresh readiness; do not replay operation";
        if (r->generation!=generation) return "off_preempted: operation cancelled by a newer Off; explicit new operation required";
        return std::nullopt;
    }
    void accepted(const Request& r, uint32_t sequence, int mask) {
        accepted_id=r.id; accepted_sequence=sequence; target_mask=mask;
        if (rejected_id==r.id) { rejected_id.clear(); rejection_reason.clear(); }
        accepted_at_serial=feedback_serial;
    }
    void off(const std::optional<Request>& r, uint32_t sequence) {
        // Repeated packets of the same Off keep one generation; a later Off
        // after a new enable always starts another cancellation boundary.
        if (!r || accepted_id!=r->id || target_mask!=0) ++generation;
        fault.clear();
        if (r && rejected_id==r->id) { rejected_id.clear(); rejection_reason.clear(); }
        accepted_id=r ? r->id : "legacy-off";
        accepted_sequence=sequence; target_mask=0; accepted_at_serial=feedback_serial;
    }
    void rejected(const std::optional<Request>& r, const std::string& why, bool latch=true) {
        rejected_id=r ? r->id : ""; rejection_reason=why;
        if (latch && fault.empty()) fault=why;
    }
    void feedback(int mask,int64_t now) { feedback_mask=mask; power_time=now; ++feedback_serial; }
};
inline std::string power_json_string(const std::string& value) {
    std::ostringstream out; out << '"';
    for (unsigned char c:value) {
        if (c=='"' || c=='\\') out << '\\' << c;
        else if (c<32) out << ' ';
        else out << c;
    }
    out << '"'; return out.str();
}
} // namespace rinbo_ros_bridge
