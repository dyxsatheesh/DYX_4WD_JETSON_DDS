#pragma once
namespace dyx::bridge {
struct RuntimeFaultDecision { bool latch; bool request_disarm; };
inline RuntimeFaultDecision wheel_lift_runtime_fault(bool armed, bool rejected_ack,
        bool ack_expired, bool previously_offboard, bool currently_offboard) noexcept {
    const bool fault = rejected_ack || ack_expired || (previously_offboard && !currently_offboard);
    return {fault, fault && armed};
}
}
