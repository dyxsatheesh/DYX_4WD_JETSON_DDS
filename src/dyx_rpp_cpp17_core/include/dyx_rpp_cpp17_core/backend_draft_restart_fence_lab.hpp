#pragma once
// DYX Autonomous Phase 2H4W: OFFLINE restart replay fence, not production IO.
// The caller-supplied store promises atomic *durable* compare-and-swap, but
// this lab implements neither a disk journal nor fsync, authentication, HTTPS,
// PX4/DDS state verification, network transport, or actuator control.
// Only a separately provisioned trusted store may supply the initial record.

#include "dyx_rpp_cpp17_core/backend_draft_security_boundary_lab.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace dyx::rpp {

struct DraftRestartLabRecord {
  std::string operator_session_id{};
  std::uint64_t session_epoch{0};
  std::uint64_t last_ingress_nonce{0};
  double last_received_sec{0.0};
  std::uint64_t generation{0};
};

inline bool draft_restart_lab_record_valid(const DraftRestartLabRecord& r) {
  return valid_native_mission_run_id(r.operator_session_id) &&
         r.session_epoch != 0 && r.generation != 0 &&
         std::isfinite(r.last_received_sec) && r.last_received_sec >= 0.0;
}

inline bool draft_restart_lab_record_equal(const DraftRestartLabRecord& a,
                                           const DraftRestartLabRecord& b) {
  return a.operator_session_id == b.operator_session_id &&
         a.session_epoch == b.session_epoch &&
         a.last_ingress_nonce == b.last_ingress_nonce &&
         a.last_received_sec == b.last_received_sec &&
         a.generation == b.generation;
}

// For real use, this API would require audited crash-atomic persistence,
// integrity verification, a single-writer or transactional concurrency model,
// and power-loss testing. An absent, corrupt or unavailable record MUST DENY.
// Never initialize a missing record from a request or from Android metadata.
class DraftRestartLabStore {
 public:
  virtual ~DraftRestartLabStore() = default;
  virtual bool load_existing(DraftRestartLabRecord& record) = 0;
  virtual bool compare_exchange_durable(const DraftRestartLabRecord& expected,
                                        const DraftRestartLabRecord& replacement) = 0;
};

// Wraps an H4V mock privilege source. Saves the verified nonce BEFORE the
// request can reach the H4T backend. A backend rejection still consumes nonce.
// Loss of reply cannot be interpreted as safe permission to replay the POST.
// This laboratory adapter is NOT thread-safe; concurrent real ingress requires
// serialized requests plus durable transactional replay/mission semantics.
class NativeOfflineDraftRestartFenceSource final
    : public DraftBoundaryLabTrustedSource {
 public:
  NativeOfflineDraftRestartFenceSource(DraftBoundaryLabTrustedSource& source,
                                      DraftRestartLabStore& store)
      : source_(source), store_(store) {}

  bool monotonic_receive_time(double& seconds) override {
    clock_obtained_ = false;
    double observed = 0.0;
    if (!source_.monotonic_receive_time(observed) ||
        !std::isfinite(observed) || observed < 0.0) return false;
    last_observed_sec_ = observed;
    clock_obtained_ = true;
    seconds = observed;
    return true;
  }

  bool verified_session(DraftBoundaryLabSession& session) override {
    if (!clock_obtained_) return false;
    clock_obtained_ = false; // one proof per independent clock observation
    DraftBoundaryLabSession candidate;
    if (!source_.verified_session(candidate) ||
        !candidate.verified_transport || !candidate.principal.authenticated ||
        !valid_native_mission_run_id(candidate.principal.operator_session_id) ||
        candidate.session_epoch == 0 || candidate.ingress_nonce == 0) return false;

    DraftRestartLabRecord previous;
    if (!store_.load_existing(previous) ||
        !draft_restart_lab_record_valid(previous) ||
        previous.operator_session_id != candidate.principal.operator_session_id ||
        previous.session_epoch != candidate.session_epoch ||
        candidate.ingress_nonce <= previous.last_ingress_nonce ||
        last_observed_sec_ < previous.last_received_sec ||
        previous.generation == std::numeric_limits<std::uint64_t>::max())
      return false;

    DraftRestartLabRecord next = previous;
    next.last_ingress_nonce = candidate.ingress_nonce;
    next.last_received_sec = last_observed_sec_;
    ++next.generation;
    if (!store_.compare_exchange_durable(previous, next)) return false;
    // All accepted and backend-denied requests have a reserved nonce. A real
    // store must guarantee this record survives process/power loss before ACK.
    session = candidate;
    return true;
  }

  bool independently_observed_px4(DraftBoundaryLabPx4& px4) override {
    return source_.independently_observed_px4(px4);
  }

 private:
  DraftBoundaryLabTrustedSource& source_;
  DraftRestartLabStore& store_;
  double last_observed_sec_{0.0};
  bool clock_obtained_{false};
};

} // namespace dyx::rpp
