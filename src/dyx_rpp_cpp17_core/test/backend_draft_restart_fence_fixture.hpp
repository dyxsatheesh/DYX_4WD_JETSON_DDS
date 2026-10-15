#pragma once
#include "backend_draft_security_boundary_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_restart_fence_lab.hpp"
#include <cassert>
#include <cstdint>
using namespace dyx::rpp;

// In-memory fake of an already PROVISIONED store, shared across lab restarts.
// NOT actual disk durability, encryption, tamper protection or crash atomicity.
class RestartLabMemoryStore final : public DraftRestartLabStore {
 public:
  bool available{true};
  bool present{true};
  bool write_available{true};
  bool force_conflict{false};
  int reads{0};
  int write_attempts{0};
  int commits{0};
  DraftRestartLabRecord record{"SESSION_H4R_OPERATOR", 3, 0, 0.0, 1};

  bool load_existing(DraftRestartLabRecord& output) override {
    ++reads;
    if (!available || !present) return false;
    output=record;
    return true;
  }
  bool compare_exchange_durable(const DraftRestartLabRecord& expected,
                                const DraftRestartLabRecord& replacement) override {
    ++write_attempts;
    if (!available || !present || !write_available || force_conflict ||
        !draft_restart_lab_record_equal(record,expected) ||
        !draft_restart_lab_record_valid(replacement) ||
        replacement.generation != expected.generation+1 ||
        replacement.last_ingress_nonce <= expected.last_ingress_nonce ||
        replacement.last_received_sec < expected.last_received_sec) return false;
    record = replacement;
    ++commits;
    return true;
  }
};

inline AndroidDraftSyncDecision restart_sync(
    NativeOfflineAndroidDraftCoordinator& app,
    NativeOfflineDraftSecurityBoundaryLab& boundary,
    BoundaryLabFixtureSource& source,double start,std::uint64_t nonce) {
  auto request=app.begin_resync(start);
  assert(request.valid);
  source.set_time_nonce(start+0.02,nonce);
  const auto outcome=boundary.snapshot(
      framed_lab_request(request.method,request.path,request.body),request);
  assert(outcome.reached_backend && outcome.trusted_reply_provenance &&
         !outcome.network_io_performed && !outcome.hardware_command_emitted);
  return app.complete_resync(request,
      draft_ingress_lab_sync_reply(request,outcome,start,start+0.04),start+0.05);
}
inline AndroidDraftCoordinatorResult restart_mutate(
    NativeOfflineAndroidDraftCoordinator& app,
    NativeOfflineDraftSecurityBoundaryLab& boundary,
    BoundaryLabFixtureSource& source, AndroidEditorIntent intent,
    double start,std::uint64_t nonce) {
  auto request=app.begin_mutation(intent,start);
  assert(request.valid);
  source.set_time_nonce(start+0.02,nonce);
  const auto outcome=boundary.mutation(
      framed_lab_request(request.method,request.path,request.body),request);
  assert(outcome.reached_backend && outcome.trusted_reply_provenance &&
         !outcome.network_io_performed && !outcome.hardware_command_emitted);
  return app.complete_mutation(request,
      draft_ingress_lab_mutation_reply(request,outcome,start,start+0.04),start+0.05);
}
