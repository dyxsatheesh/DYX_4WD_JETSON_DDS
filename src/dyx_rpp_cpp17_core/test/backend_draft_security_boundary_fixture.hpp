#pragma once
#include "backend_draft_http_framing_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_security_boundary_lab.hpp"
#include <cassert>
#include <cstdint>
#include <string>
using namespace dyx::rpp;

class BoundaryLabFixtureSource final : public DraftBoundaryLabTrustedSource {
 public:
  bool clock_available{true};
  bool session_available{true};
  bool px4_available{true};
  int clock_calls{0};
  int session_calls{0};
  int px4_calls{0};
  double time{10.02};
  DraftBoundaryLabSession session{};
  DraftBoundaryLabPx4 px4{true,true,10.0};

  BoundaryLabFixtureSource() { set_time_nonce(10.02,1); }
  void set_time_nonce(double now,std::uint64_t nonce) {
    time=now;
    session.verified_transport=true;
    session.principal={true,true,true,"SESSION_H4R_OPERATOR"};
    session.session_epoch=3;
    session.ingress_nonce=nonce;
    session.verified_auth_sec=now-0.05;
    px4={true,true,now-0.02};
  }
  bool monotonic_receive_time(double& seconds) override {
    ++clock_calls;
    if (!clock_available) return false;
    seconds=time;
    return true;
  }
  bool verified_session(DraftBoundaryLabSession& value) override {
    ++session_calls;
    if (!session_available) return false;
    value=session;
    return true;
  }
  bool independently_observed_px4(DraftBoundaryLabPx4& value) override {
    ++px4_calls;
    if (!px4_available) return false;
    value=px4;
    return true;
  }
};

inline AndroidDraftSyncDecision boundary_sync(
     NativeOfflineAndroidDraftCoordinator& app,
     NativeOfflineDraftSecurityBoundaryLab& boundary,
     BoundaryLabFixtureSource& source,double start,std::uint64_t nonce) {
  const auto request=app.begin_resync(start);
  assert(request.valid);
  source.set_time_nonce(start+0.02,nonce);
  const auto outcome=boundary.snapshot(
      framed_lab_request(request.method,request.path,request.body),request);
  assert(outcome.reached_backend && outcome.trusted_reply_provenance &&
         !outcome.hardware_command_emitted && !outcome.network_io_performed);
  return app.complete_resync(request,
     draft_ingress_lab_sync_reply(request,outcome,start,start+0.04),start+0.05);
}
inline AndroidDraftCoordinatorResult boundary_mutate(
     NativeOfflineAndroidDraftCoordinator& app,
     NativeOfflineDraftSecurityBoundaryLab& boundary,
     BoundaryLabFixtureSource& source, AndroidEditorIntent intent,
     double start,std::uint64_t nonce) {
  const auto request=app.begin_mutation(intent,start);
  assert(request.valid);
  source.set_time_nonce(start+0.02,nonce);
  const auto outcome=boundary.mutation(
      framed_lab_request(request.method,request.path,request.body),request);
  assert(outcome.reached_backend && outcome.trusted_reply_provenance &&
         !outcome.hardware_command_emitted && !outcome.network_io_performed);
  return app.complete_mutation(request,
     draft_ingress_lab_mutation_reply(request,outcome,start,start+0.04),start+0.05);
}
