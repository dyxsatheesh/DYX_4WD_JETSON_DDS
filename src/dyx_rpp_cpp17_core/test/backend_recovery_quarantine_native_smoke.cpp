// Phase 2H5L: true native (not SQLite-stubbed) C++17 gateway and backend
// quarantine tests. No physical rover, transport, ROS, or actuator outputs.
#include "backend_upload_fixture.hpp"
#include "mission_command_gateway_fixture.hpp"
#include <cassert>
#include <iostream>
#include <string>
using namespace dyx::rpp;

static void legacy() {
  NativeMissionCommandGateway g;
  NativeBackendMissionUpload b(g);
  assert(!g.recovery_quarantined());
  const auto r=b.handle(upload_req(kBackendMissionUploadPath,1,10));
  assert(r.accepted() && b.has_draft());
}
static void upload() {
  NativeMissionCommandGateway g;
  NativeBackendMissionUpload b(g);
  b.latch_recovery_quarantine();
  assert(b.recovery_quarantined());
  assert(b.handle(upload_req(kBackendMissionUploadPath,1,10)).http_status==423);
  assert(b.handle(draft_req()).http_status==423);
  assert(!b.discard_unloaded(true,true));
  assert(b.preview_snapshot()==nullptr && !b.has_draft());
}
static void alternate() {
  NativeMissionCommandGateway g;
  NativeBackendMissionUpload b(g),other(g);
  b.latch_recovery_quarantine();
  assert(other.recovery_quarantined());
  assert(other.handle(upload_req(kBackendMissionUploadPath,1,10)).http_status==423);
}
static void load() {
  NativeMissionCommandGateway g;
  g.latch_recovery_quarantine();
  assert(g.load(gateway_run,gateway_session,{},MissionMode::automatic,
                true,true,true)==ClientCommandStatus::recovery_quarantined);
  assert(!g.has_active_mission());
}
static void actions() {
  NativeMissionCommandGateway g;
  g.latch_recovery_quarantine();
  for (auto action : {ClientMissionAction::start,ClientMissionAction::pause,
                      ClientMissionAction::resume,ClientMissionAction::next,
                      ClientMissionAction::skip})
    assert(g.dispatch(gateway_request(action,1,0.1)).status==
           ClientCommandStatus::recovery_quarantined);
  auto unauth=gateway_request(ClientMissionAction::start,1,0.1);
  unauth.authenticated_operator=false;
  assert(g.dispatch(unauth).status==ClientCommandStatus::unauthorized);
}
static void stop() {
  NativeMissionCommandGateway g;
  g.latch_recovery_quarantine();
  auto req=gateway_request(ClientMissionAction::stop,1,-1.0);
  req.authorized_control_lease=false;
  assert(g.dispatch(req).accepted());
  g.force_stop();
  assert(g.safe_stop_requested());
}
static void epoch() {
  NativeMissionCommandGateway g;
  g.latch_recovery_quarantine();
  assert(!g.begin_new_disarmed_px4_epoch(true,true));
  assert(g.recovery_quarantined());
}
static void active() {
  NativeMissionCommandGateway g;
  gateway_load(g);
  g.latch_recovery_quarantine();
  assert(g.safe_stop_requested());
  assert(g.recovery_quarantined());
  assert(g.dispatch(gateway_request(ClientMissionAction::start,1,0.1)).status==
         ClientCommandStatus::recovery_quarantined);
}
static void sticky() {
  NativeMissionCommandGateway g;
  NativeBackendMissionUpload b(g);
  b.latch_recovery_quarantine();
  b.latch_recovery_quarantine();
  gateway_tick(g,0);
  assert(g.recovery_quarantined() && g.safe_stop_requested());
  assert(g.dispatch(gateway_request(ClientMissionAction::resume,1,0.1)).status==
         ClientCommandStatus::recovery_quarantined);
  assert(!g.begin_new_disarmed_px4_epoch(true,true));
}
int main(int argc,char** argv) {
  if (argc!=2) return 2;
  const std::string name=argv[1];
  if(name=="legacy")legacy();
  else if(name=="upload")upload();
  else if(name=="alternate")alternate();
  else if(name=="load")load();
  else if(name=="actions")actions();
  else if(name=="stop")stop();
  else if(name=="epoch")epoch();
  else if(name=="active")active();
  else if(name=="sticky")sticky();
  else return 2;
  std::cout<<"PASS H5L "<<name<<"\n";
}
