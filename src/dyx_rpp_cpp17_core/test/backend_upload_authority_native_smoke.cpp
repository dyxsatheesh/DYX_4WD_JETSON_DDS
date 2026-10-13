#include "backend_upload_fixture.hpp"
#include <limits>
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload upload(gateway);
  auto request=upload_req(kBackendMissionUploadPath,1,1);
  auto t=request;t.method="GET";
  assert(upload.handle(t).http_status==405);
  t=request;t.principal.authenticated=false;
  assert(upload.handle(t).http_status==401);
  t=request;t.principal.owns_control_lease=false;
  assert(upload.handle(t).http_status==403);
  t=request;t.mission_run_id="bad id";
  assert(upload.handle(t).http_status==400);
  t=request;t.host_received_sec=std::numeric_limits<double>::quiet_NaN();
  assert(upload.handle(t).http_status==400);
  t=request;t.extension_distance_m=20.01;
  assert(upload.handle(t).http_status==422);
  t=request;t.mission_mode=static_cast<MissionMode>(99);
  assert(upload.handle(t).http_status==422);
  t=request;t.trusted_px4_disarmed=false;
  assert(upload.handle(t).http_status==412);
  assert(!upload.has_draft());
  auto prepared=upload.handle(request);
  assert(prepared.accepted());
  auto view=draft_req();view.principal.authenticated=false;
  assert(upload.handle(view).http_status==401);
  view=draft_req();view.principal.can_read_mission=false;
  assert(upload.handle(view).http_status==403);
  view=draft_req();view.principal.operator_session_id="ANOTHER_SESSION";
  const auto private_view = upload.handle(view);
  assert(private_view.http_status==403 && private_view.signature.empty() &&
         private_view.navigation_count==0);
  auto foreign=update_req(prepared,2,2);
  foreign.principal.operator_session_id="ANOTHER_SESSION";
  assert(upload.handle(foreign).http_status==409);
  auto replay=update_req(prepared,1,2);
  assert(upload.handle(replay).code=="COMMAND_REPLAY_REJECTED");
  auto backward=update_req(prepared,2,0.5);
  assert(upload.handle(backward).code=="NON_MONOTONIC_HOST_TIME");
  auto wrong=update_req(prepared,2,2);
  wrong.expected_revision=0;
  assert(upload.handle(wrong).code=="DRAFT_REVISION_CONFLICT");
  assert(upload.handle(update_req(prepared,2,2)).code=="COMMAND_REPLAY_REJECTED");
  assert(upload.discard_unloaded(false,true)==false);
  assert(upload.discard_unloaded(true,false)==false);
  assert(upload.discard_unloaded(true,true));
  assert(!upload.has_draft());
}
