#include "backend_upload_http_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  const auto before=wire.handle(draft_get());
  assert(before.status==404 && reply_has(before,"NO_PENDING_DRAFT"));
  const auto first=wire.handle(draft_http("POST",kBackendMissionUploadPath,draft_upload_json()));
  assert(first.status==200 && reply_has(first,"UPLOAD_PREVIEW_READY"));
  assert(reply_has(first,"\"revision\":1") && reply_has(first,"\"ready\":true"));
  assert(!first.physical_command_emitted && !first.network_io_performed);
  assert(!gateway.has_active_mission());
  const auto preview=wire.handle(draft_get(10.2));
  assert(preview.status==200 && reply_has(preview,"DRAFT_PREVIEW_READY"));
  const auto signature=backend.handle([](){
    auto q=BackendMissionUploadRequest{};
    q.path=kBackendMissionDraftPath; q.method="GET";
    q.principal={true,true,false,"SESSION_H4P_OPERATOR"};
    q.mission_run_id="RUN_H4P_0001";return q;
  }()).signature;
  assert(valid_path_signature(signature));
  const auto updated=wire.handle(draft_http("POST",kBackendMissionUpdatePath,
      draft_update_json(2,1,signature),11));
  assert(updated.status==200 && reply_has(updated,"UPDATED_PREVIEW_READY"));
  assert(reply_has(updated,"\"revision\":2"));
  const auto loaded=wire.handle(draft_http("POST",kBackendMissionLoadPath,
      draft_load_json(3,2,backend.handle([](){
       BackendMissionUploadRequest r; r.method="GET";
       r.path=kBackendMissionDraftPath;
       r.mission_run_id="RUN_H4P_0001";
       r.principal={true,true,false,"SESSION_H4P_OPERATOR"};return r;
      }()).signature),12));
  assert(loaded.status==200 && reply_has(loaded,"MISSION_LOADED_READY_NOT_STARTED"));
  assert(reply_has(loaded,"\"loaded\":true"));
  assert(gateway.has_active_mission() && gateway.mission().stage()==MissionStage::ready);
  assert(!gateway.emits_hardware_commands());
  assert(wire.handle(draft_get(13)).status==200);
  assert(wire.handle(draft_http("POST",kBackendMissionLoadPath,
      draft_load_json(4,2,signature),14)).status==409);
  assert(loaded.http11_bytes().find("HTTP/1.1 200 OK\r\n")==0);
}
