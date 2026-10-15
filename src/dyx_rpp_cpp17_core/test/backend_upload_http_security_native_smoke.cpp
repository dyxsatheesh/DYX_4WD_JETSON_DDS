#include "backend_upload_http_fixture.hpp"
int main(){
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  auto q=draft_http("POST",kBackendMissionUploadPath,draft_upload_json());
  q.http.principal.authenticated=false;
  assert(wire.handle(q).status==401 && !backend.has_draft());
  q.http.principal.authenticated=true;
  q.http.principal.owns_control_lease=false;
  assert(wire.handle(q).status==403 && !backend.has_draft());
  q.http.principal.owns_control_lease=true;
  q.trusted_px4_disarmed=false;
  assert(wire.handle(q).status==412 && !backend.has_draft());
  q.trusted_px4_disarmed=true;
  assert(wire.handle(q).status==200 && backend.has_draft());
  q=draft_get(11);
  q.http.principal.operator_session_id="SESSION_H4P_INTRUDER";
  assert(wire.handle(q).status==403);
  assert(!reply_has(wire.handle(q),"\"signature\""));
  q=draft_get(12); q.http.principal.can_read_mission=false;
  assert(wire.handle(q).status==403);
  q=draft_http("POST",kBackendMissionUpdatePath,
      draft_update_json(2,1,std::string(64,'0')),13);
  assert(wire.handle(q).status==409); // backend checks revision/digest
  assert(wire.handle(q).status==409); // replay rejected too
  q=draft_http("POST",kBackendMissionUploadPath,draft_upload_json(4),14);
  assert(wire.handle(q).status==409);
  assert(!gateway.has_active_mission());
}
