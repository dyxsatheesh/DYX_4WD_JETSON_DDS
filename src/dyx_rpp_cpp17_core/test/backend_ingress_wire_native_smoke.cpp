#include "backend_ingress_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionApi api(gateway);
  NativeBackendLiveStatus live(gateway);
  NativeBackendHttpWire wire(api,live);
  NativeOfflineControlSessions sessions;
  NativeOfflineTrustedRequestAdapter adapter(sessions,wire);
  auto att=operator_a();
  BackendTrustedHttpRequest read;
  read.method="GET";read.path=kBackendMissionStatusPath;read.host_received_sec=1.0;
  read.principal={true,true,true,"CLIENT_SUPPLIED_FORGED_SESSION"};
  auto bad=att;bad.authentication_verified=false;
  const auto denied=adapter.handle(bad,0,read);
  assert(denied.status==401);
  assert(!denied.network_io_performed && !denied.physical_command_emitted);
  assert(adapter.handle(viewer(),0,read).status==200);
  const auto lease=sessions.acquire(att,1.1);
  assert(lease.allowed());
  BackendTrustedHttpRequest command;
  command.method="POST";command.path=kBackendMissionStartPath;
  command.host_received_sec=1.2;
  command.body="{\"mission_run_id\":\"RUN_INGRESS_0001\",\"sequence\":1}";
  command.principal={true,true,true,"FORGED_SESSION"};
  assert(adapter.handle(bad,lease.generation,command).status==401);
  // No mission loaded: the existing mission gateway correctly rejects START.
  const auto reject=adapter.handle(att,lease.generation,command);
  assert(reject.status==409 && !reject.physical_command_emitted);
  assert(adapter.handle(att,lease.generation,command).status==403); // replay
  command.body="{\"mission_run_id\":\"RUN_INGRESS_0001\",\"sequence\":2}";
  assert(adapter.handle(att,lease.generation-1,command).status==403);
  // Unknown paths and raw JSON authentication fields cannot bypass the gate.
  command.path="/api/v1/mission/activate";
  assert(adapter.handle(att,lease.generation,command).status==405);
  command.path=kBackendMissionStartPath;
  command.body="{\"mission_run_id\":\"RUN_INGRESS_0001\",\"sequence\":3,\"authenticated\":true}";
  assert(adapter.handle(att,lease.generation,command).status==400);
  // STOP is advisory only and does NOT rely on the lease.
  sessions.revoke();
  command.path=kBackendMissionStopPath;
  command.body="{\"mission_run_id\":\"RUN_INGRESS_0001\",\"sequence\":4}";
  const auto stop=adapter.handle(att,0,command);
  assert(stop.status==200 && !stop.network_io_performed && !stop.physical_command_emitted);
}
