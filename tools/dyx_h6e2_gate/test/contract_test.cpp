#include "test_fixtures.hpp"
#include <cassert>
#include <string>
int main(){
  NativeLegacyReadOnlyAdapter adapter;
  auto m=request(mission_path);
  assert(adapter.handle(m).http_status==503);
  m.principal={false,false};assert(adapter.handle(m).http_status==401);
  m.principal={true,false};assert(adapter.handle(m).http_status==403);
  m.principal={true,true};m.method="POST";assert(adapter.handle(m).http_status==405);
  m.method="GET";m.body="{}";assert(adapter.handle(m).http_status==400);
  m.body.clear();
  assert(adapter.ingest(Stream::mission,snapshot(mission_fixture)));
  auto result=adapter.handle(m);
  assert(result.http_status==200 && result.body==mission_fixture);
  assert(result.status_verified_snapshot && result.read_only && !result.physical_command_emitted);
  assert(adapter.handle(request(telemetry_path)).http_status==503);
  assert(adapter.ingest(Stream::telemetry,snapshot(telemetry_fixture,1,100.2)));
  auto t=adapter.handle(request(telemetry_path,100.3));
  assert(t.http_status==200 && t.body==telemetry_fixture && !t.physical_command_emitted);
  assert(adapter.handle(request(telemetry_path,101.0)).http_status==503);
  assert(adapter.handle(request(mission_path,102.01)).http_status==503);
  auto unk=request("/api/mission/start");assert(adapter.handle(unk).http_status==404);
  // Untrusted producer input fails closed. Faults cannot be cleared by GET.
  NativeLegacyReadOnlyAdapter b;
  auto fake=snapshot(mission_fixture);fake.provenance_verified=false;
  assert(!b.ingest(Stream::mission,fake) && b.fault_latched());
  assert(b.handle(request(mission_path)).http_status==503);
  assert(!b.reset_new_epoch(false,true,11));
  assert(!b.reset_new_epoch(true,false,11));
  assert(b.reset_new_epoch(true,true,11));
  assert(!b.fault_latched());
  assert(b.handle(request(mission_path)).http_status==503);
  assert(b.ingest(Stream::mission,snapshot(mission_fixture,1,110.0,11)));
  assert(!b.ingest(Stream::mission,snapshot(mission_fixture,1,110.1,11)));
  assert(b.fault_latched());
  assert(b.handle(request(mission_path,110.2)).http_status==503);
  return 0;
}
