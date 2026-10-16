#include "test_fixtures.hpp"
#include <cassert>
#include <stdexcept>
#include <string>
template<class F>bool reject(F f){try{f();return false;}catch(const std::exception&){return true;}}
int main(){
  assert(mission_contract(StrictJson(mission_fixture).parse()));
  assert(telemetry_contract(StrictJson(telemetry_fixture).parse()));
  for(const std::string s:{"{} trailing","{", "{\"x\":1,\"x\":2}",
     "{\"x\":01}","{\"x\":1e}","{\"x\":NaN}","{\"x\":\"\\uD800\"}",
     "{\"x\":\"\\uDC00\"}","{\"x\":\"\\x\"}","[]{}","{\"x\":true,}"}){
    assert(reject([&]{(void)StrictJson(s).parse();}));
  }
  assert(reject([&]{(void)StrictJson(std::string(262145,'a')).parse();}));
  assert(reject([&]{(void)StrictJson(std::string("\"\xff\"",3)).parse();}));
  assert(StrictJson("{\"\\u0061\":1}").parse().find("a"));
  assert(reject([&]{(void)StrictJson("{\"a\":1,\"\\u0061\":2}").parse();}));
  NativeLegacyReadOnlyAdapter a;
  auto incomplete=snapshot("{\"success\":true,\"mission\":{},\"report\":{}}");
  assert(!a.ingest(Stream::mission,incomplete) && a.fault_latched());
  NativeLegacyReadOnlyAdapter b;
  auto stale=snapshot(telemetry_fixture);stale.body.replace(stale.body.find("\"safety\":{\"emergency_stop\":true"),
    std::string("\"safety\":{\"emergency_stop\":true").size(),
    "\"safety\":{\"emergency_stop\":false");
  assert(!b.ingest(Stream::telemetry,stale) && b.fault_latched());
  // Prove time rollback latches and does not release previous potentially stale data.
  NativeLegacyReadOnlyAdapter c;assert(c.ingest(Stream::mission,snapshot(mission_fixture)));
  assert(c.handle(request(mission_path,99.0)).http_status==503);
  assert(c.fault_latched());
  return 0;
}
