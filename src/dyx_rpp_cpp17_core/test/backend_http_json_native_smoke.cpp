#include "backend_http_fixture.hpp"
int main() {
  using namespace dyx::rpp::http_json;
  assert(quoted(std::string("a\"\\\n\r\t")+char(1))=="\"a\\\"\\\\\\n\\r\\t\\u0001\"");
  assert(valid_utf8("rover"));
  assert(valid_utf8(std::string("\xE2\x82\xAC",3)));
  assert(!valid_utf8(std::string("\xC0\xAF",2))); // overlong
  assert(!valid_utf8(std::string("\xE2\x82",2))); // truncated
  assert(!valid_utf8(std::string("\xED\xA0\x80",3))); // surrogate
  bool threw=false;
  try { (void)quoted(std::string("\xFF",1)); }
  catch(const std::invalid_argument&) {threw=true;}
  assert(threw);
  BackendMissionApiResponse m;
  m.code="A\"\n\\";
  m.reason="\"},\"privileged\":true";
  m.stage_name="ready";
  m.total=2;
  auto json=mission_json(m);
  assert(json.find("\"reason\":\"\\\"},\\\"privileged\\\":true\"")!=std::string::npos);
  assert(json.find("\"privileged\":true")==std::string::npos);
  assert(json.find("\"total\":2")!=std::string::npos);
  BackendHttpWireResponse bad{400,error_json("BAD_DATA")};
  assert(bad.http11_bytes().find("HTTP/1.1 400 Bad Request\r\n")==0);
  bad.status=201;
  threw=false;
  try{(void)bad.http11_bytes();}catch(const std::invalid_argument&){threw=true;}
  assert(threw);
  NativeMissionCommandGateway g;
  NativeBackendMissionApi api(g);
  NativeBackendLiveStatus live(g);
  NativeBackendHttpWire small(api,live,{512,1024});
  auto wire=small.handle(http_read(kBackendLiveStatusPath,10.0));
  assert(wire.status==200 || wire.status==500); // hard cap is fail-closed
  if (wire.status==500) assert(wire.body.find("RESPONSE_TOO_LARGE")!=std::string::npos);
}
