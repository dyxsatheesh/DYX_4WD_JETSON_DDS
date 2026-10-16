#include "dyx_h6e/route_inventory.hpp"
#include <cassert>
#include <iostream>
int main(){
  using namespace dyx::h6e;
  const std::string with_prefix="mission_router = APIRouter(\n prefix=\"/api/mission\", tags=[\"mission\"]\n)\n@mission_router.post(\"/start\")\n";
  assert(router_prefix(with_prefix,"mission_router")=="/api/mission");
  assert(router_prefix("system_router = APIRouter(tags=[\"x\"])\n","system_router")=="");
  assert(quote("A\"B\\C\n")=="\"A\\\"B\\\\C\\n\"");
  bool rejected=false;
  try{router_prefix("other_router = APIRouter()", "mission_router");}
  catch(const std::exception&){rejected=true;}
  assert(rejected);
  const auto j=json(Inventory{});
  assert(j.find("\"production_ready\": false")!=std::string::npos);
  std::cout<<"H6E1 parser and fail-closed release status passed\n";
}
