#include "safety.hpp"
#include <cassert>
#include <string>
int main(){
  dyx::preflight::InhibitState i;
  assert(!i.motion_enabled());assert(!i.spray_enabled());
  assert(!i.dds_writer_attached());assert(!i.physically_stopped_confirmed());
  assert(i.json().find("\"physical_stop_confirmed\":false")!=std::string::npos);
}
