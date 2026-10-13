#include "dyx_rpp_cpp17_core/dds_actuator_transport.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace dyx::rpp;
#define CHECK(x) do{if(!(x)){std::cerr<<"FAIL "<<__LINE__<<" " #x "\n";return 1;}}while(false)
template<class F> bool rejects(F f){try{f();}catch(const std::exception&){return true;}return false;}
int main(){
  const auto on=encode_px4_actuator_intent(DdsActuatorOwner::mission_spray,
     DdsActuatorAction::on,12,"RUN|PATH|P0001",-0.2,1);
  const auto w=px4_wire_command(on,12345001,1,1,1,191);
  CHECK(w.command==187 && w.timestamp_us==12345001 && w.from_external);
  CHECK(w.param1< -0.19999f && w.param1> -0.20001f);
  CHECK(std::isnan(w.param2)&&std::isnan(w.param3)&&std::isnan(w.param4));
  CHECK(std::isnan(w.param5)&&std::isnan(w.param6)&&w.param7==0.0f);
  CHECK(w.source_component==191 && w.target_component==1 && w.confirmation==0);
  const auto laser=encode_px4_actuator_intent(DdsActuatorOwner::laser,
      DdsActuatorAction::off,7,"RUN|P0002|LASER",-1.0,2);
  const auto lw=px4_wire_command(laser,12345003);
  CHECK(std::isnan(lw.param1)&&lw.param2==-1.0f&&std::isnan(lw.param3));
  CHECK(rejects([&]{(void)px4_wire_command(laser,0);}));
  CHECK(rejects([&]{(void)px4_wire_command(laser,2,0);}));
  auto dirty=on;dirty.params[3]=0.0;
  CHECK(rejects([&]{(void)px4_wire_command(dirty,2);}));
  dirty=on;dirty.local_sequence=0;
  CHECK(rejects([&]{(void)px4_wire_command(dirty,2);}));
  dirty=on;dirty.params[0]=1.0;
  CHECK(rejects([&]{(void)px4_wire_command(dirty,2);}));
  std::cout<<"DDS VehicleCommand wire fields and unrelated set isolation passed\n";
}
