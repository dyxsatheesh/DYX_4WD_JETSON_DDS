#include "dyx_rpp_cpp17_core/px4_actuator_dds_intent.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace dyx::rpp;
#define CHECK(x) do { if (!(x)) { std::cerr << "FAIL line " << __LINE__ << ": " #x "\n"; return 1; } } while (false)
template<typename F> bool rejects(F f) { try { f(); } catch (const std::exception &) { return true; } return false; }
int main() {
  SprayOutput mission;
  mission.command=SprayCommand::press;mission.command_sequence=12;mission.requested_pwm_us=1400;
  const auto on=encode_spray_dds(mission,"runA|P0001|instance");
  CHECK(on.vehicle_command==187 && on.action==DdsActuatorAction::on);
  CHECK(on.actuator_set==1 && on.params[6]==0.0);
  CHECK(std::abs(on.params[0]-(-0.2))<1e-12);
  for(int i=1;i<6;++i) CHECK(std::isnan(on.params[static_cast<std::size_t>(i)]));
  mission.command=SprayCommand::release;mission.command_sequence=13;mission.requested_pwm_us=1200;
  const auto off=encode_spray_dds(mission,"runA|P0001|instance");
  CHECK(off.action==DdsActuatorAction::off && std::abs(off.params[0]-(-0.6))<1e-12);
  ManualSprayOutput manual;
  manual.command=SprayCommand::press;manual.command_sequence=4;manual.requested_pwm_us=1400;
  auto test=encode_manual_spray_dds(manual,"manual-operator-42");
  CHECK(test.owner==DdsActuatorOwner::manual_spray && test.actuator_set==1);
  LaserOutput laser;
  laser.command=LaserCommand::on;laser.command_sequence=8;
  laser.aux_channel=6;laser.actuator_set=2;laser.normalized_value=1.0;
  auto lon=encode_laser_dds(laser,"runB|P0004|laser");
  CHECK(lon.params[1]==1.0 && std::isnan(lon.params[0]));
  laser.command=LaserCommand::off;laser.command_sequence=9;laser.normalized_value=-1.0;
  auto loff=encode_laser_dds(laser,"runB|P0004|laser");
  CHECK(loff.params[1]==-1.0 && loff.params[6]==0.0);
  CHECK(rejects([&] { (void)encode_px4_actuator_intent(DdsActuatorOwner::laser,DdsActuatorAction::on,0,"x",1,2); }));
  CHECK(rejects([&] { (void)encode_px4_actuator_intent(DdsActuatorOwner::laser,DdsActuatorAction::on,1,"",1,2); }));
  CHECK(rejects([&] { (void)encode_px4_actuator_intent(DdsActuatorOwner::laser,DdsActuatorAction::on,1,"x",2,2); }));
  CHECK(rejects([&] { (void)encode_px4_actuator_intent(DdsActuatorOwner::laser,DdsActuatorAction::on,1,"x",1,1); }));
  CHECK(rejects([&] { (void)encode_spray_dds(SprayOutput{},"run"); }));
  CHECK(rejects([&] { (void)encode_laser_dds(LaserOutput{},"run"); }));
  CHECK(rejects([&] { (void)pwm_to_normalized(2400,Px4ActuatorDdsConfig{}); }));
  CHECK(rejects([&] { auto c=Px4ActuatorDdsConfig{};c.spray_pwm_min_us=1500;c.spray_pwm_max_us=1500;c.validate(); }));
  std::cout << "DDS VehicleCommand(187) normalized mapping/pass-through isolation passed\n";
}
