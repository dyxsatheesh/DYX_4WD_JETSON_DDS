#include "dyx_mission_manager/native_px4_ingress_observation.hpp"
#include <stdexcept>
#include <iostream>
using dyx::http_backend::NativePx4IngressObservation;
static void require(bool good) { if(!good) throw std::runtime_error("ingress diagnostic regression"); }
int main() {
  NativePx4IngressObservation p;
  auto first = p.snapshot(10.0);
  require(!first["vehicle"]["received"].get<bool>());
  require(!first["vehicle"]["recent_transport"].get<bool>());
  require(!first["authorizes_motion"].get<bool>());
  p.observe_vehicle(10.0);
  p.observe_battery(10.1);  // Raw zero-volt samples still count as receipt.
  p.observe_position(10.2); // An invalid EKF sample still counts as receipt.
  auto sample = p.snapshot(10.3);
  require(sample["vehicle"]["received"].get<bool>());
  require(sample["vehicle"]["recent_transport"].get<bool>());
  require(sample["battery"]["recent_transport"].get<bool>());
  require(sample["position"]["messages_observed"].get<int>() == 1);
  require(!sample["gps"]["received"].get<bool>());
  require(!sample["rover_rate"]["received"].get<bool>());
  require(!p.snapshot(12.0)["vehicle"]["recent_transport"].get<bool>());
  require(!p.snapshot(10.3)["authorizes_motion"].get<bool>());
  p.observe_vehicle(-1.0);
  require(p.snapshot(10.4)["vehicle"]["messages_observed"].get<int>() == 1);
  std::cout << "PASS: raw telemetry receipt diagnostic, no actuation" << std::endl;
}
