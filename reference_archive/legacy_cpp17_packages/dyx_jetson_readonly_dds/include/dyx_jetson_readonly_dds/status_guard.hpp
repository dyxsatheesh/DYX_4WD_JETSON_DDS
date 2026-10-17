#pragma once
#include <cmath>
#include <cstdint>
namespace dyx::readonly_dds {
// Pure observation contract: no method can authorize or emit a command.
struct State {
  bool status_seen{false};
  bool position_seen{false};
  bool px4_armed{false};
  bool px4_offboard{false};
  bool position_valid{false};
  double last_status_sec{-1};
  double last_position_sec{-1};
  bool output_allowed() const noexcept { return false; }
  bool observation_fresh(double now,double max_age=0.5) const noexcept {
    return std::isfinite(now) && std::isfinite(max_age) && max_age>0.0 &&
      status_seen && position_seen && position_valid &&
      now>=last_status_sec && now>=last_position_sec &&
      now-last_status_sec<=max_age && now-last_position_sec<=max_age;
  }
  void observe_status(bool armed,bool offboard,double t) noexcept {
    if(!std::isfinite(t)||t<0 || (status_seen&&t<last_status_sec)){
      status_seen=false;px4_armed=false;px4_offboard=false;return;}
    status_seen=true;px4_armed=armed;px4_offboard=offboard;last_status_sec=t;
  }
  void observe_position(bool valid,double t) noexcept {
    if(!std::isfinite(t)||t<0 || (position_seen&&t<last_position_sec)){
      position_seen=false;position_valid=false;return;}
    position_seen=true;position_valid=valid;last_position_sec=t;
  }
};
}
