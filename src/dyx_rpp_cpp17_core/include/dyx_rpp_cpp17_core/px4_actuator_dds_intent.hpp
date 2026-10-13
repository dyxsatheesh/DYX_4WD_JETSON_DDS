#pragma once
// Phase 2H3A: non-actuating C++17 PX4 VehicleCommand (187) payload contract.
// Produces DATA only. A separate ROS 2 DDS transport must publish/call the
// correct PX4 v1.17 endpoint, uniquely correlate responses and enforce safety.
// DO_SET_ACTUATOR uses NORMALIZED [-1, +1] values, not PWM microseconds.
#include "dyx_rpp_cpp17_core/spray_transaction.hpp"
#include "dyx_rpp_cpp17_core/manual_spray.hpp"
#include "dyx_rpp_cpp17_core/laser_marking.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace dyx::rpp {

enum class DdsActuatorOwner { mission_spray=1, manual_spray=2, laser=3 };
enum class DdsActuatorAction { on, off };
struct Px4ActuatorDdsConfig {
  unsigned spray_set{1};  // AUX5: PWM_AUX_FUNC5=301, requires device verification
  unsigned laser_set{2};  // AUX6: PWM_AUX_FUNC6=302, requires device verification
  int spray_pwm_min_us{1000};
  int spray_pwm_max_us{2000};
  void validate() const {
    if (spray_set != 1 || laser_set != 2 || spray_set == laser_set ||
        spray_pwm_min_us < 800 || spray_pwm_max_us > 2200 ||
        spray_pwm_max_us - spray_pwm_min_us < 100)
      throw std::invalid_argument("unverified PX4 actuator set/PWM calibration");
  }
};
struct DdsActuatorIntent {
  DdsActuatorOwner owner{DdsActuatorOwner::mission_spray};
  DdsActuatorAction action{DdsActuatorAction::off};
  std::uint64_t local_sequence{0};
  std::string identity{};
  std::uint16_t vehicle_command{187}; // MAV_CMD_DO_SET_ACTUATOR
  // Parameters 1..6: normalized output or NaN (untouched actuator set).
  // Param 7: first actuator group index = 0. The ROS layer is responsible
  // for converting to VehicleCommand's mixed float32/float64 field types.
  std::array<double,7> params{};
  unsigned actuator_set{0};
  double normalized_value{0};
};

inline DdsActuatorIntent encode_px4_actuator_intent(
    DdsActuatorOwner owner, DdsActuatorAction action,
    std::uint64_t seq, const std::string &identity,
    double normalized, unsigned set) {
  if (seq==0 || identity.empty() || identity.size()>256 ||
      !std::isfinite(normalized) || normalized < -1.0 || normalized > 1.0 ||
      set<1 || set>6 ||
      (owner!=DdsActuatorOwner::mission_spray &&
       owner!=DdsActuatorOwner::manual_spray &&
       owner!=DdsActuatorOwner::laser) ||
      (action!=DdsActuatorAction::on && action!=DdsActuatorAction::off) ||
      (owner==DdsActuatorOwner::laser && set!=2) ||
      (owner!=DdsActuatorOwner::laser && set!=1))
    throw std::invalid_argument("invalid actuator intent or identity");
  DdsActuatorIntent result;
  result.owner=owner;result.action=action;
  result.local_sequence=seq;result.identity=identity;
  result.actuator_set=set;result.normalized_value=normalized;
  result.params.fill(std::numeric_limits<double>::quiet_NaN());
  result.params[set-1]=normalized;
  result.params[6]=0.0;
  return result;
}
inline double pwm_to_normalized(int pwm_us, const Px4ActuatorDdsConfig &cfg) {
  cfg.validate();
  if (pwm_us < cfg.spray_pwm_min_us || pwm_us > cfg.spray_pwm_max_us)
    throw std::invalid_argument("PWM intent outside independently calibrated PX4 range");
  return 2.0*(static_cast<double>(pwm_us)-cfg.spray_pwm_min_us)/
            (cfg.spray_pwm_max_us-cfg.spray_pwm_min_us)-1.0;
}
inline DdsActuatorIntent encode_spray_dds(const SprayOutput &output,
    const std::string &run_scoped_identity, Px4ActuatorDdsConfig cfg={}) {
  if (output.command==SprayCommand::none || output.requested_pwm_us==0)
    throw std::invalid_argument("no spray command to encode");
  return encode_px4_actuator_intent(DdsActuatorOwner::mission_spray,
      output.command==SprayCommand::press?DdsActuatorAction::on:DdsActuatorAction::off,
      output.command_sequence,run_scoped_identity,
      pwm_to_normalized(output.requested_pwm_us,cfg),cfg.spray_set);
}
inline DdsActuatorIntent encode_manual_spray_dds(const ManualSprayOutput &output,
    const std::string &request_id, Px4ActuatorDdsConfig cfg={}) {
  if (output.command==SprayCommand::none || output.requested_pwm_us==0)
    throw std::invalid_argument("no manual spray command to encode");
  return encode_px4_actuator_intent(DdsActuatorOwner::manual_spray,
      output.command==SprayCommand::press?DdsActuatorAction::on:DdsActuatorAction::off,
      output.command_sequence,request_id,
      pwm_to_normalized(output.requested_pwm_us,cfg),cfg.spray_set);
}
inline DdsActuatorIntent encode_laser_dds(const LaserOutput &output,
    const std::string &run_scoped_identity, Px4ActuatorDdsConfig cfg={}) {
  cfg.validate();
  if (output.command==LaserCommand::none || output.actuator_set!=static_cast<int>(cfg.laser_set) ||
      output.aux_channel!=6)
    throw std::invalid_argument("no validated laser command to encode");
  return encode_px4_actuator_intent(DdsActuatorOwner::laser,
      output.command==LaserCommand::on?DdsActuatorAction::on:DdsActuatorAction::off,
      output.command_sequence,run_scoped_identity,
      output.normalized_value,cfg.laser_set);
}
} // namespace dyx::rpp
