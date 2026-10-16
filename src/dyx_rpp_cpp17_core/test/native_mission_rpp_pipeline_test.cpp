#include "dyx_rpp_cpp17_core/native_mission_rpp_pipeline.hpp"

#include <cassert>
#include <iostream>

using namespace dyx::rpp;

static DdsMissionRuntimeOutput healthy_tick(double now) {
  DdsMissionRuntimeOutput t{};

  t.telemetry.safety.now_sec = now;
  t.telemetry.safety.gnss_fix_type = 6;
  t.telemetry.safety.battery_voltage_v = 24.9;
  t.telemetry.safety.estimator_valid = true;
  t.telemetry.safety.obstacle_clear = true;
  t.telemetry.safety.obstacle_age_sec = 0.0;

  t.telemetry.gps_sample_fresh = true;
  t.telemetry.battery_sample_fresh = true;
  t.telemetry.telemetry_integrity_fault = false;

  t.decision.safety.motion_allowed = true;
  t.decision.safety.hold = SafetyHold::none;
  t.decision.safety.manual_ack_required = false;

  t.decision.mission.stage = MissionStage::approaching;
  t.decision.mission.hold_zero = false;

  t.decision.preview_forward_permitted = true;
  t.preview_forward_permitted = true;

  return t;
}

static NativeMissionRppInput valid_request(
    double now, std::uint64_t seq) {
  NativeMissionRppInput in{};

  in.sequence = seq;
  in.guidance.now_sec = now;
  in.guidance.dt_sec = 0.02;
  in.guidance.rover_yaw_enu_rad = 0.0;
  in.guidance.target_yaw_enu_rad = 0.05;
  in.guidance.requested_forward_speed_mps = 0.6;
  in.guidance.motion_enabled = true;
  in.guidance.telemetry_fresh = true;

  in.trusted_operator_lease = true;
  in.trusted_estop_released = true;
  in.calibrated_marking_reference = true;

  return in;
}

int main() {
  NativeMissionRppPipeline p;

  // Startup is locked.
  assert(!p.step(
      healthy_tick(1.0),
      valid_request(1.0, 1)).permitted);

  assert(!p.begin_new_disarmed_run(false, true));
  assert(!p.begin_new_disarmed_run(true, false));
  assert(p.begin_new_disarmed_run(true, true));

  // Synthetic healthy mission and safety decision.
  assert(p.step(
      healthy_tick(2.0),
      valid_request(2.0, 1)).permitted);

  // Immediate RTK FLOAT rejection.
  auto floating = healthy_tick(2.02);
  floating.telemetry.safety.gnss_fix_type = 5;

  assert(!p.step(
      floating,
      valid_request(2.02, 2)).permitted);

  // Return to FIXED cannot restart without explicit recovery.
  assert(!p.step(
      healthy_tick(2.04),
      valid_request(2.04, 3)).permitted);

  // Stale or mismatched mission tick.
  assert(p.begin_new_disarmed_run(true, true));

  assert(!p.step(
      healthy_tick(3.0),
      valid_request(3.2, 1)).permitted);

  // Positive telemetry but missing mission authority.
  assert(p.begin_new_disarmed_run(true, true));

  auto paused = healthy_tick(4.0);
  paused.decision.mission.stage = MissionStage::paused;

  assert(!p.step(
      paused,
      valid_request(4.0, 1)).permitted);

  // Battery below critical threshold.
  assert(p.begin_new_disarmed_run(true, true));

  auto battery = healthy_tick(5.0);
  battery.telemetry.safety.battery_voltage_v = 21.9;

  assert(!p.step(
      battery,
      valid_request(5.0, 1)).permitted);

  // Missing validated obstacle evidence.
  assert(p.begin_new_disarmed_run(true, true));

  auto obstacle = healthy_tick(6.0);
  obstacle.telemetry.safety.obstacle_age_sec = 1.0;

  assert(!p.step(
      obstacle,
      valid_request(6.0, 1)).permitted);

  // Operator lease must be trusted.
  assert(p.begin_new_disarmed_run(true, true));

  auto lease = valid_request(7.0, 1);
  lease.trusted_operator_lease = false;

  assert(!p.step(
      healthy_tick(7.0),
      lease).permitted);

  // Rear marking reference calibration is mandatory.
  assert(p.begin_new_disarmed_run(true, true));

  auto marking = valid_request(8.0, 1);
  marking.calibrated_marking_reference = false;

  assert(!p.step(
      healthy_tick(8.0),
      marking).permitted);

  // Physical E-stop release evidence must be trusted.
  assert(p.begin_new_disarmed_run(true, true));

  auto estop = valid_request(9.0, 1);
  estop.trusted_estop_released = false;

  assert(!p.step(
      healthy_tick(9.0),
      estop).permitted);

  std::cout
      << "PASS: Native mission-to-RPP safety integration\n";
}
