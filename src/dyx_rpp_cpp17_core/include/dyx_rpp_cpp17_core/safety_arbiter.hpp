#pragma once
// Phase 2H1A. Pure C++17 safety decisions, NEVER hardware commands.
// Adapters must validate the origin and monotonic clock of each input.
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace dyx::rpp {
enum class SafetyHold { none, disabled, emergency_stop, bad_clock,
  battery_stale, battery_critical, gnss_stale, rtk_lost, rtk_float_paused,
  corrections_stale, obstacle_stale, obstacle_detected, px4_stale,
  px4_not_offboard, estimator_invalid, manual_ack_required };
inline const char *safety_hold_name(SafetyHold h) {
  switch(h) {
    case SafetyHold::none: return "READY";
    case SafetyHold::disabled: return "MISSION_DISABLED";
    case SafetyHold::emergency_stop: return "EMERGENCY_STOP";
    case SafetyHold::bad_clock: return "BAD_MONOTONIC_TIME";
    case SafetyHold::battery_stale: return "BATTERY_TELEMETRY_STALE";
    case SafetyHold::battery_critical: return "BATTERY_CRITICAL_LOCKOUT";
    case SafetyHold::gnss_stale: return "GNSS_TELEMETRY_STALE";
    case SafetyHold::rtk_lost: return "RTK_LOST_MANUAL_RECOVERY";
    case SafetyHold::rtk_float_paused: return "RTK_FLOAT_CONFIRMED";
    case SafetyHold::corrections_stale: return "RTCM_CORRECTIONS_STALE";
    case SafetyHold::obstacle_stale: return "OBSTACLE_TELEMETRY_STALE";
    case SafetyHold::obstacle_detected: return "OBSTACLE_DETECTED";
    case SafetyHold::px4_stale: return "PX4_TELEMETRY_STALE";
    case SafetyHold::px4_not_offboard: return "PX4_OFFBOARD_NOT_READY";
    case SafetyHold::estimator_invalid: return "ESTIMATOR_INVALID";
    case SafetyHold::manual_ack_required: return "MANUAL_RECOVERY_REQUIRED";
  }
  return "INVALID_HOLD";
}
struct SafetyConfig {
  double battery_full_voltage_v{26.50},battery_critical_voltage_v{22.00};
  double battery_replacement_min_v{22.50},battery_debounce_sec{30.0};
  double battery_sample_max_age_sec{1.5},battery_sample_max_gap_sec{1.5};
  double gnss_sample_max_age_sec{1.5},gnss_sample_max_gap_sec{1.5};
  double float_confirm_sec{5.0},px4_max_age_sec{0.5};
  double obstacle_max_age_sec{0.5},rtcm_max_age_sec{20.0};
  int required_fix_type{6},float_fix_type{5};
  void validate() const {
    for (double v:{battery_full_voltage_v,battery_critical_voltage_v,
                   battery_replacement_min_v,battery_debounce_sec,
                   battery_sample_max_age_sec,battery_sample_max_gap_sec,
                   gnss_sample_max_age_sec,gnss_sample_max_gap_sec,
                   float_confirm_sec,px4_max_age_sec,obstacle_max_age_sec,
                   rtcm_max_age_sec})
      if (!std::isfinite(v)||v<=0) throw std::invalid_argument("invalid safety config");
    if (battery_full_voltage_v<=battery_replacement_min_v ||
        battery_replacement_min_v<=battery_critical_voltage_v ||
        required_fix_type==float_fix_type)
      throw std::invalid_argument("safety thresholds conflict");
  }
};
struct SafetyInput {
  double now_sec{0};
  bool mission_running{false},operator_motion_enabled{false},emergency_stop{true};
  bool px4_offboard_armed{false},estimator_valid{false};
  double px4_age_sec{std::numeric_limits<double>::infinity()};
  bool obstacle_clear{false};
  double obstacle_age_sec{std::numeric_limits<double>::infinity()};
  double battery_voltage_v{0},battery_sample_time_sec{-1};
  int gnss_fix_type{0};
  double gnss_sample_time_sec{-1};
  bool correction_stream_healthy{false};
  double correction_age_sec{std::numeric_limits<double>::infinity()};
};
struct SafetyOutput {
  bool motion_allowed{false},auto_resume_rtk_float{false};
  bool rtk_float_confirming{false},battery_debounce_pending{false};
  bool battery_critical_latched{false},manual_ack_required{false};
  bool rc_allowed{true},telemetry_must_remain_live{true},stop_allowed{true};
  bool start_resume_next_skip_allowed{false},battery_report_valid{false};
  double battery_report_percent{0};
  SafetyHold hold{SafetyHold::disabled};
  const char *reason{"MISSION_DISABLED"};
};
class NativeSafetyArbiter final {
 public:
  explicit NativeSafetyArbiter(SafetyConfig config={}) : cfg_(config){cfg_.validate();}
  SafetyOutput step(const SafetyInput &s){
    SafetyOutput o;
    if (!std::isfinite(s.now_sec)||s.now_sec<0 || (last_now_ && s.now_sec<*last_now_)){
      manual_ack_=true; return block(o,SafetyHold::bad_clock);
    }
    last_now_=s.now_sec;
    if (s.emergency_stop) estop_latched_=true;
    const bool battery_fresh=sample_fresh(s.battery_sample_time_sec,s.now_sec,cfg_.battery_sample_max_age_sec)
        && std::isfinite(s.battery_voltage_v) && s.battery_voltage_v>0
        && (!last_battery_sample_ || s.battery_sample_time_sec>=*last_battery_sample_);
    if (battery_fresh){
      latest_battery_voltage_=s.battery_voltage_v;
      latest_battery_stamp_=s.battery_sample_time_sec;
      if (!last_battery_sample_ || s.battery_sample_time_sec>*last_battery_sample_){
        if (last_battery_sample_ && s.battery_sample_time_sec-*last_battery_sample_>
                                   cfg_.battery_sample_max_gap_sec) reset_battery_window();
        last_battery_sample_=s.battery_sample_time_sec;
        if (s.battery_voltage_v>cfg_.battery_critical_voltage_v) reset_battery_window();
        else {
          if (!battery_low_since_) battery_low_since_=s.battery_sample_time_sec;
          ++battery_distinct_samples_;
          if (battery_distinct_samples_>=2 &&
              s.battery_sample_time_sec-*battery_low_since_>=cfg_.battery_debounce_sec)
            battery_latched_=true;
        }
      }
      o.battery_report_valid=true;
      o.battery_report_percent=10.0+90.0*std::clamp(
           (s.battery_voltage_v-cfg_.battery_critical_voltage_v)/
           (cfg_.battery_full_voltage_v-cfg_.battery_critical_voltage_v),0.0,1.0);
    }else {reset_battery_window();last_battery_sample_.reset();}
    o.battery_debounce_pending=battery_low_since_.has_value()&&!battery_latched_;
    o.battery_critical_latched=battery_latched_;
    if (o.battery_debounce_pending) o.battery_report_percent=11.0;

    const bool gnss_fresh=sample_fresh(s.gnss_sample_time_sec,s.now_sec,cfg_.gnss_sample_max_age_sec)
         && (!last_gnss_sample_ || s.gnss_sample_time_sec>=*last_gnss_sample_);
    if (gnss_fresh && (!last_gnss_sample_ || s.gnss_sample_time_sec>*last_gnss_sample_)){
      if (last_gnss_sample_ && s.gnss_sample_time_sec-*last_gnss_sample_>
                               cfg_.gnss_sample_max_gap_sec) float_since_.reset();
      last_gnss_sample_=s.gnss_sample_time_sec;
    }
    if (!gnss_fresh){float_since_.reset();last_gnss_sample_.reset();}
    bool recovered_float=false;
    if (gnss_fresh && s.gnss_fix_type==cfg_.required_fix_type){
      recovered_float=float_paused_;
      float_paused_=false;float_since_.reset();
    }else if (gnss_fresh && s.gnss_fix_type==cfg_.float_fix_type){
      if (!float_since_) float_since_=s.gnss_sample_time_sec;
      if (s.gnss_sample_time_sec-*float_since_>=cfg_.float_confirm_sec) float_paused_=true;
      o.rtk_float_confirming=!float_paused_;
    }else{
      float_since_.reset();
      if (s.mission_running && s.operator_motion_enabled) manual_ack_=true;
    }
    if (estop_latched_) return block(o,SafetyHold::emergency_stop);
    if (battery_latched_) return block(o,SafetyHold::battery_critical);
    if (!s.mission_running || !s.operator_motion_enabled) return block(o,SafetyHold::disabled);
    if (!battery_fresh){manual_ack_=true;return block(o,SafetyHold::battery_stale);}
    if (!gnss_fresh){manual_ack_=true;return block(o,SafetyHold::gnss_stale);}
    if (s.gnss_fix_type!=cfg_.required_fix_type && s.gnss_fix_type!=cfg_.float_fix_type){
      manual_ack_=true;return block(o,SafetyHold::rtk_lost);
    }
    if (float_paused_) return block(o,SafetyHold::rtk_float_paused);
    if (!s.correction_stream_healthy || !fresh_age(s.correction_age_sec,cfg_.rtcm_max_age_sec)){
      manual_ack_=true;return block(o,SafetyHold::corrections_stale);
    }
    if (!fresh_age(s.obstacle_age_sec,cfg_.obstacle_max_age_sec)){
      manual_ack_=true;return block(o,SafetyHold::obstacle_stale);
    }
    if (!s.obstacle_clear){manual_ack_=true;return block(o,SafetyHold::obstacle_detected);}
    if (!fresh_age(s.px4_age_sec,cfg_.px4_max_age_sec)){
      manual_ack_=true;return block(o,SafetyHold::px4_stale);
    }
    if (!s.px4_offboard_armed){manual_ack_=true;return block(o,SafetyHold::px4_not_offboard);}
    if (!s.estimator_valid){manual_ack_=true;return block(o,SafetyHold::estimator_invalid);}
    if (manual_ack_) return block(o,SafetyHold::manual_ack_required);
    o.motion_allowed=true;o.start_resume_next_skip_allowed=true;
    o.auto_resume_rtk_float=recovered_float;
    o.hold=SafetyHold::none;
    o.reason=o.rtk_float_confirming?"RTK_FLOAT_CONFIRMING_GRACE":"READY";
    return o;
  }
  bool acknowledge_manual_hold(bool authorized,bool px4_disarmed){
    if (!authorized||!px4_disarmed||estop_latched_||battery_latched_) return false;
    manual_ack_=false;return true;
  }
  bool release_emergency_stop(bool authorized,bool px4_disarmed,bool physical_estop_released){
    if (!authorized||!px4_disarmed||!physical_estop_released) return false;
    estop_latched_=false;manual_ack_=true;return true;
  }
  bool acknowledge_battery_replacement(bool authorized,bool px4_disarmed){
    if (!authorized||!px4_disarmed||estop_latched_||!latest_battery_voltage_||
        !latest_battery_stamp_||!last_now_||
        !sample_fresh(*latest_battery_stamp_,*last_now_,cfg_.battery_sample_max_age_sec)||
        *latest_battery_voltage_<cfg_.battery_replacement_min_v) return false;
    battery_latched_=false;reset_battery_window();last_battery_sample_.reset();
    manual_ack_=true;return true;
  }
  bool battery_locked()const{return battery_latched_;}
  bool manual_ack_required()const{return manual_ack_;}
 private:
  static bool sample_fresh(double stamp,double now,double max_age){
    return std::isfinite(stamp)&&stamp>=0&&stamp<=now&&now-stamp<=max_age;
  }
  static bool fresh_age(double age,double max_age){
    return std::isfinite(age)&&age>=0&&age<=max_age;
  }
  SafetyOutput block(SafetyOutput o,SafetyHold why)const{
    o.motion_allowed=false;o.start_resume_next_skip_allowed=false;
    o.auto_resume_rtk_float=false;o.hold=why;o.reason=safety_hold_name(why);
    o.manual_ack_required=manual_ack_||estop_latched_||battery_latched_;
    if (why==SafetyHold::emergency_stop) o.rc_allowed=false;
    return o;
  }
  void reset_battery_window(){battery_low_since_.reset();battery_distinct_samples_=0;}
  SafetyConfig cfg_{};
  std::optional<double> last_now_{},last_battery_sample_{},battery_low_since_{},
     latest_battery_voltage_{},latest_battery_stamp_{},last_gnss_sample_{},float_since_{};
  unsigned battery_distinct_samples_{0};
  bool estop_latched_{false},battery_latched_{false},manual_ack_{false},float_paused_{false};
};
} // namespace dyx::rpp
