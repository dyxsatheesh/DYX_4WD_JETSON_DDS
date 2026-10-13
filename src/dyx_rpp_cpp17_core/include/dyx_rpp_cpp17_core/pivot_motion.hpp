#pragma once
// ROS/PX4-INDEPENDENT parity port of rpp_controller/motion_state_machine.py.
// The state machine emits *directives*, never wheel, yaw-rate, or DDS commands.
// The adapter must still enforce RTK/obstacle/battery/e-stop and identity gates.

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::rpp {

constexpr double pivot_pi = 3.141592653589793238462643383279502884;

enum class MotionState {TRACK, CORNER_APPROACH, BRAKE_TO_ANCHOR, STOP_SETTLE,
                        PIVOT, PIVOT_SETTLE, POSITION_CHECK, RECENTER, REALIGN,
                        RECAPTURE, HOLD_FAIL};
enum class MotionDirective {TRACK, CORNER_APPROACH, BRAKE_TO_ANCHOR, HOLD_ZERO,
                            PIVOT, RECENTER, REALIGN, RECAPTURE, HOLD_FAIL};

inline const char *motion_name(MotionState s) {
  switch (s) {
    case MotionState::TRACK: return "track";
    case MotionState::CORNER_APPROACH: return "corner_approach";
    case MotionState::BRAKE_TO_ANCHOR: return "brake_to_anchor";
    case MotionState::STOP_SETTLE: return "stop_settle";
    case MotionState::PIVOT: return "pivot";
    case MotionState::PIVOT_SETTLE: return "pivot_settle";
    case MotionState::POSITION_CHECK: return "position_check";
    case MotionState::RECENTER: return "recenter";
    case MotionState::REALIGN: return "realign";
    case MotionState::RECAPTURE: return "recapture";
    case MotionState::HOLD_FAIL: return "hold_fail";
  }
  return "unknown";
}
inline const char *motion_name(MotionDirective d) {
  switch (d) {
    case MotionDirective::TRACK: return "track";
    case MotionDirective::CORNER_APPROACH: return "corner_approach";
    case MotionDirective::BRAKE_TO_ANCHOR: return "brake_to_anchor";
    case MotionDirective::HOLD_ZERO: return "hold_zero";
    case MotionDirective::PIVOT: return "pivot";
    case MotionDirective::RECENTER: return "recenter";
    case MotionDirective::REALIGN: return "realign";
    case MotionDirective::RECAPTURE: return "recapture";
    case MotionDirective::HOLD_FAIL: return "hold_fail";
  }
  return "unknown";
}

struct PivotMotionConfig {
  double pivot_anchor_tolerance_m{0.020};
  double pivot_recenter_threshold_m{0.020};
  double stop_speed_tolerance_mps{0.030};
  double stop_yaw_rate_tolerance_radps{0.050};
  double release_heading_tolerance_rad{2.0*pivot_pi/180.0};
  double stop_settle_sec{0.20};
  double pivot_release_settle_sec{0.20};
  double control_dt_max_sec{0.10};
  double brake_timeout_sec{8.0};
  double pivot_timeout_sec{10.0};
  double recenter_timeout_sec{5.0};
  double realign_timeout_sec{10.0};
  double recapture_timeout_sec{8.0};
  int max_recenter_attempts{2};

  void validate() const {
    for (double v : {pivot_anchor_tolerance_m,stop_speed_tolerance_mps,
                     stop_yaw_rate_tolerance_radps,release_heading_tolerance_rad,
                     stop_settle_sec,pivot_release_settle_sec,control_dt_max_sec,
                     brake_timeout_sec,pivot_timeout_sec,recenter_timeout_sec,
                     realign_timeout_sec,recapture_timeout_sec}) {
      if (!std::isfinite(v) || v<=0.0) throw std::invalid_argument("pivot config positive finite field");
    }
    if (!std::isfinite(pivot_recenter_threshold_m) ||
        std::abs(pivot_recenter_threshold_m-pivot_anchor_tolerance_m)>1e-12 ||
        release_heading_tolerance_rad>pivot_pi || max_recenter_attempts<0) {
      throw std::invalid_argument("pivot config geometry/recenter limit");
    }
  }
};

struct PivotMotionInput {
  double monotonic_time_sec{},dt_sec{},anchor_radial_error_m{};
  double measured_linear_speed_mps{},measured_yaw_rate_radps{},heading_error_rad{};
  bool telemetry_fresh{};
  bool pivot_requested{false},brake_to_anchor_requested{false},recapture_complete{false};
};
struct StopCertificate {
  bool position_ok{},linear_speed_ok{},yaw_rate_ok{},telemetry_fresh{};
  double held_sec{},required_sec{};
  bool valid{};
};
struct ReleaseCertificate {
  bool position_ok{},heading_ok{},linear_speed_ok{},yaw_rate_ok{},telemetry_fresh{};
  double held_sec{},required_sec{};
  bool valid{};
};
struct PivotMotionResult {
  MotionState state{},previous_state{};
  MotionDirective directive{};
  std::string transition_reason{};
  StopCertificate stop_certificate{};
  ReleaseCertificate release_certificate{};
  double max_pivot_drift_m{};
  int recenter_attempts{};
  double bounded_dt_sec{},state_elapsed_sec{};
  bool failed{};
};

class VerifiedPivotStateMachine {
public:
  explicit VerifiedPivotStateMachine(PivotMotionConfig config = {}) : c_(config) {
    c_.validate(); reset();
  }
  MotionState state() const {return state_;}
  void reset(double monotonic_time_sec=0.0) {
    if (!std::isfinite(monotonic_time_sec) || monotonic_time_sec<0.0)
      throw std::invalid_argument("invalid reset time");
    state_=MotionState::TRACK;
    state_enter_time_sec_=last_time_sec_=monotonic_time_sec;
    alignment_start_time_sec_.reset();
    stop_settle_destination_=MotionState::PIVOT;
    stop_held_sec_=release_held_sec_=max_pivot_drift_m_=0.0;
    recenter_attempts_=0;
  }
  PivotMotionResult step(const PivotMotionInput &s) {
    validate(s);
    if(s.monotonic_time_sec<last_time_sec_)
      throw std::invalid_argument("monotonic_time_sec must not move backwards");
    last_time_sec_=s.monotonic_time_sec;
    const double dt=std::min(std::max(s.dt_sec,0.0),c_.control_dt_max_sec);
    const MotionState previous=state_;
    std::string reason;
    const bool position_ok=s.telemetry_fresh && s.anchor_radial_error_m<=c_.pivot_anchor_tolerance_m;
    const bool speed_ok=s.telemetry_fresh && std::abs(s.measured_linear_speed_mps)<=c_.stop_speed_tolerance_mps;
    const bool yaw_rate_ok=s.telemetry_fresh && std::abs(s.measured_yaw_rate_radps)<=c_.stop_yaw_rate_tolerance_radps;
    const bool heading_ok=s.telemetry_fresh && std::abs(wrap_pi(s.heading_error_rad))<=c_.release_heading_tolerance_rad;
    const bool stop_ok=position_ok && speed_ok && yaw_rate_ok;
    const bool release_ok=stop_ok && heading_ok;
    stop_held_sec_=stop_ok?stop_held_sec_+dt:0.0;
    release_held_sec_=release_ok?release_held_sec_+dt:0.0;
    const double stop_held_result=stop_held_sec_,release_held_result=release_held_sec_;
    if(state_==MotionState::PIVOT || state_==MotionState::PIVOT_SETTLE ||
       state_==MotionState::POSITION_CHECK || state_==MotionState::RECENTER || state_==MotionState::REALIGN)
      max_pivot_drift_m_=std::max(max_pivot_drift_m_,s.anchor_radial_error_m);

    switch(state_) {
      case MotionState::TRACK:
        if(s.pivot_requested) {
          begin_maneuver(s.monotonic_time_sec);
          reason=transition(MotionState::CORNER_APPROACH,s.monotonic_time_sec,"pivot_requested");
        }
        break;
      case MotionState::CORNER_APPROACH:
        if(!s.telemetry_fresh && elapsed(s)>=c_.brake_timeout_sec)
          reason=fail(s.monotonic_time_sec,"corner_approach_telemetry_timeout");
        else if(!s.telemetry_fresh) {}
        else if(!s.pivot_requested)
          reason=transition(MotionState::TRACK,s.monotonic_time_sec,"pivot_request_cancelled");
        else if(s.brake_to_anchor_requested)
          reason=transition(MotionState::BRAKE_TO_ANCHOR,s.monotonic_time_sec,"corner_braking_required");
        break;
      case MotionState::BRAKE_TO_ANCHOR:
        if(position_ok) {
          stop_settle_destination_=MotionState::PIVOT;
          reason=transition(MotionState::STOP_SETTLE,s.monotonic_time_sec,"anchor_position_reached");
        } else if(elapsed(s)>=c_.brake_timeout_sec)
          reason=fail(s.monotonic_time_sec,"brake_to_anchor_timeout");
        break;
      case MotionState::STOP_SETTLE:
        if(!s.telemetry_fresh) {
          if(elapsed(s)>=c_.brake_timeout_sec) reason=fail(s.monotonic_time_sec,"stop_settle_telemetry_timeout");
        } else if(!position_ok) {
          const MotionState dst=stop_settle_destination_==MotionState::PIVOT?
              MotionState::BRAKE_TO_ANCHOR:MotionState::RECENTER;
          reason=dst==MotionState::RECENTER?
              enter_recenter(s.monotonic_time_sec,"anchor_lost_during_stop_settle"):
              transition(dst,s.monotonic_time_sec,"anchor_lost_during_stop_settle");
        } else if(stop_held_sec_>=c_.stop_settle_sec) {
          const MotionState dst=stop_settle_destination_;
          if(dst==MotionState::PIVOT || dst==MotionState::REALIGN)
            alignment_start_time_sec_=s.monotonic_time_sec;
          reason=transition(dst,s.monotonic_time_sec,"verified_stop_settled");
        }
        break;
      case MotionState::PIVOT:
        if(alignment_timed_out(s,c_.pivot_timeout_sec))
          reason=fail(s.monotonic_time_sec,"pivot_timeout");
        else if(s.telemetry_fresh && s.anchor_radial_error_m>c_.pivot_recenter_threshold_m)
          reason=enter_recenter(s.monotonic_time_sec,"pivot_drift_exceeded");
        else if(heading_ok)
          reason=transition(MotionState::PIVOT_SETTLE,s.monotonic_time_sec,"heading_entered_release_tolerance");
        break;
      case MotionState::PIVOT_SETTLE:
        if(alignment_timed_out(s,c_.pivot_timeout_sec))
          reason=fail(s.monotonic_time_sec,"pivot_settle_timeout");
        else if(!s.telemetry_fresh) {}
        else if(!position_ok)
          reason=enter_recenter(s.monotonic_time_sec,"position_bad_during_pivot_settle");
        else if(!heading_ok)
          reason=transition(MotionState::PIVOT,s.monotonic_time_sec,"heading_left_release_tolerance");
        else if(release_held_sec_>=c_.pivot_release_settle_sec)
          reason=transition(MotionState::POSITION_CHECK,s.monotonic_time_sec,"verified_pivot_release_settled");
        break;
      case MotionState::POSITION_CHECK:
        if(!position_ok) reason=enter_recenter(s.monotonic_time_sec,"position_check_failed");
        else if(!release_ok || release_held_sec_<c_.pivot_release_settle_sec) {
          alignment_start_time_sec_=s.monotonic_time_sec;
          reason=transition(MotionState::REALIGN,s.monotonic_time_sec,"release_evidence_changed");
        } else reason=transition(MotionState::RECAPTURE,s.monotonic_time_sec,"position_and_release_verified");
        break;
      case MotionState::RECENTER:
        if(position_ok) {
          stop_settle_destination_=MotionState::REALIGN;
          reason=transition(MotionState::STOP_SETTLE,s.monotonic_time_sec,"recenter_position_recovered");
        } else if(elapsed(s)>=c_.recenter_timeout_sec) {
          reason=recenter_attempts_<c_.max_recenter_attempts?
            enter_recenter(s.monotonic_time_sec,"recenter_retry"):
            fail(s.monotonic_time_sec,"recenter_attempts_exhausted");
        }
        break;
      case MotionState::REALIGN:
        if(alignment_timed_out(s,c_.realign_timeout_sec))
          reason=fail(s.monotonic_time_sec,"realign_timeout");
        else if(!s.telemetry_fresh) {}
        else if(!position_ok)
          reason=enter_recenter(s.monotonic_time_sec,"position_bad_during_realign");
        else if(release_held_sec_>=c_.pivot_release_settle_sec)
          reason=transition(MotionState::RECAPTURE,s.monotonic_time_sec,"verified_realign_release_settled");
        break;
      case MotionState::RECAPTURE:
        if(!s.telemetry_fresh)
          reason=fail(s.monotonic_time_sec,"telemetry_stale_during_recapture");
        else if(s.recapture_complete)
          reason=transition(MotionState::TRACK,s.monotonic_time_sec,"next_leg_recaptured");
        else if(elapsed(s)>=c_.recapture_timeout_sec)
          reason=fail(s.monotonic_time_sec,"recapture_timeout");
        break;
      case MotionState::HOLD_FAIL: break;
    }
    StopCertificate stop{position_ok,speed_ok,yaw_rate_ok,s.telemetry_fresh,
      stop_held_result,c_.stop_settle_sec,stop_ok && stop_held_result>=c_.stop_settle_sec};
    ReleaseCertificate release{position_ok,heading_ok,speed_ok,yaw_rate_ok,s.telemetry_fresh,
      release_held_result,c_.pivot_release_settle_sec,
      release_ok && release_held_result>=c_.pivot_release_settle_sec};
    MotionDirective directive=directive_for(state_);
    const auto is_moving=[](MotionState state){return state==MotionState::CORNER_APPROACH||
      state==MotionState::BRAKE_TO_ANCHOR||state==MotionState::PIVOT||
      state==MotionState::RECENTER||state==MotionState::REALIGN||state==MotionState::RECAPTURE;};
    if(!s.telemetry_fresh && state_!=MotionState::HOLD_FAIL && (is_moving(previous)||is_moving(state_)))
      directive=MotionDirective::HOLD_ZERO;
    return {state_,previous,directive,reason,stop,release,max_pivot_drift_m_,
            recenter_attempts_,dt,elapsed(s),state_==MotionState::HOLD_FAIL};
  }

private:
  static void validate(const PivotMotionInput &s) {
    for(double x:{s.monotonic_time_sec,s.dt_sec,s.anchor_radial_error_m,s.measured_linear_speed_mps,
                  s.measured_yaw_rate_radps,s.heading_error_rad})
      if(!std::isfinite(x)) throw std::invalid_argument("nonfinite pivot input");
    if(s.monotonic_time_sec<0.0||s.anchor_radial_error_m<0.0)
      throw std::invalid_argument("negative pivot time/radial");
  }
  static double wrap_pi(double x) {
    double r=std::fmod(x+pivot_pi,2.0*pivot_pi);
    if(r<0.0)r+=2.0*pivot_pi;
    return r-pivot_pi;
  }
  static MotionDirective directive_for(MotionState s) {
    switch(s) {
      case MotionState::TRACK:return MotionDirective::TRACK;
      case MotionState::CORNER_APPROACH:return MotionDirective::CORNER_APPROACH;
      case MotionState::BRAKE_TO_ANCHOR:return MotionDirective::BRAKE_TO_ANCHOR;
      case MotionState::PIVOT:return MotionDirective::PIVOT;
      case MotionState::RECENTER:return MotionDirective::RECENTER;
      case MotionState::REALIGN:return MotionDirective::REALIGN;
      case MotionState::RECAPTURE:return MotionDirective::RECAPTURE;
      case MotionState::HOLD_FAIL:return MotionDirective::HOLD_FAIL;
      default:return MotionDirective::HOLD_ZERO;
    }
  }
  void begin_maneuver(double now) {
    max_pivot_drift_m_=0.0;recenter_attempts_=0;
    stop_settle_destination_=MotionState::PIVOT;
    alignment_start_time_sec_.reset();
    stop_held_sec_=release_held_sec_=0.0;state_enter_time_sec_=now;
  }
  std::string transition(MotionState s,double now,const char *reason) {
    const MotionState old=state_;
    state_=s;state_enter_time_sec_=now;
    // Faithful to the reference: every transition clears accumulated stop dwell.
    stop_held_sec_=0.0;
    if(s==MotionState::POSITION_CHECK && old==MotionState::PIVOT_SETTLE) {}
    else if(s==MotionState::PIVOT_SETTLE || s==MotionState::REALIGN) {
      if(old!=s) release_held_sec_=0.0;
    } else release_held_sec_=0.0;
    return reason;
  }
  std::string enter_recenter(double now,const char *reason) {
    if(recenter_attempts_>=c_.max_recenter_attempts) return fail(now,"recenter_attempts_exhausted");
    ++recenter_attempts_;alignment_start_time_sec_.reset();
    return transition(MotionState::RECENTER,now,reason);
  }
  std::string fail(double now,const char *reason) {return transition(MotionState::HOLD_FAIL,now,reason);}
  double elapsed(const PivotMotionInput &s) const {return std::max(0.0,s.monotonic_time_sec-state_enter_time_sec_);}
  bool alignment_timed_out(const PivotMotionInput &s,double limit) {
    if(!alignment_start_time_sec_) alignment_start_time_sec_=s.monotonic_time_sec;
    return s.monotonic_time_sec-*alignment_start_time_sec_>=limit;
  }
  PivotMotionConfig c_;
  MotionState state_{MotionState::TRACK},stop_settle_destination_{MotionState::PIVOT};
  double state_enter_time_sec_{},last_time_sec_{},stop_held_sec_{},release_held_sec_{},max_pivot_drift_m_{};
  std::optional<double> alignment_start_time_sec_{};
  int recenter_attempts_{};
};
} // namespace dyx::rpp
