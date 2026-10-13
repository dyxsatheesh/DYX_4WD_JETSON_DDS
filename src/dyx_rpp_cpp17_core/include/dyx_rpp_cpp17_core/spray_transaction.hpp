#pragma once
// Phase 2H2A: pure C++17 NON-ACTUATING mission spray transaction.
// Intent is NOT a physical PWM command. A separate, tested PX4 DDS adapter
// must translate, persist command history and verify actuator feedback.
// Defaults reflect proposed AUX5 OFF=1200us, ON=1400us, 3s operation;
// no implied normalized MAV_CMD_DO_SET_ACTUATOR mapping is assumed here.
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::rpp {

enum class SprayState { idle, waiting_press_ack, holding, waiting_release_ack,
                        recovering_release, wait_goal_clear, fault };
enum class SprayCommand { none, press, release };
enum class SprayCompletion { none, completed, failed, cancelled };
inline const char *spray_state_name(SprayState s) {
  switch(s) {
    case SprayState::idle:return "IDLE";
    case SprayState::waiting_press_ack:return "WAIT_PRESS_ACK";
    case SprayState::holding:return "HOLDING";
    case SprayState::waiting_release_ack:return "WAIT_RELEASE_ACK";
    case SprayState::recovering_release:return "RECOVERY_RELEASE";
    case SprayState::wait_goal_clear:return "WAIT_GOAL_CLEAR";
    case SprayState::fault:return "FAULT";
  }
  return "UNKNOWN";
}
struct SprayConfig {
  int off_pwm_us{1200};
  int on_pwm_us{1400};
  double press_duration_sec{3.0};
  double command_ack_timeout_sec{1.0};
  double release_retry_sec{0.25};
  double hard_press_limit_sec{5.0};
  void validate() const {
    if (off_pwm_us < 800 || off_pwm_us > 2200 ||
        on_pwm_us < 800 || on_pwm_us > 2200 || on_pwm_us == off_pwm_us ||
        !std::isfinite(press_duration_sec) || press_duration_sec <= 0 ||
        !std::isfinite(command_ack_timeout_sec) || command_ack_timeout_sec <= 0 ||
        !std::isfinite(release_retry_sec) || release_retry_sec <= 0 ||
        !std::isfinite(hard_press_limit_sec) ||
        hard_press_limit_sec <= press_duration_sec + command_ack_timeout_sec)
      throw std::invalid_argument("invalid spray timing/PWM configuration");
  }
};
struct SprayAck {
  std::uint64_t sequence{0};
  bool accepted{false};
  // Receipt in the same host monotonic clock as SprayInput::now_sec.
  double received_at_sec{0};
};
struct SprayInput {
  double now_sec{0};
  bool request_spray{false};
  // Unique RUN|PATH|GOAL|INSTANCE identity from the mission sequencer.
  std::string terminal_identity{};
  // All booleans below are externally verified; default is BLOCKED.
  bool mission_enabled{false};
  bool mission_spray_hold{false};
  bool safety_motion_hold_verified{false};
  bool raw_rtk_fixed_survey_pass{false};
  bool px4_armed_offboard{false};
  bool physical_estop_released{false};
  bool controller_enabled{false};
  bool persistent_press_lock_recorded{false};
  std::optional<SprayAck> command_ack{};
};
struct SprayOutput {
  SprayState state{SprayState::idle};
  SprayCommand command{SprayCommand::none};
  std::uint64_t command_sequence{0};
  int requested_pwm_us{0};
  bool hold_zero{true};
  bool may_be_physically_on{false};
  bool release_acknowledged{false};
  SprayCompletion completion{SprayCompletion::none};
  std::string completed_identity{};
  double completion_time_sec{0};
  std::string reason{"IDLE"};
};

class NativeSprayTransaction final {
 public:
  explicit NativeSprayTransaction(SprayConfig cfg={}) : cfg_(cfg) {cfg_.validate();}
  // At startup, do not send a PRESS or blindly assume the last servo state.
  // A new mission PRESS is refused until an independently verified OFF state
  // has been established (e.g. disarmed PX4 release confirmation).
  bool establish_off_state(bool operator_authorized, bool px4_disarmed,
                           bool confirmed_off) {
    if (!operator_authorized || !px4_disarmed || !confirmed_off ||
        state_ != SprayState::idle || fault_latched_) return false;
    off_verified_=true;
    return true;
  }
  // Called by the restart adapter after loading a durable journal containing
  // an uncertain PRESS/release. Never replay PRESS across process restarts.
  // A RELEASE intent is emitted at the next tick and retried until ACK.
  void restore_uncertain(const std::string &identity) {
    if (identity.empty() || state_!=SprayState::idle)
      throw std::invalid_argument("invalid restart journal");
    active_=identity;
    used_.insert(identity);
    fault_latched_=true;
    off_verified_=false;
    state_=SprayState::recovering_release;
    last_issue_sec_.reset();
    pending_.reset();
    result_=SprayCompletion::failed;
    reason_="UNCERTAIN_RESTART_RELEASE_REQUIRED";
  }
  // Safe/explicit re-arm after a fault; never clears replay history.
  bool reset_fault(bool authorized, bool px4_disarmed, bool off_confirmed) {
    if (!authorized || !px4_disarmed || !off_confirmed ||
        state_ != SprayState::fault) return false;
    fault_latched_=false;
    off_verified_=true;
    active_.clear();
    state_=SprayState::idle;
    pending_.reset();
    last_issue_sec_.reset();
    result_=SprayCompletion::none;
    reason_="MANUAL_FAULT_RESET";
    return true;
  }
  // STOP/disable/ESTOP must always be permitted. They generate a RELEASE
  // intent when a PRESS could have been accepted; no automatic restart.
  void stop() { cancel_requested_=true; }
  SprayOutput step(const SprayInput &in) {
    SprayOutput out;
    if (!std::isfinite(in.now_sec) || in.now_sec < 0 ||
        (last_time_ && in.now_sec < *last_time_)) {
      fault_latched_=true;
      reason_="BAD_MONOTONIC_TIME";
      if (state_!=SprayState::idle && state_!=SprayState::fault) {
        state_=SprayState::recovering_release;
        pending_.reset(); last_issue_sec_.reset(); result_=SprayCompletion::failed;
        return issue(out,SprayCommand::release,0.0);
      }
      state_=SprayState::fault;
      return decorate(out);
    }
    last_time_=in.now_sec;
    const bool safe = gates_ok(in);
    // Only the SAME semantic goal is permitted to maintain an active press.
    const bool active = state_ == SprayState::waiting_press_ack ||
        state_ == SprayState::holding || state_ == SprayState::waiting_release_ack;
    if (active && (!safe || in.terminal_identity!=active_ || cancel_requested_)) {
      result_ = cancel_requested_ ? SprayCompletion::cancelled : SprayCompletion::failed;
      reason_=cancel_requested_?"STOP_ABORT_RELEASE":"SAFETY_OR_IDENTITY_LOST_RELEASE";
      fault_latched_ = !cancel_requested_;
      state_=SprayState::recovering_release;
      pending_.reset(); last_issue_sec_.reset();
      return issue(out,SprayCommand::release,in.now_sec);
    }
    if (state_==SprayState::idle) {
      cancel_requested_=false;
      if (in.request_spray) {
        if (in.terminal_identity.empty() || !safe || !off_verified_ || fault_latched_ ||
            !in.persistent_press_lock_recorded || used_.count(in.terminal_identity)) {
          reason_="PRESS_BLOCKED_BY_SAFETY_OR_JOURNAL";
          return decorate(out);
        }
        active_=in.terminal_identity;
        // A durable write-ahead journal for this identity MUST already exist
        // before the PX4 adapter transmits this intent.
        used_.insert(active_);
        off_verified_=false;
        first_press_sec_=in.now_sec;
        result_=SprayCompletion::none;
        state_=SprayState::waiting_press_ack;
        reason_="PRESS_INTENT_WAIT_ACK";
        return issue(out,SprayCommand::press,in.now_sec);
      }
      return decorate(out);
    }
    if (state_==SprayState::wait_goal_clear) {
      if (in.terminal_identity!=active_ && !in.request_spray) {
        active_.clear(); state_=SprayState::idle;
      }
      return decorate(out);
    }
    if (state_==SprayState::fault) return decorate(out);
    if (state_==SprayState::waiting_press_ack) {
      if (ack_received(in) && pending_->kind==SprayCommand::press) {
        if (!in.command_ack->accepted ||
            in.command_ack->received_at_sec > pending_->sent_at_sec + cfg_.command_ack_timeout_sec)
          return start_release(out,in.now_sec,"PRESS_REJECTED_OR_ACK_LATE",true);
        pending_.reset();
        state_=SprayState::holding;
        hold_start_sec_=in.now_sec;
        reason_="PRESS_ACKNOWLEDGED";
      } else if (age_exceeded(in.now_sec,cfg_.command_ack_timeout_sec)) {
        return start_release(out,in.now_sec,"PRESS_ACK_TIMEOUT_RELEASE",true);
      }
      return decorate(out);
    }
    if (state_==SprayState::holding) {
      if (in.now_sec-*first_press_sec_>=cfg_.hard_press_limit_sec)
        return start_release(out,in.now_sec,"HARD_PRESS_WATCHDOG_RELEASE",true);
      if (in.now_sec-*hold_start_sec_>=cfg_.press_duration_sec) {
        result_=SprayCompletion::completed;
        state_=SprayState::waiting_release_ack;
        reason_="TIMED_RELEASE_WAIT_ACK";
        return issue(out,SprayCommand::release,in.now_sec);
      }
      return decorate(out);
    }
    if (state_==SprayState::waiting_release_ack ||
        state_==SprayState::recovering_release) {
      if (ack_received(in) && pending_->kind==SprayCommand::release) {
        if (in.command_ack->accepted) return finish_release(out,in.now_sec);
        // Rejected RELEASE is never taken as confirmation of OFF.
        fault_latched_=true; result_=SprayCompletion::failed;
        reason_="RELEASE_REJECTED_RETRY";
        state_=SprayState::recovering_release;
        pending_.reset(); last_issue_sec_.reset();
        return issue(out,SprayCommand::release,in.now_sec);
      }
      if (!pending_) return issue(out,SprayCommand::release,in.now_sec);
      const double retry_after = state_==SprayState::recovering_release ?
          cfg_.release_retry_sec : cfg_.command_ack_timeout_sec;
      if (age_exceeded(in.now_sec,retry_after)) {
        fault_latched_=true;
        result_=SprayCompletion::failed;
        state_=SprayState::recovering_release;
        reason_="RELEASE_NOT_ACKNOWLEDGED_RETRY";
        return issue(out,SprayCommand::release,in.now_sec);
      }
      return decorate(out);
    }
    return decorate(out);
  }
  SprayState state() const {return state_;}
  bool fault_latched() const {return fault_latched_;}
  bool off_verified() const {return off_verified_;}
  bool identity_used(const std::string &id)const{return used_.count(id)!=0;}
 private:
  struct Pending {SprayCommand kind;std::uint64_t sequence;double sent_at_sec;};
  static bool gates_ok(const SprayInput &s) {
    return s.mission_enabled && s.mission_spray_hold &&
        s.safety_motion_hold_verified && s.raw_rtk_fixed_survey_pass &&
        s.px4_armed_offboard && s.physical_estop_released &&
        s.controller_enabled;
  }
  bool ack_received(const SprayInput &in) const {
    return pending_ && in.command_ack &&
        in.command_ack->sequence == pending_->sequence &&
        std::isfinite(in.command_ack->received_at_sec) &&
        in.command_ack->received_at_sec >= pending_->sent_at_sec &&
        in.command_ack->received_at_sec <= in.now_sec;
  }
  bool age_exceeded(double now_sec,double max_age_sec)const {
    return pending_ && now_sec-pending_->sent_at_sec>=max_age_sec;
  }
  SprayOutput issue(SprayOutput out,SprayCommand kind,double now_sec) {
    if (!std::isfinite(now_sec) || now_sec<0) now_sec=0; // release on clock fault
    pending_=Pending{kind,++sequence_,now_sec};
    last_issue_sec_=now_sec;
    if (kind==SprayCommand::release) off_verified_=false;
    out.command=kind;
    out.command_sequence=sequence_;
    out.requested_pwm_us=kind==SprayCommand::press?cfg_.on_pwm_us:cfg_.off_pwm_us;
    return decorate(out);
  }
  SprayOutput start_release(SprayOutput out,double now_sec,const char *why,bool failed) {
    state_=SprayState::recovering_release;
    reason_=why;
    result_=failed?SprayCompletion::failed:SprayCompletion::cancelled;
    fault_latched_=failed;
    pending_.reset();
    return issue(out,SprayCommand::release,now_sec);
  }
  SprayOutput finish_release(SprayOutput out,double now_sec) {
    pending_.reset();
    off_verified_=true;
    out.release_acknowledged=true;
    if (result_==SprayCompletion::none) result_=SprayCompletion::failed;
    out.completion=result_;
    out.completed_identity=active_;
    out.completion_time_sec=now_sec;
    if (fault_latched_) {
      state_=SprayState::fault;reason_="RELEASE_CONFIRMED_FAULT_LATCHED";
    } else {
      state_=SprayState::wait_goal_clear;
      reason_=result_==SprayCompletion::completed?"SPRAY_COMPLETED_RELEASE_CONFIRMED":
          "SPRAY_CANCELLED_RELEASE_CONFIRMED";
    }
    return decorate(out);
  }
  SprayOutput decorate(SprayOutput out) const {
    out.state=state_;
    out.hold_zero=true;
    out.may_be_physically_on= !off_verified_ &&
        (state_==SprayState::waiting_press_ack || state_==SprayState::holding ||
         state_==SprayState::waiting_release_ack || state_==SprayState::recovering_release);
    out.reason=reason_;
    return out;
  }
  SprayConfig cfg_{};
  SprayState state_{SprayState::idle};
  std::optional<Pending> pending_{};
  std::optional<double> last_time_{},first_press_sec_{},hold_start_sec_{},last_issue_sec_{};
  std::string active_{};
  std::string reason_{"IDLE"};
  std::set<std::string> used_{};
  std::uint64_t sequence_{0};
  SprayCompletion result_{SprayCompletion::none};
  bool off_verified_{false},fault_latched_{false},cancel_requested_{false};
};
} // namespace dyx::rpp
