#pragma once
// Phase 2H2C: pure, NON-ACTUATING C++17 laser marking FSM for AUX6.
// ON/OFF are normalized actuator-set-2 intent values. A future PX4 DDS
// adapter must implement mapping, persistent journaling, feedback and hardware
// safety independently. No ROS, Python, MAVROS, AUX5 spray or motor commands.
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::rpp {

enum class LaserMode { none, steady, blink };
enum class LaserState {
  idle, awaiting_on_ack, steady_on, blink_on, blink_off,
  awaiting_off_ack, recovering_off, wait_goal_clear, fault
};
enum class LaserCommand { none, on, off };
enum class LaserEvent { none, on_acknowledged, off_acknowledged, fault };

inline const char *laser_state_name(LaserState s) {
  switch (s) {
    case LaserState::idle: return "IDLE";
    case LaserState::awaiting_on_ack: return "AWAIT_ON_ACK";
    case LaserState::steady_on: return "STEADY_ON";
    case LaserState::blink_on: return "BLINK_ON";
    case LaserState::blink_off: return "BLINK_OFF";
    case LaserState::awaiting_off_ack: return "AWAIT_OFF_ACK";
    case LaserState::recovering_off: return "RECOVERING_OFF";
    case LaserState::wait_goal_clear: return "WAIT_GOAL_CLEAR";
    case LaserState::fault: return "FAULT";
  }
  return "INVALID";
}

struct LaserConfig {
  int aux_channel{6};
  int actuator_set{2};
  double normalized_on{1.0};
  double normalized_off{-1.0};
  double blink_half_period_sec{0.50};
  double steady_handoff_sec{1.50};
  double command_ack_timeout_sec{1.0};
  double off_retry_sec{0.25};
  void validate() const {
    if (aux_channel != 6 || actuator_set != 2 ||
        !std::isfinite(normalized_on) || !std::isfinite(normalized_off) ||
        normalized_on < -1.0 || normalized_on > 1.0 ||
        normalized_off < -1.0 || normalized_off > 1.0 ||
        normalized_on == normalized_off ||
        !std::isfinite(blink_half_period_sec) || blink_half_period_sec <= 0.0 ||
        !std::isfinite(steady_handoff_sec) || steady_handoff_sec <= 0.0 ||
        !std::isfinite(command_ack_timeout_sec) || command_ack_timeout_sec <= 0.0 ||
        !std::isfinite(off_retry_sec) || off_retry_sec <= 0.0)
      throw std::invalid_argument("invalid AUX6 laser configuration");
  }
};

struct LaserAck {
  std::uint64_t sequence{0};
  bool accepted{false};
  double received_at_sec{0}; // host monotonic clock; transport ACK, not sensor feedback
};

struct LaserInput {
  double now_sec{0};
  LaserMode requested_mode{LaserMode::none}; // first start request only
  std::string terminal_identity{}; // run-scoped identity for one marking
  bool mission_enabled{false};
  bool laser_tool_selected{false};
  bool controller_enabled{false};
  bool estop_released{false};
  bool px4_armed_offboard{false};
  bool px4_link_fresh{false};
  bool stationary_hold_verified{false};
  bool exclusive_aux6_lease{false};
  bool raw_rtk_fixed_fresh{false};
  // Mutually exclusive raw GNSS survey evidence for this exact point.
  bool raw_survey_within_tolerance{false};
  bool raw_survey_outside_tolerance{false};
  bool persistent_on_lock_recorded{false}; // write-ahead journal persisted
  bool current_goal_live{false}; // only for brief steady-mode hold handoff
  bool hold_active{false};
  std::string hold_identity{};
  LaserMode hold_mode{LaserMode::none};
  std::optional<LaserAck> ack{};
};

struct LaserOutput {
  LaserState state{LaserState::idle};
  LaserCommand command{LaserCommand::none};
  LaserEvent event{LaserEvent::none};
  std::uint64_t command_sequence{0};
  int aux_channel{6};
  int actuator_set{2};
  double normalized_value{0};
  bool hold_zero{true};
  bool on_command_may_have_been_accepted{false};
  bool off_command_acknowledged{false};
  bool marking_completed{false}; // ALWAYS false: no physical laser measurement
  std::string identity{};
  std::string reason{"IDLE"};
};

class NativeLaserMarking final {
 public:
  explicit NativeLaserMarking(LaserConfig cfg={}) : cfg_(cfg) { cfg_.validate(); }

  // Independent OFF confirmation must be obtained while disarmed; a PX4
  // command ACK alone does not prove physical output is OFF.
  bool establish_safe_off(bool authorized, bool px4_disarmed,
                          bool independently_off_verified) {
    if (!authorized || !px4_disarmed || !independently_off_verified ||
        state_ != LaserState::idle || fault_latched_) return false;
    initial_off_verified_=true;
    return true;
  }

  void stop() { cancel_requested_=true; }

  // The durable journal is restored BEFORE accepting new ON intents.
  // Recovery never emits ON, and retains the old identity in used_.
  void restore_uncertain(const std::string &identity) {
    if (identity.empty() || state_ != LaserState::idle)
      throw std::invalid_argument("invalid laser restart journal");
    identity_=identity;
    used_.insert(identity);
    initial_off_verified_=false;
    fault_latched_=true;
    active_mode_=LaserMode::none;
    state_=LaserState::recovering_off;
    pending_.reset();
    next_off_retry_sec_=0;
    reason_="UNCERTAIN_RESTART_OFF_ONLY";
  }

  bool reset_fault(bool authorized, bool px4_disarmed, bool off_verified) {
    if (!authorized || !px4_disarmed || !off_verified ||
        state_ != LaserState::fault) return false;
    fault_latched_=false;
    initial_off_verified_=true;
    identity_.clear();
    active_mode_=LaserMode::none;
    pending_.reset();
    state_=LaserState::idle;
    cancel_requested_=false;
    reason_="OPERATOR_FAULT_RESET";
    return true;
  }

  LaserOutput step(const LaserInput &in) {
    LaserOutput out;
    if (!std::isfinite(in.now_sec) || in.now_sec < 0 ||
        (last_time_ && in.now_sec < *last_time_)) {
      fault_latched_=true;
      reason_="INVALID_MONOTONIC_CLOCK";
      if (state_ != LaserState::idle && state_ != LaserState::fault) {
        state_=LaserState::recovering_off;
        pending_.reset();
        return issue(out,LaserCommand::off,0.0);
      }
      state_=LaserState::fault;
      out.event=LaserEvent::fault;
      return decorate(out);
    }
    last_time_=in.now_sec;
    const bool safe=common_gates(in);
    const bool cancel=cancel_requested_ || !safe ||
        (!identity_.empty() && in.terminal_identity != identity_);

    if (state_==LaserState::idle) {
      if (cancel_requested_) { cancel_requested_=false; return decorate(out); }
      if (in.requested_mode==LaserMode::none) return decorate(out);
      const bool survey_matches = in.requested_mode==LaserMode::steady
          ? in.raw_survey_within_tolerance && !in.raw_survey_outside_tolerance
          : in.raw_survey_outside_tolerance && !in.raw_survey_within_tolerance;
      const bool correct_hold = in.requested_mode==LaserMode::blink
          ? (in.hold_active && in.hold_identity==in.terminal_identity &&
             in.hold_mode==LaserMode::blink) : in.current_goal_live;
      if (!safe || !initial_off_verified_ || !in.persistent_on_lock_recorded ||
          in.terminal_identity.empty() || !survey_matches || !correct_hold ||
          used_.count(in.terminal_identity)) {
        reason_="LASER_ON_BLOCKED_BY_GATES_OR_JOURNAL";
        return decorate(out);
      }
      identity_=in.terminal_identity;
      active_mode_=in.requested_mode;
      used_.insert(identity_);
      initial_off_verified_=false;
      state_=LaserState::awaiting_on_ack;
      reason_="LASER_ON_INTENT_WAIT_ACK";
      return issue(out,LaserCommand::on,in.now_sec);
    }
    if (state_==LaserState::wait_goal_clear) {
      // No new ON for the same identity; future mission goal requires an
      // explicit new safe-OFF attestation before its ON.
      if (!in.hold_active && in.terminal_identity != identity_) {
        identity_.clear(); active_mode_=LaserMode::none;
        state_=LaserState::idle;
        reason_="IDLE_AFTER_GOAL_CLEAR";
      }
      return decorate(out);
    }
    if (state_==LaserState::fault) return decorate(out);

    if (state_==LaserState::recovering_off || state_==LaserState::awaiting_off_ack) {
      if (blink_off_in_flight_ &&
          (cancel || !matched_hold(in,LaserMode::blink))) {
        cancel_requested_=true;
        if (!safe || in.terminal_identity!=identity_) fault_latched_=true;
      }
      if (pending_ && ack_matches(in)) {
        if (in.ack->accepted && !ack_is_late(*in.ack))
          return finish_off(out);
        fault_latched_=true;
        pending_.reset();
        state_=LaserState::recovering_off;
        next_off_retry_sec_=in.now_sec+cfg_.off_retry_sec;
        reason_="LASER_OFF_REJECTED_OR_LATE_RETRY";
        return decorate(out);
      }
      if (pending_ && in.now_sec-pending_->sent_at_sec>=cfg_.command_ack_timeout_sec) {
        fault_latched_=true;
        pending_.reset();
        state_=LaserState::recovering_off;
        next_off_retry_sec_=in.now_sec+cfg_.off_retry_sec;
        reason_="LASER_OFF_TIMEOUT_RETRY";
        return decorate(out);
      }
      if (!pending_ && in.now_sec>=next_off_retry_sec_)
        return issue(out,LaserCommand::off,in.now_sec);
      return decorate(out);
    }

    // An ON toggle may be in flight. Never send another toggle over an
    // outstanding ACK. On STOP/loss, settle it or wait for timeout, then OFF.
    if (state_==LaserState::awaiting_on_ack) {
      if (cancel ||
          (active_mode_==LaserMode::blink &&
           !matched_hold(in,LaserMode::blink)) ||
          (active_mode_==LaserMode::steady &&
           !in.current_goal_live && !matched_hold(in,LaserMode::steady))) {
        cancel_requested_=true;
        if (!safe || in.terminal_identity!=identity_) fault_latched_=true;
      }
      if (pending_ && ack_matches(in)) {
        if (!in.ack->accepted || ack_is_late(*in.ack))
          return begin_off(out,in.now_sec,true,"LASER_ON_REJECTED_OR_LATE");
        pending_.reset();
        const double acknowledged_at=in.ack->received_at_sec;
        if (cancel_requested_) return begin_off(out,in.now_sec,fault_latched_,"LASER_ON_CANCELLED_OFF");
        if (active_mode_==LaserMode::steady) {
          state_=LaserState::steady_on;
          handoff_deadline_sec_=acknowledged_at+cfg_.steady_handoff_sec;
          out.event=LaserEvent::on_acknowledged;
          reason_="LASER_STEADY_ON_ACKNOWLEDGED";
        } else {
          state_=LaserState::blink_on;
          next_toggle_sec_=acknowledged_at+cfg_.blink_half_period_sec;
          reason_="LASER_BLINK_ON_ACKNOWLEDGED";
        }
        return decorate(out);
      }
      if (pending_ && in.now_sec-pending_->sent_at_sec>=cfg_.command_ack_timeout_sec)
        return begin_off(out,in.now_sec,true,"LASER_ON_ACK_TIMEOUT_OFF");
      return decorate(out);
    }

    if (cancel) {
      const bool safety_loss=!safe || in.terminal_identity!=identity_;
      return begin_off(out,in.now_sec,safety_loss,
                       safety_loss?"LASER_SAFETY_LOST_OFF":"OPERATOR_STOP_OFF");
    }

    if (state_==LaserState::steady_on) {
      const bool keep=matched_hold(in,LaserMode::steady) ||
          (in.current_goal_live && in.now_sec<handoff_deadline_sec_);
      if (!keep) return begin_off(out,in.now_sec,false,"LASER_HOLD_RELEASED_OFF");
      return decorate(out);
    }
    if (state_==LaserState::blink_on || state_==LaserState::blink_off) {
      if (!matched_hold(in,LaserMode::blink))
        return begin_off(out,in.now_sec,false,"LASER_BLINK_HOLD_RELEASED_OFF");
      if (in.now_sec < next_toggle_sec_) return decorate(out);
      if (state_==LaserState::blink_on) {
        // Blink OFF is also an acknowledged command, but the cycle continues.
        state_=LaserState::awaiting_off_ack;
        blink_off_in_flight_=true;
        reason_="LASER_BLINK_TOGGLE_OFF";
        return issue(out,LaserCommand::off,in.now_sec);
      }
      state_=LaserState::awaiting_on_ack;
      reason_="LASER_BLINK_TOGGLE_ON";
      return issue(out,LaserCommand::on,in.now_sec);
    }
    return decorate(out);
  }

  LaserState state() const {return state_;}
  bool fault_latched() const {return fault_latched_;}
  bool identity_used(const std::string &identity) const {return used_.count(identity)>0;}
 private:
  struct Pending {LaserCommand kind;std::uint64_t sequence;double sent_at_sec;};
  bool common_gates(const LaserInput &in) const {
    return in.mission_enabled && in.laser_tool_selected && in.controller_enabled &&
       in.estop_released && in.px4_armed_offboard && in.px4_link_fresh &&
       in.stationary_hold_verified && in.exclusive_aux6_lease &&
       in.raw_rtk_fixed_fresh;
  }
  bool matched_hold(const LaserInput &in,LaserMode mode) const {
    return in.hold_active && !identity_.empty() &&
        in.hold_identity==identity_ && in.hold_mode==mode;
  }
  bool ack_matches(const LaserInput &in) const {
    return pending_ && in.ack && in.ack->sequence==pending_->sequence &&
        std::isfinite(in.ack->received_at_sec) &&
        in.ack->received_at_sec>=pending_->sent_at_sec &&
        in.ack->received_at_sec<=in.now_sec;
  }
  bool ack_is_late(const LaserAck &ack) const {
    return pending_ && ack.received_at_sec > pending_->sent_at_sec+cfg_.command_ack_timeout_sec;
  }
  LaserOutput issue(LaserOutput out,LaserCommand kind,double now_sec) {
    if (!std::isfinite(now_sec) || now_sec<0) now_sec=0; // OFF-only clock fault recovery
    pending_=Pending{kind,++sequence_,now_sec};
    out.command=kind;
    out.command_sequence=sequence_;
    out.normalized_value=kind==LaserCommand::on ? cfg_.normalized_on : cfg_.normalized_off;
    out.aux_channel=cfg_.aux_channel;
    out.actuator_set=cfg_.actuator_set;
    return decorate(out);
  }
  LaserOutput begin_off(LaserOutput out,double now_sec,bool fault,const char *reason) {
    fault_latched_=fault_latched_ || fault;
    state_=LaserState::awaiting_off_ack;
    blink_off_in_flight_=false;
    pending_.reset();
    reason_=reason;
    return issue(out,LaserCommand::off,now_sec);
  }
  LaserOutput finish_off(LaserOutput out) {
    pending_.reset();
    out.event=LaserEvent::off_acknowledged;
    out.off_command_acknowledged=true;
    // OFF ACK is NOT independent physical OFF verification.
    initial_off_verified_=false;
    if (blink_off_in_flight_ && !cancel_requested_ && !fault_latched_) {
      blink_off_in_flight_=false;
      state_=LaserState::blink_off;
      next_toggle_sec_=last_time_.value_or(0)+cfg_.blink_half_period_sec;
      reason_="LASER_BLINK_OFF_ACKNOWLEDGED";
      return decorate(out);
    }
    blink_off_in_flight_=false;
    state_=fault_latched_ ? LaserState::fault : LaserState::wait_goal_clear;
    reason_=fault_latched_?"LASER_OFF_ACK_FAULT_LATCHED":"LASER_OFF_ACK_WAIT_GOAL_CLEAR";
    cancel_requested_=false;
    return decorate(out);
  }
  LaserOutput decorate(LaserOutput out) const {
    out.state=state_;
    out.identity=identity_;
    out.reason=reason_;
    out.hold_zero=true;
    out.marking_completed=false;
    out.aux_channel=cfg_.aux_channel;
    out.actuator_set=cfg_.actuator_set;
    out.on_command_may_have_been_accepted=state_==LaserState::awaiting_on_ack ||
        state_==LaserState::steady_on || state_==LaserState::blink_on ||
        state_==LaserState::blink_off || state_==LaserState::awaiting_off_ack ||
        state_==LaserState::recovering_off;
    return out;
  }
  LaserConfig cfg_{};
  LaserState state_{LaserState::idle};
  LaserMode active_mode_{LaserMode::none};
  std::optional<Pending> pending_{};
  std::optional<double> last_time_{};
  double handoff_deadline_sec_{0};
  double next_toggle_sec_{0};
  double next_off_retry_sec_{0};
  std::set<std::string> used_{};
  std::string identity_{};
  std::string reason_{"IDLE"};
  std::uint64_t sequence_{0};
  bool initial_off_verified_{false};
  bool fault_latched_{false};
  bool cancel_requested_{false};
  bool blink_off_in_flight_{false};
};
} // namespace dyx::rpp
