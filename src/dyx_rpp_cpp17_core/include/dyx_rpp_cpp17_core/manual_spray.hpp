#pragma once
// Phase 2H2B. Non-actuating, bounded MANUAL spray test for AUX5.
// No ROS/Python/MAVROS; does not emit or satisfy mission marking completion.
// A future transport MUST persist the unique request identity BEFORE PRESS,
// send the resulting intent, correlate acknowledgements, and verify the
// actuator OFF independently before clearing unknown startup/restart state.
#include "dyx_rpp_cpp17_core/spray_transaction.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>

namespace dyx::rpp {

enum class ManualSprayState {
  idle, waiting_press_ack, holding, waiting_release_ack, recovering_release, fault
};

struct ManualSprayConfig {
  int off_pwm_us{1200};
  int on_pwm_us{1400};
  double duration_sec{3.0};
  double ack_timeout_sec{0.8};
  double hard_press_limit_sec{5.0};
  double retry_release_sec{0.25};
  double px4_max_age_sec{0.5};
  void validate() const {
    if (off_pwm_us < 800 || off_pwm_us > 2200 || on_pwm_us < 800 ||
        on_pwm_us > 2200 || off_pwm_us == on_pwm_us)
      throw std::invalid_argument("invalid manual spray PWM");
    for (double x : {duration_sec,ack_timeout_sec,hard_press_limit_sec,
                     retry_release_sec,px4_max_age_sec}) {
      if (!std::isfinite(x) || x <= 0)
        throw std::invalid_argument("invalid manual spray timing");
    }
    if (hard_press_limit_sec <= duration_sec + ack_timeout_sec ||
        retry_release_sec > ack_timeout_sec)
      throw std::invalid_argument("manual spray watchdog/ack timing inconsistent");
  }
};

struct ManualSprayInput {
  double now_sec{0};
  bool manual_on_request{false};
  bool manual_off_request{false};
  // Unique ID per operator press. Stored durably by the surrounding runtime.
  std::string request_id{};
  std::string journal_committed_request_id{};
  bool authorized_operator{false};
  bool controller_enabled{false};
  bool px4_connected{false};
  bool px4_armed{false};
  double px4_age_sec{std::numeric_limits<double>::infinity()};
  bool physical_estop_released{false};
  bool mission_idle{false};
  // Granted by an external single-owner AUX5 command arbiter. Never assume
  // manual and mission spray can command the same servo independently.
  bool exclusive_aux5_lease{false};
  std::optional<SprayAck> command_ack{};
};

struct ManualSprayOutput {
  ManualSprayState state{ManualSprayState::idle};
  SprayCommand command{SprayCommand::none};
  std::uint64_t command_sequence{0};
  int requested_pwm_us{0};
  bool press_may_be_active{false};
  bool release_acknowledged{false};
  // True ONLY after PRESS acknowledgement, timed hold, and RELEASE ack.
  bool manual_success{false};
  // There is deliberately no mission completion or marking result field.
  bool manual_failed{false};
  bool manual_cancelled{false};
  std::string resolved_request_id{};
  std::string reason{"IDLE"};
};

class NativeManualSpray final {
 public:
  explicit NativeManualSpray(ManualSprayConfig cfg={}) : cfg_(cfg) { cfg_.validate(); }

  // Off verification is NEVER assumed from arm/disarm or from a command ACK.
  bool establish_off_state(bool operator_authorized,bool px4_disarmed,bool confirmed_off) {
    if (!operator_authorized || !px4_disarmed || !confirmed_off ||
        state_ != ManualSprayState::idle || fault_latched_) return false;
    off_verified_=true;
    return true;
  }

  // At an uncertain crash/restart, do not replay the interrupted PRESS.
  void restore_uncertain(const std::string &request_id) {
    if (request_id.empty() || state_!=ManualSprayState::idle)
      throw std::invalid_argument("invalid manual spray recovery journal");
    used_.insert(request_id);
    active_=request_id;
    off_verified_=false;
    fault_latched_=true;
    state_=ManualSprayState::recovering_release;
    pending_.reset();
    outcome_=SprayCompletion::failed;
    reason_="RESTART_RELEASE_REQUIRED";
  }

  bool reset_fault(bool operator_authorized,bool px4_disarmed,bool confirmed_off) {
    if (!operator_authorized || !px4_disarmed || !confirmed_off ||
        state_!=ManualSprayState::fault) return false;
    fault_latched_=false;
    off_verified_=true;
    active_.clear();
    pending_.reset();
    state_=ManualSprayState::idle;
    outcome_=SprayCompletion::none;
    reason_="MANUAL_FAULT_RESET";
    return true;
  }
  // STOP is always accepted; it never generates another PRESS.
  void stop() { stop_requested_=true; }
  bool request_used(const std::string &id) const { return used_.count(id)!=0; }
  bool off_verified() const { return off_verified_; }
  ManualSprayState state() const { return state_; }
  bool fault_latched() const { return fault_latched_; }

  ManualSprayOutput step(const ManualSprayInput &in) {
    ManualSprayOutput out;
    if (!std::isfinite(in.now_sec) || in.now_sec<0 ||
        (last_now_ && in.now_sec<*last_now_)) {
      fault_latched_=true;
      outcome_=SprayCompletion::failed;
      reason_="INVALID_MONOTONIC_CLOCK";
      if (state_!=ManualSprayState::idle && state_!=ManualSprayState::fault) {
        state_=ManualSprayState::recovering_release;
        pending_.reset();
        return issue(out,SprayCommand::release,last_now_.value_or(0.0));
      }
      state_=ManualSprayState::fault;
      return decorate(out);
    }
    last_now_=in.now_sec;
    const bool permit_press=gates_ok(in);
    const bool stopping=in.manual_off_request || stop_requested_;

    // Revoke an uncertain or active PRESS on ANY safety loss. Off intent must
    // remain available even when disconnected; transport retries on reconnect.
    if ((state_==ManualSprayState::waiting_press_ack ||
         state_==ManualSprayState::holding) &&
        (!permit_press || stopping)) {
      outcome_=stopping ? SprayCompletion::cancelled : SprayCompletion::failed;
      fault_latched_=!stopping;
      reason_=stopping?"MANUAL_OFF_RELEASE":"MANUAL_SAFETY_GATE_LOST_RELEASE";
      state_=ManualSprayState::recovering_release;
      pending_.reset();
      stop_requested_=false;
      return issue(out,SprayCommand::release,in.now_sec);
    }

    if (state_==ManualSprayState::idle) {
      stop_requested_=false;
      if (in.manual_off_request && !off_verified_) {
        state_=ManualSprayState::recovering_release;
        outcome_=SprayCompletion::cancelled;
        reason_="OFF_REQUEST_WITH_UNVERIFIED_ACTUATOR";
        return issue(out,SprayCommand::release,in.now_sec);
      }
      if (in.manual_on_request && !in.manual_off_request) {
        if (!permit_press || !off_verified_ || fault_latched_ ||
            in.request_id.empty() || used_.count(in.request_id) ||
            in.journal_committed_request_id!=in.request_id) {
          reason_="MANUAL_PRESS_REJECTED";
          return decorate(out);
        }
        active_=in.request_id;
        used_.insert(active_);
        outcome_=SprayCompletion::none;
        off_verified_=false;
        first_press_sec_=in.now_sec;
        state_=ManualSprayState::waiting_press_ack;
        reason_="MANUAL_PRESS_WAIT_ACK";
        return issue(out,SprayCommand::press,in.now_sec);
      }
      return decorate(out);
    }
    if (state_==ManualSprayState::fault) return decorate(out);
    // Once a RELEASE is already in flight, do not generate a second PRESS.
    // Safety/lease loss during this phase invalidates an otherwise successful
    // timed cycle; wait for (or retry) RELEASE before reporting the fault.
    if (state_==ManualSprayState::waiting_release_ack && !permit_press) {
      state_=ManualSprayState::recovering_release;
      fault_latched_=true;
      outcome_=SprayCompletion::failed;
      reason_="SAFETY_LOST_DURING_RELEASE";
    }
    if (state_==ManualSprayState::waiting_press_ack) {
      if (in.now_sec-*first_press_sec_>=cfg_.ack_timeout_sec)
        return begin_release(out,in.now_sec,"PRESS_ACK_TIMEOUT",true);
      if (matched_ack(in,SprayCommand::press)) {
        if (!in.command_ack->accepted)
          return begin_release(out,in.now_sec,"PRESS_REJECTED",true);
        pending_.reset();
        started_hold_sec_=in.now_sec;
        state_=ManualSprayState::holding;
        reason_="MANUAL_HOLD_BOUNDED";
      }
      return decorate(out);
    }
    if (state_==ManualSprayState::holding) {
      if (in.now_sec-*first_press_sec_>=cfg_.hard_press_limit_sec)
        return begin_release(out,in.now_sec,"HARD_PRESS_WATCHDOG",true);
      if (in.now_sec-*started_hold_sec_>=cfg_.duration_sec) {
        outcome_=SprayCompletion::completed;
        state_=ManualSprayState::waiting_release_ack;
        reason_="MANUAL_TIMED_RELEASE_WAIT_ACK";
        return issue(out,SprayCommand::release,in.now_sec);
      }
      return decorate(out);
    }
    if (state_==ManualSprayState::waiting_release_ack ||
        state_==ManualSprayState::recovering_release) {
      if (stopping && outcome_==SprayCompletion::completed)
        outcome_=SprayCompletion::cancelled;
      stop_requested_=false;
      if (matched_ack(in,SprayCommand::release)) {
        if (in.command_ack->accepted) return finish_release(out);
        fault_latched_=true;
        outcome_=SprayCompletion::failed;
        state_=ManualSprayState::recovering_release;
        reason_="RELEASE_REJECTED_RETRY";
        pending_.reset();
        return issue(out,SprayCommand::release,in.now_sec);
      }
      if (!pending_) return issue(out,SprayCommand::release,in.now_sec);
      const double timeout=state_==ManualSprayState::recovering_release ?
          cfg_.retry_release_sec:cfg_.ack_timeout_sec;
      if (in.now_sec-pending_->sent_at_sec>=timeout) {
        outcome_=SprayCompletion::failed;
        fault_latched_=true;
        state_=ManualSprayState::recovering_release;
        reason_="RELEASE_ACK_TIMEOUT_RETRY";
        return issue(out,SprayCommand::release,in.now_sec);
      }
      return decorate(out);
    }
    return decorate(out);
  }

 private:
  struct Pending { SprayCommand kind; std::uint64_t sequence; double sent_at_sec; };
  bool gates_ok(const ManualSprayInput &in) const {
    return in.authorized_operator && in.controller_enabled &&
        in.px4_connected && in.px4_armed &&
        std::isfinite(in.px4_age_sec) && in.px4_age_sec>=0 &&
        in.px4_age_sec<=cfg_.px4_max_age_sec &&
        in.physical_estop_released && in.mission_idle &&
        in.exclusive_aux5_lease;
  }
  bool matched_ack(const ManualSprayInput &in,SprayCommand kind) const {
    return pending_ && pending_->kind==kind && in.command_ack &&
        in.command_ack->sequence==pending_->sequence &&
        std::isfinite(in.command_ack->received_at_sec) &&
        in.command_ack->received_at_sec>=pending_->sent_at_sec &&
        in.command_ack->received_at_sec<=in.now_sec;
  }
  ManualSprayOutput issue(ManualSprayOutput out,SprayCommand kind,double now_sec) {
    pending_=Pending{kind,++sequence_,now_sec};
    if (kind==SprayCommand::release) off_verified_=false;
    out.command=kind;
    out.command_sequence=sequence_;
    out.requested_pwm_us=kind==SprayCommand::press ? cfg_.on_pwm_us:cfg_.off_pwm_us;
    return decorate(out);
  }
  ManualSprayOutput begin_release(ManualSprayOutput out,double now,const char *reason,bool failed) {
    outcome_=failed?SprayCompletion::failed:SprayCompletion::cancelled;
    fault_latched_=failed;
    state_=ManualSprayState::recovering_release;
    pending_.reset();
    reason_=reason;
    return issue(out,SprayCommand::release,now);
  }
  ManualSprayOutput finish_release(ManualSprayOutput out) {
    pending_.reset();
    // A transport/PX4 ACK confirms command acceptance, NOT actuator position.
    // Require a separate verified OFF before the NEXT manual PRESS.
    off_verified_=false;
    out.release_acknowledged=true;
    out.resolved_request_id=active_;
    out.manual_success=outcome_==SprayCompletion::completed && !fault_latched_;
    out.manual_cancelled=outcome_==SprayCompletion::cancelled && !fault_latched_;
    out.manual_failed=outcome_==SprayCompletion::failed || fault_latched_;
    state_=fault_latched_?ManualSprayState::fault:ManualSprayState::idle;
    reason_=out.manual_success?"MANUAL_RELEASE_ACK_COMPLETE":
        out.manual_cancelled?"MANUAL_RELEASE_ACK_CANCELLED":"MANUAL_RELEASE_ACK_FAULT";
    active_.clear();
    return decorate(out);
  }
  ManualSprayOutput decorate(ManualSprayOutput out) const {
    out.state=state_;
    // Conservative: without independent OFF verification, the physical state
    // remains unknown even after PX4 accepts a RELEASE command.
    out.press_may_be_active=!off_verified_;
    out.reason=reason_;
    return out;
  }
  ManualSprayConfig cfg_{};
  ManualSprayState state_{ManualSprayState::idle};
  std::optional<Pending> pending_{};
  std::optional<double> last_now_{}, first_press_sec_{},started_hold_sec_{};
  std::string active_{},reason_{"IDLE"};
  std::set<std::string> used_{};
  std::uint64_t sequence_{0};
  SprayCompletion outcome_{SprayCompletion::none};
  bool off_verified_{false},fault_latched_{false},stop_requested_{false};
};
} // namespace dyx::rpp
