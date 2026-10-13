#pragma once
// Phase 2G1A: NON-ACTUATING multi-point marking mission sequencer.
// Port of mission-management decision CONTRACT, not of ROS middleware.
// No ROS, MAVROS, Python, motor/spray outputs or reverse-driving logic.
// A 20mm RPP estimator-frame stop is NOT authorization to mark. Separate
// validated, post-stop RAW RTK-FIXED GNSS survey evidence (<=30mm) is required.

#include <cmath>
#include <cstddef>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {

enum class MissionMode { automatic, manual };
enum class MissionStage { unloaded, ready, approaching, verify_hold,
                          spray_ack, resolved, await_next, paused, finished,
                          stopped, fault };
enum class MarkingResult { pending, completed, failed, skipped };
enum class SurveyVerdict { waiting, pass, fail };
enum class SprayVerdict { none, success, failed };
enum class MissionEvent { none, stop_verified, request_spray, point_resolved,
                          advanced, mission_finished, paused, fault };

inline const char *mission_stage_name(MissionStage s) {
  switch (s) {
    case MissionStage::unloaded: return "unloaded";
    case MissionStage::ready: return "ready";
    case MissionStage::approaching: return "approaching";
    case MissionStage::verify_hold: return "verify_hold";
    case MissionStage::spray_ack: return "spray_ack";
    case MissionStage::resolved: return "resolved";
    case MissionStage::await_next: return "await_next";
    case MissionStage::paused: return "paused";
    case MissionStage::finished: return "finished";
    case MissionStage::stopped: return "stopped";
    case MissionStage::fault: return "fault";
  }
  return "unknown";
}

struct MarkingGoal {
  int raw_path_index{-1};
  int marking_index{-1};
  // Examples: P0001, P0002. This is a label, NOT a certificate identity.
  std::string goal_label{};
  // Full mission-run + path + goal-instance identity; must be unique per goal.
  std::string terminal_identity{};
};
struct MarkingRecord {
  MarkingGoal goal{};
  MarkingResult result{MarkingResult::pending};
  std::string reason{};
};
struct MissionSequenceConfig {
  double terminal_tolerance_m{0.020};
  double raw_gnss_tolerance_m{0.030};
  double stationary_speed_tolerance_mps{0.030};
  double marking_hold_sec{3.0};
  double raw_gnss_timeout_sec{3.0};
  double spray_ack_timeout_sec{6.0};
  void validate() const {
    for (double v : {terminal_tolerance_m, raw_gnss_tolerance_m,
                     stationary_speed_tolerance_mps, marking_hold_sec,
                     raw_gnss_timeout_sec, spray_ack_timeout_sec}) {
      if (!std::isfinite(v) || v <= 0.0)
        throw std::invalid_argument("invalid mission sequence configuration");
    }
    if (terminal_tolerance_m > raw_gnss_tolerance_m)
      throw std::invalid_argument("terminal tolerance exceeds raw GNSS tolerance");
  }
};

struct MissionSequenceInput {
  double monotonic_time_sec{0.0};
  bool motion_health_authorized{false};  // external battery/RTK/obstacle/offboard arbitration
  bool emergency_stop{true};
  // Fresh, identity-matched RPP terminal-stop heartbeat (estimator frame).
  bool terminal_present{false};
  std::string terminal_identity{};
  bool terminal_certificate_valid{false};
  bool terminal_zero_latched{false};
  bool terminal_stationary{false};
  double terminal_radial_error_m{0.0};
  double measured_forward_speed_mps{0.0};
  bool past_goal_plane{false};
  bool rpp_reported_missed{false};
  // Separate RAW GNSS survey verdict, from a validated RTK-FIXED window.
  SurveyVerdict survey{SurveyVerdict::waiting};
  std::string survey_identity{};
  bool survey_validated_raw_fixed{false};
  double survey_radial_error_m{0.0};
  double survey_last_sample_sec{0.0};
  // Acknowledgement from a separately implemented spray controller.
  SprayVerdict spray_result{SprayVerdict::none};
  std::string spray_identity{};
  double spray_received_sec{0.0};
};

struct MissionSequenceOutput {
  MissionStage stage{MissionStage::unloaded};
  MissionEvent event{MissionEvent::none};
  bool hold_zero{true};
  bool allow_next_leg_preview{false};
  bool request_spray{false}; // one-shot INTENT ONLY; no actuator call
  std::string active_terminal_identity{};
  std::string reason{};
  std::size_t current_marking{0};
  std::size_t total{0};
  std::size_t completed{0};
  std::size_t failed{0};
  std::size_t skipped{0};
  double hold_elapsed_sec{0.0};
};

class MissionSequencer {
 public:
  explicit MissionSequencer(MissionSequenceConfig config = {}) : cfg_(config) {
    cfg_.validate();
  }
  void load(std::vector<MarkingGoal> goals, MissionMode mode) {
    if (goals.empty()) throw std::invalid_argument("empty marking mission");
    std::set<std::string> instances;
    int last_raw = -1;
    for (std::size_t i = 0; i < goals.size(); ++i) {
      const auto &g = goals[i];
      if (g.raw_path_index <= last_raw || g.marking_index != static_cast<int>(i) ||
          g.goal_label.empty() || g.terminal_identity.empty() ||
          !instances.insert(g.terminal_identity).second)
        throw std::invalid_argument("invalid or duplicate semantic goal identity");
      last_raw = g.raw_path_index;
    }
    records_.clear();
    for (auto &g : goals) records_.push_back({std::move(g), MarkingResult::pending, {}});
    mode_ = mode;
    cursor_ = 0;
    stage_ = MissionStage::ready;
    last_time_.reset();
    hold_start_.reset();
    spray_request_time_.reset();
    survey_pass_latched_ = false;
    authorized_this_step_ = false;
    reason_ = "MISSION_LOADED";
  }
  void start(bool external_health_authorized) {
    if (stage_ != MissionStage::ready) throw std::logic_error("start requires ready");
    if (!external_health_authorized) throw std::logic_error("start requires verified safety gate");
    authorized_this_step_ = false;
    stage_ = MissionStage::approaching;
    reason_ = "MISSION_STARTED";
  }
  // External safety arbiter may authorize resume after a prior soft pause.
  // Never recover a mission fault or stale spray acknowledgement implicitly.
  void resume(bool external_health_authorized) {
    if (stage_ != MissionStage::paused) throw std::logic_error("resume requires paused");
    if (!external_health_authorized) throw std::logic_error("resume requires verified safety gate");
    authorized_this_step_ = false;
    stage_ = MissionStage::approaching;
    hold_start_.reset();
    survey_pass_latched_ = false;
    reason_ = "EXPLICIT_RESUME_REVERIFY";
  }
  void pause() {
    authorized_this_step_ = false;
    if (stage_ == MissionStage::spray_ack) {
      fault("SPRAY_PENDING_CANNOT_PAUSE_AND_RETRY");
    } else if (stage_ == MissionStage::approaching ||
               stage_ == MissionStage::verify_hold ||
               stage_ == MissionStage::resolved) {
      stage_ = MissionStage::paused;
      hold_start_.reset();
      survey_pass_latched_ = false;
      reason_ = "EXPLICIT_PAUSE";
    }
  }
  void stop() {
    authorized_this_step_ = false;
    // STOP is permitted even after a fault, and never requests motor movement.
    stage_ = MissionStage::stopped;
    hold_start_.reset();
    spray_request_time_.reset();
    survey_pass_latched_ = false;
    reason_ = "STOPPED";
  }
  void next() {
    if (stage_ != MissionStage::await_next)
      throw std::logic_error("NEXT requires manual resolved point");
    advance();
  }
  void skip() {
    if (stage_ != MissionStage::approaching)
      throw std::logic_error("SKIP permitted only while approaching");
    resolve(MarkingResult::skipped, "OPERATOR_SKIP");
  }
  MissionSequenceOutput step(const MissionSequenceInput &s) {
    authorized_this_step_ = false;
    if (!std::isfinite(s.monotonic_time_sec) || s.monotonic_time_sec < 0.0 ||
        (last_time_ && s.monotonic_time_sec < *last_time_)) {
      fault("INVALID_OR_BACKWARD_TIME");
      return output(MissionEvent::fault);
    }
    last_time_ = s.monotonic_time_sec;
    // E-stop latches even when PAUSED, WAIT_NEXT or READY. Only a separately
    // verified new mission load can clear the fault.
    if (s.emergency_stop && stage_ != MissionStage::unloaded &&
        stage_ != MissionStage::stopped && stage_ != MissionStage::finished &&
        stage_ != MissionStage::fault) {
      fault("EMERGENCY_STOP");
      return output(MissionEvent::fault);
    }
    if (stage_ == MissionStage::stopped || stage_ == MissionStage::fault ||
        stage_ == MissionStage::finished || stage_ == MissionStage::ready ||
        stage_ == MissionStage::unloaded || stage_ == MissionStage::paused ||
        stage_ == MissionStage::await_next)
      return output();
    if (!s.motion_health_authorized) {
      pause();
      return output(stage_ == MissionStage::fault ? MissionEvent::fault : MissionEvent::paused);
    }
    if (!std::isfinite(s.measured_forward_speed_mps) ||
        s.measured_forward_speed_mps < 0.0 ||
        !std::isfinite(s.terminal_radial_error_m) ||
        !std::isfinite(s.survey_radial_error_m) ||
        !std::isfinite(s.survey_last_sample_sec) ||
        !std::isfinite(s.spray_received_sec)) {
      fault("NONFINITE_OR_REVERSE_EVIDENCE");
      return output(MissionEvent::fault);
    }
    if (stage_ == MissionStage::resolved) {
      if (mode_ == MissionMode::automatic) {
        advance();
        return output(stage_ == MissionStage::finished ? MissionEvent::mission_finished
                                                       : MissionEvent::advanced);
      }
      stage_ = MissionStage::await_next;
      reason_ = "MANUAL_WAIT_NEXT";
      return output();
    }
    const auto &identity = active_identity();
    // An explicitly presented terminal certificate for another goal is a
    // cross-mission safety violation, never a reason to advance.
    if (s.terminal_present && s.terminal_identity != identity) {
      fault("TERMINAL_IDENTITY_MISMATCH");
      return output(MissionEvent::fault);
    }
    const bool stopped = s.terminal_present && s.terminal_certificate_valid &&
        s.terminal_zero_latched && s.terminal_stationary &&
        s.terminal_radial_error_m >= 0.0 &&
        s.terminal_radial_error_m <= cfg_.terminal_tolerance_m &&
        s.measured_forward_speed_mps <= cfg_.stationary_speed_tolerance_mps;
    if (stage_ == MissionStage::approaching) {
      authorized_this_step_ = true;
      if (stopped) {
        hold_start_ = s.monotonic_time_sec;
        survey_pass_latched_ = false;
        stage_ = MissionStage::verify_hold;
        reason_ = "STOP_CERTIFIED_WAIT_RAW_GNSS";
        return output(MissionEvent::stop_verified);
      }
      if (s.measured_forward_speed_mps <= cfg_.stationary_speed_tolerance_mps &&
          (s.rpp_reported_missed || s.past_goal_plane)) {
        resolve(MarkingResult::failed, "STOPPED_OUTSIDE_MARKING_POINT");
        return output(MissionEvent::point_resolved);
      }
      return output();
    }
    if (stage_ == MissionStage::verify_hold) {
      if (!stopped) {
        // Certificate loss must NOT return to forward guidance on its own.
        stage_ = MissionStage::paused;
        hold_start_.reset();
        survey_pass_latched_ = false;
        reason_ = "TERMINAL_CERTIFICATE_LOST";
        return output(MissionEvent::paused);
      }
      if (!hold_start_) {
        fault("MISSING_STOP_TIME");
        return output(MissionEvent::fault);
      }
      const double elapsed = s.monotonic_time_sec - *hold_start_;
      // Raw-survey FAIL can mean unavailable RTK evidence: never require a
      // successful FIXED window just to mark the point FAILED safely.
      if (s.survey == SurveyVerdict::fail) {
        if (s.survey_identity != identity) {
          fault("RAW_GNSS_FAILURE_IDENTITY_MISMATCH");
          return output(MissionEvent::fault);
        }
        resolve(MarkingResult::failed, "RAW_GNSS_GATE_FAILED");
        return output(MissionEvent::point_resolved);
      }
      // A PASS delivered after the raw measurement deadline is not a valid
      // late retry; an already latched timely PASS may finish the hold.
      if (!survey_pass_latched_ && elapsed > cfg_.raw_gnss_timeout_sec) {
        resolve(MarkingResult::failed, "RAW_GNSS_TIMEOUT");
        return output(MissionEvent::point_resolved);
      }
      if (s.survey == SurveyVerdict::pass) {
        if (s.survey_identity != identity ||
            !s.survey_validated_raw_fixed ||
            s.survey_last_sample_sec < *hold_start_ ||
            s.survey_last_sample_sec > s.monotonic_time_sec + 1.e-9) {
          fault("INVALID_RAW_GNSS_EVIDENCE");
          return output(MissionEvent::fault);
        }
        if (s.survey_radial_error_m < 0.0 ||
            s.survey_radial_error_m > cfg_.raw_gnss_tolerance_m) {
          resolve(MarkingResult::failed, "RAW_GNSS_MARKING_ACCURACY_FAILED");
          return output(MissionEvent::point_resolved);
        }
        survey_pass_latched_ = true;
      }
      if (!survey_pass_latched_ && elapsed >= cfg_.raw_gnss_timeout_sec) {
        resolve(MarkingResult::failed, "RAW_GNSS_TIMEOUT");
        return output(MissionEvent::point_resolved);
      }
      if (survey_pass_latched_ && elapsed >= cfg_.marking_hold_sec) {
        stage_ = MissionStage::spray_ack;
        spray_request_time_ = s.monotonic_time_sec;
        reason_ = "REQUEST_SPRAY_ONCE";
        auto o = output(MissionEvent::request_spray);
        o.request_spray = true;
        return o;
      }
      return output();
    }
    if (stage_ == MissionStage::spray_ack) {
      if (!stopped) {
        fault("MOTION_DURING_SPRAY_ACK");
        return output(MissionEvent::fault);
      }
      if (!spray_request_time_) {
        fault("MISSING_SPRAY_REQUEST_TIME");
        return output(MissionEvent::fault);
      }
      if (s.spray_result != SprayVerdict::none) {
        if (s.spray_identity != identity ||
            s.spray_received_sec < *spray_request_time_ ||
            s.spray_received_sec > s.monotonic_time_sec + 1.e-9) {
          fault("INVALID_OR_REPLAYED_SPRAY_ACK");
          return output(MissionEvent::fault);
        }
        resolve(s.spray_result == SprayVerdict::success ? MarkingResult::completed
                                                        : MarkingResult::failed,
                s.spray_result == SprayVerdict::success ? "SPRAY_ACK_SUCCESS"
                                                        : "SPRAY_ACK_FAILED");
        return output(MissionEvent::point_resolved);
      }
      if (s.monotonic_time_sec - *spray_request_time_ >= cfg_.spray_ack_timeout_sec) {
        resolve(MarkingResult::failed, "SPRAY_ACK_TIMEOUT");
        return output(MissionEvent::point_resolved);
      }
      return output();
    }
    fault("UNHANDLED_SEQUENCE_STAGE");
    return output(MissionEvent::fault);
  }

  MissionStage stage() const { return stage_; }
  std::size_t current_marking() const { return cursor_; }
  const std::vector<MarkingRecord> &records() const { return records_; }
  MissionSequenceOutput status() const { return output(); }

 private:
  const std::string &active_identity() const {
    if (cursor_ >= records_.size()) throw std::logic_error("no active marking");
    return records_[cursor_].goal.terminal_identity;
  }
  void fault(const char *why) {
    authorized_this_step_ = false;
    stage_ = MissionStage::fault;
    reason_ = why;
    hold_start_.reset();
    survey_pass_latched_ = false;
  }
  void resolve(MarkingResult result, const char *why) {
    authorized_this_step_ = false;
    if (cursor_ >= records_.size() || records_[cursor_].result != MarkingResult::pending)
      throw std::logic_error("duplicate point resolution");
    records_[cursor_].result = result;
    records_[cursor_].reason = why;
    stage_ = MissionStage::resolved;
    reason_ = why;
    hold_start_.reset();
    spray_request_time_.reset();
    survey_pass_latched_ = false;
  }
  void advance() {
    authorized_this_step_ = false;
    if (cursor_ + 1 >= records_.size()) {
      cursor_ = records_.size();
      stage_ = MissionStage::finished;
      reason_ = "ALL_MARKINGS_RESOLVED";
    } else {
      ++cursor_;
      stage_ = MissionStage::approaching;
      reason_ = "NEXT_GOAL_READY";
    }
  }
  MissionSequenceOutput output(MissionEvent event = MissionEvent::none) const {
    MissionSequenceOutput o;
    o.stage = stage_;
    o.event = event;
    o.reason = reason_;
    o.current_marking = cursor_;
    o.total = records_.size();
    for (const auto &r : records_) {
      if (r.result == MarkingResult::completed) ++o.completed;
      if (r.result == MarkingResult::failed) ++o.failed;
      if (r.result == MarkingResult::skipped) ++o.skipped;
    }
    if (cursor_ < records_.size()) o.active_terminal_identity = active_identity();
    // Positive permission is advisory; a future ROS adapter must independently
    // pass all safety gates and must not treat it as PX4 motion authorization.
    o.allow_next_leg_preview = (stage_ == MissionStage::approaching && authorized_this_step_);
    o.hold_zero = !o.allow_next_leg_preview;
    if (hold_start_ && last_time_ && stage_ == MissionStage::verify_hold)
      o.hold_elapsed_sec = *last_time_ - *hold_start_;
    return o;
  }
  MissionSequenceConfig cfg_{};
  std::vector<MarkingRecord> records_{};
  MissionMode mode_{MissionMode::automatic};
  MissionStage stage_{MissionStage::unloaded};
  std::size_t cursor_{0};
  std::optional<double> last_time_{};
  std::optional<double> hold_start_{};
  std::optional<double> spray_request_time_{};
  bool survey_pass_latched_{false};
  bool authorized_this_step_{false};
  std::string reason_{"UNLOADED"};
};
} // namespace dyx::rpp
