#pragma once
// Session-bound audit of non-actuating mission-command decisions.
// This is NOT an operator motion lease and cannot grant one.
#include <nlohmann/json.hpp>
#include <cstdint>
#include <mutex>
#include <string>
namespace dyx::http_backend {
class NativeCommandDecisionAudit final {
 public:
  bool reserve(const std::string& session, const std::string& run,
               std::uint64_t sequence, bool stop_latched) {
    std::lock_guard<std::mutex> lock(mu_);
    if (stop_latched || stopped_ || session.empty() || run.empty() || sequence == 0)
      return false;
    if (!session_.empty() && (session_ != session || run_ != run)) return false;
    if (sequence <= last_sequence_) return false;
    session_ = session;
    run_ = run;
    last_sequence_ = sequence;
    ++submitted_;
    state_ = "COMMAND_QUEUED_FOR_SAFETY_DENIAL";
    return true;
  }
  void denied(const std::string& session, const std::string& run,
              std::uint64_t sequence) {
    std::lock_guard<std::mutex> lock(mu_);
    if (stopped_ || session != session_ || run != run_ || sequence == 0) return;
    ++denied_count_;
    state_ = "COMMAND_REJECTED_BY_ROS_SAFETY_GATE";
  }
  void stop() {
    std::lock_guard<std::mutex> lock(mu_);
    stopped_ = true;
    state_ = "STOP_LATCHED";
  }
  nlohmann::json status(const std::string& session, bool stop_latched) const {
    std::lock_guard<std::mutex> lock(mu_);
    if (stop_latched || stopped_) return {
        {"state", "STOP_LATCHED"}, {"decision_only", true},
        {"physical_stop_confirmed", false}};
    if (session_.empty() || session != session_) return {
        {"state", "COMMAND_SESSION_REQUIRED"}, {"decision_only", true}};
    return {{"state", state_}, {"mission_run_id", run_},
            {"last_sequence", last_sequence_}, {"submitted", submitted_},
            {"ros_safety_denials", denied_count_}, {"decision_only", true},
            {"mission_commands_enabled", false}, {"motor_commands_enabled", false},
            {"spray_commands_enabled", false}, {"physical_command_emitted", false}};
  }
 private:
  mutable std::mutex mu_;
  std::string session_, run_, state_{"NO_COMMANDS"};
  std::uint64_t last_sequence_{0}, submitted_{0}, denied_count_{0};
  bool stopped_{false};
};
} // namespace dyx::http_backend
