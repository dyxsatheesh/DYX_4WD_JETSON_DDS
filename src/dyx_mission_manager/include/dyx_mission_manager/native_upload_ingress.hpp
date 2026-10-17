#pragma once
// Batch 5: bounded, ROS-owned mission UPLOAD ingress. HTTP only submits;
// the ROS executor is the only mutator of NativeControlOwner.
// This component neither supplies PX4 evidence nor authorizes actuation.
#include <nlohmann/json.hpp>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace dyx::http_backend {

class NativeUploadIngress final {
 public:
  using Clock = std::chrono::steady_clock;
  struct Pending {
    nlohmann::json payload;
    std::string session_id;
    std::uint64_t sequence{0};
    Clock::time_point submitted_at{};
  };
  enum class LeaseResult { acquired, already_owned, conflict, stopped, invalid };
  enum class EnqueueResult { accepted, no_lease, busy, stopped, invalid };

  LeaseResult acquire(const std::string& session_id, bool stopped) {
    if (session_id.empty() || session_id.size() > 128) return LeaseResult::invalid;
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped || stop_latched_) return LeaseResult::stopped;
    if (owner_session_.empty()) {
      owner_session_ = session_id;
      return LeaseResult::acquired;
    }
    if (owner_session_ == session_id) return LeaseResult::already_owned;
    return LeaseResult::conflict;
  }

  EnqueueResult enqueue(const std::string& session_id,
                        nlohmann::json payload,
                        bool stopped) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped || stop_latched_) return EnqueueResult::stopped;
    if (session_id.empty() || session_id != owner_session_)
      return EnqueueResult::no_lease;
    if (!payload.is_object()) return EnqueueResult::invalid;
    if (pending_) return EnqueueResult::busy;
    // At most one request is pending. No caller-supplied sequence or evidence.
    pending_ = Pending{std::move(payload), session_id, ++next_sequence_, Clock::now()};
    last_status_ = {{"state", "UPLOAD_QUEUED"}, {"decision_only", true},
                    {"motor_commands_enabled", false},
                    {"spray_commands_enabled", false},
                    {"physical_command_emitted", false}};
    return EnqueueResult::accepted;
  }

  std::optional<Pending> pop_for_ros(bool stopped) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped || stop_latched_) {
      stop_locked_();
      return std::nullopt;
    }
    if (!pending_) return std::nullopt;
    // Request is never executed if it waited too long for the ROS executor.
    if (Clock::now() - pending_->submitted_at > std::chrono::milliseconds(500)) {
      last_status_ = {{"state", "UPLOAD_REJECTED"},
                      {"detail", "EXPIRED_BEFORE_ROS_EXECUTION"},
                      {"decision_only", true}};
      pending_.reset();
      return std::nullopt;
    }
    processing_sequence_ = pending_->sequence;
    auto out = std::move(pending_);
    pending_.reset();
    return out;
  }

  void publish_result(const std::string& session_id,
                      int http_status,
                      const std::string& code,
                      const std::string& run_id,
                      const std::string& signature,
                      std::uint64_t revision) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_latched_ || session_id != owner_session_) return;
    staged_sequence_ = http_status == 200 ? processing_sequence_ : 0;
    last_status_ = {{"state", http_status == 200 ? "UPLOAD_STAGED" : "UPLOAD_REJECTED"},
                    {"result_http_status", http_status}, {"detail", code},
                    {"mission_run_id", run_id}, {"signature", signature},
                    {"revision", revision}, {"decision_only", true},
                    {"mission_loaded", false}, {"mission_commands_enabled", false},
                    {"motor_commands_enabled", false},
                    {"spray_commands_enabled", false},
                    {"physical_command_emitted", false}};
  }

  nlohmann::json status_for(const std::string& session_id,
                            bool stopped) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped || stop_latched_) {
      return {{"state", "STOP_LATCHED"}, {"decision_only", true},
              {"physical_stop_confirmed", false}};
    }
    if (session_id.empty() || session_id != owner_session_) {
      return {{"state", "CONTROL_LEASE_REQUIRED"}, {"decision_only", true}};
    }
    return last_status_;
  }

  // Read-only saved draft identity. HTTP cannot claim PX4 disarm.
  bool staged_for(const std::string& session, const std::string& run,
                  const std::string& signature, std::uint64_t revision) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !stop_latched_ && !owner_session_.empty() &&
           owner_session_ == session &&
           last_status_.value("state", std::string{}) == "UPLOAD_STAGED" &&
           last_status_.value("mission_run_id", std::string{}) == run &&
           last_status_.value("signature", std::string{}) == signature &&
           last_status_.value("revision", std::uint64_t{0}) == revision;
  }
  std::uint64_t staged_sequence_for(const std::string& session) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_latched_ || session != owner_session_ ||
        last_status_.value("state", std::string{}) != "UPLOAD_STAGED") return 0;
    return staged_sequence_;
  }
  void cancel_on_stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_locked_();
  }

  bool stop_latched() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stop_latched_;
  }

 private:
  void stop_locked_() {
    stop_latched_ = true;
    pending_.reset();
    owner_session_.clear();
    last_status_ = {{"state", "STOP_LATCHED"}, {"decision_only", true}};
  }
  mutable std::mutex mutex_;
  std::string owner_session_;
  std::optional<Pending> pending_;
  nlohmann::json last_status_{{"state", "NO_UPLOAD_SUBMITTED"},
                              {"decision_only", true}};
  std::uint64_t next_sequence_{0};
  std::uint64_t processing_sequence_{0};
  std::uint64_t staged_sequence_{0};
  bool stop_latched_{false};
};
}  // namespace dyx::http_backend
