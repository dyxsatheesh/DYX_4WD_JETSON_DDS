#pragma once

#include "dyx_rpp_cpp17_core/backend_mission_api.hpp"

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace dyx::http_backend {

class NativeCommandMailbox final {
 public:
  using Request = dyx::rpp::BackendMissionApiRequest;

  static constexpr std::size_t kCapacity = 32;

  enum class SubmitResult {
    accepted,
    full,
    rejected_unverified,
    stopped_latched
  };

  // Caller must be a trusted authentication adapter, not an
  // HTTP JSON deserializer. This verification flag is only a
  // transitional interface; production must use an
  // unforgeable server-side verified-session capability.
  SubmitResult submit_verified(
      Request request,
      bool trusted_authentication_verified) {

    if (!trusted_authentication_verified ||
        !request.principal.authenticated)
      return SubmitResult::rejected_unverified;

    std::lock_guard<std::mutex> lock(mutex_);

    if (request.path ==
        dyx::rpp::kBackendMissionStopPath) {
      // Cancel every queued normal command immediately.
      // No queued START or RESUME can follow this STOP.
      stopped_latched_ = true;
      queue_.clear();

      if (!pending_stop_)
        pending_stop_ = std::move(request);

      return SubmitResult::accepted;
    }

    // This mailbox is permanently closed to normal commands
    // after STOP in this development build. Recovery will
    // require an independently verified new-session design.
    if (stopped_latched_)
      return SubmitResult::stopped_latched;

    if (queue_.size() >= kCapacity)
      return SubmitResult::full;

    queue_.push_back(std::move(request));
    return SubmitResult::accepted;
  }

  // Called exclusively by the ROS executor.
  // STOP always takes priority over normal commands.
  std::optional<Request> pop_for_ros() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (pending_stop_) {
      auto value = std::move(pending_stop_);
      pending_stop_.reset();
      return value;
    }

    if (queue_.empty())
      return std::nullopt;

    Request value = std::move(queue_.front());
    queue_.pop_front();
    return value;
  }

  std::size_t queued_normal() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  bool stop_pending() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_stop_.has_value();
  }

  bool stopped_latched() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stopped_latched_;
  }

 private:
  mutable std::mutex mutex_;
  std::deque<Request> queue_;
  std::optional<Request> pending_stop_;
  bool stopped_latched_{false};
};

}  // namespace dyx::http_backend
