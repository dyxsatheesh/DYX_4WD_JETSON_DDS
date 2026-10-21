#pragma once
// D1A: single ROS-owner command dispatch, non-actuating.
// STOP has priority, PAUSE needs backend-authenticated control lease,
// START/RESUME/NEXT/SKIP remain denied until commissioned.
#include <cstddef>

namespace dyx::http_backend {
class NativeMissionCommandExecutor final {
 public:
  static constexpr int kMaxDecisionsPerTick = 4;
  struct Result {
    std::size_t processed{0};
    std::size_t pause_accepted{0};
    std::size_t pause_denied{0};
    std::size_t positive_denied{0};
    bool stop_latched{false};
  };

  template <typename Mailbox, typename Audit, typename StopCallback,
            typename PauseCallback, typename UploadIngress,
            typename LoadIngress, typename StopPath, typename PausePath>
  Result drain(Mailbox& mailbox, Audit& audit, StopCallback&& stop_callback,
               PauseCallback&& pause_callback, UploadIngress& upload,
               LoadIngress& load, const StopPath& stop_path,
               const PausePath& pause_path) const {
    Result result{};
    for (int i = 0; i < kMaxDecisionsPerTick; ++i) {
      auto command = mailbox.pop_for_ros();
      if (!command) break;
      if (command->path == stop_path) {
        stop_callback();
      } else if (command->path == pause_path) {
        // The mailbox's authentication gate alone is not a control lease.
        // The caller's callback is the sole ROS-owner of mission state.
        if (command->principal.authenticated &&
            command->principal.owns_control_lease &&
            pause_callback(*command)) {
          ++result.pause_accepted;
        } else {
          audit.denied(command->principal.operator_session_id,
                       command->mission_run_id, command->sequence);
          ++result.pause_denied;
        }
      } else {
        // Positive commands must NEVER be dispatched by D1A.
        audit.denied(command->principal.operator_session_id,
                     command->mission_run_id, command->sequence);
        ++result.positive_denied;
      }
      ++result.processed;
    }
    result.stop_latched = mailbox.stopped_latched();
    if (result.stop_latched) {
      upload.cancel_on_stop();
      load.cancel_on_stop();
      audit.stop();
    }
    return result;
  }
};
} // namespace dyx::http_backend
