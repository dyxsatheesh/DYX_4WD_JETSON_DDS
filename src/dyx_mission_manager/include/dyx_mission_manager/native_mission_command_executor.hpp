#pragma once
// Mission command executor: single ROS-owner execution and fail-closed decisions.
// NO PX4 command publication, no hardware STOP acknowledgement, no positive dispatch.
#include <cstddef>

namespace dyx::http_backend {
class NativeMissionCommandExecutor final {
 public:
  static constexpr int kMaxDecisionsPerTick = 4;
  struct Result { std::size_t processed{0}; bool stop_latched{false}; };

  template <typename Mailbox, typename Audit, typename StopCallback,
            typename UploadIngress, typename LoadIngress, typename StopPath>
  Result drain(Mailbox& mailbox, Audit& audit, StopCallback&& stop_callback,
               UploadIngress& upload, LoadIngress& load,
               const StopPath& stop_path) const {
    Result result{};
    for (int i = 0; i < kMaxDecisionsPerTick; ++i) {
      auto command = mailbox.pop_for_ros();
      if (!command) break;
      if (command->path == stop_path) {
        stop_callback();
      } else {
        // Deliberately NO positive mission action call here.
        audit.denied(command->principal.operator_session_id,
                     command->mission_run_id, command->sequence);
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
}  // namespace dyx::http_backend
