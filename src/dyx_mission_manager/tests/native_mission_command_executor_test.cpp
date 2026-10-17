#include "dyx_mission_manager/native_mission_command_executor.hpp"
#include <cassert>
#include <deque>
#include <iostream>
#include <optional>
#include <string>
struct Command {
  std::string path, mission_run_id;
  unsigned sequence{};
  struct Principal { std::string operator_session_id; } principal;
};
struct Mailbox {
  std::deque<Command> queue;
  bool stopped{};
  std::optional<Command> pop_for_ros() {
    if (queue.empty()) return std::nullopt;
    Command value = queue.front(); queue.pop_front(); return value;
  }
  bool stopped_latched() const { return stopped; }
};
struct Audit {
  int denied_count{}, stop_count{};
  void denied(const std::string&, const std::string&, unsigned) { ++denied_count; }
  void stop() { ++stop_count; }
};
struct Ingress { int cancels{}; void cancel_on_stop() { ++cancels; } };
int main() {
  dyx::http_backend::NativeMissionCommandExecutor executor;
  Mailbox mailbox; Audit audit; Ingress upload, load;
  int stop_calls = 0;
  for (unsigned i = 1; i <= 7; ++i) mailbox.queue.push_back({"/start", "run", i, {"operator"}});
  auto result = executor.drain(mailbox, audit, [&]{ ++stop_calls; }, upload, load, "/stop");
  assert(result.processed == 4 && audit.denied_count == 4);
  assert(mailbox.queue.size() == 3 && stop_calls == 0);
  mailbox.queue.clear();
  mailbox.queue.push_back({"/stop", "run", 8, {"operator"}});
  mailbox.stopped = true;
  result = executor.drain(mailbox, audit, [&]{ ++stop_calls; }, upload, load, "/stop");
  assert(result.processed == 1 && result.stop_latched && stop_calls == 1);
  assert(upload.cancels == 1 && load.cancels == 1 && audit.stop_count == 1);
  assert(audit.denied_count == 4);
  std::cout << "PASS: bounded ROS-only command drain; positive denied; STOP invokes decision-only callback and cancels ingress\n";
}
