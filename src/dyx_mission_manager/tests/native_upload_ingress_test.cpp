#include "dyx_mission_manager/native_upload_ingress.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
using dyx::http_backend::NativeUploadIngress;
int main() {
  NativeUploadIngress inbox;
  assert(inbox.acquire("session_alpha", false) == NativeUploadIngress::LeaseResult::acquired);
  assert(inbox.acquire("session_alpha", false) == NativeUploadIngress::LeaseResult::already_owned);
  assert(inbox.acquire("session_other", false) == NativeUploadIngress::LeaseResult::conflict);
  assert(inbox.enqueue("session_other", {{"mission_run_id", "m_run_a01"}}, false) == NativeUploadIngress::EnqueueResult::no_lease);
  assert(inbox.enqueue("session_alpha", {{"mission_run_id", "m_run_a01"}}, false) == NativeUploadIngress::EnqueueResult::accepted);
  assert(inbox.enqueue("session_alpha", {{"mission_run_id", "m_run_a02"}}, false) == NativeUploadIngress::EnqueueResult::busy);
  auto cmd = inbox.pop_for_ros(false);
  assert(cmd && cmd->sequence == 1 && cmd->session_id == "session_alpha");
  inbox.publish_result("session_alpha", 412, "PX4_DISARM_REQUIRED", "m_run_a01", "", 0);
  assert(inbox.status_for("session_alpha", false).at("detail") == "PX4_DISARM_REQUIRED");
  assert(inbox.status_for("session_other", false).at("state") == "CONTROL_LEASE_REQUIRED");
  assert(inbox.enqueue("session_alpha", {{"mission_run_id", "m_run_a03"}}, false) == NativeUploadIngress::EnqueueResult::accepted);
  inbox.cancel_on_stop();
  assert(!inbox.pop_for_ros(false));
  assert(inbox.acquire("session_alpha", false) == NativeUploadIngress::LeaseResult::stopped);
  assert(inbox.enqueue("session_alpha", {{"mission_run_id", "m_run_a04"}}, false) == NativeUploadIngress::EnqueueResult::stopped);
  assert(inbox.status_for("session_alpha", false).at("state") == "STOP_LATCHED");
  std::cout << "PASS: Native lease exclusivity, queue limits, ROS consumption, STOP cancellation/lockout\n";
}
