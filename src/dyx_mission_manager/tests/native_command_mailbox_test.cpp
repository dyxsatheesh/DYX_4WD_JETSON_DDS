#include "dyx_mission_manager/native_command_mailbox.hpp"

#include <cassert>
#include <iostream>

using dyx::http_backend::NativeCommandMailbox;

static NativeCommandMailbox::Request make_request(
    const char* path,
    unsigned long long sequence) {
  NativeCommandMailbox::Request r;
  r.method = "POST";
  r.path = path;
  r.sequence = sequence;
  r.principal.authenticated = true;
  r.principal.owns_control_lease = true;
  r.principal.operator_session_id = "test_session";
  r.mission_run_id = "test_mission";
  return r;
}

int main() {
  using Result = NativeCommandMailbox::SubmitResult;

  NativeCommandMailbox inbox;

  const auto start = make_request(
      dyx::rpp::kBackendMissionStartPath, 1);

  // An HTTP caller setting authenticated=true is not sufficient.
  assert(inbox.submit_verified(start, false) ==
         Result::rejected_unverified);

  assert(inbox.queued_normal() == 0);

  // Verify normal-command FIFO before STOP.
  assert(inbox.submit_verified(
      make_request(dyx::rpp::kBackendMissionPausePath, 10),
      true) == Result::accepted);

  assert(inbox.submit_verified(
      make_request(dyx::rpp::kBackendMissionResumePath, 11),
      true) == Result::accepted);

  auto first = inbox.pop_for_ros();
  auto second = inbox.pop_for_ros();

  assert(first && first->sequence == 10);
  assert(second && second->sequence == 11);

  // Fill the queue to its configured capacity.
  for (std::size_t i = 0;
       i < NativeCommandMailbox::kCapacity; ++i) {
    assert(inbox.submit_verified(
        make_request(
            dyx::rpp::kBackendMissionPausePath,
            i + 100),
        true) == Result::accepted);
  }

  assert(inbox.queued_normal() ==
         NativeCommandMailbox::kCapacity);

  assert(inbox.submit_verified(start, true) ==
         Result::full);

  // STOP must bypass queue capacity and cancel old commands.
  assert(inbox.submit_verified(
      make_request(dyx::rpp::kBackendMissionStopPath, 999),
      true) == Result::accepted);

  assert(inbox.stopped_latched());
  assert(inbox.stop_pending());
  assert(inbox.queued_normal() == 0);

  // A subsequent START must not be accepted, even with
  // the trusted-submission flag supplied.
  assert(inbox.submit_verified(start, true) ==
         Result::stopped_latched);

  auto stop = inbox.pop_for_ros();

  assert(stop);
  assert(stop->path == dyx::rpp::kBackendMissionStopPath);
  assert(stop->sequence == 999);

  // Nothing may execute after the STOP.
  assert(!inbox.pop_for_ros().has_value());

  // Duplicate STOP is idempotent at the mailbox boundary.
  assert(inbox.submit_verified(
      make_request(dyx::rpp::kBackendMissionStopPath, 1000),
      true) == Result::accepted);

  auto repeated_stop = inbox.pop_for_ros();
  assert(repeated_stop);
  assert(repeated_stop->path ==
         dyx::rpp::kBackendMissionStopPath);

  assert(!inbox.pop_for_ros().has_value());
  assert(inbox.stopped_latched());

  std::cout
      << "PASS: Native mailbox FIFO, capacity, "
         "STOP cancellation and lockout\n";
}
