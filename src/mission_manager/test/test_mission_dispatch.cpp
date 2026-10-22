#include "dyx_mission_manager/native_command_mailbox.hpp"
#include "dyx_mission_manager/native_mission_command_executor.hpp"
#include "dyx_mission_manager/native_mission_transition_gate.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace dyx::rpp;
using namespace dyx::http_backend;

static void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

struct FakeAudit {
    int denials = 0;
    int stops = 0;

    void denied(const std::string&,
                const std::string&,
                std::uint64_t) {
        ++denials;
    }

    void stop() { ++stops; }
};

struct FakeIngress {
    int cancellations = 0;
    void cancel_on_stop() { ++cancellations; }
};

static BackendMissionApiRequest make_request(
    const char* path,
    bool lease = true) {

    BackendMissionApiRequest r{};
    r.method = "POST";
    r.path = path;
    r.mission_run_id = "test_run";
    r.sequence = 1;
    r.host_received_sec = 1.0;
    r.principal.authenticated = true;
    r.principal.owns_control_lease = lease;
    r.principal.operator_session_id = "test_operator";
    return r;
}

int main() {
    int checks = 0;

    auto verify = [&](bool condition, const char* message) {
        check(condition, message);
        ++checks;
    };

    NativeMissionCommandExecutor executor{};

    // CASE 1: Valid stage is not sufficient to authorize START.
    {
        NativeCommandMailbox mailbox{};
        FakeAudit audit{};
        FakeIngress upload{}, load{};
        int stops = 0;

        verify(mailbox.submit_verified(
            make_request(kBackendMissionStartPath), true)
            == NativeCommandMailbox::SubmitResult::accepted,
            "START enqueue failed");

        auto result = executor.drain(
            mailbox, audit,
            [&] { ++stops; },
            [&](const BackendMissionApiRequest&) { return false; },
            [&](const BackendMissionApiRequest& r) {
                return assess_mission_transition(
                    r, MissionStage::ready, false);
            },
            upload, load,
            kBackendMissionStopPath,
            kBackendMissionPausePath);

        verify(result.positive_denied == 1,
               "START not denied");
        verify(result.positive_stage_eligible_denied == 1,
               "START eligibility audit failed");
        verify(stops == 0,
               "Unexpected STOP");
        verify(audit.denials == 1,
               "Missing denial audit");
    }

    // CASE 2: PAUSE without control lease must not dispatch.
    {
        NativeCommandMailbox mailbox{};
        FakeAudit audit{};
        FakeIngress upload{}, load{};
        int pause_calls = 0;

        verify(mailbox.submit_verified(
            make_request(kBackendMissionPausePath, false), true)
            == NativeCommandMailbox::SubmitResult::accepted,
            "PAUSE enqueue failed");

        auto result = executor.drain(
            mailbox, audit,
            [] {},
            [&](const BackendMissionApiRequest&) {
                ++pause_calls;
                return true;
            },
            [&](const BackendMissionApiRequest& r) {
                return assess_mission_transition(
                    r, MissionStage::paused, false);
            },
            upload, load,
            kBackendMissionStopPath,
            kBackendMissionPausePath);

        verify(result.pause_denied == 1,
               "Unleased PAUSE not denied");
        verify(pause_calls == 0,
               "Unleased PAUSE reached owner");
    }

    // CASE 3: STOP cancels an earlier queued START.
    {
        NativeCommandMailbox mailbox{};
        FakeAudit audit{};
        FakeIngress upload{}, load{};
        int stops = 0;
        int transitions = 0;

        verify(mailbox.submit_verified(
            make_request(kBackendMissionStartPath), true)
            == NativeCommandMailbox::SubmitResult::accepted,
            "START enqueue failed");

        verify(mailbox.submit_verified(
            make_request(kBackendMissionStopPath), true)
            == NativeCommandMailbox::SubmitResult::accepted,
            "STOP enqueue failed");

        auto result = executor.drain(
            mailbox, audit,
            [&] { ++stops; },
            [&](const BackendMissionApiRequest&) { return false; },
            [&](const BackendMissionApiRequest& r) {
                ++transitions;
                return assess_mission_transition(
                    r, MissionStage::ready, true);
            },
            upload, load,
            kBackendMissionStopPath,
            kBackendMissionPausePath);

        verify(stops == 1,
               "STOP was not executed");
        verify(transitions == 0,
               "Queued START executed after STOP");
        verify(result.stop_latched,
               "STOP not latched");
        verify(upload.cancellations == 1 &&
               load.cancellations == 1,
               "STOP did not cancel ingress");
        verify(mailbox.submit_verified(
            make_request(kBackendMissionResumePath), true)
            == NativeCommandMailbox::SubmitResult::stopped_latched,
            "RESUME accepted after STOP");
    }

    std::cout << "PASS: " << checks
              << " native mission dispatch checks\n";
    return EXIT_SUCCESS;
}
