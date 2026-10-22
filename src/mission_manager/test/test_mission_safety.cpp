
#include "dyx_mission_manager/native_mission_transition_gate.hpp"
#include "dyx_mission_manager/native_positive_command_preflight.hpp"

#include <cassert>
#include <iostream>

using namespace dyx::rpp;
using namespace dyx::http_backend;

static BackendMissionApiRequest request(const char* path) {
    BackendMissionApiRequest r{};
    r.method = "POST";
    r.path = path;
    r.principal.authenticated = true;
    r.principal.owns_control_lease = true;
    return r;
}

int main() {
    int checks = 0;

    const auto verify = [&](bool condition) {
        assert(condition);
        if (!condition) std::abort();
        ++checks;
    };

    struct Case {
        const char* path;
        MissionStage stage;
    };

    const Case eligible[] = {
        {kBackendMissionStartPath, MissionStage::ready},
        {kBackendMissionResumePath, MissionStage::paused},
        {kBackendMissionNextPath, MissionStage::await_next},
        {kBackendMissionSkipPath, MissionStage::approaching},
    };

    for (const auto& c : eligible) {
        auto d = assess_mission_transition(
            request(c.path), c.stage, false);
        verify(d.eligible_stage);
        verify(!d.dispatch_authorized);
        verify(d.block == MissionTransitionBlock::missing_safety_authority);
    }

    for (const auto& c : eligible) {
        auto d = assess_mission_transition(
            request(c.path), c.stage, true);
        verify(!d.dispatch_authorized);
        verify(d.block == MissionTransitionBlock::stop_latched);
    }

    {
        auto r = request(kBackendMissionStartPath);
        r.principal.authenticated = false;
        auto d = assess_mission_transition(r, MissionStage::ready, false);
        verify(d.block == MissionTransitionBlock::untrusted_principal);
        verify(!d.dispatch_authorized);
    }

    {
        auto r = request(kBackendMissionResumePath);
        r.principal.owns_control_lease = false;
        auto d = assess_mission_transition(r, MissionStage::paused, false);
        verify(d.block == MissionTransitionBlock::untrusted_principal);
        verify(!d.dispatch_authorized);
    }

    {
        auto d = assess_mission_transition(
            request(kBackendMissionStartPath),
            MissionStage::paused, false);
        verify(d.block == MissionTransitionBlock::wrong_stage);
        verify(!d.dispatch_authorized);
    }

    {
        auto r = request(kBackendMissionStartPath);
        r.method = "GET";
        auto d = assess_mission_transition(r, MissionStage::ready, false);
        verify(!d.dispatch_authorized);
    }

    {
        PositiveCommandEvidence e{};
        auto d = assess_positive_command_preflight(e);
        verify(d.missing_gate_count > 0);
        verify(!d.motion_authorized);
    }

    {
        PositiveCommandEvidence e{};
        e.mission_loaded = true;
        e.stop_latched = false;
        e.telemetry_fault = false;
        e.rtk_fixed = true;
        e.receiver_age_trusted = true;
        e.position_fresh = true;
        e.rear_reference_calibrated = true;
        e.battery_reserve_verified = true;
        e.obstacle_clear_verified = true;
        e.physical_estop_clear_verified = true;
        e.operator_lease_verified = true;
        e.px4_command_path_commissioned = true;
        e.spray_safety_commissioned = true;
        e.timing_fault = false;

        auto d = assess_positive_command_preflight(e);
        verify(d.diagnostic_preconditions_met);
        verify(!d.motion_authorized);
    }

    std::cout << "PASS: " << checks
              << " Mission Manager safety assertions\n";
}
