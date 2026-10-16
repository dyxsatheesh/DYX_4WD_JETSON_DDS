#include "dyx_rpp_cpp17_core/safety_mission_bridge.hpp"

#include <cstdlib>
#include <iostream>

using namespace dyx::rpp;

static int passed = 0;

static void check(bool condition, const char* name) {
    if (!condition) {
        std::cerr << "FAIL: " << name << '\n';
        std::exit(1);
    }
    ++passed;
    std::cout << "PASS: " << name << '\n';
}

static SafetyInput healthy(double t) {
    SafetyInput s{};
    s.now_sec = t;
    s.mission_running = true;
    s.operator_motion_enabled = true;
    s.emergency_stop = false;
    s.px4_offboard_armed = true;
    s.estimator_valid = true;
    s.px4_age_sec = 0.0;
    s.obstacle_clear = true;
    s.obstacle_age_sec = 0.0;
    s.battery_voltage_v = 25.0;
    s.battery_sample_time_sec = t;
    s.gnss_fix_type = 6;
    s.gnss_sample_time_sec = t;
    s.correction_stream_healthy = true;
    s.correction_age_sec = 0.2;
    return s;
}

static MissionSequenceInput evidence(double t) {
    MissionSequenceInput m{};
    m.monotonic_time_sec = t;
    m.emergency_stop = false;
    return m;
}

int main() {
    NativeSafetyMissionBridge bridge;

    bridge.load(
        {{0, 0, "P1", "H8F_SIM_P1"},
         {80, 1, "P2", "H8F_SIM_P2"}},
        MissionMode::manual, true, true
    );

    auto out = bridge.step(healthy(1.0), evidence(1.0));
    check(out.mission.stage == MissionStage::ready,
          "Healthy mission initially READY");

    check(bridge.start(1.01),
          "Healthy START accepted");

    out = bridge.step(healthy(1.1), evidence(1.1));
    check(out.mission.stage == MissionStage::approaching,
          "Mission APPROACHING");

    auto floating = healthy(2.0);
    floating.gnss_fix_type = 5;

    out = bridge.step(floating, evidence(2.0));

    check(!out.safety.motion_allowed,
          "RTK FLOAT immediately inhibits motion");

    check(out.mission.stage == MissionStage::paused,
          "RTK FLOAT pauses mission");

    check(!out.preview_forward_permitted,
          "No forward preview while FLOAT");

    check(!out.auto_resumed_float,
          "No automatic RESUME while FLOAT");

    check(!bridge.resume_manually(2.01),
          "Manual RESUME denied while FLOAT");

    out = bridge.step(healthy(3.0), evidence(3.0));

    check(out.safety.motion_allowed,
          "RTK FIXED restores safety eligibility");

    check(out.mission.stage == MissionStage::paused,
          "Mission remains PAUSED after FIXED recovery");

    check(!out.auto_resumed_float,
          "FIXED recovery never auto-resumes");

    check(!out.preview_forward_permitted,
          "No forward preview before manual RESUME");

    check(!bridge.resume_manually(3.30),
          "Stale safety authorization denies RESUME");

    check(bridge.resume_manually(3.01),
          "Fresh authorization permits manual RESUME");

    out = bridge.step(healthy(3.1), evidence(3.1));

    check(out.mission.stage == MissionStage::approaching,
          "Mission approaches only after manual RESUME");

    check(!out.auto_resumed_float,
          "Manual recovery is not reported as automatic");

    bridge.stop();

    check(bridge.mission().stage() == MissionStage::stopped,
          "Operator STOP accepted");

    std::cout
        << "ALL H8F MANUAL RESUME TESTS PASSED ("
        << passed << " checks)\n";

    return 0;
}
