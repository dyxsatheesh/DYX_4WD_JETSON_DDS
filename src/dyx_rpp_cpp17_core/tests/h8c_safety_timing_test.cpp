#include "dyx_rpp_cpp17_core/mission_safety.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

using dyx::safety::Arbiter;
using dyx::safety::Decision;
using dyx::safety::Input;
using dyx::safety::RtkState;

static int passed = 0;

static void check(bool condition, const std::string& name) {
    if (!condition) {
        std::cerr << "FAIL: " << name << '\n';
        std::exit(1);
    }
    ++passed;
    std::cout << "PASS: " << name << '\n';
}

static Input healthy(double now) {
    Input s{};
    s.now_s = now;
    s.rtk = RtkState::fixed;
    s.battery_v = 25.0;
    s.emergency_stop = false;
    s.estimator_valid = true;
    s.px4_heartbeat_s = now;
    s.backend_heartbeat_s = now;
    return s;
}

int main() {
    {
        Arbiter a;
        auto d = a.evaluate(healthy(10.0));
        check(d.start_resume_allowed && d.continue_allowed,
              "Healthy START/CONTINUE permitted");
    }

    {
        Arbiter a;
        auto s = healthy(10.0);
        s.rtk = RtkState::floating;

        auto d = a.evaluate(s);
        check(!d.continue_allowed && d.rtk_grace_active,
              "FLOAT at zero elapsed: immediately inhibited");

        s = healthy(14.999);
        s.rtk = RtkState::floating;
        d = a.evaluate(s);
        check(d.rtk_grace_active && !d.pause_required,
              "FLOAT 4.999 seconds: grace active");

        s = healthy(15.0);
        s.rtk = RtkState::floating;
        d = a.evaluate(s);
        check(d.rtk_grace_active && !d.pause_required,
              "FLOAT exactly 5 seconds: grace boundary");

        s = healthy(15.001);
        s.rtk = RtkState::floating;
        d = a.evaluate(s);
        check(d.pause_required && !d.continue_allowed,
              "FLOAT over 5 seconds: PAUSE required");

        d = a.evaluate(healthy(16.0));
        check(d.start_resume_allowed && d.continue_allowed,
              "RTK FIX recovery: eligible for explicit RESUME");
    }

    {
        Arbiter a;
        auto s = healthy(10.0);
        s.battery_v = 21.9;

        auto d = a.evaluate(s);
        check(!d.continue_allowed && !d.battery_lockout,
              "Battery low: immediate inhibit, debounce begins");

        s = healthy(39.999);
        s.battery_v = 21.9;
        d = a.evaluate(s);
        check(!d.battery_lockout && !d.continue_allowed,
              "Battery 29.999 seconds: no lockout yet");

        s = healthy(40.0);
        s.battery_v = 21.9;
        d = a.evaluate(s);
        check(d.battery_lockout && !d.start_resume_allowed,
              "Battery exactly 30 seconds: lockout latched");

        d = a.evaluate(healthy(41.0));
        check(d.battery_lockout && !d.continue_allowed,
              "Battery recovery cannot clear lockout");
    }

    {
        Arbiter a;
        auto s = healthy(10.0);
        s.battery_v = 21.5;
        a.evaluate(s);

        a.evaluate(healthy(20.0));

        s = healthy(25.0);
        s.battery_v = 21.5;
        a.evaluate(s);

        s = healthy(54.999);
        s.battery_v = 21.5;
        auto d = a.evaluate(s);
        check(!d.battery_lockout,
              "Battery recovery resets prior debounce");

        s = healthy(55.0);
        s.battery_v = 21.5;
        d = a.evaluate(s);
        check(d.battery_lockout,
              "New continuous 30-second low-battery interval latches");
    }

    {
        Arbiter a;
        auto s = healthy(10.0);
        s.battery_v = 22.0;
        const auto d = a.evaluate(s);
        check(d.start_resume_allowed && d.continue_allowed,
              "Battery exactly 22.0 V is not below threshold");
    }

    {
        Arbiter a;
        auto s = healthy(10.0);
        s.px4_heartbeat_s = 9.5;
        auto d = a.evaluate(s);
        check(d.continue_allowed,
              "PX4 heartbeat age exactly 0.5 seconds: fresh");

        s = healthy(10.001);
        s.px4_heartbeat_s = 9.5;
        d = a.evaluate(s);
        check(!d.continue_allowed &&
                  d.reason == "PX4_HEARTBEAT_STALE",
              "PX4 heartbeat beyond 0.5 seconds: stale");
    }

    {
        Arbiter a;
        auto s = healthy(10.0);
        s.backend_heartbeat_s = 8.5;
        auto d = a.evaluate(s);
        check(d.continue_allowed,
              "Backend heartbeat age exactly 1.5 seconds: fresh");

        s = healthy(10.001);
        s.backend_heartbeat_s = 8.5;
        d = a.evaluate(s);
        check(!d.continue_allowed &&
                  d.reason == "BACKEND_HEARTBEAT_STALE",
              "Backend heartbeat beyond 1.5 seconds: stale");
    }

    {
        Arbiter a;
        auto s = healthy(10.0);
        s.emergency_stop = true;
        auto d = a.evaluate(s);
        check(d.emergency_latched && !d.continue_allowed,
              "Emergency stop immediately latched");

        d = a.evaluate(healthy(11.0));
        check(d.emergency_latched && !d.start_resume_allowed,
              "Emergency latch survives healthy recovery");
    }

    {
        Arbiter a;
        a.evaluate(healthy(10.0));
        auto d = a.evaluate(healthy(9.0));
        check(!d.continue_allowed,
              "Backward monotonic clock inhibits");

        d = a.evaluate(healthy(11.0));
        check(!d.continue_allowed,
              "Clock fault remains latched");
    }

    std::cout << "ALL H8C SAFETY TIMING TESTS PASSED ("
              << passed << " checks)\n";
    return 0;
}
