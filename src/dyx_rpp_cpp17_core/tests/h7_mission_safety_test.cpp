#include "mission_safety.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
using dyx::safety::Arbiter;
using dyx::safety::Input;
using dyx::safety::RtkState;
void check(bool ok, const std::string& description) {
    if (!ok) throw std::runtime_error("FAIL: " + description);
    std::cout << "PASS: " << description << '\n';
}
Input healthy(double t) {
    Input x;
    x.now_s = t;
    x.rtk = RtkState::fixed;
    x.battery_v = 25.0;
    x.emergency_stop = false;
    x.estimator_valid = true;
    x.px4_heartbeat_s = t;
    x.backend_heartbeat_s = t;
    return x;
}
int main() {
    try {
        { Arbiter a;
          auto s = healthy(0); auto d = a.evaluate(s);
          check(d.start_resume_allowed && d.continue_allowed, "Healthy FIX permits START/RESUME");
          s.rtk = RtkState::floating; s.now_s = 1; s.px4_heartbeat_s=1; s.backend_heartbeat_s=1;
          d=a.evaluate(s);
          check(!d.start_resume_allowed && !d.continue_allowed && !d.pause_required && d.rtk_grace_active, "FLOAT grace blocks new START");
          s.now_s=6; s.px4_heartbeat_s=6; s.backend_heartbeat_s=6;
          d=a.evaluate(s);
          check(!d.continue_allowed && !d.pause_required, "FLOAT exactly 5s keeps motion inhibited");
          s.now_s=6.01; s.px4_heartbeat_s=6.01; s.backend_heartbeat_s=6.01;
          d=a.evaluate(s);
          check(d.pause_required && !d.continue_allowed && d.reason=="RTK_FLOAT_TIMEOUT", "FLOAT over 5s requests PAUSE");
          s.rtk=RtkState::fixed; s.now_s=7; s.px4_heartbeat_s=7; s.backend_heartbeat_s=7;
          d=a.evaluate(s);
          check(d.start_resume_allowed, "FIX recovery permits reauthorization (not auto-resume)");
        }
        { Arbiter a; auto s=healthy(0); s.battery_v=21.9;
          auto d=a.evaluate(s);
          check(!d.battery_lockout && !d.start_resume_allowed && !d.continue_allowed && !d.pause_required && d.reason=="BATTERY_BELOW_22V_DEBOUNCE", "Low voltage immediately inhibits but starts debounce");
          s.now_s=29.9; s.px4_heartbeat_s=29.9; s.backend_heartbeat_s=29.9;
          d=a.evaluate(s);
          check(!d.battery_lockout && !d.continue_allowed && !d.pause_required, "Low voltage less than 30s remains inhibited, not latched");
          s.now_s=30; s.px4_heartbeat_s=30; s.backend_heartbeat_s=30;
          d=a.evaluate(s);
          check(d.battery_lockout && d.pause_required, "Battery under 22V for 30s locks mission");
          s.now_s=31; s.battery_v=25; s.px4_heartbeat_s=31; s.backend_heartbeat_s=31;
          d=a.evaluate(s);
          check(d.battery_lockout, "Battery lockout persists after voltage recovery");
        }
        { Arbiter a; auto s=healthy(0); s.battery_v=21.9; a.evaluate(s);
          s=healthy(20); a.evaluate(s); s=healthy(21); s.battery_v=21.9; a.evaluate(s);
          s.now_s=40; s.px4_heartbeat_s=40; s.backend_heartbeat_s=40;
          check(!a.evaluate(s).battery_lockout, "Recovered voltage resets debounce timer");
        }
        { Arbiter a; auto s=healthy(0); s.battery_v=22.0;
          auto d=a.evaluate(s);
          check(d.start_resume_allowed && d.continue_allowed, "Battery exactly 22V is not below threshold");
        }
        { Arbiter a; auto s=healthy(0); s.rtk=RtkState::floating;
          auto d=a.evaluate(s);
          check(!d.start_resume_allowed && !d.continue_allowed && !d.pause_required && d.rtk_grace_active,
                "FLOAT immediately inhibits without formal pause");
          s=healthy(4.9);
          d=a.evaluate(s);
          check(d.start_resume_allowed && d.continue_allowed && !d.pause_required,
                "FIX before five seconds clears FLOAT debounce");
        }
        { Arbiter a; auto s=healthy(0); s.battery_v=21.9; a.evaluate(s);
          s=healthy(29.9); s.battery_v=21.9;
          check(!a.evaluate(s).battery_lockout, "Battery threshold not latched before 30 seconds");
          s=healthy(30.0); s.battery_v=21.9;
          auto d=a.evaluate(s);
          check(d.battery_lockout && !d.continue_allowed && d.pause_required,
                "Battery lockout latches at exactly 30 seconds");
        }
        { Arbiter a; auto s=healthy(0); s.emergency_stop=true;
          auto d=a.evaluate(s);
          check(d.emergency_latched && !d.continue_allowed, "Emergency stop latches immediately");
          s=healthy(1);
          check(a.evaluate(s).emergency_latched, "E-stop remains latched after input clears");
        }
        { Arbiter a; auto s=healthy(2); s.px4_heartbeat_s=1;
          check(a.evaluate(s).reason=="PX4_HEARTBEAT_STALE", "Stale PX4 heartbeat denies");
        }
        { Arbiter a; auto s=healthy(2); s.backend_heartbeat_s=0;
          check(a.evaluate(s).reason=="BACKEND_HEARTBEAT_STALE", "Stale backend heartbeat denies");
        }
        { Arbiter a; auto s=healthy(0); s.estimator_valid=false;
          check(a.evaluate(s).reason=="INVALID_ESTIMATOR", "Invalid EKF denies");
        }
        { Arbiter a; auto s=healthy(0); s.rtk=RtkState::unknown;
          check(a.evaluate(s).pause_required, "Unknown RTK denies");
        }
        { Arbiter a; auto s=healthy(1); a.evaluate(s); s=healthy(0.5);
          check(a.evaluate(s).pause_required, "Backward monotonic clock denies");
        }
        std::cout << "ALL H7 SAFETY ARBITER TESTS PASSED\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n'; return 1;
    }
}
