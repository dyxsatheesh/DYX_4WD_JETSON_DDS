#pragma once
// H7: ROS-free simulation safety-policy candidate. NOT a hardware motion permit.
#include <cmath>
#include <optional>
#include <string>

namespace dyx::safety {
enum class RtkState { unknown, fixed, floating };

struct Input {
    double now_s{0.0};                 // monotonic seconds
    RtkState rtk{RtkState::unknown};
    double battery_v{0.0};
    bool emergency_stop{true};
    bool estimator_valid{false};
    double px4_heartbeat_s{-1.0};     // last received monotonic timestamp
    double backend_heartbeat_s{-1.0};
};

struct Decision {
    bool start_resume_allowed{false};
    bool continue_allowed{false};
    bool pause_required{true};
    bool battery_lockout{false};
    bool emergency_latched{false};
    bool rtk_grace_active{false};
    std::string reason{"NOT_EVALUATED"};
};

class Arbiter final {
public:
    Decision evaluate(const Input& s) {
        // Invalid or regressed clock latches a fault. Never auto-reset safety state.
        if (!std::isfinite(s.now_s) || s.now_s < 0.0 ||
            (last_time_ && s.now_s < *last_time_)) {
            clock_fault_ = true;
            return deny("INVALID_OR_BACKWARD_CLOCK");
        }
        last_time_ = s.now_s;
        if (s.emergency_stop) emergency_latched_ = true;
        if (std::isfinite(s.battery_v) && s.battery_v < 22.0) {
            if (!low_battery_since_) low_battery_since_ = s.now_s;
            if (s.now_s - *low_battery_since_ >= 30.0) battery_lockout_ = true;
        } else {
            // The debounce counter resets if voltage recovers, but lockout never auto-clears.
            low_battery_since_.reset();
        }
        if (s.rtk == RtkState::floating) {
            if (!float_since_) float_since_ = s.now_s;
        } else {
            float_since_.reset();
        }

        if (clock_fault_) return deny("CLOCK_FAULT_LATCHED");
        if (emergency_latched_) return deny("EMERGENCY_STOP_LATCHED");
        if (battery_lockout_) return deny("BATTERY_LOCKOUT_LATCHED");
        if (!std::isfinite(s.battery_v) || s.battery_v <= 0.0)
            return deny("INVALID_BATTERY_TELEMETRY");
        if (!s.estimator_valid) return deny("INVALID_ESTIMATOR");
        if (!fresh(s.now_s, s.px4_heartbeat_s, 0.5))
            return deny("PX4_HEARTBEAT_STALE");
        if (!fresh(s.now_s, s.backend_heartbeat_s, 1.5))
            return deny("BACKEND_HEARTBEAT_STALE");
        // Separate immediate autonomous-motion inhibition from the delayed
        // mission-state pause/lockout timers. These decisions are diagnostic
        // policy outputs, never hardware motor/servo authorization.
        if (s.battery_v < 22.0)
            return {false, false, false, false, false, false,
                    "BATTERY_BELOW_22V_DEBOUNCE"};
        if (s.rtk == RtkState::fixed)
            return {true, true, false, false, false, false, "RTK_FIXED_HEALTHY"};
        if (s.rtk == RtkState::floating && float_since_ &&
            s.now_s - *float_since_ <= 5.0)
            return {false, false, false, false, false, true,
                    "RTK_FLOAT_GRACE_MOTION_INHIBITED"};
        return deny(s.rtk == RtkState::floating ? "RTK_FLOAT_TIMEOUT" : "RTK_NOT_FIXED");
    }

private:
    static bool fresh(double now, double last, double maximum_age) {
        return std::isfinite(last) && last >= 0.0 && last <= now &&
               now - last <= maximum_age;
    }
    Decision deny(const char* why) const {
        return {false, false, true, battery_lockout_, emergency_latched_, false, why};
    }
    std::optional<double> last_time_;
    std::optional<double> low_battery_since_;
    std::optional<double> float_since_;
    bool battery_lockout_{false};
    bool emergency_latched_{false};
    bool clock_fault_{false};
};
} // namespace dyx::safety
