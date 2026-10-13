# DYX Autonomous — Phase 2H1B: C++17 safety-to-mission bridge

**Prepared by:** DYX Autonomous engineering
**Base repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS`, `Jetson_DDS_Upgrade` at `7de936d`
**Hardware:** Jetson Orin Nano + Pixhawk 6X, to be validated later
**Build target:** Windows VS2022 / C++17; ROS-free test lane

## Scope

`dyx_rpp_cpp17_core/safety_mission_bridge.hpp` integrates the previously ported
`NativeSafetyArbiter` (2H1A) with `MissionSequencer` (2G1A). It has **no ROS,
no Python, no MAVROS, and no actuation**. The return flag
`preview_forward_permitted` is strictly *advisory*, not permission for an
actuating ROS 2 DDS publisher.

* An active APPROACHING leg remains eligible to continue while RTK FLOAT is
  within its five-second confirmation window (other safety gates remain
  mandatory).
* After RTK FLOAT confirmation, the bridge invokes an internal mission PAUSE.
  Recovery is automatic only if the pause originated in APPROACHING, the
  original mission session is still enabled, and the arbiter emits a fresh
  FLOAT-to-RTK-FIXED recovery **edge** with all interlocks healthy.
* An RTK FLOAT pause during terminal verification or spray acknowledgment is
  **not auto-resumable**. The mission must not blindly re-enable movement
  after a settled or actively marking waypoint.
* RTK LOST, stale GNSS, stale NTRIP/RTCM, PX4 link/Offboard problems, obstacle,
  battery lock, explicit operator-disable, or any other safety fault revoke
  FLOAT-only automatic recovery. Manual recovery requires a fresh positive
  arbiter observation and explicitly authorized command.
* Emergency stop faults the mission and remains latched in the arbiter. A
  mismatched/bad monotonic clock stops the mission and invalidates cached
  command permission.
* Operator START, RESUME, NEXT and SKIP require a recent positive safety
  result; STOP never requires safety permission.
* Loading a mission requires an authorized request, verified disarmed state,
  and a non-active mission state. Snapshot authentication/run identities are
  owned by Phase 2G2B/2G2C; callers must use that verification before calling
  this bridge.
* RC remains permitted on battery lock; health telemetry and STOP remain
  available. E-stop deliberately denies RC until a separately authorized
  release and should also engage independent physical safety provisions.

## Integration contract

The caller must provide trustworthy `SafetyInput` and `MissionSequenceInput`
from the **same host monotonic timebase**. `SafetyInput.mission_running` means
an authorized mission *session* exists (including a FLOAT-paused session);
it does **not** request motors to run. `operator_motion_enabled` is always
checked independently. Do not feed estimator latitude/longitude in place of
raw RTK-FIXED receiver samples. `MissionSequenceInput` still requires
identity-matched terminal and survey evidence from the existing C++17
validators.

The future PX4 adapter must separately enforce armed state, telemetry QoS,
Offboard watchdog/setpoint deadlines, NED/ENU transforms, RC override and
stale-data response. The bridge does not call PX4, move motors, operate spray,
or certify 20 mm/30 mm real-world positioning.

## Windows offline verification

```
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Target: **31/31 Python-free native tests**. New tests:

* `safety_mission_float_native_smoke`: 5-second FLOAT confirmation, pause,
  fixed-edge automatic recovery, fresh START requirement and STOP.
* `safety_mission_fault_native_smoke`: obstacle manual-ack hold, emergency
  stop latching, stale action gate, disarmed acknowledgment and clock skew.
* `safety_mission_hold_native_smoke`: FLOAT during terminal verification must
  not automatically resume into the next motor-guidance tick.

## Not yet complete

No Jetson ROS 2 / PX4 DDS compilation; no physical motor/spray emergency-stop
verification; no hardware-triggered RC takeover; no safety integration into
live DDS publishers. Python reference packages remain in the repository until
production feature parity and final cutover. Final deployment has no Python
controller or reverse-driving maneuvers.
