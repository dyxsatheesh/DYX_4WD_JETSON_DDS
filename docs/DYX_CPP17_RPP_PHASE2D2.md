# DYX 4WD — Phase 2D2: C++17 RPP terminal radial stop

**Base:** `dyxsatheesh/DYX_4WD_JETSON_DDS`, remote branch `Jetson_DDS_Upgrade`, commit `b3e3070`.

**Status:** Native C++17 mathematical library only. **Not connected to DDS, ROS2 motion, spray, PX4, or the rover launch.**

## Reference Python implementation

`src/rpp_controller/rpp_controller/terminal_stop_regulator.py`: `TerminalStopRegulator`, `RadialStopConfig`, `RadialStopInput`, `RadialStopOutput` and `RadialStopCertificate`. The prior C++17 Phase 2D1 `stationary_detector.hpp` is reused without changing its logic.

## Included

- Config validation; explicit `reset()` and `cancel()` boundaries.
- `TRACK -> TERMINAL_GUIDANCE -> BRAKE_PROFILE -> ZERO_LATCH -> SETTLE -> CERTIFIED -> HOLD_ZERO` state transitions (with actual Python transition order).
- Terminal braking profile driven by **position-derived speed**, bounded by the current tracking speed command.
- Goal-plane crossing or radial error <= 20 mm triggers irreversible zero-speed latch until an explicit reset.
- Position-window stationarity: 0.50 s, <= 5 mm window displacement, <= 0.05 rad/s yaw, max sample gap 0.20 s.
- Certificate version 2 includes terminal identity, timestamp, along/cross/radial error, final position and stationarity evidence. Certificate truth frame is **PX4 estimator frame only**.
- Fail-closed held-zero reasons: invalid measurements, stale telemetry, changed target identity, lost terminal request, terminal timeout, settle timeout, and settled outside tolerance.
- No reverse command, no pivot command, no side effects, and no automatic resume from a fail latch.

## Build and comparison test on Windows

From repo root:

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native -B build/dyx_rpp_native -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_rpp_native --config Release
ctest --test-dir build/dyx_rpp_native -C Release --output-on-failure
```

Expected **15 tests** including newly added `python_cpp_terminal_stop_parity` and `terminal_stop_native_smoke`. The parity test imports the original Python implementation from the same checkout, compares state/output fields and certificate data, and covers successful certification, recovery/hold paths, stale data, identity changes, invalid data, overshoot, position-sample time regression, and timeout.

## Important production limitations

- Passing mathematical parity does not prove the rover will stop within 20 mm in the physical field. The 20 mm comparison is against PX4 **estimator coordinates**, not an independent surveyed ground-truth mark.
- Speed/deceleration constants and yaw/position observability require on-robot calibration.
- The RPP C++17 controller, reverse-pivot motion FSM, yaw-rate output, DDS safety gate, battery lockout, RTK FLOAT behavior, obstacles and spray are NOT enabled or validated by this patch.
- Leave the Python RPP active, keep `enable_motion=false` in the C++ DDS bridge and do not connect a second active setpoint publisher during validation.
