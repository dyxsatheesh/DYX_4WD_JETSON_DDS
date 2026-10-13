# DYX 4WD — Phase 2E1: verified pivot state-machine parity

**Base:** `dyxsatheesh/DYX_4WD_JETSON_DDS` / `Jetson_DDS_Upgrade` / `cd7e1ad`.

**Reference:** `src/rpp_controller/rpp_controller/motion_state_machine.py` — `VerifiedPivotStateMachine`, `PivotMotionConfig`, input/result/certificates.

**Output status: non-actuating library only.** No firmware, ROS launch, DDS bridge, actual yaw-rate setpoint, reversing, spray, battery logic or motor command is changed.

### Implemented

- Tracks `TRACK`, `CORNER_APPROACH`, `BRAKE_TO_ANCHOR`, `STOP_SETTLE`, `PIVOT`, `PIVOT_SETTLE`, `POSITION_CHECK`, `RECENTER`, `REALIGN`, `RECAPTURE` and `HOLD_FAIL`.
- Returns explicit directives plus separate measured stop and heading-release certificates.
- Checks bounded cycle dt, monotonic time, heading error, radial position, linear speed, yaw rate, stale telemetry, dwell gates, recenter attempt limits and watchdog timeouts.
- Tests a normal repeated pivot, failed release/recenter/stale telemetry and 650 seeded pseudo-random state cycles against original Python source.
- **Do not interpret directive `PIVOT` as permission to command yaw rate.** A later, separate adapter must enforce real safety authorities, require valid GNSS/RTK, and validate PX4 rover offboard yaw-rate mode on the actual installed firmware.

### Windows native parity test

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native -B build/dyx_rpp_native -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_rpp_native --config Release
ctest --test-dir build/dyx_rpp_native -C Release --output-on-failure
```

Expected 17 tests, including `python_cpp_pivot_motion_parity` and `pivot_motion_native_smoke`. All must pass before committing.

### Next steps / gates

1. Review whether the legacy FSM's **20 mm anchor recenter** assumption matches the planned **reverse-then-pivot center capture** behavior; these are different geometry contracts. Do not replace the current production logic blindly.
2. Port straight-reverse/pivot planner separately with measured 4WD wheelbase, track, GNSS-to-pivot offsets and correct motor direction; older `reverse_arc_pivot.py` carries field-derived left/right offset values that require review.
3. Implement yaw-rate generation with ENU-positive-CCW to PX4 NED sign conversion, clamp and failsafe, plus rover-specific safety gates.
4. Reconcile RTK, battery lockout, obstacle stop and spray/marking ownership in Jetson ROS2 node; replay field recordings; bench test with wheels raised before enabling any movement.

Python RPP is still the rover's baseline; 17 native tests are **not production acceptance**.
