# DYX Autonomous — Phase 2H1A: Native C++17 Safety Arbitration

**Prepared by Satheesh | DYX Autonomous**
**Source baseline:** `Jetson_DDS_Upgrade` commit `fbe1554aa2c17dcf042f46efec6da3a7453f9094`.
**Module:** `dyx_rpp_cpp17_core/safety_arbiter.hpp` — pure C++17, no ROS/Python/MAVROS requirement, no motor/servo writer.

## Safety rules

| Condition | Decision |
|---|---|
| Fresh RTK FIXED (`fix_type=6`) | Normal forward motion *may* be permitted, only if every other gate passes |
| Fresh RTK FLOAT (`fix_type=5`) | A float blip shorter than 5.0 seconds does not pause; FLOAT persisting for 5 seconds of consecutively received fresh samples pauses |
| RTK FLOAT -> FIXED | Emits a one-tick `auto_resume_rtk_float` intent immediately on a valid FIXED sample **only if no other safety gate blocks it** |
| RTK degraded, missing, stale, or out-of-order | Immediate motion hold; explicit manual acknowledgment before restart |
| LiFePO4 pack <=22.00 V | 30-second debounce requiring >=2 distinct fresh low-voltage samples; pending state reported as 11% for display |
| Confirmed critical battery | Latched mission lockout; operator acknowledgment and valid replacement voltage >=22.50 V with PX4 disarmed are required before manual restart |
| Stale/invalid battery voltage | Fail-closed; no simulated battery voltage is substituted |
| Obstacle detected or stale obstacle status | Motion hold and manual recovery |
| Invalid PX4 estimator, stale PX4 state, not armed/offboard | Motion hold and manual recovery |
| RTCM source unhealthy or corrections older than 20 seconds | Motion hold and manual recovery |
| Emergency stop | Latched no-motion state; blocks RC motor control too, while telemetry and STOP remain available |
| Battery lockout | RC control stays available for safe recovery; START/RESUME/NEXT/SKIP are disabled, STOP is available, telemetry stays live |
| Clock regression | Fail-closed and require manual recovery |

**Architecture boundary:** `motion_allowed` is an advisory output and NOT PX4 actuation. The later production ROS 2 adapter must independently enforce all fail-safe gating on outgoing setpoints, stale command watchdogs, hardware E-stop and mode changes. An auto-resume intent alone does **not** resume `MissionSequencer`; that integration is still pending. No backward-chassis-driving code is included.

**Deliberate policy update:** existing Python `rtk_runtime_policy.py` at the verified repository revision defaults to `float_confirm_sec=1.0` and `fixed_hold_sec=0.5`. This C++17 phase implements the rover's subsequently requested 5-second FLOAT confirmation and immediate FIXED auto-resume **intent**. Exact stopping/spray interaction still needs full-system tests.

## Windows (no Jetson / Python)

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected **28/28** native tests (previous 25 and 3 new safety tests). A Linux/GCC C++17 reconstruction from the project phase source archives passed 28/28 during patch preparation. Test on the Windows repository before committing.

## Pending work

1. DDS/PX4 and actual Jetson transport, clock domains and telemetry source verification.
2. ROS 2 safety node and binding to native mission sequencer and RPP.
3. Native spray actuator, motor disable/E-stop electrical integration and physical tests.
4. Native C++ backend, RTK/NTRIP injection, obstacle driver, status/report persistence and final Python removal.
5. Real-world battery voltage calibration, RTK field observations and acceptance of marking accuracy.

This patch does not alter the Pixhawk firmware, RoboClaw driver or Android app. It does not assert the rover is deployment-ready.
