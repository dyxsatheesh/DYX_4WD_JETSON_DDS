# Phase 2H4B — DDS telemetry-to-mission safety runtime

**DYX Autonomous | Prepared by Satheesh | 13 October 2026**

**Baseline:** `Jetson_DDS_Upgrade` commit `36edb34` (Phase 2H4A).

## Purpose

Integrate the previously validated C++17 `NativeDdsSafetyTelemetry` with
`NativeSafetyMissionBridge` in one **non-actuating** offline-testable runtime.
The adapter consumes validated PX4-compatible observations and mission
evidence measured using the same host monotonic clock.

## Implemented contract

- All telemetry observations invalidate cached START/RESUME/NEXT/SKIP authority
  until the mission runtime executes a **new** safety tick. Even a duplicate
  observation cannot reuse older approval.
- Mission START, RESUME, NEXT and SKIP additionally use the bridge's ≤0.25 s
  fresh safety authority. PAUSE and STOP are never blocked by unhealthy PX4,
  RTK, battery, obstacle or DDS input.
- A mismatched evidence clock is handed to the existing mission bridge,
  which stops the mission. The wrapper never fabricates a matching timestamp.
- RTK FLOAT ≥5 s pauses while approaching, with FIXED-edge auto-resume only
  when all other safety conditions are healthy. No recovery from terminal
  marking/spray verification or an operator pause.
- Any malformed input, backward host time, backward PX4 sample time or GNSS
  master device change **latches a telemetry fault and immediately stops** the
  mission. Disarmed, explicitly authorized new PX4 epoch clears **all** prior
  telemetry evidence; previous mission cannot be resumed and must be reloaded.
- This module does not call a PX4 service, publish ROS 2, arm, disarm, generate
  motor setpoints, enable AUX5/AUX6, or claim physical emergency stop.
- Callers must serialize `observe_*()`, `step()` and operator command calls
  on one event loop or guard them with an external mutex; this pure C++17
  core does not implement concurrent callback synchronization.
- Motion-permitted and preview-permitted bits are **advisory for the current
  tick only**. A future production controller must independently enforce
  the real actuator/output and watchdog interlocks; NEVER cache these bits.

### Input trust boundaries

Operator authorization, physically disarmed PX4 state and the physical E-stop
come from externally verified adapters; passing `true` is **not** authorization
or authentication. RTCM correction health, obstacle clear, estimator validity
and per-goal RTK survey evidence must also be independently verified. Tests
inject synthetic evidence and cannot prove those external sensors.

## Windows verification

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

**Expected: 59/59 tests** (existing 55 plus 4). The added tests cover
nominal mission START, RTK FLOAT pause/FIXED recovery, GNSS source switch and
latch, reboot epoch reset, invalid evidence timing, and stale-gate revocation.

## Hardware deferred

No Jetson Orin Nano or PX4 hardware is needed for this phase. ROS 2 Humble
build, exact `px4_msgs` compatibility, safety executor/watchdog integration,
actual PX4 transport, motors, physical spray/laser OFF and field GNSS accuracy
remain explicitly pending in `DYX_CPP17_DEFERRED_JETSON_ACCEPTANCE.md`.
