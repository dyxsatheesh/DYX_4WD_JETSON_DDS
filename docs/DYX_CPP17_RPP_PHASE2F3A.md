# DYX Autonomous 4WD — Phase 2F3A: Native Forward-Only Mission-Leg Integration

**Target:** Windows C++17 native testing before Jetson integration. **Authority:** NON-ACTUATING.

**Patch base:** Phase 2F2 Python-free test lane, applied and committed after Jetson branch `78191e4` (Phase 2F1). This patch is additive: it does **not** remove Python source, introduce a reverse planner, or publish to `/fmu/in/*`.

## Scope

* New `mission_leg_preview.hpp` joins the existing path index, progress tracker, moving-lookahead steering, tracking recovery, longitudinal speed cap, terminal radial-stop certificate, and forward-only yaw-rate planner into one deterministic C++17 single-leg diagnostic flow.
* The ROS adapter must provide a **calibrated rear-master GNSS marking-reference position in local ENU**. Raw PX4 local position or IMU/chassis pivot-center coordinates are **not** interchangeable. The boolean marking-reference gate does not itself prove calibration.
* This preview requires mission permission, RTK FIX, fresh telemetry, battery/obstacle safety and no E-stop. A failed gate latches zero diagnostic commands for the remainder of that instance; recreating it requires an external mission-control decision. **This is deliberately stricter than the final 5-second RTK FLOAT debounce policy**, which must be integrated and tested separately.
* Velocity is always nonnegative and aligned with the vehicle's measured forward heading. A stationary differential pivot can have opposing wheel directions but has zero requested chassis translation.
* The terminal stop uses the actual goal coordinate, final inbound leg bearing, and position-derived stationary-window evidence. Virtual lookahead extension never moves the marking goal.
* Marking success is **estimator-frame evidence only** (20 mm tolerance), not a surveyed-world guarantee.
* No automatic transition to the next goal; no spraying, mission uploading, RTK caster, or production DDS motor command. No Jetson package build has been run.

## What is tested on Windows

Run the Python-free native C++17 lane:

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

**Expected: 12/12** native C++17 tests (previous 10 plus two new integration tests).

Critical test-harness fix: native Release builds now explicitly **undefine `NDEBUG`**, so legacy `assert(...)` statements run rather than silently compiling out. New tests use explicit runtime `CHECK(...)` that always executes, including Release.

New tests:

* `mission_leg_preview_native_smoke`: forward approach with positive but bounded preview speed, correct ENU goal identity, no yaw on straight, zero-latched radial stop, and stationary certificate after the observation window.
* `mission_leg_preview_safety_native_smoke`: missing marking-reference calibration, E-stop, RTK loss, negative measured speed, stale position timestamp, NaN heading, unauthorized stationary pivot, and missing marking anchor.

## Remaining migration work

Build a non-actuating full mission-sequencing layer, route/extension parser, marking/spray event engine, RTK FIX/FLOAT debounce/resume policy, NTRIP/serial transport, battery/obstacle safety controller, HTTP/WebSocket backend, and DDS command interface with independent source and hardware review. The Jetson C++17 ROS2 build, PX4 XRCE-DDS runtime, yaw-rate sign/frame, RoboClaw wheel behavior, GNSS lever arm and real-world stop accuracy **remain unverified**.

Do not delete or depend on a Python runtime for these new C++17 tests; keep Python source only as a historical migration reference until the native application replaces every feature and automated fixture.
