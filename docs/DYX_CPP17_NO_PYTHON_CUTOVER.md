# DYX Autonomous — C++17-only rover software cutover

**Baseline:** `Jetson_DDS_Upgrade` at commit `78191e4` (Phase 2F1 shadow node).
**Product decision:** remove **all DYX-owned Python** from the *final repository and deployed application*. There is **no Python controller and no Python rollback mode**. **No vehicle reverse-driving code** is to be added. The Android application is a separate Kotlin + Compose migration.

## Current baseline: incomplete; do not delete functional services yet

The verified branch contained **319 project `.py` files** on 2026-10-13 (264 under `src/`, including test modules and backups). Approximately 104 under `src/` are production/source candidates after excluding `test` and obvious backup paths. These figures are a source-inventory snapshot, **not** a line-by-line production-dependency audit. The C++17 implementation presently contains an RPP math/state-machine core, a DDS bridge (motion disabled by default), and a DDS shadow node. None alone provides the full commercial rover workflow.

| Existing Python package | `.py` files in src package | Required production C++17 replacement |
|---|---:|---|
| rover_backend | 82 | HTTP/WebSocket backend, authentication, mission upload and updates, reporting, battery policy, RTK/NTRIP controls, settings and telemetry |
| rpp_controller | 51 | ROS2 C++ executor, guidance, speed, terminal stop, pivot, path handover, point events; **forward-only** |
| mission_manager | 30 | Mission state machine, waypoint/marking policy, RTK and battery pauses, mission recovery |
| rtk_correction_bridge | 29 | Secure NTRIP, RTCM parser, UART sink, correction freshness, lifecycle |
| trajectory_generator | 17 | Mission geometry, extension ON/OFF, serpentine and segment generation |
| dyx_rpp_cpp17_core | 12 | Replace Python-driven parity/source tests with checked-in native C++ fixtures and property tests |
| jetson_4wd_control | 11 | Eliminate MAVROS command/telemetry bridge; retain DDS bridge only |
| rover_bringup | 11 | XML launch, C++ node supervision, startup configuration, systemd |
| spray_controller | 9 | AUX5 spray/laser mode and timing, strict no-actuation on arm/disarm |
| localization | 6 | DDS localization transforms, estimator/RTK health gates |
| obstacle_monitor | 6 | Obstacle sensor decoding, zones and fail-safe stop |

The repo also contains legacy Python backups and utility scripts outside `src/`. Remove them at cutover; do not use a file deletion as a substitute for implementing their required behavior. Python code may remain temporarily as a **reference for development and numerical comparison only**; it is not a permitted fallback in production.

## Offline-only Windows development sequence

1. **Phase 2F2 (this patch):** add Python-free native C++17 test lane and read-only cutover auditor. It does not modify the operational controller.
2. **Phase 2F3:** complete ROS2 C++17 RPP executor and message contracts, using the existing tested core, with no motor publication until gates are validated.
3. **Phase 2G:** port mission state, geometry/extension, marking/spray policy and point/event certification into C++17; compare recorded mission outputs.
4. **Phase 2H:** port NTRIP/RTCM, RTK FIX/FLOAT pause rules, GNSS lever arms, battery threshold/debounce and obstacle gates into native C++17.
5. **Phase 2I:** replace the FastAPI Python backend with native C++17 HTTP/WebSocket endpoints preserving the existing Android API contracts; move bringup to XML launch and native systemd units.
6. **Phase 2J:** remove every DYX-owned Python file, setup.py, Python test runner, Python-based service and MAVROS runtime wiring. Native tests + golden-fixture recordings become release gates. Python-only reference tests must be replaced, **not just deleted to turn failing tests green**.
7. **Final hardware phase:** on Jetson Orin Nano + Pixhawk 6X confirm Humble/px4_msgs compatibility, build, QoS, DDS handshake, RTK, battery and obstacle failsafes, controlled rover motion, spray interlock, point accuracy and longitudinal/yaw-rate tuning. **No hardware acceptance can be claimed from Windows compilation.**

## Absolute technical rules

- Single motion owner: `rclcpp` C++17 RPP command publisher → native C++17 DDS bridge → custom PX4 v1.17.0 combined velocity + body-rate Offboard. No concurrent MAVROS publisher.
- Forward velocity only; at turns use zero-translation differential stationary pivot. No reverse-to-pivot-center or reverse-arc logic.
- RTK correction path and waypoint behavior remain fail-safe: FLOAT pause after the approved debounce, resume on FIX; speed 0.6–1.0 m/s target; 800 mm acceleration/deceleration zones; LiFePO4 8S 22.0 V mission-lockout policy; no spray output during arm/disarm.
- Protect against stale position/heading, GNSS jumps, DDS loss, emergency stop, battery lockout, obstacle stop, unbounded speed/yaw-rate, duplicate command publisher and missing mission state. No deliberate rotation on a hard safety stop.
- Keep the PX4 rear-master GNSS marking reference and pivot-center geometry separated. A 20 mm estimator-frame radial stop is not proof of 20 mm ground-truth marking accuracy.
- Product release gate: `tools/dyx_cpp17_python_cutover_audit.ps1 -Enforce` returns success, production build/CI uses no project Python code, all C++ modules and recorded mission equivalence tests pass, and Jetson/PX4 hardware acceptance has been completed.

**Note:** ROS 2 Humble `colcon`, message generation and OS tools can internally require system Python. "Python-free application" means there is **no DYX-owned Python application code, runtime entry point or Python service**, not that Python is uninstalled from Ubuntu or ROS 2.

## Build the separate Python-free C++ test lane (Windows)

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
powershell -NoProfile -ExecutionPolicy Bypass -File tools/dyx_cpp17_python_cutover_audit.ps1
```

This lane contains **10 existing native C++ smoke tests** and **does not invoke the Python executable**. Keep the existing 22-test parity lane during conversion only; retire it after replacing comparisons with native C++ tests and checked-in golden fixtures.
