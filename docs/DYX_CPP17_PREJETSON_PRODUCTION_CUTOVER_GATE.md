# DYX Autonomous — Pre-Jetson production cutover gate
Prepared by Satheesh | 15 October 2026 | Based on `874ecfd75505727b42b3b7e32c7215a65f001dfe`

## Status
**NOT PRODUCTION READY.** The 339/339 H5U–H5Z tests validate legacy native code and non-actuating preflight diagnostics. They do NOT prove a production replacement for the Python/MAVROS runtime.

## This patch actually changes
1. Adds a cross-platform, ROS-free C++17 source-inventory executable with deterministic CTest regression tests. It reports the 11 major old runtime launch/dependency blockers and refuses to mistake observer or shadow nodes for complete production modules. JSON says `production_ready=false` unconditionally: no automatic release signature exists.
2. Hardens the opt-in experimental DDS bridge: when `enable_motion=false` it creates no PX4 Offboard keepalive/setpoint publications; it continues to publish only local diagnostic status. This prevents a DISABLED bridge from accidentally refreshing Offboard by default. Real PX4 watchdog/physical stop behavior remains unverified.
3. Provides offline Windows and WSL audit runners. Does not modify `rover.launch.py`, install services, publish actuators, or delete Python.

## Still required before declaring *software* cutover
- Complete actual `rclcpp` C++17 production nodes that call the tested core, with a single motion owner.
- Preserve functional parity for trajectory extension, mission lifecycle, marking, Android HTTP/SSE/commands, RTK/NTRIP/RTCM, battery and obstacle stops, spray/laser, logs, recovery and update semantics.
- Supply true production ROS 2 XML launch and systemd files with no MAVROS/Python application nodes; installable in an isolated ROS 2 Humble environment.
- Match PX4 v1.17 `px4_msgs` and ROS2 DDS QoS/timestamp behavior; simulate link loss, GNSS FLOAT, battery critical, stale commands and process restart. Test Android against a production API implementation, not a local Unix diagnostic socket.
- Run signed feature-by-feature trace parity against a fixed Python/reference golden-data corpus; then remove DYX-owned Python app files and MAVROS runtime. Removing files without parity is **not** completion.

## Required after Jetson and physical rover arrive
Verify Ethernet XRCE-DDS; GNSS and RTCM; RoboClaw & rear encoder; offboard motion control; 22 V lockout; RTK pause; independent E-STOP and actual sprayer OFF; physical marking accuracy; headless service/power cut; supervisor and RC takeover. Begin with power stages disabled.

## Run
**Ubuntu WSL2:** `bash scripts/DYX_PREJETSON_CUTOVER_AUDIT.sh`

**Windows PowerShell:** `powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/DYX_PREJETSON_CUTOVER_AUDIT.ps1`

Both generate `dyx_cutover_status.json` in an ignored build directory. Audit success means the *auditor* ran; it does NOT mean the rover passed cutover. The `--enforce` flag deliberately fails with exit code 2 to prevent CI falsely approving a release.

## Current audit evidence and gap ownership

| Work area | Current evidence at `874ecfd` | Native production acceptance still required |
|---|---|---|
| RPP straight/pivot control | `dyx_rpp_cpp17_core` guidance and math, DDS shadow diagnostic, Python `rpp_controller` launch | Trusted 50 Hz C++ `rclcpp` executor, recorded trajectory parity and rate/limits verification |
| Mission | Native mission sequence/recovery core and Python `mission_manager` launch | Start/pause/resume/next/skip/stop authority, restart journal, marking certificates under real ROS 2 |
| Trajectory | Native `trajectory_builder.hpp`, Python `trajectory_generator` runtime | Mission upload to live path generation, extension behavior, Android controls and point-to-point parity |
| Backend | Native HTTP/recovery diagnostics; Python FastAPI `rover_backend` still launched | Full authenticated Android REST/WebSocket/SSE and telemetry parity with native process |
| GNSS/RTK/NTRIP | Pure native telemetry gates, Python NTRIP/RTCM runtime | Real C++ TLS caster client, serial routing, correction ownership, RTK transitions, Linux permission handling |
| Safety | C++17 `safety_arbiter`, `dds_mission_runtime` units | Verified PX4 telemetry origin, physical e-stop, motor disable, obstacle and battery wiring, independent loss handling |
| Spray/laser | Native transaction/safety functions and Python spray service | Actual PX4 command delivery gated by authorization, OFF on arm/disarm/reboot with hardware proof |
| DDS | Opt-in disabled DDS command bridge and read-only observer | Firmware-compatible PX4 topic/schema discovery, single motion owner, offboard and QoS handling |
| Operations | Observer-only preflight systemd service | Installable complete native service, durable state, logs, watchdog and deterministic crash recovery |
| Python cutover | 322 project `.py` files in GitHub snapshot (includes tests/backups); old production launch uses Python/MAVROS | Substitute equivalent tested services first; then retire old launch and runtime dependencies |

**Conflicting project decisions to settle before actuation:** the checked-in `DYX_CPP17_NO_PYTHON_CUTOVER.md` specifies forward-only operation without reverse driving, whereas prior rover plans included reverse-to-pivot-center movement. Do not silently choose a motion behavior in a safety-critical production cutover.

## Important distinction about enforce mode
`dyx_cutover_audit --enforce` is intentionally a **release barrier**, returning nonzero even if the 11 static legacy markers are absent. This tool does not have access to Android acceptance evidence, ROS 2 hardware connectivity or measured stopping accuracy. It is designed to make an unsupported 'all complete' claim impossible, not to substitute for the full functional tests.
