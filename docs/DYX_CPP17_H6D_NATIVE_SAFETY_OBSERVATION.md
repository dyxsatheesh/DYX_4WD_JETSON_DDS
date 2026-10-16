# DYX Autonomous — H6D Native DDS Safety Observation
Prepared by Satheesh's engineering workflow | Baseline `db100a3ec0307021150a2e83305fa7174cb383c8`

## Actual change
- Adds a C++17 ROS2 rclcpp **observation-only** node for PX4 v1.17 DDS BatteryStatus, SensorGps, VehicleStatus, VehicleLocalPosition.
- Projects real message fields into the EXISTING `NativeDdsSafetyTelemetry` and `NativeSafetyArbiter` C++17 model. Receipts use the host monotonic clock; PX4 timestamp/sample timestamp only guards source sequence.
- Exercises existing 8S 22.00V 30-second debounce and 5-second RTK FLOAT confirmation in offline C++17 regression tests; does NOT change threshold formulas or release modes.
- Detects selected GPS source changes, stale/invalid frames, and PX4 estimator resets. Reports only diagnostic `std_msgs/String` on `/dyx/h6d/safety_observer/status`.
- `observe_enabled=false` default; optional standalone `safety_observe.launch.xml` for a later supervised, DISARMED, read-only integration test.

## Critical limitations — explicitly INCOMPLETE
- **NO trusted RTCM source or obstacle evidence** is wired: both are *always reported unverified*. Untrusted topics must not be accepted as clearance proofs.
- The arbiter is stepped with a **hypothetical** mission-active control condition strictly to inspect safety states. This is never a request to arm or move. `PHYSICAL_COMMAND_PERMITTED=false`, `MISSION_TRANSITION_PERMITTED=false` and `SPRAY_PERMITTED=false` remain invariant, even if the underlying health model independently reports READY.
- No actual NTRIP TLS client, RTCM caster connectivity, rover motor, firmware, QGC, PX4 agent, ROS 2 colcon build, Android command handler, or production systemd change.
- The default GPS topic `/fmu/out/vehicle_gps_position` might not be streamed by the customized PX4 DDS `dds_topics.yaml`; verify firmware export and `px4_msgs` definitions before ROS2 integration. Alternative `gps_topic:=/fmu/out/sensor_gps` accepted only if actually exported. There is no fallback to the wrong sensor.
- Battery `cell_count=0` means unknown according to PX4 semantics and core currently admits it; this is not proof an 8S pack is connected. Verify smart power monitor in hardware.
- A 22.0V 30-second debounce protects *battery-critical lockout*; genuine imminent poweroff/warning failures must immediately inhibit through the platform hardware safety chain, not rely on this observer.
- No automatic FIX recovery or mission pause is actuated by this node. H6D cannot supersede an independently verified physical E-stop.

## Offline tests
Windows PowerShell: `powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/DYX_H6D_WINDOWS_OFFLINE_TEST.ps1`
Ubuntu WSL2: `bash scripts/DYX_H6D_UBUNTU_OFFLINE_TEST.sh`

ROS2 Humble build (when available, do not launch motors):
`source /opt/ros/humble/setup.bash && colcon build --packages-up-to dyx_safety_cpp17_observer --symlink-install`

**Acceptance before physical operation:** prove px4_msgs v1.17 schema, PX4 DDS topic export, trusted correction/obstacle adapters, E-stop and RC semantics, battery source/8S calibration, GNSS/RTK pause and resume on hardware; then update the broader cutover audit and production launch separately. Offline passing tests do not clear the 11 legacy runtime blockers.
