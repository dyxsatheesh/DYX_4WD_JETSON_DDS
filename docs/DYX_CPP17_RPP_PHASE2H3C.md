# Phase 2H3C — Native C++17 ROS 2 DDS actuator monitor

**DYX Autonomous | Prepared by Satheesh | 13 October 2026**

**Baseline:** `Jetson_DDS_Upgrade`, Phase 2H3B commit `de1afac`.

## Purpose

Create the first ROS 2 Humble C++17 DDS-monitor node without enabling any
PX4 commands, motor movement, spray or laser actuation. Match PX4
`VehicleCommandAck` topic layout while the Jetson implementation is being
prepared and keep native Windows tests independent of ROS 2.

## Files

- `src/dyx_dds_actuator_monitor`: `ament_cmake` ROS 2 node, read-only topic
  subscriber and ROS graph introspection for `/fmu/vehicle_command` service.
- `dds_actuator_monitor.hpp`: pure C++17 input-observation/health helper,
  explicitly cannot authorize actuator dispatch/transaction success.
- Three native tests: observation freshness, replay/invalid-input safety,
  configuration validation.

## Safety rules

- No publisher, service client, or PX4 command transmit path in this phase.
- Raw `VehicleCommandAck` indicates only a command/result observation; it
  cannot identify an individual AUX5/AUX6 command by journal generation.
- `result=ACCEPTED` never proves unique dispatch, physical ON or OFF, or
  marking completion. Duplicate, out-of-order, malformed, stale, and clock-
  invalid observations do not enable any command.
- A PX4 reboot resets its uptime timestamp: restart/reinitialize the
  monitor and independently verify safety; do not accept older ACKs.
- Actuator command publisher and service client are deliberately absent.
- For future live transport, verify the exact firmware `dds_topics.yaml`,
  `px4_msgs` revision, service availability, QoS, release status, and physical
  OFF. The service response can correlate an actual request; a shared ACK
  topic alone cannot.

## Windows verification

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

**Expected:** 51/51 native C++17 tests.

The ROS 2 package requires a separate **Jetson ROS 2 Humble build** (no ROS 2
or px4_msgs installation is provided by this Windows lane). No physical
command is executed; no field readiness, GNSS tolerance or fail-safe
performance is certified by native smoke tests.
