# Phase 2H4A — C++17 DDS telemetry-to-safety bridge

**DYX Autonomous | Prepared by Satheesh | 13 October 2026**

**Baseline:** `Jetson_DDS_Upgrade`, Phase 2H3D verified Windows commit `4253fcb`.

## Purpose

Continue the Jetson C++17 production migration **without Jetson hardware**.
Implement a ROS-independent adapter that accepts selected fields from PX4 v1.17
`px4_msgs` messages and prepares the existing `SafetyInput` for the native
safety arbiter. No ROS runtime, MAVROS, Python backend, motor command, actuator
publisher, laser/spray output, or reverse-driving implementation is added.

The adapter makes no claim that DDS connectivity or real rover safety has
been verified. Phase 2H3D's Jetson compilation and graph checks remain deferred.

## PX4 release/1.17 source fields and mapping

| Message/producer | Required source fields | Native safety outcome |
|---|---|---|
| `BatteryStatus` | `timestamp`, `voltage_v`, `connected`, `cell_count`, `warning`, `is_powering_off` | Fresh pack voltage for 30 s debounce only if safe; severe PX4 warning/poweroff prevents motion immediately |
| `SensorGps` | **`timestamp_sample`**, `device_id`, `latitude_deg`, `longitude_deg`, `fix_type`, `system_error`, `rtcm_crc_failed` | GNSS fix and distinct-sensor-sample freshness; no fake FIX from duplicate packets |
| `VehicleStatus` | `timestamp`, `arming_state`, `nav_state`, `vehicle_type`, `failsafe`, `failure_detector_status` | OFFBOARD+armed+rover+no failsafe needed |
| `VehicleLocalPosition` | **`timestamp_sample`**, `xy_valid`, `v_xy_valid`, `heading_good_for_control`, `dead_reckoning`, `x`, `y`, `heading` | Independent finite and freshness estimator gate |
| Jetson NTRIP/receiver-health adapter (later) | Host monotonic receive time, independently measured correction age and health | Healthy RTCM/corrections, otherwise blocked |
| Obstacle adapter (later) | Host monotonic receive time and actual clear indication | Fresh obstacle clearance, otherwise blocked |

Field contracts checked against `PX4/px4_msgs` branch `release/1.17` and
PX4 1.17's DDS topic mapping. Do not interpret PX4 boot-time microseconds as
host time, and do not infer correction health from RTK FIX alone.

## Safety

- Unreceived, stale, invalid, duplicate, or source-switched data cannot authorize motion.
- A PX4 timestamp decrease is treated as a possible reboot/epoch change;
  stop and require authenticated, disarmed approval for new epoch initialization.
- Freshness is measured from *unique sample* acceptance in one host steady
  clock. Replayed GNSS `timestamp_sample` does not refresh RTK FIX.
- The first selected GPS `device_id` is pinned. Receiver changes require new
  authorized/disarmed epoch initialization; this prevents accidental antenna
  source swaps. Production must separately verify master antenna device ID.
- Critical PX4 battery warnings, imminent poweroff, disconnected battery, or
  unexpected nonzero cell count make battery evidence unusable, triggering an
  immediate safety hold. The 30-second <=22.0 V debounce remains in the arbiter.
- This phase prepares inputs only. The actual ROS2 subscribers, NTRIP health,
  corrections age, obstacle sensor adapter, time synchronization, mission
  runtime and physical safety tests are **NOT implemented or verified here**.
- `snapshot()` does **not** authorize driving or spraying; only the independently
  integrated and hardware-tested safety controller may do that in production.

## Windows verification

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected: **55/55** native C++17 tests (51 previous + 4 new).
Keep the Jetson-only Phase 2H3D verification in the deferred acceptance log.
