# Phase 2H3A — Native C++17 PX4 actuator DDS payload and write-ahead journal

**DYX Autonomous • Prepared by Satheesh • 13 October 2026**

## Goal

Continue migrating the marking rover from Python/MAVROS to C++17/ROS 2/XRCE-DDS without enabling any motors or actuators. This is a **non-actuating foundation**, not a working ROS 2 transport node and **not ready for field deployment**.

## Implemented

1. `px4_actuator_dds_intent.hpp` converts mission spray, manual spray and laser output *intents* into an independently checked representation of PX4 `VehicleCommand` number **187**, `MAV_CMD_DO_SET_ACTUATOR`.
2. AUX5 spray uses actuator set 1 (candidate `PWM_AUX_FUNC5=301`). AUX6 laser uses actuator set 2 (candidate `PWM_AUX_FUNC6=302`). Other actuator set fields stay **NaN**, avoiding accidental commands to the other tool. `param7=0` for the first actuator group. No motor command is generated.
3. **PWM microseconds must not be sent directly as normalized PX4 parameters.** With an *explicitly configured and verified* example range `1000–2000 μs`, 1200 μs OFF maps to **−0.6**, and 1400 μs ON maps to **−0.2**. The laser currently uses normalized +1.0 ON/−1.0 OFF. Real channel min/max, direction, disarmed values and hardware behavior must be checked on the exact Pixhawk 6X firmware.
4. `actuator_write_ahead_journal.hpp` creates a file-backed, sync-before-command **write-ahead lock** for every ON intent. An uncertain ON or OFF blocks all new ON intents across AUX5 and AUX6 owners. A run-scoped identity is retained as used across restarts to block replay.
5. After restart, the only supported recovery is **OFF**. A local FSM command sequence may restart at 1; the file journal has its **own persisted dispatch generation** for response correlation. A matching accepted OFF transport response still does **not** clear the journal without independent physical-safe-OFF verification.
6. POSIX writes use `fflush` + `fsync` of the file followed by an atomic same-directory rename and parent directory `fsync`. On Windows, `_commit` is used before rename. Failed writes and corrupted existing journals fail closed.

## Not implemented yet

- No `rclcpp` node, PX4 `px4_msgs::msg::VehicleCommand` publisher or DDS `/fmu/vehicle_command` service client in this phase. The next phase must confirm which interface and message version are available in the custom **PX4 v1.17.0** firmware and use matched `px4_msgs` definitions.
- No actual servo PWM, laser activation, NTRIP interaction, PX4 arming, reverse-driving maneuver or rover motor control.
- No physical output feedback sensor or proof that the servo/laser is OFF. A PX4 command acknowledgment is **not** such proof.
- No multi-process locking or secure deployment directory management. The journal currently assumes **one trusted writer**, on a filesystem that supports the required flush/rename semantics; verify actual power-loss/restart behavior and permissions on Jetson storage before production.
- No shared DDS command dispatcher/queue. Multiple command-187 requests **must not** be correlated from the command ID alone; production transport must use a single-flight or specifically matched request/response mechanism and handle timeouts/failures.

## Windows verification

Baseline: GitHub `Jetson_DDS_Upgrade` commit `29886d1` (Phase 2H2C).

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected: **44 native C++17 tests** (41 existing + 3 new). The three new tests cover normalized actuator payloads and set isolation, journal crash/restart and sequence reset, and replay/ACK/corruption safety.

**Important:** These are **offline behavioral tests**, not integration or safety certification. Do not connect loads or enable AUX5/AUX6 output on the strength of these tests.
