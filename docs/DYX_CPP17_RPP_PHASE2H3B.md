# DYX 4WD Rover — Phase 2H3B: PX4 DDS Actuator Transport Safety Boundary

Prepared for DYX Autonomous — Windows native C++17 validation. Baseline remote
commit: `e131d14` on `Jetson_DDS_Upgrade`.

## Scope

This is an **offline/non-actuating** C++17 command transport boundary. It does
not publish ROS 2 topics or move AUX5/AUX6. Builds and tests using MSVC/GCC
without ROS 2, PX4, Jetson, MAVROS or Python. Its wire structure mirrors PX4
v1.17 `px4_msgs/msg/VehicleCommand`, converting parameters 1–4 and 7 to
`float32` and 5–6 to `float64` while leaving unrelated actuator sets as NaN.

Expected future PX4 ROS 2 topics, verify against **custom firmware**:

- `/fmu/in/vehicle_command` — `px4_msgs/msg/VehicleCommand` (C++ publisher)
- `/fmu/out/vehicle_command_ack` — `px4_msgs/msg/VehicleCommandAck` (subscriber)
- Command ID `187` (`MAV_CMD_DO_SET_ACTUATOR`)
- AUX5: actuator set 1; AUX6: actuator set 2
- PX4 clock synchronization, source/target IDs, type-support revision and QoS
  must be resolved in ROS 2 integration with the flashed PX4 firmware.

## Added classes

`px4_wire_command`: validates one pre-encoded `DdsActuatorIntent` and produces
field-typed `VehicleCommandWire` data, not a published command.

`NativeDdsActuatorTransport`: allows a single tracked ON transaction; performs
synchronous durable write-ahead journaling *before* returning an ON ticket;
never automatically retries ON after timeout; supports priority OFF overriding
uncertain ON; restarts can issue OFF only. OFF retries get new durable generation
numbers. Rejects stale/mismatched receipts and never clears the journal until
both a uniquely correlated, accepted OFF receipt and **independent physical
OFF evidence** are present.

### Critical limitation: PX4 command ACK is NOT request-correlated

PX4 v1.17 `VehicleCommandAck` reports the command number, result, timestamp,
target and `from_external`; it **does not echo an application dispatch ID or
sequence**. Consequently, observing `command=187` and `result=0` after a DDS
publish cannot uniquely prove which of two nearby/replayed 187 commands was
accepted. `observe_unkeyed_px4_ack(...)` therefore **never** marks either ON or
OFF successful. It is telemetry only.

`UniquelyCorrelatedReceipt` is a contract for a **future separately audited
correlated transport**; it must NOT be fabricated using `VehicleCommandAck`
alone or a timing window. The current stock PX4 topic pair does not satisfy
that requirement. Either add a verified transaction identity in an approved
custom firmware protocol or use a separately designed feedback/command path.
Without that proof, leave actuator dispatch inhibited and the durable lock
active. An accepted PX4 command is also **not physical position feedback**.

### Safety limits

The upstream write-ahead journal is single-process/single-writer; production
requires exclusive process ownership, validated directory permissions,
filesystem durability testing, resilient corrupt-file handling and a physical
fail-off mechanism independent of Jetson software. `source_system`,
`source_component`, PX4 timestamps, DDS topic QoS, actuator calibration and
GPIO/PWM disarmed behavior must be verified on hardware.

Do not connect any ON path to a real PX4 publisher in this phase.

## Native tests

Four new tests:

1. `dds_transport_wire_native_smoke` — mixed field types, NaN isolation,
   malformed wire-data rejection.
2. `dds_transport_correlated_native_smoke` — no success from generic ACK,
   matching receipt, OFF proof and replay safety.
3. `dds_transport_timeout_native_smoke` — no ON retry, priority OFF, rejected
   OFF response, stale response rejection.
4. `dds_transport_restart_native_smoke` — uncertain journal restart,
   durable generation, OFF-only recovery and permanent identity replay guard.

Expected full native suite: **48/48 tests**. Tests are software simulations,
not hardware or ROS 2 integration evidence.

## Next

Phase 2H3C: ROS 2 Humble `rclcpp` transport package, PX4 v1.17 `px4_msgs`
compatibility and topic/QoS verification, initially in **monitor/dry-run**
mode. Active ON dispatch must remain disabled until independently validated
ack correlation, safety, failsafe-off and physical feedback exist.
