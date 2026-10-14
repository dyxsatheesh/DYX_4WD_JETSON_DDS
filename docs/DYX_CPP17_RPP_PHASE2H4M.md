# Phase 2H4M — Android Telemetry Compatibility / Recovery (OFFLINE)

Prepared for DYX Autonomous 4WD marking rover — C++17 Windows/Linux native checks.

## Scope

Adds `backend_android_telemetry_contract.hpp`, a lightweight *offline C++17 replica* of the Android read-only mission telemetry screen. It consumes only **trusted in-process** SSE decisions created by Phase 2H4L; it is **not** an Android app, a general-purpose JSON or network parser, a network transport, or a replacement for authentication. Android Kotlin/Compose or React Native code will need its own independently reviewed client and JSON decoder.

- Require full SSE snapshots at initial connection, reconnection, and service epoch changes. Client Last-Event-ID is only a cursor hint, not identity/authority.
- Enforce monotonic positive event IDs, strict sequential ordering, and reject lost, duplicated, stale, or malformed frames. Failed validation clears displayed RTK FIXED and triggers a full resync.
- Display RTK FIXED **only** when the server's canonical telemetry snapshot reports fresh, integrity-good FIXED. A stale/fault response never retains the previous FIXED indication.
- Clear displayed telemetry on disconnect, event validation failure, heartbeat timeout, or host clock regression. A host clock fault remains latched until trusted disarmed recovery and a strictly newer service epoch.
- Require explicit read-only and no-hardware-command metadata; the Android replica provides **no control authority** and cannot activate motors, spray, or laser.
- All code is **hardware-free** and may be compiled on MSVC or GCC without Jetson, PX4, ROS2, DDS, mobile networking, or Android SDK.

## Boundaries and unimplemented items

SSE frame validation deliberately recognizes only the existing canonical backend JSON shape. It is **not** an arbitrary JSON decoder; it does not make raw internet traffic trustworthy. A real API gateway must separately implement TLS, identity verification, lifecycle limits, request authorization, and field-level Android JSON decoding. Phase 2H4L also does not yet attach SSE to a running socket listener. No motor or actuator commands are emitted by these tests. Passing native tests cannot substitute for on-device or vehicle safety acceptance.

## Tests

Four additional native CTest targets (total **104**):

1. `backend_android_nominal_native_smoke`: fresh FIXED, server stale transition, heartbeat.
2. `backend_android_reconnect_native_smoke`: disconnection, cursor hint, full resynchronization, replay rejection.
3. `backend_android_ordering_native_smoke`: malformed frame, lost/duplicate event, forged hardware flag.
4. `backend_android_safety_native_smoke`: no-heartbeat expiry, clock rollback, human/disarmed epoch recovery, no actuation.

Expected MSVC command sequence: configure `src/dyx_rpp_cpp17_core/test/native_cpp_only`, build `Release`, run `ctest -C Release --output-on-failure` and commit only after **104/104** pass.
