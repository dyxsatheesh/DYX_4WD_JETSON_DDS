# DYX Autonomous — Phase 2H4F: C++17 Live Mission Status and Telemetry Contract

**Prepared by Satheesh** | Windows development milestone | Hardware-free tests

## Goal

Offer the Android mission screen a transport-independent read-only representation of mission state, completed/failed/skipped/pending marking counts, active waypoint, battery voltage, optional PX4-reported remaining charge, GNSS RTK FIXED/FLOAT, satellites used, correction age and safety hold reason.

## Implementation

- `backend_live_status.hpp` adds the explicit `GET /api/v1/telemetry/live` contract and read-only `NativeBackendLiveStatus` assembler.
- Status comes from the *same* mission gateway used by the command API, never from an independent shadow mission.
- Safety/GNSS/battery comes from the last **actual** `gateway.tick()` output. Status reads never advance or grant control authority.
- GNSS FIXED only appears when a fresh, validated GNSS safety sample reports fix_type 6. FLOAT is type 5. Stale/failed data appears as `UNAVAILABLE`.
- Satellites are optional, verified against the same GPS device **and host reception timestamp** as the safety GNSS sample. The adapter must take `satellites_used` from the matching SensorGps sample; no satellite count is invented.
- Battery remaining percent is optional, and must be obtained from an actual PX4 BatteryStatus `remaining` fraction on the matching voltage/sample. **No voltage-only SOC estimate** is advertised as a sensor measurement.
- Marking radial accuracy is expressly unavailable until an identity- and freshness-validated position accuracy source is integrated. We do not claim 20 mm from RTK FIX alone.
- The response exposes no motion-permission bit, authentication secrets, spray release authorization or physical actuator results. Denied requests return no mission/telemetry details.
- A stale tick, backward tick time or PX4 telemetry integrity fault suppresses displayed sensor validity. A time regression latches a UI integrity fault; rebuilding the read model after a verified PX4 epoch change is required.
- This phase is pure C++17; **not** HTTP, WebSocket, JSON, TLS, DDS publishing, ROS2 installation or any hardware control.

## Explicit trust boundary

The future backend ingress must validate TLS client/authentication, permission scopes and request sizes; obtain monotonic times from the host; feed the same tick as mission runtime; and derive GPS/battery auxiliary fields from corresponding PX4 telemetry, not Android-provided JSON.

## Native offline tests

1. `backend_live_nominal_native_smoke` — mission progress and fresh FIXED/voltage/satellites data.
2. `backend_live_stale_native_smoke` — stale data suppression and monotonic integrity fault.
3. `backend_live_authority_native_smoke` — read permission, method, route and no information leaks.
4. `backend_live_sources_native_smoke` — wrong GPS source, mismatched battery, replay and FLOAT.

## Deferred acceptance

Jetson ROS2 Humble compilation; PX4 v1.17 `px4_msgs` compatibility; actual satellite/battery source mapping; client auth/HTTP/WebSocket data transport; full rover emergency-stop and RTK accuracy tests. All physical actuation remains disabled.
