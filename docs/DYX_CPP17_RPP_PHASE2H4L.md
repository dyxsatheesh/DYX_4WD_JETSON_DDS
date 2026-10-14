# DYX Autonomous — Phase 2H4L: Offline Android Live Telemetry Stream Recovery

**Prepared by Satheesh**
**Base:** Phase 2H4K (`41deb77`), branch `Jetson_DDS_Upgrade`.

## Purpose

Implement a small, offline C++17 read-only Server-Sent Events (SSE) policy for future Android telemetry clients. The earlier HTTP wire layer could format one event but did not own a server epoch, session identity, reconnect behavior, heartbeat schedule or monotonic event sequence.

## Delivered functionality

- Server-generated, epoch-scoped event IDs `epoch-sequence`, never derived from Android.
- Fresh authorized GET snapshots on every reconnect. `Last-Event-ID` is untrusted and used only as a cursor hint. There is NO event replay, backlog, or gap-fill guarantee; reconnect always sends an authoritative *current* snapshot.
- Session binding, per-poll scope revalidation and immediate disconnect after revocation; no telemetry on authorization failure.
- JSON value-change events and periodic bounded full-snapshot heartbeats. When telemetry expires, a new event explicitly changes to `fresh:false`, `rtk:UNAVAILABLE` rather than displaying old FIXED/satellite values.
- Host monotonic-time regression latches a stream fault. Only an independently verified operator with PX4 confirmed DISARMED can reset to a strictly newer trusted epoch. This resets **only the offline stream**, not PX4/safety/mission state.
- Config validation, maximum SSE event frame size, event-ID overflow guard, no network I/O, no physical actuation.

## API and limits

`NativeOfflineLiveStream` lives at `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_live_stream.hpp`. The caller supplies a trusted, durable unique epoch, verified principal and monotonic host clock; this code does not implement identity verification or persistence. Default snapshot-heartbeat interval is 2 seconds (configurable 0.1–30 seconds), maximum serialized frame 32768 bytes (configurable 1024–65536). A status event uses `event: rover.status` and `id: epoch-sequence`.

`delivered=true` only means *an event string was produced in memory*; it never proves that Android received the frame. It is not proof of GNSS accuracy, motor motion, paint spray, laser action or PX4 output. No GET status field grants control authority.

## Safety boundary

This is a **single-owner, single-subscriber, synchronous offline policy**, NOT a live network SSE socket server, Android client, WebSocket transport, DDS subscription, TLS endpoint or real authenticated ingress. Keep `NativeLoopbackReadOnlyHttpServer` GET-only and loopback-bound. A production integration must supply robust credentials/TLS or other authenticated transport, SSE write deadlines/backpressure, synchronized access, epoch persistence, per-subscriber limits, and stream lifecycle cancellation. The offline streaming contract should be reviewed with the actual Android client before deployment.

## Tests (Windows Visual Studio C++17)

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected: **100/100 tests** (96 prior + 4 new). ROS 2/Jetson/PX4/field acceptance deferred.
