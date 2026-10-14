# DYX Autonomous — Phase 2H4J: Offline HTTP Request Lifecycle Hardening

**Prepared by Satheesh**
**Base:** `d202fc0` (`Jetson_DDS_Upgrade`)
**Verification platform:** Native C++17 with Visual Studio 2022 on Windows; GCC cross-check. **No Jetson required.**

## Goal

Establish a deterministic, strictly bounded, single-executor lifecycle for future trusted HTTP ingress and ensure the Android mission command/status route contract can be checked on a Windows PC without connecting to PX4.

## Delivered

- An **offline-only** admission/completion helper in `backend_request_lifecycle.hpp`. Never opens sockets, sends HTTP, authenticates credentials, or issues PX4 / motor / spray / laser commands.
- Configurable maximum active requests (default 4), maximum request JSON size (512 bytes), maximum formatted response size (16,384 bytes), arrival-age threshold (0.5 s), processing deadline (2 s). Invalid configurations throw.
- Distinct, trusted-ingress-generated **monotonic request IDs**, never IDs supplied inside Android JSON; replay/reuse rejected. Tickets are consumed even when completion is late or oversized.
- Explicit **epoch fencing**. Old responses cannot be accepted after an owner/PX4 epoch reset. A host monotonic clock regression latches a fault and requires independent operator authorization **and verified disarm** to reset.
- API compatibility for `GET /api/v1/mission/status`, `GET /api/v1/telemetry/live`, and exact POST mission command routes with strict two-field JSON. The typical Android `application/json` and `application/json; charset=UTF-8` content types are recognized. Unknown routes, wrong methods, extra JSON fields, oversized bodies and URL query fragments fail closed.
- **STOP is excluded from the ordinary queue** and classified as `STOP_REQUIRES_SEPARATE_VERIFIED_GATE`; no STOP action is executed. Production must route authenticated STOP through the existing independent mission safety gateway rather than wait on this ordinary request queue. This contract does not implement physical emergency stop.
- Four new deterministic C++17 regression tests for nominal completion, deadlines and clock faults, queue capacity/epoch fencing, and Android API route/payload compatibility.

## Usage / boundaries

This is a **single-owner, transport-independent helper**, not connected to `NativeLoopbackReadOnlyHttpServer` or the live authenticated-ingress contract. The current real TCP server remains localhost-only and GET-only. A production front end must separately implement TLS, genuine credential verification, request parsing/framing, thread synchronization, bounded socket read/write timeouts, authenticated authorization, rate limits, session persistence and independently verified safety hardware.

No test in this phase establishes that Android connectivity, ROS2, XRCE-DDS, PX4, actuator shutoff or GNSS tracking works on real rover hardware.

## Windows verification

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected **92/92** native tests (88 previous + 4 new). Only commit after all Windows tests pass.
