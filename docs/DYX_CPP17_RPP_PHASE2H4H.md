# DYX Autonomous — Phase 2H4H: Native C++17 Offline HTTP Listener

**Prepared by: Satheesh**
**Base:** `Jetson_DDS_Upgrade` commit `795ce4f` (Phase 2H4G)
**Hardware:** Not required. **Target:** Windows x64 (Visual Studio 2022) and Linux C++17.

## Purpose

Convert the previously tested **HTTP/JSON contract** into a real but intentionally
restricted local TCP server. This lets the team use `curl` or an Android UI
simulator on the same development PC to inspect the API response format
before deploying Jetson hardware or integrating ROS 2.

## Implemented

- Native C++17 cross-platform HTTP/1.1 **socket listener**, bound specifically to **127.0.0.1**.
- The operating system chooses an ephemeral test port; the optional demo uses port **18765**.
- Accepts only **GET** requests; mission-changing methods are refused **before** reaching the command gateway.
- Routes read-only requests to the existing Phase 2H4G wire contract:
  - `GET /api/v1/mission/status`
  - `GET /api/v1/telemetry/live`
- Rejects unknown hosts, duplicate Host headers, request bodies, transfer encodings,
  malformed lines, oversized headers, and other HTTP methods.
- Bounded 4 KiB request headers, 1 s socket I/O timeout, one request per connection,
  `Connection: close` response, and worker shutdown/join.
- Uses `steady_clock` only for read-only freshness evaluation.
- Denies reads by default; an explicitly supplied **in-process synthetic debug reader**
  permits read-only test access. Network headers never grant authority.
- No user secrets, tokens, control leases, motor setpoints, spray, laser, DDS or MAVROS.

## Strict deployment limits

**NOT a production-grade HTTP server.** It has **no TLS, no real authentication,
no rate limiting, no Android Wi-Fi access, no concurrent mission-data locking,
no backpressure queues, no real PX4 DDS telemetry, and no network actuator commands.**
The debug principal is intended only for a local synthetic test process;
any other local program may reach its loopback port. Do not forward or proxy
port 18765 to the LAN or use a browser connecting through arbitrary hostnames.

The server cannot be used as a physical emergency stop. STOP on the real rover
requires an independent, physically verified PX4/motor safety path.

## Windows build and tests

From the repository root:

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected: **84/84 native tests**, up from 80; four new tests perform actual TCP I/O
with Winsock (`ws2_32`). The earlier 80 tests are retained.

## Optional localhost demonstration (Windows)

```powershell
.\build\dyx_cpp_only\Release\backend_loopback_readonly_demo.exe --offline-readonly
```

While it is running, open a second PowerShell window:

```powershell
curl.exe --noproxy "*" -i http://127.0.0.1:18765/api/v1/mission/status
curl.exe --noproxy "*" -i http://127.0.0.1:18765/api/v1/telemetry/live
curl.exe --noproxy "*" -i -X POST http://127.0.0.1:18765/api/v1/mission/start
```

Expected: HTTP **200**, HTTP **200** (unavailable sensor data), and HTTP **405**.
Press Enter in the server's original console to stop. The demo deliberately
contains **no loaded mission and no live RTK observations**.

## Deferred milestones

Production HTTPS ingress must enforce verified operator identity, scoped
leases, request limits, CORS/CSRF protections, server-side mission ownership,
and audited safe-stop integration. ROS 2/PX4 message compatibility, the Jetson
build, physical E-stop and actuator output isolation remain to be verified.
