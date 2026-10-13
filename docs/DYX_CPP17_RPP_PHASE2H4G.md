# DYX Autonomous — Phase 2H4G: Native C++17 HTTP/JSON Wire Contract

**Prepared by Satheesh**
**Target branch:** `Jetson_DDS_Upgrade`
**Foundation:** Phase 2H4F (`90b1b58`), 75 native tests

## Purpose

Bridge the Windows-verified native C++17 mission status, safe mission decisions and read-only PX4 telemetry data to an **HTTP/JSON wire contract**, without relying on Python, MAVROS or a Jetson. The result is a portable, testable serializer and strict request interpreter, **not** a listening HTTP server.

## Implemented

- `GET /api/v1/mission/status` — read-only mission progress/status JSON.
- `GET /api/v1/telemetry/live` — read-only JSON, with valid RTK status, satellites, battery, correction age, safety hold, and unavailable/stale readings as `null` / `UNAVAILABLE`.
- `POST /api/v1/mission/{start,pause,resume,next,skip,stop}` — strictly parsed JSON command envelope, forwarded only to the existing mission-command safety gateway.
- POST JSON is restricted to **exactly** `mission_run_id` (string) and `sequence` (unsigned integer). No client-supplied role, lease, session ID or clock fields are accepted. Duplicate fields, JSON coercion, overflowing sequence IDs, unknown fields or malformed JSON are rejected.
- `BackendVerifiedPrincipal`, operator session ID, control lease, host monotonic timestamp and verified RTK/safety evidence MUST be supplied by a trusted external ingress component. The source JSON never authorizes itself.
- JSON output properly escapes control characters and rejects invalid UTF-8. Sensor numbers are never emitted as NaN or Infinity; unavailable data uses `null`.
- Prepared HTTP/1.1 bytes set `Content-Length`, `Cache-Control: no-store`, `Content-Type: application/json`, `Connection: close` and `X-Content-Type-Options: nosniff`.
- Optional **single SSE event-frame encoding** uses an externally assigned event ID and the same authorized live-status JSON. It is not a connected SSE endpoint.
- Read/command failures return minimal `{ "ok": false, "code": ... }` JSON without mission-status leakage.
- Hard per-request and response size limits, safe method/path dispatch, and no packet-level side effects.

## Files

- `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_http_json.hpp`
- `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_http_wire.hpp`
- `src/dyx_rpp_cpp17_core/test/backend_http_fixture.hpp`
- Five `backend_http_*_native_smoke.cpp` tests and updated native `CMakeLists.txt`.

## Verification

Run the existing Visual Studio 2022 Release native C++17 test lane. Expected **80/80 tests**. This phase requires **no Jetson, ROS 2 middleware or physical rover**.

## Explicitly deferred / NOT implemented

1. **No TCP/HTTP server, TLS, Android network connection, WebSocket or operational SSE stream.** Another verified ingress/runtime component must supply these.
2. No credential checking, user authentication, cryptographic verification, network control-lease management or request rate-limiting. Bool fields in tests are placeholders for **externally trusted** verification, never proof.
3. No ROS 2 DDS subscriptions in this module, no PX4 setpoint publisher, and no physical STOP, spray, laser or motor command. Safe-stop responses indicate *intent only*.
4. Future integration needs authenticated TLS, origin policy, Android JSON contract tests, request timeouts/rate limits, mission source validation, persistent replay protection across process restarts, ROS 2 build and Jetson/hardware acceptance.
5. SSE events here are a **single JSON event snapshot**. No open stream, publish scheduler, retry semantics or authentication refresh is provided.

**Production rule:** Do not expose this stand-alone wire adapter to a network or energize any actuator. Complete all deferred acceptance criteria before deployment.
