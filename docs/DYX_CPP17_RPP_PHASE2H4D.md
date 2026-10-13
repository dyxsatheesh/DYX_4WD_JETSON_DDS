# Phase 2H4D — Native C++17 backend mission API contract

**DYX Autonomous | Prepared by Satheesh | 13 October 2026**

**Baseline:** `Jetson_DDS_Upgrade`, Phase 2H4C GitHub commit `b40aa61`.

## Purpose and strict scope

This phase defines a transport-neutral C++17 contract between the future Android/HTTP ingress and the Phase 2H4C mission command safety gateway. It is **not** a network listener, an HTTP server, a JSON parser, a ROS 2 node, a source of authentication, or a physical stop/output interface. A caller-provided `BackendVerifiedPrincipal` is **not proof** of authorization: in production it MUST be built by a separately verified TLS/authentication/lease adapter. A caller-provided host timestamp must come from the server's monotonic clock, not the app.

## Fixed API routes and contract

| Method | Path | Verified privilege | Result |
|---|---|---|---|
| GET | `/api/v1/mission/status` | authenticated + read scope | Read-only mission counters, stage, display label, reason, advisory stop intent |
| POST | `/api/v1/mission/start` | authenticated + control lease | START if fresh arbiter tick and READY stage |
| POST | `/api/v1/mission/pause` | authenticated + control lease | PAUSE if stage allows it (no healthy GNSS required) |
| POST | `/api/v1/mission/resume` | authenticated + control lease | Explicit RESUME if safe |
| POST | `/api/v1/mission/next` | authenticated + control lease | NEXT only from manually resolved point |
| POST | `/api/v1/mission/skip` | authenticated + control lease | SKIP only during APPROACHING |
| POST | `/api/v1/mission/stop` | authenticated; control lease optional | Idempotent decision-side STOP even if telemetry is stale |

POST envelopes carry `mission_run_id`, operator-session identity sourced from verified context, strictly increasing `sequence`, and **server-side** host monotonic receive time. Wrong runs and replayed/out-of-order commands are rejected by the existing gateway. The API exposes stable 200/400/401/403/404/405/409/412 response codes. Exact matching disallows extra route suffixes or query-string rewriting. External HTTP adapters must handle methods, URLs, canonicalization, length limits and transport before calling this contract.

`GET /status` exposes `stage`, reason, total/completed/failed/skipped/pending, the active **display label** and advisory `safe_stop_intent`. It does **not** return raw GNSS certificates, actuator feedback, or an asserted motion-permission bit. A 200 API response acknowledges a C++ decision only; **it is not proof of PX4 STOP, laser OFF or a completed spray**.

## Windows acceptance

The Windows offline suite increases from 63 to **67 native C++17 tests**, with four new suites:

- `backend_api_routes_native_smoke`: read-only routing, exact method/path matching, status counters.
- `backend_api_authority_native_smoke`: read scope, auth, lease, run/session identity and clock handling.
- `backend_api_commands_native_smoke`: START/SKIP/NEXT/PAUSE/RESUME/STOP, safety-tick consumption and replay rejection.
- `backend_api_fault_native_smoke`: obstacle safety holds, PX4 epoch fault, STOP after telemetry loss.

Run existing Visual Studio 2022 native CMake and CTest commands. This package requires no Jetson, ROS 2, Python application or MAVROS.

## Pending production engineering

A trusted HTTPS or IPC ingress must authenticate each user, bind sessions and leases, issue monotonic sequences, rate-limit and validate JSON; the future mission-upload/update endpoint must use the signed mission snapshot installer and safe disarmed gate; a telemetry/status DTO must be wired to real ROS 2 DDS; and emergency motor/servo OFF commands need a hardware-verified independent safety path. All of these remain **pending**, not represented as complete by the native tests.
