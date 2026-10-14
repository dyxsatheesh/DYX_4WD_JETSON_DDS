# DYX Autonomous — Phase 2H4I: Offline Trusted Ingress and Control-Session Safety

**Prepared by Satheesh**
**Git base:** `0e91798` on `Jetson_DDS_Upgrade` (Phase 2H4H).
**Target:** C++17 Windows Visual Studio 2022 and GCC. **No Jetson required.**

## Objective

Provide a non-networked contract for a future independently authenticated backend ingress and protect mission decision requests against expired or transferred leases and repeated commands.

## Added in Phase 2H4I

- An initially denied, **one-owner control lease** with configurable 0.5–60 s expiry and explicit renewal/revocation.
- Host monotonic-time validation: backward or invalid time latches an integrity fault, revokes control, and requires explicit **authorized + independently disarmed** recovery.
- Generation/epoch fencing: commands tied to the prior lease generation are rejected after release, expiry or takeover; server-managed generation is **not** an authentication token.
- Per-control-lease monotonically increasing sequence guard, consumed before the downstream mission-safety decision (including rejected commands).
- Read-only queries require verified read scope but not a control lease.
- Authenticated STOP decision requests do **not** depend on an active lease or healthy host time. The underlying mission gateway independently checks the mission/session; this is **not** an emergency-stop output.
- A **transport-independent offline adapter** strips any forged client principal, validates exact route/method and strictly parses JSON before passing a verified principal to the existing Phase 2H4G HTTP/JSON *decision* layer.
- Four native regression tests: lease ownership/replay; expiry and clock reset; generation/identity isolation; HTTP wire permission boundary and STOP fallback.

## Critical production limitation

`TrustedIngressIdentity.authentication_verified`, role scopes, session ID and disarm assertions are **trusted input contract placeholders**, NOT evidence or cryptography. The testing harness supplies synthetic truth values. Do not populate them from request headers, JSON, Android fields or externally supplied booleans.

**No integration into `NativeLoopbackReadOnlyHttpServer`.** It remains bound to 127.0.0.1 and GET-only. This phase creates no TLS server, password verification, JWT/certificate validation, HTTPS listener, Android client link or outbound DDS transport. Only a future independently verified TLS/authentication adapter may use these types in an actual control path. A fully deployed system additionally needs session persistence, independent watchdog/physical E-stop, bounded concurrency, CORS/CSRF and threat-model review.

## Verification

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

**Expected 88/88 native tests.** No physical outputs, ROS2 or PX4 are used.
