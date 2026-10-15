# DYX Autonomous — Phase 2H4V

## Offline privileged-evidence source separation — prepared by Satheesh

**Baseline:** GitHub `dyxsatheesh/DYX_4WD_JETSON_DDS`, branch `Jetson_DDS_Upgrade`, commit `cd7ba40` (Phase 2H4U).

**Scope:** An **in-process C++17 laboratory** security-boundary wrapper around Phase 2H4U framing and Phase 2H4T ingress. The new API obtains session and PX4 observations through a privileged **mock source** instead of accepting these from HTTP, JSON or Android UI state. It does not start a network server, add cryptography, consult real PX4 hardware, connect to DDS, or issue commands to motors, spray, or laser.

### Architecture

`Android prepared GET/POST + canonical offline HTTP bytes → early framing/correlation rejection → privileged lab clock + session source → mutation-only PX4 evidence source → H4U strict frame/JSON binding → H4T session/lease/nonce/TTL gate → existing mission draft backend`.

The laboratory verifier **simulates** independent provenance; its booleans and timestamps are not a substitute for TLS peer certificate validation, external operator identity/lease services or live PX4 safety attestations. No HTTP header may declare itself authenticated. No Android UI field can make `px4_disarmed` true.

### Added tests

1. `backend_draft_boundary_nominal_native_smoke`: GET/UPLOAD/GET/UPDATE/GET/LOAD/GET; GET never fetches PX4 proof; LOAD remains READY, no actuator output.
2. `backend_draft_boundary_spoof_native_smoke`: untrusted headers, duplicate length, pipelining and wrong proposed request are blocked **before** calling privileged sources.
3. `backend_draft_boundary_authority_native_smoke`: missing time/session/PX4 source, false transport proof, missing lease, stale session, false/armed/stale PX4, changed session epoch, replay and latched monotonic clock regression.
4. `backend_draft_boundary_recovery_native_smoke`: lost response and uncertain POST status, nonce replay denial, server-side duplicate 409 and GET reconciliation without automatic LOAD.

### Production design gates (not implemented)

- A maintained HTTP server and parser behind properly configured TLS 1.3 with authenticated client identity (e.g., pinned CA or mutual TLS where appropriate), certificate revocation/rotation and a restricted Jetson listener.
- Separate, independently enforced operator authorization and short-lived, exclusive control leases; never trust Android session/lease booleans.
- Real trusted PX4 health/disarm/arming-mode evidence, sourced from verified DDS/PX4 publishers with stale-data, reboot and fault handling; independent motor inhibit/emergency-stop paths.
- Durable, per-operator replay protection and idempotency records across Jetson process restart; deterministic recovery after unknown POST outcomes.
- Rate limits, audit logs, denial-of-service controls, credentials/key storage, network-security review and physical emergency-stop/RC takeover tests.

### Windows validation

From a clean checkout at the stated baseline, apply `DYX_CPP17_RPP_PHASE2H4V.patch`, then run:

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

**Expected suite size:** 140 tests (136 baseline + 4 new). This is an expected count, **not** a claim that Windows MSVC or all 140 tests have been run here. Do not commit or push until actual Windows results pass. No production commands may be enabled on the basis of this lab alone.
