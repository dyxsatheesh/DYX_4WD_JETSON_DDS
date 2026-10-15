# DYX Autonomous — Phase 2H4T

## Offline authenticated-ingress integration LAB — prepared by Satheesh

**Status:** native C++17 lab test only. Baseline: Phase 2H4S, branch `Jetson_DDS_Upgrade`, commit `b31ac30`. **Not production networking.**

Phase 2H4T joins the Phase 2H4S Android draft transaction coordinator to the Phase 2H4P strict server-side mission draft HTTP/JSON wire, through an explicitly bounded, **in-process** test boundary. Unlike previous direct fixture calls, the lab maintains a simulated verified operator session, security epoch and strictly increasing request nonce, gates read vs control permission, requires fresh independent PX4 DISARMED evidence for POST, and denies time rollback. The server still checks mission sequence, expected revision, owner, geometry and ready-versus-started state.

| Contract | Lab behavior |
|---|---|
| GET /draft | Requires authenticated, read-scoped, fresh verified session; does not require PX4 disarm |
| POST upload/update/load | Requires authenticated owner/control lease plus fresh, independently attested DISARMED |
| Security epoch / session | First valid exchange binds epoch and operator; switches are rejected for lifetime of lab object |
| Replay | Transport nonce must strictly increase; backend independently rejects repeat mutation sequence |
| Time rollback | Latches fail-closed; recreate lab only under independent operator-supervised verification |
| Missing POST response | Android transaction remains uncertain until authoritative GET; never auto-retry POST |
| LOAD | Backend accepts installed path as READY; not mission START and not physical actuation |

### Fundamental security limitation

`DraftIngressLabEvidence` is a *fixture container*, not a credential or cryptographic attestation. A `true` Boolean cannot establish identity or PX4 safety. Only an external trusted adapter may create these values in real deployment, from independently verified TLS/authentication, lease authority, monotonic host clock and fresh PX4 state. This patch implements **none** of those external attestations. Never expose `NativeOfflineDraftIngressLab` on a network, or treat `trusted_reply_provenance` as proof of real-world transport authenticity.

Neither the Phase 2H4T lab nor the existing loopback listener accepts external mission POSTs; no DDS, rover motors, spray, laser or other physical commands are emitted. The navigation gateway's LOAD still installs READY-only mission state, subject to backend validation.

### Tests

- `backend_draft_ingress_lab_nominal_native_smoke`: GET, UPLOAD, GET, UPDATE, GET, LOAD, GET; ends READY and no actuator output.
- `backend_draft_ingress_lab_security_native_smoke`: missing auth/read lease, invalid or stale PX4 disarm, wrong session and stale auth are denied before backend.
- `backend_draft_ingress_lab_replay_native_smoke`: transport nonce replay, wrong epoch/session, malformed payload, and monotonic clock rollback.
- `backend_draft_ingress_lab_recovery_native_smoke`: lost response, no automatic retry, backend mutation replay rejection, and authoritative GET recovery.

Native-only build/test, from the repository root:

```sh
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only
cmake --build build/dyx_cpp_only
ctest --test-dir build/dyx_cpp_only --output-on-failure
```

**Linux validation:** 132/132 tests passed using previously compiled unchanged Phase 2H4S targets plus newly compiled Phase 2H4T tests.

**Not yet verified:** Windows MSVC run, Jetson, ROS 2 / DDS bridge, authenticated HTTPS, Kotlin Android integration, PX4 disarm source, physically tested interlocks.
