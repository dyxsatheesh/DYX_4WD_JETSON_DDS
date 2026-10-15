# DYX Autonomous — Phase 2H4U

## Strict offline HTTP/1.1 framing + draft correlation LAB — prepared by Satheesh

**Baseline:** Phase 2H4T, branch `Jetson_DDS_Upgrade`, commit `8189cb4`.

**Scope:** Unit-tested **in-process lab only**, **not a production HTTP endpoint**. The listener remains read-only; the code opens **no** sockets and cannot command PX4, DDS, motors, spray or laser. All credential/lease/PX4 evidence is supplied by simulated, trusted in-process fixtures rather than independently authenticated hardware or TLS.

### Why this phase matters

Phases 2H4P–2H4T established strict JSON mission semantics, Android transaction correlation and a simulated operator/PX4 authority gate. Phase 2H4U addresses an additional boundary **before** those layers: ambiguity in HTTP request framing can cause two components to disagree about what was received. It adds a canonical laboratory-only HTTP/1.1 envelope check and binds the received method, route, body and mission sequence to the Android proposal **before** handing control to the 2H4T ingress lab.

| Rule | Offline behavior |
|---|---|
| Framing | Require canonical HTTP/1.1 request line, `\r\n` delimiters, exact length, zero trailing bytes |
| Headers | Only one `Host: offline.invalid`, one `Content-Length`, and `Content-Type: application/json` for POST |
| Smuggling defense | Reject duplicate/leading-zero/overflow length, `Transfer-Encoding`, folded or unknown headers, pipelining |
| Trust separation | Reject client-supplied `Authorization`, `X-Verified-*` or other trust-claim headers rather than interpreting them |
| Android correlation | Require canonical method/path/body identical to the prepared request; independently parse JSON and compare `sequence` |
| GET /draft | Require empty body and `Content-Length: 0`, matching GET route and preexisting Android resync request |
| Authority | All accepted frames still pass H4T session/nonce/lease/independent PX4 DISARMED *simulated* evidence gate |
| No side effects | Reject frames without invoking backend; accepted LOAD leaves the mission READY, never STARTED |

**Important boundary:** Exact case-sensitive header names, an `offline.invalid` host, and rejection of all unrecognized headers are intentionally restrictive for the lab. This is **not a drop-in HTTP/1.1 server/parser**. A future real server must supply its own hardened, maintained HTTP parser and independently verified mutual TLS/authorization/session replay controls, restrict listener scope, and handle certificate/clock/PX4 failures with fail-closed policy. Do not create authenticated context from HTTP fields, caller booleans, Android UI state or the example hostname.

### Native test additions

1. `backend_draft_framing_nominal_native_smoke`: GET → UPLOAD → GET → UPDATE → GET → LOAD → GET; READY/no actuator output.
2. `backend_draft_framing_smuggling_native_smoke`: duplicate/ambiguous length, unknown/trust/spoofed headers, malformed line endings, extra bytes and pipelining are rejected *before* ingress.
3. `backend_draft_framing_binding_native_smoke`: mismatched method/path/request sequence/body and malformed JSON are rejected before mission mutation.
4. `backend_draft_framing_recovery_native_smoke`: lost POST reply, replay denial, no automatic retransmit and authoritative GET recovery.

To build and test on Windows from repository root:

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

**Expected number of native tests:** 136 (132 existing + four new). All tests still require Windows MSVC verification before accepting the phase.

**Still pending for production:** audited real HTTP/HTTPS ingress, cryptographic client authentication, least-privilege operator sessions, key rotation, rate limits, security logging, application-layer replay protection backed by durable persistence, live Jetson ↔ Pixhawk 6X XRCE-DDS testing, PX4 arming/mode/failsafe validation, and physical safety interlock tests.
