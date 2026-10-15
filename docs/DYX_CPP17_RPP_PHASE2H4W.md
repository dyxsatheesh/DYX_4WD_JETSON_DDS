# DYX Autonomous — Phase 2H4W

## Offline restart replay fence and mock durable-store contract
**Prepared by Satheesh** · Jetson C++17 / XRCE-DDS migration

**GitHub baseline:** `dyxsatheesh/DYX_4WD_JETSON_DDS`, branch `Jetson_DDS_Upgrade`, commit `5a644c30e2b53c03797a19fb11d8fe10d2887c71` (Phase 2H4V).

**Scope:** Offline unit-test laboratory **only**. This patch does **not** add a real persistent database, disk journal, HTTPS, authentication, socket listener, real PX4/DDS safety reader or physical actuation. The memory store simulates a provisioned durable CAS interface; it does not survive an actual Jetson process or power restart. The tests recreate the ingress/security objects while retaining the same in-memory mock store, and retain the backend when testing mission recovery.

### Why add this layer?

The H4T/H4V nonce and session checks live in process memory. Reconstructing those objects resets that local state. A deployed rover must not interpret a crash or restart as permission to repeat a previously accepted command. H4W adds a dedicated **store boundary** between the independent H4V simulated session evidence source and the existing H4U framing/H4T ingress gate:

`Prepared Android request → H4V strict framing preflight → independent lab clock + session → H4W mock store read + reserve nonce (CAS) → mutation-only PX4 observation → H4T session/nonce/lease/TTL gate → mission draft backend`

Store records contain a pre-provisioned operator session ID, session epoch, most recently reserved ingress nonce, last observed monotonic receive time and a generation counter. The adapter fails closed if the record is missing, corrupt, unavailable, for a different session/epoch, subject to clock rollback, or if the comparison-and-swap fails. It never silently initializes the record from untrusted Android/HTTP data. A nonce is reserved **before** the backend can be invoked, even if the downstream request is later denied. This is intentional; the caller must perform authenticated GET reconciliation rather than assuming a rejected or lost POST is safe to retry.

### Four new Windows tests

| Test | Lab coverage |
|---|---|
| `backend_draft_restart_nominal_native_smoke` | GET→UPLOAD→GET→UPDATE→GET→LOAD→GET; 7 durable-mock reservations, READY not STARTED, no hardware output |
| `backend_draft_restart_replay_native_smoke` | Recreate ingress/security objects with retained mock store; previously used nonce, changed epoch and changed operator denied |
| `backend_draft_restart_store_fault_native_smoke` | Missing/unavailable/corrupt mock record, failed write/CAS conflict, nonce reuse, clock rollback and generation overflow deny before backend |
| `backend_draft_restart_recovery_native_smoke` | Simulated lost POST response, recreate ingress/security objects, stale replay denied, new-nonce duplicate rejected by retained backend, authoritative GET resync |

**Expected native suite:** 144 tests, up from 140. This is an expected count, **not** a claim that all tests have been run in Windows.

### Windows build + verification

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
if ($LASTEXITCODE -ne 0) { throw "CMake failed" }
cmake --build build/dyx_cpp_only --config Release
if ($LASTEXITCODE -ne 0) { throw "Build failed" }
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Native tests failed; do not commit" }
```

### Production release blockers

1. Implement and audit **actual** transactional, crash-consistent, durable journal persistence with atomic compare-and-swap, integrity checking, fsync/power-loss tests, controlled initial provisioning, and operator re-enrollment/rotation workflow. A mere in-memory or unflushed file implementation does not qualify.
2. Persist mission mutation identities/outcomes in the same recoverable transaction model. Persisting only nonces prevents stale replay but **does not** guarantee exactly-once mission execution, durable backend mission state or safe recovery of a newer-nonce retry after a crash.
3. Decide how the trusted clock and security enrollment are reestablished across OS reboot, as a process-monotonic tick count may reset; deny operations until trusted re-enrollment when continuity is unprovable.
4. Implement real authenticated HTTPS ingress, operator control leases, real PX4 disarm evidence over validated DDS identity, rate limiting, audit trails, emergency-stop/RC safety, and Jetson/Pixhawk physical end-to-end testing.
5. Validate concurrent request serialization, cross-process locking and storage fault-injection under Linux on Jetson; these offline source/fixture classes are **not thread-safe**.

**Safety:** Keep Android command transport and physical actuation **disabled** for this phase. The code adds only non-actuating offline test-lab interfaces.
