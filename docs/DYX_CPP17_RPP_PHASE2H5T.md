# DYX Autonomous — Phase 2H5T: Offline Recovery Host Supervisor

**Prepared by Satheesh | 15 October 2026**

## Required baseline

- Repository: `dyxsatheesh/DYX_4WD_JETSON_DDS`
- Branch: `Jetson_DDS_Upgrade` (Windows checkout branch may be `JETSON_DDS_Upgrade`)
- **Exact prerequisite commit:** `ac8e4f7967b96bab8fc3f061ac85440447455075` (Phase 2H5S).
- This patch must be applied **once** to a clean checkout at that commit.

## Scope and objective

Phase 2H5T adds `backend_recovery_host_supervisor_lab.hpp` as an **offline, single-process supervisory wrapper** above the H5S trusted-ingress contract. It provides explicit simulated host startup admission, a manually observed monotonic host heartbeat/watchdog deadline, request receive-time consistency checks, one-way shutdown, and serialized dispatch. It never opens a socket, loads a systemd unit, emits ROS 2/PX4/DDS messages, or touches motor/spray outputs.

Architecture (simulation only):

`Future trusted host adapter (NOT implemented)` → `H5T host supervisor` → `H5S ingress` → `H5R service lifecycle` → `H5Q/H5P quarantined owner` → `read-only SQLite review`.

## Behavioral contract

1. **Starts closed**; all requests are blocked until a one-time recovery-only admission is approved by provided host evidence and a validated offline configuration. Any failed admission permanently closes this instance.
2. Configuration checks require a finite heartbeat gap of 0.1–60 seconds and test-only declarations that no listener, DDS publisher, or actuator is attached. **These booleans are not hardware or process attestation.**
3. The independently supplied reviewed operator session must match the session pinned at construction. The H5S layer independently binds principal, monotonic receive-time evidence and strictly increasing request IDs.
4. A future privileged host scheduling loop must call `observe_heartbeat(trusted_host_now_sec)`. **There is no automatic watchdog thread.** A missed deadline or backward/nonfinite host clock retires this simulated service. A late heartbeat cannot reopen it.
5. Before forwarding requests, receipt time must be finite, no earlier than the simulated host boot time, no later than host current time, and no older than the maximum gap. An impossible trusted timestamp fails closed.
6. All H5S responses are inspected for forbidden network/hardware output flags or oversized bodies; any violation closes the host. START/PAUSE/RESUME/NEXT/SKIP and upload/update/load remain locked while recovery quarantine is active. Valid recovered draft PREVIEW remains authorized, metadata-only and read-only.
7. Authenticated **STOP is software intent only**, **not physical braking**, motor disarm, RC override or an emergency-stop signal. The independent physical emergency-stop path must function regardless of this service and whether its watchdog is expired.
8. `shutdown()` is one-way and serialized with requests. A fresh instance must independently re-review its state. Request replay IDs are **only process-local**; durable cross-process replay protection remains outstanding.

## Files in patch

- `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_recovery_host_supervisor_lab.hpp` — new header-only laboratory supervisory contract.
- `src/dyx_rpp_cpp17_core/test/draft_sqlite_recovery_http_owner_native_smoke.cpp` — 15 additional H5T regression modes.
- `src/dyx_rpp_cpp17_core/test/sqlite_lab/CMakeLists.txt` — 15 new CTest registrations; no replacement of existing cases.
- `docs/DYX_CPP17_RPP_PHASE2H5T.md` — this team handoff.

## Validation criteria

| Host | Test suite | Target |
|---|---|---:|
| Windows / Visual Studio 2022 / C++17 | Existing native tests | 160 / 160 |
| Ubuntu WSL2 / SQLite | New H5T cases | 15 / 15 |
| Ubuntu WSL2 / SQLite | All SQLite tests | 165 / 165 |
| Combined, non-overlapping host suites | Windows + full SQLite | **325 / 325** |

New test modes: `boot_closed`, `bad_config`, `bad_start`, `admission`, `commands`, `preview`, `identity`, `watchdog`, `heartbeat`, `clock`, `receipt_time`, `shutdown`, `restart`, `parallel`, `shutdown_race`.

**Assistant-side evidence:** In an isolated local Linux C++17/SQLite source fixture reconstructed from the preceding patch artifacts, the 15 new H5T CTest cases passed (15/15), and the full local SQLite suite passed (165/165). The full suite was compiled with `-UNDEBUG` so assertions remain active even for Release builds. The actual Windows native run and the user's Windows/WSL2 checkout run **have not yet been performed for H5T**.

## Team application flow

1. Run the included Windows application and native test script from your verified H5S baseline.
2. Run the included WSL2 script and verify both 15 new cases and 165 full SQLite tests.
3. Only after both environments pass, run the included Windows commit/push script.
4. Do not reapply the patch if Git indicates existing H5T changes. Inspect the worktree instead of resetting or discarding local work.

## Not yet production-ready

No real systemd process, Linux scheduler integration, actual watchdog firing, trusted identity provider, client TLS, trusted clock, independent PX4 disarm evidence, multi-process exclusivity, persistent replay fence, ROS 2 XRCE-DDS output, physical motor inhibitor, spray inhibitor, physical STOP or field acceptance is implemented or verified by H5T. Treat the H5T model as **offline design/regression coverage**, not Jetson/PX4 rover deployment approval. Do not enable rover motion or spray on the strength of these tests.

Suggested next engineering milestone: a **disarmed, output-inhibited Jetson Linux executable/service harness** with real process lifecycle and explicit, externally verified dependencies, initially with all DDS/motor/spray outputs disabled. Hardware safety and physical STOP testing must precede any live motion.
