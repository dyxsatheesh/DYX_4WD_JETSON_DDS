# DYX Autonomous — Phase 2H5S: Trusted Recovery Ingress Contract (Offline)

**Prepared by Satheesh | 15 October 2026**

## Baseline and objective

- GitHub: `dyxsatheesh/DYX_4WD_JETSON_DDS`, branch `Jetson_DDS_Upgrade`
- **Required base commit:** `b494b0d29df2a97365dc08d4b06bd1d4bc765716` (Phase 2H5R).
- Add an **offline C++17 verification contract** above the H5R recovery-only service without creating a real HTTP listener, TLS session, privileged identity verifier, PX4/DDS publisher, motor output, or hardware STOP.

## Changes

The new `backend_recovery_trusted_ingress_lab.hpp` owns exactly one H5R service and pins the independently reviewed operator session at construction. It stays closed until `admit_recovery_only()` succeeds; after shutdown it never reopens. It only accepts a typed `RecoveryTrustedIngressEvidenceLab` **that a future privileged adapter must populate independently**, never from Android payloads or request headers. The request's existing principal, timestamp and PX4-disarm claim are completely ignored and overwritten with the independently supplied evidence. `trusted_px4_disarmed` is always forced to **false**.

Before dispatch, the H5S lab checks authenticated evidence, exact reviewed session, strictly increasing trusted request IDs, nondecreasing finite monotonic timestamps, request-path/body limits and valid framing. The request cursor is advanced **before** invoking the H5R dispatcher, including requests denied downstream. Replays are rejected as HTTP 409; missing trusted authentication as 401; session mismatch as 403; invalid time as 400; oversize as 413. Shutdown is one-way and concurrent dispatch is serialized. If the lower-layer reply reports hardware/network output or an unexpectedly large response, the gate closes permanently.

The quarantined H5R/H5P owner continues to block START/PAUSE/RESUME/NEXT/SKIP and upload/update/load; recovered SQLite draft previews are metadata-only, read-only and independently checked. Authenticated STOP is **software stop intent only**, never proof of motor braking or physical emergency stop.

## Test coverage

14 new Linux/WSL SQLite tests: `boot_closed`, `status`, `forged_client`, `unauth`, `identity`, `replay`, `clock`, `limits`, `commands`, `preview`, `tamper`, `shutdown`, `race`, `restart`.

- Windows C++ only suite: **160** regression tests expected (unchanged).
- Ubuntu WSL2 SQLite suite: **150** regression tests expected (136 existing + 14 new).
- Combined acceptance target: **310 / 310**.
- A local **synthetic offline development fixture** passed 14/14 new and 150/150 full Linux SQLite tests in Debug with assertions enabled. The official Windows and WSL2 checkout suites have **not** been executed by the assistant and must be run by the team before commit/push.

## Validation steps

First download `DYX_CPP17_RPP_PHASE2H5S.patch` to `%USERPROFILE%\Downloads`. From a clean exact H5R baseline, apply with `git apply --check --whitespace=error` and then `git apply`. Build Windows and run `ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure`; confirm 160. In Ubuntu WSL2, configure and build `src/dyx_rpp_cpp17_core/test/sqlite_lab` and run `ctest -R '^draft_sqlite_recovery_h5s_'` then full CTest; confirm 14 and 150 respectively. The provided scripts enforce these counts.

Do **not** reapply the patch after successful application. If `git status --porcelain` is nonempty before the first application, inspect changes rather than discarding them. Commit and push **only after both host tests succeed**.

## Safety and integration limitations

**No real privileged verifier exists in H5S.** Anyone with direct in-process access to its public laboratory evidence type can construct fields claiming authentication. A production adapter must prevent untrusted code from minting evidence and must independently validate identity, session, authority, request sequence, time and transport. H5S is not a network trust boundary or an authorization substitute. The synthetic tests cannot demonstrate live Jetson service readiness or actual security against a hostile process.

Also outstanding: Jetson systemd supervised executable, trusted TLS/network implementation, DDS / PX4 bridge integration, independent physical STOP and RC override, power safety, actuator inhibition, RTK safety, and hardware acceptance. Keep motors and sprayer physically inhibited for development integration.
