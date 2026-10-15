# DYX Autonomous — Phase 2H5F

**Offline disarmed PREVIEW recovery preflight, without backend rehydration**

**Prepared by Satheesh**
**Project:** DYX Autonomous 4WD Jetson Orin Nano / Pixhawk 6X marking rover
**Git baseline:** `d010ecad3e005840179032075608e5c7e0e95770` on `dyxsatheesh/DYX_4WD_JETSON_DDS` / `Jetson_DDS_Upgrade`

## Scope

H5A–H5E implemented SQLite mutation journal, immutable committed snapshots, read-only reconciliation, snapshot audit, full-history screening and version-1 PREVIEW checkpoint. H5F adds a **strictly read-only offline preflight record** identifying the last committed nonce, mutation sequence, timestamp, revision, original operator session and a **proposed** newer epoch. It exists to design and test a future, carefully controlled PREVIEW-only restore protocol.

No network ingress, ROS2 XRCE-DDS, PX4 communications or physical actuation are involved. `NativeOfflineDraftRecoveryPreflightLab::inspect()` NEVER mutates SQLite, rewrites the replay fence, changes session epoch, restores `NativeBackendMissionUpload`, changes `NativeMissionCommandGateway`, starts or loads a mission, or unquarantines H5A.

For a successful offline rehearsal, the caller must assert all of: independently verified operator identity with read scope and fresh session evidence; explicit operator review; proposed new session epoch greater than persisted epoch and representable by SQLite int64; independently disarmed vehicle, stationary vehicle and absence of any active mission. **These booleans are test inputs, not real attestation. A production service MUST obtain and verify them independently from trustworthy live sources; never accept them from Android JSON.**

The preflight pins a SQLite read transaction, reads the replay fence, invokes H5E's existing full-history/checkpoint verification, then checks the final COMMITTED PREVIEW record's identity, epoch, nonce, run ID and mutation sequence against the captured fence. A concurrent mutation invalidates the comparison. The result includes read-only preview points plus replay metadata, and is explicitly not a restoration token. Contents are not cryptographically authenticated; signatures only check consistency.

All action flags are permanently false: `can_rehydrate_backend`, `can_load`, `can_start_or_resume`, `hardware_command_emitted`.

## Files

- `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_draft_recovery_preflight_lab.hpp` — new read-only preflight API.
- `src/dyx_rpp_cpp17_core/test/draft_sqlite_recovery_preflight_native_smoke.cpp` — seven offline smoke scenarios.
- `src/dyx_rpp_cpp17_core/test/sqlite_lab/CMakeLists.txt` — register seven additional SQLite tests.
- `docs/DYX_CPP17_RPP_PHASE2H5F.md` — this guide.

## Tests

| Scenario | Expectation |
|---|---|
| nominal | Valid two-revision PREVIEW produces read-only original nonce/sequence/time + proposed epoch |
| reopen | Fresh backend remains empty; SQLite record can still be inspected |
| authorization | Reject invalid identity, missing operator review, stale/invalid epoch, missing safety evidence |
| stale | A journal UPDATE invalidates previously sealed checkpoint |
| pending | An ambiguous PENDING mutation prevents preflight |
| ready | A LOAD-to-READY event prevents PREVIEW-only preflight |
| no_checkpoint | Without H5E seal, no preflight candidate exists |

Windows PowerShell, from repository root:

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected **144/144**.

Ubuntu WSL2:

```bash
cd '/mnt/d/4WD_ROVER WORKSPACE FOR GITHUB/DYX_4WD_JETSON_DDS'
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

Expected **42/42** (35 existing + 7 new). Test on your complete checkout before committing. Any local synthetic/stub regression is not proof of full-repository or target-device acceptance.

## Production blockers

Still not implemented: tamper-resistant storage/publisher authentication, atomic session-epoch rotation, synchronized durable backend journal transactions and final replay-fence ownership, restoring backend metadata without reopening old nonces, live operator control lease, trusted independent PX4 safety telemetry, and target hardware power-loss tests. A future phase must ensure STOP/E-stop/RC takeover remain fully independent of recovery and never automatically permit physical LOAD/START/RESUME.
