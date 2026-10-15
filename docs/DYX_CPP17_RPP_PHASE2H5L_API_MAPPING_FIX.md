# DYX Autonomous — Phase 2H5L API Warning Fix

**Prepared by Satheesh**
**Repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS`
**Baseline:** `276fa870b0343b1b7d0b388288282854f6f39419` (`Jetson_DDS_Upgrade`)

## Problem

Phase 2H5L introduced `ClientCommandStatus::recovery_quarantined` in the core mission gateway, but `NativeBackendMissionApi::handle()` had no case for that status. GCC therefore produced `-Wswitch`, and an otherwise valid quarantined start request would receive the fallback `500 UNRECOGNIZED_GATEWAY_RESULT` rather than a clear locked response.

## Corrective change

- Map `recovery_quarantined` to HTTP `423` with `MISSION_RECOVERY_QUARANTINED`.
- Extend the existing `backend_api_commands_native_smoke` to test that start is rejected with 423, status remains read-only, and authenticated STOP remains available despite missing control lease or invalid clock.
- No new command authority, DDS output, movement, spraying, recovery unlock, or schema change.

## Test and deployment policy

Expected regression suites remain **153 Windows native tests** and **88 Ubuntu WSL2 SQLite tests**; a regression scenario is added to an existing native test, not as a new registered CTest test. Apply to a clean H5L checkout; run both suites and ensure the `-Wswitch` warning disappears before commit/push. The MSVC `D9025` `/DNDEBUG` versus `/UNDEBUG` messages reflect the existing CMake test assertion configuration; they are distinct from the GCC missing enum case and can be handled separately.

This host-test patch does not establish a production-grade HTTP ingress, DDS fail-safe, or PX4 safety case. Independent STOP/RC and hardware validation are still required.
