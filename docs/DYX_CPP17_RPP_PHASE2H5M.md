# Phase 2H5M — HTTP and Android recovery quarantine status

**Prepared by Satheesh**
**Company:** DYX Autonomous
**Repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS`
**Branch:** `Jetson_DDS_Upgrade`
**Required parent commit:** `e99de1daedd1a2cffd77840c21cb561b90af47f1`
**Scope:** Offline C++17 integration; no physical outputs or production authenticated ingress.

## Scope and changes

1. The transport-neutral mission status response includes `recovery_quarantined` as a read-only advisory (never a permit to drive).
2. `/api/v1/mission/status` JSON includes `mission.recovery_quarantined` to help Android disable commands.
3. The HTTP/1.1 serializer explicitly supports `423 Locked` and preserves JSON error codes.
4. The offline Android controls model latches an observed quarantined status or matching `423 MISSION_RECOVERY_QUARANTINED` command response. START/PAUSE/RESUME/NEXT/SKIP are not proposed after latching. Authenticated STOP proposal remains available.
5. Adds seven CTest cases for status, commands, STOP, upload, Android status, Android response, and authentication rejection.

## Security and limitations

- UI-facing flags are **advisory**, never trusted authorization. The backend H5L gateway retains authoritative one-way quarantine for its own instance.
- Neither the offline Android model nor the HTTP wire performs network transport, real TLS, credential verification, DDS publication, PX4 disarm attestation, motor control, spraying, or laser actuation.
- The existing loopback demonstration is read-only; this patch does **not** transform it into a production command server.
- STOP is an accepted software intent, **not** proof that Pixhawk or motors stopped. Hardware emergency stop and RC takeover must be independently validated.
- The ordinary draft GET remains blocked under gateway quarantine; only the separately verified H5K read-only quarantined PREVIEW review is intended for a recovered draft.
- A newly constructed gateway has no knowledge of prior durable quarantine until an authenticated startup guard loads and latches it. Production startup wiring remains unfinished.

## Acceptance

| Environment | Baseline | Added | Target |
| --- | ---: | ---: | ---: |
| Windows Visual Studio C++17 native tests | 153 | 7 | **160** |
| WSL2 Ubuntu SQLite tests | 88 | 0 | **88** |
| Combined | 241 | 7 | **248** |

Run full native and SQLite suites and commit only after both pass. If a compiler warning/error or a test fails, **do not commit or push**. Then audit actual process ownership and production HTTP/Android ingress in the next phase.
