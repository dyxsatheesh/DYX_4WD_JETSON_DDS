# DYX Autonomous — H6E2A C++17 legacy read-only REST compatibility (first implementation)
Prepared for engineering review | 16 October 2026 | Git baseline `af6638e7f7603c0d73c46402bb076068ec7a9473`

## Source contracts checked
- `rover_backend/mission_routes.py` defines **GET `/api/mission/status`** and returns `{ "success": true, "mission": _mission_state(), "report": mission_report_store.status() }`. The mission state is **not** the `build_mission_status_payload()` projection in `system_routes.py`.
- `rover_backend/system_routes.py` defines **GET `/api/telemetry/latest`** and returns `build_telemetry_payload()`: nested backend, ros, vehicle, position, gps, estimator, rtk, battery, accuracy, safety, obstacle, mission plus flat compatibility fields.
- Both legacy endpoints require the existing `require_auth` dependency. A verified principal must come from an independently trusted authenticator; **client-supplied booleans are not authorization**.

## Implemented in this phase
New standalone ROS-independent C++17 header-only package `dyx_backend_cpp17_legacy_readonly`, class `NativeLegacyReadOnlyAdapter`:
1. Dispatches precisely `GET /api/mission/status` and `GET /api/telemetry/latest` **in process**, before any network listener exists. POST, other methods, unknown paths and request bodies do not mutate mission or motor state.
2. Rejects unverified principal (401) or read-scope failure (403). Never loads a privileged principal from client JSON; the trusted host ingress is **still missing**.
3. Accepts separately supplied `TrustedSnapshot` strings only when caller attests provenance; strictly parses bounded UTF-8 JSON, rejects duplicate keys, validates baseline object/key types, requires monotonic sequence/time and one epoch. The attestation bool is **not cryptographic proof** and MUST be bound to a real verified producer later.
4. Replies **byte-for-byte** with the whole supplied legacy-format JSON, rather than constructing a partial fake Android response. Defaults to HTTP 503 when no verified fresh snapshot; aged data is never shown as fresh. It does **not** validate every nested legacy field; source parity remains unproven.
5. Latches on invalid/untrusted/replayed/cross-epoch snapshots or clock regression, clearing cached data. Reset requires trusted operator approval, verified PX4 DISARMED and a *new* epoch. Reset is not exposed on HTTP.
6. Pure native C++17 tests for routing/auth/read-only, JSON abuse/freshness/epoch latch, and no actuation + legacy source route drift. Test JSON is **synthetic**; not an actual Android or backend capture.

## Not yet implemented (DO NOT CLAIM PRODUCTION COMPATIBILITY)
- NO HTTP listener, TLS, login/session backend, production bearer-token validation, Socket.IO, SSE, Android client, ROS 2 subscriber, DDS binding, PX4 vehicle connection, live telemetry adapter, NTRIP, spray, START/RESUME/NEXT/SKIP/STOP dispatch, emergency-stop hardware acknowledgement, or systemd cutover.
- No API parity for other 69 legacy REST routes and no actual complete live endpoint response captured from running Python production backend.
- No deployment on same LAN/port as the Python server. H6E2A does NOT replace any production service; it produces **zero** ROS command publications.
- Readiness for native rover control must be established by a later secure authenticated ingress, verified event/correction provenance, API contract golden traces, single motion owner, ROS2 Humble `colcon` and Jetson/PX4 field tests.
- This phase cannot claim that the Android app will already work with native backend.

## Tests
Windows **PowerShell**:
`powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\DYX_H6E2A_WINDOWS_OFFLINE_TEST.ps1`

Ubuntu **Bash**:
`bash scripts/DYX_H6E2A_UBUNTU_OFFLINE_TEST.sh`

Expected: three CTest tests pass in each environment.

## Follow-up order
H6E2B authenticated socket/service ingress and *actual* read-only live producer; H6E2C remaining legacy mission/control endpoints with safety; H6E3 authenticated Socket.IO/golden Android parity; H6F complete production cutover tests.
