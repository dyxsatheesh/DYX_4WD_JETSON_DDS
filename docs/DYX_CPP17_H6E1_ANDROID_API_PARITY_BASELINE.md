# DYX Autonomous — H6E1 Android/backend API compatibility baseline
Prepared by Satheesh | 16 October 2026 | source baseline `af220e92cdd2fa3d65e22de07b04b5f29da0fc7d`

## Purpose
Establish an independently executable C++17 source-code inventory **before** migrating the Python FastAPI + Socket.IO backend. The 4WD rover must preserve the current Android application interface or supply a separately validated compatibility migration. No attempt is made to switch production launch, remove Python, enable DDS motion, or expose an unauthenticated network service.

## Verified static gaps
- Eight DYX Python FastAPI router modules define **71 decorator-based REST method/path entries** (auth 3, system 8, mission 25, RTK 13, RTK policy 2, spray 10, obstacles 2, voice/TTS 8).
- Existing `dyx_rpp_cpp17_core` backend HTTP interface declares **eight** `/api/v1/*` model routes (one live GET, one mission-status GET, six mission-command POSTs). Their **method + path exact overlaps with the 71 old routes are zero**. Functional equivalence is NOT inferred.
- Python `realtime.py` declares Socket.IO. Native HTTP JSON/SSE models are not a production replacement for authenticated Socket.IO events and session lifecycle.
- This inventory excludes dynamically registered routes, FastAPI metadata routes, standalone endpoints, alternative aliases, response-body schema, authentication policy, port/TLS, mission journal, or Android-client behavior. The count is not a proof of the full route surface.

## What this patch adds
- `tools/dyx_h6e_api_parity`: standalone C++17 inventory scanner, deterministic route parser, fail-closed report. It reads application source only (does not run Python), outputs `dyx_h6e1_api_parity.json`, and emits `production_ready=false` unconditionally.
- Cross-platform regression tests pinned to this baseline. Route count drift fails the test on purpose and requires explicit review. No network sockets, ROS, PX4 command topics, motor or sprayer outputs.
- Windows PowerShell/Ubuntu Bash offline runners and LF Git attribute.

## H6E implementation sequencing (not completed here)
1. Export captured Python endpoint schemas and real Android request-response golden fixtures without credentials.
2. Implement a native C++17 authenticated service on the intended network with reliable bounded HTTP transport; match exact legacy routes, error codes and JSON.
3. Implement Socket.IO-compatible real-time protocol or intentionally migrate and verify the Android client, including reconnect/status correctness.
4. Bind status from verified ROS 2/DDS telemetry without promoting untrusted diagnostics to motion authority; preserve mission controls, safe-stop, NTRIP, spray and recovery policies.
5. Run side-by-side Android integration tests, ROS 2 Humble `colcon`, Jetson/PX4 bench tests; only then cut over systemd and retire Python.

## Offline tests
- Windows: `powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/DYX_H6E1_WINDOWS_OFFLINE_TEST.ps1`
- Ubuntu WSL2: `bash scripts/DYX_H6E1_UBUNTU_OFFLINE_TEST.sh`

## Release protection
Running `dyx_h6e_api_audit --root <repo> --enforce` always returns exit code 2, deliberately preventing a false release decision based on static source inventory. This phase is H6E **planning/evidence** only; it is **not** completion of H6E production backend integration.
