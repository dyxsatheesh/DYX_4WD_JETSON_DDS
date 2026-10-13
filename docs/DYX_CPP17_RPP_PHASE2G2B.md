# DYX Autonomous — Phase 2G2B: C++17 signed prepared-path snapshots

**Base:** `f24cfbd` on `Jetson_DDS_Upgrade`. **Status:** native Windows/GCC tests, not ROS 2/PX4 hardware validated.

- C++17 `mission_snapshot.hpp` calculates the *existing* mission `v1` canonical SHA-256 signature: `NAVIGATION` + binary network-order double pairs, `MARKINGS` + binary double pairs, `TYPES` + byte values, `INDICES` + network-order signed 32-bit values. No OpenSSL, Python, MAVROS or Jetson dependency.
- Path and metadata validation requires exactly matching navigation, marking and semantic metadata, correct P1..Pn identity/order, all finite coordinates, valid point types and sizes. Extension dummies remain non-marking points. Navigation/marking coordinates are not modified.
- `PendingPreparedSnapshot` retains each cross-topic source independently. `READY` can arrive first; no install before all five components are present with a matching canonical signature. Duplicate installation of the same signature is blocked until explicit `clear_all()` at an authorized mission boundary. Signature is required; legacy unsigned snapshots are intentionally rejected for the production design.
- A signature detects accidental or malicious payload mixing/tampering *assuming the authoritative signature is trustworthy*. It is **not** authentication or freshness: an attacker can rehash changed data, and DDS retained topics can be replayed. The future mission service must authenticate/authorize the upload, bind the payload to an approved mission/run ID, and gate readiness and PX4 control independently.
- No real mission install, HTTP server, DDS subscription, motor, spray command or autonomous movement is enabled. Tests use C++17 only. Do not remove Python source until all production capabilities have native replacements and hardware integration is verified.

## Windows validation

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected **23/23** tests, adding `mission_snapshot_signature_native_smoke`, `mission_snapshot_staging_native_smoke`, and `mission_snapshot_safety_native_smoke`.
