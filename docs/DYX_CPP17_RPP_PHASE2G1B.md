# DYX Autonomous — Phase 2G1B: Native RAW RTK-FIXED Survey Accuracy

**Baseline:** Jetson `Jetson_DDS_Upgrade` at commit `75bcf2b` (Phase 2G1A).  **Scope:** C++17 only, ROS-free, Python-free, non-actuating.  This is source-level porting and offline test evidence, **not** a field accuracy certificate.

## Source contracts / engineering intent

Ported from the existing Python `mission_manager/survey_truth.py` and `raw_gnss_spray_gate.py`, with a small, deliberately stricter set of fail-closed checks. It is deliberately independent of PX4 EKF local coordinates:

1. The RPP terminal-stop controller may certify a 20 mm **estimator-frame** stop. That alone is NEVER permission to spray.
2. Raw Septentrio/PX4 GPS fixes must be RTK FIXED (`fix_type=6`) after the stop, relative to the mission-uploaded **surveyed latitude/longitude** at the rear-master antenna's marking reference.
3. A candidate sample window begins no earlier than stop time, lasts 0.5 s, uses >=3 individually timestamped fixes, requires all fixes to remain FIXED, and requires a stationary raw-GNSS cluster with maximum 20 mm scatter.
4. The legacy survey routine can trim braking samples from the *old* end for reporting, but the legacy spray gate DISQUALIFIES any trimmed window. The native implementation also disqualifies that window and reports `GNSS_WINDOW_NOT_STATIONARY`.
5. The final radial distance is the WGS84/Vincenty inverse between uploaded target and raw-fix centroid. North/east, signed along-track and cross-track errors are **diagnostic** only. No absolute EKF/local projection is used for raw radial accuracy.
6. The first available PASS or FAIL is latched by `OneShotRawSurveyGate`, bound to a terminal identity and a stop time; unavailable measurements wait until the 3 s timeout, then FAIL.
7. No approach-bearing measurement is necessary for radial accuracy; signed diagnostic errors prefer previous surveyed target, otherwise an explicit fallback NED bearing.

### Deliberate additional safety checks beyond the Python implementation

* Geographic coordinates must fall within valid latitude/longitude bounds; valid time-window fixes with invalid position/satellite/accuracy data make the entire window unavailable rather than silently disappearing.
* Repeated GNSS sensor timestamps are rejected instead of being counted as multiple independent samples.
* The configured radial tolerance may not exceed 30 mm, at least three raw fixes are always required, and `fix_type=6` cannot be relaxed.
* The decision carries its terminal identity and last accepted raw sample timestamp. Any cross-goal identity or malformed PASS is rejected by the `apply_raw_survey_evidence()` adapter.

These tighter gates can reject data that the older Python path accepted; review with recorded GNSS logs before deployment. No GNSS error estimate, 30 mm gate or terminal certificate guarantees 30 mm real-world painted-point accuracy without receiver/lever-arm/tilt and surveyed-field validation.

## New files

- `raw_survey_accuracy.hpp` — WGS84 geodesic, raw stationary window, RTK type-6 check, one-shot PASS/FAIL/WAIT gate.
- `raw_survey_mission_adapter.hpp` — maps identity-checked raw survey evidence to the existing `MissionSequencer` without granting terminal, motion or spray authority.
- `raw_survey_accuracy_native_smoke.cpp` — metric/geodesic verification and passing/failing fixed windows.
- `raw_survey_safety_native_smoke.cpp` — float, stale/pre-stop fixes, invalid samples, duplicate timestamps, drift and one-shot evidence.
- `raw_survey_mission_link_native_smoke.cpp` — identity and full mission intent chain: stopped → raw PASS → 3 s hold → spray request → verified spray ACK → resolved point.

## Offline verification (Windows 2022)

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected **17/17 native tests**, no Python, no Jetson.

## Deferred work before field operation

Implement certified PX4/SBF/raw GNSS sample ingestion with trusted receive timestamps, RTK FLOAT debounce, correction-age checking, time-aligned GNSS-to-marking antenna lever arms and PPS/source-clock validation. Integrate mission and spray handshakes through a safety-reviewed ROS 2 DDS adapter, verify C++17 Jetson compilation and custom PX4 behavior, and perform physical survey acceptance with the actual 735 mm dual-antenna baseline. **No motor, spray, reverse command, ROS publisher or MAVROS import is added in this phase.**
