# DYX Autonomous — Phase 2G1A: C++17 multi-point mission sequencer

**User-verified starting baseline:** `e297b02`, branch `Jetson_DDS_Upgrade`.
**Scope:** deterministic non-actuating state machine; **no Python, reverse movement, ROS2 publishers, spray driver, or PX4 commands**.

## Original Python policy consulted

* `mission_manager/marking_arrival_policy.py`: only a stationary in-tolerance arrival starts marking verification; a stopped miss fails; AUTO proceeds and MANUAL waits for NEXT.
* `mission_manager/raw_gnss_spray_gate.py`: only a *post-stop stationary, validated raw RTK-FIXED* GNSS window against the uploaded surveyed coordinate may authorize a 30 mm pass; RPP's estimator-frame 20 mm certificate is not survey truth. One-shot fail verdicts do not get retried.
* `mission_manager/precision_terminal_policy.py`: terminal identity must bind mission-run, path, goal and goal-instance; an untrusted stale/foreign stop heartbeat cannot be reused.
* `mission_manager/mission_manager_node.py`: safety/mission commands and spray request/result handshake belong to the mission manager, not to path steering.

This patch implements only the *sequencing* contracts, not their upstream producers. In particular, the `survey_validated_raw_fixed` input MUST eventually be provided by a separately audited native C++ raw GNSS/window validator, and `motion_health_authorized` MUST be produced by the native safety arbiter. Never pass `true` unconditionally in deployed code.

## State flow

`READY -> APPROACHING -> VERIFY_HOLD -> SPRAY_ACK -> RESOLVED -> NEXT/FINISHED`

* Only a fresh, exact-goal RPP stop certificate <=20 mm, zero latch, stationary and measured speed <=0.03 m/s starts the stationary hold.
* A separate identity-matched, post-stop raw-GNSS verdict <=30 mm is required. The raw survey window must have been verified as RTK-FIXED by the future native GNSS gate. No raw truth -> fail; **never spray**. A raw-GNSS gate FAIL caused by unavailable measurements resolves the point as FAILED without requiring proof of RTK FIX. A PASS arriving after the survey deadline cannot rescue a previously unavailable measurement.
* Three-second stationary marking hold; spray request emitted exactly once after both requirements pass. Completion requires a later matching spray success acknowledgment; a timeout/failure marks FAILED, not COMPLETED.
* AUTO advances on the following control tick after a completed/failed/skipped point; MANUAL remains stopped at `await_next` until an explicit NEXT command. STOP never advances.
* E-stop (including while paused/awaiting NEXT) or invalid/replayed identity/timestamps enters latched FAULT; loss of movement health enters PAUSED. START/RESUME require the caller to assert current verified safety authorization. Explicit RESUME restarts current-point verification rather than reusing pre-pause evidence.
* No reverse speed, reverse arc, pivot motion or motor PWM output exists in this header. `allow_next_leg_preview` is false until a current verified-health `step()`; it is never motor authorization.

## Bounded verification and what remains

Native Windows C++17 tests validate deterministic mission transitions and adverse cases, independently of Python. They do **not** validate live DDS timestamps, physically stationary ground truth, raw GNSS quality, spray electrical behavior, mission upload JSON, ROS2 service interface, power/battery/RTK debounce, or PX4 Offboard control. Continue these as independent phases before Jetson/hardware approval.

To test on Windows, run the existing `native_cpp_only` CMake build and CTest. Expected **14 tests** (prior 12 + 2 sequencer tests). Commit only when all pass.
