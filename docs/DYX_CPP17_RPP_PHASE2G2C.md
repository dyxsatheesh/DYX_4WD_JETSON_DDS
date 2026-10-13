# DYX Autonomous | Phase 2G2C: Native prepared-mission installation

**Base:** Jetson_DDS_Upgrade commit `05655c6` | **Scope:** ROS-free, non-actuating C++17 integration.

`mission_runtime_install.hpp` connects an SHA-256-validated `PendingPreparedSnapshot`
to `PathGeometryIndex` and `MissionSequencer` in one staged runtime object. It
builds mark-goal identities with the existing Python contract:
`RUN:<run>|PATH:<sha256>|RAW:<path_idx>|GOAL:P0001|INSTANCE:<sha256>`.
The instance hash uses the zero-based *semantic* goal sequence; alignment
(dummies) count as semantic goals even though they are never spray targets.

## Safety boundary

- Installation requires two separately supplied booleans: authenticated upload
  authorization and confirmed PX4 disarmed state. **The C++ core cannot prove
  these external claims; ROS 2/backend integration must establish them.**
- No auto-start, no arming, no DDS publishing, no spray command, no reverse
  driving. An installed mission begins in `READY` state only.
- Installation remains blocked when another runtime is active; replacement
  requires an authorized state-only STOP and a confirmed-disarmed retirement.
- Mission run identifiers cannot be repeated within the same installer object.
  Reboot-resistant replay protection requires persistent session tracking.
- SHA-256 proves consistency, **not** who uploaded the mission. Authenticity,
  transport replay defense, and upload authorization remain to be implemented.
- Staging `try_install()` consumes a signed snapshot. If downstream binding
  fails, the staging sources require an explicit safe reset before retry.
- `stop_sequence()` only changes the C++ FSM; it **does not stop PX4 motors**.
  Never use this as a physical emergency-stop implementation.

## Windows verification

Build the Python-free lane using `src/dyx_rpp_cpp17_core/test/native_cpp_only`
and run CTest. Expected: 25/25 native tests (23 existing + two installation tests).
The full ROS 2 build, real authorization, PX4 disarm feedback, real-time timing,
and Jetson deployment remain unverified until hardware is available.
