# DYX 4WD Jetson — C++17 RPP Phase 2B (Path Geometry Index)

Prepared for DYX Autonomous engineering. **Target:** `dyxsatheesh/DYX_4WD_JETSON_DDS`, GitHub branch `Jetson_DDS_Upgrade`, following commit `acb51f8` (Phase 2A).

**Status:** non-actuating library; leave Python mission manager/RPP and existing launch configuration unchanged. Not approved to drive a rover.

## Scope and parity contract

This patch ports the following behaviors from the live Python reference `src/rpp_controller/rpp_controller/path_geometry.py` into `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/path_index.hpp`:

- Build geometry from all authoritative `/nav_path` raw points **without discarding or renumbering raw indices**. Zero-length consecutive samples remain raw points but do not create geometry segments.
- Validate pass-through (0), dummy alignment (1) and marking (2) metadata, including consecutive marking indices (`P0001`, `P0002`...).
- Build cumulative distance, segment headings, semantic anchors and thresholded corners (45° default), tracking leg IDs.
- Create an active semantic span bounded by the current marking/dummy point, preventing projection onto unrelated mission legs.
- Closest-point projection with *signed* cross-track (ENU left positive), local segment hint windows, reference progress jump limits and optional full-span reacquisition.
- Exact cumulative-distance interpolation, bounded lookahead targets and strictly future corners.
- Stateful progress tracker which does not retract progress after small GNSS position noise; explicit `reset()` for intentional reacquisition.

**Important:** this is not a complete RPP implementation. Full `compute_precision_guidance`, velocity regulation, stationary pivot, terminal 20 mm radial stop, spraying and mission-manager authority remain in Python until later phases. No DDS topics, PX4 firmware files or existing launch files are modified.

## Patch changes

- Adds `path_index.hpp` under the existing `dyx_rpp_cpp17_core` include tree.
- Adds `test/geometry_index_probe.cpp` and `test/geometry_index_parity.py`: compares the new C++17 index directly to the **existing Python reference** in the cloned repo; no precomputed answers.
- Adds `test/path_index_native_test.cpp`: independent C++17 smoke checks for raw indices, duplicates, semantic labels, corner selection, lookahead, cross-track sign and monotonic progress.
- Updates `test/native/CMakeLists.txt` to register two new CTest tests alongside the previous Phase 2A parity test.

## Windows 11 VS Code test steps (same repository; no re-cloning)

Run from `D:\4WD_ROVER WORKSPACE FOR GITHUB\DYX_4WD_JETSON_DDS`:

```powershell
# Confirm that Phase 2A was committed and is synced.
git status --short
git log -1 --oneline

# Check, then apply the new downloaded patch once only.
$Patch = "$env:USERPROFILE\Downloads\DYX_CPP17_RPP_PHASE2B.patch"
Test-Path $Patch
git apply --check $Patch
git apply $Patch

# Existing Visual Studio 2022 generator and build directory are reusable.
cmake -S src/dyx_rpp_cpp17_core/test/native -B build/dyx_rpp_native -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_rpp_native --config Release
ctest --test-dir build/dyx_rpp_native -C Release --output-on-failure
```

Expected CTest test names (3 total): `python_cpp_parity` (Phase 2A), `python_cpp_path_index_parity` (new), and `path_index_native_smoke` (new).

**If a parity comparison fails, do not push the patch**: capture the failing scenario, inspect the numerical or semantic discrepancy, and fix the C++ port while leaving original Python behavior intact. Numerical parity is a prerequisite, not evidence of field accuracy.

After the three tests pass:

```powershell
git add src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/path_index.hpp
git add src/dyx_rpp_cpp17_core/test/native/CMakeLists.txt
git add src/dyx_rpp_cpp17_core/test/geometry_index_probe.cpp
git add src/dyx_rpp_cpp17_core/test/geometry_index_parity.py
git add src/dyx_rpp_cpp17_core/test/path_index_native_test.cpp
git add docs/DYX_CPP17_RPP_PHASE2B.md
git diff --cached --check
git commit -m "feat(jetson): port semantic RPP path index to C++17"
git push origin HEAD:refs/heads/Jetson_DDS_Upgrade
```

## Local engineering verification before distribution

A standalone Linux `g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror` native compile and C++ invariant smoke test passed. **Full Python parity has not run in the authoring environment**, which lacks the complete cloned Python RPP reference. It must run in your Windows checkout using CTest before committing.

## Follow-up

**Phase 2C:** port full precision guidance and steering speed regulation from `guidance.py`, `tracking_control.py` and `speed_regulator.py`, exercising replay parity on straight segments and representative corners before any ROS/PX4 controller integration.
