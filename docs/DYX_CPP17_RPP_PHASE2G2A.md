# DYX Autonomous — Phase 2G2A: Native metric trajectory builder

**Baseline:** `Jetson_DDS_Upgrade` at `fa2d461`.
**Status:** non-actuating C++17 component; not an authorized motor command stream.

## Source reference

Ported the metric path generation and row-topology decisions from
`trajectory_generator/trajectory_generator_node.py` (`_generate_navigation_path`,
`_append_interpolated_segment`) and `trajectory_generator/topology.py`
(`compile_metric_topology`), using their rules as of the baseline commit.

- Navigation path starts at **P1**, not at the live vehicle position; the live
  C-to-P1 path remains an independent controller responsibility.
- Local coordinates are ENU (x=East, y=North), already in the PX4-aligned metric frame.
  This phase deliberately does not port geodetic-to-PX4-origin conversion.
- 0.05 m interpolation spacing by default; pass-through type 0, dummy-alignment
  type 1, marking type 2. Every uploaded marking coordinate is copied exactly
  to one and only one type-2 point, with consecutive marking indices.
- Extension default OFF. When enabled, a **strictly shorter than 3.0 m** transfer
  may gain a 4.0 m incoming-heading extension only if the transfer angle is
  45–135 degrees and the next row reverses direction by at least 135 degrees.
- Explicit operator row-end skip is supported. Two-point missions and the
  final marking pair **cannot** receive a row dummy; there is no following row.
- Configurable bounds reject nonfinite/duplicate points, out-of-range geometry,
  invalid settings and navigation point-count overflow before allocation.

## No reverse driving

The term *reversal angle* in the topological test describes the **direction
of the NEXT marking row**, not reverse-driving commands. The generated path
is geometry only. Motion execution must use forward translation and, when
needed, separately authorized stationary pivots. Dummy points must never
be sprayed or counted as completed marking points.

## Explicitly pending

DDS ROS2 publishing and runtime PX4 local-origin conversion; path signature,
atomic mission snapshot install, trajectory edit/reload protocol, full source
and clearance-warning report parity, motion feasibility of extension transitions,
mission-manager synchronization, hardware validation. No guarantee of
20–30 mm marking accuracy without live RTK and geometry verification.

## Windows tests

Run native C++17 CMake lane under `src/dyx_rpp_cpp17_core/test/native_cpp_only`.
Expected **20** tests: previous 17 plus three trajectory-builder tests.
Python, Jetson and ROS 2 are not required for this offline native test lane.
