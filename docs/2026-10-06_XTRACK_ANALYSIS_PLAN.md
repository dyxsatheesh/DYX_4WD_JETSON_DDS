# Straight-line cross-track: analysis plan for the next session (2026-10-06)

Goal: reduce the cross-track error on straight in-row legs. The operator sees
-4..+4 cm, often 5 cm, which is at the edge of the 30 mm raw-GNSS spray gate.
Analyse today's other bags first, then propose a fix with that data. **Do not
change code before the data step confirms the pattern.**

## What one bag already showed

Bag: `bags_jet/Oct_06/mission.csv_20261006_142238` (44 points, 4 rows of 11,
5 m spacing, 1.0 m/s cruise, extension 1 m). 36 straight in-row legs.

| Leg phase | p50 abs xt | p95 | max | > 30 mm |
|---|---|---|---|---|
| start (first 1 m) | 7.8 mm | 26 mm | 43 mm | 2.7 % |
| cruise | 9.2 mm | 27 mm | 46 mm | 3.6 % |
| decel (1.0 -> 0.3 m to go) | 12.4 mm | 33 mm | 78 mm | 7.0 % |
| terminal (last 0.3 m) | 8.4 mm | 40 mm | 80 mm | 10.0 % |

- Cruise tracking is good. The 4-8 cm errors are made in the last ~0.8 m.
- At P0003 and P0028 (failed, 37-40 mm) the COMMANDED yaw sent to PX4 turns
  3.5-5 deg away from the line between 0.8 and 0.3 m to go, then stays fixed.
  PX4 follows it. The nozzle is ~0.5 m behind the turning point, so the turn
  swings it ~40 mm sideways. Commanded yaw turns > 2 deg off the line in the
  last 1 m on 24 of 36 legs.
- Likely cause (code read, not yet tested): inside
  `radial_stop_terminal_guidance_distance_m` = 0.75 m the radial20 branch
  steers by `atan2(goal - nozzle)` (`rpp_controller_node.py` ~11391), i.e. it
  points the NOZZLE at the goal with a lookahead that shrinks to zero. Outside
  0.75 m `line_guidance()` steers the turning point
  (`steering_control_point_ahead_m` 0.50). Inside 0.30 m
  `radial20_heading_freeze` holds whatever heading the rover has (up to 5 deg).
  The 0.75 -> 0.30 m zone is the only zone still steered on the nozzle.
- A small constant right-of-travel bias: +3.5 / +7.1 / +7.7 / +5.1 mm on rows
  1-4. Rows alternate direction, so this is vehicle-side, not a frame/GNSS
  offset. Mostly from the decel zone (+10 mm mean there, +6 mm at cruise).

## Step 1: collect today's bags

Mac must be on the `DYX-4WD` hotspot (192.168.3.101).

```bash
ssh flash@192.168.3.101 'ls -dt ~/bags_jet/*20261006_*'
```

For each bundle, read `manifest.json` (`identity.total_points`,
`dummy_point_count`, `dummy_point_distance_m`, `execution_mode`) and its size.
Pull complete AUTO missions with >= 2 rows into `bags_jet/Oct_06/`:

```bash
rsync -a flash@192.168.3.101:bags_jet/<bundle> bags_jet/Oct_06/
```

The hotspot is slow (~0.5 MB/s): a 300 MB bag takes ~10 min. Run it in the
background. Skip bundles with no `/rpp/terminal_result` (aborted runs). Note
which runs used extension 1 m vs 2 m, and which ran at ~0.6 m/s instead of
1.0 m/s (pending task 3); keep those apart, because the lookahead and braking
depend on speed.

## Step 2: run the report on every bundle

```bash
python3 scripts/straight_leg_tracking_report.py bags_jet/Oct_06/<bundle> [...]
```

It prints, per leg: outcome, max abs xt, cruise p95, decel/terminal max, xt at
the stop, and the commanded-yaw turn in the last 1 m; then phase stats, the
signed bias per row, and the leg count over 40/50 mm. Method notes are in its
docstring.

## Step 3: questions the data must answer

1. **Is the end-of-leg swing general?** Across all bags, does p95 jump from
   cruise to decel/terminal, and are > 40 mm legs mostly made after 0.8 m to
   go? If cruise p95 is also > 35 mm on other runs, the problem is wider than
   the terminal zone. Then look at the cruise loop: lookahead, the
   `moving_course_bias`, and yaw damping.
2. **Does the commanded yaw explain it?** Per leg, correlate the commanded-yaw
   turn in the last 1 m with (xt at stop - xt at 0.75 m). A strong correlation
   with the expected sign (yaw left -> nozzle right) confirms the controller
   causes it. If the yaw setpoint stays on the line but the measured yaw
   moves, it is the drivetrain (uneven braking), not the steering.
   Caveat: the published yaw includes `moving_course_bias`
   (`course_compensated_yaw`). Subtract the bias, from `/rpp/debug`
   `course_bias_deg`, before reading the turn.
3. **Where does the turn start?** Find the distance to go where the
   commanded yaw first moves > 1 deg. If it starts at 0.75 m (the radial20
   hand-off), that confirms the code read. If it starts earlier, look at
   `approach_slowdown` (1.0 m) and `line_guidance` instead.
4. **Is the right bias in every run, in both directions?** If yes, it is
   vehicle-side; check `moving_course_bias` and PX4 yaw tracking. It is a
   separate, smaller item from the terminal swing.
5. **RPP vs raw GNSS.** For each failed point compare RPP xt with the
   raw-GNSS radial in `/mission_manager/point_event`. On Oct-06, P0017 read
   23 mm (RPP) vs 38 mm (raw), and P0038 21 vs 32. Large gaps point at the
   ~1.7 s EKF settle after stop, not at steering.

Keep the reporting rules from the `analyse-missions` skill: name the source of
every number (odom vs RPP self-graded vs raw GNSS). Never compare positions
across an RTK re-fix (`RTK_PAUSED` events). These numbers are the rover's
belief vs its line, not paint truth.

## Step 4: fix proposal to confirm or adjust with the data

Only if steps 3.1-3.3 confirm the 0.75 -> 0.30 m nozzle steering:

- In the radial20 terminal branch, replace `desired_goal_bearing =
  atan2(goal - nozzle)` with the bearing from `line_guidance(path_bearing,
  goal_x, goal_y, terminal_decel_correction_limit)`. That steers the turning
  point with a lookahead of >= 0.35 m, the same steering as cruise. Keep
  `terminal_bounded_guidance` (12 -> 3 deg limit, 15 deg/s slew), the heading
  freeze, radial20 speed/stop and the raw-GNSS spray gate unchanged.
- Gate it with a new parameter `radial20_turning_point_steering`: default
  `False` in code, `True` in `rover.launch.py`, so it can be rolled back with
  one launch line and a restart.
- Tests: update `test_radial20_uses_bounded_goal_bearing_for_steering`
  (it pins today's goal-pointing). Add a closed-loop case to
  `test_steer_turning_point.py` (it has a 0.5 m lever-arm skid-steer model):
  enter at 0.75 m with a 15 mm offset, decel to 0.1 m/s, compare xt at stop,
  old vs new. The freeze and turning-point tests must still pass.
- Expected: commanded-yaw turn in the last 0.75 m < ~1.5 deg; xt at stop
  close to the xt on entry (5-20 mm) instead of 37-40 mm. It does not remove
  an offset that is already there at 0.75 m, the right bias, or
  along-track short stops.
- Field check after deploy: same report on the next mission; compare decel and
  terminal p95 and the > 40 mm leg count against this plan's table.

Commit, push and deploy only on the operator's go-ahead (`rover-ship` skill).
