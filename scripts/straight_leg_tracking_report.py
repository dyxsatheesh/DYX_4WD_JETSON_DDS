#!/usr/bin/env python3
"""Straight in-row leg tracking report for one bag bundle (2026-10-06).

Measures the nozzle (odom origin = GNSS antenna = nozzle) against the FIXED
line previous-marking-point -> marking-point, independently of RPP's own
cross-track. Legs: consecutive points in the same mission.csv `row`, skipping
the first leg of each row (it starts with a pivot) and any leg after an
extension. Sign: + = RIGHT of travel. Speed is position-derived (0.5 s).

Also reports, per leg, how far the COMMANDED yaw (/mavros/setpoint_raw/local,
explicit-yaw mask) turns away from the line inside the last 1.0 m: this is
the 2026-10-06 finding (radial20 nozzle-at-goal steering, 0.75->0.30 m).

Usage: python3 scripts/straight_leg_tracking_report.py <bundle-dir> [...]
"""
import csv
import math
import sqlite3
import statistics as st
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_mission import _CDR, _p_odometry, _p_pose_stamped, _p_string  # noqa: E402

PHASES = ("start(0-1m)", "cruise", "decel(0.3-1m)", "terminal(<0.3m)")


def _rows(db, topic):
    return db.execute(
        "select m.timestamp,m.data from messages m join topics t on t.id=m.topic_id "
        "where t.name=? order by m.timestamp", (topic,)).fetchall()


def _setpoint(d):
    r = _CDR(d); r.header(); r.u8(); mask = r.u16()
    p = [r.f64() for _ in range(9)]
    return mask, p[3], p[4], r.f32()


def _phase(rem, s, v):
    if v < 0.05:
        return None
    if rem < 0.30:
        return PHASES[3]
    if rem < 1.00:
        return PHASES[2]
    if s < 1.00:
        return PHASES[0]
    return PHASES[1]


def _q(sorted_abs, p):
    return sorted_abs[int(p * (len(sorted_abs) - 1))]


def report(bundle):
    bundle = Path(bundle)
    db = sqlite3.connect(bundle / "bag" / "bag_0.db3")
    odom = []
    for ts, d in _rows(db, "/mavros/local_position/odom"):
        o = _p_odometry(d)
        odom.append((ts / 1e9, o["x"], o["y"]))
    sp = []
    for ts, d in _rows(db, "/mavros/setpoint_raw/local"):
        mask, vx, vy, yaw = _setpoint(d)
        if mask == 0x1C7 and math.hypot(vx, vy) > 1e-3:
            sp.append((ts / 1e9, yaw))
    goals = {}
    for ts, d in _rows(db, "/rpp/terminal_result"):
        j = _p_string(d)["json"]
        if j.get("marking_number"):
            goals[j["marking_number"]] = (j["goal_x"], j["goal_y"], ts / 1e9, j.get("outcome"))
    with open(bundle / "mission.csv") as f:
        row_of = {i: r.get("row") for i, r in enumerate(csv.DictReader(f), 1)}
    sg = [(ts / 1e9, _p_pose_stamped(d)) for ts, d in _rows(db, "/segment_goal")]

    print(f"\n### {bundle.name}  ({len(goals)} marking results)")
    print(f"{'leg':>6} {'out':>8} {'max|xt|':>8} {'cruise95':>8} {'decelMax':>8} {'termMax':>8} {'stopXt':>7} {'yawSpTurn':>9}")
    bins, by_row, turns, legmax = {}, {}, [], []
    for n in sorted(goals):
        if n - 1 not in goals or row_of.get(n) != row_of.get(n - 1):
            continue
        if row_of.get(n - 2) != row_of.get(n - 1):
            continue
        ax, ay, _, _ = goals[n - 1]
        bx, by, tcap, outcome = goals[n]
        t0 = next((t for t, p in sg if math.hypot(p["x"] - bx, p["y"] - by) < 0.01), None)
        if t0 is None:
            continue
        L = math.hypot(bx - ax, by - ay)
        ux, uy = (bx - ax) / L, (by - ay) / L
        path_yaw = math.atan2(uy, ux)
        pts = [(t, (x - ax) * ux + (y - ay) * uy, (x - ax) * uy - (y - ay) * ux)
               for t, x, y in odom if t0 <= t <= tcap]
        g, j, rem_t = {}, 0, []
        for t, s, xt in pts:
            while pts[j][0] < t - 0.5:
                j += 1
            dt = t - pts[j][0]
            v = (s - pts[j][1]) / dt if dt > 0.3 else 0.0
            k = _phase(L - s, s, v)
            if k:
                g.setdefault(k, []).append(xt * 1000)
                bins.setdefault(k, []).append(xt * 1000)
                by_row.setdefault(row_of[n], []).append(xt * 1000)
            if L - s < 1.0:
                rem_t.append(t)
        if not g:
            continue
        turn = float("nan")
        if rem_t:
            ys = [math.degrees(math.remainder(y - path_yaw, math.tau))
                  for t, y in sp if rem_t[0] <= t <= tcap]
            if ys:
                turn = max(ys, key=abs)
                turns.append(turn)
        mx = lambda k: max((abs(x) for x in g.get(k, [])), default=float("nan"))
        c = sorted(abs(x) for x in g.get(PHASES[1], []))
        allm = max(abs(x) for k in g for x in g[k])
        legmax.append(allm)
        print(f"P{n:04d} {str(outcome)[:8]:>8} {allm:8.1f} {(_q(c, .95) if c else float('nan')):8.1f} "
              f"{mx(PHASES[2]):8.1f} {mx(PHASES[3]):8.1f} {pts[-1][2] * 1000:+7.1f} {turn:+9.2f}")
    print()
    for k in PHASES:
        a = bins.get(k, [])
        if not a:
            continue
        ab = sorted(abs(x) for x in a)
        print(f"{k:17s} n={len(a):5d} mean={st.mean(a):+6.1f} p50|xt|={_q(ab, .5):5.1f} "
              f"p95={_q(ab, .95):5.1f} max={ab[-1]:6.1f} >30mm={100 * sum(x > 30 for x in ab) / len(ab):4.1f}%")
    for r, a in sorted(by_row.items()):
        print(f"row {r}: mean signed {st.mean(a):+.1f} mm (n={len(a)})")
    if turns:
        print(f"commanded-yaw turn off line in last 1 m: p50 |{st.median(abs(x) for x in turns):.2f}| deg, "
              f">2 deg on {sum(abs(x) > 2 for x in turns)}/{len(turns)} legs")
    print(f"legs={len(legmax)}  max|xt|>40mm: {sum(m > 40 for m in legmax)}  >50mm: {sum(m > 50 for m in legmax)}")


if __name__ == "__main__":
    for b in sys.argv[1:]:
        report(b)
