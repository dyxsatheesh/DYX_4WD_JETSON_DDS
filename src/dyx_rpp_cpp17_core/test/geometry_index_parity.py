#!/usr/bin/env python3
"""End-to-end Phase 2B geometry parity against unchanged authoritative Python.
No ROS or hardware; mismatches fail the test.
"""
from __future__ import annotations
import math
from pathlib import Path
import random
import subprocess
import sys

PROBE = Path(sys.argv[1]).resolve()
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "rpp_controller"))
from rpp_controller.path_geometry import (PathGeometryIndex, GeometryProgressTracker,
                                          Point2D)

checks = 0

def fopt(value):
    return -999 if value is None else value

def flatten(p):
    return [fopt(p.segment_index) if p.segment_index is not None else -1,
            p.projected_s, p.progress_s, p.point.x, p.point.y,
            p.signed_cross_track_m, p.distance_m, p.t, int(p.used_full_reacquire),
            p.remaining_to_active_stop_m, p.remaining_path_m,
            fopt(p.next_corner_distance_m), fopt(p.next_corner_angle_rad),
            fopt(p.next_leg_heading_rad), int(p.monotonic_clamped)]

def compare(mode, raw, start, stop, xy, opt=None, lookahead=0.33, second=None):
    global checks
    opt = opt or {}
    geom = PathGeometryIndex.build([Point2D(x, y) for x,y,t,m in raw],
                                   [t for x,y,t,m in raw],
                                   [m for x,y,t,m in raw])
    span = geom.active_span(start_raw_index=start, stop_raw_index=stop)
    prefix = [mode, str(len(raw))]
    for x,y,t,m in raw:
        prefix.extend((repr(x),repr(y),str(t),str(m)))
    prefix.extend((str(-1 if start is None else start),str(stop)))
    if mode == "index":
        kwargs = dict(hint_segment_index=opt.get("hint"),
                      reference_s=opt.get("reference"),
                      back_window_segments=opt.get("back",2),
                      forward_window_segments=opt.get("forward",4),
                      max_backward_jump_m=opt.get("maxback",0.10),
                      max_forward_jump_m=opt.get("maxforward"),
                      full_reacquire_distance_m=opt.get("reacquire"))
        ref = geom.project(Point2D(*xy),active_span=span,**kwargs)
        tgt = geom.lookahead_target(ref.progress_s,lookahead,active_span=span)
        expected = [geom.total_length,len(geom.segments),len(geom.corners),
            len(geom.semantic_anchors),span.start_s,span.stop_s,
            span.first_segment_index if span.first_segment_index is not None else -1,
            span.last_segment_index if span.last_segment_index is not None else -1]
        expected.extend(flatten(ref))
        expected.extend([tgt.segment_index if tgt.segment_index is not None else -1,
            tgt.t,tgt.point.x,tgt.point.y,tgt.s,fopt(tgt.heading_rad)])
        for c in geom.corners:expected.extend([c.raw_index,c.s,c.turn_angle_rad])
        prefix.extend([*(repr(v) for v in xy),
            str(opt.get("hint",-1)),repr(fopt(opt.get("reference"))),
            str(opt.get("back",2)),str(opt.get("forward",4)),
            repr(opt.get("maxback",0.10)),repr(fopt(opt.get("maxforward"))),
            repr(fopt(opt.get("reacquire"))),repr(lookahead)])
    elif mode == "track":
        if second is None:raise ValueError("second query needed")
        kwargs = dict(back_window_segments=opt.get("back",2),
                      forward_window_segments=opt.get("forward",4),
                      max_backward_jump_m=opt.get("maxback",0.10),
                      max_forward_jump_m=opt.get("maxforward"),
                      full_reacquire_distance_m=opt.get("reacquire"))
        tracker=GeometryProgressTracker(geom)
        p1=tracker.update(Point2D(*xy),active_span=span,**kwargs)
        p2=tracker.update(Point2D(*second),active_span=span,**kwargs)
        expected=flatten(p1)+flatten(p2)
        prefix.extend([*(repr(v) for v in (*xy,*second)),
            str(opt.get("back",2)),str(opt.get("forward",4)),
            repr(opt.get("maxback",0.10)),repr(fopt(opt.get("maxforward"))),
            repr(fopt(opt.get("reacquire")))])
    else:raise ValueError(mode)
    result=subprocess.run([str(PROBE),*prefix],capture_output=True,text=True,check=True)
    actual=[float(x) for x in result.stdout.split()]
    if len(actual)!=len(expected):
        raise AssertionError(f"{mode}: shape mismatch: {len(actual)} vs {len(expected)}: {result.stdout}")
    for i,(a,b) in enumerate(zip(actual,expected)):
        if not math.isclose(a,float(b),rel_tol=1e-10,abs_tol=1e-10):
            raise AssertionError(f"{mode}: element {i} {a} != Python {b}, raw={raw}, options={opt}")
    checks+=1

cases = [
    # Straight, cross-track polarity, and semantic bounds.
    ([(0,0,0,-1),(2,0,2,0),(4,0,2,1)],0,1,[(1,0.02),(2,-0.02),(3,0.02)]),
    # Repeated raw samples must not be renumbered or become segments.
    ([(0,0,0,-1),(1,0,1,-1),(1,0,0,-1),(1,2,2,0),(0,2,2,1)],0,3,[(0.9,1),(1.1,1),(1,0)]),
    # A marking point at origin is a valid active span with zero segments.
    ([(0,0,2,0),(0,0,2,1)],0,1,[(0,0),(.1,.1)]),
    # Nearly overlapping and self-intersecting paths.
    ([(0,0,0,-1),(2,2,2,0),(0,2,0,-1),(2,0,2,1),(0,0,2,2)],0,3,[(1,1),(2,2),(1.01,.99)]),
    # Corners, +90 and -90 degrees, and local progress.
    ([(0,0,0,-1),(2,0,1,-1),(2,2,2,0),(4,2,2,1)],0,2,[(2.01,.7),(1.8,1.9),(2,0)]),
]
for raw,start,stop,points in cases:
    for q in points:
        compare("index",raw,start,stop,q)
        compare("index",raw,start,stop,q,lookahead=0.80)

# Optional hint windows, bounded reacquisition and progress clamping.
base=[(0,0,0,-1),(1,0,0,-1),(2,0,0,-1),(3,0,1,-1),
      (4,0,0,-1),(5,0,2,0)]
for q,hint,reacquire in [((4.3,.02),0,None),((4.3,.02),0,.20),
                         ((1.2,.03),2,.20),((4.3,.02),1,.01)]:
    compare("index",base,0,5,q,{"hint":hint,"back":0,"forward":0,
                                    "reacquire":reacquire})
for pts in [((1,.02),(.95,.02)),((1,.02),(1.04,-.02)),
            ((2.1,0),(2.09,.03)),((0.1,.01),(1.4,-.01))]:
    compare("track",base,0,5,pts[0],second=pts[1])

# Deterministic off-path noise; Python remains the semantic oracle.
rng=random.Random(20261012)
for _ in range(30):
    x=rng.uniform(0,4)
    y=rng.uniform(-.2,.2)
    compare("index",base,0,5,(x,y),lookahead=rng.uniform(0,.6))
print(f"PASS: {checks} C++17 / current Python full path geometry parity scenarios")
