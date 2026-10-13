#!/usr/bin/env python3
"""Phase 2C1: compare complete C++17 precision guidance with live Python.
No ROS or hardware, and no changes to the Python reference implementation.
"""
import math
from pathlib import Path
import random
import subprocess
import sys

PROBE = Path(sys.argv[1]).resolve() if len(sys.argv) == 2 else None
if PROBE is None or not PROBE.is_file():
    raise SystemExit("Usage: python guidance_parity.py <guidance_probe_executable>")
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "rpp_controller"))
from rpp_controller.path_geometry import PathGeometryIndex, Point2D
from rpp_controller.guidance import GuidanceConfig, compute_precision_guidance

CHECKS = 0

def verify(coords, types, markings, start, stop, pos, yaw, speed, cfg):
    global CHECKS
    geometry=PathGeometryIndex.build(coords, types, markings)
    span=geometry.active_span(start_raw_index=start, stop_raw_index=stop)
    proj=geometry.project(Point2D(*pos), active_span=span)
    p=compute_precision_guidance(cfg, geometry=geometry, projection=proj,
        active_span=span, rover_position=Point2D(*pos),
        rover_yaw_rad=yaw, speed_mps=speed)
    expected=[p.lookahead_distance_m,p.lookahead_target_s,
        -1 if p.lookahead_segment_index is None else p.lookahead_segment_index,
        p.lookahead_point.x,p.lookahead_point.y,
        p.steering_target_point.x,p.steering_target_point.y,
        p.actual_steering_target_distance_m,int(p.endpoint_extension_used),
        p.endpoint_extension_distance_m,int(p.target_behind_rover),
        p.lookahead_bearing_rad,p.local_path_heading_rad,
        p.path_heading_error_rad,p.heading_error_rad,p.signed_cross_track_m,
        p.desired_movement_bearing_rad,p.limited_command_bearing_rad,
        p.final_command_correction_rad,int(p.bearing_clamp_fired),
        int(p.zero_vector_fallback_used)]
    params=[*pos,yaw,speed,cfg.lookahead_min_m,cfg.lookahead_max_m,
        cfg.lookahead_time_s,cfg.xtrack_lookahead_gain,
        cfg.moving_bearing_cone_rad,start,stop,len(coords)]
    for (x,y),t,m in zip(coords,types,markings):
        params.extend([x,y,t,m])
    cmd=[str(PROBE),*[repr(v) for v in params]]
    result=subprocess.run(cmd,capture_output=True,text=True,check=True)
    actual=[float(x) for x in result.stdout.strip().split()]
    if len(actual)!=len(expected):
        raise AssertionError(f"expected {len(expected)} outputs, got {len(actual)}")
    for index,(got,want) in enumerate(zip(actual,expected)):
        if not math.isclose(got,float(want),abs_tol=2e-10,rel_tol=2e-10):
            raise AssertionError(f"Case {CHECKS}, field {index}: C++={got}, Python={want}; {pos=}")
    CHECKS+=1

cfg=GuidanceConfig()
# Straight-line tracking; 20mm left/right offset, positive and negative yaw,
# goal overshoot, target coincidence, speed transition, and endpoint extension.
for pos in [(1.2,0.02),(1.2,-0.02),(3.95,0.01),(4.0,0.0),
            (4.20,0.0),(0.0,0.0),(1.3,0.15),(-0.1,0.0)]:
    for yaw in [0.0,0.25,-0.25,math.pi]:
        for speed in [0.0,0.6,1.0]:
            verify([(0,0),(2,0),(4,0)], [0,0,2],[-1,-1,0],
                   0,2,pos,yaw,speed,cfg)
# Corners and active semantic spans (do not extend through a marking point).
for pos in [(2.8,0.05),(3.0,0.0),(3.0,0.15),(3.12,0.05),(3.0,1.2)]:
    for yaw in [0.0,math.pi/2,math.pi,-math.pi/2]:
        verify([(0,0),(3,0),(3,3)], [0,2,2],[-1,0,1],
               1,2,pos,yaw,0.6,cfg)
# Duplicate/zero-length input and single semantic anchor without any segment.
verify([(0,0),(2,0),(2,0),(4,0)], [0,0,1,2],[-1,-1,-1,0],
       2,3,(3,0.01),0.0,0.6,cfg)
verify([(1,2)], [2],[0],0,0,(1,2),0.8,0.0,cfg)
# Fixed seed for repeatable heading/cone and lookahead stress testing.
rng=random.Random(20261013)
for _ in range(80):
    pos=(rng.uniform(-0.5,4.6),rng.uniform(-0.3,0.3))
    yaw=rng.uniform(-math.pi,math.pi)
    cfg2=GuidanceConfig(lookahead_min_m=0.2,
        lookahead_max_m=1.0,lookahead_time_s=0.55,
        xtrack_lookahead_gain=rng.uniform(0.0,1.0),
        moving_bearing_cone_rad=math.radians(30))
    verify([(0,0),(4,0)], [0,2],[-1,0],0,1,
           pos,yaw,rng.uniform(0.0,1.0),cfg2)
print(f"PASS: {CHECKS} full precision-guidance Python/C++17 parity cases")
