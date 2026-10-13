#!/usr/bin/env python3
"""Contract checks for NEW forward-only C++17 planner (not Python parity).

The current Python RPP has no equivalent direct signed-rate control contract.
Check geometric sign conventions and fail-closed state transitions independently.
"""
import math
from pathlib import Path
import random
import subprocess
import sys

probe = Path(sys.argv[1]).resolve()
assert probe.is_file(), probe
rng = random.Random(20261013)
actions = []
def reset(): actions.append("reset")
def step(now, yaw, target, speed=0.6, dt=0.05, rate=0.0,
         enabled=True, fresh=True, permission=False, stop=False):
    actions.append("step " + " ".join(map(repr, [now,dt,yaw,target,rate,speed,
        int(enabled),int(fresh),int(permission),int(stop)])))

reset()
for i in range(100):
    yaw = rng.uniform(-math.pi, math.pi)
    delta = rng.uniform(-0.2, 0.2)  # < 15 degrees, no pivot needed
    step(i*0.05, yaw, yaw+delta, speed=rng.uniform(0, 1.2))
reset()
step(0.0,0,math.radians(20),permission=False)
step(0.05,0,math.radians(20),permission=True)
for i in range(1,6):
    step(0.05+i*0.05,math.radians(20),math.radians(20),permission=True)
reset()
step(0,0,-math.pi/2,permission=True)
step(0.05,0,-math.pi/3,permission=True)  # goal swap => latched safety
step(0.1,0,0)                            # remains latched
reset()
step(0,0,0,speed=-0.01)  # negative body-speed request is invalid
step(0.05,0,0,speed=0.5)
reset()
step(0,0,0,enabled=False)
reset()
step(0,0,0,fresh=False)
reset()
step(0,0,0,stop=True)

p = subprocess.run([str(probe)], input="\n".join(actions)+"\n",
                   capture_output=True,text=True,check=True)
rows=[line.split() for line in p.stdout.splitlines()]
assert len(rows) == sum(a.startswith('step') for a in actions), len(rows)
idx=0
for a in actions:
    if a=='reset': continue
    row=rows[idx];idx+=1
    assert len(row)==10,(idx,row)
    state,reason=row[:2]
    forward,north,east,enu,ned,err=map(float,row[2:8]); safe,done=map(int,row[8:])
    assert 0<=forward<=1.0+1e-12,(idx,row)
    assert math.isclose(enu,-ned,abs_tol=1e-12),(idx,row)
    assert abs(enu)<=0.18+1e-12,(idx,row)
    assert math.isclose(math.hypot(north,east),forward,abs_tol=1e-12),(idx,row)
    if state in ('PIVOT','SETTLE','WAIT_PIVOT','SAFE_HOLD') or done:
        assert forward==0.0,(idx,row)
    if safe or state=='SAFE_HOLD':
        assert forward==0.0 and enu==0.0,(idx,row)
    if 1<=idx<=100:
        vals=a.split();yaw=float(vals[3]);target=float(vals[4]);speed=float(vals[6]);
        assert state=='TRACK',(idx,row)
        assert math.isclose(forward,min(1.0,speed),abs_tol=1e-12)
        assert math.isclose(north,forward*math.sin(yaw),abs_tol=1e-12)
        assert math.isclose(east,forward*math.cos(yaw),abs_tol=1e-12)
        # Body-forward projection in custom PX4 combined mode is exactly +forward.
        yaw_ned=math.pi/2-yaw
        body_x=north*math.cos(yaw_ned)+east*math.sin(yaw_ned)
        assert body_x>=-1e-12 and math.isclose(body_x,forward,abs_tol=1e-12)
        reference_error=math.atan2(math.sin(target-yaw),math.cos(target-yaw))
        expected=0.0 if abs(reference_error)<=math.radians(.5) else max(-.15,min(.15,.85*reference_error))
        assert math.isclose(enu,expected,abs_tol=1e-11),(idx,enu,expected)
assert rows[100][0]=='WAIT_PIVOT'
assert rows[101][0]=='PIVOT'
assert rows[107][0]=='PIVOT' and rows[108][0]=='SAFE_HOLD'  # target swap
assert rows[109][0]=='SAFE_HOLD'
assert rows[110][0]=='SAFE_HOLD' and rows[111][0]=='SAFE_HOLD'  # reverse latch
assert rows[112][0]=='SAFE_HOLD' and rows[113][0]=='SAFE_HOLD'
assert rows[114][1]=='STOP_REQUESTED'
print(f'PASS: {len(rows)} forward-only yaw-rate contract frames; no physical actuation')
