#!/usr/bin/env python3
"""One additional independent non-actuating ENU/NED telemetry contract check.
The native executable covers runtime rejections and forward-only behavior.
"""
from __future__ import annotations
import math
from pathlib import Path
import subprocess
import sys

PROBE = Path(sys.argv[1]).resolve()
assert PROBE.is_file(), PROBE
for ned, enu in [(0.0, math.pi / 2), (math.pi / 2, 0.0),
                 (-math.pi / 2, math.pi), (math.pi, -math.pi / 2)]:
    assert math.isclose((math.pi / 2 - ned + math.pi) % (2 * math.pi) - math.pi,
                        -math.pi if math.isclose(enu, math.pi) else enu,
                        abs_tol=1e-12)
subprocess.run([str(PROBE)], check=True)
print("PASS: PX4 NED/ENU preview contract; native non-actuation smoke")
