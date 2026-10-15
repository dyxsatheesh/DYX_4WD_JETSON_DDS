#!/usr/bin/env python3
"""Opt-in unit & manifest lint. No actual systemd or Jetson verification."""
from pathlib import Path
root = Path(__file__).resolve().parents[3]
unit = (root / 'tools/dyx_jetson_preflight/systemd/dyx-jetson-preflight.service').read_text()
assert 'Type=notify' in unit and 'WatchdogSec=' in unit
assert 'NoNewPrivileges=yes' in unit and 'DevicePolicy=closed' in unit
assert 'RestrictAddressFamilies=AF_UNIX' in unit
assert 'IPAddressDeny=any' in unit
assert 'WantedBy=' not in unit and '[Install]' not in unit.splitlines()
assert 'ros2 launch' not in unit and 'mavros' not in unit
assert 'enable_motion' not in unit
ros = (root / 'src/dyx_jetson_readonly_dds/src/readonly_dds_node.cpp').read_text()
assert 'create_subscription<' in ros and 'create_publisher<' not in ros
assert '/fmu/in/' not in ros
print('H5Z deployment/static source checks PASS (NOT hardware verification)')
