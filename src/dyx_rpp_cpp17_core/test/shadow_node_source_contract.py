#!/usr/bin/env python3
"""Fail a Windows-native CTest if the shadow ROS2 node grows any actuator writer.
This guards the source contract but does NOT substitute for Jetson compilation.
"""
from pathlib import Path
import re

root_src = Path(__file__).resolve().parents[2]
node = root_src / 'dyx_rpp_cpp17_shadow' / 'src' / 'shadow_node.cpp'
source = node.read_text(encoding='utf-8')
publish_types = re.findall(r'create_publisher\s*<\s*([^>]+)>', source)
assert publish_types == ['std_msgs::msg::String'], publish_types
assert '"/dyx/rpp_cpp17_shadow/status"' in source
assert '"/fmu/out/vehicle_local_position"' in source
assert '"/fmu/out/vehicle_angular_velocity"' in source
assert 'declare_parameter<bool>("shadow_enabled", false)' in source
assert 'declare_parameter<double>("preview_speed_mps", 0.0)' in source
assert 'in.pivot_permission = false;' in source
assert 'in.stop_requested = radial <= 0.020;' in source
print('PASS: shadow ROS2 node owns only the diagnostic String publisher')
