#!/usr/bin/env python3
"""Static gate for the added PX4 observer (NOT a runtime safety proof)."""
from pathlib import Path
root = Path(__file__).resolve().parents[3]
source = (root / 'src/dyx_jetson_readonly_dds/src/readonly_dds_node.cpp').read_text()
for forbidden in ('create_publisher', '/fmu/in/', 'trajectory_setpoint.hpp', 'mavros',
                  'VehicleCommand', 'actuator_motors', 'execute_process'):
    assert forbidden not in source, f'Unexpected control path: {forbidden}'
for required in ('/fmu/out/vehicle_status', '/fmu/out/vehicle_local_position',
                 'SensorDataQoS', 'output_enabled=false'):
    assert required in source, f'Missing observer field: {required}'
print('Read-only ROS source gate PASS (static only)')
