import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from px4_msgs.msg import (
    BatteryStatus, SensorGps,
    VehicleStatus, VehicleLocalPosition
)

STATUS = "/dyx/h6d/safety_observer/status"

class Harness(Node):
    def __init__(self):
        super().__init__("dyx_h8e_dds_harness")
        self.statuses = []

        self.battery = self.create_publisher(
            BatteryStatus, "/fmu/out/battery_status_v1",
            qos_profile_sensor_data)
        self.gps = self.create_publisher(
            SensorGps, "/fmu/out/vehicle_gps_position",
            qos_profile_sensor_data)
        self.vehicle = self.create_publisher(
            VehicleStatus, "/fmu/out/vehicle_status_v1",
            qos_profile_sensor_data)
        self.position = self.create_publisher(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            qos_profile_sensor_data)

        self.sub = self.create_subscription(
            String, STATUS,
            lambda msg: self.statuses.append(msg.data), 10)

    def wait(self, fields, since=0, seconds=6):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            for line in self.statuses[since:]:
                if all(f in line for f in fields):
                    return line
        raise AssertionError(
            f"Expected {fields}, got {self.statuses[-5:]}")

    def publish_frame(self, stamp):
        b = BatteryStatus()
        b.timestamp = stamp
        b.voltage_v = 25.0
        b.connected = True
        b.is_powering_off = False
        b.cell_count = 8
        b.warning = 0
        self.battery.publish(b)

        g = SensorGps()
        g.timestamp = stamp
        g.timestamp_sample = stamp
        g.device_id = 1
        g.latitude_deg = 12.9
        g.longitude_deg = 79.1
        g.fix_type = 6
        self.gps.publish(g)

        v = VehicleStatus()
        v.timestamp = stamp
        v.arming_state = 1
        v.nav_state = 0
        v.vehicle_type = 3
        v.failsafe = False
        v.failure_detector_status = 0
        self.vehicle.publish(v)

        p = VehicleLocalPosition()
        p.timestamp = stamp
        p.timestamp_sample = stamp
        p.xy_valid = True
        p.v_xy_valid = True
        p.heading_good_for_control = True
        p.dead_reckoning = False
        p.x = 0.0
        p.y = 0.0
        p.heading = 0.0
        self.position.publish(p)

    def pump(self, frames, initial_stamp):
        for i in range(frames):
            self.publish_frame(initial_stamp + i * 50000)
            rclpy.spin_once(self, timeout_sec=0.05)

rclpy.init()
h = Harness()
process = None

try:
    with open("/tmp/dyx_h8e_dds_observer.log", "w") as log:
        process = subprocess.Popen([
            "ros2", "run", "dyx_safety_cpp17_observer",
            "dyx_safety_cpp17_observer_node",
            "--ros-args", "-p", "observe_enabled:=true"
        ], stdout=log, stderr=subprocess.STDOUT,
           env=os.environ.copy())

        deadline = time.monotonic() + 8

        while h.battery.get_subscription_count() == 0:
            if process.poll() is not None:
                raise RuntimeError("Observer exited")
            if time.monotonic() > deadline:
                raise RuntimeError("DDS subscription timeout")
            rclpy.spin_once(h, timeout_sec=0.1)

        h.wait(["battery_fresh=0",
                "PHYSICAL_COMMAND_PERMITTED=false"])
        print("PASS: Missing telemetry fails closed")

        cursor = len(h.statuses)
        h.pump(30, 1000000)

        h.wait(["battery_fresh=1",
                "gps_fresh=1",
                "vehicle_fresh=1"], cursor)
        print("PASS: Fresh PX4 DDS messages received")

        cursor = len(h.statuses)
        h.wait(["battery_fresh=0",
                "gps_fresh=0",
                "vehicle_fresh=0"], cursor, seconds=5)
        print("PASS: Missing updates become stale")

        cursor = len(h.statuses)
        for _ in range(40):
            h.publish_frame(2450000)
            rclpy.spin_once(h, timeout_sec=0.05)

        h.wait(["battery_fresh=0",
                "gps_fresh=0",
                "vehicle_fresh=0"], cursor)
        latest = h.wait(
            ["battery_fresh=0",
             "gps_fresh=0",
             "vehicle_fresh=0"],
            cursor
        )
        if "reason=TELEMETRY_INTEGRITY_LATCHED" in latest:
            raise AssertionError(
                "Duplicate timestamps unexpectedly latched integrity fault"
            )
        print("PASS: Duplicate PX4 stamps do not refresh telemetry")

        h.wait(["PHYSICAL_COMMAND_PERMITTED=false",
                "MISSION_TRANSITION_PERMITTED=false",
                "SPRAY_PERMITTED=false"])
        print("PASS: Observer remains non-actuating")

        for topic in (
            "/fmu/in/trajectory_setpoint",
            "/fmu/in/offboard_control_mode",
            "/rpp/command"
        ):
            if h.get_publishers_info_by_topic(topic):
                raise AssertionError(
                    f"Unexpected publisher on {topic}")

        print("PASS: No checked command publishers")
        print("ALL H8E DDS OBSERVER TESTS PASSED")

finally:
    if process is not None:
        process.terminate()
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    h.destroy_node()
    rclpy.shutdown()
