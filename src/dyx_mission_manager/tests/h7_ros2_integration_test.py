import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

COMMAND = "/dyx_test/mission_lifecycle/command"
STATUS = "/dyx_test/mission_lifecycle/status"

class Harness(Node):
    def __init__(self):
        super().__init__("dyx_h7_test_harness")
        self.events = []
        self.pub = self.create_publisher(String, COMMAND, 10)
        self.sub = self.create_subscription(
            String, STATUS,
            lambda msg: self.events.append(msg.data), 10
        )

    def wait_for(self, expected, start=0, timeout=6):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            for event in self.events[start:]:
                if expected in event:
                    return event
        raise AssertionError(
            f"Expected {expected!r}, recent={self.events[-5:]}"
        )

    def send(self, command, expected="ACCEPTED"):
        start = len(self.events)
        msg = String()
        msg.data = command
        self.pub.publish(msg)
        result = self.wait_for(
            f"COMMAND_{expected}={command}", start
        )
        print(f"PASS: {command} -> {expected}")
        return result

def run_case(h, label, healthy, test_fn):
    args = [
        "ros2", "run", "dyx_mission_manager",
        "dyx_mission_lifecycle_test_node",
        "--ros-args", "-p", "simulation_only:=true",
        "-p", f"simulation_initial_healthy:={'true' if healthy else 'false'}"
    ]
    log_path = f"/tmp/dyx_h7_{label}.log"

    with open(log_path, "w") as log:
        proc = subprocess.Popen(
            args, stdout=log, stderr=subprocess.STDOUT,
            env=os.environ.copy()
        )
        try:
            deadline = time.monotonic() + 8
            while h.count_subscribers(COMMAND) == 0:
                if proc.poll() is not None:
                    raise RuntimeError(
                        f"ROS node exited. Inspect {log_path}"
                    )
                if time.monotonic() > deadline:
                    raise RuntimeError("ROS node did not start")
                rclpy.spin_once(h, timeout_sec=0.1)

            cursor = len(h.events)
            test_fn(h, cursor)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

def fail_closed(h, cursor):
    h.wait_for("SAFETY=EMERGENCY_STOP_LATCHED", cursor)
    h.wait_for("START_ALLOWED=false", cursor)
    print("PASS: Default startup fails closed")
    h.send("START", "REJECTED")

def healthy(h, cursor):
    h.wait_for("SAFETY=RTK_FIXED_HEALTHY", cursor)
    h.wait_for("STATE=ready", cursor)
    print("PASS: Healthy simulation READY")

    h.send("START")
    h.wait_for("STATE=approaching")
    print("PASS: START -> approaching")

    h.send("SIM_FLOAT")
    h.wait_for("STATE=paused")
    h.wait_for("CONTINUE_ALLOWED=false")
    print("PASS: RTK FLOAT inhibits and pauses mission")

    h.send("RESUME", "REJECTED")
    print("PASS: Unsafe RESUME blocked")

    h.send("SIM_HEALTHY")
    h.wait_for("SAFETY=RTK_FIXED_HEALTHY")
    h.send("RESUME")
    h.wait_for("STATE=approaching")
    print("PASS: FIX recovery plus explicit RESUME")

    h.send("SIM_BATTERY_LOW")
    h.wait_for("STATE=paused")
    h.wait_for("SAFETY=BATTERY_BELOW_22V_DEBOUNCE")
    print("PASS: Low battery immediately inhibits")

    h.send("RESUME", "REJECTED")

    h.send("SIM_ESTOP")
    h.wait_for("SAFETY=EMERGENCY_STOP_LATCHED")
    h.wait_for("STATE=fault")
    print("PASS: Emergency stop latches fault")

    h.send("SIM_HEALTHY")
    h.wait_for("SAFETY=EMERGENCY_STOP_LATCHED")
    print("PASS: Healthy input cannot clear E-stop latch")

    h.send("STOP")
    h.wait_for("STATE=stopped")
    print("PASS: STOP accepted after fault")

    for topic in (
        "/fmu/in/trajectory_setpoint",
        "/fmu/in/offboard_control_mode",
        "/rpp/command"
    ):
        if h.get_publishers_info_by_topic(topic):
            raise AssertionError(f"Unexpected publisher: {topic}")
    print("PASS: No physical command publishers in test domain")

rclpy.init()
h = Harness()

try:
    run_case(h, "default_closed", False, fail_closed)
    run_case(h, "healthy_faults", True, healthy)
    print("ALL H7 ROS2 INTEGRATION TESTS PASSED")
finally:
    h.destroy_node()
    rclpy.shutdown()
