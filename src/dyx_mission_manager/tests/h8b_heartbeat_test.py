import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

CMD = "/dyx_test/mission_lifecycle/command"
STATUS = "/dyx_test/mission_lifecycle/status"

class Harness(Node):
    def __init__(self):
        super().__init__("dyx_h8b_heartbeat_harness")
        self.events = []
        self.pub = self.create_publisher(String, CMD, 10)
        self.sub = self.create_subscription(
            String, STATUS, self.on_status, 10
        )

    def on_status(self, msg):
        self.events.append(msg.data)

    def wait(self, required, since=0, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            for event in self.events[since:]:
                if all(s in event for s in required):
                    return event
        raise AssertionError(
            f"Expected {required}; recent={self.events[-5:]}"
        )

    def send(self, command, accepted=True):
        cursor = len(self.events)
        msg = String()
        msg.data = command
        self.pub.publish(msg)

        result = (
            "COMMAND_ACCEPTED=" if accepted
            else "COMMAND_REJECTED="
        ) + command

        self.wait([result], cursor)
        print("PASS:", command,
              "accepted" if accepted else "rejected")

def run_case(h, label, fault_command, fault_reason):
    args = [
        "ros2", "run", "dyx_mission_manager",
        "dyx_mission_lifecycle_test_node",
        "--ros-args",
        "-p", "simulation_only:=true",
        "-p", "simulation_initial_healthy:=true"
    ]

    with open(f"/tmp/dyx_h8b_{label}.log", "w") as log:
        proc = subprocess.Popen(
            args, stdout=log, stderr=subprocess.STDOUT,
            env=os.environ.copy()
        )

        try:
            deadline = time.monotonic() + 8
            while h.count_subscribers(CMD) == 0:
                if proc.poll() is not None:
                    raise RuntimeError("Mission node exited")
                if time.monotonic() > deadline:
                    raise RuntimeError("Mission node startup timeout")
                rclpy.spin_once(h, timeout_sec=0.1)

            cursor = len(h.events)
            h.wait(["STATE=ready",
                    "SAFETY=RTK_FIXED_HEALTHY"], cursor)

            h.send("START")
            h.wait(["STATE=approaching",
                    "CONTINUE_ALLOWED=true"])
            print("PASS:", label, "healthy mission started")

            cursor = len(h.events)
            h.send(fault_command)

            event = h.wait(
                ["STATE=paused",
                 "CONTINUE_ALLOWED=false",
                 "SAFETY=" + fault_reason],
                cursor,
                timeout=6
            )

            assert "HOLD_ZERO=true" in event
            assert "PHYSICAL_COMMAND_PERMITTED=false" in event
            print("PASS:", label,
                  "heartbeat timeout inhibits mission")

            h.send("RESUME", accepted=False)

            cursor = len(h.events)
            h.send("SIM_HEALTHY")

            h.wait(
                ["STATE=paused",
                 "SAFETY=RTK_FIXED_HEALTHY"],
                cursor
            )
            print("PASS:", label,
                  "recovery does not auto-resume")

            h.send("RESUME")
            h.wait(["STATE=approaching",
                    "CONTINUE_ALLOWED=true"])
            print("PASS:", label, "explicit RESUME")

            h.send("STOP")
            h.wait(["STATE=stopped"])
            print("PASS:", label, "STOP")

        finally:
            proc.terminate()
            try:
                proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

rclpy.init()
h = Harness()

try:
    run_case(
        h, "px4",
        "SIM_PX4_STALE",
        "PX4_HEARTBEAT_STALE"
    )

    run_case(
        h, "backend",
        "SIM_BACKEND_STALE",
        "BACKEND_HEARTBEAT_STALE"
    )

    for topic in (
        "/fmu/in/trajectory_setpoint",
        "/fmu/in/offboard_control_mode",
        "/rpp/command"
    ):
        if h.get_publishers_info_by_topic(topic):
            raise AssertionError(
                f"Unexpected command publisher: {topic}"
            )

    print("PASS: No actuator command publishers detected")
    print("ALL H8B HEARTBEAT TESTS PASSED")

finally:
    h.destroy_node()
    rclpy.shutdown()
