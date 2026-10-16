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
        super().__init__("dyx_h8d_command_test")
        self.events = []
        self.publisher = self.create_publisher(String, CMD, 10)
        self.subscription = self.create_subscription(
            String, STATUS, self.receive, 10
        )

    def receive(self, msg):
        self.events.append(msg.data)

    def wait(self, required, since=0, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            for event in self.events[since:]:
                if all(item in event for item in required):
                    return event
        raise AssertionError(
            f"Expected {required}; recent={self.events[-6:]}"
        )

    def send(self, command, accepted, reason=None):
        cursor = len(self.events)
        msg = String()
        msg.data = command
        self.publisher.publish(msg)

        marker = (
            "COMMAND_ACCEPTED=" if accepted
            else "COMMAND_REJECTED="
        ) + command

        requirements = [marker]
        if reason:
            requirements.append(reason)

        event = self.wait(requirements, cursor)
        print(
            "PASS:", command,
            "ACCEPTED" if accepted else "REJECTED"
        )
        return event


def scenario(h, label, healthy, test):
    args = [
        "ros2", "run", "dyx_mission_manager",
        "dyx_mission_lifecycle_test_node",
        "--ros-args",
        "-p", "simulation_only:=true",
        "-p", "simulation_initial_healthy:=" +
        ("true" if healthy else "false")
    ]

    with open(f"/tmp/dyx_h8d_{label}.log", "w") as log:
        process = subprocess.Popen(
            args, stdout=log,
            stderr=subprocess.STDOUT,
            env=os.environ.copy()
        )

        try:
            deadline = time.monotonic() + 8

            while h.count_subscribers(CMD) == 0:
                if process.poll() is not None:
                    raise RuntimeError(
                        f"Node exited during {label}"
                    )
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"Node startup timeout: {label}"
                    )
                rclpy.spin_once(h, timeout_sec=0.1)

            cursor = len(h.events)
            test(h, cursor)

        finally:
            process.terminate()
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def fail_closed(h, cursor):
    h.wait(["SAFETY=EMERGENCY_STOP_LATCHED"], cursor)

    h.send("START", False)
    h.send("NEXT", False, "Safety gate denies NEXT")
    h.send("SKIP", False, "Safety gate denies SKIP")

    h.send("STOP", True)
    h.wait(["STATE=stopped"])

    print("PASS: Fail-closed command restrictions")


def healthy_then_unsafe(h, cursor):
    h.wait(["STATE=ready",
            "SAFETY=RTK_FIXED_HEALTHY"], cursor)

    # Healthy NEXT must still respect mission state.
    h.send("NEXT", False, "NEXT requires")

    h.send("START", True)
    h.wait(["STATE=approaching"])

    h.send("SIM_FLOAT", True)
    h.wait(["STATE=paused",
            "CONTINUE_ALLOWED=false"])

    h.send("NEXT", False, "Safety gate denies NEXT")
    h.send("SKIP", False, "Safety gate denies SKIP")
    h.send("RESUME", False)

    h.send("SIM_HEALTHY", True)
    h.wait(["STATE=paused",
            "SAFETY=RTK_FIXED_HEALTHY"])

    h.send("RESUME", True)
    h.wait(["STATE=approaching"])

    h.send("STOP", True)
    h.wait(["STATE=stopped"])

    print("PASS: Safety recovery requires explicit RESUME")


rclpy.init()
h = Harness()

try:
    scenario(h, "fail_closed", False, fail_closed)

    scenario(
        h, "healthy_then_unsafe",
        True, healthy_then_unsafe
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

    print("PASS: No checked actuator command publishers")
    print("ALL H8D MISSION COMMAND SAFETY TESTS PASSED")

finally:
    h.destroy_node()
    rclpy.shutdown()
