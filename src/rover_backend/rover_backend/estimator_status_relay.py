"""Forward only MAVLink ESTIMATOR_STATUS (#230) to rover_backend.

MAVROS publishes every FCU MAVLink frame on /uas1/mavlink_source (~350-420
Hz on this rover). rover_backend needs only ESTIMATOR_STATUS, which PX4 sends
at under 1 Hz. rclpy costs ~550 us per delivered message on the Jetson, even
when the callback discards it (measured 2026-09-30), so subscribing to the
raw stream kept the backend's executor thread ~50% busy and slowed every
API command through the GIL. This separate process absorbs that stream and
republishes only the frames the backend decodes.
"""

from __future__ import annotations

import rclpy
from mavros_msgs.msg import Mavlink
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

SOURCE_TOPIC = "/uas1/mavlink_source"
RELAY_TOPIC = "/rover_backend/estimator_status_mavlink"
ESTIMATOR_STATUS_MSGID = 230


class EstimatorStatusRelay(Node):
    def __init__(self) -> None:
        super().__init__("estimator_status_relay")
        self._publisher = self.create_publisher(
            Mavlink,
            RELAY_TOPIC,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.create_subscription(
            Mavlink,
            SOURCE_TOPIC,
            self._source_callback,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )

    def _source_callback(self, message: Mavlink) -> None:
        if int(message.msgid) != ESTIMATOR_STATUS_MSGID:
            return
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EstimatorStatusRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
