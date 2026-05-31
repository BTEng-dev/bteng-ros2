"""RosTopicMixin -- publisher and subscriber helpers for BTEng nodes."""

from __future__ import annotations

from bteng_ros2._mixin import RosNodeMixin


class RosTopicMixin(RosNodeMixin):
    """Adds ROS 2 publisher and subscriber helpers to any BTEng node.

    Thin wrappers around the underlying rclpy.Node calls. Combine freely
    with other mixins and BTEng node types:

        class StatusPublisher(RosTopicMixin, StatefulActionNode):
            def on_start(self):
                self._pub = self.create_publisher(String, "/bt_status", 10)

            def on_running(self):
                self._pub.publish(String(data="running"))
                return NodeStatus.SUCCESS

        class ObstacleCheck(RosTopicMixin, ConditionNode):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self._latest = None

            def setup(self):
                self._sub = self.create_subscription(
                    LaserScan, "/scan", lambda m: setattr(self, "_latest", m), 10
                )

            def tick(self):
                if self._latest is None:
                    return NodeStatus.FAILURE
                return NodeStatus.SUCCESS if min(self._latest.ranges) > 0.5 else NodeStatus.FAILURE

    Sensor topics (LaserScan, PointCloud2, Image, etc.) are often published
    with BEST_EFFORT QoS. Pass a QoSProfile instead of an integer to match:

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        SENSOR_QOS = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, SENSOR_QOS)
    """

    def create_publisher(self, msg_type, topic: str, qos=10):
        """Create a publisher. qos accepts an integer depth or a QoSProfile."""
        return self._require_ros_node().create_publisher(msg_type, topic, qos)

    def create_subscription(self, msg_type, topic: str, callback, qos=10):
        """Create a subscription. qos accepts an integer depth or a QoSProfile."""
        return self._require_ros_node().create_subscription(msg_type, topic, callback, qos)

    def ros_logger(self):
        return self._require_ros_node().get_logger()
