"""RosConditionNode -- BTEng condition driven by a ROS 2 topic."""

from __future__ import annotations

from bteng import ConditionNode, NodeStatus
from bteng_ros2._endpoint import declare_endpoint_port, resolve_endpoint
from bteng_ros2._topic import RosTopicMixin


class RosConditionNode(RosTopicMixin, ConditionNode):
    """Base class for BTEng conditions that evaluate the latest message on a topic.

    Declare topic_type, topic_name, and optionally topic_qos as class attributes,
    then implement evaluate(msg). The subscription is created lazily on first tick.

        class ObstacleDetected(RosConditionNode):
            topic_type = LaserScan
            topic_name = "/scan"

            def evaluate(self, msg: LaserScan) -> bool:
                return min(msg.ranges) < 0.5

    Sensor topics (LaserScan, PointCloud2, Image) are typically published with
    BEST_EFFORT QoS. Set topic_qos to match the publisher, otherwise no messages
    will be received:

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        class ObstacleDetected(RosConditionNode):
            topic_type = LaserScan
            topic_name = "/scan"
            topic_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10,
            )

            def evaluate(self, msg: LaserScan) -> bool:
                return min(msg.ranges) < 0.5

    Returns FAILURE when no message has been received yet.
    Returns SUCCESS / FAILURE based on evaluate() thereafter.

    Need more control (e.g. multiple topics)?
    Combine RosTopicMixin directly on ConditionNode:

        class BothSensorsOk(RosTopicMixin, ConditionNode):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self._scan = None
                self._imu = None

            def setup(self):
                self.create_subscription(LaserScan, "/scan",
                    lambda m: setattr(self, "_scan", m), 10)
                self.create_subscription(Imu, "/imu",
                    lambda m: setattr(self, "_imu", m), 10)

            def tick(self):
                if self._scan is None or self._imu is None:
                    return NodeStatus.FAILURE
                return NodeStatus.SUCCESS if self._ok() else NodeStatus.FAILURE
    """

    topic_type = None
    topic_name: str = ""
    topic_qos = 10

    def __init_subclass__(cls, **kwargs) -> None:
        # See RosActionNode: the endpoint becomes settable from a tree, so a
        # condition can be pointed at a second sensor without a subclass.
        super().__init_subclass__(**kwargs)
        declare_endpoint_port(cls, "topic_name")

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._latest_msg = None
        self._subscription = None

    def tick(self) -> NodeStatus:
        if self._subscription is None:
            if self.topic_type is None:
                raise RuntimeError(f"{type(self).__name__}.topic_type is not set")
            resolve_endpoint(self, "topic_name")
            if not self.topic_name:
                raise RuntimeError(f"{type(self).__name__}.topic_name is not set")
            self._subscription = self.create_subscription(
                self.topic_type, self.topic_name, self._on_msg, self.topic_qos
            )
        if self._latest_msg is None:
            return NodeStatus.FAILURE
        return NodeStatus.SUCCESS if self.evaluate(self._latest_msg) else NodeStatus.FAILURE

    def _on_msg(self, msg) -> None:
        self._latest_msg = msg

    def evaluate(self, msg) -> bool:
        raise NotImplementedError(f"{type(self).__name__}.evaluate() not implemented")
