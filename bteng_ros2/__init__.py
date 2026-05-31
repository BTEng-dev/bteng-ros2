"""bteng_ros2 -- ROS 2 base classes for the BTEng behavior tree engine."""

from bteng_ros2._mixin import RosNodeMixin
from bteng_ros2._action_client import RosActionClientMixin
from bteng_ros2._service_client import RosServiceClientMixin
from bteng_ros2._topic import RosTopicMixin
from bteng_ros2.nodes.action import RosActionNode
from bteng_ros2.nodes.stateful import RosStatefulActionNode
from bteng_ros2.nodes.condition import RosConditionNode
from bteng_ros2.nodes.service import RosServiceNode
from bteng_ros2.clock import RosClock
from bteng_ros2.executor import RosBTExecutor
from bteng_ros2.lifecycle import LifecycleBTExecutor

from importlib.metadata import version, PackageNotFoundError
try:
    __version__ = version("bteng-ros2")
except PackageNotFoundError:
    __version__ = "unknown"

__all__ = [
    # Mixins -- combine freely with any BTEng node type
    "RosNodeMixin",
    "RosActionClientMixin",
    "RosServiceClientMixin",
    "RosTopicMixin",
    # Pre-combined base classes -- subclass these for common patterns
    "RosActionNode",
    "RosStatefulActionNode",
    "RosConditionNode",
    "RosServiceNode",
    # Executors
    "RosBTExecutor",
    "LifecycleBTExecutor",
    # Utilities
    "RosClock",
    # Version
    "__version__",
]
