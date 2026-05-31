"""RosNodeMixin — foundation for all bteng-ros2 nodes."""

from __future__ import annotations


class RosNodeMixin:
    """Injects a rclpy.Node reference into any BTEng node.

    Supports deferred injection: construct without ros_node and let
    RosBTExecutor inject itself when set_tree() is called.

        node = MyNode("name")               # no ros_node yet
        bt = RosBTExecutor(tree)            # injects itself into all nodes

    Or inject at construction time:

        node = MyNode("name", ros_node=my_node)

    All capability mixins (RosActionClientMixin, RosTopicMixin, etc.)
    inherit from this class — ros_node is set once and shared.
    """

    def __init__(self, *args, ros_node=None, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._ros_node = ros_node

    def set_ros_node(self, node) -> None:
        self._ros_node = node

    def _require_ros_node(self):
        if self._ros_node is None:
            raise RuntimeError(
                f"{type(self).__name__}: ros_node is not set. "
                "Pass ros_node= at construction or use RosBTExecutor which "
                "injects itself automatically."
            )
        return self._ros_node
