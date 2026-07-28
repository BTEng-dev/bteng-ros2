"""Example 4 — LifecycleBTExecutor: production-grade lifecycle management.

Use LifecycleBTExecutor when you need proper ROS 2 lifecycle control:
  - ros2 lifecycle set /bt_executor configure
  - ros2 lifecycle set /bt_executor activate
  - ros2 lifecycle set /bt_executor deactivate
  - ros2 lifecycle set /bt_executor cleanup

The tree is built only on configure (not at startup), so you can reconfigure
without restarting the process. The tick timer only runs while active.

Run:

    python3 examples/04_lifecycle_executor.py

Then from another terminal:
    ros2 lifecycle set /bt_executor configure
    ros2 lifecycle set /bt_executor activate
"""

import rclpy
from bteng import SequenceNode, Tree, TreeMetadata, ExecutorConfig, NodeStatus
from bteng_ros2 import RosActionNode, RosConditionNode
from bteng_ros2.lifecycle import LifecycleBTExecutor


# ── Stub types ────────────────────────────────────────────────────────────────

class _LaserScan:
    def __init__(self, ranges):
        self.ranges = ranges

class _NavigateToPose:
    class Goal:
        pass


# ── Nodes ─────────────────────────────────────────────────────────────────────

class PathIsClear(RosConditionNode):
    topic_type = _LaserScan
    topic_name = "/scan"

    def evaluate(self, msg) -> bool:
        return min(msg.ranges) > 0.5


class GoToGoal(RosActionNode):
    action_type = _NavigateToPose
    action_name = "/navigate_to_pose"

    def make_goal(self):
        return _NavigateToPose.Goal()

    # ros_logger() lives on RosTopicMixin, which RosActionNode does not include —
    # reach the injected node directly instead.
    def on_success(self):
        self._require_ros_node().get_logger().info("Navigation complete.")

    def on_failure(self):
        self._require_ros_node().get_logger().warn("Navigation failed.")


# ── Lifecycle executor ────────────────────────────────────────────────────────

class RobotBT(LifecycleBTExecutor):
    """Robot behavior tree — managed via ROS 2 lifecycle transitions."""

    def build_tree(self) -> Tree:
        root = SequenceNode("root", children=[
            PathIsClear("check_path"),
            GoToGoal("navigate"),
        ])
        return Tree(TreeMetadata(id="robot_bt"), root)


def main():
    rclpy.init()
    node = RobotBT(
        config=ExecutorConfig(tick_interval=0.05),
        node_name="bt_executor",
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
