"""Example 1 — RosActionNode: navigate to a pose.

The simplest way to wrap a ROS 2 action server.
Declare action_type and action_name, implement make_goal(). Done.

Run (after sourcing ROS 2 and installing bteng + bteng-ros2):

    python3 examples/01_action_node.py
"""

import rclpy
from rclpy.action import ActionClient  # noqa: F401 — used by rclpy internally
from bteng import SequenceNode, Tree, TreeMetadata, ExecutorConfig
from bteng_ros2 import RosActionNode
from bteng_ros2.executor import RosBTExecutor

# ── Normally imported from your interface package ────────────────────────────
# from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import PoseStamped
# Here we use stubs so the example runs standalone.

class _GoalStub:
    pass

class _NavigateToPose:
    class Goal:
        def __init__(self):
            self.pose = None
    Goal = Goal


# ── Node definition ───────────────────────────────────────────────────────────

class GoToWaypoint(RosActionNode):
    """Navigate to a fixed waypoint using Nav2."""

    action_type = _NavigateToPose
    action_name = "/navigate_to_pose"

    def make_goal(self):
        goal = _NavigateToPose.Goal()
        # In a real node: goal.pose = self.blackboard.get("target_pose")
        goal.pose = "waypoint_A"
        return goal

    def on_success(self):
        print("Navigation succeeded!")

    def on_failure(self):
        print("Navigation failed or was rejected.")


# ── Tree + executor ───────────────────────────────────────────────────────────

def build_tree():
    root = SequenceNode("root", children=[
        GoToWaypoint("navigate"),
    ])
    return Tree(TreeMetadata(id="navigation"), root)


def main():
    rclpy.init()
    tree = build_tree()
    bt = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.1))
    try:
        rclpy.spin(bt)
    except KeyboardInterrupt:
        pass
    finally:
        bt.halt()
        bt.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
