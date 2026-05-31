"""Example 2 — RosConditionNode: obstacle detection from a laser scan.

A condition node that subscribes to a topic and evaluates each message.
No boilerplate — just declare the topic and implement evaluate().

Run:

    python3 examples/02_condition_node.py
"""

import rclpy
from bteng import SequenceNode, Tree, TreeMetadata, ExecutorConfig, NodeStatus
from bteng_ros2 import RosConditionNode, RosActionNode
from bteng_ros2.executor import RosBTExecutor


# ── Stub types (replace with real ROS 2 message types) ───────────────────────

class _LaserScan:
    def __init__(self, ranges):
        self.ranges = ranges

class _NavigateToPose:
    class Goal:
        pass


# ── Condition: path is clear ──────────────────────────────────────────────────

class PathIsClear(RosConditionNode):
    """Returns SUCCESS when the nearest obstacle is more than 0.5 m away."""

    topic_type = _LaserScan        # replace with sensor_msgs.msg.LaserScan
    topic_name = "/scan"

    def evaluate(self, msg: _LaserScan) -> bool:
        return min(msg.ranges) > 0.5


# ── Action: navigate ──────────────────────────────────────────────────────────

class GoToGoal(RosActionNode):
    action_type = _NavigateToPose
    action_name = "/navigate_to_pose"

    def make_goal(self):
        return _NavigateToPose.Goal()


# ── Tree: only navigate when the path is clear ────────────────────────────────
#
#   Sequence
#   ├── PathIsClear     ← ticked first; returns FAILURE if scan says obstacle
#   └── GoToGoal        ← only reached when path is clear

def build_tree():
    root = SequenceNode("root", children=[
        PathIsClear("obstacle_check"),
        GoToGoal("navigate"),
    ])
    return Tree(TreeMetadata(id="guarded_nav"), root)


def main():
    rclpy.init()
    tree = build_tree()
    bt = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05))
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
