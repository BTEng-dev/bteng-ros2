"""Example 3 — RosStatefulActionNode: combine action + service + topic in one node.

When a single node needs to call an action server, call a service, AND publish
to a topic simultaneously, compose the mixins directly on StatefulActionNode
(or subclass RosStatefulActionNode which already bundles all three).

Pattern:
  on_start()   — initialise clients, send the goal
  on_running() — poll status, publish feedback
  on_halted()  — cancel the in-flight goal

Run:

    python3 examples/03_stateful_with_mixins.py
"""

import rclpy
from bteng import SequenceNode, Tree, TreeMetadata, ExecutorConfig, NodeStatus
from bteng_ros2 import RosStatefulActionNode
from bteng_ros2.executor import RosBTExecutor


# ── Stub interface types ──────────────────────────────────────────────────────

class _NavigateToPose:
    class Goal:
        pass

class _SetBool:
    class Request:
        def __init__(self, data: bool):
            self.data = data

class _String:
    def __init__(self, data: str):
        self.data = data


# ── Node: navigate, publish progress, enable motors via service ───────────────

class NavigateWithStatus(RosStatefulActionNode):
    """Navigate to a goal while publishing a status string and using a service."""

    def on_start(self) -> None:
        # Enable motors via a service call before starting navigation
        self._init_service_client(_SetBool, "/enable_motors")
        self.call_service(_SetBool.Request(data=True))

        # Create a publisher for status feedback
        self._status_pub = self.create_publisher(_String, "/bt_status", 10)

        # Send the navigation goal
        self._init_action_client(_NavigateToPose, "/navigate_to_pose")
        goal = _NavigateToPose.Goal()
        self.send_goal(goal)

    def on_running(self) -> NodeStatus:
        self._status_pub.publish(_String(data="navigating"))
        status = self.action_status()
        if status == NodeStatus.SUCCESS:
            self._status_pub.publish(_String(data="arrived"))
        elif status == NodeStatus.FAILURE:
            self._status_pub.publish(_String(data="nav_failed"))
        return status

    def on_halted(self) -> None:
        self.cancel_goal()
        self._status_pub.publish(_String(data="cancelled"))


# ── Tree ──────────────────────────────────────────────────────────────────────

def build_tree():
    root = SequenceNode("root", children=[
        NavigateWithStatus("nav_with_status"),
    ])
    return Tree(TreeMetadata(id="stateful_demo"), root)


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
