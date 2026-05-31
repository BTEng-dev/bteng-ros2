"""Mock rclpy and related ROS 2 packages so tests run without a ROS 2 install."""

import sys
import types
from unittest.mock import MagicMock


def _mock_rclpy() -> None:
    if "rclpy" in sys.modules:
        return

    class _Node:
        def __init__(self, name: str) -> None:
            self._name = name

        def get_name(self) -> str:
            return self._name

        def get_logger(self):
            return MagicMock()

        def get_clock(self):
            clk = MagicMock()
            clk.now.return_value = MagicMock(nanoseconds=0)
            return clk

        def create_timer(self, period, cb):
            t = MagicMock()
            t.cancel = MagicMock()
            return t

        def create_publisher(self, msg_type, topic, qos=10):
            return MagicMock()

        def create_subscription(self, msg_type, topic, callback, qos=10):
            return MagicMock()

        def create_client(self, srv_type, srv_name):
            return MagicMock()

    class _LifecycleNode(_Node):
        pass

    class _TransitionCallbackReturn:
        SUCCESS = "success"
        FAILURE = "failure"

    rclpy_mod = types.ModuleType("rclpy")
    node_mod = types.ModuleType("rclpy.node")
    action_mod = types.ModuleType("rclpy.action")
    lifecycle_mod = types.ModuleType("rclpy.lifecycle")
    lifecycle_node_mod = types.ModuleType("rclpy.lifecycle.node")

    node_mod.Node = _Node
    action_mod.ActionClient = MagicMock
    lifecycle_mod.LifecycleNode = _LifecycleNode
    lifecycle_node_mod.LifecycleState = MagicMock
    lifecycle_node_mod.TransitionCallbackReturn = _TransitionCallbackReturn

    rclpy_mod.node = node_mod
    rclpy_mod.action = action_mod
    rclpy_mod.lifecycle = lifecycle_mod
    rclpy_mod.init = MagicMock()
    rclpy_mod.shutdown = MagicMock()

    sys.modules.update({
        "rclpy": rclpy_mod,
        "rclpy.node": node_mod,
        "rclpy.action": action_mod,
        "rclpy.lifecycle": lifecycle_mod,
        "rclpy.lifecycle.node": lifecycle_node_mod,
    })

    class _GoalStatus:
        STATUS_SUCCEEDED = 4
        STATUS_ABORTED = 6
        STATUS_CANCELED = 5

    action_msgs_mod = types.ModuleType("action_msgs")
    action_msgs_msg_mod = types.ModuleType("action_msgs.msg")
    action_msgs_msg_mod.GoalStatus = _GoalStatus
    action_msgs_mod.msg = action_msgs_msg_mod
    sys.modules["action_msgs"] = action_msgs_mod
    sys.modules["action_msgs.msg"] = action_msgs_msg_mod


_mock_rclpy()
