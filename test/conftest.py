"""Mock rclpy and related ROS 2 packages so tests run without a ROS 2 install."""

import sys
import types
from unittest.mock import MagicMock


def _mock_rclpy() -> None:
    if "rclpy" in sys.modules:
        return

    class _Timer:
        """Timer stub with real cancel semantics, so a spin can fire it."""

        def __init__(self, period, callback) -> None:
            self.period = period
            self.callback = callback
            self._canceled = False

        def cancel(self) -> None:
            self._canceled = True

        def is_canceled(self) -> bool:
            return self._canceled

        def fire(self) -> None:
            if not self._canceled:
                self.callback()

    class _Node:
        def __init__(self, name: str, **kwargs) -> None:
            self._name = name
            self.timers: list = []

        def get_name(self) -> str:
            return self._name

        def get_logger(self):
            return MagicMock()

        def get_clock(self):
            clk = MagicMock()
            clk.now.return_value = MagicMock(nanoseconds=0)
            return clk

        def create_timer(self, period, cb):
            t = _Timer(period, cb)
            self.timers.append(t)
            return t

        def destroy_timer(self, timer) -> None:
            timer.cancel()
            if timer in self.timers:
                self.timers.remove(timer)

        def destroy_node(self) -> None:
            self.timers.clear()

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

    class _SingleThreadedExecutor:
        """Spin stub: one spin_once fires every live timer on every added node.

        Enough to drive RosBTExecutor.run() deterministically — the real
        executor's callback ordering is not what these tests are about.
        """

        def __init__(self) -> None:
            self.nodes: list = []
            self.spins = 0

        def add_node(self, node) -> None:
            if node in self.nodes:
                raise ValueError(f"node {node} already added to an executor")
            self.nodes.append(node)

        def remove_node(self, node) -> None:
            if node in self.nodes:
                self.nodes.remove(node)

        def spin_once(self, timeout_sec=None) -> None:
            self.spins += 1
            for node in list(self.nodes):
                for timer in list(getattr(node, "timers", [])):
                    timer.fire()

        def shutdown(self) -> None:
            self.nodes.clear()

    rclpy_mod = types.ModuleType("rclpy")
    node_mod = types.ModuleType("rclpy.node")
    executors_mod = types.ModuleType("rclpy.executors")
    action_mod = types.ModuleType("rclpy.action")
    lifecycle_mod = types.ModuleType("rclpy.lifecycle")
    lifecycle_node_mod = types.ModuleType("rclpy.lifecycle.node")

    node_mod.Node = _Node
    executors_mod.SingleThreadedExecutor = _SingleThreadedExecutor
    executors_mod.MultiThreadedExecutor = _SingleThreadedExecutor
    action_mod.ActionClient = MagicMock
    lifecycle_mod.LifecycleNode = _LifecycleNode
    lifecycle_node_mod.LifecycleState = MagicMock
    lifecycle_node_mod.TransitionCallbackReturn = _TransitionCallbackReturn

    rclpy_mod.node = node_mod
    rclpy_mod.executors = executors_mod
    rclpy_mod.action = action_mod
    rclpy_mod.lifecycle = lifecycle_mod
    rclpy_mod.init = MagicMock()
    rclpy_mod.shutdown = MagicMock()
    rclpy_mod.ok = lambda *a, **kw: True
    rclpy_mod.spin = MagicMock()
    rclpy_mod.spin_once = MagicMock()

    sys.modules.update({
        "rclpy": rclpy_mod,
        "rclpy.node": node_mod,
        "rclpy.executors": executors_mod,
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
