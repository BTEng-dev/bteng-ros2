"""RosBTExecutor -- rclpy.Node that drives a BTEng tree via a ROS 2 timer."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from bteng import TreeExecutor, ExecutorConfig, NodeStatus
from bteng.core.tree import Tree


def _inject_ros_node(tree: Tree, ros_node) -> None:
    """Walk tree and inject ros_node into any node with set_ros_node() and no node set yet."""
    def _walk(node):
        if hasattr(node, "set_ros_node") and getattr(node, "_ros_node", None) is None:
            node.set_ros_node(ros_node)
        for child in node.get_children():
            _walk(child)
    if tree is not None and tree.root is not None:
        _walk(tree.root)


def _wire_clocks(tree: Tree, ros_node) -> None:
    """Replace WallClock with RosClock on all Timeout/RateController nodes in the tree.

    Called automatically by set_tree() so that /use_sim_time=true is respected
    without any manual wiring. Preserves any user-supplied custom clock.
    """
    from bteng_ros2.clock import RosClock
    try:
        from bteng.concurrency.clock import WallClock
    except ImportError:
        return
    clock = RosClock(ros_node)

    def _walk(node):
        if hasattr(node, "_clock") and isinstance(node._clock, WallClock):
            node._clock = clock
        for child in node.get_children():
            _walk(child)

    if tree is not None and tree.root is not None:
        _walk(tree.root)


class RosBTExecutor(Node):
    """rclpy.Node that drives a BTEng TreeExecutor via a ROS 2 timer.

    Injects itself into all ROS-aware nodes in the tree at set_tree() time,
    so nodes can be constructed without a ros_node reference.

        rclpy.init()

        tree = build_my_tree()   # nodes constructed without ros_node
        bt = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05))
        rclpy.spin(bt)
        bt.halt()

        rclpy.shutdown()
    """

    def __init__(
        self,
        tree: Tree,
        config: ExecutorConfig | None = None,
        node_name: str = "bt_executor",
    ) -> None:
        super().__init__(node_name)
        cfg = config or ExecutorConfig()
        self._bt = TreeExecutor(cfg)
        self.set_tree(tree)
        self._timer = self.create_timer(cfg.tick_interval, self._tick)

    def set_tree(self, tree: Tree) -> None:
        _inject_ros_node(tree, self)
        _wire_clocks(tree, self)
        self._bt.set_tree(tree)

    def _tick(self) -> None:
        status = self._bt.tick_once()
        if status != NodeStatus.RUNNING:
            self._timer.cancel()
            self._bt.halt_tree()
            self.get_logger().info(f"BT completed: {status.value}")

    def halt(self) -> None:
        self._timer.cancel()
        self._bt.halt_tree()

    @property
    def bt_executor(self) -> TreeExecutor:
        return self._bt
