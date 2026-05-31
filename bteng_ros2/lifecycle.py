"""LifecycleBTExecutor — lifecycle-managed rclpy node for BTEng trees."""

from __future__ import annotations

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from bteng import TreeExecutor, ExecutorConfig, NodeStatus
from bteng_ros2.executor import _inject_ros_node, _wire_clocks


class LifecycleBTExecutor(LifecycleNode):
    """Lifecycle-managed rclpy node that drives a BTEng tree.

    Lifecycle transitions:
        configure  → calls build_tree(), validates, ready to activate
        activate   → starts the tick timer
        deactivate → stops the tick timer, halts the tree
        cleanup    → destroys the tree

    Subclass and implement build_tree():

        class RobotBT(LifecycleBTExecutor):
            def build_tree(self) -> Tree:
                nav  = NavigateToPose("nav")
                root = SequenceNode("root", children=[nav])
                return Tree(TreeMetadata(id="robot"), root)

        rclpy.init()
        node = RobotBT(ExecutorConfig(tick_interval=0.05))
        rclpy.spin(node)
        rclpy.shutdown()

    Nodes in the tree are injected with self (the lifecycle node) at
    configure time, so they can call ROS APIs without holding a reference
    at construction.
    """

    def __init__(
        self,
        config: ExecutorConfig | None = None,
        node_name: str = "bt_executor",
    ) -> None:
        super().__init__(node_name)
        self._cfg = config or ExecutorConfig()
        self._bt: TreeExecutor | None = None
        self._timer = None

    def build_tree(self):
        raise NotImplementedError(f"{type(self).__name__}.build_tree() not implemented")

    # ── Lifecycle transitions ─────────────────────────────────────────────────

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            tree = self.build_tree()
            _inject_ros_node(tree, self)
            _wire_clocks(tree, self)
            self._bt = TreeExecutor(self._cfg)
            self._bt.set_tree(tree)
        except Exception as exc:
            self.get_logger().error(f"Failed to build tree: {exc}")
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._timer = self.create_timer(self._cfg.tick_interval, self._tick)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        if self._bt is not None:
            self._bt.halt_tree()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self._bt is not None:
            self._bt.halt_tree()
        self._bt = None
        return TransitionCallbackReturn.SUCCESS

    # ── Tick ─────────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self._bt is None:
            return
        status = self._bt.tick_once()
        if status != NodeStatus.RUNNING:
            self._timer.cancel()
            self._bt.halt_tree()
            self.get_logger().info(f"BT completed: {status.value}")
