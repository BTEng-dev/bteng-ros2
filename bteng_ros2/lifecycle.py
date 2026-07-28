"""LifecycleBTExecutor — lifecycle-managed rclpy node for BTEng trees."""

from __future__ import annotations

from bteng import TreeExecutor, ExecutorConfig, NodeStatus
from bteng_ros2.executor import _inject_ros_node, _missing_rclpy, _wire_clocks

try:
    from rclpy.lifecycle import LifecycleNode
    from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
except ImportError:  # No ROS 2 on this interpreter — stay importable.
    # See _missing_rclpy(): the class statement below needs a base, and every
    # use of LifecycleState / TransitionCallbackReturn is inside a transition
    # callback, which only a constructed instance can reach.
    LifecycleNode = _missing_rclpy(
        "rclpy.lifecycle.LifecycleNode", "LifecycleBTExecutor"
    )
    LifecycleState = _missing_rclpy(
        "rclpy.lifecycle.node.LifecycleState", "LifecycleBTExecutor"
    )
    TransitionCallbackReturn = _missing_rclpy(
        "rclpy.lifecycle.node.TransitionCallbackReturn", "LifecycleBTExecutor"
    )


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
        self._final_status: NodeStatus | None = None

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
        # A fresh activation is a fresh run, so last time's result must not
        # linger and look like this run's outcome.
        self._final_status = None
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
            if not isinstance(status, NodeStatus):
                # Belt-and-braces on bteng >= 0.3.1, exactly as in
                # RosBTExecutor._tick: the core rejects a non-NodeStatus at the
                # offending node now, so a real tree raises before reaching
                # here. Kept for stubbed executors and patched tick_once(), and
                # because formatting a None terminal result would raise inside a
                # timer callback, where rclpy swallows the traceback and the
                # node looks alive while the tree is dead.
                self.get_logger().error(
                    f"BT root returned {status!r}, not a NodeStatus — treating as FAILURE"
                )
                status = NodeStatus.FAILURE
            self._timer.cancel()
            self._bt.halt_tree()
            self._final_status = status
            self.get_logger().info(f"BT completed: {status.value}")

    def halt(self) -> None:
        """Stop ticking and halt the tree without a lifecycle transition."""
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        if self._bt is not None:
            self._bt.halt_tree()

    @property
    def final_status(self) -> NodeStatus | None:
        """Status the tree settled at, or None if it has not settled.

        Reset on every activate, since one lifecycle node can run a tree again
        after deactivate/cleanup/configure.
        """
        return self._final_status
