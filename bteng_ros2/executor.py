"""RosBTExecutor -- rclpy.Node that drives a BTEng tree via a ROS 2 timer."""

from __future__ import annotations

import contextlib
import time

from bteng import TreeExecutor, ExecutorConfig, NodeStatus
from bteng.core.tree import Tree


def _missing_rclpy(symbol: str, needed_by: str) -> type:
    """Build a stand-in for an rclpy symbol that is not importable.

    ``import bteng_ros2`` must succeed with no ROS 2 install so that CLIs can
    serve ``--help``/``--dry-run`` and unit tests can run off-robot.  The
    executors, though, subclass ``rclpy.node.Node`` /
    ``rclpy.lifecycle.LifecycleNode``, and a ``class`` statement needs its base
    at module-import time.  Subclassing this placeholder keeps the module
    importable and the names bound; constructing the result raises ImportError
    with the fix, rather than half-working or blowing up somewhere obscure.

    Never used when rclpy is importable — the real base is then used verbatim,
    so nothing about the ROS path changes.
    """
    message = (
        f"{needed_by} requires rclpy ({symbol}), which is not importable. "
        f"`import bteng_ros2` works without ROS 2 so that CLIs and unit tests "
        f"can run off-robot, but {needed_by} is a real ROS 2 node — source a "
        f"ROS 2 environment (e.g. `source /opt/ros/jazzy/setup.bash`) first."
    )

    class _Meta(type):
        def __getattr__(cls, attr: str):
            # Dunders must keep raising AttributeError: copy, pickle, inspect
            # and typing all probe for them and expect that answer.
            if attr.startswith("__") and attr.endswith("__"):
                raise AttributeError(attr)
            raise ImportError(message)

    class _MissingRclpySymbol(metaclass=_Meta):
        def __init__(self, *args, **kwargs) -> None:
            raise ImportError(message)

    name = symbol.rsplit(".", 1)[-1]
    _MissingRclpySymbol.__name__ = name
    _MissingRclpySymbol.__qualname__ = name
    return _MissingRclpySymbol


try:
    import rclpy
    from rclpy.node import Node

    #: True when rclpy imported. Public: branch on it if your program needs to
    #: know whether the executors below can actually be constructed.
    RCLPY_AVAILABLE = True
except ImportError:  # No ROS 2 on this interpreter — stay importable.
    rclpy = None
    Node = _missing_rclpy("rclpy.node.Node", "RosBTExecutor")
    RCLPY_AVAILABLE = False


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

    Injects a rclpy node into all ROS-aware nodes in the tree at set_tree()
    time, so tree nodes can be constructed without a ros_node reference.
    That is ``self`` by default, or ``ros_node`` when one is supplied — pass
    the same node the tree nodes already use so all ROS traffic is served by
    a single node.

        rclpy.init()

        tree = build_my_tree()
        bt = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05))
        status = bt.run(timeout=60.0)

        rclpy.shutdown()

    run() never calls rclpy.init()/rclpy.shutdown() — process-wide context
    setup and teardown stay with the caller.
    """

    #: Seconds to keep spinning after a halt so queued goal cancellations are
    #: actually transmitted. 0 disables the drain.
    cancel_grace: float = 0.5

    def __init__(
        self,
        tree: Tree,
        config: ExecutorConfig | None = None,
        node_name: str = "bt_executor",
        ros_node: Node | None = None,
    ) -> None:
        super().__init__(node_name)
        cfg = config or ExecutorConfig()
        self._cfg = cfg
        # set_tree() below does the injecting, so this must exist first.
        self._ros_node = ros_node
        self._final_status: NodeStatus | None = None
        self._halted = False
        self._spinning = False
        self._bt = TreeExecutor(cfg)
        self.set_tree(tree)
        self._timer = self.create_timer(cfg.tick_interval, self._tick)

    def set_tree(self, tree: Tree) -> None:
        target = self._ros_node if self._ros_node is not None else self
        _inject_ros_node(tree, target)
        _wire_clocks(tree, target)
        self._bt.set_tree(tree)

    def run(self, timeout: float | None = None) -> NodeStatus:
        """Spin until the tree settles, then return the root's final status.

        Ticking is driven by the timer created in __init__; this only spins the
        ROS executor so timers and client callbacks fire. An external ros_node
        is spun too — otherwise its action/service responses are never
        delivered and every tree node stays RUNNING forever.

        timeout=None waits indefinitely. On expiry the tree is halted and
        FAILURE is returned. Does not touch rclpy.init()/rclpy.shutdown().

        halt() from another thread ends the run promptly with FAILURE, so a
        supervisor can cancel a tree without waiting out the timeout. That is
        the cooperative shape to use: the canceller calls halt(), the thread
        that owns the run gets the result.

        Only one run() may spin these nodes at a time; a concurrent call raises
        rather than letting rclpy reject the second registration deep inside.

        After a halt or a timeout the spin continues for cancel_grace seconds so
        queued cancellations actually leave the process -- see _drain_cancels().
        """
        # Imported here, not at module scope, so importing this module stays
        # possible under the mocked rclpy used by the test suites downstream.
        from rclpy.executors import SingleThreadedExecutor

        if self._spinning:
            raise RuntimeError(
                f"{type(self).__name__}.run() is already spinning these nodes; "
                "rclpy allows only one executor per node"
            )
        if not self._is_tickable():
            return self._final_status if self._final_status is not None else NodeStatus.FAILURE
        self._final_status = None
        self._spinning = True

        spin = SingleThreadedExecutor()
        spin.add_node(self)
        if self._ros_node is not None:
            spin.add_node(self._ros_node)
        # Bound each spin_once so the deadline stays responsive even when no
        # callback is pending, and so a slow tick_interval cannot stretch the
        # timeout by a whole tick period.
        slice_sec = min(max(self._cfg.tick_interval, 0.001), 0.05)
        deadline = None if timeout is None else time.monotonic() + timeout
        try:
            while self._final_status is None:
                if self._halted:
                    # Somebody cancelled us — the tree is already halted, so no
                    # tick will ever settle it and spinning on would just burn
                    # the whole timeout.
                    self._drain_cancels(spin)
                    self.get_logger().info("BT run halted")
                    return NodeStatus.FAILURE
                if not rclpy.ok():
                    break
                if deadline is not None and time.monotonic() >= deadline:
                    self.halt()
                    self._drain_cancels(spin)
                    self.get_logger().error(f"BT run timed out after {timeout}s")
                    return NodeStatus.FAILURE
                spin.spin_once(timeout_sec=slice_sec)
        finally:
            self._spinning = False
            with contextlib.suppress(Exception):
                spin.remove_node(self)
            if self._ros_node is not None:
                with contextlib.suppress(Exception):
                    spin.remove_node(self._ros_node)
            with contextlib.suppress(Exception):
                spin.shutdown()

        return self._final_status if self._final_status is not None else NodeStatus.FAILURE

    def _drain_cancels(self, spin) -> None:
        """Spin briefly after a halt so queued cancellations are transmitted.

        halt() reaches every RUNNING node's on_halted(), and for an action node
        that means cancel_goal_async() -- which only *queues* the request. Return
        the moment the halt flag is seen and nothing ever spins to send it: the
        tree stops ticking while the robot keeps executing the goal. Draining is
        bounded so a halt still returns promptly.
        """
        if self.cancel_grace <= 0:
            return
        end = time.monotonic() + self.cancel_grace
        while time.monotonic() < end:
            if not rclpy.ok():
                return
            with contextlib.suppress(Exception):
                spin.spin_once(timeout_sec=0.01)

    def _is_tickable(self) -> bool:
        """False once the tree settled or was halted — the tick timer is
        cancelled in both cases, so spinning again would hang forever.

        Tracked with our own flag rather than by asking the timer: a halted
        executor must stay unrunnable even where the timer object is a stub.
        """
        return self._final_status is None and not self._halted

    def _tick(self) -> None:
        status = self._bt.tick_once()
        if status != NodeStatus.RUNNING:
            if not isinstance(status, NodeStatus):
                # Belt-and-braces on bteng >= 0.3.1: the core's execute_tick()
                # now rejects a non-NodeStatus at the offending node, so a real
                # tree raises there first and never reaches this branch. It is
                # kept because it still catches a stubbed executor or a patched
                # tick_once(), and because the failure it guards is nasty —
                # formatting a None terminal result raises inside a timer
                # callback, where rclpy swallows the traceback and the run
                # hangs instead of failing.
                self.get_logger().error(
                    f"BT root returned {status!r}, not a NodeStatus — treating as FAILURE"
                )
                status = NodeStatus.FAILURE
            self._timer.cancel()
            self._bt.halt_tree()
            self._final_status = status
            self.get_logger().info(f"BT completed: {status.value}")

    def halt(self) -> None:
        self._timer.cancel()
        self._bt.halt_tree()
        self._halted = True

    @property
    def final_status(self) -> NodeStatus | None:
        """Status the tree settled at, or None if it has not settled yet."""
        return self._final_status

    @property
    def bt_executor(self) -> TreeExecutor:
        return self._bt
