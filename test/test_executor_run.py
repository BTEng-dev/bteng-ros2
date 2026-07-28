"""Tests for RosBTExecutor.run() and the ros_node= injection target.

Both APIs are what the docstring and every downstream example advertise, so
they are covered here directly rather than through a node-level test.
"""

import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

from bteng import ActionNode, ExecutorConfig, NodeStatus, SequenceNode
from bteng.core.tree import Tree, TreeMetadata
from bteng_ros2._mixin import RosNodeMixin
from bteng_ros2.executor import RosBTExecutor
from bteng_ros2.testing import FakeRosNode


class _SucceedAfter(ActionNode):
    """SUCCESS on the nth tick, RUNNING before that."""

    def __init__(self, name, ticks=1, **kw):
        super().__init__(name, **kw)
        self.ticks = ticks
        self.seen = 0

    def tick(self):
        self.seen += 1
        return NodeStatus.SUCCESS if self.seen >= self.ticks else NodeStatus.RUNNING


class _Fail(ActionNode):
    def tick(self):
        return NodeStatus.FAILURE


class _Forever(ActionNode):
    def __init__(self, name, **kw):
        super().__init__(name, **kw)
        self.halted = 0

    def tick(self):
        return NodeStatus.RUNNING

    def halt(self):
        self.halted += 1
        super().halt()


class _RosAware(RosNodeMixin, ActionNode):
    def tick(self):
        return NodeStatus.SUCCESS


def _tree(root):
    return Tree(TreeMetadata(id="t"), root)


def _executor(root, ros_node=None, tick_interval=0.001):
    return RosBTExecutor(
        _tree(root),
        ExecutorConfig(tick_interval=tick_interval),
        node_name="bt_test",
        ros_node=ros_node,
    )


# ── run() result ────────────────────────────────────────────────────────────────

def test_run_returns_success_when_tree_settles():
    bt = _executor(_SucceedAfter("a"))
    assert bt.run(timeout=5.0) == NodeStatus.SUCCESS


def test_run_returns_failure_status_from_tree():
    bt = _executor(_Fail("a"))
    assert bt.run(timeout=5.0) == NodeStatus.FAILURE


def test_run_keeps_spinning_until_a_later_tick_settles():
    node = _SucceedAfter("a", ticks=4)
    bt = _executor(node)
    assert bt.run(timeout=5.0) == NodeStatus.SUCCESS
    assert node.seen >= 4


def test_run_drives_a_whole_sequence():
    first, second = _SucceedAfter("first", ticks=2), _SucceedAfter("second", ticks=2)
    bt = _executor(SequenceNode("root", children=[first, second]))
    assert bt.run(timeout=5.0) == NodeStatus.SUCCESS
    assert first.seen >= 2 and second.seen >= 2


def test_final_status_is_none_before_run_and_set_after():
    bt = _executor(_SucceedAfter("a"))
    assert bt.final_status is None
    bt.run(timeout=5.0)
    assert bt.final_status == NodeStatus.SUCCESS


# ── timeout ─────────────────────────────────────────────────────────────────────

def test_run_times_out_and_halts_the_tree():
    node = _Forever("a")
    bt = _executor(node)
    assert bt.run(timeout=0.05) == NodeStatus.FAILURE
    assert node.halted >= 1


def test_timed_out_executor_is_not_runnable_again():
    bt = _executor(_Forever("a"))
    bt.run(timeout=0.05)
    # Must not hang: the tick timer is cancelled, so nothing could ever settle.
    assert bt.run(timeout=0.05) == NodeStatus.FAILURE


# ── re-entry ────────────────────────────────────────────────────────────────────

def test_second_run_on_settled_tree_returns_recorded_status():
    bt = _executor(_SucceedAfter("a"))
    assert bt.run(timeout=5.0) == NodeStatus.SUCCESS
    assert bt.run(timeout=5.0) == NodeStatus.SUCCESS


def test_run_after_halt_returns_failure_without_spinning(monkeypatch):
    bt = _executor(_Forever("a"))
    bt.halt()
    spun = []
    monkeypatch.setattr(SingleThreadedExecutor, "spin_once",
                        lambda self, timeout_sec=None: spun.append(1))
    assert bt.run(timeout=0.05) == NodeStatus.FAILURE
    assert spun == []


def test_run_stops_when_context_is_shut_down(monkeypatch):
    bt = _executor(_Forever("a"))
    monkeypatch.setattr(rclpy, "ok", lambda *a, **kw: False)
    assert bt.run(timeout=5.0) == NodeStatus.FAILURE


# ── ros_node= injection target ──────────────────────────────────────────────────

def test_default_injects_the_executor_itself():
    node = _RosAware("a")
    bt = _executor(node)
    assert node._ros_node is bt


def test_external_ros_node_is_injected_instead_of_the_executor():
    node = _RosAware("a")
    fake = FakeRosNode("shared")
    bt = _executor(node, ros_node=fake)
    assert node._ros_node is fake
    assert node._ros_node is not bt


def test_preexisting_ros_node_is_never_overwritten():
    own = FakeRosNode("own")
    node = _RosAware("a", ros_node=own)
    _executor(node, ros_node=FakeRosNode("shared"))
    assert node._ros_node is own


def test_external_ros_node_is_spun_too(monkeypatch):
    """Its client callbacks would never fire otherwise, so every node would
    stay RUNNING forever."""
    added = []
    real_add = SingleThreadedExecutor.add_node

    def spy(self, node):
        added.append(node)
        return real_add(self, node)

    monkeypatch.setattr(SingleThreadedExecutor, "add_node", spy)
    fake = FakeRosNode("shared")
    bt = _executor(_SucceedAfter("a"), ros_node=fake)
    bt.run(timeout=5.0)
    assert bt in added and fake in added


def test_only_the_executor_is_spun_without_an_external_node(monkeypatch):
    added = []
    real_add = SingleThreadedExecutor.add_node
    monkeypatch.setattr(SingleThreadedExecutor, "add_node",
                        lambda self, node: (added.append(node), real_add(self, node))[1])
    bt = _executor(_SucceedAfter("a"))
    bt.run(timeout=5.0)
    assert added == [bt]


def test_spin_nodes_are_released_after_run(monkeypatch):
    """A node left registered with a dead executor cannot be added to the next
    one — rclpy rejects double registration."""
    instances = []
    real_init = SingleThreadedExecutor.__init__

    def track(self, *a, **kw):
        real_init(self, *a, **kw)
        instances.append(self)

    monkeypatch.setattr(SingleThreadedExecutor, "__init__", track)
    bt = _executor(_SucceedAfter("a"))
    bt.run(timeout=5.0)
    assert instances and instances[0].nodes == []


def test_a_none_root_status_is_recorded_as_failure_not_raised(monkeypatch):
    """A node whose tick() falls off the end yields None.

    bteng >= 0.3.0 raises TypeError at the offending node, so the None can no
    longer reach us through a real tree — but this executor still supports older
    cores that pass it straight through, and formatting it would raise inside a
    timer callback where rclpy swallows the traceback. Feed the guard directly.
    """
    bt = _executor(_SucceedAfter("a"))
    monkeypatch.setattr(bt.bt_executor, "tick_once", lambda: None)
    bt._tick()
    assert bt.final_status == NodeStatus.FAILURE


# ── cooperative cancel ──────────────────────────────────────────────────────────

def test_halt_from_another_thread_ends_the_run_promptly():
    """A supervisor cancelling a tree must not have to wait out the timeout."""
    import threading

    bt = _executor(_Forever("a"))
    threading.Timer(0.05, bt.halt).start()
    t0 = time.monotonic()
    assert bt.run(timeout=5.0) == NodeStatus.FAILURE
    assert time.monotonic() - t0 < 1.0


def test_concurrent_run_raises_instead_of_confusing_rclpy():
    """rclpy allows one executor per node; say so plainly rather than letting
    the second add_node() fail deep inside."""
    bt = _executor(_Forever("a"))
    errors = []

    def second_run():
        try:
            bt.run(timeout=0.5)
        except RuntimeError as exc:
            errors.append(str(exc))

    import threading

    watcher = threading.Timer(0.05, second_run)
    watcher.start()
    threading.Timer(0.30, bt.halt).start()
    bt.run(timeout=5.0)
    watcher.join()
    assert errors and "already spinning" in errors[0]


def test_spinning_flag_is_cleared_so_a_later_run_is_allowed():
    bt = _executor(_SucceedAfter("a"))
    bt.run(timeout=5.0)
    assert bt._spinning is False


# ── cancel drain ────────────────────────────────────────────────────────────────

class _CancelsOnHalt(ActionNode):
    """Stands in for an action node: halting queues a cancellation that only
    leaves the process if something keeps spinning."""

    def __init__(self, name, **kw):
        super().__init__(name, **kw)
        self.cancel_queued = False
        self.spins_after_cancel = 0

    def tick(self):
        return NodeStatus.RUNNING

    def halt(self):
        self.cancel_queued = True
        super().halt()


def test_halt_keeps_spinning_so_a_queued_cancel_is_transmitted(monkeypatch):
    """Without the drain, run() returned the instant it saw the halt flag and the
    cancel_goal_async() queued by on_halted() was never sent — the tree stopped
    ticking while the robot kept driving."""
    node = _CancelsOnHalt("a")
    bt = _executor(node)
    bt.cancel_grace = 0.1

    spins = []
    real_spin = SingleThreadedExecutor.spin_once

    def counting_spin(self, timeout_sec=None):
        if node.cancel_queued:
            spins.append(1)
        return real_spin(self, timeout_sec=timeout_sec)

    monkeypatch.setattr(SingleThreadedExecutor, "spin_once", counting_spin)
    threading.Timer(0.05, bt.halt).start()

    assert bt.run(timeout=5.0) == NodeStatus.FAILURE
    assert node.cancel_queued
    assert spins, "run() returned without spinning once after the halt"


def test_cancel_grace_zero_returns_immediately():
    bt = _executor(_Forever("a"))
    bt.cancel_grace = 0.0
    bt.halt()
    t0 = time.monotonic()
    assert bt.run(timeout=1.0) == NodeStatus.FAILURE
    assert time.monotonic() - t0 < 0.5


def test_timeout_also_drains_cancels():
    node = _CancelsOnHalt("a")
    bt = _executor(node)
    bt.cancel_grace = 0.05
    t0 = time.monotonic()
    assert bt.run(timeout=0.1) == NodeStatus.FAILURE
    assert node.cancel_queued
    assert time.monotonic() - t0 >= 0.1
