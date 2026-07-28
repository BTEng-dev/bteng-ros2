"""Tests for LifecycleBTExecutor: transitions, tick guard, final_status.

The lifecycle node stays spin-driven on purpose -- ticking is gated by external
transitions, so there is no run() here. What it does share with RosBTExecutor is
the tick callback, and that is where a None-returning node used to raise inside
a timer callback with rclpy swallowing the traceback.
"""

from rclpy.lifecycle.node import TransitionCallbackReturn

from bteng import ActionNode, ExecutorConfig, NodeStatus
from bteng.core.tree import Tree, TreeMetadata
from bteng_ros2.lifecycle import LifecycleBTExecutor


class _Ok(ActionNode):
    def tick(self):
        return NodeStatus.SUCCESS


class _Forever(ActionNode):
    def tick(self):
        return NodeStatus.RUNNING


def _node(root_cls, name="lc_test"):
    class _BT(LifecycleBTExecutor):
        def build_tree(self):
            return Tree(TreeMetadata(id="t"), root_cls("root"))

    return _BT(ExecutorConfig(tick_interval=0.001), node_name=name)


def _fire(node):
    """Fire every live timer once -- the stub timer records its callback."""
    for timer in list(getattr(node, "timers", [])):
        timer.fire()


def test_build_tree_must_be_implemented():
    node = LifecycleBTExecutor(ExecutorConfig(), node_name="bare")
    assert node.on_configure(None) == TransitionCallbackReturn.FAILURE


def test_configure_then_activate_then_tick_records_success():
    node = _node(_Ok)
    assert node.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert node.final_status is None
    assert node.on_activate(None) == TransitionCallbackReturn.SUCCESS
    _fire(node)
    assert node.final_status == NodeStatus.SUCCESS


def test_a_none_root_status_is_recorded_as_failure_not_raised(monkeypatch):
    """Same guard as RosBTExecutor: bteng >= 0.3.0 raises at the offending node,
    but an older core hands None through and formatting it inside a timer
    callback would be swallowed by rclpy. Feed the guard directly."""
    node = _node(_Ok)
    node.on_configure(None)
    node.on_activate(None)
    monkeypatch.setattr(node._bt, "tick_once", lambda: None)
    _fire(node)  # must not raise
    assert node.final_status == NodeStatus.FAILURE


def test_activate_clears_the_previous_result():
    node = _node(_Ok)
    node.on_configure(None)
    node.on_activate(None)
    _fire(node)
    assert node.final_status == NodeStatus.SUCCESS
    node.on_deactivate(None)
    node.on_activate(None)
    assert node.final_status is None, "last run's result leaked into a fresh activation"


def test_deactivate_stops_ticking():
    node = _node(_Forever)
    node.on_configure(None)
    node.on_activate(None)
    assert node.on_deactivate(None) == TransitionCallbackReturn.SUCCESS
    _fire(node)
    assert node.final_status is None


def test_halt_stops_ticking_without_a_transition():
    node = _node(_Forever)
    node.on_configure(None)
    node.on_activate(None)
    node.halt()
    _fire(node)
    assert node.final_status is None


def test_tick_before_configure_is_a_no_op():
    node = LifecycleBTExecutor(ExecutorConfig(), node_name="bare2")
    node._tick()  # no tree yet: must not raise
    assert node.final_status is None


def test_cleanup_drops_the_tree():
    node = _node(_Ok)
    node.on_configure(None)
    assert node.on_cleanup(None) == TransitionCallbackReturn.SUCCESS
    assert node._bt is None
