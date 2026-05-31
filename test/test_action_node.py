import sys
import pytest
from bteng import NodeStatus
from bteng_ros2.nodes.action import RosActionNode
from bteng_ros2.testing import FakeRosNode
from test.helpers import FakeActionClient, SlowActionClient


_GOAL = object()


class _Nav(RosActionNode):
    action_type = object
    action_name = "/navigate"

    def make_goal(self):
        return _GOAL


class _NoType(RosActionNode):
    action_name = "/navigate"
    def make_goal(self): return _GOAL


class _NoName(RosActionNode):
    action_type = object
    def make_goal(self): return _GOAL


class _NoGoal(RosActionNode):
    action_type = object
    action_name = "/navigate"


def _use_fake_client(succeed=True, accepted=True):
    sys.modules["rclpy.action"].ActionClient = (
        lambda n, t, name: FakeActionClient(n, t, name, succeed=succeed, accepted=accepted)
    )


def test_raises_on_missing_action_type():
    fake = FakeRosNode()
    n = _NoType("n", ros_node=fake)
    with pytest.raises(RuntimeError, match="action_type is not set"):
        n.on_start()


def test_raises_on_missing_action_name():
    fake = FakeRosNode()
    n = _NoName("n", ros_node=fake)
    _use_fake_client()
    with pytest.raises(RuntimeError, match="action_name is not set"):
        n.on_start()


def test_raises_on_missing_make_goal():
    fake = FakeRosNode()
    n = _NoGoal("n", ros_node=fake)
    _use_fake_client()
    with pytest.raises(NotImplementedError):
        n.on_start()


def test_on_start_sends_goal():
    fake = FakeRosNode()
    n = _Nav("n", ros_node=fake)
    _use_fake_client()
    n.on_start()
    client = n._RosActionClientMixin__action_client
    assert client.goals_sent == [_GOAL]


def test_on_running_returns_success():
    fake = FakeRosNode()
    n = _Nav("n", ros_node=fake)
    _use_fake_client(succeed=True)
    n.on_start()
    assert n.on_running() == NodeStatus.SUCCESS


def test_on_running_returns_failure():
    fake = FakeRosNode()
    n = _Nav("n", ros_node=fake)
    _use_fake_client(succeed=False)
    n.on_start()
    assert n.on_running() == NodeStatus.FAILURE


def test_on_success_hook_called():
    called = []

    class _WithHook(_Nav):
        def on_success(self):
            called.append(True)

    fake = FakeRosNode()
    n = _WithHook("n", ros_node=fake)
    _use_fake_client(succeed=True)
    n.on_start()
    n.on_running()
    assert called == [True]


def test_on_failure_hook_called():
    called = []

    class _WithHook(_Nav):
        def on_failure(self):
            called.append(True)

    fake = FakeRosNode()
    n = _WithHook("n", ros_node=fake)
    _use_fake_client(succeed=False)
    n.on_start()
    n.on_running()
    assert called == [True]


def test_on_halted_cancels_goal():
    fake = FakeRosNode()
    sys.modules["rclpy.action"].ActionClient = (
        lambda rn, t, name: SlowActionClient(rn, t, name)
    )
    n = _Nav("n", ros_node=fake)
    n.on_start()
    # Resolve acceptance so goal_handle exists and state is RUNNING
    client = n._RosActionClientMixin__action_client
    client._pending.resolve()
    n.on_halted()
    assert client._pending._result.cancelled is True
