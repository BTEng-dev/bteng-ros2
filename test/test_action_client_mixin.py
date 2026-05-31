import sys
import pytest
from bteng import StatefulActionNode, NodeStatus
from bteng_ros2._action_client import RosActionClientMixin
from bteng_ros2.testing import FakeRosNode
from test.helpers import FakeActionClient, SlowActionClient, FakeGoalHandle, FakeFuture


class _ActNode(RosActionClientMixin, StatefulActionNode):
    def on_start(self): pass
    def on_running(self): return NodeStatus.RUNNING
    def on_halted(self): pass


_MOCK_TYPE = object


def _init(fake, client_cls=FakeActionClient, **kw):
    """Create node, init action client with the given fake client class."""
    node = _ActNode("n", ros_node=fake)
    sys.modules["rclpy.action"].ActionClient = lambda n, t, name: client_cls(n, t, name, **kw)
    node._init_action_client(_MOCK_TYPE, "/act")
    return node


def test_init_creates_action_client():
    fake = FakeRosNode()
    node = _init(fake)
    assert node._RosActionClientMixin__action_client is not None


def test_status_is_running_before_send():
    fake = FakeRosNode()
    node = _ActNode("n", ros_node=fake)
    sys.modules["rclpy.action"].ActionClient = lambda n, t, name: FakeActionClient(n, t, name)
    node._init_action_client(_MOCK_TYPE, "/act")
    assert node.action_status() == NodeStatus.RUNNING


def test_send_goal_transitions_to_running():
    fake = FakeRosNode()
    node = _init(fake)
    node.send_goal("my_goal")
    # FakeActionClient resolves goal immediately → state should be SUCCEEDED or still RUNNING
    # depending on callbacks. Since it resolves synchronously → SUCCEEDED path.
    assert node.action_status() == NodeStatus.SUCCESS


def test_action_succeeds():
    fake = FakeRosNode()
    node = _init(fake, succeed=True)
    node.send_goal("goal")
    assert node.action_status() == NodeStatus.SUCCESS
    assert node.goal_succeeded is True


def test_action_fails():
    fake = FakeRosNode()
    node = _init(fake, succeed=False)
    node.send_goal("goal")
    assert node.action_status() == NodeStatus.FAILURE
    assert node.goal_succeeded is False


def test_action_rejected():
    fake = FakeRosNode()
    node = _init(fake, accepted=False)
    node.send_goal("goal")
    assert node.action_status() == NodeStatus.FAILURE


def test_action_running_while_goal_pending():
    fake = FakeRosNode()
    node = _ActNode("n", ros_node=fake)
    sys.modules["rclpy.action"].ActionClient = lambda n, t, name: SlowActionClient(n, t, name)
    node._init_action_client(_MOCK_TYPE, "/act")
    node.send_goal("goal")
    assert node.action_status() == NodeStatus.RUNNING
    # Resolve acceptance — still RUNNING while result is pending
    client = node._RosActionClientMixin__action_client
    client._pending.resolve()
    assert node.action_status() == NodeStatus.RUNNING
    # Resolve result → SUCCESS
    client._pending._result._result_future.resolve()
    assert node.action_status() == NodeStatus.SUCCESS


def test_cancel_goal_marks_cancelled():
    fake = FakeRosNode()
    sys.modules["rclpy.action"].ActionClient = lambda n, t, name: SlowActionClient(n, t, name)
    node = _ActNode("n", ros_node=fake)
    node._init_action_client(_MOCK_TYPE, "/act")
    node.send_goal("goal")
    # Resolve acceptance so goal_handle exists and state is RUNNING
    client = node._RosActionClientMixin__action_client
    client._pending.resolve()
    node.cancel_goal()
    assert client._pending._result.cancelled is True


def test_action_result_property():
    fake = FakeRosNode()
    node = _init(fake, succeed=True)
    node.send_goal("goal")
    assert node.action_result is not None
    assert node.action_result.status == 4  # STATUS_SUCCEEDED
