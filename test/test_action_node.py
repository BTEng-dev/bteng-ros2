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


def test_on_start_returns_a_status_not_none():
    """StatefulActionNode.tick() RETURNS on_start(), so None here would make the
    tree read a live action as a terminal result on its very first tick."""
    _use_fake_client()
    n = _Nav("n", ros_node=FakeRosNode())
    status = n.on_start()
    assert isinstance(status, NodeStatus)


def test_on_start_reports_running_while_the_goal_is_pending():
    sys.modules["rclpy.action"].ActionClient = (
        lambda rn, t, name: SlowActionClient(rn, t, name)
    )
    n = _Nav("n", ros_node=FakeRosNode())
    assert n.on_start() == NodeStatus.RUNNING


def test_first_tick_reports_running_while_the_goal_is_pending():
    sys.modules["rclpy.action"].ActionClient = (
        lambda rn, t, name: SlowActionClient(rn, t, name)
    )
    n = _Nav("n", ros_node=FakeRosNode())
    assert n.tick() == NodeStatus.RUNNING


def test_first_tick_reports_failure_when_the_server_rejects_the_goal():
    _use_fake_client(accepted=False)
    n = _Nav("n", ros_node=FakeRosNode())
    assert n.tick() == NodeStatus.FAILURE


def test_on_success_fires_when_the_goal_resolves_during_on_start():
    """A fast server can complete the goal before on_start() returns. Without
    routing that first status through the same handler as on_running(), the node
    reported SUCCESS with on_success() never called — so the result never
    reached the output port."""
    _use_fake_client()
    seen = []

    class _Nav2(_Nav):
        def on_success(self):
            seen.append(self.action_result)

    n = _Nav2("n", ros_node=FakeRosNode())
    assert n.on_start() == NodeStatus.SUCCESS
    assert len(seen) == 1


def test_terminal_callback_is_not_fired_twice():
    _use_fake_client()
    calls = []

    class _Nav3(_Nav):
        def on_success(self):
            calls.append(1)

    n = _Nav3("n", ros_node=FakeRosNode())
    n.tick()
    assert calls == [1]


# ── Server discovery (see test_discovery.py for the full matrix) ───────────────


class _UndiscoveredActionClient:
    """Action client whose server never shows up — send_goal_async is a trap.

    A real ActionClient silently drops a goal sent to an undiscovered server, so
    a node that sends one anyway sits at RUNNING for ever. Raising here turns
    that silent bug into a test failure.
    """

    def __init__(self, *a, **kw) -> None:
        self.goals_sent: list = []

    def server_is_ready(self) -> bool:
        return False

    def send_goal_async(self, goal):
        raise AssertionError("goal sent to a server that has not been discovered")


def test_discovery_timeout_defaults_to_five_seconds():
    assert RosActionNode.discovery_timeout == 5.0


def test_on_start_waits_instead_of_sending_into_the_void():
    sys.modules["rclpy.action"].ActionClient = (
        lambda rn, t, name: _UndiscoveredActionClient()
    )
    n = _Nav("n", ros_node=FakeRosNode())
    assert n.on_start() == NodeStatus.RUNNING
    assert "/navigate" in n.feedback_message


def test_discovery_timeout_zero_fails_on_the_first_tick():
    class _FailFast(_Nav):
        discovery_timeout = 0.0

    sys.modules["rclpy.action"].ActionClient = (
        lambda rn, t, name: _UndiscoveredActionClient()
    )
    n = _FailFast("n", ros_node=FakeRosNode())
    assert n.on_start() == NodeStatus.FAILURE
    assert "/navigate" in n.feedback_message


def test_action_server_ready_before_init_is_false_not_an_error():
    """Downstream had to poke _RosActionClientMixin__action_client for this."""
    n = _Nav("n", ros_node=FakeRosNode())
    assert n.action_server_ready() is False


def test_a_ready_server_still_succeeds_on_the_first_tick():
    _use_fake_client(succeed=True)
    n = _Nav("n", ros_node=FakeRosNode())
    assert n.tick() == NodeStatus.SUCCESS
    assert n.action_client.goals_sent == [_GOAL]
