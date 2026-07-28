"""Server discovery for RosServiceNode / RosActionNode.

Both base classes used to assume their server was already there on the first
tick: RosServiceNode called service_is_ready() microseconds after create_client()
and returned FAILURE, and RosActionNode sent a goal into a void that
send_goal_async() drops without raising. On a real ROS graph DDS discovery takes
tens to hundreds of milliseconds, so neither worked against a freshly built tree.

These tests pin the replacement: a non-blocking wait spread across ticks, RUNNING
while it lasts, FAILURE naming the endpoint when discovery_timeout elapses, and
nothing blocking the tick thread.
"""

from __future__ import annotations

import sys
import time

import pytest
from bteng import NodeStatus
from bteng_ros2.nodes.action import RosActionNode
from bteng_ros2.nodes.service import RosServiceNode
from bteng_ros2.testing import FakeRosNode
from test.helpers import DeferredFuture, FakeFuture, FakeGoalHandle


NEVER = 10 ** 9  # "not ready" for more polls than any test performs


# ── Fakes ──────────────────────────────────────────────────────────────────────
# FakeRosNode's service client reports ready immediately and the shipped action
# fakes have no readiness probe at all, so the late-discovery doubles live here.


class LateServiceClient:
    """Service client that reports not ready for the first `not_ready_polls` polls."""

    def __init__(self, srv_name: str, not_ready_polls: int = 2, response="ok",
                 defer: bool = False) -> None:
        self.srv_name = srv_name
        self.polls = 0
        self.requests: list = []
        self._not_ready_polls = not_ready_polls
        self._response = response
        # defer=True: the call never completes, so the node stays RUNNING and a
        # re-send would be visible instead of being masked by a re-activation.
        self._defer = defer

    def service_is_ready(self) -> bool:
        self.polls += 1
        return self.polls > self._not_ready_polls

    def call_async(self, request):
        self.requests.append(request)
        return DeferredFuture(self._response) if self._defer else FakeFuture(self._response)


class LateDiscoveryRosNode(FakeRosNode):
    """FakeRosNode handing out LateServiceClient instead of the always-ready one."""

    def __init__(self, not_ready_polls: int = 2, response="ok", defer: bool = False) -> None:
        super().__init__()
        self._not_ready_polls = not_ready_polls
        self._response = response
        self._defer = defer

    def create_client(self, srv_type, srv_name: str) -> LateServiceClient:
        client = LateServiceClient(srv_name, self._not_ready_polls, self._response,
                                   defer=self._defer)
        self.service_clients[srv_name] = client
        return client


class LateActionClient:
    """Action client that reports not ready for the first `not_ready_polls` polls.

    Unlike testing.FakeActionClient it has a server_is_ready() probe — that is
    the whole point — and it records every goal so "sent exactly once" is
    checkable.
    """

    def __init__(self, not_ready_polls: int = 2, succeed: bool = True,
                 defer: bool = False) -> None:
        self.polls = 0
        self.goals_sent: list = []
        self._not_ready_polls = not_ready_polls
        self._succeed = succeed
        # defer=True: the goal is never accepted, so the node stays RUNNING and a
        # re-send would be visible instead of being masked by a re-activation.
        self._defer = defer

    def server_is_ready(self) -> bool:
        self.polls += 1
        return self.polls > self._not_ready_polls

    def send_goal_async(self, goal):
        self.goals_sent.append(goal)
        handle = FakeGoalHandle(accepted=True, succeed=self._succeed)
        return DeferredFuture(handle) if self._defer else FakeFuture(handle)


@pytest.fixture(autouse=True)
def _restore_action_client():
    """Each test installs its own ActionClient; put the original back afterwards."""
    original = sys.modules["rclpy.action"].ActionClient
    yield
    sys.modules["rclpy.action"].ActionClient = original


def _install(client: LateActionClient) -> LateActionClient:
    """Make every _init_action_client() hand back this one client instance.

    One instance across activations is what lets the halt/re-activate test keep
    a server that never appears while the node starts a fresh deadline.
    """
    sys.modules["rclpy.action"].ActionClient = lambda n, t, name: client
    return client


# ── Nodes under test ───────────────────────────────────────────────────────────


class _Svc(RosServiceNode):
    service_type = object
    service_name = "/late_srv"

    def make_request(self):
        return object()


class _Act(RosActionNode):
    action_type = object
    action_name = "/late_action"

    def make_goal(self):
        return object()


def _service_client(node: RosServiceNode) -> LateServiceClient:
    return node._svc_client


# ── Service: waiting ───────────────────────────────────────────────────────────


def test_service_node_sends_on_the_tick_its_server_appears():
    """Server shows up on the 3rd tick: the request goes out then, and succeeds."""
    ros = LateDiscoveryRosNode(not_ready_polls=2)
    n = _Svc("n", ros_node=ros)

    assert n.execute_tick() == NodeStatus.RUNNING          # tick 1: poll 1, not ready
    client = _service_client(n)
    assert client.requests == []
    assert n.execute_tick() == NodeStatus.RUNNING          # tick 2: poll 2, not ready
    assert client.requests == []
    assert n.execute_tick() == NodeStatus.RUNNING          # tick 3: ready → request out
    assert len(client.requests) == 1

    responses = []
    n.on_response = responses.append
    assert n.execute_tick() == NodeStatus.SUCCESS          # tick 4: response delivered
    assert responses == ["ok"]


def test_service_node_reports_running_not_failure_while_waiting():
    """The old behaviour — FAILURE on the first tick — is what this replaces."""
    n = _Svc("n", ros_node=LateDiscoveryRosNode(not_ready_polls=NEVER))
    assert [n.execute_tick() for _ in range(4)] == [NodeStatus.RUNNING] * 4


def test_service_node_feedback_message_names_the_service_while_waiting():
    n = _Svc("n", ros_node=LateDiscoveryRosNode(not_ready_polls=NEVER))
    n.execute_tick()
    assert "/late_srv" in n.feedback_message


def test_service_node_fails_naming_the_service_once_the_timeout_elapses():
    class _Short(_Svc):
        discovery_timeout = 0.25

    n = _Short("n", ros_node=LateDiscoveryRosNode(not_ready_polls=NEVER))
    assert n.execute_tick() == NodeStatus.RUNNING
    # Pull the deadline into the past rather than sleeping: the arithmetic is
    # what is under test, not the wall clock.
    n._discovery_deadline = time.monotonic() - 1.0
    assert n.execute_tick() == NodeStatus.FAILURE
    assert "/late_srv" in n.feedback_message
    assert "0.25" in n.feedback_message
    assert _service_client(n).requests == []


def test_service_node_really_gives_up_after_the_timeout_elapses():
    """The same failure driven by the real monotonic clock, not a poked deadline."""

    class _Short(_Svc):
        discovery_timeout = 0.02

    n = _Short("n", ros_node=LateDiscoveryRosNode(not_ready_polls=NEVER))
    assert n.execute_tick() == NodeStatus.RUNNING
    time.sleep(0.05)
    assert n.execute_tick() == NodeStatus.FAILURE


def test_service_discovery_timeout_zero_fails_on_the_first_tick():
    """Opt back in to the old fail-fast behaviour."""

    class _FailFast(_Svc):
        discovery_timeout = 0.0

    n = _FailFast("n", ros_node=LateDiscoveryRosNode(not_ready_polls=NEVER))
    assert n.execute_tick() == NodeStatus.FAILURE
    assert "/late_srv" in n.feedback_message


def test_service_deadline_is_monotonic_not_wall_clock():
    class _Short(_Svc):
        discovery_timeout = 0.5

    n = _Short("n", ros_node=LateDiscoveryRosNode(not_ready_polls=NEVER))
    n.execute_tick()
    assert n._discovery_deadline == pytest.approx(time.monotonic() + 0.5, abs=0.1)


def test_service_request_is_sent_once_not_once_per_tick():
    """The request goes out on the discovery tick and is never re-issued."""
    ros = LateDiscoveryRosNode(not_ready_polls=1, defer=True)
    n = _Svc("n", ros_node=ros)
    statuses = [n.execute_tick() for _ in range(6)]
    assert statuses == [NodeStatus.RUNNING] * 6   # call never completes
    assert len(_service_client(n).requests) == 1


def test_service_halt_mid_wait_starts_a_fresh_deadline():
    class _Short(_Svc):
        discovery_timeout = 0.25

    n = _Short("n", ros_node=LateDiscoveryRosNode(not_ready_polls=NEVER))
    assert n.execute_tick() == NodeStatus.RUNNING
    n._discovery_deadline = time.monotonic() - 1.0   # deadline blown
    n.halt()
    # Re-activation must not inherit the expired deadline.
    assert n.execute_tick() == NodeStatus.RUNNING
    assert n._discovery_deadline > time.monotonic()


def test_ready_service_still_works_on_the_first_tick():
    """The common case — server already discovered — costs no extra tick."""
    n = _Svc("n", ros_node=LateDiscoveryRosNode(not_ready_polls=0, response="done"))
    responses = []
    n.on_response = responses.append

    assert n.execute_tick() == NodeStatus.RUNNING     # request goes out on tick 1
    client = _service_client(n)
    assert len(client.requests) == 1
    assert client.polls == 1                          # one readiness check, no waiting
    assert n.execute_tick() == NodeStatus.SUCCESS
    assert responses == ["done"]


# ── Action: waiting ────────────────────────────────────────────────────────────


def test_action_node_sends_on_the_tick_its_server_appears():
    client = _install(LateActionClient(not_ready_polls=2))
    n = _Act("n", ros_node=FakeRosNode())

    assert n.execute_tick() == NodeStatus.RUNNING          # tick 1: not ready
    assert client.goals_sent == []
    assert n.execute_tick() == NodeStatus.RUNNING          # tick 2: not ready
    assert client.goals_sent == []
    # Tick 3: server there → goal sent, and this fake resolves it synchronously.
    assert n.execute_tick() == NodeStatus.SUCCESS
    assert len(client.goals_sent) == 1


def test_action_node_reports_running_not_failure_while_waiting():
    """Previously the goal went nowhere and the node sat at RUNNING for ever."""
    _install(LateActionClient(not_ready_polls=NEVER))
    n = _Act("n", ros_node=FakeRosNode())
    assert [n.execute_tick() for _ in range(4)] == [NodeStatus.RUNNING] * 4


def test_action_node_feedback_message_names_the_action_while_waiting():
    _install(LateActionClient(not_ready_polls=NEVER))
    n = _Act("n", ros_node=FakeRosNode())
    n.execute_tick()
    assert "/late_action" in n.feedback_message


def test_action_node_fails_naming_the_action_once_the_timeout_elapses():
    class _Short(_Act):
        discovery_timeout = 0.25

    client = _install(LateActionClient(not_ready_polls=NEVER))
    n = _Short("n", ros_node=FakeRosNode())
    assert n.execute_tick() == NodeStatus.RUNNING
    n._discovery_deadline = time.monotonic() - 1.0
    assert n.execute_tick() == NodeStatus.FAILURE
    assert "/late_action" in n.feedback_message
    assert "0.25" in n.feedback_message
    assert client.goals_sent == []


def test_action_node_really_gives_up_after_the_timeout_elapses():
    class _Short(_Act):
        discovery_timeout = 0.02

    _install(LateActionClient(not_ready_polls=NEVER))
    n = _Short("n", ros_node=FakeRosNode())
    assert n.execute_tick() == NodeStatus.RUNNING
    time.sleep(0.05)
    assert n.execute_tick() == NodeStatus.FAILURE


def test_action_discovery_failure_fires_on_failure_once():
    class _Short(_Act):
        discovery_timeout = 0.0

        def __init__(self, *a, **kw) -> None:
            super().__init__(*a, **kw)
            self.failures = 0

        def on_failure(self) -> None:
            self.failures += 1

    _install(LateActionClient(not_ready_polls=NEVER))
    n = _Short("n", ros_node=FakeRosNode())
    assert n.execute_tick() == NodeStatus.FAILURE
    assert n.failures == 1


def test_action_discovery_timeout_zero_fails_on_the_first_tick():
    class _FailFast(_Act):
        discovery_timeout = 0.0

    client = _install(LateActionClient(not_ready_polls=NEVER))
    n = _FailFast("n", ros_node=FakeRosNode())
    assert n.execute_tick() == NodeStatus.FAILURE
    assert client.goals_sent == []


def test_action_deadline_is_monotonic_not_wall_clock():
    class _Short(_Act):
        discovery_timeout = 0.5

    _install(LateActionClient(not_ready_polls=NEVER))
    n = _Short("n", ros_node=FakeRosNode())
    n.execute_tick()
    assert n._discovery_deadline == pytest.approx(time.monotonic() + 0.5, abs=0.1)


def test_action_goal_is_sent_once_not_once_per_tick():
    """The goal goes out on the discovery tick and is never re-sent."""
    client = _install(LateActionClient(not_ready_polls=1, defer=True))
    n = _Act("n", ros_node=FakeRosNode())
    statuses = [n.execute_tick() for _ in range(6)]
    assert statuses == [NodeStatus.RUNNING] * 6   # goal never resolves
    assert len(client.goals_sent) == 1


def test_action_halt_mid_wait_starts_a_fresh_deadline():
    class _Short(_Act):
        discovery_timeout = 0.25

    _install(LateActionClient(not_ready_polls=NEVER))
    n = _Short("n", ros_node=FakeRosNode())
    assert n.execute_tick() == NodeStatus.RUNNING
    n._discovery_deadline = time.monotonic() - 1.0
    n.halt()
    assert n.execute_tick() == NodeStatus.RUNNING
    assert n._discovery_deadline > time.monotonic()


def test_ready_action_server_still_succeeds_on_the_first_tick():
    """The common case — server already discovered — costs no extra tick."""
    client = _install(LateActionClient(not_ready_polls=0))
    seen = []

    class _WithHook(_Act):
        def on_success(self) -> None:
            seen.append(self.action_result)

    n = _WithHook("n", ros_node=FakeRosNode())
    assert n.execute_tick() == NodeStatus.SUCCESS
    assert len(client.goals_sent) == 1
    assert len(seen) == 1


# ── action_server_ready() ──────────────────────────────────────────────────────


def test_action_server_ready_is_false_before_init():
    """No AttributeError on the name-mangled client, and no lie about readiness."""
    n = _Act("n", ros_node=FakeRosNode())
    assert n.action_server_ready() is False
    assert n.action_client is None


def test_action_server_ready_follows_the_client_probe():
    client = _install(LateActionClient(not_ready_polls=1))
    n = _Act("n", ros_node=FakeRosNode())
    n._init_action_client(object, "/late_action")
    assert n.action_client is client
    assert n.action_server_ready() is False   # poll 1
    assert n.action_server_ready() is True    # poll 2


def test_action_server_ready_treats_a_probe_less_double_as_ready():
    """The shipped fakes have no server_is_ready(); they must stay two methods long."""
    from test.helpers import FakeActionClient

    sys.modules["rclpy.action"].ActionClient = (
        lambda n, t, name: FakeActionClient(n, t, name)
    )
    n = _Act("n", ros_node=FakeRosNode())
    n._init_action_client(object, "/late_action")
    assert n.action_server_ready() is True
