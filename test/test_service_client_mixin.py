import time
import pytest
from unittest.mock import MagicMock
from bteng import StatefulActionNode, NodeStatus
from bteng_ros2._service_client import RosServiceClientMixin
from bteng_ros2.testing import FakeRosNode
from test.helpers import DeferredFuture


class _SvcNode(RosServiceClientMixin, StatefulActionNode):
    def on_start(self): pass
    def on_running(self): return NodeStatus.RUNNING
    def on_halted(self): pass


def _node(fake=None):
    return _SvcNode("n", ros_node=fake or FakeRosNode())


def test_init_service_client_creates_client_on_node():
    fake = FakeRosNode()
    n = _node(fake)
    n._init_service_client(object, "/my_srv")
    assert "/my_srv" in fake.service_clients


def test_call_service_invokes_call_async():
    fake = FakeRosNode()
    n = _node(fake)
    n._init_service_client(object, "/srv")
    fake.service_clients["/srv"].set_response("resp")
    n.call_service("req")
    assert n.service_status() == NodeStatus.SUCCESS


def test_service_status_success_when_response_ready():
    fake = FakeRosNode()
    n = _node(fake)
    n._init_service_client(object, "/srv")
    n._svc_client = _make_immediate_client("response_obj")
    n.call_service("req")
    assert n.service_status() == NodeStatus.SUCCESS


def test_service_status_running_while_pending():
    from unittest.mock import MagicMock
    fake = FakeRosNode()
    n = _node(fake)
    deferred = DeferredFuture("response")
    mock_client = MagicMock()
    mock_client.call_async.return_value = deferred
    n._init_service_client(object, "/srv")
    n._svc_client = mock_client
    n.call_service("req")
    assert n.service_status() == NodeStatus.RUNNING
    deferred.resolve()
    assert n.service_status() == NodeStatus.SUCCESS


def test_service_response_property():
    fake = FakeRosNode()
    n = _node(fake)
    n._init_service_client(object, "/srv")
    n._svc_client = _make_immediate_client("my_response")
    n.call_service("req")
    assert n.service_response == "my_response"


def test_service_is_ready_delegates_to_client():
    fake = FakeRosNode()
    n = _node(fake)
    n._init_service_client(object, "/srv")
    assert n.service_is_ready() is True


def test_raises_without_ros_node():
    n = _SvcNode("n")
    with pytest.raises(RuntimeError):
        n._init_service_client(object, "/srv")


# ── helpers ───────────────────────────────────────────────────────────────────

class _ImmediateFuture:
    def __init__(self, result):
        self._result = result

    def add_done_callback(self, cb):
        cb(self)

    def result(self):
        return self._result


def _make_immediate_client(response):
    client = MagicMock()
    client.call_async.return_value = _ImmediateFuture(response)
    client.service_is_ready.return_value = True
    return client


# ── new tests ─────────────────────────────────────────────────────────────────

def test_stale_callback_ignored():
    """Generation counter: resolving a stale future must not advance the state."""
    fake = FakeRosNode()
    n = _node(fake)
    n._init_service_client(object, "/srv")

    first_future = DeferredFuture("first")
    second_future = DeferredFuture("second")
    mock_client = MagicMock()
    mock_client.call_async.side_effect = [first_future, second_future]
    n._svc_client = mock_client

    # gen=1 — first call
    n.call_service("req1")
    # gen=2 — second call resets done/response
    n.call_service("req2")

    # Resolve the stale future (gen=1 < current gen=2) — must stay RUNNING
    first_future.resolve()
    assert n.service_status() == NodeStatus.RUNNING

    # Resolve the current future (gen=2) — must become SUCCESS
    second_future.resolve()
    assert n.service_status() == NodeStatus.SUCCESS
    assert n.service_response == "second"


def test_call_timeout_returns_failure():
    """A call that exceeds call_timeout must return FAILURE from service_status()."""
    fake = FakeRosNode()
    n = _node(fake)
    n._init_service_client(object, "/srv", call_timeout=0.001)

    deferred = DeferredFuture("x")
    mock_client = MagicMock()
    mock_client.call_async.return_value = deferred
    n._svc_client = mock_client

    n.call_service("req")
    time.sleep(0.01)
    assert n.service_status() == NodeStatus.FAILURE


def test_init_service_client_is_idempotent():
    """Calling _init_service_client twice must not register a second DDS client."""
    fake = FakeRosNode()
    n = _node(fake)

    n._init_service_client(object, "/srv")
    first_client = n._svc_client

    n._init_service_client(object, "/srv")

    assert len(fake.service_clients) == 1
    assert n._svc_client is first_client
