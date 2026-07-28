import pytest

from bteng import NodeStatus, StatefulActionNode
from bteng_ros2 import RosServiceClientMixin
from bteng_ros2.testing import FakeRosNode
from test.helpers import FakeFuture


def test_create_publisher_registers_in_dict():
    fake = FakeRosNode()
    fake.create_publisher(object, "/t", 10)
    assert "/t" in fake.publishers


def test_publisher_stores_published_messages():
    fake = FakeRosNode()
    pub = fake.create_publisher(object, "/t")
    pub.publish("a")
    pub.publish("b")
    assert fake.publishers["/t"].published == ["a", "b"]


def test_create_subscription_registers_in_dict():
    fake = FakeRosNode()
    fake.create_subscription(object, "/t", lambda m: None)
    assert "/t" in fake.subscriptions


def test_subscription_inject_calls_callback():
    fake = FakeRosNode()
    received = []
    fake.create_subscription(object, "/t", received.append)
    fake.subscriptions["/t"].inject("msg")
    assert received == ["msg"]


def test_create_client_registers_in_dict():
    fake = FakeRosNode()
    fake.create_client(object, "/srv")
    assert "/srv" in fake.service_clients


def test_fake_service_client_resolves_immediately():
    fake = FakeRosNode()
    fake.create_client(object, "/srv")
    client = fake.service_clients["/srv"]
    client.set_response("resp")
    future = client.call_async("req")
    results = []
    future.add_done_callback(lambda f: results.append(f.result()))
    assert results == ["resp"]


def test_fake_service_client_is_ready_by_default():
    fake = FakeRosNode()
    fake.create_client(object, "/srv")
    assert fake.service_clients["/srv"].service_is_ready() is True


def test_fake_service_client_readiness_is_settable():
    fake = FakeRosNode()
    client = fake.create_client(object, "/srv")
    client.ready = False
    assert client.service_is_ready() is False
    client.set_ready(True)
    assert client.service_is_ready() is True


def test_fake_service_client_counts_readiness_polls():
    """A discovery test needs to assert polling happened, not infer it."""
    fake = FakeRosNode()
    client = fake.create_client(object, "/srv")
    for _ in range(3):
        client.service_is_ready()
    assert client.ready_polls == 3


def test_node_seeds_client_readiness():
    """Nodes create their own clients, so the default must be settable up front."""
    fake = FakeRosNode(service_ready=False)
    client = fake.create_client(object, "/srv")
    assert client.service_is_ready() is False


def test_create_client_kwarg_overrides_node_default():
    fake = FakeRosNode(service_ready=False)
    client = fake.create_client(object, "/srv", ready=True)
    assert client.service_is_ready() is True


def test_deferred_client_does_not_resolve_until_told():
    fake = FakeRosNode()
    client = fake.create_client(object, "/srv", deferred=True)
    client.set_response("resp")
    future = client.call_async("req")
    results = []
    future.add_done_callback(lambda f: results.append(f.result()))
    assert results == []
    client.resolve()
    assert results == ["resp"]


def test_deferred_client_resolve_can_supply_the_response():
    fake = FakeRosNode(service_deferred=True)
    client = fake.create_client(object, "/srv")
    future = client.call_async("req")
    results = []
    future.add_done_callback(lambda f: results.append(f.result()))
    client.resolve("late")
    assert results == ["late"]


def test_deferred_client_resolve_can_supply_none():
    """None is a real response (the mixin reads it as FAILURE), not 'no argument'."""
    fake = FakeRosNode()
    client = fake.create_client(object, "/srv", deferred=True)
    client.set_response("preloaded")
    future = client.call_async("req")
    results = []
    future.add_done_callback(lambda f: results.append(f.result()))
    client.resolve(None)
    assert results == [None]


def test_resolve_without_deferred_call_raises():
    fake = FakeRosNode()
    client = fake.create_client(object, "/srv")
    client.call_async("req")
    with pytest.raises(AssertionError, match="no deferred call outstanding"):
        client.resolve()


def test_client_records_requests():
    fake = FakeRosNode()
    client = fake.create_client(object, "/srv")
    client.call_async("a")
    client.call_async("b")
    assert client.requests == ["a", "b"]


def test_deferred_service_node_reports_running_then_success():
    """The end-to-end shape the deferred mode exists for."""
    class SetParam(RosServiceClientMixin, StatefulActionNode):
        def on_start(self):
            self._init_service_client(object, "/set_param")
            self.call_service("req")
            return self.service_status()

        def on_running(self):
            return self.service_status()

    fake = FakeRosNode(service_deferred=True)
    node = SetParam("svc", ros_node=fake)
    assert node.on_start() == NodeStatus.RUNNING

    client = fake.service_clients["/set_param"]
    assert node.on_running() == NodeStatus.RUNNING   # several ticks, still pending
    assert node.on_running() == NodeStatus.RUNNING

    client.resolve("ok")
    assert node.on_running() == NodeStatus.SUCCESS
    assert node.service_response == "ok"


def test_get_logger_returns_object():
    fake = FakeRosNode()
    logger = fake.get_logger()
    assert logger is not None


def test_get_clock_now_returns_nanoseconds():
    fake = FakeRosNode()
    t = fake.get_clock().now()
    assert hasattr(t, "nanoseconds")
    assert isinstance(t.nanoseconds, int)


def test_get_name_returns_name():
    fake = FakeRosNode("my_node")
    assert fake.get_name() == "my_node"
