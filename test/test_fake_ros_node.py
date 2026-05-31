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


def test_fake_service_client_is_always_ready():
    fake = FakeRosNode()
    fake.create_client(object, "/srv")
    assert fake.service_clients["/srv"].service_is_ready() is True


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
