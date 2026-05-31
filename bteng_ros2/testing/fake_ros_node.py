"""FakeRosNode — in-process ROS node stub for testing without rclpy."""

from __future__ import annotations

import time
from typing import Any, Callable, List


class FakeRosNode:
    """In-process stub that satisfies RosNodeMixin without a running ROS 2 environment.

    Use in unit tests to exercise ROS-aware BTEng nodes without rclpy:

        from bteng_ros2.testing import FakeRosNode

        fake = FakeRosNode()
        node = MyRosActionNode("test", ros_node=fake)

        # Inject a fake message into a subscription
        sub = fake.subscriptions["/scan"]
        sub.inject(LaserScan(ranges=[1.0, 2.0]))
    """

    def __init__(self, name: str = "fake_node") -> None:
        self._name = name
        self._publishers: dict[str, _FakePublisher] = {}
        self._subscriptions: dict[str, _FakeSubscription] = {}
        self._service_clients: dict[str, _FakeServiceClient] = {}
        self._logger = _FakeLogger(name)

    # ── rclpy.Node interface ──────────────────────────────────────────────────

    def get_logger(self) -> "_FakeLogger":
        return self._logger

    def get_clock(self) -> "_FakeClock":
        return _FakeClock()

    def get_name(self) -> str:
        return self._name

    def create_publisher(self, msg_type, topic: str, qos=10) -> "_FakePublisher":
        pub = _FakePublisher(topic)
        self._publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic: str, callback: Callable, qos=10) -> "_FakeSubscription":
        sub = _FakeSubscription(topic, callback)
        self._subscriptions[topic] = sub
        return sub

    def create_client(self, srv_type, srv_name: str) -> "_FakeServiceClient":
        client = _FakeServiceClient(srv_name)
        self._service_clients[srv_name] = client
        return client

    # ── Test helpers ──────────────────────────────────────────────────────────

    @property
    def publishers(self) -> dict[str, "_FakePublisher"]:
        return self._publishers

    @property
    def subscriptions(self) -> dict[str, "_FakeSubscription"]:
        return self._subscriptions

    @property
    def service_clients(self) -> dict[str, "_FakeServiceClient"]:
        return self._service_clients


class _FakeLogger:
    def __init__(self, name: str) -> None:
        self._name = name

    def info(self, msg: str) -> None: pass
    def warn(self, msg: str) -> None: pass
    def error(self, msg: str) -> None: pass
    def debug(self, msg: str) -> None: pass


class _FakeClock:
    def now(self) -> "_FakeTime":
        return _FakeTime(int(time.monotonic() * 1e9))


class _FakeTime:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds


class _FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.published: List[Any] = []

    def publish(self, msg: Any) -> None:
        self.published.append(msg)


class _FakeSubscription:
    def __init__(self, topic: str, callback: Callable) -> None:
        self.topic = topic
        self._callback = callback

    def inject(self, msg: Any) -> None:
        """Simulate receiving a message — calls the subscription callback."""
        self._callback(msg)


class _FakeServiceClient:
    def __init__(self, srv_name: str) -> None:
        self.srv_name = srv_name
        self._next_response: Any = None

    def service_is_ready(self) -> bool:
        return True

    def set_response(self, response: Any) -> None:
        """Pre-load the response that call_async() will return."""
        self._next_response = response

    def call_async(self, request: Any) -> "_FakeFuture":
        future = _FakeFuture(self._next_response)
        return future


class _FakeFuture:
    def __init__(self, result: Any) -> None:
        self._result = result

    def add_done_callback(self, cb: Callable) -> None:
        cb(self)

    def result(self) -> Any:
        return self._result
