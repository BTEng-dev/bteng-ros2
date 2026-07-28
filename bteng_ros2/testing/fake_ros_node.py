"""FakeRosNode — in-process ROS node stub for testing without rclpy."""

from __future__ import annotations

import time
from typing import Any, Callable, List


class _Unset:
    """Sentinel: 'no argument given', distinct from a response of None."""

    def __repr__(self) -> str:
        return "<unset>"


_UNSET = _Unset()


class FakeRosNode:
    """In-process stub that satisfies RosNodeMixin without a running ROS 2 environment.

    Use in unit tests to exercise ROS-aware BTEng nodes without rclpy:

        from bteng_ros2.testing import FakeRosNode

        fake = FakeRosNode()
        node = MyRosActionNode("test", ros_node=fake)

        # Inject a fake message into a subscription
        sub = fake.subscriptions["/scan"]
        sub.inject(LaserScan(ranges=[1.0, 2.0]))

    ``service_ready`` and ``service_deferred`` seed every client this node
    creates.  Nodes create their own clients from inside ``on_start()``, so
    seeding here is the only way to configure a client that is polled or called
    before the test can reach ``fake.service_clients[...]``.  The defaults —
    ready, immediate — are the historical behaviour.
    """

    def __init__(
        self,
        name: str = "fake_node",
        service_ready: bool = True,
        service_deferred: bool = False,
    ) -> None:
        self._name = name
        self.service_ready = service_ready
        self.service_deferred = service_deferred
        self._publishers: dict[str, _FakePublisher] = {}
        self._subscriptions: dict[str, _FakeSubscription] = {}
        self._service_clients: dict[str, _FakeServiceClient] = {}
        self._logger = _FakeLogger(name)
        # Public: a spin stub fires these, so a test can drive a tick loop.
        self.timers: List["_FakeTimer"] = []

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

    def create_client(
        self,
        srv_type,
        srv_name: str,
        ready: Any = _UNSET,
        deferred: Any = _UNSET,
    ) -> "_FakeServiceClient":
        client = _FakeServiceClient(
            srv_name,
            ready=self.service_ready if ready is _UNSET else ready,
            deferred=self.service_deferred if deferred is _UNSET else deferred,
        )
        self._service_clients[srv_name] = client
        return client

    def create_timer(self, period: float, callback: Callable) -> "_FakeTimer":
        timer = _FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer: "_FakeTimer") -> None:
        timer.cancel()
        if timer in self.timers:
            self.timers.remove(timer)

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


class _FakeTimer:
    """Timer stub with real cancel semantics — fire() stands in for a spin."""

    def __init__(self, period: float, callback: Callable) -> None:
        self.period = period
        self.callback = callback
        self._canceled = False

    def cancel(self) -> None:
        self._canceled = True

    def is_canceled(self) -> bool:
        return self._canceled

    def fire(self) -> None:
        if not self._canceled:
            self.callback()


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
    """Service-client stub with controllable readiness and response timing.

    Immediate mode (the default) keeps the historical behaviour: the future
    returned by ``call_async()`` fires its done-callback the moment one is
    added, so the response is already there when ``call_service()`` returns.

    Deferred mode holds the callback until the test says so, which is what
    lets a test observe RUNNING, or model a service that takes several ticks::

        client = fake.service_clients["/set_param"]
        client.deferred = True
        client.set_response(resp)

        node.on_start()
        assert node.on_running() == NodeStatus.RUNNING   # still in flight
        client.resolve()                                 # response lands now
        assert node.on_running() == NodeStatus.SUCCESS

    Readiness is a plain attribute, so a test can model a server that is not
    discovered yet and count how often the node polled for it::

        client.ready = False
        ...                              # node polls, stays RUNNING
        client.ready = True
        assert client.ready_polls == 3
    """

    def __init__(self, srv_name: str, ready: bool = True, deferred: bool = False) -> None:
        self.srv_name = srv_name
        # Public and settable: service_is_ready() just reports this.
        self.ready = ready
        # Public and settable: True makes call_async() return a future that
        # waits for resolve() instead of firing its callback inline.
        self.deferred = deferred
        # Every request passed to call_async(), oldest first.
        self.requests: List[Any] = []
        # The deferred future from the most recent call_async(), or None.
        self.pending: "_DeferredFuture | None" = None
        # How many times service_is_ready() was asked — lets a discovery test
        # assert that polling actually happened rather than inferring it.
        self.ready_polls = 0
        self._next_response: Any = None

    def service_is_ready(self) -> bool:
        self.ready_polls += 1
        return self.ready

    def set_ready(self, ready: bool = True) -> None:
        """Set what service_is_ready() reports from now on."""
        self.ready = ready

    def set_deferred(self, deferred: bool = True) -> None:
        """Switch between deferred and immediate response delivery."""
        self.deferred = deferred

    def set_response(self, response: Any) -> None:
        """Pre-load the response that call_async() will return."""
        self._next_response = response

    def call_async(self, request: Any) -> "_FakeFuture | _DeferredFuture":
        self.requests.append(request)
        if self.deferred:
            self.pending = _DeferredFuture(self._next_response)
            return self.pending
        return _FakeFuture(self._next_response)

    def resolve(self, response: Any = _UNSET) -> None:
        """Deliver the outstanding deferred call's response.

        ``response`` overrides whatever set_response() pre-loaded, so a test
        that only decides the answer once the call is in flight does not have
        to pre-load anything.
        """
        if self.pending is None:
            raise AssertionError(
                f"{self.srv_name}: resolve() with no deferred call outstanding — "
                "set client.deferred = True before the node calls the service"
            )
        if response is not _UNSET:
            self.pending._result = response
        self.pending.resolve()


class _FakeFuture:
    def __init__(self, result: Any) -> None:
        self._result = result

    def add_done_callback(self, cb: Callable) -> None:
        cb(self)

    def result(self) -> Any:
        return self._result


class _DeferredFuture:
    """Future that holds its done-callback until resolve() is called.

    Mirrors ``bteng_ros2.testing.action_helpers.DeferredFuture``, which does the
    same job for action goals.
    """

    def __init__(self, result: Any) -> None:
        self._result = result
        self._cb: Callable | None = None
        self.resolved = False

    def add_done_callback(self, cb: Callable) -> None:
        self._cb = cb

    def resolve(self) -> None:
        self.resolved = True
        if self._cb is not None:
            self._cb(self)

    def result(self) -> Any:
        return self._result
