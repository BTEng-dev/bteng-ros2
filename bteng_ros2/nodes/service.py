"""RosServiceNode — base class for BTEng nodes that call a ROS 2 service."""

from __future__ import annotations

import time

from bteng import StatefulActionNode, NodeStatus
from bteng_ros2._endpoint import declare_endpoint_port, resolve_endpoint
from bteng_ros2._service_client import RosServiceClientMixin


class RosServiceNode(RosServiceClientMixin, StatefulActionNode):
    """Base class for BTEng nodes that call a single ROS 2 service.

    Declare service_type and service_name as class attributes, then implement
    make_request(). The service call lifecycle (send, poll) is handled for you.

        class ClearCostmap(RosServiceNode):
            service_name = "/clear_costmap_global_srv"

            def on_start(self):
                from nav2_msgs.srv import ClearEntireCostmap
                self.service_type = ClearEntireCostmap
                return super().on_start()

            def make_request(self):
                from nav2_msgs.srv import ClearEntireCostmap
                return ClearEntireCostmap.Request()

    Need to also publish or subscribe while the service call is in flight?
    Don't subclass this — combine mixins directly on StatefulActionNode:

        class CallAndPublish(RosServiceClientMixin, RosTopicMixin, StatefulActionNode):
            def on_start(self):
                self._pub = self.create_publisher(String, "/status", 10)
                self._init_service_client(MySrv, "/my_srv")
                self.call_service(MySrv.Request())
                return NodeStatus.RUNNING

    Server discovery is spread across ticks. Creating a client and asking
    service_is_ready() microseconds later always answers "no" on a real ROS
    graph -- DDS discovery has not finished yet -- so on_start() reports RUNNING
    and the request goes out on the first tick where the server is actually
    there. The tick thread is never blocked: no wait_for_service() call, which
    would stall the whole tree. Set discovery_timeout = 0 to keep the old
    fail-fast behaviour.
    """

    service_type = None
    service_name: str = ""

    def __init_subclass__(cls, **kwargs) -> None:
        # See RosActionNode: the endpoint becomes settable from a tree.
        super().__init_subclass__(**kwargs)
        declare_endpoint_port(cls, "service_name")
    #: Seconds to wait for the service server to appear before reporting
    #: FAILURE. 0 requires the server to be ready on the very first tick.
    discovery_timeout: float = 5.0
    # Monotonic instant at which the wait gives up, and whether this node is in
    # the discovery phase at all. Class-level defaults, and _awaiting_discovery
    # only ever becomes True in on_start(): a subclass with a hand-written
    # on_start() that issues its own call never enters the wait branch, so its
    # on_running() behaves exactly as it did before discovery existed.
    _discovery_deadline: float = 0.0
    _awaiting_discovery: bool = False

    def on_start(self) -> NodeStatus:
        if self.service_type is None:
            raise RuntimeError(f"{type(self).__name__}.service_type is not set")
        resolve_endpoint(self, "service_name")
        if not self.service_name:
            raise RuntimeError(f"{type(self).__name__}.service_name is not set")
        self._init_service_client(self.service_type, self.service_name)
        # Every activation starts a fresh deadline and a fresh discovery flag, so
        # halting mid-wait leaves nothing behind for the next activation.
        self._response_delivered = False
        self._awaiting_discovery = False
        self._discovery_deadline = time.monotonic() + max(float(self.discovery_timeout), 0.0)
        return self._call_or_wait()

    def on_running(self) -> NodeStatus:
        if self._awaiting_discovery:
            return self._call_or_wait()
        status = self.service_status()
        if status == NodeStatus.SUCCESS and not self._response_delivered:
            self._response_delivered = True
            self.on_response(self.service_response)
        return status

    def _call_or_wait(self) -> NodeStatus:
        """Send the request once the server is up; otherwise wait, then give up.

        RUNNING while the server may still turn up, FAILURE once it cannot.
        Never blocks the tick thread — no wait_for_service() anywhere.
        """
        if self.service_is_ready():
            self._awaiting_discovery = False
            self.call_service(self.make_request())
            return NodeStatus.RUNNING
        if time.monotonic() >= self._discovery_deadline:
            self._awaiting_discovery = False
            self.set_feedback_message(
                f"no service server at {self.service_name} "
                f"after {float(self.discovery_timeout):g}s"
            )
            return NodeStatus.FAILURE
        self._awaiting_discovery = True
        self.set_feedback_message(f"waiting for service {self.service_name}")
        return NodeStatus.RUNNING

    def on_halted(self) -> None:
        pass

    def make_request(self):
        raise NotImplementedError(f"{type(self).__name__}.make_request() not implemented")

    def on_response(self, _) -> None:
        """Called once when the service responds. Override to handle response."""
