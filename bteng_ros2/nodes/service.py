"""RosServiceNode — base class for BTEng nodes that call a ROS 2 service."""

from __future__ import annotations

from bteng import StatefulActionNode, NodeStatus
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
    """

    service_type = None
    service_name: str = ""

    def on_start(self) -> NodeStatus:
        if self.service_type is None:
            raise RuntimeError(f"{type(self).__name__}.service_type is not set")
        if not self.service_name:
            raise RuntimeError(f"{type(self).__name__}.service_name is not set")
        self._init_service_client(self.service_type, self.service_name)
        if not self.service_is_ready():
            return NodeStatus.FAILURE
        self._response_delivered = False
        self.call_service(self.make_request())
        return NodeStatus.RUNNING

    def on_running(self) -> NodeStatus:
        status = self.service_status()
        if status == NodeStatus.SUCCESS and not self._response_delivered:
            self._response_delivered = True
            self.on_response(self.service_response)
        return status

    def on_halted(self) -> None:
        pass

    def make_request(self):
        raise NotImplementedError(f"{type(self).__name__}.make_request() not implemented")

    def on_response(self, _) -> None:
        """Called once when the service responds. Override to handle response."""
