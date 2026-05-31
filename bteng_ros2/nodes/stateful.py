"""RosStatefulActionNode — stateful BTEng node with full ROS 2 capability."""

from __future__ import annotations

from bteng import StatefulActionNode
from bteng_ros2._action_client import RosActionClientMixin
from bteng_ros2._service_client import RosServiceClientMixin
from bteng_ros2._topic import RosTopicMixin


class RosStatefulActionNode(RosActionClientMixin, RosServiceClientMixin, RosTopicMixin, StatefulActionNode):
    """Base class for stateful BTEng nodes with full ROS 2 capability.

    All capabilities are available out of the box:
        self.create_publisher(...)          from RosTopicMixin
        self.create_subscription(...)       from RosTopicMixin
        self._init_action_client(...)       from RosActionClientMixin
        self.send_goal(...)                 from RosActionClientMixin
        self.action_status()                from RosActionClientMixin
        self.cancel_goal()                  from RosActionClientMixin
        self._init_service_client(...)      from RosServiceClientMixin
        self.call_service(...)              from RosServiceClientMixin
        self.service_status()               from RosServiceClientMixin

    Implement on_start(), on_running(), on_halted() as with any StatefulActionNode.

        class NavigateAndPublish(RosStatefulActionNode):
            def on_start(self):
                self._pub = self.create_publisher(String, "/status", 10)
                self._init_action_client(NavigateToPose, "/navigate_to_pose")
                self.send_goal(self._build_goal())

            def on_running(self):
                self._pub.publish(String(data="navigating"))
                return self.action_status()

            def on_halted(self):
                self.cancel_goal()

    If you only need one capability (e.g. only action, no pub/sub), use the
    targeted mixin directly on StatefulActionNode to keep the class lighter:

        class Navigate(RosActionClientMixin, StatefulActionNode):
            ...
    """
