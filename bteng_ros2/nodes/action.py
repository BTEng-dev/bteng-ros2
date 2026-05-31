"""RosActionNode — base class for BTEng nodes that call a ROS 2 action server."""

from __future__ import annotations

from bteng import StatefulActionNode, NodeStatus
from bteng_ros2._action_client import RosActionClientMixin


class RosActionNode(RosActionClientMixin, StatefulActionNode):
    """Base class for BTEng nodes that call a single ROS 2 action server.

    Declare action_type and action_name as class attributes, then implement
    make_goal(). The action lifecycle (send, poll, cancel) is handled for you.

        class NavigateToPose(RosActionNode):
            action_type = NavigateToPoseAction
            action_name = "/navigate_to_pose"

            def make_goal(self):
                goal = NavigateToPoseAction.Goal()
                goal.pose = self.blackboard.get("target_pose")
                return goal

            def on_success(self):
                self.blackboard.set("nav_done", True)

    Need to also publish or call a service while the action runs?
    Don't subclass this — combine mixins directly on StatefulActionNode:

        class NavigateAndPublish(RosActionClientMixin, RosTopicMixin, StatefulActionNode):
            def on_start(self):
                self._pub = self.create_publisher(String, "/status", 10)
                self._init_action_client(NavigateToPoseAction, "/navigate_to_pose")
                self.send_goal(self._build_goal())

            def on_running(self):
                self._pub.publish(String(data="navigating"))
                return self.action_status()

            def on_halted(self):
                self.cancel_goal()
    """

    action_type = None
    action_name: str = ""

    def on_start(self) -> None:
        if self.action_type is None:
            raise RuntimeError(f"{type(self).__name__}.action_type is not set")
        if not self.action_name:
            raise RuntimeError(f"{type(self).__name__}.action_name is not set")
        self._init_action_client(self.action_type, self.action_name)
        self.send_goal(self.make_goal())

    def on_running(self) -> NodeStatus:
        status = self.action_status()
        if status == NodeStatus.SUCCESS:
            self.on_success()
        elif status == NodeStatus.FAILURE:
            self.on_failure()
        return status

    def on_halted(self) -> None:
        self.cancel_goal()

    def make_goal(self):
        raise NotImplementedError(f"{type(self).__name__}.make_goal() not implemented")

    def on_success(self) -> None:
        """Called once when the action succeeds. Override to react."""

    def on_failure(self) -> None:
        """Called once when the action fails or is rejected. Override to react."""
