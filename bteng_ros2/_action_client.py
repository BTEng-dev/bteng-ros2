"""RosActionClientMixin — non-blocking ROS 2 action client for BTEng nodes."""

from __future__ import annotations

import enum
from bteng import NodeStatus
from bteng_ros2._mixin import RosNodeMixin


class _GoalState(enum.Enum):
    IDLE = "idle"
    SENDING = "sending"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    REJECTED = "rejected"
    CANCELLING = "cancelling"


class RosActionClientMixin(RosNodeMixin):
    """Adds non-blocking ROS 2 action client capability to any BTEng node.

    All calls are non-blocking and safe to poll inside on_running().
    The action client is created lazily via _init_action_client().

    Typical usage with StatefulActionNode:

        class Navigate(RosActionClientMixin, StatefulActionNode):
            def on_start(self):
                self._init_action_client(NavigateToPose, "/navigate_to_pose")
                self.send_goal(self._build_goal())

            def on_running(self):
                return self.action_status()

            def on_halted(self):
                self.cancel_goal()

    Combine freely with other mixins:

        class PublishThenNavigate(RosActionClientMixin, RosTopicMixin, StatefulActionNode):
            def on_start(self):
                self._pub = self.create_publisher(String, "/status", 10)
                self._init_action_client(NavigateToPose, "/navigate_to_pose")
                self.send_goal(self._build_goal())

            def on_running(self):
                self._pub.publish(String(data="navigating"))
                return self.action_status()
    """

    def _init_action_client(self, action_type, action_name: str) -> None:
        from rclpy.action import ActionClient
        node = self._require_ros_node()
        self.__action_client = ActionClient(node, action_type, action_name)
        self.__goal_handle = None
        self.__action_result = None
        self.__goal_state = _GoalState.IDLE

    def send_goal(self, goal) -> None:
        self.__goal_state = _GoalState.SENDING
        future = self.__action_client.send_goal_async(goal)
        future.add_done_callback(self.__on_goal_accepted)

    def __on_goal_accepted(self, future) -> None:
        self.__goal_handle = future.result()
        if not self.__goal_handle.accepted:
            self.__goal_state = _GoalState.REJECTED
            return
        self.__goal_state = _GoalState.RUNNING
        result_future = self.__goal_handle.get_result_async()
        result_future.add_done_callback(self.__on_result)

    def __on_result(self, future) -> None:
        self.__action_result = future.result()
        from action_msgs.msg import GoalStatus
        if self.__action_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.__goal_state = _GoalState.SUCCEEDED
        else:
            self.__goal_state = _GoalState.FAILED

    def action_status(self) -> NodeStatus:
        """Return current NodeStatus based on action state. Call in on_running()."""
        if self.__goal_state in (_GoalState.IDLE, _GoalState.SENDING, _GoalState.RUNNING):
            return NodeStatus.RUNNING
        if self.__goal_state == _GoalState.SUCCEEDED:
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

    def cancel_goal(self) -> None:
        """Cancel the in-flight goal. Call from on_halted()."""
        if self.__goal_handle is not None and self.__goal_state == _GoalState.RUNNING:
            self.__goal_handle.cancel_goal_async()
            self.__goal_state = _GoalState.CANCELLING

    @property
    def action_result(self):
        """Result message from the last completed action, or None."""
        return self.__action_result

    @property
    def goal_succeeded(self) -> bool:
        return self.__goal_state == _GoalState.SUCCEEDED
