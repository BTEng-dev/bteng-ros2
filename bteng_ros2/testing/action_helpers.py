"""Reusable action client fakes for testing RosActionClientMixin-based nodes."""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock


class FakeFuture:
    """Future that resolves immediately when add_done_callback is called."""

    def __init__(self, result: Any) -> None:
        self._result = result

    def add_done_callback(self, cb) -> None:
        cb(self)

    def result(self) -> Any:
        return self._result


class DeferredFuture:
    """Future that does NOT resolve until resolve() is called explicitly.

    Use to test RUNNING state and cancellation:

        client = SlowActionClient(...)
        node.send_goal(goal)
        assert node.action_status() == NodeStatus.RUNNING
        client._pending.resolve()   # acceptance
        assert node.action_status() == NodeStatus.RUNNING
        client._pending._result._result_future.resolve()  # result
        assert node.action_status() == NodeStatus.SUCCESS
    """

    def __init__(self, result: Any) -> None:
        self._result = result
        self._cb = None

    def add_done_callback(self, cb) -> None:
        self._cb = cb

    def resolve(self) -> None:
        if self._cb:
            self._cb(self)

    def result(self) -> Any:
        return self._result


class FakeGoalHandle:
    def __init__(self, accepted: bool = True, succeed: bool = True) -> None:
        self.accepted = accepted
        self._succeed = succeed
        self.cancelled = False

    def get_result_async(self) -> FakeFuture:
        result = MagicMock()
        result.status = 4 if self._succeed else 6  # STATUS_SUCCEEDED / STATUS_ABORTED
        return FakeFuture(result)

    def cancel_goal_async(self) -> None:
        self.cancelled = True


class _SlowGoalHandle:
    def __init__(self, accepted: bool = True) -> None:
        self.accepted = accepted
        self.cancelled = False
        self._result_future: DeferredFuture | None = None

    def get_result_async(self) -> DeferredFuture:
        result = MagicMock()
        result.status = 4  # STATUS_SUCCEEDED
        self._result_future = DeferredFuture(result)
        return self._result_future

    def cancel_goal_async(self) -> None:
        self.cancelled = True


class FakeActionClient:
    """Synchronous fake for rclpy.action.ActionClient.

    Goals are accepted and resolved immediately. Use in tests that only care
    about the SUCCESS / FAILURE outcome, not the RUNNING state:

        import sys
        from bteng_ros2.testing import FakeActionClient

        sys.modules["rclpy.action"].ActionClient = (
            lambda n, t, name: FakeActionClient(n, t, name, succeed=True)
        )
        node.on_start()
        assert node.on_running() == NodeStatus.SUCCESS
    """

    def __init__(self, ros_node, action_type, action_name,
                 accepted: bool = True, succeed: bool = True) -> None:
        self.goals_sent: list = []
        self._accepted = accepted
        self._succeed = succeed

    def send_goal_async(self, goal) -> FakeFuture:
        self.goals_sent.append(goal)
        handle = FakeGoalHandle(accepted=self._accepted, succeed=self._succeed)
        return FakeFuture(handle)


class SlowActionClient:
    """Action client that defers both acceptance and result.

    Use to test RUNNING state and goal cancellation:

        sys.modules["rclpy.action"].ActionClient = (
            lambda n, t, name: SlowActionClient(n, t, name)
        )
        node.on_start()
        assert node.action_status() == NodeStatus.RUNNING

        client = node._RosActionClientMixin__action_client
        client._pending.resolve()   # resolve acceptance
        assert node.action_status() == NodeStatus.RUNNING

        node.on_halted()            # cancels the goal
        assert client._pending._result.cancelled is True
    """

    def __init__(self, ros_node, action_type, action_name, **kw) -> None:
        self.goals_sent: list = []
        self._pending: DeferredFuture | None = None

    def send_goal_async(self, goal) -> DeferredFuture:
        self.goals_sent.append(goal)
        handle = _SlowGoalHandle(accepted=True)
        self._pending = DeferredFuture(handle)
        return self._pending
