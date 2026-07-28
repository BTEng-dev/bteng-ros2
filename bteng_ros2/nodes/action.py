"""RosActionNode — base class for BTEng nodes that call a ROS 2 action server."""

from __future__ import annotations

import time

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

    Server discovery is spread across ticks. send_goal_async() to a server that
    DDS has not discovered yet is silently dropped -- no raise, no queue -- so
    the goal is only sent on the first tick where action_server_ready() says
    yes. Until then the node reports RUNNING; after discovery_timeout seconds it
    reports FAILURE naming the action. The tick thread is never blocked: no
    wait_for_server() call, which would stall the whole tree. Set
    discovery_timeout = 0 to require the server on the very first tick.
    """

    action_type = None
    action_name: str = ""
    #: Seconds to wait for the action server to appear before reporting
    #: FAILURE. 0 requires the server to be ready on the very first tick.
    discovery_timeout: float = 5.0
    # Guards the terminal callback so it fires exactly once per activation, no
    # matter how many times the status is polled.
    _settled: bool = False
    # Monotonic instant at which the wait gives up, and whether this node is in
    # the discovery phase at all. Class-level defaults, and _awaiting_discovery
    # only ever becomes True in on_start(): a subclass with a hand-written
    # on_start() that sends its own goal never enters the wait branch, so its
    # on_running() behaves exactly as it did before discovery existed.
    _discovery_deadline: float = 0.0
    _awaiting_discovery: bool = False

    def on_start(self) -> NodeStatus:
        """Create the client, send the goal if the server is there, report status.

        StatefulActionNode.tick() RETURNS on_start() on the first tick, so this
        must hand back a NodeStatus — returning None made every subclass report
        None on its first tick, which the tree then read as a terminal result.
        action_status() is RUNNING while the goal is pending and FAILURE if the
        server rejected it outright, matching RosServiceNode.on_start().
        """
        if self.action_type is None:
            raise RuntimeError(f"{type(self).__name__}.action_type is not set")
        if not self.action_name:
            raise RuntimeError(f"{type(self).__name__}.action_name is not set")
        self._init_action_client(self.action_type, self.action_name)
        # Every activation starts a fresh deadline and a fresh discovery flag, so
        # halting mid-wait leaves nothing behind for the next activation.
        self._settled = False
        self._awaiting_discovery = False
        self._discovery_deadline = time.monotonic() + max(float(self.discovery_timeout), 0.0)
        return self._send_or_wait()

    def on_running(self) -> NodeStatus:
        if self._awaiting_discovery:
            return self._send_or_wait()
        return self._settle(self.action_status())

    def _send_or_wait(self) -> NodeStatus:
        """Send the goal once the server is up; otherwise wait, then give up.

        Sending routes the resulting status through _settle() exactly as
        on_start() always has: a goal that resolves before send_goal() returns
        (a fast server, or any fake in a test) would otherwise report SUCCESS
        with on_success() never called, so the result would never reach the
        output port.

        A node with no client at all (a subclass or a test that stubbed
        _init_action_client out) has nothing to wait for, so the goal goes
        straight out — the wait exists to cover DDS discovery on a real client,
        not to second-guess a caller who replaced the client machinery.
        """
        if self.action_client is None or self.action_server_ready():
            self._awaiting_discovery = False
            self.send_goal(self.make_goal())
            return self._settle(self.action_status())
        if time.monotonic() >= self._discovery_deadline:
            self._awaiting_discovery = False
            self.set_feedback_message(
                f"no action server at {self.action_name} "
                f"after {float(self.discovery_timeout):g}s"
            )
            # Through _settle() so on_failure() fires exactly once, like any
            # other way this node reaches FAILURE.
            return self._settle(NodeStatus.FAILURE)
        self._awaiting_discovery = True
        self.set_feedback_message(f"waiting for action server {self.action_name}")
        return NodeStatus.RUNNING

    def _settle(self, status: NodeStatus) -> NodeStatus:
        """Fire the terminal callback for a status, exactly once per activation.

        Both on_start() and on_running() route through here, so a goal that
        resolves inside on_start() still gets its callback -- and a caller that
        polls both never gets it twice.
        """
        if status in (NodeStatus.SUCCESS, NodeStatus.FAILURE) and not self._settled:
            self._settled = True
            if status == NodeStatus.SUCCESS:
                self.on_success()
            else:
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
