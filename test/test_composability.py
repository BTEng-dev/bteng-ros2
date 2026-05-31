"""Verify that mixins compose freely without MRO conflicts."""

import sys
from bteng import StatefulActionNode, ConditionNode, NodeStatus
from bteng_ros2._action_client import RosActionClientMixin
from bteng_ros2._service_client import RosServiceClientMixin
from bteng_ros2._topic import RosTopicMixin
from bteng_ros2._mixin import RosNodeMixin
from bteng_ros2.testing import FakeRosNode
from test.helpers import FakeActionClient


# ── Composed classes (user-defined patterns) ──────────────────────────────────

class ActionAndTopic(RosActionClientMixin, RosTopicMixin, StatefulActionNode):
    """Action client + publisher — most common combo."""
    def on_start(self): pass
    def on_running(self): return NodeStatus.SUCCESS
    def on_halted(self): pass


class AllCapabilities(RosActionClientMixin, RosServiceClientMixin, RosTopicMixin, StatefulActionNode):
    """Everything at once."""
    def on_start(self): pass
    def on_running(self): return NodeStatus.SUCCESS
    def on_halted(self): pass


class TopicCondition(RosTopicMixin, ConditionNode):
    """Custom multi-topic condition."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._msg = None

    def tick(self):
        if self._msg is None:
            return NodeStatus.FAILURE
        return NodeStatus.SUCCESS


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_action_and_topic_can_publish():
    fake = FakeRosNode()
    n = ActionAndTopic("n", ros_node=fake)
    n.create_publisher(object, "/status")
    fake.publishers["/status"].publish("ok")
    assert fake.publishers["/status"].published == ["ok"]


def test_action_and_topic_can_send_goal():
    fake = FakeRosNode()
    sys.modules["rclpy.action"].ActionClient = (
        lambda rn, t, name: FakeActionClient(rn, t, name, succeed=True)
    )
    n = ActionAndTopic("n", ros_node=fake)
    n._init_action_client(object, "/act")
    n.send_goal("g")
    assert n.action_status() == NodeStatus.SUCCESS


def test_all_capabilities_combined():
    fake = FakeRosNode()
    n = AllCapabilities("n", ros_node=fake)
    # Topic
    n.create_publisher(object, "/p")
    n.create_subscription(object, "/s", lambda m: None)
    # Service
    n._init_service_client(object, "/srv")
    # All succeed without errors
    assert n._ros_node is fake


def test_deferred_injection_works_on_composed_node():
    n = ActionAndTopic("n")
    assert n._ros_node is None
    fake = FakeRosNode()
    n.set_ros_node(fake)
    assert n._ros_node is fake


def test_ros_node_mixin_in_mro_of_all_composed():
    assert issubclass(ActionAndTopic, RosNodeMixin)
    assert issubclass(AllCapabilities, RosNodeMixin)
    assert issubclass(TopicCondition, RosNodeMixin)


def test_topic_condition_receives_message():
    fake = FakeRosNode()
    n = TopicCondition("n", ros_node=fake)
    n.create_subscription(object, "/data", lambda m: setattr(n, "_msg", m))
    assert n.tick() == NodeStatus.FAILURE
    fake.subscriptions["/data"].inject("payload")
    assert n.tick() == NodeStatus.SUCCESS
