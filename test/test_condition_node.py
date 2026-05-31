import pytest
from bteng import NodeStatus
from bteng_ros2.nodes.condition import RosConditionNode
from bteng_ros2.testing import FakeRosNode


class _Obstacle(RosConditionNode):
    topic_type = object
    topic_name = "/scan"

    def evaluate(self, msg) -> bool:
        return msg < 0.5


class _NoType(RosConditionNode):
    topic_name = "/scan"
    def evaluate(self, msg): return True


class _NoName(RosConditionNode):
    topic_type = object
    def evaluate(self, msg): return True


def _node(cls=_Obstacle, fake=None):
    return cls("n", ros_node=fake or FakeRosNode())


def test_returns_failure_before_any_message():
    n = _node()
    assert n.tick() == NodeStatus.FAILURE


def test_creates_subscription_on_first_tick():
    fake = FakeRosNode()
    n = _node(fake=fake)
    n.tick()
    assert "/scan" in fake.subscriptions


def test_subscription_not_recreated_on_second_tick():
    fake = FakeRosNode()
    n = _node(fake=fake)
    n.tick()
    sub1 = fake.subscriptions["/scan"]
    n.tick()
    assert fake.subscriptions["/scan"] is sub1


def test_returns_success_when_evaluate_true():
    fake = FakeRosNode()
    n = _node(fake=fake)
    n.tick()
    fake.subscriptions["/scan"].inject(0.3)
    assert n.tick() == NodeStatus.SUCCESS


def test_returns_failure_when_evaluate_false():
    fake = FakeRosNode()
    n = _node(fake=fake)
    n.tick()
    fake.subscriptions["/scan"].inject(0.8)
    assert n.tick() == NodeStatus.FAILURE


def test_latest_msg_updates_on_each_inject():
    fake = FakeRosNode()
    n = _node(fake=fake)
    n.tick()
    fake.subscriptions["/scan"].inject(0.3)
    assert n.tick() == NodeStatus.SUCCESS
    fake.subscriptions["/scan"].inject(0.9)
    assert n.tick() == NodeStatus.FAILURE


def test_raises_without_topic_type():
    fake = FakeRosNode()
    n = _NoType("n", ros_node=fake)
    with pytest.raises(RuntimeError, match="topic_type is not set"):
        n.tick()


def test_raises_without_topic_name():
    fake = FakeRosNode()
    n = _NoName("n", ros_node=fake)
    with pytest.raises(RuntimeError, match="topic_name is not set"):
        n.tick()
