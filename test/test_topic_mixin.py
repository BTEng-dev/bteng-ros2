import pytest
from bteng import ActionNode, NodeStatus
from bteng_ros2._topic import RosTopicMixin
from bteng_ros2.testing import FakeRosNode


class _TopicNode(RosTopicMixin, ActionNode):
    def tick(self):
        return NodeStatus.SUCCESS


def _node(fake=None):
    return _TopicNode("n", ros_node=fake or FakeRosNode())


def test_create_publisher_returns_publisher():
    fake = FakeRosNode()
    n = _node(fake)
    from unittest.mock import MagicMock
    pub = n.create_publisher(MagicMock, "/topic", 10)
    assert pub is fake.publishers["/topic"]


def test_create_subscription_returns_subscription():
    fake = FakeRosNode()
    n = _node(fake)
    from unittest.mock import MagicMock
    cb = lambda m: None
    sub = n.create_subscription(MagicMock, "/topic", cb, 10)
    assert sub is fake.subscriptions["/topic"]


def test_subscription_callback_called_on_inject():
    fake = FakeRosNode()
    n = _node(fake)
    received = []
    n.create_subscription(object, "/t", received.append, 10)
    fake.subscriptions["/t"].inject("msg")
    assert received == ["msg"]


def test_ros_logger_returns_logger():
    n = _node()
    logger = n.ros_logger()
    assert logger is not None


def test_create_publisher_raises_without_ros_node():
    n = _TopicNode("n")
    with pytest.raises(RuntimeError, match="ros_node is not set"):
        n.create_publisher(object, "/t")


def test_create_subscription_raises_without_ros_node():
    n = _TopicNode("n")
    with pytest.raises(RuntimeError, match="ros_node is not set"):
        n.create_subscription(object, "/t", lambda m: None)
