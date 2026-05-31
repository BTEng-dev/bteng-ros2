import pytest
from bteng import ActionNode, NodeStatus
from bteng_ros2._mixin import RosNodeMixin
from bteng_ros2.testing import FakeRosNode


class _SimpleNode(RosNodeMixin, ActionNode):
    def tick(self):
        return NodeStatus.SUCCESS


def test_init_ros_node_is_none_by_default():
    node = _SimpleNode("n")
    assert node._ros_node is None


def test_init_with_ros_node():
    fake = FakeRosNode()
    node = _SimpleNode("n", ros_node=fake)
    assert node._ros_node is fake


def test_set_ros_node_injects_after_construction():
    fake = FakeRosNode()
    node = _SimpleNode("n")
    node.set_ros_node(fake)
    assert node._ros_node is fake


def test_require_ros_node_raises_when_not_set():
    node = _SimpleNode("n")
    with pytest.raises(RuntimeError, match="ros_node is not set"):
        node._require_ros_node()


def test_require_ros_node_returns_node_when_set():
    fake = FakeRosNode()
    node = _SimpleNode("n", ros_node=fake)
    assert node._require_ros_node() is fake


def test_set_ros_node_overwrites_existing():
    fake1 = FakeRosNode("a")
    fake2 = FakeRosNode("b")
    node = _SimpleNode("n", ros_node=fake1)
    node.set_ros_node(fake2)
    assert node._ros_node is fake2


def test_ros_node_not_passed_to_bteng_base():
    fake = FakeRosNode()
    # BTEng ActionNode.__init__ must not receive ros_node kwarg
    node = _SimpleNode("n", ros_node=fake)
    assert node.name == "n"
