"""Tests for _inject_ros_node tree-walking logic (no full rclpy needed)."""

from bteng import SequenceNode, ActionNode, NodeStatus
from bteng.core.tree import Tree, TreeMetadata
from bteng_ros2._mixin import RosNodeMixin
from bteng_ros2.executor import _inject_ros_node
from bteng_ros2.testing import FakeRosNode


class _RosAware(RosNodeMixin, ActionNode):
    def tick(self): return NodeStatus.SUCCESS


class _Plain(ActionNode):
    def tick(self): return NodeStatus.SUCCESS


def _tree(*nodes):
    root = SequenceNode("root", children=list(nodes))
    return Tree(TreeMetadata(id="t"), root)


def test_injects_into_single_ros_node():
    node = _RosAware("n")
    fake = FakeRosNode()
    _inject_ros_node(_tree(node), fake)
    assert node._ros_node is fake


def test_skips_plain_node_without_set_ros_node():
    node = _Plain("n")
    fake = FakeRosNode()
    _inject_ros_node(_tree(node), fake)
    assert not hasattr(node, "_ros_node")


def test_skips_already_injected_node():
    existing = FakeRosNode("existing")
    node = _RosAware("n", ros_node=existing)
    new_fake = FakeRosNode("new")
    _inject_ros_node(_tree(node), new_fake)
    assert node._ros_node is existing  # not overwritten


def test_walks_nested_tree():
    n1 = _RosAware("n1")
    n2 = _RosAware("n2")
    inner = SequenceNode("inner", children=[n2])
    outer = SequenceNode("outer", children=[n1, inner])
    tree = Tree(TreeMetadata(id="t"), outer)
    fake = FakeRosNode()
    _inject_ros_node(tree, fake)
    assert n1._ros_node is fake
    assert n2._ros_node is fake


def test_mixed_tree_only_injects_ros_aware_nodes():
    ros_node = _RosAware("ros")
    plain_node = _Plain("plain")
    tree = _tree(ros_node, plain_node)
    fake = FakeRosNode()
    _inject_ros_node(tree, fake)
    assert ros_node._ros_node is fake
    assert not hasattr(plain_node, "_ros_node")


def test_handles_none_tree_safely():
    _inject_ros_node(None, FakeRosNode())  # must not raise
