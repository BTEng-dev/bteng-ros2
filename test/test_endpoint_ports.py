"""A node's ROS endpoint can be set from a tree, not only by subclassing.

Every node on these base classes pins its endpoint as a class attribute, so
before this a tree could not aim one anywhere else: no second planner, no second
controller server, no per-robot namespace without writing a subclass. Downstream
felt it immediately -- bteng_nav2 ships 38 nodes and exactly one had grown the
port by hand.

Declaring the port on the base class alone would not work: a subclass that
defines provided_ports() replaces the base's, and nearly all of them do. So the
base hooks __init_subclass__ and appends the port to whatever the subclass
declared.
"""
import sys

import pytest

from bteng import Blackboard, NodeConfig, NodeStatus
from bteng_ros2 import RosActionNode, RosConditionNode, RosServiceNode
from bteng_ros2.testing import FakeActionClient, FakeRosNode


@pytest.fixture(autouse=True)
def _fake_action_client():
    """on_start() sends a goal, so the action path needs a client that answers."""
    real = sys.modules["rclpy.action"].ActionClient
    sys.modules["rclpy.action"].ActionClient = (
        lambda node, action_type, name: FakeActionClient(node, action_type, name)
    )
    yield
    sys.modules["rclpy.action"].ActionClient = real


class _Nav(RosActionNode):
    action_type = object
    action_name = "/navigate_to_pose"

    def make_goal(self):
        return object()


class _Clear(RosServiceNode):
    service_type = object
    service_name = "/clear_costmap"

    def make_request(self):
        return object()


class _Battery(RosConditionNode):
    topic_type = object
    topic_name = "/battery_state"

    def evaluate(self, msg):
        return True


class _NavWithPorts(_Nav):
    """A subclass that declares its own ports, like every real node does."""

    @classmethod
    def provided_ports(cls):
        from bteng import InputPort

        return [InputPort("goal_pose")]


def _configured(cls, **ports):
    bb = Blackboard(scope_name=f"ep_{cls.__name__}")
    for key, value in ports.items():
        bb.set(key, value)
    node = cls("n", config=NodeConfig(blackboard=bb, input_ports={k: k for k in ports}))
    node.set_ros_node(FakeRosNode())
    return node


# ── the port exists on every subclass ───────────────────────────────────────────

def test_action_subclass_gets_an_action_name_port():
    ports = {p.name: p.default for p in _Nav.provided_ports()}
    assert ports["action_name"] == "/navigate_to_pose"


def test_service_subclass_gets_a_service_name_port():
    ports = {p.name: p.default for p in _Clear.provided_ports()}
    assert ports["service_name"] == "/clear_costmap"


def test_condition_subclass_gets_a_topic_name_port():
    ports = {p.name: p.default for p in _Battery.provided_ports()}
    assert ports["topic_name"] == "/battery_state"


def test_a_subclass_that_declares_its_own_ports_keeps_them():
    names = {p.name for p in _NavWithPorts.provided_ports()}
    assert names == {"goal_pose", "action_name"}


def test_the_port_is_declared_once_down_an_inheritance_chain():
    names = [p.name for p in _NavWithPorts.provided_ports()]
    assert names.count("action_name") == 1


# ── and it actually retargets the node ──────────────────────────────────────────

def test_action_endpoint_comes_from_the_blackboard():
    node = _configured(_Nav, action_name="/robot1/navigate_to_pose")
    node.on_start()
    assert node.action_name == "/robot1/navigate_to_pose"


def test_service_endpoint_comes_from_the_blackboard():
    node = _configured(_Clear, service_name="/robot1/clear_costmap")
    node.on_start()
    assert node.service_name == "/robot1/clear_costmap"


def test_condition_endpoint_comes_from_the_blackboard():
    node = _configured(_Battery, topic_name="/robot1/battery_state")
    node.tick()
    assert node.topic_name == "/robot1/battery_state"
    assert "/robot1/battery_state" in node._ros_node.subscriptions


def test_without_a_mapping_the_class_attribute_still_wins():
    node = _Nav("n")
    node.set_ros_node(FakeRosNode())
    node.on_start()
    assert node.action_name == "/navigate_to_pose"


def test_an_empty_endpoint_still_raises_the_original_error():
    class _NoName(RosActionNode):
        action_type = object

        def make_goal(self):
            return object()

    node = _NoName("n")
    node.set_ros_node(FakeRosNode())
    try:
        node.on_start()
    except RuntimeError as exc:
        assert "action_name is not set" in str(exc)
    else:
        raise AssertionError("expected the missing-endpoint error")


def test_a_second_activation_re_reads_the_endpoint():
    node = _configured(_Nav, action_name="/robot1/navigate_to_pose")
    node.on_start()
    node.config.blackboard.set("action_name", "/robot2/navigate_to_pose")
    node.on_start()
    assert node.action_name == "/robot2/navigate_to_pose"


def test_a_tree_can_still_tick_a_retargeted_node():
    node = _configured(_Battery, topic_name="/robot1/battery_state")
    assert node.tick() in (NodeStatus.SUCCESS, NodeStatus.FAILURE)
