import sys
from bteng import NodeStatus
from bteng_ros2.nodes.stateful import RosStatefulActionNode
from bteng_ros2._action_client import RosActionClientMixin
from bteng_ros2._service_client import RosServiceClientMixin
from bteng_ros2._topic import RosTopicMixin
from bteng_ros2.testing import FakeRosNode
from test.helpers import FakeActionClient


class _Noop(RosStatefulActionNode):
    def on_start(self): pass
    def on_running(self): return NodeStatus.SUCCESS
    def on_halted(self): pass


def test_has_action_client_mixin():
    assert issubclass(RosStatefulActionNode, RosActionClientMixin)


def test_has_service_client_mixin():
    assert issubclass(RosStatefulActionNode, RosServiceClientMixin)


def test_has_topic_mixin():
    assert issubclass(RosStatefulActionNode, RosTopicMixin)


def test_can_publish():
    fake = FakeRosNode()
    n = _Noop("n", ros_node=fake)
    pub = n.create_publisher(object, "/out", 10)
    pub.publish("hello")
    assert fake.publishers["/out"].published == ["hello"]


def test_can_create_subscription():
    fake = FakeRosNode()
    n = _Noop("n", ros_node=fake)
    received = []
    n.create_subscription(object, "/in", received.append, 10)
    fake.subscriptions["/in"].inject("msg")
    assert received == ["msg"]


def test_can_call_action():
    fake = FakeRosNode()
    sys.modules["rclpy.action"].ActionClient = (
        lambda rn, t, name: FakeActionClient(rn, t, name, succeed=True)
    )
    n = _Noop("n", ros_node=fake)
    n._init_action_client(object, "/act")
    n.send_goal("g")
    assert n.action_status() == NodeStatus.SUCCESS
