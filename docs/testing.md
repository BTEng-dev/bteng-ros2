# Testing

bteng-ros2 nodes can be unit-tested without a running ROS 2 environment.
The `bteng_ros2.testing` module provides `FakeRosNode`, and the `conftest.py`
pattern mocks `rclpy` at import time so tests run with plain `pytest`.

---

## Setup: mock rclpy in conftest.py

Create `test/conftest.py` in your project:

```python
import sys
from unittest.mock import MagicMock

# Mock rclpy before any bteng_ros2 import
rclpy_mock         = MagicMock()
rclpy_node_mock    = MagicMock()
rclpy_lc_mock      = MagicMock()
rclpy_action_mock  = MagicMock()

rclpy_node_mock.Node         = object
rclpy_lc_mock.LifecycleNode  = object
rclpy_action_mock.ActionClient = MagicMock()

sys.modules.setdefault("rclpy",                     rclpy_mock)
sys.modules.setdefault("rclpy.node",                rclpy_node_mock)
sys.modules.setdefault("rclpy.lifecycle",           rclpy_lc_mock)
sys.modules.setdefault("rclpy.lifecycle.node",      MagicMock())
sys.modules.setdefault("rclpy.action",              rclpy_action_mock)

action_msgs_mock            = MagicMock()
goal_status_mock            = MagicMock()
goal_status_mock.STATUS_SUCCEEDED = 4
action_msgs_mock.msg.GoalStatus   = goal_status_mock
sys.modules.setdefault("action_msgs",               action_msgs_mock)
sys.modules.setdefault("action_msgs.msg",           action_msgs_mock.msg)
```

This file is loaded automatically by pytest before any test module.

---

## FakeRosNode

`FakeRosNode` implements the same interface as `rclpy.Node` but stores
everything in-memory. Use it to construct nodes and inject messages without
any IPC.

```python
from bteng_ros2.testing import FakeRosNode

fake = FakeRosNode("test_node")
```

### Publishers

```python
pub = fake.create_publisher(object, "/status", 10)
pub.publish("hello")

assert fake.publishers["/status"].published == ["hello"]
```

### Subscriptions

```python
received = []
fake.create_subscription(object, "/scan", received.append, 10)
fake.subscriptions["/scan"].inject(laser_scan_msg)

assert received == [laser_scan_msg]
```

### Service clients

```python
client = fake.create_client(object, "/set_param")
client.set_response("ok")
future = client.call_async("req")
# resolves immediately — callback already fired
```

---

## Testing RosConditionNode

```python
from bteng_ros2 import RosConditionNode
from bteng_ros2.testing import FakeRosNode
from bteng import NodeStatus

class ObstacleFree(RosConditionNode):
    topic_type = object
    topic_name = "/scan"
    def evaluate(self, msg) -> bool:
        return msg > 0.5

def test_returns_failure_before_message():
    fake = FakeRosNode()
    n = ObstacleFree("check", ros_node=fake)
    assert n.tick() == NodeStatus.FAILURE

def test_returns_success_when_clear():
    fake = FakeRosNode()
    n = ObstacleFree("check", ros_node=fake)
    n.tick()  # creates subscription
    fake.subscriptions["/scan"].inject(1.0)
    assert n.tick() == NodeStatus.SUCCESS

def test_returns_failure_when_obstacle():
    fake = FakeRosNode()
    n = ObstacleFree("check", ros_node=fake)
    n.tick()
    fake.subscriptions["/scan"].inject(0.2)
    assert n.tick() == NodeStatus.FAILURE
```

---

## Testing RosActionNode

The action client is mocked via `sys.modules["rclpy.action"].ActionClient`.
`FakeActionClient` and `SlowActionClient` are part of `bteng_ros2.testing`:

```python
import sys
from bteng_ros2 import RosActionNode
from bteng_ros2.testing import FakeRosNode
from bteng import NodeStatus

class GoTo(RosActionNode):
    action_type = object
    action_name = "/navigate"
    def make_goal(self): return "goal"

def _use_fake_client(succeed=True):
    from bteng_ros2.testing import FakeActionClient
    sys.modules["rclpy.action"].ActionClient = (
        lambda n, t, name: FakeActionClient(n, t, name, succeed=succeed)
    )

def test_succeeds():
    fake = FakeRosNode()
    n = GoTo("nav", ros_node=fake)
    _use_fake_client(succeed=True)
    n.on_start()
    assert n.on_running() == NodeStatus.SUCCESS

def test_fails():
    fake = FakeRosNode()
    n = GoTo("nav", ros_node=fake)
    _use_fake_client(succeed=False)
    n.on_start()
    assert n.on_running() == NodeStatus.FAILURE
```

For testing the RUNNING → SUCCESS transition (deferred result):

```python
def test_running_then_success():
    from bteng_ros2.testing import SlowActionClient
    fake = FakeRosNode()
    sys.modules["rclpy.action"].ActionClient = (
        lambda n, t, name: SlowActionClient(n, t, name)
    )
    n = GoTo("nav", ros_node=fake)
    n.on_start()
    assert n.on_running() == NodeStatus.RUNNING

    client = n._RosActionClientMixin__action_client
    client._pending.resolve()                         # acceptance
    assert n.on_running() == NodeStatus.RUNNING

    client._pending._result._result_future.resolve()  # result
    assert n.on_running() == NodeStatus.SUCCESS
```

---

## Running the test suite

```bash
# No ROS 2 install needed
PYTHONPATH=/path/to/BTEng python3 -m pytest test/ -v
```

`PYTHONPATH` only needed if `bteng` is not installed as a pip package. If you
ran `pip install bteng`, omit it:

```bash
python3 -m pytest test/ -v
```
