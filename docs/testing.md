# Testing

bteng-ros2 nodes can be unit-tested without a running ROS 2 environment.
The `bteng_ros2.testing` module provides `FakeRosNode`, and the `conftest.py`
pattern mocks `rclpy` at import time so tests run with plain `pytest`.

---

## What works with no rclpy at all

`import bteng_ros2` does **not** require rclpy. The package imports, and every
class it exports is defined and subclassable, on a machine that has never seen
ROS 2. Concretely, with rclpy absent and unimportable:

**Works**

- `import bteng_ros2`, `bteng_ros2.__version__`, everything in `__all__`
- Defining and instantiating `RosActionNode`, `RosStatefulActionNode`,
  `RosConditionNode`, `RosServiceNode`, and the four mixins
- Ticking those nodes against a `FakeRosNode` — publish, subscribe, inject,
  service calls, timers
- Building and validating a `Tree` of them and driving it with a plain
  `bteng.TreeExecutor`
- Subclassing `RosBTExecutor` / `LifecycleBTExecutor` (the `class` statement
  itself)

**Does not work**

- **Constructing** `RosBTExecutor` or `LifecycleBTExecutor`. They *are* rclpy
  nodes (`rclpy.node.Node` / `rclpy.lifecycle.LifecycleNode`), so there is
  nothing to fall back to. Construction raises `ImportError` naming the missing
  symbol and telling you to source a ROS 2 environment — it never half-works.
- Anything that reaches real ROS traffic: `_init_action_client()`,
  `_init_service_client()` against a real node, real QoS profiles.

Branch on `bteng_ros2.executor.RCLPY_AVAILABLE` if your program needs to know.

This is why a CLI built on bteng-ros2 can honestly advertise a ROS-free
`--help` and `--dry-run`. Mocking rclpy in `conftest.py` (below) is still what
you want when the test needs the *executors* — the mock supplies the base
classes the real ones would.

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

Immediate mode is the default and is unchanged: the future fires its
done-callback the moment one is added, so the response is already there when
`call_service()` returns.

```python
client = fake.create_client(object, "/set_param")
client.set_response("ok")
future = client.call_async("req")
# resolves immediately — callback already fired
```

#### Deferred mode — decide when the response lands

Immediate mode means a test can only install a response *after* the first tick,
and it cannot model a service that takes several ticks. Set `deferred` and the
future waits for `resolve()`, mirroring `SlowActionClient` /
`DeferredFuture` for actions.

```python
class SetParam(RosServiceClientMixin, StatefulActionNode):
    def on_start(self):
        self._init_service_client(object, "/set_param")
        self.call_service("req")
        return self.service_status()

    def on_running(self):
        return self.service_status()

def test_running_then_success():
    # Nodes create their own clients inside on_start(), so seed the default
    # on the node rather than reaching for the client afterwards.
    fake = FakeRosNode(service_deferred=True)
    n = SetParam("svc", ros_node=fake)

    assert n.on_start() == NodeStatus.RUNNING

    client = fake.service_clients["/set_param"]
    assert n.on_running() == NodeStatus.RUNNING   # tick 2, still in flight
    assert n.on_running() == NodeStatus.RUNNING   # tick 3, still in flight

    client.resolve("ok")                          # response lands now
    assert n.on_running() == NodeStatus.SUCCESS
    assert n.service_response == "ok"
```

`resolve(response)` overrides anything `set_response()` pre-loaded, so a test
that only decides the answer once the call is in flight need not pre-load.
`resolve(None)` is a real `None` response (`service_status()` reads it as
`FAILURE`), not "no argument". Calling `resolve()` with nothing outstanding
raises `AssertionError` rather than passing silently.

An already-created client can be switched at any time:

```python
client.deferred = True          # or client.set_deferred()
```

#### Readiness — model a server that is not discovered yet

`service_is_ready()` reports the settable `ready` attribute (default `True`)
and counts every poll in `ready_polls`, so a discovery test can assert that
polling actually happened instead of inferring it.

```python
# Every client this node creates starts undiscovered.
fake = FakeRosNode(service_ready=False)
client = fake.create_client(object, "/set_param")

assert client.service_is_ready() is False
assert client.service_is_ready() is False    # node keeps polling across ticks

client.ready = True                          # or client.set_ready()
assert client.service_is_ready() is True
assert client.ready_polls == 3               # all three polls counted
```

Seeding the flag on `FakeRosNode` rather than on the client matters: nodes call
`create_client()` from inside `on_start()`, so a client that must be
undiscovered on its very first poll cannot be configured after the fact.
`create_client(..., ready=..., deferred=...)` overrides the node-level default
for a single client.

Every request passed to `call_async()` is recorded in `client.requests`, oldest
first — that is how you assert a node did *not* send while the server was still
undiscovered.

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
