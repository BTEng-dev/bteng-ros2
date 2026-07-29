# Node types

bteng-ros2 provides three pre-built base classes that cover the most common
robot behavior patterns. All of them work the same way: subclass, declare a
few class attributes, implement the required methods.

Two behaviours come for free. The endpoint you declare is also an **input port**,
so a tree can retarget the node without a subclass — `RosActionNode`,
`RosServiceNode` and `RosConditionNode` subclasses all get one. And action and
service nodes **wait for their server to appear** instead of failing on the
first tick, because DDS discovery has not finished at the moment a client is
created. Both are described at the bottom of this page.

---

## RosActionNode

**Base:** `StatefulActionNode` + `RosActionClientMixin`

Use when a single BT node must drive one ROS 2 action server to completion.

```python
from nav2_msgs.action import NavigateToPose
from bteng_ros2 import RosActionNode

class GoToWaypoint(RosActionNode):
    action_type = NavigateToPose          # required
    action_name = "/navigate_to_pose"     # required

    def make_goal(self):                  # required
        goal = NavigateToPose.Goal()
        goal.pose = self.blackboard.get("target_pose")
        return goal

    def on_success(self):                 # optional
        self.blackboard.set("arrived", True)

    def on_failure(self):                 # optional
        self.blackboard.set("nav_error", True)
```

### Lifecycle

| Tick | What happens |
|---|---|
| First tick | `on_start()` → validates attrs → creates client → sends goal |
| Subsequent ticks | `on_running()` → polls `action_status()` → calls hook on terminal status |
| Halted by parent | `on_halted()` → cancels the in-flight goal |

### When NOT to use RosActionNode

If you need to call a service or publish a topic *while* the action is running,
use `RosStatefulActionNode` (or compose mixins manually). `RosActionNode` only
bundles the action client mixin.

---

## RosConditionNode

**Base:** `ConditionNode` + `RosTopicMixin`

Use when a BT condition should evaluate the latest message on a ROS 2 topic.

```python
from sensor_msgs.msg import LaserScan
from bteng_ros2 import RosConditionNode

class PathIsClear(RosConditionNode):
    topic_type = LaserScan           # required
    topic_name = "/scan"             # required

    def evaluate(self, msg: LaserScan) -> bool:   # required
        return min(msg.ranges) > 0.5
```

### Lifecycle

- The subscription is created **lazily** on the first tick -- no setup method needed.
- Returns `NodeStatus.FAILURE` if no message has arrived yet.
- Returns `SUCCESS` or `FAILURE` based on `evaluate()` thereafter.

### QoS

The default `topic_qos = 10` uses RELIABLE QoS. Sensor topics (LaserScan,
PointCloud2, Image, Imu) are typically published with BEST_EFFORT QoS by
their drivers. If no messages arrive, check QoS compatibility first:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)

class ObstacleDetected(RosConditionNode):
    topic_type = LaserScan
    topic_name = "/scan"
    topic_qos = SENSOR_QOS

    def evaluate(self, msg: LaserScan) -> bool:
        return min(msg.ranges) < 0.5
```

### Multiple topics

For conditions that depend on more than one topic, use `RosTopicMixin` directly:

```python
from sensor_msgs.msg import LaserScan, Imu
from bteng import ConditionNode, NodeStatus
from bteng_ros2 import RosTopicMixin

class BothSensorsReady(RosTopicMixin, ConditionNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._scan = None
        self._imu  = None

    def setup(self):
        self.create_subscription(LaserScan, "/scan",
            lambda m: setattr(self, "_scan", m), 10)
        self.create_subscription(Imu, "/imu",
            lambda m: setattr(self, "_imu", m), 10)

    def tick(self):
        if self._scan is None or self._imu is None:
            return NodeStatus.FAILURE
        return NodeStatus.SUCCESS
```

---

## RosStatefulActionNode

**Base:** `StatefulActionNode` + all three mixins (`RosActionClientMixin`,
`RosServiceClientMixin`, `RosTopicMixin`)

Use when a node needs the full set of ROS 2 capabilities at once.

```python
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool
from std_msgs.msg import String
from bteng import NodeStatus
from bteng_ros2 import RosStatefulActionNode

class NavigateWithStatus(RosStatefulActionNode):

    def on_start(self) -> None:
        # enable motors
        self._init_service_client(SetBool, "/enable_motors")
        self.call_service(SetBool.Request(data=True))

        # status publisher
        self._pub = self.create_publisher(String, "/bt_status", 10)

        # navigation goal
        self._init_action_client(NavigateToPose, "/navigate_to_pose")
        self.send_goal(NavigateToPose.Goal())

    def on_running(self) -> NodeStatus:
        self._pub.publish(String(data="navigating"))
        return self.action_status()

    def on_halted(self) -> None:
        self.cancel_goal()
        self._pub.publish(String(data="cancelled"))
```

### StatefulActionNode protocol

Implement three methods:

| Method | Called when | Must return |
|---|---|---|
| `on_start()` | Node transitions from IDLE to RUNNING | — |
| `on_running()` | Node is RUNNING on a tick | `NodeStatus` |
| `on_halted()` | Parent halts the node mid-execution | — |

`on_start()` is called exactly once per execution cycle, not on every tick.

---

## Accessing the blackboard

All BTEng nodes have a `blackboard` attribute when added to a tree:

```python
def make_goal(self):
    target = self.blackboard.get("target_pose")   # read
    return build_goal(target)

def on_success(self):
    self.blackboard.set("last_waypoint", self.get_name())  # write
```

See the [BTEng documentation](https://github.com/mdirzpr/BTEng) for the full
blackboard API.

---

## The endpoint is a port

A node that pins its endpoint as a class attribute can only ever talk to that
one endpoint. A second planner, a second sensor or a per-robot namespace then
needs a subclass per target — and a package of 38 nodes needs 38 of them.

So the endpoint becomes an input port whose default is the class attribute you
declared:

| Subclass of | Gets the port |
|---|---|
| `RosActionNode` | `action_name` |
| `RosServiceNode` | `service_name` |
| `RosConditionNode` | `topic_name` |
| `RosStatefulActionNode` | none — three mixins, no single endpoint to name |

The port is on **your subclass**, not on the base: `RosActionNode.provided_ports()`
is still empty, and `class GoToWaypoint(RosActionNode)` reports
`action_name` defaulting to `/navigate_to_pose`.

```xml
<GoToWaypoint name="to_dock" action_name="/robot2/navigate_to_pose" />
<GoToWaypoint name="home"    action_name="{home_action}" />
```

Say nothing and the class attribute is used, exactly as before. The port is
resolved at activation, so a blackboard-bound value can change between runs.

The port is added by `__init_subclass__`, not declared on the base: a subclass
that defines `provided_ports()` *replaces* the base's rather than extending it,
and requiring every subclass to call `super().provided_ports()` would silently
break the ones that forgot. The hook wraps whatever the subclass ended up with
and appends the port unless the subclass declared it itself.

`RosTopicMixin` is not covered — a publisher's `topic` stays a class attribute.

---

## Discovery

A client created microseconds ago cannot reach anything yet: DDS discovery takes
time, and `service_is_ready()` asked immediately answers no. A node that treats
that as "no server" fails on its first tick against a graph that is perfectly
healthy — which is what happened to every service node in bteng-nav2 before
0.2.1.

Each node therefore waits for its server to appear, reporting `RUNNING` while it
waits, up to `discovery_timeout` (5.0 s by default):

```python
class SlowToStart(RosServiceNode):
    discovery_timeout = 30.0     # a stack that comes up with the robot

class MustBeThereNow(RosActionNode):
    discovery_timeout = 0.0      # require the server on the very first tick
```

The wait is spread across ticks rather than blocking, so the rest of the tree
keeps running. On timeout the node fails and names the endpoint it waited for —
a name nothing serves and a stack that is not running look identical from inside
the node, so the message has to carry the name.
