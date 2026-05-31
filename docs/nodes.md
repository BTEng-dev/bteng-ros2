# Node types

bteng-ros2 provides three pre-built base classes that cover the most common
robot behavior patterns. All of them work the same way: subclass, declare a
few class attributes, implement the required methods.

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
