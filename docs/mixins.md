# Mixins

The mixin layer is what makes bteng-ros2 composable. Each mixin adds one ROS 2
capability to any BTEng node type. You can combine them freely using Python
multiple inheritance.

---

## RosNodeMixin

Foundation for all other mixins. Holds the `_ros_node` reference and exposes
`set_ros_node()` for deferred injection.

```python
from bteng_ros2 import RosNodeMixin
```

| Method | Description |
|---|---|
| `set_ros_node(node)` | Inject or replace the underlying rclpy node |
| `_require_ros_node()` | Returns the ros node or raises `RuntimeError` if not set |

You rarely use this directly — it is the base of all other mixins.

---

## RosActionClientMixin

Non-blocking action client. All calls return immediately; results are delivered
via callbacks and polled via `action_status()`.

```python
from bteng_ros2 import RosActionClientMixin
```

| Method | Description |
|---|---|
| `_init_action_client(action_type, action_name)` | Create the `ActionClient`. Call once in `on_start()`. |
| `send_goal(goal)` | Send a goal asynchronously. |
| `action_status()` | Returns `RUNNING`, `SUCCESS`, or `FAILURE`. |
| `cancel_goal()` | Cancel the in-flight goal. Only fires if goal is RUNNING. |
| `action_result` | The raw result message, or `None` if not complete. |
| `goal_succeeded` | `True` if last goal reached `STATUS_SUCCEEDED`. |

### State machine

```
IDLE ──send_goal()──► SENDING ──accepted──► RUNNING ──result──► SUCCEEDED
                                         │                   └──► FAILED
                                         └──rejected──► REJECTED
                                         └──cancel_goal()──► CANCELLING
```

### Example: custom stateful action

```python
from bteng import StatefulActionNode, NodeStatus
from bteng_ros2 import RosActionClientMixin
from my_msgs.action import Dock

class DockRobot(RosActionClientMixin, StatefulActionNode):
    def on_start(self):
        self._init_action_client(Dock, "/dock")
        self.send_goal(Dock.Goal())

    def on_running(self):
        status = self.action_status()
        if status == NodeStatus.SUCCESS:
            self.blackboard.set("docked", True)
        return status

    def on_halted(self):
        self.cancel_goal()
```

---

## RosServiceClientMixin

Non-blocking service client. The future is polled via `service_status()`.

```python
from bteng_ros2 import RosServiceClientMixin
```

| Method | Description |
|---|---|
| `_init_service_client(srv_type, srv_name)` | Create the client. Call in `on_start()`. |
| `call_service(request)` | Send request asynchronously. |
| `service_status()` | Returns `RUNNING` until response arrives, then `SUCCESS` or `FAILURE`. |
| `service_response` | The response object, or `None`. |
| `service_is_ready()` | `True` if the service server is available. |

### Example

```python
from std_srvs.srv import SetBool
from bteng import StatefulActionNode, NodeStatus
from bteng_ros2 import RosServiceClientMixin

class EnableMotors(RosServiceClientMixin, StatefulActionNode):
    def on_start(self):
        self._init_service_client(SetBool, "/enable_motors")
        self.call_service(SetBool.Request(data=True))

    def on_running(self):
        return self.service_status()

    def on_halted(self):
        pass
```

---

## RosTopicMixin

Thin wrappers over `rclpy.Node.create_publisher` and `create_subscription`.

```python
from bteng_ros2 import RosTopicMixin
```

| Method | Description |
|---|---|
| `create_publisher(msg_type, topic, qos=10)` | Returns an `rclpy` publisher. |
| `create_subscription(msg_type, topic, callback, qos=10)` | Returns an `rclpy` subscription. |
| `ros_logger()` | Returns the underlying node's logger. |

### Example

```python
from std_msgs.msg import Bool
from bteng import ActionNode, NodeStatus
from bteng_ros2 import RosTopicMixin

class PublishReady(RosTopicMixin, ActionNode):
    def tick(self):
        if not hasattr(self, "_pub"):
            self._pub = self.create_publisher(Bool, "/robot_ready", 10)
        self._pub.publish(Bool(data=True))
        return NodeStatus.SUCCESS
```

---

## Free composition

Any combination of mixins works as long as:
1. All mixins precede the BTEng base class in the MRO.
2. All `__init__` calls use `super().__init__(*args, **kwargs)`.

```python
from bteng import StatefulActionNode, NodeStatus
from bteng_ros2 import RosActionClientMixin, RosServiceClientMixin, RosTopicMixin

# All three on a single node — same as RosStatefulActionNode
class FullCapNode(
    RosActionClientMixin,
    RosServiceClientMixin,
    RosTopicMixin,
    StatefulActionNode,
):
    def on_start(self): ...
    def on_running(self): ...
    def on_halted(self): ...
```

```python
# Action + topic only
class NavWithFeedback(RosActionClientMixin, RosTopicMixin, StatefulActionNode):
    ...
```

```python
# Service + topic only (no action)
class ConfigureAndReport(RosServiceClientMixin, RosTopicMixin, StatefulActionNode):
    ...
```

The order of mixins in the class definition does not matter as long as the
BTEng base class (`StatefulActionNode`, `ConditionNode`, etc.) comes last.
