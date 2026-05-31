# bteng-ros2

ROS 2 base classes for the [BTEng](https://github.com/mdirzpr/BTEng) behavior tree engine.

bteng-ros2 is a **pure Python library** — it is not a ROS 2 package and does not require
`colcon` or `ament`. Install it with `pip`, inherit from its base classes in your own ROS 2
package, and wire your nodes into a BTEng tree.

---

## Design philosophy

ROS 2 capabilities are exposed as **Python mixins**. Each mixin adds one concern:

| Mixin | Adds |
|---|---|
| `RosActionClientMixin` | Non-blocking action client (`send_goal`, `action_status`, `cancel_goal`) |
| `RosServiceClientMixin` | Non-blocking service client (`call_service`, `service_status`) |
| `RosTopicMixin` | Publisher and subscriber helpers (`create_publisher`, `create_subscription`) |

Combine only what you need. Three pre-built classes cover the most common patterns:

| Class | Base | Mixins included |
|---|---|---|
| `RosActionNode` | `StatefulActionNode` | `RosActionClientMixin` |
| `RosConditionNode` | `ConditionNode` | `RosTopicMixin` |
| `RosStatefulActionNode` | `StatefulActionNode` | All three mixins |

Two executor classes drive the tree from a ROS 2 timer:

| Class | Base | When to use |
|---|---|---|
| `RosBTExecutor` | `rclpy.Node` | Simple nodes, scripts, tests |
| `LifecycleBTExecutor` | `rclpy.lifecycle.LifecycleNode` | Production robots, managed bringup |

---

## Quick start

```bash
pip install bteng bteng-ros2
```

```python
from bteng import SequenceNode, Tree, TreeMetadata, ExecutorConfig
from bteng_ros2 import RosActionNode
from bteng_ros2.executor import RosBTExecutor
import rclpy

class GoToWaypoint(RosActionNode):
    action_type = NavigateToPose
    action_name = "/navigate_to_pose"

    def make_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = self.blackboard.get("target_pose")
        return goal

rclpy.init()
root = SequenceNode("root", children=[GoToWaypoint("nav")])
tree = Tree(TreeMetadata(id="demo"), root)
bt   = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05))
rclpy.spin(bt)
```

---

## Documentation

- [Getting started](getting-started.md) — installation, first node, running it
- [Node types](nodes.md) — `RosActionNode`, `RosConditionNode`, `RosStatefulActionNode`
- [Mixins](mixins.md) — mixin reference and free composition guide
- [Executors](executors.md) — `RosBTExecutor` vs `LifecycleBTExecutor`
- [Testing](testing.md) — unit testing nodes without a running ROS 2 environment
