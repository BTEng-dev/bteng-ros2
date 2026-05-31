<p align="center">
  <img src="docs/images/BTEng-ROS2.png" alt="bteng-ros2 — ROS 2 extension for BTEng" width="720">
</p>

# bteng-ros2

ROS 2 base classes for the [BTEng](https://pypi.org/project/bteng/) behavior tree engine.

A pure Python pip library. No ROS 2 workspace or colcon required.

## Install

```bash
source /opt/ros/humble/setup.bash   # or iron / jazzy
pip install bteng-ros2
```

`bteng` is pulled in automatically.

## Design

`bteng-ros2` provides **mixins and base classes** — not a standalone node.
Users inherit from them inside their own ROS 2 packages.

```
Mixins (combine freely)         Pre-combined bases (common patterns)
─────────────────────────       ───────────────────────────────────
RosNodeMixin                    RosActionNode
RosActionClientMixin            RosStatefulActionNode
RosServiceClientMixin           RosConditionNode
RosTopicMixin
```

## Usage

### Simple action node

```python
from bteng_ros2 import RosActionNode
from nav2_msgs.action import NavigateToPose

class Navigate(RosActionNode):
    action_type = NavigateToPose
    action_name = "/navigate_to_pose"

    def make_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = self.blackboard.get("target_pose")
        return goal

    def on_success(self):
        self.blackboard.set("arrived", True)
```

### Stateful node with multiple ROS capabilities

```python
from bteng_ros2 import RosStatefulActionNode
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

class NavigateAndPublish(RosStatefulActionNode):
    def on_start(self):
        self._pub = self.create_publisher(String, "/status", 10)
        self._init_action_client(NavigateToPose, "/navigate_to_pose")
        self.send_goal(self._build_goal())

    def on_running(self):
        self._pub.publish(String(data="navigating"))
        return self.action_status()

    def on_halted(self):
        self.cancel_goal()
```

### Mix freely — combine only what you need

```python
from bteng import StatefulActionNode, NodeStatus
from bteng_ros2 import RosActionClientMixin, RosTopicMixin

# Only action + topic, no service client overhead
class Navigate(RosActionClientMixin, RosTopicMixin, StatefulActionNode):
    ...
```

### Condition from a topic

```python
from bteng_ros2 import RosConditionNode
from sensor_msgs.msg import LaserScan

class ObstacleDetected(RosConditionNode):
    topic_type = LaserScan
    topic_name = "/scan"

    def evaluate(self, msg: LaserScan) -> bool:
        return min(msg.ranges) < 0.5
```

### Wire into your ROS 2 node

```python
import rclpy
from bteng import SequenceNode, Tree, TreeMetadata, ExecutorConfig
from bteng_ros2 import RosBTExecutor

rclpy.init()

nav   = Navigate("nav")        # no ros_node needed at construction
check = ObstacleDetected("obs")
root  = SequenceNode("root", children=[check, nav])
tree  = Tree(TreeMetadata(id="robot"), root)

bt = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05))
# ↑ injects itself into nav and check automatically

rclpy.spin(bt)
bt.halt()
rclpy.shutdown()
```

### Lifecycle variant

```python
from bteng_ros2 import LifecycleBTExecutor

class RobotBT(LifecycleBTExecutor):
    def build_tree(self) -> Tree:
        return Tree(TreeMetadata(id="robot"), ...)
```

Import from `bteng_ros2.lifecycle`:
```python
from bteng_ros2.lifecycle import LifecycleBTExecutor
```

### Testing without ROS 2

```python
from bteng_ros2.testing import FakeRosNode

fake = FakeRosNode()
node = Navigate("nav", ros_node=fake)

# Inject a fake message into a subscription
fake.subscriptions["/scan"].inject(LaserScan(ranges=[1.0, 2.0]))
```

## License

Apache 2.0
