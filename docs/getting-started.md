# Getting started

## Requirements

- Python 3.10+
- ROS 2 Humble, Iron, or Jazzy (sourced in your shell)
- `bteng` >= 0.2.7

## Installation

```bash
pip install bteng bteng-ros2
```

Or, for development (from the repo root):

```bash
pip install -e /path/to/BTEng
pip install -e /path/to/BTEng_ros2
```

Verify:

```bash
python3 -c "import bteng_ros2; print('ok')"
```

---

## Your first node

Create a file `my_bt/navigate.py` in your ROS 2 package:

```python
from nav2_msgs.action import NavigateToPose
from bteng_ros2 import RosActionNode

class GoToWaypoint(RosActionNode):
    action_type = NavigateToPose
    action_name = "/navigate_to_pose"

    def make_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = 1.0
        goal.pose.pose.position.y = 2.0
        return goal

    def on_success(self):
        self.ros_logger().info("Reached waypoint.")

    def on_failure(self):
        self.ros_logger().warn("Navigation failed.")
```

`RosActionNode` calls `make_goal()` on the first tick, sends it to the action
server, and polls non-blocking on every subsequent tick. When the result
arrives it calls `on_success()` or `on_failure()` and returns the corresponding
`NodeStatus`.

---

## Running the tree

Create a `main.py` entry point:

```python
import rclpy
from bteng import SequenceNode, Tree, TreeMetadata, ExecutorConfig
from bteng_ros2.executor import RosBTExecutor
from my_bt.navigate import GoToWaypoint

def build_tree():
    root = SequenceNode("root", children=[
        GoToWaypoint("navigate"),
    ])
    return Tree(TreeMetadata(id="demo"), root)

def main():
    rclpy.init()
    tree = build_tree()
    bt = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05))
    try:
        rclpy.spin(bt)
    except KeyboardInterrupt:
        pass
    finally:
        bt.halt()
        bt.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

`RosBTExecutor` is a plain `rclpy.Node`. It injects itself into all ROS-aware
nodes in the tree when you pass the tree to it, so your nodes do not need to
hold a reference to the ros node at construction time.

---

## Node construction without a ros_node

Nodes can be constructed without a `ros_node` and receive one later via
`RosBTExecutor` (or `LifecycleBTExecutor`). This is the recommended pattern:

```python
# Constructed without ros_node — fine
node = GoToWaypoint("navigate")

# RosBTExecutor injects itself at tree set time
bt = RosBTExecutor(tree, ...)   # injects here
```

If you need to inject manually (e.g. in tests):

```python
from bteng_ros2.testing import FakeRosNode

fake = FakeRosNode()
node = GoToWaypoint("navigate", ros_node=fake)
```

Or pass `ros_node=` at construction:

```python
node = GoToWaypoint("navigate", ros_node=rclpy_node)
```

---

## Running tests without ROS 2

See [Testing](testing.md). Tests run with plain `pytest` — no ROS 2 install needed.
