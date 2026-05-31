# Executors

bteng-ros2 provides two executor classes that drive a BTEng tree from a ROS 2
timer. Choose based on how much lifecycle control you need.

---

## RosBTExecutor

A plain `rclpy.Node` that creates a timer and calls `TreeExecutor.tick_once()`
on every tick. The timer cancels itself when the tree returns a terminal status.

```python
from bteng_ros2.executor import RosBTExecutor
```

### Usage

```python
import rclpy
from bteng import Tree, TreeMetadata, ExecutorConfig, SequenceNode
from bteng_ros2.executor import RosBTExecutor

rclpy.init()

tree = build_my_tree()   # nodes are constructed without ros_node
bt   = RosBTExecutor(
    tree,
    config=ExecutorConfig(tick_interval=0.05),  # 20 Hz
    node_name="bt_executor",
)

rclpy.spin(bt)

bt.halt()           # cancel timer + halt tree
bt.destroy_node()
rclpy.shutdown()
```

### Constructor

```python
RosBTExecutor(
    tree: Tree,
    config: ExecutorConfig | None = None,  # defaults to ExecutorConfig()
    node_name: str = "bt_executor",
)
```

### Methods

| Method | Description |
|---|---|
| `set_tree(tree)` | Replace the tree at runtime. Re-injects `ros_node` into all nodes. |
| `halt()` | Cancel the tick timer and halt the BTEng tree. |
| `bt_executor` | Property — the underlying `TreeExecutor` instance. |

### Deferred ros_node injection

When `set_tree()` is called, the executor walks the entire tree and calls
`set_ros_node(self)` on every node that has a `_ros_node` of `None`. This means
you never pass `ros_node=` at node construction time — the executor injects
itself automatically:

```python
# No ros_node here
nav  = GoToWaypoint("navigate")
scan = PathIsClear("scan")
root = SequenceNode("root", children=[scan, nav])
tree = Tree(TreeMetadata(id="robot"), root)

# Injection happens here
bt = RosBTExecutor(tree, ...)
```

If a node was constructed with an explicit `ros_node=`, the executor respects
that and does not overwrite it.

---

## LifecycleBTExecutor

A `rclpy.lifecycle.LifecycleNode` that separates tree construction from
execution using standard ROS 2 lifecycle transitions.

```python
from bteng_ros2.lifecycle import LifecycleBTExecutor
```

### Lifecycle transitions

| Transition | What happens |
|---|---|
| `configure` | Calls `build_tree()` → injects ros_node → validates tree |
| `activate` | Creates the tick timer |
| `deactivate` | Cancels the timer, halts the tree |
| `cleanup` | Destroys the tree |

### Usage

```python
from bteng import Tree, TreeMetadata, SequenceNode, ExecutorConfig
from bteng_ros2 import RosActionNode
from bteng_ros2.lifecycle import LifecycleBTExecutor

class RobotBT(LifecycleBTExecutor):
    def build_tree(self) -> Tree:
        root = SequenceNode("root", children=[
            GoToWaypoint("navigate"),
        ])
        return Tree(TreeMetadata(id="robot"), root)


import rclpy
rclpy.init()
node = RobotBT(
    config=ExecutorConfig(tick_interval=0.05),
    node_name="bt_executor",
)
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
```

Control from the command line:

```bash
ros2 lifecycle set /bt_executor configure
ros2 lifecycle set /bt_executor activate
ros2 lifecycle set /bt_executor deactivate
ros2 lifecycle set /bt_executor cleanup
```

### Constructor

```python
LifecycleBTExecutor(
    config: ExecutorConfig | None = None,
    node_name: str = "bt_executor",
)
```

### Abstract method

```python
def build_tree(self) -> Tree:
    ...
```

Called during `configure`. Raise any exception here to abort configure and
return `TransitionCallbackReturn.FAILURE`.

---

## Choosing between them

| | `RosBTExecutor` | `LifecycleBTExecutor` |
|---|---|---|
| **Base class** | `rclpy.Node` | `rclpy.lifecycle.LifecycleNode` |
| **Tree build time** | At construction | On `configure` |
| **Restart without restart** | No | Yes — deactivate → cleanup → configure → activate |
| **Integration with `nav2_bringup`** | Basic | Full lifecycle managed |
| **When to use** | Scripts, CI, simple robots | Production robots, managed bringup |

---

## Tick rate and ExecutorConfig

```python
from bteng import ExecutorConfig

config = ExecutorConfig(
    tick_interval=0.05,   # seconds — 0.05 = 20 Hz
)
```

The tick interval is set once at construction. To change it at runtime, call
`set_tree()` again (which recreates the timer) or cancel and recreate the node.

## Sim time

Both executors automatically replace the default `WallClock` in any `Timeout`
or `RateController` node in the tree with `RosClock`, which delegates to
`node.get_clock()`. This means `/use_sim_time=true` is respected without any
manual wiring -- timeouts and rate limits follow simulation time in Gazebo or
other simulators.
