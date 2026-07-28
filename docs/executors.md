# Executors

bteng-ros2 provides two executor classes that drive a BTEng tree from a ROS 2
timer. Choose based on how much lifecycle control you need.

!!! note "These two classes are the only part of the package that needs rclpy"

    `import bteng_ros2` succeeds without a ROS 2 install, and so does defining
    or subclassing these executors. **Constructing** one does not: they *are*
    rclpy nodes (`rclpy.node.Node` / `rclpy.lifecycle.LifecycleNode`), so with
    no rclpy the constructor raises `ImportError` naming the missing symbol
    rather than half-working. Branch on `bteng_ros2.executor.RCLPY_AVAILABLE`
    if your program needs to know. See
    [Testing](testing.md#what-works-with-no-rclpy-at-all).

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

status = bt.run(timeout=60.0)   # spins until the tree settles

bt.destroy_node()
rclpy.shutdown()
```

`rclpy.spin(bt)` also works, but it never returns on its own — the tick timer
cancels itself when the tree settles and the spin keeps going. Use `run()` when
the program's job is to execute one tree and exit with its result.

### Constructor

```python
RosBTExecutor(
    tree: Tree,
    config: ExecutorConfig | None = None,  # defaults to ExecutorConfig()
    node_name: str = "bt_executor",
    ros_node: Node | None = None,          # injection target; defaults to self
)
```

### Methods

| Method | Description |
|---|---|
| `run(timeout=None)` | Spin until the tree settles; returns its final `NodeStatus`. On timeout: halts the tree and returns `FAILURE`. Never calls `rclpy.init()`/`shutdown()`. |
| `set_tree(tree)` | Replace the tree at runtime. Re-injects `ros_node` into all nodes. |
| `halt()` | Cancel the tick timer and halt the BTEng tree. |
| `final_status` | Property — the status the tree settled at, or `None` if it has not settled. |
| `bt_executor` | Property — the underlying `TreeExecutor` instance. |

### run()

Ticking is driven by the timer created in the constructor; `run()` only spins a
`SingleThreadedExecutor` so that timer and any client callbacks fire, and
returns as soon as the root reports a terminal status.

- `timeout=None` waits indefinitely. On expiry the tree is halted and `FAILURE`
  is returned — the tree is not left half-running.
- An external `ros_node` (below) is added to the same spin. Without that, its
  action and service responses would never be delivered and every node in the
  tree would stay `RUNNING` forever.
- Calling `run()` again after the tree settled, or after `halt()`, returns the
  recorded status immediately instead of hanging: the tick timer is cancelled,
  so nothing could ever settle a second time.
- `halt()` from another thread ends a spinning `run()` promptly and it returns
  `FAILURE` — a supervisor can cancel a tree without waiting out the timeout:

  ```python
  threading.Timer(5.0, bt.halt).start()   # cancel from anywhere
  status = bt.run(timeout=300.0)          # returns ~immediately on halt
  ```

  That is the cooperative shape to use: the canceller sets the halt, the thread
  that owns the run reports the result. Do not tick or halt the same tree from
  two threads at once.
- A second `run()` while one is still spinning raises `RuntimeError` — rclpy
  allows only one executor per node, so this fails loudly instead of letting the
  second `add_node()` fail deep inside rclpy.
- An exception raised by a node's `tick()` propagates out of `run()`; the spin is
  still torn down and the nodes released, so the executor can be discarded
  safely. Catch it if one bad tree must not kill the process.
- A root result that is not a `NodeStatus` is logged and coerced to `FAILURE`
  rather than formatted (which would raise inside a timer callback, where rclpy
  swallows the traceback and the run hangs). On `bteng>=0.3.1` this is
  belt-and-braces: the core's `execute_tick()` now rejects a non-`NodeStatus` at
  the offending node, so a real tree raises there first. The guard is kept for
  stubbed executors and patched `tick_once()`. `LifecycleBTExecutor._tick()`
  carries the identical guard.

### Sharing one rclpy node

By default the executor injects **itself** into the tree, so all ROS traffic is
served by the `bt_executor` node. Pass `ros_node=` to inject a node you already
own instead — useful when the same process holds parameters, TF listeners or
publishers that the tree nodes should share:

```python
ros_node = Node("robot_bt", parameter_overrides=[Parameter("use_sim_time", value=True)])
bt = RosBTExecutor(tree, node_name="robot_bt_ticker", ros_node=ros_node)
status = bt.run(timeout=60.0)      # spins ros_node too
```

Clocks follow the same target: `RosClock` is wired from whichever node was
injected, so `use_sim_time` on *that* node is what `Timeout`/`RateController`
decorators obey.

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

### Methods

| Method | Description |
|---|---|
| `halt()` | Stop ticking and halt the tree without a lifecycle transition. |
| `final_status` | Property — status the tree settled at, or `None`. Cleared on every `activate`, since one node can run a tree again after deactivate → cleanup → configure → activate. |

There is deliberately **no `run()`** here. Ticking is gated by external
transitions: the node has no tree until `configure` and no timer until
`activate`, so a `run()` would have to spin while unconfigured, waiting for
`ros2 lifecycle set ... activate` from somewhere else — which is exactly what
`rclpy.spin(node)` already does. A single "final status" would also be
ill-defined across repeated activations.

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
