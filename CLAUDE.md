# CLAUDE.md — bteng-ros2 Project Context

Read this at the start of every session before touching code.
Update whenever a significant decision is made, a bug is fixed, or a direction changes.

---

## Project Identity

**bteng-ros2** — ROS 2 base classes for the [BTEng](https://pypi.org/project/bteng/)
behavior tree engine.

| | |
|---|---|
| Remote | `github.com/BTEng-dev/bteng-ros2` |
| Branch | `main` (release branch; `gh-pages` holds the built docs) |
| Version | `0.2.1` — single source of truth in `pyproject.toml` |
| Requires | Python >= 3.10, `bteng>=0.3.1` (older cores unsupported) |
| Tests | 164, all passing on bteng 0.3.1 |

A pure Python pip library. No ROS 2 workspace or colcon required. It ships
**mixins and base classes**, not a standalone node — users subclass them inside
their own ROS 2 packages.

The primary downstream consumer is `bteng_nav2`.

---

## Repository Layout

```
bteng-ros2/
├── bteng_ros2/
│   ├── __init__.py            Public API — all exports; __version__ via importlib.metadata
│   ├── py.typed               PEP 561 marker
│   ├── _mixin.py              RosNodeMixin — ros_node plumbing, _require_ros_node()
│   ├── _action_client.py      RosActionClientMixin — non-blocking action client
│   ├── _service_client.py     RosServiceClientMixin — non-blocking service client
│   ├── _topic.py              RosTopicMixin — publishers / subscriptions
│   ├── clock.py               RosClock — delegates monotonic() to node.get_clock()
│   ├── executor.py            RosBTExecutor — rclpy.Node driving a TreeExecutor
│   │                          also _missing_rclpy() + RCLPY_AVAILABLE (see 4b)
│   ├── lifecycle.py           LifecycleBTExecutor — managed-node variant
│   ├── nodes/
│   │   ├── action.py          RosActionNode
│   │   ├── stateful.py        RosStatefulActionNode
│   │   ├── condition.py       RosConditionNode
│   │   └── service.py         RosServiceNode
│   └── testing/
│       ├── fake_ros_node.py   FakeRosNode — test double, no ROS install needed
│       └── action_helpers.py  Fake action clients / goal handles
├── test/                      164 tests; conftest.py mocks rclpy so no ROS is required
├── examples/                  01–04 working examples
├── docs/                      index, getting-started, nodes, mixins, executors, testing
│   └── images/                BTEng-ROS2.png (used by README and docs index)
├── mkdocs.yml                 Docs site config; nav mirrors docs/*.md
├── requirements-docs.txt      mkdocs toolchain for the docs build
├── .github/workflows/
│   ├── test.yml               Unit tests on Python 3.10 / 3.11 / 3.12
│   └── docs.yml               mkdocs gh-deploy on push to main
├── MANIFEST.in                sdist contents (excludes test/, examples/, docs/, CLAUDE.md)
└── pyproject.toml             Single source of truth for version
```

---

## Architecture

```
RosBTExecutor (rclpy.Node)
    ├── rclpy.Timer            fires every tick_interval → TreeExecutor.tick_once()
    └── TreeExecutor (bteng)   Tree + Blackboard + Inspector/Logger hooks
```

**Tick flow:** `RosBTExecutor` replaces BTEng's sleep-based thread with a
`rclpy.Timer`.  The timer fires every `tick_interval` → `TreeExecutor.tick_once()`.
Topic subscription callbacks write to the Blackboard → dirty flag → reactive
re-eval on the next tick.

**Injection flow:** `set_tree()` walks the whole tree and injects an rclpy node
into every ROS-aware node, so tree nodes can be constructed without a
`ros_node` reference.  `RosClock` is wired from the same target, so
`use_sim_time` on *that* node is what `Timeout` / `RateController` obey.

**Client flow:** `RosActionNode` / `RosServiceNode` poll rclpy futures
non-blocking inside `on_running()`, on the same rclpy executor thread — nothing
in this package blocks a callback.

---

## Key Design Decisions

### 1. `run()` vs `rclpy.spin()`
`rclpy.spin(bt)` never returns on its own: the tick timer cancels itself when the
tree settles, but the spin keeps going.  `run(timeout=None)` is the
one-tree-then-exit form — it spins a `SingleThreadedExecutor` until the root
reports a terminal status and returns that `NodeStatus`.  On timeout it halts the
tree and returns `FAILURE`.  It never calls `rclpy.init()` / `rclpy.shutdown()`;
process-wide context setup stays with the caller.

`run()` is re-entrant-safe: a second call after settle or after `halt()` returns
the recorded status instead of hanging.  This is tracked with an explicit
`_halted` flag rather than by asking the timer, because a stubbed timer must not
make a halted executor look runnable.

`SingleThreadedExecutor` is imported **inside** `run()`, not at module scope, so
downstream suites that mock rclpy loosely can still import this module.

### 2. `ros_node=` is the injection target
`None` (default) injects `self`, preserving the original behaviour.  An external
node is injected into the tree *and added to the same spin* — otherwise its
action and service responses are never delivered and every tree node stays
`RUNNING` forever.

### 3. `on_start()` must return a `NodeStatus`
`StatefulActionNode.tick()` *returns* `on_start()` on the first tick, so
returning `None` makes the tree read a live action as a terminal result.
`RosActionNode.on_start()` returns `self.action_status()`: `RUNNING` while the
goal is pending, `FAILURE` if the server rejected it outright.  This matches
`RosServiceNode.on_start()`.  Any user subclass overriding `on_start()` owes the
same contract — see the docstring example in `nodes/stateful.py`.

### 4. A non-`NodeStatus` root result is coerced to `FAILURE`
A node whose `tick()` falls off the end returns `None`.  Formatting that as a
terminal result would raise inside a timer callback, where rclpy swallows the
traceback and the run hangs instead of failing.  `_tick()` logs an error and
substitutes `FAILURE`.  Both `RosBTExecutor._tick()` and
`LifecycleBTExecutor._tick()` carry it.

**On `bteng>=0.3.1` this is belt-and-braces, not the primary defence.**  The
core's `execute_tick()` now rejects a non-`NodeStatus` *at the offending node*,
so a real tree raises there and never reaches this branch.  Kept deliberately:
it is two lines, and it still catches a stubbed executor or a patched
`tick_once()` — which is exactly how the tests reach it, since a real tree no
longer can.  Do not delete it or its tests when tidying.

### 4b. rclpy is optional at import time
`import bteng_ros2` must succeed with **no rclpy installed**.  Downstream CLIs
(`bteng_nav2`, the turtlesim project's `turtle-bt`) advertise a ROS-free
`--help` / `--dry-run` and could not deliver while `executor.py` did
`import rclpy` at module scope.

Every other module already deferred its rclpy import into the method that needs
it (`_action_client.py`, `_topic.py`, `nodes/condition.py`, and `run()`'s own
`SingleThreadedExecutor`).  Only the two executors could not: they subclass
`rclpy.node.Node` / `rclpy.lifecycle.LifecycleNode`, and a `class` statement
needs its base at module-import time.

The fix is `executor._missing_rclpy(symbol, needed_by)` — a factory returning a
placeholder base whose `__init__` raises `ImportError` naming the missing symbol
and the fix.  `executor.py` and `lifecycle.py` each wrap their rclpy imports in
`try/except ImportError` and fall back to it.  When rclpy *is* importable the
real base is used verbatim, so the ROS path is byte-for-byte what it was — no
lazy-class machinery, no extra indirection, no slower common path.  Public API
and annotations are unchanged (`from __future__ import annotations` keeps them
strings, and the placeholder is still bound to the name `Node` etc.).

`bteng_ros2/__init__.py` did **not** need to change: it eagerly does
`from bteng_ros2.executor import RosBTExecutor`, so a PEP-562 lazy submodule
`__getattr__` there would have been defeated by that very line.  Binding the
class against a fallback base is what keeps the eager import working.

The placeholder's metaclass raises `ImportError` on unknown attribute access but
re-raises `AttributeError` for dunders — `copy`, `pickle`, `inspect` and
`typing` all probe for dunders and expect that answer.

**The guarantee, precisely.**  With rclpy absent and unimportable:

- *works*: `import bteng_ros2`, `__version__`, everything in `__all__`;
  defining/instantiating all four mixins and all four node base classes;
  ticking them against `FakeRosNode`; building and validating a `Tree`;
  subclassing `RosBTExecutor` / `LifecycleBTExecutor`.
- *raises `ImportError`*: **constructing** either executor.  They are real rclpy
  nodes; there is nothing to fall back to and half-working would be worse.
- `bteng_ros2.executor.RCLPY_AVAILABLE` is the public boolean to branch on.

### 4c. `FakeRosNode` service clients: readiness and deferred responses
`_FakeServiceClient.call_async()` used to fire its done-callback inline, so a
test could only install a response *after* the first tick and could not model a
service taking several ticks.  It now mirrors `action_helpers.DeferredFuture` /
`SlowActionClient`:

- `client.deferred = True` → `call_async()` returns a `_DeferredFuture` that
  waits for `client.resolve()`.  `resolve(response)` overrides `set_response()`;
  `resolve(None)` is a real `None` (a `_UNSET` sentinel distinguishes the two);
  `resolve()` with nothing outstanding raises `AssertionError`.
- `client.ready` (default `True`) is what `service_is_ready()` reports, and
  `client.ready_polls` counts the polls — a discovery test asserts polling
  happened instead of inferring it.
- `client.requests` records every request, oldest first.
- `FakeRosNode(service_ready=..., service_deferred=...)` seeds both for every
  client the node creates.  This matters because nodes call `create_client()`
  from inside `on_start()`: a client that must be undiscovered on its *first*
  poll cannot be configured after the fact.  `create_client(..., ready=,
  deferred=)` overrides per client.

Immediate + ready stays the default, so every pre-existing test is unaffected.

### 5. Name-mangled client state has class-level defaults
`RosActionClientMixin` declares `__action_client` / `__goal_handle` /
`__action_result` / `__goal_state` at class level so `action_status()` and
`cancel_goal()` answer sanely before `_init_action_client()` has run.  Without
them a node polled that early raises `AttributeError`.

### 6. `RosServiceClientMixin` — thread-safe, generation-counter design
A `threading.Lock` protects `_svc_done` / `_svc_response` because
`rclpy.MultiThreadedExecutor` removes GIL guarantees for callbacks.  Write order
under the lock: `_svc_response` first, then `_svc_done = True` — the reader
snapshots both atomically.

Stale callback rejection uses a generation counter (`_svc_generation`), not
future cancellation.  `call_service()` increments the counter and closes over the
current value; `__on_response()` discards callbacks whose generation no longer
matches.  This handles the nav2 case where `on_halted()` fires mid-call and a new
`on_start()` follows immediately.

`_init_service_client()` is idempotent — it skips `node.create_client()` if a
client already exists, so a BT retry loop reuses the same DDS endpoint rather
than accumulating stale ones.

`_svc_*` uses a single-underscore prefix deliberately: double-underscore mangling
in a mixin is hard to test and fragile when combined with other mixins.

### 7. `RosServiceNode` call lifecycle
`on_start()` checks `service_is_ready()` before `call_async()` and returns
`FAILURE` immediately if the service is unreachable.  Retry logic belongs in the
BT (`Retry` decorator, `FallbackNode`) — the node itself does not retry.

`call_timeout: float = 0.0` class attribute wires through to the mixin; zero
means no timeout.  `_response_delivered` ensures `on_response()` fires exactly
once per call cycle even if the node is re-ticked while returning `SUCCESS`.

### 7b. `bteng>=0.3.1` is the floor
Three 0.3.1 fixes are load-bearing for this package's tests and for the guards
above: `execute_tick()` rejecting a non-`NodeStatus` child result,
`ParallelNode` validating without an explicit `success_threshold`, and reactive
guards re-ticking.  Older cores are not supported and the docs no longer claim
otherwise (`docs/getting-started.md` used to say `bteng >= 0.2.7`).

### 8. Version is single source of truth in `pyproject.toml`
`bteng_ros2.__version__` resolves at runtime via
`importlib.metadata.version("bteng-ros2")`.  Never hardcode the version
elsewhere — bump `pyproject.toml`, and the README version badge alongside it.

### 9. The test suite mocks rclpy
`test/conftest.py` installs stub `rclpy`, `rclpy.node`, `rclpy.executors`,
`rclpy.action` and `rclpy.lifecycle` modules, so the whole suite runs with no ROS
2 installation.  `_Timer` has real cancel semantics plus `fire()`, and
`_SingleThreadedExecutor.spin_once()` fires every added node's timers — enough to
drive `RosBTExecutor.run()` deterministically.  `FakeRosNode` mirrors this with
`create_timer` / `destroy_timer` and a public `.timers` list.

---

## Usage Patterns

### Wire a tree into a ROS 2 process

```python
import rclpy
from bteng import SequenceNode, Tree, TreeMetadata, ExecutorConfig
from bteng_ros2 import RosBTExecutor

rclpy.init()

nav   = Navigate("nav")            # no ros_node needed at construction
check = ObstacleDetected("obs")
root  = SequenceNode("root", children=[check, nav])
tree  = Tree(TreeMetadata(id="robot"), root)

bt = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05))
status = bt.run(timeout=60.0)      # spins until the tree settles, then returns
bt.destroy_node()
rclpy.shutdown()
```

### Share an rclpy node you already own

```python
ros_node = Node("robot_bt", parameter_overrides=[Parameter("use_sim_time", value=True)])
bt = RosBTExecutor(tree, node_name="robot_bt_ticker", ros_node=ros_node)
status = bt.run(timeout=60.0)      # spins ros_node too
```

### Testing without ROS 2

```python
from bteng_ros2.testing import FakeRosNode

fake = FakeRosNode()
node = Navigate("nav", ros_node=fake)

fake.subscriptions["/scan"].inject(LaserScan(ranges=[1.0, 2.0]))
for timer in fake.timers:
    timer.fire()
```

---

## Running Tests

```bash
pip install -e ".[dev]"

python3 -m pytest test -v          # full suite, 164 tests

python3 -c "import bteng_ros2; print(bteng_ros2.__version__)"
```

No ROS 2 install is needed — `test/conftest.py` mocks rclpy.

---

## Completed (2026-07-28) — v0.2.1 (fixes only)

- **`import bteng_ros2` no longer requires rclpy.**  `executor.py` did
  `import rclpy` / `from rclpy.node import Node` at module scope and
  `lifecycle.py` imported `LifecycleNode`, so importing *anything* from the
  package needed a ROS install — while the same `executor.py` already deferred
  `SingleThreadedExecutor` into `run()` for exactly this reason.  The intent was
  clear and half-done.  Both files now wrap their rclpy imports in
  `try/except ImportError` and fall back to `executor._missing_rclpy()`.  See
  Key Design Decision 4b for the precise guarantee and why
  `bteng_ros2/__init__.py` did **not** need to change.
  - **Why it mattered downstream**: `bteng_nav2`'s CLI advertises a ROS-free
    `--help` and `--dry-run` and could not deliver; the turtlesim project's
    `turtle-bt --help` needed rclpy despite never creating a context.
  - Proved with a subprocess that installs a `sys.meta_path` finder raising
    `ImportError` for `rclpy` and every `rclpy.*` submodule: `import bteng_ros2`
    succeeds, `RCLPY_AVAILABLE` is `False`, `RosConditionNode` /
    `RosStatefulActionNode` tick against `FakeRosNode`, a `Tree` validates and
    ticks, and both executors raise `ImportError` on construction.
- **`FakeRosNode` service clients gained deferred responses and settable
  readiness** — `deferred` / `resolve()`, `ready` / `ready_polls`, `requests`,
  and the `FakeRosNode(service_ready=, service_deferred=)` seeds.  Immediate +
  ready remains the default.  See Key Design Decision 4c.  **10 new tests in
  `test/test_fake_ros_node.py`: 116 → 126.**
- **`bteng` floor raised to `>=0.3.1`** in `pyproject.toml`;
  `docs/getting-started.md` no longer claims `>= 0.2.7` works.  See 7b.
- **Both non-`NodeStatus` guards KEPT** (`RosBTExecutor._tick`,
  `LifecycleBTExecutor._tick`), comments rewritten to say they are
  belt-and-braces on 0.3.1+ rather than the primary defence.  Reasoning: on
  0.3.1 the core raises at the offending node, so neither is reachable through a
  real tree — but they are two lines each, they still catch a stubbed executor
  or a patched `tick_once()` (which is how their tests reach them), and the
  failure mode they prevent is a silent hang inside a timer callback.  Deleting
  them would buy nothing and cost a class of un-debuggable hang if a future core
  regresses.
- **Version 0.2.1** in `pyproject.toml` and the README badge; a `bteng>=0.3.1`
  badge added alongside.
- Docs: `docs/testing.md` gained a "What works with no rclpy at all" section and
  full coverage of the new fake capabilities; `docs/executors.md` gained the
  rclpy-requirement admonition and the guard note; `docs/index.md` and
  `docs/getting-started.md` gained the floor and the import guarantee.

---

## Completed (2026-07-28) — v0.2.2

- **`run()` now drains cancellations after a halt.**  `halt()` reaches every
  RUNNING node's `on_halted()`, and for an action node that is
  `cancel_goal_async()` — which only *queues* the request.  `run()` returned the
  instant it saw the halt flag, so nothing ever spun to send it: offline the tree
  stops and everything looks right, but on a robot the tree stops ticking while
  the goal keeps executing.  `cancel_grace` (0.5 s, 0 disables) keeps the spin
  alive after both the halt and the timeout path.  Found by writing the live
  layer of `bteng_nav2_test` — a class of bug no stub can catch.

---

## Completed (2026-07-27) — v0.2.0

- **`RosBTExecutor.run(timeout=None)`** and the **`ros_node=` constructor kwarg**,
  plus the `final_status` property and the `_halted` re-entry flag.  See Key
  Design Decisions 1–2.
- **`RosActionNode.on_start()` now returns `self.action_status()`** instead of
  `None`, so a subclass no longer reports a terminal result on its first tick.
  See Key Design Decision 3.
- **Class-level defaults for the name-mangled action-client attributes**, so an
  early poll cannot raise `AttributeError`.  See Key Design Decision 5.
- **Why**: both executor APIs were already advertised by the class docstring, the
  README, `docs/executors.md` and every downstream `bteng_nav2` call site — none
  of which could actually run.  `rclpy.spin(bt)` never returns after the tree
  settles, so a one-tree-then-exit program had no supported path.
- Test-side: `test/conftest.py` gained `_Timer`, `_SingleThreadedExecutor` and
  `rclpy.ok/spin/spin_once`; `FakeRosNode` gained `create_timer`/`destroy_timer`
  and `.timers`.  17 new tests in `test/test_executor_run.py` and 4 more in
  `test/test_action_node.py`: **82 → 103, all passing on bteng 0.2.8 and 0.3.0.**
- **`run()` now reacts to `halt()` from another thread** — it used to keep
  spinning until the timeout expired, so a supervisor cancelling a tree waited
  out the full bound (measured: 5.0 s instead of 0.16 s).  The loop watches
  `_halted` and returns FAILURE.  This is the cooperative-cancel shape bteng
  0.3.0's own runner-pattern doc recommends: the canceller sets the flag, the
  thread that owns the run reports the result.
- **A concurrent `run()` raises `RuntimeError`** (`_spinning` guard) instead of
  letting rclpy reject the second `add_node()` from deep inside its executor.
- **`LifecycleBTExecutor`**: mirrored the non-`NodeStatus` guard in `_tick`
  (same swallowed-traceback hazard), added `halt()` and a `final_status`
  property that is cleared on every `activate` — one lifecycle node can run a
  tree again after deactivate → cleanup → configure → activate, so a stale
  result must not look like the current one.  Deliberately **no `run()`** there:
  ticking is gated by external transitions, so `rclpy.spin(node)` is correct and
  a single final status would be ill-defined across activations.
- **`ros_logger()` moved from `RosTopicMixin` to `RosNodeMixin`.**  It only needs
  `_require_ros_node()`, but living on the topic mixin meant `RosActionNode` and
  `RosServiceNode` did not have it — `self.ros_logger()` raised `AttributeError`
  in exactly the callbacks (`on_success`/`on_failure`/`on_response`) where you
  reach for a logger.  `examples/04_lifecycle_executor.py` and
  `docs/getting-started.md` both contained that broken call.
- Found by a stress probe (16 scenarios: 200-leaf trees, 60-deep decorator
  nesting, 50 sequential executors on one shared `ros_node`, timer-leak checks,
  exception propagation, timeout accuracy, `tick_interval=0`, cross-thread halt,
  concurrent runs).  8 further tests in `test/test_lifecycle.py` and 3 in
  `test/test_executor_run.py`: **103 → 114, passing on bteng 0.2.8 and 0.3.1.**
- **`RosActionNode.on_start()` routes its first status through `_settle()`.**  It
  used to return `action_status()` directly, so a goal that resolved *before*
  `on_start()` returned reported SUCCESS with `on_success()` never called — the
  result never reached the output port.  `_settled` makes the terminal callback
  fire exactly once per activation, whether the status is first seen in
  `on_start()` or in `on_running()`.  Found by the turtlesim validation project
  `/home/mdirzpr/scripts/github/bteng_ros2_test`, which now asserts the fix.
  **114 → 116.**
- Downstream `bteng_nav2` re-verified against this version: 394 passing.

### Two bteng-core bugs found while stressing this — FIXED upstream in bteng 0.3.1

Both are fixed and pushed on `bteng` `main` (`b387fc3`); the `dev-async` branch was
merged forward into `main` and deleted. Kept here because they explain two of the
guards below:

1. **`ParallelNode` cannot be constructed without a `success_threshold` param.**
   `provided_ports()` declares `InputPort("success_threshold")` with no default
   and `Tree.validate()` requires every input port to be mapped or static, so
   `ParallelNode("p", children=[...])` (the documented default `-1` = all
   children) and a bare `<Parallel>` in XML are both rejected at `set_tree()`.
   Only an explicit attribute/param works.  Reproduced on 0.2.8 and 0.3.0.
   Fix: `InputPort("success_threshold", ..., default=-1)`.
2. **Control nodes read a `None` child result as success.**  A `SequenceNode`
   whose child's `tick()` falls off the end advances past it and the tree reports
   SUCCESS — the exact class of bug that hid the `on_start()` defect above.
   Worth coercing or rejecting in `TreeNode.execute_tick()`.

---

## Completed (2026-05-17)

- **`RosServiceNode` + `RosServiceClientMixin`** — non-blocking ROS 2 service
  client for BTEng nodes.  Thread-safe (Lock), generation counter for stale
  callback rejection, idempotent `_init_service_client()`, `call_timeout` class
  attribute, `_response_delivered` guard, uninitialized-state guards.
  82/82 tests passed.  See Key Design Decisions 6–7.

---

## Pending / Future Work

1. **Type stubs (`.pyi` files)** — `py.typed` is present (PEP 561), but full
   stubs are not written.  Low priority; inline annotations already drive IDE
   autocompletion.
2. **`[project.urls]` in `pyproject.toml`** still points at
   `github.com/mdirzpr/BTEng_ros2` while the remote is `BTEng-dev/bteng-ros2`.
   Left as-is pending an owner decision on the canonical URL.
