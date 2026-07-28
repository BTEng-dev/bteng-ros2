"""`import bteng_ros2` must succeed with no rclpy installed.

Downstream CLIs (`bteng_nav2`, the turtlesim project's `turtle-bt`) advertise a
ROS-free `--help` / `--dry-run`. That only holds if importing this package does
not drag in rclpy — see CLAUDE.md, Key Design Decision 4b.

These run in a subprocess with a `sys.meta_path` finder that makes rclpy and
every `rclpy.*` submodule unimportable. A subprocess is required because
`test/conftest.py` installs a mock rclpy into this interpreter's `sys.modules`
before any test runs, so the missing-rclpy path cannot be observed in-process.
"""

from __future__ import annotations

import subprocess
import sys
import textwrap

import pytest

_BLOCK_RCLPY = """
import sys

class _BlockRclpy:
    def find_spec(self, name, path=None, target=None):
        if name == "rclpy" or name.startswith("rclpy."):
            raise ImportError(f"No module named {name!r} (blocked by the test harness)")
        return None

for _m in [m for m in sys.modules if m == "rclpy" or m.startswith("rclpy.")]:
    del sys.modules[_m]
sys.meta_path.insert(0, _BlockRclpy())

try:
    import rclpy
except ImportError:
    pass
else:
    raise SystemExit("harness failed: rclpy is still importable")
"""


def _run_without_rclpy(body: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, "-c", _BLOCK_RCLPY + textwrap.dedent(body)],
        capture_output=True,
        text=True,
    )


def _ok(body: str) -> str:
    proc = _run_without_rclpy(body)
    assert proc.returncode == 0, f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
    return proc.stdout


def test_package_imports_without_rclpy():
    out = _ok("""
        import bteng_ros2
        assert "rclpy" not in sys.modules
        print(bteng_ros2.executor.RCLPY_AVAILABLE)
    """)
    assert out.strip() == "False"


def test_every_public_export_is_bound_without_rclpy():
    _ok("""
        import bteng_ros2
        for name in bteng_ros2.__all__:
            getattr(bteng_ros2, name)
    """)


def test_node_classes_tick_against_the_fake_without_rclpy():
    _ok("""
        from bteng import NodeStatus
        from bteng_ros2 import RosConditionNode
        from bteng_ros2.testing import FakeRosNode

        class ObstacleFree(RosConditionNode):
            topic_type = object
            topic_name = "/scan"
            def evaluate(self, msg):
                return msg > 0.5

        fake = FakeRosNode()
        n = ObstacleFree("check", ros_node=fake)
        assert n.tick() == NodeStatus.FAILURE
        fake.subscriptions["/scan"].inject(1.0)
        assert n.tick() == NodeStatus.SUCCESS
    """)


def test_a_tree_builds_and_ticks_without_rclpy():
    _ok("""
        from bteng import (ExecutorConfig, SequenceNode, Tree, TreeExecutor,
                           TreeMetadata)
        from bteng_ros2 import RosConditionNode
        from bteng_ros2.testing import FakeRosNode

        class Always(RosConditionNode):
            topic_type = object
            topic_name = "/t"
            def evaluate(self, msg):
                return True

        fake = FakeRosNode()
        tree = Tree(TreeMetadata(id="t"),
                    SequenceNode("root", children=[Always("c", ros_node=fake)]))
        ex = TreeExecutor(ExecutorConfig(tick_interval=0.01))
        ex.set_tree(tree)          # validates
        ex.tick_once()
    """)


@pytest.mark.parametrize(
    "cls, ctor, symbol",
    [
        ("RosBTExecutor", "cls(None)", "rclpy.node.Node"),
        ("LifecycleBTExecutor", "cls()", "rclpy.lifecycle.LifecycleNode"),
    ],
)
def test_executors_are_subclassable_but_raise_on_construction(cls, ctor, symbol):
    """They *are* rclpy nodes, so there is nothing to fall back to — but they
    must fail loudly and namefully, not half-work."""
    out = _ok(f"""
        from bteng_ros2 import {cls} as cls

        class Sub(cls):          # the class statement itself must work
            pass

        try:
            {ctor}
        except ImportError as exc:
            print(exc)
        else:
            raise SystemExit("constructed {cls} without rclpy")
    """)
    assert symbol in out
    assert cls in out
    assert "source a ROS 2 environment" in out


def test_rclpy_available_is_true_when_rclpy_is_importable():
    """The mocked rclpy from conftest counts — this is the ROS-present path."""
    from bteng_ros2.executor import RCLPY_AVAILABLE

    assert RCLPY_AVAILABLE is True
