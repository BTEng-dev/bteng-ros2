"""Make a node's ROS endpoint settable from XML without touching the class.

Every node built on these base classes pins its endpoint as a class attribute::

    class Navigate(RosActionNode):
        action_name = "/navigate_to_pose"

which means a tree cannot aim that node anywhere else: no second planner, no
second controller server, no per-robot namespace, without writing a subclass.
Downstream packages hit this immediately -- bteng_nav2 ships 38 nodes and only
one of them (ManageLifecycleNodes) grew the port by hand.

The obvious fix, declaring the port on the base class, does not work: a subclass
that defines ``provided_ports()`` *replaces* the base's, and every one of those
38 does. Asking each to call ``super().provided_ports()`` would mean editing all
of them and would silently break any node that forgot.

So the base classes hook subclass creation instead and wrap whatever
``provided_ports()`` the subclass ended up with, appending the endpoint port if
it is not already declared. The subclass is untouched, the port is always there,
and its default is the class attribute -- so a tree that says nothing behaves
exactly as before.
"""
from __future__ import annotations

_WRAPPED = "_bteng_ros2_endpoint_port"


def declare_endpoint_port(cls: type, attr: str) -> None:
    """Ensure ``cls.provided_ports()`` includes an input port named *attr*.

    Idempotent across an inheritance chain: the marker records which attribute a
    class already wraps, so ``class A(RosActionNode)`` / ``class B(A)`` declares
    ``action_name`` once, not twice.
    """
    if getattr(cls, _WRAPPED, None) == attr and _WRAPPED in vars(cls):
        return

    inherited = cls.provided_ports

    def provided_ports(_cls=cls, _inherited=inherited, _attr=attr):
        from bteng import InputPort

        ports = list(_inherited())
        if any(p.name == _attr for p in ports):
            return ports  # the node declared it itself; leave it alone
        default = getattr(_cls, _attr, "") or ""
        ports.append(
            InputPort(
                _attr,
                f"ROS endpoint this node talks to (default {default!r})",
                default=default,
            )
        )
        return ports

    cls.provided_ports = classmethod(lambda c, _p=provided_ports: _p())  # type: ignore[assignment]
    setattr(cls, _WRAPPED, attr)


def resolve_endpoint(node, attr: str) -> str:
    """Endpoint for this activation: the port if bound, else the class attribute.

    Assigns the result back onto the instance, since the mixins read the
    attribute when they create the client.
    """
    value = node.get_input(attr, None)
    if value is None or value == "":
        value = getattr(type(node), attr, "") or getattr(node, attr, "")
    setattr(node, attr, value)
    return value
