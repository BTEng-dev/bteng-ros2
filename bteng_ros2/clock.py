"""RosClock — BTEng Clock protocol backed by rclpy for sim time support."""

from __future__ import annotations


class RosClock:
    """BTEng Clock backed by rclpy so that /use_sim_time=true is respected.

    Pass to any BTEng node that accepts a clock argument (Timeout, RateController):

        clock = RosClock(my_ros_node)
        timeout = Timeout(child_node, duration=5.0, clock=clock)

    RosBTExecutor wires this automatically into Timeout and RateController nodes
    that were constructed with the default WallClock.
    """

    def __init__(self, ros_node) -> None:
        self._node = ros_node

    def monotonic(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9
