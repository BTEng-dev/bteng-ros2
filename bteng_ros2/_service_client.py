"""RosServiceClientMixin — non-blocking ROS 2 service client for BTEng nodes."""

from __future__ import annotations

import threading
import time

from bteng import NodeStatus
from bteng_ros2._mixin import RosNodeMixin


class RosServiceClientMixin(RosNodeMixin):
    """Adds non-blocking ROS 2 service client capability to any BTEng node.

    Typical usage with StatefulActionNode:

        class SetParam(RosServiceClientMixin, StatefulActionNode):
            def on_start(self):
                from rcl_interfaces.srv import SetParameters
                self._init_service_client(SetParameters, "/my_node/set_parameters")
                self.call_service(self._build_request())

            def on_running(self):
                return self.service_status()
    """

    def _init_service_client(
        self, srv_type, srv_name: str, call_timeout: float = 0.0
    ) -> None:
        # Fix 3: only create a new DDS client if one does not already exist
        if getattr(self, "_svc_client", None) is None:
            node = self._require_ros_node()
            self._svc_client = node.create_client(srv_type, srv_name)
            # Fix 1: one lock shared across all service-call state
            self._svc_lock = threading.Lock()

        # Always reset per-call state on every (re-)init
        self._svc_response = None
        self._svc_done = False
        # Fix 2: generation counter — invalidates stale callbacks
        self._svc_generation = 0
        # Fix 4: optional call timeout
        self._svc_call_timeout = call_timeout
        self._svc_call_start: float = 0.0

    def call_service(self, request) -> None:
        # Fix 6: guard against missing init
        if getattr(self, "_svc_client", None) is None:
            raise RuntimeError(
                f"{type(self).__name__}: call _init_service_client() before call_service()"
            )
        with self._svc_lock:
            self._svc_done = False
            self._svc_response = None
            # Fix 2: bump generation so any in-flight callback is ignored
            self._svc_generation += 1
            current_gen = self._svc_generation
        # Fix 4: record call start time
        self._svc_call_start = time.monotonic()
        future = self._svc_client.call_async(request)
        # Fix 2: pass generation into callback via closure
        future.add_done_callback(
            lambda f: self._RosServiceClientMixin__on_response(f, current_gen)
        )

    def __on_response(self, future, generation: int) -> None:
        with self._svc_lock:
            # Fix 2: discard stale callbacks from a previous call
            if generation != self._svc_generation:
                return
            # Fix write order (Fix 1): response before done flag
            try:
                self._svc_response = future.result()
            except Exception:
                self._svc_response = None
            self._svc_done = True

    def service_status(self) -> NodeStatus:
        """Return current NodeStatus based on service call state. Call in on_running()."""
        # Fix 6: guard against missing init
        if getattr(self, "_svc_client", None) is None:
            raise RuntimeError(
                f"{type(self).__name__}: call _init_service_client() before service_status()"
            )
        with self._svc_lock:
            done = self._svc_done
            response = self._svc_response
        if not done:
            # Fix 4: timeout check — return FAILURE when elapsed exceeds limit
            if self._svc_call_timeout > 0.0:
                elapsed = time.monotonic() - self._svc_call_start
                if elapsed > self._svc_call_timeout:
                    return NodeStatus.FAILURE
            return NodeStatus.RUNNING
        if response is None:
            return NodeStatus.FAILURE
        return NodeStatus.SUCCESS

    @property
    def service_response(self):
        """Response from the last completed service call, or None."""
        if getattr(self, "_svc_client", None) is None:
            return None
        with self._svc_lock:
            return self._svc_response

    def service_is_ready(self) -> bool:
        """True when the service server has been discovered — non-blocking.

        Safe to poll every tick: DDS discovery of an already-running server
        takes tens to hundreds of milliseconds after create_client(), so a
        first-tick "no" says nothing about whether the server exists.
        """
        # Fix 6: guard against missing init
        if getattr(self, "_svc_client", None) is None:
            raise RuntimeError(
                f"{type(self).__name__}: call _init_service_client() before service_is_ready()"
            )
        # A test double without service_is_ready() counts as ready — the
        # counterpart of RosActionClientMixin.action_server_ready().
        probe = getattr(self._svc_client, "service_is_ready", None)
        if not callable(probe):
            return True
        return bool(probe())
