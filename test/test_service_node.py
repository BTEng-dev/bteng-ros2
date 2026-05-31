"""Tests for RosServiceNode (bteng_ros2/nodes/service.py)."""

from __future__ import annotations

import time
from unittest.mock import MagicMock

import pytest
from bteng import NodeStatus
from bteng_ros2.nodes.service import RosServiceNode
from bteng_ros2.testing import FakeRosNode
from test.helpers import DeferredFuture


# ── shared helpers ─────────────────────────────────────────────────────────────

class _SimpleServiceNode(RosServiceNode):
    """Minimal concrete subclass — service_type and service_name set as class attrs."""
    service_type = object
    service_name = "/srv"

    def make_request(self):
        return object()


def _make_node(fake=None):
    return _SimpleServiceNode("n", ros_node=fake or FakeRosNode())


class _NotReadyFakeRosNode(FakeRosNode):
    """FakeRosNode whose service clients always report service_is_ready() == False."""

    def create_client(self, srv_type, srv_name):
        client = super().create_client(srv_type, srv_name)
        client.service_is_ready = lambda: False
        return client


# ── tests ──────────────────────────────────────────────────────────────────────

def test_on_start_returns_running_when_service_ready():
    """on_start() returns RUNNING when the service is reachable."""
    fake = FakeRosNode()
    fake.service_clients["/srv"] = MagicMock()
    fake.service_clients["/srv"].service_is_ready.return_value = True
    fake.service_clients["/srv"].call_async.return_value = DeferredFuture("r")

    n = _SimpleServiceNode("n", ros_node=FakeRosNode())
    result = n.on_start()
    assert result == NodeStatus.RUNNING


def test_on_start_returns_failure_when_not_ready():
    """on_start() returns FAILURE immediately when the service is not yet available."""
    fake = _NotReadyFakeRosNode()
    n = _SimpleServiceNode("n", ros_node=fake)
    result = n.on_start()
    assert result == NodeStatus.FAILURE


def test_on_running_returns_success_and_calls_on_response():
    """on_running() returns SUCCESS and invokes on_response once when call completes."""
    fake = FakeRosNode()
    n = _SimpleServiceNode("n", ros_node=fake)
    # Pre-load a non-None response so the immediate fake client resolves to SUCCESS
    fake.service_clients  # ensure dict exists (populated lazily by create_client)
    n.on_start()
    fake.service_clients["/srv"].set_response("ok")
    # Re-issue the call so the pre-loaded response is in effect
    n.call_service(object())

    responses = []
    n.on_response = lambda resp: responses.append(resp)

    result = n.on_running()
    assert result == NodeStatus.SUCCESS
    assert len(responses) == 1


def test_on_response_called_only_once():
    """on_response is guarded by _response_delivered — calling on_running() twice
    must not invoke on_response a second time."""
    fake = FakeRosNode()
    n = _SimpleServiceNode("n", ros_node=fake)
    n.on_start()
    fake.service_clients["/srv"].set_response("ok")
    n.call_service(object())

    call_count = [0]

    def counting_on_response(resp):
        call_count[0] += 1

    n.on_response = counting_on_response

    n.on_running()  # first call — delivers response
    n.on_running()  # second call — must not call on_response again
    assert call_count[0] == 1


def test_call_timeout_attribute():
    """Subclass with call_timeout = 0.001: _svc_call_timeout is set correctly and
    service_status() returns FAILURE after the timeout elapses."""

    class _SlowServiceNode(RosServiceNode):
        service_type = object
        service_name = "/srv"
        call_timeout = 0.001

        def on_start(self) -> NodeStatus:
            self._init_service_client(self.service_type, self.service_name,
                                      call_timeout=self.call_timeout)
            if not self.service_is_ready():
                return NodeStatus.FAILURE
            self._response_delivered = False
            self.call_service(self.make_request())
            return NodeStatus.RUNNING

        def make_request(self):
            return object()

    fake = FakeRosNode()
    n = _SlowServiceNode("n", ros_node=fake)

    # Run on_start once so _init_service_client creates the lock and registers the
    # client under "/srv". The fake client resolves immediately, but that is fine —
    # we only care about the timeout path, which we exercise in the second call.
    n.on_start()

    # Now swap _svc_client for a deferred (never-resolving) mock.  The lock already
    # exists, so call_service() can acquire it safely.
    deferred = DeferredFuture("never")
    mock_client = MagicMock()
    mock_client.service_is_ready.return_value = True
    mock_client.call_async.return_value = deferred
    n._svc_client = mock_client

    # Reset _response_delivered and issue a fresh call through the slow mock
    n._response_delivered = False
    n.call_service(object())

    assert n._svc_call_timeout == 0.001

    time.sleep(0.01)
    assert n.on_running() == NodeStatus.FAILURE
