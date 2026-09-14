"""ROS 2 transport for the G1 sport request/response topics.

Imports are intentionally local to ``start`` so the concurrency core remains
usable and testable on development machines without Unitree ROS 2 installed.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from .g1_api_client import SportRequest, SportResponse


class Ros2SportTransport:
    REQUEST_TOPIC = "/api/sport/request"
    RESPONSE_TOPIC = "/api/sport/response"

    def __init__(
        self,
        node_name: str = "unigoal_g1_read_only",
        depth: int = 16,
        endpoint_timeout: float = 5.0,
    ) -> None:
        if depth < 1:
            raise ValueError("depth must be at least 1")
        self._node_name = node_name
        self._depth = depth
        self._endpoint_timeout = endpoint_timeout
        self._publish_lock = threading.Lock()
        self._started = False
        self._closed = False
        self._context = None
        self._node = None
        self._publisher = None
        self._subscription = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._request_type = None
        self._rclpy = None

    def start(self, response_callback: Callable[[SportResponse], None]) -> None:
        if self._started:
            raise RuntimeError("ROS 2 sport transport is already started")
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from unitree_api.msg import Request, Response
        except ImportError as exc:
            raise RuntimeError(
                "Ros2SportTransport requires rclpy and unitree_api; run it in "
                "the verified G1 Unitree ROS 2 container"
            ) from exc

        context = Context()
        rclpy.init(args=None, context=context)
        try:
            node = Node(self._node_name, context=context)
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=self._depth,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )

            def on_ros_response(message: Response) -> None:
                response_callback(
                    SportResponse(
                        request_id=int(message.header.identity.id),
                        api_id=int(message.header.identity.api_id),
                        status_code=int(message.header.status.code),
                        data=message.data,
                    )
                )

            publisher = node.create_publisher(Request, self.REQUEST_TOPIC, qos)
            subscription = node.create_subscription(
                Response, self.RESPONSE_TOPIC, on_ros_response, qos
            )
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
        except BaseException:
            rclpy.shutdown(context=context)
            raise

        self._rclpy = rclpy
        self._context = context
        self._node = node
        self._publisher = publisher
        # Keep a strong reference.  It also lets startup verify DDS discovery
        # before the first request is published.
        self._subscription = subscription
        self._executor = executor
        self._request_type = Request
        self._started = True
        self._spin_thread = threading.Thread(
            target=executor.spin,
            name="g1-sport-response-executor",
            daemon=True,
        )
        self._spin_thread.start()
        deadline = time.monotonic() + self._endpoint_timeout
        while time.monotonic() < deadline:
            if (
                publisher.get_subscription_count() > 0
                and node.count_publishers(self.RESPONSE_TOPIC) > 0
            ):
                return
            time.sleep(0.05)
        request_readers = publisher.get_subscription_count()
        response_writers = node.count_publishers(self.RESPONSE_TOPIC)
        self.close()
        raise RuntimeError(
            "G1 sport DDS endpoints were not discovered before startup timeout "
            f"(request readers={request_readers}, response writers={response_writers})"
        )

    def publish(self, request: SportRequest) -> None:
        if not self._started or self._closed:
            raise RuntimeError("ROS 2 sport transport is not running")
        message = self._request_type()
        message.header.identity.id = request.request_id
        message.header.identity.api_id = request.api_id
        message.header.lease.id = 0
        message.header.policy.priority = 0
        message.header.policy.noreply = False
        message.parameter = request.parameter
        message.binary = []
        with self._publish_lock:
            self._publisher.publish(message)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if not self._started:
            return
        self._executor.shutdown(timeout_sec=2.0)
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        self._node.destroy_node()
        self._rclpy.shutdown(context=self._context)
