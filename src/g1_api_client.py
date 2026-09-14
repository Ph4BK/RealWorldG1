"""Concurrent, read-only client for the G1 locomotion RPC service.

The concurrency mechanism is deliberately independent of ROS 2 so it can be
unit-tested without a robot or a Unitree installation.  The public client only
exposes API 7001--7005.  Motion-control APIs are intentionally absent.
"""

from __future__ import annotations

import json
import threading
import time
from concurrent.futures import Future
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, Protocol


@dataclass(frozen=True)
class SportRequest:
    request_id: int
    api_id: int
    parameter: str = ""


@dataclass(frozen=True)
class SportResponse:
    request_id: int
    api_id: int
    status_code: int
    data: str = ""


class SportTransport(Protocol):
    """Minimal transport used by the request matcher."""

    def start(self, response_callback: Callable[[SportResponse], None]) -> None:
        ...

    def publish(self, request: SportRequest) -> None:
        ...

    def close(self) -> None:
        ...


class G1ApiError(RuntimeError):
    """The server returned a non-zero Unitree status code."""

    def __init__(self, api_id: int, status_code: int) -> None:
        self.api_id = api_id
        self.status_code = status_code
        detail = {
            7301: "LocoState not available",
            7302: "Invalid fsm id",
            7303: "Invalid task id",
        }.get(status_code, "unknown server error")
        super().__init__(
            f"G1 API {api_id} failed with status {status_code}: {detail}"
        )


class G1ProtocolError(RuntimeError):
    """A response did not conform to the expected Unitree protocol."""


class G1RequestTimeoutError(TimeoutError):
    """No matching response arrived before the request deadline."""


class G1ClientClosedError(RuntimeError):
    """The request client is closed."""


class G1TooManyPendingError(RuntimeError):
    """The configured in-flight request limit has been reached."""


@dataclass
class _PendingRequest:
    api_id: int
    future: Future
    decoder: Callable[[str], Any]
    timer: Optional[threading.Timer] = None


class _ConcurrentRequestManager:
    """Match one response subscription to multiple in-flight requests."""

    def __init__(self, transport: SportTransport, max_pending: int = 4) -> None:
        if max_pending < 1:
            raise ValueError("max_pending must be at least 1")
        self._transport = transport
        self._slots = threading.BoundedSemaphore(max_pending)
        self._lock = threading.Lock()
        self._pending_requests: Dict[int, _PendingRequest] = {}
        self._last_request_id = 0
        self._closed = False
        self._transport.start(self._on_response)

    @property
    def pending_count(self) -> int:
        with self._lock:
            return len(self._pending_requests)

    def submit(
        self,
        api_id: int,
        *,
        timeout: float,
        decoder: Callable[[str], Any],
        parameter: str = "",
    ) -> Future:
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        if not self._slots.acquire(blocking=False):
            raise G1TooManyPendingError("maximum number of pending G1 requests reached")

        future: Future = Future()
        with self._lock:
            if self._closed:
                self._slots.release()
                raise G1ClientClosedError("G1 request client is closed")
            request_id = max(time.monotonic_ns(), self._last_request_id + 1)
            self._last_request_id = request_id
            self._pending_requests[request_id] = _PendingRequest(
                api_id=api_id,
                future=future,
                decoder=decoder,
            )

        # Registration happens before publication so an immediate response
        # cannot beat insertion into the pending-request map.
        try:
            self._transport.publish(
                SportRequest(
                    request_id=request_id,
                    api_id=api_id,
                    parameter=parameter,
                )
            )
        except BaseException as exc:
            pending = self._pop_pending(request_id)
            if pending is not None and not pending.future.done():
                pending.future.set_exception(exc)
            return future

        timer = threading.Timer(timeout, self._on_timeout, args=(request_id,))
        timer.daemon = True
        with self._lock:
            pending = self._pending_requests.get(request_id)
            if pending is not None:
                pending.timer = timer
                timer.start()
        return future

    def _pop_pending(self, request_id: int) -> Optional[_PendingRequest]:
        with self._lock:
            pending = self._pending_requests.pop(request_id, None)
        if pending is not None:
            if pending.timer is not None:
                pending.timer.cancel()
            self._slots.release()
        return pending

    def _on_timeout(self, request_id: int) -> None:
        pending = self._pop_pending(request_id)
        if pending is not None and not pending.future.done():
            pending.future.set_exception(
                G1RequestTimeoutError(
                    f"G1 API {pending.api_id} request {request_id} timed out"
                )
            )

    def _on_response(self, response: SportResponse) -> None:
        pending = self._pop_pending(response.request_id)
        if pending is None:
            # Unknown, duplicate, or late response: it belongs to no active call.
            return
        if pending.future.done():
            return
        if response.api_id != pending.api_id:
            pending.future.set_exception(
                G1ProtocolError(
                    f"request {response.request_id} expected API {pending.api_id}, "
                    f"received API {response.api_id}"
                )
            )
            return
        if response.status_code != 0:
            pending.future.set_exception(
                G1ApiError(response.api_id, response.status_code)
            )
            return
        try:
            result = pending.decoder(response.data)
        except BaseException as exc:
            if isinstance(exc, G1ProtocolError):
                error = exc
            else:
                error = G1ProtocolError(
                    f"could not decode API {response.api_id} response: {exc}"
                )
            pending.future.set_exception(error)
        else:
            pending.future.set_result(result)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            request_ids = list(self._pending_requests)
        for request_id in request_ids:
            pending = self._pop_pending(request_id)
            if pending is not None and not pending.future.done():
                pending.future.set_exception(
                    G1ClientClosedError("G1 request client closed while awaiting response")
                )
        self._transport.close()


def _decode_data_field(payload: str) -> Any:
    if not payload:
        raise G1ProtocolError("successful query response contains no JSON data")
    try:
        document = json.loads(payload)
    except (TypeError, ValueError) as exc:
        raise G1ProtocolError(f"malformed JSON response: {exc}") from exc
    if not isinstance(document, dict) or "data" not in document:
        raise G1ProtocolError("query response JSON has no 'data' field")
    return document["data"]


class G1ReadOnlyLocoClient:
    """Read-only facade for the verified G1 locomotion query APIs."""

    _GET_FSM_ID = 7001
    _GET_FSM_MODE = 7002
    _GET_BALANCE_MODE = 7003
    _GET_SWING_HEIGHT = 7004
    _GET_STAND_HEIGHT = 7005

    def __init__(self, transport: SportTransport, max_pending: int = 4) -> None:
        self._requests = _ConcurrentRequestManager(transport, max_pending=max_pending)

    def _query_async(self, api_id: int, timeout: float) -> Future:
        return self._requests.submit(
            api_id,
            timeout=timeout,
            decoder=_decode_data_field,
        )

    def get_fsm_id_async(self, timeout: float = 5.0) -> Future:
        return self._query_async(self._GET_FSM_ID, timeout)

    def get_fsm_mode_async(self, timeout: float = 5.0) -> Future:
        return self._query_async(self._GET_FSM_MODE, timeout)

    def get_balance_mode_async(self, timeout: float = 5.0) -> Future:
        return self._query_async(self._GET_BALANCE_MODE, timeout)

    def get_swing_height_async(self, timeout: float = 5.0) -> Future:
        return self._query_async(self._GET_SWING_HEIGHT, timeout)

    def get_stand_height_async(self, timeout: float = 5.0) -> Future:
        return self._query_async(self._GET_STAND_HEIGHT, timeout)

    def get_fsm_id(self, timeout: float = 5.0) -> Any:
        return self.get_fsm_id_async(timeout).result()

    def get_fsm_mode(self, timeout: float = 5.0) -> Any:
        return self.get_fsm_mode_async(timeout).result()

    def get_balance_mode(self, timeout: float = 5.0) -> Any:
        return self.get_balance_mode_async(timeout).result()

    def get_swing_height(self, timeout: float = 5.0) -> Any:
        return self.get_swing_height_async(timeout).result()

    def get_stand_height(self, timeout: float = 5.0) -> Any:
        return self.get_stand_height_async(timeout).result()

    def close(self) -> None:
        self._requests.close()

    def __enter__(self) -> "G1ReadOnlyLocoClient":
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()
