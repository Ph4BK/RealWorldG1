import threading
import time
import unittest

from RealWorldG1.src.g1_api_client import (
    G1ApiError,
    G1ClientClosedError,
    G1ProtocolError,
    G1ReadOnlyLocoClient,
    G1RequestTimeoutError,
    G1TooManyPendingError,
    SportResponse,
)


class FakeTransport:
    def __init__(self):
        self.callback = None
        self.requests = []
        self.closed = False
        self.on_publish = None

    def start(self, response_callback):
        self.callback = response_callback

    def publish(self, request):
        self.requests.append(request)
        if self.on_publish is not None:
            self.on_publish(request)

    def respond(self, request, *, api_id=None, status=0, data='{"data": 1}'):
        self.callback(
            SportResponse(
                request_id=request.request_id,
                api_id=request.api_id if api_id is None else api_id,
                status_code=status,
                data=data,
            )
        )

    def close(self):
        self.closed = True


class G1ApiClientTests(unittest.TestCase):
    def setUp(self):
        self.transport = FakeTransport()
        self.client = G1ReadOnlyLocoClient(self.transport, max_pending=4)

    def tearDown(self):
        self.client.close()

    def test_pending_is_registered_before_publish(self):
        self.transport.on_publish = lambda request: self.transport.respond(
            request, data='{"data": 42}'
        )
        self.assertEqual(self.client.get_fsm_id(), 42)

    def test_out_of_order_responses(self):
        first = self.client.get_fsm_id_async()
        second = self.client.get_fsm_mode_async()
        request1, request2 = self.transport.requests
        self.transport.respond(request2, data='{"data": "mode"}')
        self.transport.respond(request1, data='{"data": 7}')
        self.assertEqual(second.result(), "mode")
        self.assertEqual(first.result(), 7)

    def test_concurrent_same_api_has_distinct_request_ids(self):
        first = self.client.get_fsm_id_async()
        second = self.client.get_fsm_id_async()
        request1, request2 = self.transport.requests
        self.assertNotEqual(request1.request_id, request2.request_id)
        self.transport.respond(request2, data='{"data": 2}')
        self.transport.respond(request1, data='{"data": 1}')
        self.assertEqual((first.result(), second.result()), (1, 2))

    def test_unknown_and_duplicate_responses_are_ignored(self):
        future = self.client.get_fsm_id_async()
        request = self.transport.requests[-1]
        self.transport.callback(SportResponse(123, 7001, 0, '{"data": 9}'))
        self.assertFalse(future.done())
        self.transport.respond(request, data='{"data": 3}')
        self.transport.respond(request, data='{"data": 99}')
        self.assertEqual(future.result(), 3)

    def test_api_id_mismatch_is_protocol_error(self):
        future = self.client.get_fsm_id_async()
        self.transport.respond(self.transport.requests[-1], api_id=7002)
        with self.assertRaises(G1ProtocolError):
            future.result()

    def test_nonzero_status_is_api_error(self):
        future = self.client.get_balance_mode_async()
        self.transport.respond(self.transport.requests[-1], status=1203)
        with self.assertRaises(G1ApiError) as caught:
            future.result()
        self.assertEqual(caught.exception.status_code, 1203)

    def test_malformed_or_missing_json_data_is_protocol_error(self):
        for data in ("not-json", "{}", ""):
            with self.subTest(data=data):
                future = self.client.get_swing_height_async()
                self.transport.respond(self.transport.requests[-1], data=data)
                with self.assertRaises(G1ProtocolError):
                    future.result()

    def test_timeout_cleans_pending_and_late_response_is_ignored(self):
        future = self.client.get_fsm_id_async(timeout=0.03)
        request = self.transport.requests[-1]
        with self.assertRaises(G1RequestTimeoutError):
            future.result(timeout=0.5)
        self.transport.respond(request, data='{"data": 8}')
        replacement = self.client.get_fsm_id_async()
        self.transport.respond(self.transport.requests[-1], data='{"data": 9}')
        self.assertEqual(replacement.result(), 9)

    def test_max_concurrency_is_enforced_and_slot_is_reused(self):
        limited_transport = FakeTransport()
        limited = G1ReadOnlyLocoClient(limited_transport, max_pending=1)
        self.addCleanup(limited.close)
        first = limited.get_fsm_id_async()
        with self.assertRaises(G1TooManyPendingError):
            limited.get_fsm_mode_async()
        limited_transport.respond(limited_transport.requests[-1])
        self.assertEqual(first.result(), 1)
        second = limited.get_fsm_mode_async()
        limited_transport.respond(limited_transport.requests[-1], data='{"data": 2}')
        self.assertEqual(second.result(), 2)

    def test_close_fails_all_pending_requests(self):
        futures = [
            self.client.get_fsm_id_async(),
            self.client.get_fsm_mode_async(),
        ]
        self.client.close()
        self.assertTrue(self.transport.closed)
        for future in futures:
            with self.assertRaises(G1ClientClosedError):
                future.result()
        with self.assertRaises(G1ClientClosedError):
            self.client.get_fsm_id_async()

    def test_threaded_callers_can_wait_simultaneously(self):
        results = {}

        def call(name, method):
            results[name] = method(timeout=1.0)

        threads = [
            threading.Thread(target=call, args=("fsm", self.client.get_fsm_id)),
            threading.Thread(target=call, args=("mode", self.client.get_fsm_mode)),
        ]
        for thread in threads:
            thread.start()
        deadline = time.monotonic() + 0.5
        while len(self.transport.requests) < 2 and time.monotonic() < deadline:
            time.sleep(0.001)
        for request in reversed(self.transport.requests):
            self.transport.respond(request, data=f'{{"data": {request.api_id}}}')
        for thread in threads:
            thread.join(timeout=1.0)
        self.assertEqual(results, {"fsm": 7001, "mode": 7002})


if __name__ == "__main__":
    unittest.main()
