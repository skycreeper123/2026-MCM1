import json
import math
import time
import unittest
from urllib.error import URLError

from B.q3.planner import Q3Action
from B.q3.runner import (
    ProtocolError,
    SimulatorClient,
    action_payload,
    validate_base_url,
    validate_response,
    validate_robot_id,
    build_parser,
)


class MemoryRecorder:
    def __init__(self):
        self.rows = []

    def record(self, event, **fields):
        self.rows.append((event, fields))


class FakeResponse:
    status = 200

    def __init__(self, body):
        self.body = json.dumps(body).encode("utf-8")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        return False

    def getcode(self):
        return self.status

    def read(self):
        return self.body


class RunnerValidationTests(unittest.TestCase):
    def test_strategy_argument_is_explicit_and_defaults_to_v3_global(self):
        parser = build_parser()
        self.assertEqual(parser.parse_args(["--self-check"]).strategy, "v3_global")
        self.assertEqual(
            parser.parse_args(["--self-check", "--strategy", "b0_batch_fifo"]).strategy,
            "b0_batch_fifo",
        )
    def test_robot_id_validation(self):
        self.assertEqual(validate_robot_id("20260001"), "20260001")
        for value in ("", "a" * 65, "bad\nvalue"):
            with self.assertRaises(ValueError):
                validate_robot_id(value)

    def test_only_loopback_base_url_is_allowed(self):
        self.assertEqual(validate_base_url("http://127.0.0.1:2026/"), "http://127.0.0.1:2026")
        for value in (
            "https://127.0.0.1:2026",
            "http://example.com:2026",
            "http://127.0.0.1:2026/path",
        ):
            with self.assertRaises(ValueError):
                validate_base_url(value)

    def test_action_payload_matches_attachment_protocol(self):
        action = Q3Action("MEASURE", (12.5, -8.0), 7, "test")
        self.assertEqual(
            action_payload("team", "req-1", action),
            {
                "arena_id": "default",
                "robot_id": "team",
                "request_id": "req-1",
                "position": {"x": 12.5, "y": -8.0},
                "channel": 7,
            },
        )

    def test_response_validation(self):
        validate_response(
            "/measure",
            {
                "accepted": True,
                "real_timestamp_ms": 1,
                "virtual_time_s": 5,
                "measure_result": "direction",
                "svd_deg": 123.45,
            },
        )
        with self.assertRaises(ProtocolError):
            validate_response(
                "/measure",
                {
                    "accepted": True,
                    "real_timestamp_ms": 1,
                    "virtual_time_s": 5,
                    "measure_result": "direction",
                    "svd_deg": math.nan,
                },
            )

    def test_transport_retry_reuses_exact_body_and_request_id(self):
        calls = []

        def opener(request, timeout):
            calls.append((request.full_url, request.data, timeout))
            if len(calls) == 1:
                raise URLError("temporary disconnect")
            return FakeResponse(
                {
                    "accepted": True,
                    "real_timestamp_ms": 1,
                    "virtual_time_s": 0,
                    "max_virtual_duration_s": 360000,
                    "max_real_duration_s": 1200,
                    "remaining_real_duration_s": 1200,
                }
            )

        recorder = MemoryRecorder()
        client = SimulatorClient(
            "http://127.0.0.1:2026",
            recorder,
            retry_initial_s=0.001,
            retry_max_s=0.001,
            opener=opener,
        )
        prepared = client.prepare(
            "/enter",
            {"arena_id": "default", "robot_id": "team", "request_id": "same-id"},
        )
        response = client.send(prepared, time.monotonic() + 1)
        self.assertTrue(response["accepted"])
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[0][0], calls[1][0])
        self.assertEqual(calls[0][1], calls[1][1])


if __name__ == "__main__":
    unittest.main()
