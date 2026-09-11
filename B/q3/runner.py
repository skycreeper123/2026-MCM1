"""Serial HTTP runner for the Question 3 planner.

The runner does not start a practice or formal test in the simulator UI. It
waits for the operator-selected test to open the loopback Robot API.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import socket
import sys
import time
import unicodedata
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
import uuid

from .planner import (
    SUPPORTED_STRATEGIES,
    Q3Action,
    Q3Config,
    Q3Planner,
    q3_baseline_upper_bounds,
    q3_v2_upper_bounds,
)


RETRYABLE_HTTP_STATUS = frozenset({429, 500, 502, 503, 504})


class RunnerError(RuntimeError):
    """Base class for controlled runner failures."""


class TransportError(RunnerError):
    """A request outcome stayed unknown until its retry deadline."""


class ProtocolError(RunnerError):
    """The simulator returned an invalid or unexpected response."""


class RequestRejected(RunnerError):
    """The simulator returned accepted=false."""


def _json_safe(value):
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if hasattr(value, "value"):
        return value.value
    return value


class JsonlRecorder:
    """Append-only local execution log, flushed after every record."""

    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = path.open("a", encoding="utf-8", newline="\n")

    def record(self, event: str, **fields):
        row = {
            "event": event,
            "recorded_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            **_json_safe(fields),
        }
        self._stream.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")))
        self._stream.write("\n")
        self._stream.flush()

    def close(self):
        self._stream.close()


def validate_robot_id(robot_id: str) -> str:
    if not isinstance(robot_id, str):
        raise ValueError("robot_id must be a string")
    encoded = robot_id.encode("utf-8")
    if not 1 <= len(encoded) <= 64:
        raise ValueError("robot_id must contain 1 to 64 UTF-8 bytes")
    if any(unicodedata.category(char) in {"Cc", "Cf"} for char in robot_id):
        raise ValueError("robot_id cannot contain control or format characters")
    return robot_id


def validate_base_url(base_url: str) -> str:
    parsed = urlsplit(base_url)
    if parsed.scheme != "http" or parsed.hostname not in {"127.0.0.1", "localhost", "::1"}:
        raise ValueError("base URL must be an HTTP loopback address")
    if parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise ValueError("base URL cannot contain credentials, a query, or a fragment")
    if parsed.path not in {"", "/"}:
        raise ValueError("base URL cannot contain a path")
    try:
        port = parsed.port
    except ValueError as exc:
        raise ValueError("base URL contains an invalid port") from exc
    if port is None or not 1 <= port <= 65535:
        raise ValueError("base URL must include a valid port")
    return base_url.rstrip("/")


def validate_response(path: str, response):
    if not isinstance(response, dict):
        raise ProtocolError(f"{path}: response must be a JSON object")
    accepted = response.get("accepted")
    if not isinstance(accepted, bool):
        raise ProtocolError(f"{path}: accepted must be boolean")
    for field in ("real_timestamp_ms", "virtual_time_s"):
        value = response.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ProtocolError(f"{path}: {field} must be a finite number")
    if not accepted:
        if response["virtual_time_s"] != 0:
            raise ProtocolError(f"{path}: rejected response must have virtual_time_s=0")
        return response
    if path == "/enter":
        for field in ("max_virtual_duration_s", "max_real_duration_s", "remaining_real_duration_s"):
            value = response.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ProtocolError(f"{path}: {field} must be a finite number")
        if not 0 <= response["remaining_real_duration_s"] <= response["max_real_duration_s"]:
            raise ProtocolError(f"{path}: invalid remaining_real_duration_s")
    elif path == "/measure":
        result = response.get("measure_result")
        if result not in {"no_signal", "near", "direction"}:
            raise ProtocolError(f"{path}: invalid measure_result")
        if result == "direction":
            bearing = response.get("svd_deg")
            if isinstance(bearing, bool) or not isinstance(bearing, (int, float)) or not math.isfinite(bearing):
                raise ProtocolError(f"{path}: direction requires finite svd_deg")
        elif "svd_deg" in response:
            raise ProtocolError(f"{path}: svd_deg is only valid for direction")
    elif path == "/clear":
        if response.get("clear_result") not in {"success", "no_target_in_range"}:
            raise ProtocolError(f"{path}: invalid clear_result")
    elif path == "/exit":
        if not isinstance(response.get("exit_reason"), str) or not response["exit_reason"]:
            raise ProtocolError(f"{path}: exit_reason must be a non-empty string")
    return response


@dataclass(frozen=True)
class PreparedRequest:
    path: str
    payload: dict
    body: bytes


class SimulatorClient:
    """Strict serial client with same-ID/same-body transport retries."""

    def __init__(
        self,
        base_url: str,
        recorder,
        timeout_s: float = 5.0,
        retry_initial_s: float = 0.25,
        retry_max_s: float = 2.0,
        opener=urlopen,
    ):
        self.base_url = validate_base_url(base_url)
        self.recorder = recorder
        self.timeout_s = timeout_s
        self.retry_initial_s = retry_initial_s
        self.retry_max_s = retry_max_s
        self._opener = opener
        self._last_accepted_virtual_time = None

    def prepare(self, path: str, payload: dict) -> PreparedRequest:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
        if len(body) > 65536:
            raise ValueError("request JSON exceeds 65536 bytes")
        prepared = PreparedRequest(path, payload, body)
        self.recorder.record(
            "request_prepared",
            path=path,
            payload=payload,
            body_sha256=hashlib.sha256(body).hexdigest(),
        )
        return prepared

    def send(self, prepared: PreparedRequest, deadline_monotonic: float):
        delay = self.retry_initial_s
        attempt = 0
        while True:
            attempt += 1
            remaining = deadline_monotonic - time.monotonic()
            if remaining <= 0:
                raise TransportError(f"{prepared.path}: retry deadline expired")
            request = Request(
                self.base_url + prepared.path,
                data=prepared.body,
                headers={"Content-Type": "application/json; charset=utf-8"},
                method="POST",
            )
            try:
                with self._opener(request, timeout=min(self.timeout_s, remaining)) as http_response:
                    status = int(getattr(http_response, "status", http_response.getcode()))
                    raw = http_response.read()
            except HTTPError as exc:
                status = int(exc.code)
                raw = exc.read()
                self.recorder.record(
                    "http_error",
                    path=prepared.path,
                    request_id=prepared.payload.get("request_id"),
                    attempt=attempt,
                    http_status=status,
                    response_body=raw.decode("utf-8", errors="replace"),
                )
                if status in RETRYABLE_HTTP_STATUS and time.monotonic() + delay < deadline_monotonic:
                    time.sleep(delay)
                    delay = min(self.retry_max_s, delay * 2)
                    continue
                raise ProtocolError(f"{prepared.path}: HTTP {status}") from exc
            except (URLError, TimeoutError, socket.timeout, ConnectionError, OSError) as exc:
                self.recorder.record(
                    "transport_retry",
                    path=prepared.path,
                    request_id=prepared.payload.get("request_id"),
                    attempt=attempt,
                    error_type=type(exc).__name__,
                    error=str(exc),
                )
                if time.monotonic() + delay >= deadline_monotonic:
                    raise TransportError(f"{prepared.path}: transport outcome remains unknown") from exc
                time.sleep(delay)
                delay = min(self.retry_max_s, delay * 2)
                continue

            try:
                response = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                self.recorder.record(
                    "invalid_response",
                    path=prepared.path,
                    request_id=prepared.payload.get("request_id"),
                    attempt=attempt,
                    http_status=status,
                    response_body=raw.decode("utf-8", errors="replace"),
                )
                raise ProtocolError(f"{prepared.path}: response is not valid UTF-8 JSON") from exc
            if status != 200:
                raise ProtocolError(f"{prepared.path}: unexpected HTTP {status}")
            validate_response(prepared.path, response)
            self.recorder.record(
                "response_received",
                path=prepared.path,
                request_id=prepared.payload.get("request_id"),
                attempt=attempt,
                http_status=status,
                response=response,
            )
            if not response["accepted"]:
                raise RequestRejected(f"{prepared.path}: simulator returned accepted=false")
            virtual_time = float(response["virtual_time_s"])
            if self._last_accepted_virtual_time is not None and virtual_time < self._last_accepted_virtual_time:
                raise ProtocolError(f"{prepared.path}: virtual time moved backwards")
            self._last_accepted_virtual_time = virtual_time
            return response


class RequestIdSource:
    def __init__(self):
        stamp = time.strftime("%Y%m%dT%H%M%S", time.localtime())
        self.prefix = f"q3-{stamp}-{uuid.uuid4().hex[:8]}"
        self.counter = 0

    def next(self, action: str) -> str:
        self.counter += 1
        request_id = f"{self.prefix}-{self.counter:05d}-{action.lower()}"
        if len(request_id.encode("utf-8")) > 128:
            raise AssertionError("generated request_id is too long")
        return request_id


def base_payload(robot_id: str, request_id: str):
    return {"arena_id": "default", "robot_id": robot_id, "request_id": request_id}


def action_payload(robot_id: str, request_id: str, action: Q3Action):
    if action.kind not in {"MEASURE", "CLEAR"} or action.position is None or action.channel is None:
        raise ValueError("action must be MEASURE or CLEAR with position and channel")
    payload = base_payload(robot_id, request_id)
    payload["position"] = {"x": action.position[0], "y": action.position[1]}
    payload["channel"] = action.channel
    return payload


def print_progress(action_index: int, action: Q3Action, response: dict, planner: Q3Planner):
    summary = planner.summary()
    if action.kind == "MEASURE":
        outcome = response["measure_result"]
        if outcome == "direction":
            outcome += f"({response['svd_deg']:.2f}deg)"
    elif action.kind == "CLEAR":
        outcome = response["clear_result"]
    else:
        outcome = response["exit_reason"]
    print(
        f"[{action_index:04d}] {action.kind:<7} ch={action.channel or '-':>2} "
        f"outcome={outcome:<24} virtual={response['virtual_time_s']}s "
        f"cleared={summary['cleared_count']}",
        flush=True,
    )


def run_planner(robot_id, base_url, log_dir, connect_wait_s, exit_reserve_s, timeout_s,
                strategy="v2_local"):
    robot_id = validate_robot_id(robot_id)
    run_stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    log_path = log_dir / f"q3-{run_stamp}-{uuid.uuid4().hex[:8]}.jsonl"
    summary_path = log_path.with_suffix(".summary.json")
    recorder = JsonlRecorder(log_path)
    planner = Q3Planner(Q3Config(strategy=strategy))
    ids = RequestIdSource()
    client = SimulatorClient(base_url, recorder, timeout_s=timeout_s)
    entered = False
    exit_accepted = False
    task_completed = False
    exit_response = None
    run_error = None
    action_index = 0
    real_deadline = None

    recorder.record("run_started", robot_id=robot_id, base_url=base_url, planner_config=planner.config.__dict__)
    print(f"Local JSONL log: {log_path}", flush=True)
    print(f"Waiting up to {connect_wait_s:g}s for the simulator Robot API...", flush=True)
    try:
        enter_request = client.prepare("/enter", base_payload(robot_id, ids.next("enter")))
        enter_response = client.send(enter_request, time.monotonic() + connect_wait_s)
        entered = True
        remaining_real_s = float(enter_response["remaining_real_duration_s"])
        real_deadline = time.monotonic() + remaining_real_s
        recorder.record("entered", remaining_real_duration_s=remaining_real_s)
        print(f"/enter accepted; remaining real time: {remaining_real_s:g}s", flush=True)

        while True:
            planning_deadline = real_deadline - exit_reserve_s
            if time.monotonic() >= planning_deadline:
                completion_certificate = planner.has_completion_certificate()
                planner.mark_time_budget_incomplete()
                recorder.record("exit_reserve_reached", planner_summary=planner.summary())
                exit_request = client.prepare("/exit", base_payload(robot_id, ids.next("exit-reserve")))
                exit_response = client.send(exit_request, real_deadline)
                exit_accepted = True
                task_completed = completion_certificate
                if not task_completed:
                    run_error = f"real-time reserve of {exit_reserve_s:g}s reached before completion"
                break
            action = planner.propose_action(deadline_monotonic=planning_deadline)
            if action is None:
                raise ProtocolError("planner exited without an EXIT action")
            action_index += 1

            if action.kind == "EXIT":
                completion_certificate = planner.has_completion_certificate()
                prepared = client.prepare("/exit", base_payload(robot_id, ids.next("exit")))
                response = client.send(prepared, real_deadline)
                planner.apply_exit_result(action)
                exit_response = response
                exit_accepted = True
                task_completed = completion_certificate
                if not task_completed:
                    run_error = f"planner exited without completion certificate: {action.reason}"
            elif action.kind == "MEASURE":
                prepared = client.prepare("/measure", action_payload(robot_id, ids.next("measure"), action))
                response = client.send(prepared, real_deadline)
                result = response["measure_result"]
                planner.apply_measure_result(
                    action,
                    result,
                    svd_deg=response.get("svd_deg") if result == "direction" else None,
                )
            elif action.kind == "CLEAR":
                prepared = client.prepare("/clear", action_payload(robot_id, ids.next("clear"), action))
                response = client.send(prepared, real_deadline)
                planner.apply_clear_result(action, response["clear_result"])
            else:
                raise ProtocolError(f"unsupported planner action {action.kind!r}")

            recorder.record(
                "planner_advanced",
                action_index=action_index,
                action=action.as_dict(),
                planner_summary=planner.summary(),
            )
            print_progress(action_index, action, response, planner)
            if exit_accepted:
                break
    except KeyboardInterrupt:
        run_error = "operator interrupted runner; last in-flight request outcome may be unknown"
        recorder.record("run_interrupted", error=run_error)
    except RunnerError as exc:
        run_error = f"{type(exc).__name__}: {exc}"
        recorder.record("run_failed", error=run_error, planner_summary=planner.summary())
        if entered and real_deadline is not None and not isinstance(exc, TransportError):
            try:
                emergency = client.prepare("/exit", base_payload(robot_id, ids.next("exit-error")))
                exit_response = client.send(emergency, real_deadline)
                exit_accepted = True
                recorder.record("emergency_exit_succeeded", response=exit_response)
            except RunnerError as exit_exc:
                recorder.record("emergency_exit_failed", error=f"{type(exit_exc).__name__}: {exit_exc}")
    finally:
        final = {
            "completed": task_completed,
            "task_completed": task_completed,
            "exit_accepted": exit_accepted,
            "entered": entered,
            "error": run_error,
            "exit_response": exit_response,
            "planner_summary": planner.summary(),
            "action_count": action_index,
            "jsonl_log": str(log_path),
        }
        recorder.record("run_finished", **final)
        recorder.close()
        summary_path.write_text(
            json.dumps(_json_safe(final), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(f"Run summary: {summary_path}", flush=True)

    if task_completed:
        print("Question 3 planner completed and /exit was accepted.", flush=True)
        return 0
    print(f"Practice run did not complete: {run_error}", file=sys.stderr, flush=True)
    return 2


def build_parser():
    parser = argparse.ArgumentParser(description="Run Question 3 against the local simulator")
    parser.add_argument("--robot-id", help="current logged-in team number; prompts if omitted")
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument("--log-dir", type=Path, default=Path(__file__).with_name("runs"))
    parser.add_argument("--connect-wait-s", type=float, default=180.0)
    parser.add_argument("--exit-reserve-s", type=float, default=30.0)
    parser.add_argument("--timeout-s", type=float, default=5.0)
    parser.add_argument(
        "--strategy", choices=SUPPORTED_STRATEGIES, default="v2_local",
        help="b0_serial, b0_batch_fifo, or cumulative v2_local",
    )
    parser.add_argument("--self-check", action="store_true", help="validate without networking")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.self_check:
        bounds = (q3_v2_upper_bounds(Q3Config(strategy=args.strategy))
                  if args.strategy == "v2_local"
                  else q3_baseline_upper_bounds(Q3Config(strategy=args.strategy)))
        print(json.dumps({"strategy": args.strategy, "bounds": bounds},
                         ensure_ascii=False, indent=2))
        return 0
    if args.connect_wait_s <= 0 or args.exit_reserve_s < 0 or args.timeout_s <= 0:
        parser.error("timeouts must be positive and exit reserve must be non-negative")
    robot_id = args.robot_id
    if robot_id is None:
        robot_id = input("请输入当前登录的参赛队号：").strip()
    try:
        return run_planner(
            robot_id=robot_id,
            base_url=validate_base_url(args.base_url),
            log_dir=args.log_dir,
            connect_wait_s=args.connect_wait_s,
            exit_reserve_s=args.exit_reserve_s,
            timeout_s=args.timeout_s,
            strategy=args.strategy,
        )
    except ValueError as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
