"""Independent Q4 Robot API entry; does not start or change simulator modes."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
import uuid

from B.q3.runner import (JsonlRecorder, SimulatorClient, RequestIdSource, RunnerError,
                         TransportError, ProtocolError, base_payload, action_payload,
                         validate_robot_id, validate_base_url, print_progress)
from .planner import Q4Config, Q4Planner


def run_planner(robot_id, base_url="http://127.0.0.1:2026", log_dir=None,
                connect_wait_s=60, timeout_s=5, confirm_q4=False, config=None):
    if not confirm_q4:
        raise ValueError("Select Question 4 in the simulator, then explicitly confirm_q4")
    robot_id, base_url = validate_robot_id(robot_id), validate_base_url(base_url)
    config = config or Q4Config()
    # Load/verify geometry before entering the timed arena.
    planner = Q4Planner(config)
    directory = Path(log_dir) if log_dir else Path(__file__).with_name("runs")
    path = directory / f"q4-{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:8]}.jsonl"
    recorder = JsonlRecorder(path)
    client = SimulatorClient(base_url, recorder, timeout_s=timeout_s)
    ids = RequestIdSource()
    ids.prefix = ids.prefix.replace("q3-", "q4-", 1)
    entered, exited, completed, uncertain = False, False, False, False
    deadline, error, mode_verification = None, None, "operator_confirmed; API may not expose mode"
    actual_virtual_time_s = None
    try:
        recorder.record("run_started", config=asdict(config), mode_verification=mode_verification)
        enter_started = time.monotonic()
        request = client.prepare("/enter", base_payload(robot_id, ids.next("enter")))
        response = client.send(request, enter_started+connect_wait_s)
        entered = True
        actual_virtual_time_s = response.get("virtual_time_s")
        # Conservative: subtract the full enter round-trip from the received
        # remaining duration rather than assuming an instantaneous response.
        deadline = enter_started+response["remaining_real_duration_s"]
        if "question" in response:
            if str(response["question"]).lower() not in {"4", "q4"}:
                raise ValueError("Simulator reported a mode other than Q4")
            mode_verification = "API question=4"
        index = 0
        while True:
            planning_deadline = deadline-config.exit_reserve_s
            if time.monotonic() >= planning_deadline:
                planner.mark_time_budget_incomplete()
                action = None
            else:
                action = planner.propose_action(planning_deadline)
                # Planning itself consumes real time. Never send a stale next
                # action after consuming the exit reserve.
                if time.monotonic() >= planning_deadline:
                    planner.mark_time_budget_incomplete()
                    action = None
            index += 1
            if action is None or action.kind == "EXIT":
                completed = planner.has_completion_certificate()
                request = client.prepare("/exit", base_payload(robot_id, ids.next("exit")))
                response = client.send(request, deadline)
                actual_virtual_time_s = response.get("virtual_time_s", actual_virtual_time_s)
                exited = True
                if action is not None:
                    planner.apply_exit_result(action)
                if not completed:
                    error = planner.stop_reason or "Exit without completion evidence"
                break
            endpoint = "/measure" if action.kind == "MEASURE" else "/clear"
            if action.kind == "CLEAR" and planner.clear_index == 0:
                recorder.record("clear_certificate", channel=action.channel,
                                certificate=planner.channels[action.channel].clear_certificate)
            request = client.prepare(endpoint, action_payload(robot_id, ids.next(action.kind), action))
            response = client.send(request, planning_deadline)
            actual_virtual_time_s = response.get("virtual_time_s", actual_virtual_time_s)
            if action.kind == "MEASURE":
                planner.apply_measure_result(action, response["measure_result"], response.get("svd_deg"))
            else:
                planner.apply_clear_result(action, response["clear_result"])
            recorder.record("planner_advanced", action=action.as_dict(), summary=planner.summary())
            print_progress(index, action, response, planner)
    except (RunnerError, ValueError, ArithmeticError, KeyboardInterrupt) as exc:
        error = f"{type(exc).__name__}: {exc}"
        uncertain = isinstance(exc, (TransportError, KeyboardInterrupt, ProtocolError))
        recorder.record("run_failed", error=error, unresolved_request=uncertain)
        # An unknown in-flight outcome must not be followed by a different
        # state-changing request. Transport retries already used the same ID.
        if entered and deadline is not None and not uncertain:
            try:
                response = client.send(client.prepare("/exit", base_payload(robot_id, ids.next("exit-error"))), deadline)
                exited = True
            except RunnerError as exit_error:
                recorder.record("exit_failed", error=str(exit_error))
    finally:
        final = {"task_completed": completed and exited and error is None,
                 "exit_accepted": exited, "entered": entered, "error": error,
                 "unresolved_request": uncertain, "mode_verification": mode_verification,
                 "actual_virtual_time_s": actual_virtual_time_s,
                 "planner": planner.summary(), "log_path": str(path)}
        recorder.record("run_finished", **final)
        recorder.close()
        path.with_suffix(".summary.json").write_text(json.dumps(final, ensure_ascii=False, indent=2), encoding="utf-8")
    return final


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument("--confirm-q4", action="store_true", help="Operator confirms that Q4 is selected")
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument("--connect-wait-s", type=float, default=60)
    args = parser.parse_args(argv)
    if not args.confirm_q4:
        parser.error("Select Q4 manually and supply --confirm-q4; enter may not report its mode")
    result = run_planner(**vars(args))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["task_completed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
