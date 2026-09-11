from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from B.q3.runner import TransportError
from B.q4.planner import Q4Action
from B.q4.runner import run_planner


class FakePlanner:
    def __init__(self, action, on_propose=lambda: None):
        self.action, self.on_propose = action, on_propose
        self.stop_reason = None

    def propose_action(self, deadline):
        self.on_propose()
        return self.action

    def has_completion_certificate(self):
        return self.action.kind == "EXIT" and self.stop_reason is None

    def mark_time_budget_incomplete(self):
        self.stop_reason = "INCOMPLETE: reserve"

    def apply_exit_result(self, action):
        pass

    def summary(self):
        return {"cleared_count": 0, "stop_reason": self.stop_reason}


class FakeClient:
    def __init__(self, mode=4, fail_measure=False):
        self.paths, self.mode, self.fail_measure = [], mode, fail_measure

    def prepare(self, path, payload):
        return path, payload

    def send(self, request, deadline):
        path, _ = request
        self.paths.append(path)
        if path == "/enter":
            return {"remaining_real_duration_s": 100, "question": self.mode}
        if path == "/measure" and self.fail_measure:
            raise TransportError("unresolved same-ID request")
        return {"exit_reason": "requested", "accepted": True}


class RunnerTests(unittest.TestCase):
    def run_fake(self, planner, client, clock=None):
        with tempfile.TemporaryDirectory() as directory:
            with patch("B.q4.runner.Q4Planner", return_value=planner), patch("B.q4.runner.SimulatorClient", return_value=client):
                if clock is None:
                    return run_planner("test", log_dir=Path(directory), confirm_q4=True)
                with patch("B.q4.runner.time.monotonic", side_effect=lambda: clock[0]):
                    return run_planner("test", log_dir=Path(directory), confirm_q4=True)

    def test_mode_confirmation_required_before_enter(self):
        with self.assertRaises(ValueError):
            run_planner("test")

    def test_complete_exit_distinct_from_accepted_exit(self):
        planner, client = FakePlanner(Q4Action(1, "EXIT")), FakeClient()
        result = self.run_fake(planner, client)
        self.assertTrue(result["task_completed"])
        self.assertTrue(result["exit_accepted"])
        self.assertEqual(client.paths, ["/enter", "/exit"])

    def test_planning_exhausts_reserve_no_stale_measurement_sent(self):
        clock = [100.0]
        def slow_planning():
            clock[0] = 171
        planner = FakePlanner(Q4Action(1, "MEASURE", (0, 0), 1), slow_planning)
        client = FakeClient()
        result = self.run_fake(planner, client, clock)
        self.assertEqual(client.paths, ["/enter", "/exit"])
        self.assertFalse(result["task_completed"])
        self.assertTrue(result["exit_accepted"])

    def test_unknown_measurement_outcome_does_not_send_different_request(self):
        planner = FakePlanner(Q4Action(1, "MEASURE", (0, 0), 1))
        client = FakeClient(fail_measure=True)
        result = self.run_fake(planner, client)
        self.assertEqual(client.paths, ["/enter", "/measure"])
        self.assertTrue(result["unresolved_request"])
        self.assertFalse(result["task_completed"])

    def test_reported_wrong_mode_never_measures(self):
        planner, client = FakePlanner(Q4Action(1, "MEASURE", (0, 0), 1)), FakeClient(mode=3)
        result = self.run_fake(planner, client)
        self.assertNotIn("/measure", client.paths)
        self.assertFalse(result["task_completed"])


if __name__ == "__main__":
    unittest.main()
