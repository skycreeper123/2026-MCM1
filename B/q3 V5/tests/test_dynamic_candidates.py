import time
import unittest
from unittest.mock import patch

import numpy as np

from B.q2 import is_safe_candidate
from B.q3 import Q3Config, Q3Planner
from B.q3.localize import score_dynamic_detection_points


class DynamicCandidateTests(unittest.TestCase):
    def planner(self, strategy="v5_dynamic"):
        planner = Q3Planner(Q3Config(strategy=strategy))
        record = planner.channels[1]
        record.outer_vertices = np.array([[-50., -50.], [50., -50.], [50., 50.], [-50., 50.]])
        record.anchor_bearing_deg = 0.
        record.anchor_position = (0., 0.)
        record.radius_history_m = [100.]
        record.detection_score_version = record.region_version
        record.detection_scores = []
        return planner, record

    def test_current_and_route_points_are_safe_certified_and_deduplicated(self):
        p, r = self.planner()
        rows = score_dynamic_detection_points(
            r.outer_vertices, (200., 0.), [(300., 0.)], 0., (600., 0.),
            p._q2_config(), p.config, time.monotonic() + 10.,
            existing_positions=[(400., 0.)],
        )
        points = [(row["position"]["x"], row["position"]["y"]) for row in rows]
        self.assertEqual(points, [(200., 0.), (500., 0.)])
        for point, row in zip(points, rows):
            self.assertTrue(is_safe_candidate(point, r.outer_vertices, 999.))
            self.assertGreater(row["completion_upper_s"], row["movement_time_s"])
            self.assertIn("continuous_direction_envelopes", row["completion_bound_scope"])
            self.assertTrue(row["possible_exit_points"])

    def test_unsafe_points_and_expired_deadline_never_scored(self):
        p, r = self.planner()
        with patch("B.q3.localize.completion_upper_bound") as bound:
            rows = score_dynamic_detection_points(
                r.outer_vertices, (2000., 0.), [], 0., (4000., 0.),
                p._q2_config(), p.config, time.monotonic() + 1.)
            self.assertEqual(rows, [])
            rows = score_dynamic_detection_points(
                r.outer_vertices, (0., 0.), [], 0., None,
                p._q2_config(), p.config, time.monotonic() - 1.)
            self.assertEqual(rows, [])
            bound.assert_not_called()

    def test_cache_refreshes_for_motion_and_route_but_not_repeated_calls(self):
        p, r = self.planner()
        with patch("B.q3.planner.score_dynamic_detection_points", return_value=[]) as score, \
                patch.object(p, "_global_continuation", return_value=(400., 0.)) as target, \
                patch("B.q3.planner.choose_detection_from_region") as static:
            p._global_detection_tasks(r, None, None)
            p._global_detection_tasks(r, None, None)
            self.assertEqual(score.call_count, 1)
            p.current_position = (24., 0.)
            p._global_detection_tasks(r, None, None)
            self.assertEqual(score.call_count, 1)
            p.current_position = (30., 0.)
            p._global_detection_tasks(r, None, None)
            self.assertEqual(score.call_count, 2)
            target.return_value = (450., 0.)
            p._global_detection_tasks(r, None, None)
            self.assertEqual(score.call_count, 3)
            static.assert_not_called()

    def test_v3_and_exhausted_budget_skip_dynamic_work(self):
        with patch("B.q3.planner.score_dynamic_detection_points") as score:
            p, r = self.planner("v3_global")
            p._global_detection_tasks(r, None, None)
            p, r = self.planner()
            p.planning_time_s = p.config.total_planning_time_limit_s
            p._global_detection_tasks(r, None, None)
            score.assert_not_called()

    def test_new_row_enters_task_pool_and_cached_cost_tracks_robot(self):
        p, r = self.planner()
        row = {
            "position": {"x": 200., "y": 0.},
            "movement_time_s": 40., "completion_upper_s": 70.,
            "worst_updated_cover_radius_m": 20.,
            "worst_cover_point_count": 2,
            "candidate_origin": "dynamic_route",
            "possible_exit_points": ((200., 0.),),
        }
        with patch("B.q3.planner.score_dynamic_detection_points", return_value=[row]) as score:
            tasks = p._global_detection_tasks(r, None, None)
            self.assertEqual(len(tasks), 1)
            self.assertEqual(tasks[0].position, (200., 0.))
            self.assertEqual(tasks[0].kind, "LOCALIZE")
            p.current_position = (10., 0.)
            updated = p._global_detection_tasks(r, None, None)
            self.assertAlmostEqual(tasks[0].completion_upper_s - updated[0].completion_upper_s, 2.)
            self.assertEqual(score.call_count, 1)
            self.assertEqual(len(r.dynamic_detection_scores), 1)
            p.station_index = len(p.stations)
            with patch.object(p, "_select_global_task", return_value=updated[0]):
                action = p._propose_global(None)
                self.assertEqual(action.decision_budget["candidate_origin"], "dynamic_route")

    def test_geometry_version_invalidates_dynamic_cache(self):
        p, r = self.planner()
        r.dynamic_detection_scores = [{"obsolete": True}]
        r.dynamic_score_origin = (0., 0.)
        r.region_version += 1
        p.planning_time_s = p.config.total_planning_time_limit_s
        self.assertEqual(p._global_detection_tasks(r, None, None), [])
        self.assertEqual(r.dynamic_detection_scores, [])
        self.assertIsNone(r.dynamic_score_origin)


if __name__ == "__main__":
    unittest.main()
