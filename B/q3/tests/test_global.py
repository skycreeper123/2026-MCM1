import math
import time
import unittest
from dataclasses import replace
from unittest.mock import patch

import numpy as np

from B.q3 import (Q3Config, Q3Planner, build_cover_plan, completion_upper_bound,
                  verify_cover_certificate)
from B.q3.localize import spatial_candidate_subset, initialize_outer_region
from B.q3.planner import ChannelStatus, SessionStatus, Task
from B.q3.validation import inside_region, run_case


def finish_discovery(planner, initial_results):
    while True:
        action = planner.propose_action()
        if action.kind != "MEASURE" or action.purpose != "COVERAGE_SCAN":
            return action
        result, bearing = initial_results.get((action.station_id, action.channel), ("no_signal", None))
        planner.apply_measure_result(action, result, bearing)


class GlobalPlannerTests(unittest.TestCase):
    def test_strategy_specific_defaults_and_explicit_overrides(self):
        self.assertEqual(Q3Config().strategy, "v4_cooperative")
        self.assertEqual(Q3Config().max_optimized_clear_attempts_per_source, 16)
        legacy = Q3Config(strategy="v2_local")
        self.assertEqual((legacy.max_optimized_clear_attempts_per_source,
                          legacy.q2_candidate_limit, legacy.q2_calculation_time_limit_s,
                          legacy.minimum_radius_improvement_fraction), (8, 12, 1.0, 0.05))
        self.assertEqual(Q3Config(strategy="v2_local", q2_candidate_limit=24).q2_candidate_limit, 24)

    def test_discover_sixteen_before_clear_and_use_count_bound(self):
        p = Q3Planner(Q3Config(strategy="v3_global"))
        for channel in range(1, 17):
            a = p.propose_action()
            self.assertEqual((a.kind, a.channel, a.station_id), ("MEASURE", channel, 0))
            p.apply_measure_result(a, "direction", channel*20.0)
        p._mark_upper_bound_absences()
        self.assertEqual(len(p.pending_sources), 16)
        self.assertEqual(p.successful_clear_count, 0)
        self.assertFalse(p.has_completion_certificate())
        self.assertTrue(all(p.channels[c].status == ChannelStatus.DETECTED for c in range(1, 17)))
        self.assertTrue(all(p.channels[c].absence_reason == "COUNT_UPPER_BOUND" for c in range(17, 21)))

    def test_near_is_deferred_and_last_station_sources_are_serviced(self):
        p = Q3Planner(Q3Config(strategy="v3_global"))
        a = finish_discovery(p, {(0, 1): ("near", None), (6, 20): ("near", None)})
        self.assertEqual(a.kind, "CLEAR")
        self.assertEqual(a.channel, 20)  # closest task, not first-discovered FIFO
        self.assertEqual(p.station_index, 7)
        self.assertEqual(p.successful_clear_count, 0)
        p.apply_clear_result(a, "success")
        self.assertEqual(p.propose_action().channel, 1)
        p.apply_clear_result(p.propose_action(), "success")
        self.assertTrue(p.has_completion_certificate())
        self.assertFalse(p.pending_sources)

    def test_large_valid_region_is_not_sent_to_strip_when_budget_expires(self):
        p = Q3Planner(Q3Config(strategy="v3_global",
                              total_planning_time_limit_s=1e-9,
                              max_optimized_clear_attempts_per_source=1))
        a = finish_discovery(p, {(0, 1): ("direction", 0.0)})
        self.assertEqual(a.kind, "CLEAR")
        self.assertEqual(a.reason, "RECTANGLE_GRID")
        self.assertGreater(p.channels[1].service_plan.cover.point_count, 1)
        self.assertFalse(p.channels[1].fallback_used)

    def test_initial_geometry_exception_preserves_anchor_for_fallback(self):
        p = Q3Planner(Q3Config(strategy="v3_global"))
        with patch("B.q3.planner.initialize_outer_region", side_effect=ValueError("synthetic failure")):
            a = p.propose_action()
            p.apply_measure_result(a, "direction", 0.0)
        a = finish_discovery(p, {})
        self.assertEqual(a.reason, "FIRST_BEARING_STRIP")
        self.assertEqual(len(p.active_plan_points), 152)
        self.assertEqual(p.channels[1].anchor_position, (0.0, 0.0))

    def test_localize_no_signal_fallback_is_finite_and_not_absence_evidence(self):
        p = Q3Planner(Q3Config(strategy="v3_global"))
        a = finish_discovery(p, {(0, 1): ("direction", 0.0)})
        self.assertEqual(a.purpose, "LOCALIZE")
        evidence = set(p.channels[1].no_signal_station_ids)
        p.apply_measure_result(a, "no_signal")
        self.assertEqual(p.propose_action().reason, "FIRST_BEARING_STRIP")
        for _ in range(152):
            p.apply_clear_result(p.propose_action(), "no_target_in_range")
        self.assertEqual(p.session_status, SessionStatus.MODEL_OR_PROTOCOL_INCONSISTENCY)
        self.assertFalse(p.has_completion_certificate())
        self.assertEqual(p.channels[1].no_signal_station_ids, evidence)
        self.assertEqual(p.propose_action().kind, "EXIT")

    def test_certified_plan_failure_enters_emergency(self):
        p = Q3Planner(Q3Config(strategy="v3_global",
                              total_planning_time_limit_s=1e-9))
        a = finish_discovery(p, {(0, 1): ("direction", 0.0)})
        for _ in range(len(p.active_plan_points)):
            p.apply_clear_result(p.propose_action(), "no_target_in_range")
        self.assertEqual(p.channels[1].geometry_failure_reason, "CERTIFIED_CLEAR_PLAN_FAILED")
        self.assertEqual(p.propose_action().reason, "FIRST_BEARING_STRIP")

    def test_two_sources_complete_with_truth_safety_and_certificate_checks(self):
        case = {"case_id": 100, "error_mode": "alternating", "sources": [
            {"channel": 7, "position": (800., 200.), "receive_radius_m": 1000.},
            {"channel": 20, "position": (1750., -20.), "receive_radius_m": 1000.}]}
        result = run_case(case, Q3Config(strategy="v3_global"))
        self.assertEqual(result["cleared_count"], 2)
        self.assertEqual(result["fallback_source_count"], 0)
        self.assertGreater(result["safety_checks"], 0)
        self.assertGreater(result["truth_checks"], 0)
        self.assertGreater(result["certificate_checks"], 0)

    def test_response_update_releases_source_for_global_reordering(self):
        p = Q3Planner(Q3Config(strategy="v3_global"))
        a = finish_discovery(p, {(0, 1): ("direction", 0.0)})
        self.assertEqual(a.purpose, "LOCALIZE")
        bearing = math.degrees(math.atan2(-a.position[1], 800-a.position[0]))
        old_version = p.channels[1].region_version
        p.apply_measure_result(a, "direction", bearing)
        self.assertIsNone(p.active_service_channel)
        self.assertIsNone(p.active_task)
        self.assertGreater(p.channels[1].region_version, old_version)
        self.assertTrue(inside_region(p.channels[1].outer_vertices, (800., 0.)))
        self.assertIs(p.propose_action(), p.propose_action())

    def test_fixed_station_revisit_no_signal_preserves_detected_region(self):
        p = Q3Planner(Q3Config(strategy="v4_cooperative", q2_response_intervals=8))
        first = p.propose_action()
        p.apply_measure_result(first, "direction", 0.0)
        record = p.channels[1]
        before = record.outer_vertices.copy()
        absence_evidence = set(record.no_signal_station_ids)
        revisit = p._opportunistic_revisit_action(
            record, 2, time.monotonic() + 1.0,
        )
        self.assertIsNotNone(revisit)
        self.assertEqual(revisit.purpose, "OPPORTUNISTIC_REVISIT")
        p.pending_action = revisit
        p.apply_measure_result(revisit, "no_signal")
        self.assertEqual(record.status, ChannelStatus.DETECTED)
        np.testing.assert_allclose(record.outer_vertices, before)
        self.assertEqual(record.no_signal_station_ids, absence_evidence)
        self.assertEqual(record.revisit_no_signal_station_ids, {2})
        self.assertEqual(record.localization_events[-1]["response"], "no_signal")

    def test_fixed_station_direction_shrinks_region_and_is_audited(self):
        p = Q3Planner(Q3Config(strategy="v4_cooperative", q2_response_intervals=8))
        first = p.propose_action()
        p.apply_measure_result(first, "direction", 0.0)
        record = p.channels[1]
        old_radius = record.radius_history_m[-1]
        revisit = p._opportunistic_revisit_action(
            record, 2, time.monotonic() + 1.0,
        )
        self.assertIsNotNone(revisit)
        source = (800.0, 0.0)
        bearing = math.degrees(math.atan2(
            source[1] - revisit.position[1],
            source[0] - revisit.position[0],
        ))
        p.pending_action = revisit
        p.apply_measure_result(revisit, "direction", bearing)
        self.assertTrue(inside_region(record.outer_vertices, source))
        self.assertLess(record.radius_history_m[-1], old_radius)
        self.assertEqual(record.fixed_direction_station_ids, {0, 2})
        event = record.localization_events[-1]
        self.assertEqual(event["origin"], "FIXED_STATION_REVISIT")
        self.assertGreater(
            event["before"]["certified_clear_points"],
            event["after"]["certified_clear_points"],
        )

    def test_quality_rule_does_not_stop_at_direction_count_two(self):
        p = Q3Planner(Q3Config(strategy="v4_cooperative"))
        first = p.propose_action()
        p.apply_measure_result(first, "direction", 0.0)
        record = p.channels[1]
        record.observations.append(record.observations[0].copy())
        plan = p._global_cover(record, None)
        quality = p._region_metrics(record, plan)
        self.assertEqual(len(record.observations), 2)
        self.assertFalse(quality["quality_sufficient"])
        self.assertGreater(quality["certified_clear_points"], 4)

    def test_joint_route_uses_every_region_and_improves_naive_order(self):
        p = Q3Planner(Q3Config(strategy="v4_cooperative"))
        tasks = [
            Task(1, "LOCALIZE", (1000.0, 0.0), 5.0, None,
                 exit_position=(1000.0, 0.0), intrinsic_upper_s=5.0),
            Task(2, "LOCALIZE", (10.0, 0.0), 5.0, None,
                 exit_position=(10.0, 0.0), intrinsic_upper_s=5.0),
            Task(3, "LOCALIZE", (20.0, 0.0), 5.0, None,
                 exit_position=(20.0, 0.0), intrinsic_upper_s=5.0),
        ]
        naive = p._task_route_cost(tasks)
        route, optimized = p._optimize_task_route(tasks)
        self.assertCountEqual([task.channel for task in route], [1, 2, 3])
        self.assertLess(optimized, naive)
        self.assertEqual(route[0].channel, 2)


class GlobalGeometryTests(unittest.TestCase):
    def test_completion_bound_rejects_unsafe_candidate(self):
        with self.assertRaises(ValueError):
            completion_upper_bound(np.array([[0., 0.], [100., 0.]]),
                                   (2000., 0.), None, Q3Config())

    def test_spatial_limit_keeps_both_sides_and_route_representative(self):
        points = [(0., 0.)]+[(100*math.cos(a), 100*math.sin(a))
                            for a in np.arange(0, 2*math.pi, math.pi/6)]
        selected = spatial_candidate_subset(points, 4, (0., 0.), (100., 0.), 0.)
        self.assertIn((0., 0.), selected)
        self.assertIn((100., 0.), selected)
        self.assertGreater(max(p[1] for p in selected), 99)
        self.assertLess(min(p[1] for p in selected), -99)

    def test_grid_certificate_covers_interiors_and_detects_missing_cells(self):
        angle = 0.7
        rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        vertices = np.array([[0., 0.], [201., 0.], [201., 59.], [0., 59.]]) @ rotation.T
        plan = build_cover_plan(vertices, (100., -30.), (300., 200.), max_points=100)
        self.assertTrue(verify_cover_certificate(vertices, plan))
        for x in np.linspace(0, 201, 57):
            for y in np.linspace(0, 59, 23):
                point = np.array([x, y]) @ rotation.T
                self.assertLessEqual(min(math.dist(point, p) for p in plan.points), plan.cover_radius_m+1e-7)
        broken = replace(plan, points=plan.points[:-1]+(plan.points[0],))
        self.assertFalse(verify_cover_certificate(vertices, broken))

    def test_point_segment_and_cross_zero_regions(self):
        for vertices in (np.array([[1., 2.]]), np.array([[0., 0.], [100., 0.]])):
            plan = build_cover_plan(vertices, (0., 0.), max_points=100)
            self.assertTrue(verify_cover_certificate(vertices, plan))
        region = initialize_outer_region({"position": {"x": 0., "y": 0.}, "svd_deg": 359.999})
        self.assertTrue(inside_region(region["vertices"], (800., -0.01)))

    def test_continuation_and_travel_are_in_completion_bound(self):
        vertices = np.array([[800., -10.], [820., -10.], [820., 10.], [800., 10.]])
        cfg = Q3Config()
        a = completion_upper_bound(vertices, (500., 300.), None, cfg, (0., 0.))
        b = completion_upper_bound(vertices, (500., 300.), (2000., 2000.), cfg, (0., 0.))
        self.assertGreater(b["completion_upper_s"], a["completion_upper_s"])
        c = completion_upper_bound(vertices, (500., 300.), None, cfg, (500., 300.))
        self.assertAlmostEqual(a["completion_upper_s"]-c["completion_upper_s"], math.hypot(500, 300)/5)

    def test_deadline_retains_whole_region_bound(self):
        vertices = np.array([[0., -20.], [1500., -20.], [1500., 20.], [0., 20.]])
        result = completion_upper_bound(vertices, (750., 200.), None, Q3Config(),
                                        deadline_monotonic=time.monotonic()-1)
        self.assertTrue(result["whole_region_used_on_timeout"])
        self.assertGreater(result["worst_updated_cover_radius_m"], 750)
        self.assertGreater(result["worst_cover_point_count"], 16)


if __name__ == "__main__":
    unittest.main()
