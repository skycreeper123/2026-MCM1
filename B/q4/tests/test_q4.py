import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

from B.q3.localize import verify_cover_certificate
from B.q4.geometry import (SourceRecord, build_clear_plan, build_pair_probe,
                           classify_q4_reception, exact_hull, hull_candidates,
                           hull_contains, load_and_verify_station_cover,
                           min_polygon_distance_squared, update_positive_hull,
                           update_source_region)
from B.q4.planner import Q4Config, Q4Planner
from B.q4.policy import MeasureTask, score_measure_policy
from B.q4.routing import RouteTask, optimize_service_route, task_route_cost
from B.q4.validation import Source, mixed_world, run_world, truth_in_region


def record_at(point=(0.0, 0.0), bearing=0.0, channel=1):
    r = SourceRecord(channel, state="DETECTED")
    obs = {"position": point, "result": "direction", "svd_deg": bearing}
    update_positive_hull(r, obs)
    update_source_region(r, obs)
    r.used_positions.add(point)
    return r


class GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cover = load_and_verify_station_cover(allow_fallback=False)

    def test_primary_continuous_and_exact_cover(self):
        self.assertEqual(len(self.cover.stations), 21)
        self.assertEqual(self.cover.certificate["exact"]["leaf_triangles"], 268)
        self.assertEqual(sorted(self.cover.route), list(range(21)))
        length = sum(math.dist(self.cover.stations[a], self.cover.stations[b]) for a, b in zip(self.cover.route, self.cover.route[1:]))
        self.assertAlmostEqual(length, 17808.43161440987, places=6)

    def test_boundary_outward_tangent_origin_minimum_radius(self):
        for radius in (0, 1800):
            for degree in range(0, 360, 5):
                a = math.radians(degree)
                for direction in (degree, degree+90, degree+180):
                    source = Source(1, (radius*math.cos(a), radius*math.sin(a)), 1000, direction)
                    self.assertTrue(any(source.receives(p) for p in self.cover.stations))

    def test_certificate_failure_switches_to_verified_31_network(self):
        fallback = load_and_verify_station_cover(Path(__file__).with_name("missing.json"))
        self.assertEqual(len(fallback.stations), 31)
        self.assertNotEqual(fallback.network_id, self.cover.network_id)
        self.assertTrue(fallback.certificate["passed"])
        for degree in range(360):
            a = math.radians(degree)
            source = Source(1, (1800*math.cos(a), 1800*math.sin(a)), 1000, degree)
            self.assertTrue(any(source.receives(p) for p in fallback.stations))
        with self.assertRaises(ValueError):
            load_and_verify_station_cover("missing.json", allow_fallback=False)

    def test_corrupt_certificate_not_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"invalid.json"
            path.write_text('{"stations_m": []}', encoding="utf-8")
            self.assertTrue(load_and_verify_station_cover(path).certificate["fallback"])

    def test_backside_near_is_no_signal_and_clear_is_still_possible(self):
        source = Source(1, (0, 0), 1000, 0)
        self.assertEqual(source.measure((-1, 0))[0], "no_signal")
        self.assertLess(math.dist(source.position, (-1, 0)), 20)

    def test_point_and_line_hulls_have_no_outward_tolerance(self):
        r = record_at()
        self.assertEqual(hull_candidates(r), ())
        self.assertFalse(hull_contains(r.hull, (0, 1e-30)))
        r.hull = exact_hull([(0, 0), (100, 0)])
        self.assertTrue(hull_contains(r.hull, (25, 0)))
        self.assertFalse(hull_contains(r.hull, (25, 1e-30)))
        self.assertEqual(len(hull_candidates(r)), 3)

    def test_distance_to_whole_polygon_not_vertices(self):
        polygon = [(-2000, -2000), (2000, -2000), (2000, 2000), (-2000, 2000)]
        self.assertEqual(min_polygon_distance_squared(polygon, (0, 0)), 0)
        self.assertEqual(min_polygon_distance_squared(polygon, (0, 2100)), 10000)
        r = record_at()
        r.vertices = np.array(polygon)
        self.assertEqual(classify_q4_reception(r, (0, 2100)), "UNCERTAIN")
        self.assertEqual(classify_q4_reception(r, (0, 3600)), "GUARANTEED_NO_SIGNAL")

    def test_pair_exact_midpoint_and_no_legal_pair(self):
        r = record_at()
        r.vertices = np.array([[495, -5], [505, -5], [505, 5], [495, 5]])
        pair = build_pair_probe(r, (0, 0), (0, 100))
        self.assertIsNotNone(pair)
        self.assertEqual(pair.certificate, "PAIR_AT_LEAST_ONE")
        self.assertIsNone(build_pair_probe(r, (0, 0), (0, 1000)))
        # Floating a+w and a-w are not symmetric about binary64 a=0.1.
        r2 = record_at((0.1, 0.1))
        r2.vertices = r.vertices
        self.assertIsNone(build_pair_probe(r2, (0.1, 0.1), (50, 0)))

    def test_negative_result_does_not_remove_feasible_region(self):
        r = record_at()
        before = r.vertices.copy()
        update_source_region(r, {"position": (500, 0), "result": "no_signal"})
        np.testing.assert_array_equal(before, r.vertices)
        self.assertEqual(r.region_version, 1)

    def test_cross_zero_and_almost_parallel_bearings_preserve_truth(self):
        source = Source(1, (1000, -0.2), 1500, None, phase=0)
        r = SourceRecord(1)
        for point in ((0.0, 0.0), (1.0, 0.0), (500.0, 80.0)):
            result, bearing = source.measure(point)
            update_source_region(r, {"position": point, "result": result, "svd_deg": bearing})
            self.assertTrue(truth_in_region(r.vertices, source.position))
        self.assertEqual(r.region_version, 3)

    def test_grid_above_eight_points_and_strip_fallback(self):
        r = record_at()
        plan = build_clear_plan(r, (0, 0))
        self.assertGreater(plan.point_count, 8)
        self.assertLessEqual(plan.point_count, 152)
        self.assertTrue(verify_cover_certificate(r.vertices, plan))
        with patch("B.q4.geometry.build_cover_plan", return_value=None):
            strip = build_clear_plan(record_at(), (0, 0))
        self.assertEqual(strip.kind, "STRIP")
        self.assertEqual(strip.point_count, 152)
        self.assertLess(strip.cover_radius_m, 19.9)

    def test_timeout_keeps_all_response_intervals_and_negative_branch(self):
        r = record_at()
        backup = build_clear_plan(r, (0, 0))
        task = MeasureTask(((600, 0),), "DISTANCE_ONLY")
        result = score_measure_policy(r, task, (0, 0), 2, None, backup, Q4Config(), time.monotonic()-1)
        self.assertEqual(result.intervals_fallback, 32)
        self.assertEqual(result.intervals_total, 32)
        self.assertTrue(result.includes_no_signal)
        self.assertFalse(result.eligible)
        self.assertGreater(result.upper_s, 125)


class PlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cover = load_and_verify_station_cover()

    def planner(self):
        return Q4Planner(Q4Config(planning_total_s=0), self.cover)

    def test_pending_action_and_duplicate_response_are_idempotent(self):
        p = self.planner()
        action = p.propose_action()
        self.assertIs(action, p.propose_action())
        p.apply_measure_result(action, "no_signal")
        before = dict(p.metrics)
        p.apply_measure_result(action, "no_signal")
        self.assertEqual(before, p.metrics)
        with self.assertRaises(ValueError):
            p.apply_measure_result(action, "near")

    def test_near_immediate_clear_and_clear_does_not_tune_receiver(self):
        p = self.planner()
        action = p.propose_action()
        p.apply_measure_result(action, "near")
        clear = p.propose_action()
        self.assertEqual(clear.kind, "CLEAR")
        self.assertEqual(clear.position, action.position)
        p.receiver_channel = 20
        p.apply_clear_result(clear, "success")
        self.assertEqual(p.receiver_channel, 20)

    def test_failed_near_is_inconsistency_not_infinite_retry(self):
        p = self.planner()
        p.apply_measure_result(p.propose_action(), "near")
        p.apply_clear_result(p.propose_action(), "no_target_in_range")
        self.assertEqual(p.propose_action().kind, "EXIT")
        self.assertFalse(p.has_completion_certificate())

    def test_pair_first_miss_second_positive_is_mandatory(self):
        p = self.planner()
        r = record_at()
        r.vertices = np.array([[495, -5], [505, -5], [505, 5], [495, 5]])
        pair = build_pair_probe(r, (0, 0), (0, -100))
        p.channels[1], r.pending_pair, p.pair_channel = r, pair, 1
        source = Source(1, (500, 0), 1000, 91)
        self.assertTrue(source.receives((0, 0)))
        first = p._action("MEASURE", pair.points[0], 1, purpose="LOCALIZE_PAIR_FIRST", reception="PAIR_AT_LEAST_ONE")
        result, bearing = source.measure(first.position)
        self.assertEqual(result, "no_signal")
        cursor = p.station_cursor
        p.apply_measure_result(first, result, bearing)
        second = p.propose_action()
        self.assertEqual(second.position, pair.points[1])
        self.assertEqual(second.purpose, "LOCALIZE_PAIR_SECOND")
        result, bearing = source.measure(second.position)
        self.assertNotEqual(result, "no_signal")
        p.apply_measure_result(second, result, bearing)
        self.assertIsNone(p.pair_channel)
        self.assertEqual(p.station_cursor, cursor)
        self.assertEqual(r.local_measurements, 2)

    def test_pair_reserves_two_measurements_and_complete_closed_distance(self):
        p = self.planner()
        r = record_at()
        r.local_measurements = 3
        self.assertFalse(p._local_allowed(r, ((100, 0), (-100, 0))))
        r.local_measurements = 0
        p.position = (3000, 0)
        self.assertTrue(p._local_allowed(r, ((100, 0), (-100, 0))))
        self.assertFalse(p._local_allowed(r, ((100, 0), (-100, 0)), (3000, 0)))

    def test_four_local_services_cannot_starve_next_discovery_station(self):
        p = self.planner()
        p.channels[1] = record_at()
        for ch in range(2, 21):
            p.discovery_ledger[ch].add(0)
        route = (RouteTask(("CLEAR", 1), ((500, 0),), "CLEAR", channel=1),
                 RouteTask(("STATION", 6), (self.cover.stations[6],), "STATION", channels=(2,)))
        def fake_service(record, continuation, deadline):
            return p._action("MEASURE", (100+p.services_here, 0), 1, purpose="LOCALIZE_SINGLE")
        with patch.object(p, "_remaining_route", return_value=route), patch.object(p, "_service", side_effect=fake_service):
            for _ in range(4):
                action = p.propose_action()
                self.assertEqual(action.purpose, "LOCALIZE_SINGLE")
                p.apply_measure_result(action, "no_signal")
                self.assertEqual(p.station_cursor, 0)
            action = p.propose_action()
        self.assertEqual(action.purpose, "DISCOVERY_SCAN")
        self.assertEqual(action.station_id, self.cover.route[1])

    def test_station_revisit_negative_does_not_advance_discovery(self):
        p = self.planner()
        r = p.channels[1] = record_at()
        before = r.vertices.copy()
        action = p._action("MEASURE", self.cover.stations[6], 1, purpose="STATION_REVISIT", station_id=6)
        p.apply_measure_result(action, "no_signal")
        self.assertEqual(p.station_cursor, 0)
        self.assertEqual(r.state, "DETECTED")
        self.assertIn((6, 1), p.station_revisits)
        self.assertEqual(p.discovery_ledger[1], set())
        np.testing.assert_array_equal(before, r.vertices)

    def test_guaranteed_hull_reception_failure_stops_as_inconsistency(self):
        p = self.planner()
        p.channels[1] = record_at()
        action = p._action("MEASURE", (0, 0), 1, purpose="LOCALIZE_SINGLE", reception="POSITIVE_HULL")
        p.apply_measure_result(action, "no_signal")
        self.assertEqual(p.propose_action().kind, "EXIT")
        self.assertFalse(p.has_completion_certificate())

    def test_offline_fallback_station_network_full_clear(self):
        fallback = load_and_verify_station_cover("missing.json")
        result = run_world(mixed_world(101, 10), Q4Config(planning_total_s=0), fallback)
        self.assertEqual(result["station_count"], 31)
        self.assertEqual(result["cleared_count"], 10)

    def test_ten_cleared_do_not_end_unknown_search(self):
        p = self.planner()
        for ch in range(1, 11):
            p.channels[ch].state = "CLEARED"
        self.assertFalse(p.has_completion_certificate())
        self.assertEqual(p.propose_action().purpose, "DISCOVERY_SCAN")

    def test_sixteen_detected_are_not_sixteen_cleared(self):
        p = self.planner()
        for ch in range(1, 17):
            p.channels[ch].state = "DETECTED"
        self.assertFalse(p.has_completion_certificate())
        self.assertEqual(p._scan_channels(0), [])
        for ch in range(1, 17):
            p.channels[ch].state = "CLEARED"
        self.assertTrue(p.has_completion_certificate())

    def test_absent_requires_entire_active_network(self):
        p = self.planner()
        for ch in p.channels:
            p.channels[ch].state = "ABSENT"
            p.discovery_ledger[ch] = set(range(20))
        self.assertFalse(p.has_completion_certificate())
        for ch in p.channels:
            p.discovery_ledger[ch].add(20)
            p.channels[ch].absence_certificate = {"method": "FULL_NETWORK",
                                                   "network_id": p.cover.network_id}
        self.assertTrue(p.has_completion_certificate())

    def test_route_preserves_duties_and_clear_channel_semantics(self):
        tasks = [RouteTask(("a",), ((0, 0),), "MEASURE", channel=1),
                 RouteTask(("b",), ((0, 0),), "CLEAR", channel=2),
                 RouteTask(("c",), ((0, 0),), "MEASURE", channel=1)]
        self.assertEqual(task_route_cost(tasks, (0, 0), 1), 15)
        stations = [RouteTask(("STATION", i), ((100*i, 0),), "STATION", channels=(1, 2)) for i in (1, 2, 3)]
        route = optimize_service_route(stations, [tasks[1]], (0, 0), 1)
        self.assertEqual([t.key for t in route if t.kind == "STATION"], [t.key for t in stations])
        self.assertEqual(set(t.key for t in route), set(t.key for t in stations+[tasks[1]]))

    def test_offline_ten_mixed_sources_full_clear_and_discovery_interleaving(self):
        result = run_world(mixed_world(11, 10), Q4Config(planning_total_s=0), self.cover)
        self.assertEqual(result["clearance_rate"], 1)
        self.assertLess(result["first_clear_station_cursor"], 21)
        self.assertEqual(sum(s == "ABSENT" for s in result["channel_states"].values()), 10)

    def test_offline_sixteen_mixed_sources_full_clear(self):
        result = run_world(mixed_world(29, 16), Q4Config(planning_total_s=0), self.cover)
        self.assertEqual(result["cleared_count"], 16)
        self.assertTrue(result["completion"]["complete"])


if __name__ == "__main__":
    unittest.main()
