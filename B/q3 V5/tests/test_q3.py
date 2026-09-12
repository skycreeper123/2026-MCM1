import math
import random
import unittest

import numpy as np

from B.q3.geometry import (
    minimum_station_distance,
    q3_coverage_certificate,
    route_length,
    seven_station_route,
    strip_clear_points,
    strip_coverage_bound_m,
)
from B.q3.planner import (
    ChannelStatus,
    Q3Config,
    Q3Planner,
    SessionStatus,
    q3_baseline_upper_bounds,
    q3_v2_upper_bounds,
)
from B.q3.localize import build_cover_plan, initialize_outer_region, update_outer_region


class OfflineSource:
    def __init__(self, channel, position, radius=1000.0):
        self.channel = channel
        self.position = position
        self.radius = radius
        self.cleared = False


class OfflineWorld:
    def __init__(self, sources):
        self.sources = {source.channel: source for source in sources}

    @staticmethod
    def _distance(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def measure(self, action):
        source = self.sources.get(action.channel)
        if source is None or source.cleared or self._distance(action.position, source.position) > source.radius:
            return "no_signal", None
        delta = self._distance(action.position, source.position)
        if delta <= 5.0:
            return "near", None
        true_bearing = math.degrees(math.atan2(
            source.position[1] - action.position[1],
            source.position[0] - action.position[0],
        ))
        fixed_error = 1.0 if action.channel % 2 else -1.0
        return "direction", (true_bearing + fixed_error) % 360.0

    def clear(self, action):
        source = self.sources.get(action.channel)
        if source is not None and not source.cleared \
                and self._distance(action.position, source.position) <= 20.0:
            source.cleared = True
            return "success"
        return "no_target_in_range"


def run_offline(planner, world, action_limit=4000):
    for _ in range(action_limit):
        action = planner.propose_action()
        if action.kind == "MEASURE":
            result, bearing = world.measure(action)
            planner.apply_measure_result(action, result, bearing)
        elif action.kind == "CLEAR":
            planner.apply_clear_result(action, world.clear(action))
        elif action.kind == "EXIT":
            planner.apply_exit_result(action)
            return
        else:
            raise AssertionError(action)
    raise AssertionError("planner exceeded action limit")


class Question3GeometryTests(unittest.TestCase):
    def test_seven_station_certificate_and_route(self):
        stations = seven_station_route()
        certificate = q3_coverage_certificate()
        self.assertEqual(len(stations), 7)
        self.assertTrue(certificate["covered"])
        self.assertLess(certificate["certified_worst_distance_m"], 1000)
        self.assertAlmostEqual(route_length(stations), 6900.0, places=8)

    def test_dense_disk_points_are_covered(self):
        stations = seven_station_route()
        worst = 0.0
        for radius in range(0, 1801, 25):
            for degree in range(360):
                angle = math.radians(degree)
                point = (radius * math.cos(angle), radius * math.sin(angle))
                worst = max(worst, minimum_station_distance(point, stations))
        self.assertLess(worst, 1000.0)

    def test_strip_has_152_points_and_under_20m_bound(self):
        points = strip_clear_points((100.0, -200.0), 37.0)
        self.assertEqual(len(points), 152)
        self.assertLess(strip_coverage_bound_m(), 20.0)
        self.assertAlmostEqual(math.dist(points[75], points[76]), 30.0, places=8)

    def test_strip_covers_adversarial_wedge_samples(self):
        anchor = (0.0, 0.0)
        measured_bearing = 0.0
        points = strip_clear_points(anchor, measured_bearing)
        lateral_limit = 1500 * math.sin(math.radians(1.005))
        for along in range(0, 1501, 5):
            for lateral in (-lateral_limit, -15.0, 0.0, 15.0, lateral_limit):
                source = (along, lateral)
                self.assertLess(min(math.dist(source, point) for point in points), 20.0)


class Question3PlannerTests(unittest.TestCase):
    def test_batch_scans_whole_station_before_fifo_service(self):
        planner = Q3Planner(Q3Config(strategy="b0_batch_fifo"))
        first = planner.propose_action()
        planner.apply_measure_result(first, "direction", 10.0)
        second = planner.propose_action()
        self.assertEqual(second.kind, "MEASURE")
        self.assertEqual(second.channel, 2)
        for _ in range(19):
            action = planner.propose_action()
            planner.apply_measure_result(action, "no_signal")
        service = planner.propose_action()
        self.assertEqual(service.kind, "CLEAR")
        self.assertEqual(service.channel, 1)
        self.assertEqual(service.reason, "FIRST_BEARING_STRIP")

    def test_empty_world_requires_all_140_coverage_measurements(self):
        planner = Q3Planner(Q3Config(strategy="b0_batch_fifo"))
        run_offline(planner, OfflineWorld([]))
        summary = planner.summary()
        self.assertEqual(planner.session_status, SessionStatus.EXITED)
        self.assertEqual(summary["measure_count"], 140)
        self.assertEqual(summary["switch_count"], 133)
        self.assertAlmostEqual(summary["distance_m"], 6900.0, places=7)
        self.assertEqual(summary["channel_counts"]["ABSENT"], 20)

    def test_adversarial_sources_are_all_cleared(self):
        positions = [(0.0, 0.0)]
        positions.extend((1800 * math.cos(k * math.pi / 3),
                          1800 * math.sin(k * math.pi / 3)) for k in range(6))
        positions.extend((1200 * math.cos(angle), 1200 * math.sin(angle))
                         for angle in map(math.radians, (30, 150, 270)))
        sources = [OfflineSource(index + 1, point) for index, point in enumerate(positions)]
        planner = Q3Planner(Q3Config(strategy="b0_batch_fifo"))
        world = OfflineWorld(sources)
        run_offline(planner, world)
        summary = planner.summary()
        self.assertTrue(all(source.cleared for source in sources))
        self.assertEqual(summary["cleared_count"], 10)
        self.assertEqual(summary["channel_counts"]["ABSENT"], 10)
        self.assertEqual(summary["completion_reason"], "ALL_CHANNELS_RESOLVED")
        self.assertLessEqual(summary["virtual_time_s"], 19340.0)

    def test_sixteen_successes_allow_early_stop(self):
        positions = [
            (800 * math.cos(2 * math.pi * k / 16), 800 * math.sin(2 * math.pi * k / 16))
            for k in range(16)
        ]
        sources = [OfflineSource(index + 1, point, 1500.0)
                   for index, point in enumerate(positions)]
        planner = Q3Planner(Q3Config(strategy="b0_batch_fifo"))
        run_offline(planner, OfflineWorld(sources))
        summary = planner.summary()
        self.assertEqual(summary["cleared_count"], 16)
        self.assertEqual(summary["channel_counts"]["ABSENT"], 4)
        self.assertLess(summary["measure_count"], 140)

    def test_seventeenth_detected_channel_is_consistency_error(self):
        planner = Q3Planner(Q3Config(strategy="b0_batch_fifo"))
        for channel in range(1, 18):
            action = planner.propose_action()
            self.assertEqual(action.channel, channel)
            planner.apply_measure_result(action, "direction", 0.0)
        self.assertEqual(planner.session_status, SessionStatus.MODEL_OR_PROTOCOL_INCONSISTENCY)
        self.assertFalse(planner.has_completion_certificate())
        self.assertEqual(planner.propose_action().kind, "EXIT")

    def test_seeded_random_cases_all_clear(self):
        generator = random.Random(20260910)
        for case_index in range(12):
            source_count = generator.randint(10, 16)
            selected_channels = generator.sample(range(1, 21), source_count)
            sources = []
            for channel in selected_channels:
                radius = 1800 * math.sqrt(generator.random())
                angle = 2 * math.pi * generator.random()
                position = (radius * math.cos(angle), radius * math.sin(angle))
                receive_radius = generator.uniform(1000.0, 1500.0)
                sources.append(OfflineSource(channel, position, receive_radius))
            planner = Q3Planner(Q3Config(strategy="b0_batch_fifo"))
            run_offline(planner, OfflineWorld(sources))
            self.assertTrue(all(source.cleared for source in sources), case_index)
            self.assertEqual(planner.summary()["cleared_count"], source_count, case_index)

    def test_pending_action_is_stable_and_cannot_be_applied_twice(self):
        planner = Q3Planner()
        action = planner.propose_action()
        self.assertIs(action, planner.propose_action())
        planner.apply_measure_result(action, "no_signal")
        with self.assertRaises(ValueError):
            planner.apply_measure_result(action, "no_signal")

    def test_invalid_response_does_not_advance_state(self):
        planner = Q3Planner()
        action = planner.propose_action()
        with self.assertRaises(ValueError):
            planner.apply_measure_result(action, "direction")
        self.assertEqual(planner.measure_count, 0)
        self.assertEqual(planner.current_position, (0.0, 0.0))
        self.assertIs(planner.propose_action(), action)

    def test_exhausted_strip_is_reported_as_inconsistency(self):
        planner = Q3Planner(Q3Config(strategy="b0_serial"))
        measure = planner.propose_action()
        planner.apply_measure_result(measure, "direction", 0.0)
        for _ in range(152):
            clear = planner.propose_action()
            planner.apply_clear_result(clear, "no_target_in_range")
        self.assertEqual(planner.session_status, SessionStatus.MODEL_OR_PROTOCOL_INCONSISTENCY)
        self.assertEqual(planner.propose_action().kind, "EXIT")

    def test_documented_upper_bounds(self):
        bounds = q3_baseline_upper_bounds()
        self.assertEqual(bounds["scan_measurement_count"], 140)
        self.assertEqual(bounds["strip_point_count"], 152)
        self.assertEqual(bounds["logical_request_count_including_enter_exit"], 2574)
        self.assertAlmostEqual(bounds["clear_time_bound_s_per_source"], 1070.0)
        self.assertAlmostEqual(bounds["virtual_time_bound_s"], 19340.0)
        optimized = q3_v2_upper_bounds()
        self.assertEqual(optimized["logical_request_count_including_enter_exit"], 2750)
        self.assertAlmostEqual(optimized["virtual_time_bound_s"], 33068.0)

    def test_localize_measurement_keeps_scan_cursor_and_first_anchor(self):
        planner = Q3Planner(Q3Config(
            strategy="v2_local", q2_candidate_limit=8, q2_response_intervals=16,
            q2_calculation_time_limit_s=0.5,
        ))
        source = (800.0, 200.0)
        first = planner.propose_action()
        first_bearing = math.degrees(math.atan2(source[1], source[0])) + 1.0
        planner.apply_measure_result(first, "direction", first_bearing)
        for _ in range(19):
            action = planner.propose_action()
            planner.apply_measure_result(action, "no_signal")
        localize = planner.propose_action()
        self.assertEqual(localize.kind, "MEASURE")
        self.assertEqual(localize.purpose, "LOCALIZE")
        self.assertIsNone(localize.station_id)
        record = planner.channels[1]
        anchor = record.anchor_position
        old_radius = record.radius_history_m[-1]
        cursor = planner.scan_cursor
        true_bearing = math.degrees(math.atan2(
            source[1] - localize.position[1], source[0] - localize.position[0]
        )) - 1.0
        planner.apply_measure_result(localize, "direction", true_bearing)
        self.assertEqual(planner.scan_cursor, cursor)
        self.assertEqual(record.anchor_position, anchor)
        self.assertEqual(record.region_version, 2)
        self.assertEqual(len(record.observations), 2)
        self.assertLessEqual(record.radius_history_m[-1], old_radius + 1e-7)

    def test_safe_localize_no_signal_enters_finite_fallback_without_scan_evidence(self):
        planner = Q3Planner(Q3Config(
            strategy="v2_local", q2_candidate_limit=8, q2_response_intervals=16,
            q2_calculation_time_limit_s=0.5,
        ))
        first = planner.propose_action()
        planner.apply_measure_result(first, "direction", 0.0)
        for _ in range(19):
            action = planner.propose_action()
            planner.apply_measure_result(action, "no_signal")
        localize = planner.propose_action()
        self.assertEqual(localize.purpose, "LOCALIZE")
        evidence_before = set(planner.channels[1].no_signal_station_ids)
        planner.apply_measure_result(localize, "no_signal")
        fallback = planner.propose_action()
        self.assertEqual(fallback.kind, "CLEAR")
        self.assertEqual(fallback.reason, "FIRST_BEARING_STRIP")
        self.assertEqual(planner.channels[1].no_signal_station_ids, evidence_before)

    def test_v2_local_offline_loop_clears_without_fallback(self):
        source = OfflineSource(7, (800.0, 200.0), 1000.0)
        planner = Q3Planner(Q3Config(
            strategy="v2_local", q2_candidate_limit=8, q2_response_intervals=16,
            q2_calculation_time_limit_s=0.5,
        ))
        run_offline(planner, OfflineWorld([source]))
        record = planner.channels[7]
        self.assertTrue(source.cleared)
        self.assertTrue(planner.summary()["completion_certificate"])
        self.assertGreaterEqual(record.region_version, 2)
        self.assertGreater(planner.localize_measure_count, 0)
        self.assertFalse(record.fallback_used)


class Question3LocalizationTests(unittest.TestCase):
    @staticmethod
    def _inside_convex(vertices, point, tolerance=1e-7):
        values = []
        for first, second in zip(vertices, np.roll(vertices, -1, axis=0)):
            edge = second - first
            relative = point - first
            values.append(edge[0] * relative[1] - edge[1] * relative[0])
        return all(value >= -tolerance for value in values) \
            or all(value <= tolerance for value in values)

    def test_cumulative_outer_region_retains_truth_and_shrinks(self):
        truth = np.array([700.0, 100.0])
        first = {"position": {"x": 0.0, "y": 0.0},
                 "svd_deg": math.degrees(math.atan2(100.0, 700.0)) + 1.0}
        region = initialize_outer_region(first, 1.005)
        second_position = (-50.0, 500.0)
        second = {"position": {"x": second_position[0], "y": second_position[1]},
                  "svd_deg": math.degrees(math.atan2(
                      truth[1] - second_position[1], truth[0] - second_position[0]
                  )) - 1.0}
        updated = update_outer_region(region["vertices"], second, 1.005)
        self.assertEqual(updated["status"], "BOUNDED")
        self.assertTrue(np.all(np.isfinite(updated["vertices"])))
        self.assertTrue(self._inside_convex(np.asarray(updated["vertices"]), truth))
        self.assertLessEqual(updated["minimum_enclosing_circle"]["radius_m"],
                             region["minimum_enclosing_circle"]["radius_m"] + 1e-7)

    def test_cross_zero_bearing_keeps_true_source(self):
        truth = np.array([800.0, -0.1])
        true_bearing = math.degrees(math.atan2(truth[1], truth[0])) % 360.0
        first = {"position": {"x": 0.0, "y": 0.0},
                 "svd_deg": (true_bearing + 1.0) % 360.0}
        region = initialize_outer_region(first, 1.005)
        self.assertEqual(region["status"], "BOUNDED")
        self.assertTrue(self._inside_convex(np.asarray(region["vertices"]), truth))

    def test_28m_rectangle_has_one_certified_clear_point(self):
        vertices = np.array([[0.0, 0.0], [28.0, 0.0], [28.0, 28.0], [0.0, 28.0]])
        plan = build_cover_plan(vertices, (-10.0, 14.0), max_points=1)
        self.assertIsNotNone(plan)
        self.assertEqual(len(plan.points), 1)
        self.assertLessEqual(plan.cover_radius_m, 19.9)

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            Q3Planner(Q3Config(ring_radius_m=3000.0))
        with self.assertRaises(ValueError):
            Q3Planner(Q3Config(source_count_upper_bound=21))
        with self.assertRaises(ValueError):
            Q3Planner(Q3Config(strip_max_range_m=1400.0))
        with self.assertRaises(ValueError):
            Q3Planner(Q3Config(angle_half_width_deg=2.0))


if __name__ == "__main__":
    unittest.main()
