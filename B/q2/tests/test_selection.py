import math
import unittest
from dataclasses import replace
from unittest.mock import patch

import numpy as np

from B.q1.geometry import bearing_halfplanes
from B.q2.selection import (
    Q2Config,
    build_initial_outer_region,
    choose_second_detection,
    circle_outer_halfplanes,
    is_safe_candidate,
    safe_candidate_region,
    response_radius_bound,
)


FAST_CONFIG = Q2Config(
    circle_sides=48,
    coarse_spacing_m=200,
    fine_spacing_m=50,
    max_coarse_candidates=12,
    max_fine_candidates=12,
    max_response_intervals=24,
)


class Question2Tests(unittest.TestCase):
    def setUp(self):
        self.first = {"position": {"x": -1000.0, "y": 0.0}, "svd_deg": 0.5}
        self.true_source = np.array([0.0, 0.0])

    def test_outer_circle_contains_exact_circle(self):
        center = np.array([123.0, -456.0])
        A, b = circle_outer_halfplanes(center, 1000, 64)
        for angle in np.linspace(0, 2 * math.pi, 1000, endpoint=False):
            point = center + 1000 * np.array([math.cos(angle), math.sin(angle)])
            self.assertTrue(np.all(A @ point <= b + 1e-9))

    def test_initial_outer_region_contains_true_source(self):
        result, A, b = build_initial_outer_region(self.first, config=FAST_CONFIG)
        self.assertEqual(result["status"], "BOUNDED", result)
        self.assertTrue(np.all(A @ self.true_source <= b + 1e-8))
        self.assertGreaterEqual(len(result["vertices"]), 3)

    def test_safe_region_equivalent_to_vertex_minimum_circle(self):
        vertices = np.array([[-600, -20], [600, -20], [600, 20], [-600, 20]])
        region = safe_candidate_region(vertices, 999.9)
        self.assertTrue(region["nonempty"])
        witness = np.asarray(region["witness_center"])
        self.assertTrue(is_safe_candidate(witness, vertices, 999.9))
        for alpha in np.linspace(0, 1, 21):
            point = alpha * vertices[0] + (1 - alpha) * vertices[2]
            self.assertLessEqual(np.linalg.norm(point - witness), 999.9 + 1e-9)

    def test_safe_region_can_be_empty(self):
        region = safe_candidate_region([[-1001, 0], [1001, 0]], 1000)
        self.assertFalse(region["nonempty"])
        self.assertIsNone(region["witness_center"])

    def test_selected_point_has_continuous_reception_guarantee(self):
        result = choose_second_detection(self.first, config=FAST_CONFIG)
        self.assertEqual(result["status"], "OK", result)
        selected = np.array([result["selected"]["position"]["x"],
                             result["selected"]["position"]["y"]])
        vertices = np.asarray(result["initial_region"]["vertices"])
        radius = result["safe_candidate_region"]["guaranteed_radius_m"]
        self.assertTrue(is_safe_candidate(selected, vertices, radius))
        self.assertLessEqual(np.max(np.linalg.norm(vertices - selected, axis=1)), radius + 1e-8)
        self.assertGreater(np.linalg.norm(selected - np.array([-1000.0, 0.0])), 1e-6)
        self.assertGreater(result['selected']['response_interval_count'], 1)
        self.assertEqual(result['selected']['radius_bound_scope'],
                         'all_direction_responses_over_initial_outer_polygon')

    def test_selection_is_deterministic_and_improves_sampled_radius(self):
        first = choose_second_detection(self.first, config=FAST_CONFIG)
        second = choose_second_detection(self.first, config=FAST_CONFIG)
        self.assertEqual(first["selected"]["position"], second["selected"]["position"])
        self.assertAlmostEqual(first["selected"]["objective_m"], second["selected"]["objective_m"])
        initial_radius = first["initial_region"]["minimum_enclosing_circle"]["radius_m"]
        self.assertLess(first["selected"]["worst_updated_cover_radius_m"], initial_radius)

    def test_selected_point_is_off_original_centerline_for_crossing_geometry(self):
        result = choose_second_detection(self.first, config=FAST_CONFIG)
        self.assertEqual(result["status"], "OK")
        self.assertGreater(abs(result["selected"]["position"]["y"]), 10)

    def test_invalid_input(self):
        with self.assertRaises(ValueError):
            choose_second_detection({"position": {"x": 0, "y": 0}}, config=FAST_CONFIG)
        with self.assertRaises(ValueError):
            choose_second_detection(self.first, error_deg=0, config=FAST_CONFIG)
        with self.assertRaises(ValueError):
            safe_candidate_region([], 1000)
        with self.assertRaises(ValueError):
            choose_second_detection(
                self.first, config=Q2Config(movement_weight_m_per_s=-1)
            )
        with self.assertRaises(ValueError):
            choose_second_detection(
                self.first, config=Q2Config(maximum_reception_radius_m=900)
            )
        with self.assertRaises(ValueError):
            choose_second_detection(self.first, used_positions=np.array([[0.0, 0.0]]),
                                    config=FAST_CONFIG)

    def test_source_truth_satisfies_first_wedge(self):
        A, b = bearing_halfplanes([self.first])
        self.assertTrue(np.all(A @ self.true_source <= b))

    def test_tangent_and_narrow_physical_regions_always_return_safe_points(self):
        for degrees in (13., 77., 203.):
            a = math.radians(degrees)
            source = 1800*np.array([math.cos(a), math.sin(a)])
            station = source-1000*np.array([-math.sin(a), math.cos(a)])
            for error in (-1., -.999999, -.9999):
                first = {'position': dict(zip(['x', 'y'], station)),
                         'svd_deg': degrees+90+error}
                result = choose_second_detection(first, config=FAST_CONFIG)
                self.assertEqual(result['status'], 'OK', (degrees, error, result))
                p = np.array(list(result['selected']['position'].values()))
                self.assertLessEqual(np.linalg.norm(p-source), 999.9+1e-7)
                self.assertTrue(is_safe_candidate(p, result['initial_region']['vertices'], 999.9))

    def test_expired_budget_returns_unused_safe_fallback_with_finite_bound(self):
        config = replace(FAST_CONFIG, calculation_time_limit_s=1e-9)
        initial, _, _ = build_initial_outer_region(self.first, config=config)
        witness = initial['minimum_enclosing_circle']['center']
        result = choose_second_detection(self.first, used_positions=[dict(zip(['x', 'y'], witness))], config=config)
        self.assertEqual(result['status'], 'OK')
        self.assertTrue(result['timed_out'])
        self.assertEqual(result['selection_mode'], 'safe_fallback')
        self.assertEqual(result['evaluated_candidate_count'], 0)
        p = np.array(list(result['selected']['position'].values()))
        self.assertGreater(np.linalg.norm(p-witness), 1e-6)
        self.assertTrue(is_safe_candidate(p, initial['vertices'], 999.9))
        self.assertTrue(math.isfinite(result['selected']['objective_m']))
        self.assertGreaterEqual(result['selected']['worst_updated_cover_radius_m'],
                                initial['minimum_enclosing_circle']['radius_m'])

    def test_precision_is_default_and_cloud_points_are_safe(self):
        self.assertEqual(Q2Config().movement_weight_m_per_s, 0.)
        result = choose_second_detection(self.first, config=FAST_CONFIG)
        self.assertAlmostEqual(result['selected']['objective_m'],
                               result['selected']['worst_updated_cover_radius_m'])
        for p in result['near_best_candidate_cloud']:
            self.assertTrue(is_safe_candidate(np.array([p['x'], p['y']]),
                                             result['initial_region']['vertices'], 999.9))

    def test_response_bound_covers_independent_dense_bearings_and_wrap(self):
        from B.q2.validation import reference_update, reference_circle
        polygon = np.array([[0., -10.], [1000., -10.], [1000., 10.], [0., 10.]])
        for station in (np.array([-50., 0.]), np.array([500., 0.]), np.array([500., 400.])):
            bound = response_radius_bound(polygon, station, config=FAST_CONFIG)
            for angle in np.linspace(-180., 180., 181):
                p, _, _ = reference_update(polygon, station, angle)
                if len(p):
                    _, radius = reference_circle(p)
                    self.assertLessEqual(radius, bound['worst_updated_cover_radius_m']+1e-6)

    def test_interval_limit_preserves_upper_bound_and_near_is_not_zero(self):
        polygon = np.array([[-2., -2.], [2., -2.], [2., 2.], [-2., 2.]])
        bound = response_radius_bound(polygon, np.zeros(2),
                                      config=replace(FAST_CONFIG, max_response_intervals=1))
        self.assertGreaterEqual(bound['worst_updated_cover_radius_m'], math.sqrt(8))
        self.assertFalse(bound['response_bound_converged'])

    def test_timeout_during_split_retains_parent_cover(self):
        polygon = np.array([[0., -10.], [1000., -10.], [1000., 10.], [0., 10.]])
        with patch('B.q2.selection._expired', side_effect=[False, True]):
            bound = response_radius_bound(polygon, np.array([500., 400.]),
                                          config=FAST_CONFIG, deadline=1.)
        self.assertEqual(bound['response_interval_count'], 1)
        self.assertGreaterEqual(bound['worst_updated_cover_radius_m'], math.hypot(500, 10))


if __name__ == "__main__":
    unittest.main()
