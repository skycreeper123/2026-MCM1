import math
import unittest

import numpy as np

from B.q1.geometry import bearing_halfplanes
from B.q2.selection import (
    Q2Config,
    build_initial_outer_region,
    choose_second_detection,
    circle_outer_halfplanes,
    is_safe_candidate,
    safe_candidate_region,
)


FAST_CONFIG = Q2Config(
    circle_sides=48,
    coarse_spacing_m=200,
    fine_spacing_m=50,
    scenario_spacing_m=200,
    max_coarse_candidates=12,
    max_fine_candidates=12,
    max_source_scenarios=16,
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
        self.assertGreater(result["source_scenario_count"], 5)

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


if __name__ == "__main__":
    unittest.main()
