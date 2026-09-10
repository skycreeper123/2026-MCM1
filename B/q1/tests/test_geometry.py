import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np
from scipy.optimize import minimize

from B.q1.geometry import bearing_halfplanes, minimum_enclosing_circle, solve_bearings, solve_halfplanes


def observation(x, y, angle):
    return {"position": {"x": x, "y": y}, "svd_deg": angle}


def rectangle(x0, x1, y0, y1):
    return solve_halfplanes([[1, 0], [-1, 0], [0, 1], [0, -1]], [x1, -x0, y1, -y0])


def clip_polygon(polygon, normal, bound):
    """Independent Sutherland-Hodgman reference used ONLY for bounded tests."""
    out = []
    for p, q in zip(polygon, polygon[1:] + polygon[:1]):
        fp, fq = float(normal @ p - bound), float(normal @ q - bound)
        if fp <= 0:
            out.append(p)
        if (fp <= 0) != (fq <= 0):
            out.append(p + (q - p) * fp / (fp - fq))
    return out


def load_multi_station_case(name):
    path = Path(__file__).resolve().parents[1] / "examples" / name
    data = json.loads(path.read_text(encoding="utf-8"))
    return data, data["observations"]


class GeometryTests(unittest.TestCase):
    def test_analytic_rectangle_and_negative_coordinates(self):
        result = rectangle(-7, -3, -9, -6)
        self.assertEqual(result["status"], "BOUNDED")
        self.assertEqual(result["dimension"], 2)
        self.assertAlmostEqual(result["diameter_m"], 5)
        self.assertAlmostEqual(result["area_m2"], 12)
        self.assertTrue(result["diameter_circle_covers"])
        self.assertAlmostEqual(result["minimum_enclosing_circle"]["radius_m"], 2.5)

    def test_equilateral_triangle_counterexample(self):
        # Vertices (0,0), (2,0), (1,sqrt(3)).
        h = math.sqrt(3)
        result = solve_halfplanes([[0, -1], [-h, 1], [h, 1]], [0, 0, 2 * h])
        self.assertEqual(result["status"], "BOUNDED")
        self.assertAlmostEqual(result["diameter_m"], 2)
        self.assertFalse(result["diameter_circle_covers"])
        self.assertAlmostEqual(result["minimum_enclosing_circle"]["radius_m"], 2 / h)
        self.assertAlmostEqual(result["diameter_circle"]["coverage_gap_m"], h - 1)

    def test_six_station_case_covered_by_diameter_circle(self):
        case, observations = load_multi_station_case("multi_station_covers.json")
        result = solve_bearings(observations, case["error_deg"])
        target = np.array([case["true_source"]["x"], case["true_source"]["y"]])
        A, b = bearing_halfplanes(observations, case["error_deg"])

        self.assertEqual(len(observations), 6)
        self.assertTrue(all(abs(error) <= case["error_deg"] for error in case["injected_errors_deg"]))
        self.assertTrue(np.all(A @ target <= b + 1e-7))
        self.assertEqual(result["status"], "BOUNDED")
        self.assertEqual(result["diameter_circle_covers"], case["expected_diameter_circle_covers"])
        self.assertEqual(result["diameter_circle"]["outside_vertex_indices"], [])
        self.assertAlmostEqual(result["minimum_enclosing_circle"]["radius_m"], result["diameter_m"] / 2)
        self.assertAlmostEqual(result["minimum_enclosing_circle"]["radius_to_half_diameter_ratio"], 1)

    def test_six_station_case_not_covered_by_diameter_circle(self):
        case, observations = load_multi_station_case("multi_station_not_covers.json")
        result = solve_bearings(observations, case["error_deg"])
        target = np.array([case["true_source"]["x"], case["true_source"]["y"]])
        A, b = bearing_halfplanes(observations, case["error_deg"])

        self.assertEqual(len(observations), 6)
        self.assertTrue(all(abs(error) <= case["error_deg"] for error in case["injected_errors_deg"]))
        self.assertTrue(np.all(A @ target <= b + 1e-7))
        self.assertEqual(result["status"], "BOUNDED")
        self.assertEqual(result["diameter_circle_covers"], case["expected_diameter_circle_covers"])
        self.assertTrue(result["diameter_circle"]["outside_vertex_indices"])
        self.assertGreater(result["diameter_circle"]["coverage_gap_m"], 4.5)
        self.assertGreater(result["diameter_circle"]["max_thales_dot_m2"], 0)
        self.assertGreater(result["minimum_enclosing_circle"]["radius_to_half_diameter_ratio"], 1.05)

    def test_point_and_segment(self):
        for bounds, dimension, diameter in [((2, 2, -3, -3), 0, 0), ((-2, 5, 3, 3), 1, 7)]:
            result = rectangle(*bounds)
            self.assertEqual(result["status"], "BOUNDED")
            self.assertEqual(result["dimension"], dimension)
            self.assertAlmostEqual(result["diameter_m"], diameter)
            self.assertEqual(result["area_m2"], 0)
            self.assertTrue(result["diameter_circle_covers"])

    def test_empty(self):
        result = solve_bearings([observation(1, 0, 0), observation(-1, 0, 180)])
        self.assertEqual(result["status"], "EMPTY")
        self.assertIsNone(result["diameter_m"])
        self.assertIsNone(result["diameter_circle_covers"])

    def test_unbounded_plane_wedge_strip_line_and_ray(self):
        cases = [([], []), ([[1, 0]], [1]), ([[0, 1], [0, -1]], [1, 1]),
                 ([[0, 1], [0, -1]], [0, 0]), ([[0, 1], [0, -1], [-1, 0]], [0, 0, 0])]
        for A, b in cases:
            result = solve_halfplanes(A, b)
            self.assertEqual(result["status"], "UNBOUNDED")
            self.assertIsNone(result["diameter_m"])
            self.assertNotIn("Infinity", json.dumps(result, allow_nan=False))
        self.assertEqual(solve_bearings([observation(0, 0, 0)])["status"], "UNBOUNDED")
        self.assertEqual(solve_bearings([])["status"], "UNBOUNDED")

    def test_forwardness_and_zero_degree_wrap(self):
        A, b = bearing_halfplanes([observation(0, 0, 359.9)])
        self.assertTrue(np.all(A @ np.array([100, 0]) <= b))
        self.assertFalse(np.all(A @ np.array([-100, 0]) <= b))
        A2, b2 = bearing_halfplanes([observation(0, 0, -0.1)])
        np.testing.assert_allclose(A, A2, atol=1e-14)
        np.testing.assert_allclose(b, b2)

    def test_nearly_parallel_bounded_bearings(self):
        result = solve_bearings([observation(0, 0, 0), observation(0, 100, -0.01)], error_deg=0.001)
        self.assertEqual(result["status"], "BOUNDED", result)
        self.assertGreater(result["diameter_m"], 100000)

    def test_bearing_degenerate_point(self):
        result = solve_bearings([observation(12, -3, 0), observation(12, -3, 180)])
        self.assertEqual(result["status"], "BOUNDED", result)
        self.assertEqual(result["dimension"], 0)
        np.testing.assert_allclose(result["vertices"], [[12, -3]], atol=1e-7)

    def test_redundant_and_zero_normals(self):
        self.assertEqual(solve_halfplanes([[0, 0]], [-1])["status"], "EMPTY")
        self.assertEqual(solve_halfplanes([[0, 0]], [1])["status"], "UNBOUNDED")
        obs = [observation(0, 0, 45), observation(200, 0, 135)]
        one, repeated = solve_bearings(obs), solve_bearings(obs * 3)
        self.assertAlmostEqual(one["diameter_m"], repeated["diameter_m"])

    def test_scale_and_translation(self):
        original = rectangle(0, 4, 0, 3)
        moved = rectangle(1e6, 1e6 + 40, -2e6, -2e6 + 30)
        self.assertEqual(moved["status"], "BOUNDED")
        self.assertAlmostEqual(moved["diameter_m"], original["diameter_m"] * 10)

    def test_observation_monotonicity_and_known_source(self):
        target = np.array([300., 400.])
        positions = [[0, 0], [600, 0], [300, 1000], [-500, 500]]
        obs = [observation(*p, math.degrees(math.atan2(*(target - p)[::-1]))) for p in positions]
        previous = math.inf
        for count in range(2, 5):
            result = solve_bearings(obs[:count])
            self.assertEqual(result["status"], "BOUNDED")
            self.assertLessEqual(result["diameter_m"], previous + 1e-7)
            A, b = bearing_halfplanes(obs[:count])
            self.assertTrue(np.all(A @ target <= b))
            previous = result["diameter_m"]

    def test_random_regions_against_independent_clipping(self):
        rng = np.random.default_rng(20260910)
        for _ in range(40):
            source = rng.uniform(-500, 500, 2)
            obs = []
            for angle in np.deg2rad([0, 120, 240]):
                p = source + rng.uniform(100, 900) * np.array([math.cos(angle), math.sin(angle)])
                bearing = math.degrees(math.atan2(*(source - p)[::-1])) + rng.uniform(-1, 1)
                obs.append(observation(*p, bearing))
            result = solve_bearings(obs)
            self.assertEqual(result["status"], "BOUNDED", result)
            A, b = bearing_halfplanes(obs)
            self.assertTrue(np.all(A @ source <= b + 1e-7))
            poly = [np.array(p, dtype=float) for p in [(-10000, -10000), (10000, -10000),
                                                       (10000, 10000), (-10000, 10000)]]
            for normal, bound in zip(A, b):
                poly = clip_polygon(poly, normal, bound)
            self.assertTrue(poly)
            self.assertLess(np.max(np.abs(poly)), 9999)  # Bounding box is inactive.
            expected = max(float(np.linalg.norm(p - q)) for p in poly for q in poly)
            self.assertAlmostEqual(result["diameter_m"], expected, places=6)
            actual_vertices = np.asarray(result["vertices"])
            for p in poly:
                self.assertLess(float(np.min(np.linalg.norm(actual_vertices - p, axis=1))), 1e-6)

    def test_circles_against_independent_convex_optimization(self):
        rng = np.random.default_rng(731)
        for _ in range(10):
            points = rng.normal(size=(7, 2)) * 10
            circle = minimum_enclosing_circle(points)
            start = np.r_[points.mean(axis=0), 100.]
            opt = minimize(lambda z: z[2], start, method="SLSQP", bounds=[(None, None)] * 2 + [(0, None)],
                           constraints={"type": "ineq", "fun": lambda z: z[2] - np.linalg.norm(points - z[:2], axis=1)},
                           options={"ftol": 1e-10, "maxiter": 500})
            self.assertTrue(opt.success, opt.message)
            self.assertAlmostEqual(circle["radius_m"], opt.fun, places=6)
            self.assertLessEqual(np.max(np.linalg.norm(points - circle["center"], axis=1)), circle["radius_m"] + 1e-10)

    def test_invalid_inputs(self):
        for error in (0, 90, -1, math.nan, math.inf, True, "1"):
            with self.assertRaises(ValueError):
                solve_bearings([], error_deg=error)
        for obs in ([observation(math.nan, 0, 1)], [observation(0, 0, True)], [{}]):
            with self.assertRaises(ValueError):
                solve_bearings(obs)
        with self.assertRaises(ValueError):
            solve_halfplanes([[1, 0]], [])
        with self.assertRaises(ValueError):
            solve_halfplanes([[1, math.inf]], [0])

    def test_numerical_failure_is_not_empty(self):
        with patch("B.q1.geometry.linprog", side_effect=RuntimeError("solver failure")):
            result = solve_halfplanes([[1, 0]], [0])
        self.assertEqual(result["status"], "NUMERICAL_ERROR")
        self.assertIsNone(result["diameter_m"])


if __name__ == "__main__":
    unittest.main()
