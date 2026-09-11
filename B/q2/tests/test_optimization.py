"""Regression checks for multi-start ranking and continuous region predicates."""
from dataclasses import asdict, replace
import unittest

import numpy as np

from B.q1.geometry import minimum_enclosing_circle
from B.q2.selection import (Q2Config, _cover_radius, _candidate_points,
                            safe_candidate_region, choose_second_detection, response_radius_bound)
from B.q2.regions import (classify_reception, distance_to_polygon,
                          information_violation_m2, is_information_candidate, analyze_candidate_region)
from B.q2.validation import reference_circle


class OptimizationTests(unittest.TestCase):
    def test_vectorized_cover_matches_independent_support_enumeration(self):
        rng = np.random.default_rng(20260911)
        for n in range(1, 12):
            for scale in (1e-5, 1., 1000.):
                p = rng.normal(size=(n, 2))*scale + [1e5, -2e5]
                self.assertAlmostEqual(_cover_radius(p), reference_circle(p)[1], delta=1e-6)
                self.assertAlmostEqual(_cover_radius(p), minimum_enclosing_circle(p)['radius_m'], delta=1e-6)

    def test_precision_pool_does_not_reserve_current_neighbours(self):
        v = np.array([[-600., -10.], [600., -10.], [600., 10.], [-600., 10.]])
        safe = safe_candidate_region(v)
        args = (v, safe, np.array([-1200., 0.]), 75., 24, [], 1e-6)
        points = _candidate_points(*args, movement_weight=0., bearing_deg=0.)
        self.assertGreater(max(p[1] for p in points), 600)
        self.assertLess(min(p[1] for p in points), -600)
        self.assertTrue(all(np.linalg.norm(v-p, axis=1).max() <= 999.9+1e-8 for p in points))

    def test_multistart_and_selected_precision_are_exposed(self):
        r = choose_second_detection({'position': {'x': -1000., 'y': 0.}, 'svd_deg': .5},
                                    config=Q2Config(max_coarse_candidates=20, max_fine_candidates=9, refinement_starts=3))
        self.assertEqual(len(r['refinement_starts']), 3)
        self.assertEqual(r['algorithm_version'], 3)
        self.assertLessEqual(r['selected']['response_bound_gap_m'], .5)
        self.assertEqual(r['selected'], min(r['candidate_scores'], key=lambda x: (x['objective_m'], x['movement_distance_m'], x['position']['x'], x['position']['y'])))
        self.assertEqual(len(r['candidate_scores']), len({tuple(x['position'].values()) for x in r['candidate_scores']}))

    def test_higher_interval_cap_never_weakens_cover(self):
        v = np.array([[0., -15.], [1500., -15.], [1500., 15.], [0., 15.]])
        for p in (np.array([750., 600.]), np.array([1200., -300.])):
            records = [response_radius_bound(v, p, config=Q2Config(max_response_intervals=n, response_bound_tolerance_m=.05))
                       for n in (32, 128, 256)]
            for a, b in zip(records, records[1:]):
                self.assertLessEqual(b['worst_updated_cover_radius_m'], a['worst_updated_cover_radius_m']+1e-7)
                self.assertGreaterEqual(b['sampled_direction_radius_m'], a['sampled_direction_radius_m']-1e-7)

    def test_information_region_must_not_only_check_vertices(self):
        v = np.array([[0., 0.], [500., 1200.]])
        s = np.array([1000., 0.])
        self.assertTrue(np.all(np.linalg.norm(v-s, axis=1) <= np.maximum(1000., np.linalg.norm(v, axis=1))))
        self.assertFalse(is_information_candidate(s, v, np.zeros(2)))
        self.assertGreater(information_violation_m2(s, v, np.zeros(2)), 1e5)

    def test_information_extrema_dominate_independent_interior_samples(self):
        rng = np.random.default_rng(19)
        v = np.array([[0., -20.], [1500., -20.], [1500., 20.], [0., 20.]])
        w = rng.dirichlet(np.ones(4), 3000)
        probes = w@v
        for p in rng.uniform([-500., -1000.], [1500., 1000.], (20, 2)):
            truth = (np.sum((probes-p)**2, axis=1)-np.maximum(1000**2, np.sum(probes**2, axis=1))).max()
            self.assertGreaterEqual(information_violation_m2(p, v, np.zeros(2))+1e-5, truth)

    def test_three_way_reception_and_edge_distance(self):
        v = np.array([[-600., -20.], [600., -20.], [600., 20.], [-600., 20.]])
        self.assertEqual(classify_reception(np.array([0., 0.]), v)['classification'], 'guaranteed_reception')
        self.assertEqual(classify_reception(np.array([1100., 0.]), v)['classification'], 'uncertain')
        self.assertEqual(classify_reception(np.array([0., 1600.]), v)['classification'], 'guaranteed_no_signal')
        self.assertAlmostEqual(distance_to_polygon(np.array([0., 1600.]), v), 1580.)

    def test_new_configuration_validation(self):
        first = {'position': {'x': 0., 'y': 0.}, 'svd_deg': 0.}
        for kwargs in ({'refinement_starts': 0}, {'final_response_intervals': True},
                       {'near_best_relative_tolerance': -1}, {'candidate_region_mode': 'unknown'}):
            with self.assertRaises(ValueError): choose_second_detection(first, config=Q2Config(**kwargs))

    def test_constant_field_contour_area_and_recommended_point(self):
        # A point source region has U=the numerical margin everywhere. Its whole
        # reception disk is near-optimal, so contour area must match mesh area.
        result = {'status': 'OK', 'config': asdict(Q2Config()), 'error_deg': 1.,
                  'initial_region': {'vertices': [[0., 0.]]},
                  'safe_candidate_region': safe_candidate_region([[0., 0.]], 999.9),
                  'candidate_scores': [{'position': {'x': 100., 'y': 0.}}]}
        first = {'position': {'x': -1000., 'y': 0.}, 'svd_deg': 0.}
        field = analyze_candidate_region(result, first, spacing_m=250.)
        self.assertEqual(len(field['components']), 1)
        self.assertAlmostEqual(field['good_area_approx_m2'], field['safe_area_approx_m2'], delta=1e-6)
        self.assertLess(abs(field['safe_area_approx_m2']-np.pi*999.9**2)/(np.pi*999.9**2), .001)
        self.assertFalse(field['interpolated_objective_certified'])
        self.assertTrue(np.linalg.norm(field['components'][0]['recommended_point']) <= 999.9)


if __name__ == '__main__':
    unittest.main()
