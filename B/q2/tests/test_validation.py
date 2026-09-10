"""Analytic checks for the independent validation geometry and near semantics."""
import unittest

import numpy as np

from B.q2.validation import reference_circle, reference_update, evaluate


class ValidationGeometryTests(unittest.TestCase):
    def test_equilateral_enclosing_circle(self):
        points = np.array([[0., 0.], [2., 0.], [1., np.sqrt(3)]])
        center, radius = reference_circle(points)
        np.testing.assert_allclose(center, [1., np.sqrt(3)/3], atol=1e-10)
        self.assertAlmostEqual(radius, 2/np.sqrt(3))

    def test_square_cut_by_one_degree_wedge(self):
        square = np.array([[1., -1.], [2., -1.], [2., 1.], [1., 1.]])
        points, _, _ = reference_update(square, np.zeros(2), 0.)
        slope = np.tan(np.deg2rad(1))
        expected = np.array([[1., -slope], [2., -2*slope], [2., 2*slope], [1., slope]])
        distances = np.linalg.norm(points[:, None]-expected[None], axis=2)
        self.assertLess(distances.min(axis=0).max(), 1e-8)
        self.assertLess(distances.min(axis=1).max(), 1e-8)

    def test_near_is_not_zero_geometric_radius(self):
        case = {'first': {'position': {'x': -10., 'y': 0.}}}
        result = {'initial_region': {'vertices': [[-2., -2.], [2., -2.], [2., 2.], [-2., 2.]],
                                    'minimum_enclosing_circle': {'radius_m': 3.}}}
        evaluation = evaluate(case, result, np.zeros(2), np.array([[1., 0.]]), [-1., 0., 1.])
        self.assertEqual(len(evaluation['rows']), 1)
        self.assertTrue(evaluation['rows'][0]['near'])
        self.assertIsNone(evaluation['rows'][0]['radius_m'])
        self.assertIsNone(evaluation['worst_radius_m'])


if __name__ == '__main__':
    unittest.main()
