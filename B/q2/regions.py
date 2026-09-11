"""Continuous reception predicates and numerical near-optimal area extraction.

The reception predicates apply to the whole convex outer polygon. The U-field
contours are explicitly resolution-dependent approximations, not a proof of
global optimality or of every interpolated point's objective value.
"""
from dataclasses import replace
import math

import numpy as np

from .selection import (Q2Config, _cover_radius, _grid, _deduplicate,
                        is_safe_candidate, response_radius_bound)


def distance_to_polygon(point, vertices):
    p, v = np.asarray(point, float), np.asarray(vertices, float)
    if len(v) == 1:
        return float(np.linalg.norm(p-v[0]))
    edges = np.roll(v, -1, axis=0)-v
    relative = p-v
    cross = edges[:, 0]*relative[:, 1]-edges[:, 1]*relative[:, 0]
    if len(v) >= 3 and np.all(cross >= -1e-8):
        return 0.
    length2 = np.sum(edges*edges, axis=1)
    t = np.clip(np.sum(relative*edges, axis=1)/np.maximum(length2, 1e-30), 0, 1)
    return float(np.linalg.norm(p-(v+t[:, None]*edges), axis=1).min())


def classify_reception(point, vertices, config=None):
    """Conservative three-way classification; unsafe never implies no signal."""
    config = config or Q2Config()
    maximum = float(np.linalg.norm(np.asarray(vertices)-point, axis=1).max())
    minimum = distance_to_polygon(point, vertices)
    if maximum <= config.minimum_reception_radius_m-config.safety_margin_m+1e-9:
        label = 'guaranteed_reception'
    elif minimum > config.maximum_reception_radius_m+1e-9:
        label = 'guaranteed_no_signal'
    else:
        label = 'uncertain'
    return {'classification': label, 'min_distance_to_outer_polygon_m': minimum,
            'max_distance_to_outer_polygon_m': maximum}


def _information_extrema(vertices, first_station, radius, candidate):
    """All possible maximizers of |s-G|²-max(r0²,|s1-G|²).

    Inside the r0 disk the function is convex quadratic; outside it is affine.
    On the circle it is affine in G. Thus polygon vertices, edge/circle
    crossings, and the circle antipode opposite s suffice. Checking ONLY the
    vertices is incorrect for this variable-radius intersection.
    """
    v, origin, s = np.asarray(vertices, float), np.asarray(first_station, float), np.asarray(candidate, float)
    points = list(v)
    if len(v) > 1:
        for a, b in zip(v, np.roll(v, -1, axis=0)):
            d, q = b-a, a-origin
            aa, bb, cc = d@d, 2*(q@d), q@q-radius**2
            discriminant = bb*bb-4*aa*cc
            if aa > 1e-20 and discriminant >= -1e-8:
                root = math.sqrt(max(0., discriminant))
                for t in ((-bb-root)/(2*aa), (-bb+root)/(2*aa)):
                    if -1e-12 <= t <= 1+1e-12:
                        points.append(a+np.clip(t, 0, 1)*d)
    delta = s-origin
    length = np.linalg.norm(delta)
    if length > 1e-12:
        antipode = origin-radius*delta/length
        if distance_to_polygon(antipode, v) <= 1e-7:
            points.append(antipode)
    return np.asarray(points)


def information_violation_m2(candidate, vertices, first_station, minimum_radius=1000.):
    points = _information_extrema(vertices, first_station, minimum_radius, candidate)
    distances2 = np.sum((points-candidate)**2, axis=1)
    first2 = np.sum((points-first_station)**2, axis=1)
    return float(np.max(distances2-np.maximum(minimum_radius**2, first2)))


def is_information_candidate(candidate, vertices, first_station, config=None):
    """Exact C1 membership up to floating point, with no extra 0.1m margin.

    First reception proves R >= max(r0, |G-s1|). Valid for the same source with
    an unchanged omnidirectional reception radius. P+ is an outer approximation.
    """
    config = config or Q2Config()
    return information_violation_m2(np.asarray(candidate), np.asarray(vertices),
                                    np.asarray(first_station), config.minimum_reception_radius_m) <= 1e-7


def information_candidate_region(vertices, first_station, config, fixed_region):
    v = np.asarray(vertices)
    radii = np.maximum(config.minimum_reception_radius_m, np.linalg.norm(v-first_station, axis=1))
    lower, upper = np.max(v-radii[:, None], axis=0), np.min(v+radii[:, None], axis=0)
    return {**fixed_region,
            'definition': 'C1_all_outer_polygon_sources_using_first_reception',
            'guaranteed_radius_m': fixed_region['guaranteed_radius_m'],
            'extra_safety_margin_m': 0.,
            'membership': 'vertices_circle_edge_crossings_and_circle_antipode',
            'assumption': 'same_source_unchanged_omnidirectional_reception_radius',
            'bounding_box': dict(zip(('x_min', 'x_max', 'y_min', 'y_max'),
                                     map(float, (lower[0], upper[0], lower[1], upper[1]))))}


def geometric_features(point, first_station, bearing_deg, vertices):
    """Geometry diagnostics on a deterministic source probe set, not truth input.

    Crossing angle is folded to [0,90]. The reported minimum is over the probes;
    it is not a continuous lower bound over all source positions.
    """
    v, p, s = np.asarray(vertices), np.asarray(point), np.asarray(first_station)
    probes = np.vstack((v, (v+np.roll(v, -1, axis=0))/2, v.mean(axis=0)))
    a, b = probes-s, probes-p
    norms = np.linalg.norm(a, axis=1)*np.linalg.norm(b, axis=1)
    valid = norms > 1e-8
    angles = np.degrees(np.arccos(np.clip(np.abs(np.sum(a[valid]*b[valid], axis=1))/norms[valid], 0., 1.)))
    theta = math.radians(bearing_deg)
    transverse = float((p-s)@np.array([-math.sin(theta), math.cos(theta)]))
    return {'probe_min_crossing_angle_deg': float(angles.min()) if len(angles) else None,
            'probe_median_crossing_angle_deg': float(np.median(angles)) if len(angles) else None,
            'corridor_transverse_distance_m': abs(transverse),
            'corridor_side': int(np.sign(transverse))}


def _polygon_properties(polygon):
    p = np.asarray(polygon)
    q = p-p[0]
    cross = q[:, 0]*np.roll(q[:, 1], -1)-np.roll(q[:, 0], -1)*q[:, 1]
    area2 = cross.sum()
    centroid = (np.sum((q+np.roll(q, -1, axis=0))*cross[:, None], axis=0)/(3*area2)+p[0]
                if abs(area2) > 1e-12 else p.mean(axis=0))
    return abs(float(area2))/2, centroid


def analyze_candidate_region(result, first_observation, spacing_m=30., interval_limit=128,
                             tolerance_m=.25, progress=None):
    """Return full safe-domain heatmap samples and explicit 5% contour polygons.

    Safe samples are triangulated: every triangle is in the convex reception
    region. Contour boundaries, area and centroids interpolate U and must be
    accompanied by grid resolution. Recommendations are directly re-evaluated.
    """
    from scipy.spatial import Delaunay
    import matplotlib.tri as mtri
    from matplotlib.figure import Figure
    if result['status'] != 'OK':
        raise ValueError('A successful selection result is required')
    if not math.isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError('spacing_m must be finite and positive')
    config = Q2Config(**result['config'])
    v = np.asarray(result['initial_region']['vertices'])
    first = np.array([first_observation['position']['x'], first_observation['position']['y']])
    safe = result['safe_candidate_region']
    if config.candidate_region_mode == 'fixed':
        allowed = lambda p: is_safe_candidate(p, v, safe['guaranteed_radius_m'])
    else:
        allowed = lambda p: is_information_candidate(p, v, first, config)
    witness = np.asarray(safe['witness_center'])
    raw = _grid(safe['bounding_box'], spacing_m, witness)
    raw += [np.array([r['position']['x'], r['position']['y']]) for r in result['candidate_scores']]
    # Include the continuous reception boundary, rather than truncating the plot
    # at the last interior grid line.
    for theta in np.linspace(0, 2*np.pi, 144, endpoint=False):
        ray = np.array([math.cos(theta), math.sin(theta)])
        lo, hi = 0., 4*config.maximum_reception_radius_m
        for _ in range(36):
            mid = (lo+hi)/2
            if allowed(witness+mid*ray): lo = mid
            else: hi = mid
        raw.append(witness+max(0., lo-1e-6)*ray)
    points = np.asarray([p for p in _deduplicate(raw) if allowed(p)])
    scoring = replace(config, max_response_intervals=interval_limit, response_bound_tolerance_m=tolerance_m)
    values, gaps, features = [], [], []
    for i, p in enumerate(points):
        r = response_radius_bound(v, p, result['error_deg'], scoring)
        values.append(r['worst_updated_cover_radius_m'])
        gaps.append(r['response_bound_gap_m'])
        features.append(geometric_features(p, first, first_observation['svd_deg'], v))
        if progress and i % 250 == 0: progress(i, len(points))
    values = np.asarray(values)
    best_index = int(np.argmin(values))
    threshold = (1+config.near_best_relative_tolerance)*values[best_index]
    components, triangles = [], np.empty((0, 3), int)
    if len(points) >= 3 and np.linalg.matrix_rank(points-points[0], tol=1e-8) == 2:
        triangles = Delaunay(points).simplices
        tri = mtri.Triangulation(*points.T, triangles)
        fig = Figure()
        ax = fig.subplots()
        cs = ax.tricontourf(tri, values, levels=[-1., threshold])
        # Filled contours may contain holes. Split rings by MOVETO and use the
        # signed winding returned by matplotlib (outer CCW, holes CW).
        for path in cs.get_paths():
            from matplotlib.path import Path
            codes = path.codes
            if codes is None: continue
            starts = np.flatnonzero(codes == Path.MOVETO)
            rings = [path.vertices[a:b] for a, b in zip(starts, np.r_[starts[1:], len(codes)])]
            for ring in rings:
                if len(ring) < 3: continue
                signed = np.sum(ring[:, 0]*np.roll(ring[:, 1], -1)-ring[:, 1]*np.roll(ring[:, 0], -1))/2
                if signed <= 0: continue
                holes = [r for r in rings if len(r) >= 3 and
                         np.sum(r[:, 0]*np.roll(r[:, 1], -1)-r[:, 1]*np.roll(r[:, 0], -1)) < 0
                         and Path(ring).contains_point(r[0])]
                area, center = _polygon_properties(ring)
                moment = area*center
                for hole in holes:
                    ha, hc = _polygon_properties(hole)
                    area -= ha; moment -= ha*hc
                if area <= 1e-10: continue
                center = moment/area
                contained = Path(ring).contains_points(points, radius=1e-7)
                for hole in holes: contained &= ~Path(hole).contains_points(points)
                indices = np.flatnonzero(contained & (values <= threshold+1e-8))
                recommendation = points[indices[np.argmin(values[indices])]] if len(indices) else None
                components.append({'area_m2': float(area), 'centroid': center.tolist(),
                                   'boundary': ring.tolist(), 'holes': [h.tolist() for h in holes],
                                   'recommended_point': recommendation.tolist() if recommendation is not None else None,
                                   'recommended_radius_upper_m': float(values[indices].min()) if len(indices) else None})
        fig.clear()
    e1, e2 = points[triangles[:, 1]]-points[triangles[:, 0]], points[triangles[:, 2]]-points[triangles[:, 0]]
    area = np.abs(e1[:, 0]*e2[:, 1]-e1[:, 1]*e2[:, 0]).sum()/2
    return {'definition': 'C_good = {s in reception region: U(s) <= 1.05 * sampled_U_min}',
            'representation': 'piecewise_linear_contours_over_safe_triangles',
            'global_optimality_certified': False, 'interpolated_objective_certified': False,
            'spacing_m': spacing_m, 'interval_limit': interval_limit, 'bound_tolerance_m': tolerance_m,
            'sampled_min_radius_upper_m': float(values.min()), 'threshold_m': float(threshold),
            'sampled_best_point': points[best_index].tolist(),
            'safe_area_approx_m2': float(area), 'good_area_approx_m2': sum(c['area_m2'] for c in components),
            'components': components, 'points': points.tolist(), 'triangles': triangles.tolist(),
            'radius_upper_m': values.tolist(), 'bound_gap_m': gaps, 'features': features}
