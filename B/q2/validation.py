"""Independent geometric checks shared by the current Q2 validation suite.

The command-line entry forwards to optimization_validation, the current runner.
"""
from itertools import combinations
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull

from .selection import _clip_with_bearing
from B.q1.geometry import minimum_enclosing_circle


def xy(p):
    return np.array([p['x'], p['y']], float)


def reference_update(vertices, station, bearing):
    """Boundary intersection enumeration, independent of production clipping.

    Builds wedge normals directly; ConvexHull recovers the ordered intersection.
    """
    if len(vertices) < 3 or np.linalg.matrix_rank(vertices-vertices[0], tol=1e-8) < 2:
        # A segment requires endpoint constraints too. Its two opposing edge
        # normals alone describe an infinite line, which invalidates the audit.
        distances = np.linalg.norm(vertices[:, None]-vertices[None], axis=2)
        i, j = np.unravel_index(np.argmax(distances), distances.shape)
        a, c = vertices[i], vertices[j]
        direction = (c-a)/distances[i, j] if distances[i, j] > 1e-12 else np.array([1., 0.])
        normal = np.array([-direction[1], direction[0]])
        normals = np.array([-direction, direction, normal, -normal])
        bounds = np.array([-direction@a, direction@c, normal@a, -normal@a])
    else:
        edge = np.roll(vertices, -1, axis=0) - vertices
        normals = np.column_stack((edge[:, 1], -edge[:, 0]))
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        bounds = np.sum(normals * vertices, axis=1)
    lo, hi = np.deg2rad([bearing - 1, bearing + 1])
    wedge = np.array([[np.sin(lo), -np.cos(lo)], [-np.sin(hi), np.cos(hi)]])
    A = np.vstack((normals, wedge))
    b = np.r_[bounds, wedge @ station]
    pairs = np.array(list(combinations(range(len(A)), 2)))
    a, c = A[pairs[:, 0]], A[pairs[:, 1]]
    d, e = b[pairs[:, 0]], b[pairs[:, 1]]
    det = a[:, 0] * c[:, 1] - a[:, 1] * c[:, 0]
    keep = np.abs(det) > 1e-12
    a, c, d, e, det = a[keep], c[keep], d[keep], e[keep], det[keep]
    p = np.column_stack((d*c[:, 1]-a[:, 1]*e, a[:, 0]*e-d*c[:, 0]))/det[:, None]
    p = p[np.all(p @ A.T <= b + 1e-7, axis=1)]
    p = np.unique(np.round(p, 9), axis=0)
    if len(p) >= 3:
        if np.linalg.matrix_rank(p-p[0], tol=1e-8) < 2:
            axis = int(np.argmax(np.ptp(p, axis=0)))
            p = p[[np.argmin(p[:, axis]), np.argmax(p[:, axis])]]
        else:
            p = p[ConvexHull(p).vertices]
    return p, A, b


def reference_circle(p):
    """Enumerate all 1/2/3-point supported disks; no production MEC call."""
    if not len(p):
        raise ValueError('Empty validation intersection')
    origin = p.mean(axis=0)
    q = p-origin
    centers = list(q)
    centers += [(a+b)/2 for a, b in combinations(q, 2)]
    for a, b, c in combinations(q, 3):
        M = 2*np.array([b-a, c-a])
        if abs(np.linalg.det(M)) > 1e-10:
            centers.append(np.linalg.solve(M, [b@b-a@a, c@c-a@a]))
    centers = np.asarray(centers)
    radii = np.linalg.norm(centers[:, None]-q[None], axis=2).max(axis=1)
    k = np.argmin(radii)
    return centers[k]+origin, float(radii[k])


def baseline_points(result, case, rng):
    v = np.asarray(result['initial_region']['vertices'])
    safe = result['safe_candidate_region']
    s = xy(case['first']['position'])
    witness = np.array(safe['witness_center'])
    def allowed(p):
        return np.linalg.norm(p-s) > 1e-6 and np.linalg.norm(v-p, axis=1).max() <= 999.9+1e-9
    box = safe['bounding_box']
    random_point = None
    for _ in range(10000):
        p = rng.uniform([box['x_min'], box['y_min']], [box['x_max'], box['y_max']])
        if allowed(p):
            random_point = p
            break
    angle = np.deg2rad(case['first']['svd_deg'])
    ray = np.array([np.cos(angle), np.sin(angle)])
    forward = next((s+d*ray for d in np.arange(1., 2001.) if allowed(s+d*ray)), None)
    return [witness if allowed(witness) else None, random_point, forward]


def evaluate(case, result, point, sources, errors, crosscheck=False):
    v = np.asarray(result['initial_region']['vertices'])
    initial_radius = result['initial_region']['minimum_enclosing_circle']['radius_m']
    rows = []
    for j, g in enumerate(sources):
        distance = float(np.linalg.norm(point-g))
        near = distance <= 5
        for e in ([None] if near else errors):
            row = {'source_index': j, 'error_deg': e, 'near': near, 'distance_m': distance}
            if near:
                row.update(radius_m=None, diameter_m=None, area_m2=None,
                           center_error_m=None, compression=None, truth_violation_m=0.,
                           radius_reference_difference_m=None, vertex_difference_m=None)
            else:
                bearing = np.rad2deg(np.arctan2(*(g-point)[::-1]))+e
                p, A, b = reference_update(v, point, bearing)
                center, radius = reference_circle(p)
                diameter = np.linalg.norm(p[:, None]-p[None], axis=2).max()
                local = p-p[0]
                area = abs(np.sum(local[:, 0]*np.roll(local[:, 1], -1)-local[:, 1]*np.roll(local[:, 0], -1)))/2
                rd, vd = None, None
                if crosscheck and j < 3 and e in (-1., 0., 1.):
                    production = _clip_with_bearing(v, point, bearing, 1.)
                    rd = abs(minimum_enclosing_circle(production)['radius_m']-radius)
                    distances = np.linalg.norm(production[:, None]-p[None], axis=2)
                    vd = max(distances.min(axis=0).max(), distances.min(axis=1).max())
                row.update(radius_m=radius, diameter_m=float(diameter), area_m2=float(area),
                           center_error_m=float(np.linalg.norm(center-g)), compression=1-radius/initial_radius,
                           truth_violation_m=max(0., float(np.max(A@g-b))),
                           radius_reference_difference_m=rd, vertex_difference_m=vd)
            rows.append(row)
    radii = [r['radius_m'] for r in rows if not r['near']]
    return {'point': point.tolist(), 'movement_time_s': float(np.linalg.norm(point-xy(case['first']['position']))/5),
            'safety_margin_m': float(1000-np.linalg.norm(v-point, axis=1).max()),
            'worst_radius_m': max(radii) if radii else None, 'rows': rows}


def dump(path, value):
    def convert(x):
        if isinstance(x, np.ndarray): return x.tolist()
        if isinstance(x, np.generic): return x.item()
        raise TypeError(type(x).__name__)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, default=convert, allow_nan=False), encoding='utf-8')


def save(fig, out, name):
    for ext in ('png', 'svg', 'pdf'):
        fig.savefig(out/f'{name}.{ext}', dpi=300, bbox_inches='tight', facecolor='white')
    plt.close(fig)


if __name__ == '__main__':
    from .optimization_validation import main
    main()
