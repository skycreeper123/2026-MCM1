"""Reference 2-D half-plane intersection; metres and counterclockwise degrees.

Question 1 uses only forward bearing wedges, with no artificial bounding box.
SciPy/HiGHS checks feasibility and boundedness before vertices are enumerated.
This is floating-point reference code, not an interval-arithmetic certificate.
"""

from itertools import combinations
import math

import numpy as np
from scipy.optimize import linprog


DEFAULT_TOL_M = 1e-7


def _number(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return value


def bearing_halfplanes(observations, error_deg=1.0):
    """Return A, b for A @ point <= b, including the forward-ray direction.

    Each observation is {"position": {"x": ..., "y": ...}, "svd_deg": ...}.
    Angles wrap automatically. The common half-width must be strictly in (0,90).
    """
    error_deg = _number(error_deg, "error_deg")
    if not 0 < error_deg < 90:
        raise ValueError("error_deg must be strictly between 0 and 90 degrees")
    if not isinstance(observations, (list, tuple)):
        raise ValueError("observations must be an array")
    normals, bounds = [], []
    for i, observation in enumerate(observations):
        if not isinstance(observation, dict) or set(observation) != {"position", "svd_deg"}:
            raise ValueError(f"observation {i} requires exactly position and svd_deg")
        position = observation["position"]
        if not isinstance(position, dict) or set(position) != {"x", "y"}:
            raise ValueError(f"observation {i}.position requires exactly x and y")
        x, y = (_number(position[k], f"observation {i}.{k}") for k in ("x", "y"))
        angle = _number(observation["svd_deg"], f"observation {i}.svd_deg") % 360
        lower, upper = map(math.radians, (angle - error_deg, angle + error_deg))
        for a in ((math.sin(lower), -math.cos(lower)),
                  (-math.sin(upper), math.cos(upper))):
            normals.append(a)
            bounds.append(a[0] * x + a[1] * y)
    return np.asarray(normals, dtype=float).reshape(-1, 2), np.asarray(bounds, dtype=float)


def _result(status, tolerance_m, message=None):
    result = {
        "status": status,
        "vertices": None,
        "dimension": None,
        "area_m2": None,
        "diameter_m": None,
        "diameter_pair": None,
        "diameter_circle": None,
        "diameter_circle_covers": None,
        "minimum_enclosing_circle": None,
        "tolerance_m": tolerance_m,
    }
    if message:
        result["message"] = message
    return result


def _cross(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def _hull(points, tolerance_m):
    unique = []
    for point in sorted(points, key=lambda p: (p[0], p[1])):
        if not any(np.linalg.norm(point - old) <= tolerance_m for old in unique):
            unique.append(point)
    if len(unique) <= 2:
        return np.asarray(unique)
    def chain(sequence):
        out = []
        for p in sequence:
            while len(out) >= 2 and _cross(out[-1] - out[-2], p - out[-1]) <= 0:
                out.pop()
            out.append(p)
        return out
    return np.asarray(chain(unique)[:-1] + chain(reversed(unique))[:-1])


def minimum_enclosing_circle(vertices):
    """Small-case O(v^4) reference: enumerate 1-, 2-, 3-point support centres.

    Radius is always recomputed over ALL vertices, including for nearly
    collinear triples that are skipped. Every returned circle is a cover.
    """
    points = np.asarray(vertices, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points) or not np.isfinite(points).all():
        raise ValueError("vertices must be a nonempty finite (n, 2) array")
    anchor = points[0].copy()
    points = points - anchor
    best_center, best_radius = None, math.inf

    def consider(center):
        nonlocal best_center, best_radius
        radius = float(np.max(np.linalg.norm(points - center, axis=1)))
        if math.isfinite(radius) and radius < best_radius:
            best_center, best_radius = center.copy(), radius

    for p in points:
        consider(p)
    for a, b in combinations(points, 2):
        consider(a + (b - a) / 2)
    for a, b, c in combinations(points, 3):
        u, v = b - a, c - a
        det = _cross(u, v)
        if abs(det) <= 1e-14 * float(np.linalg.norm(u) * np.linalg.norm(v)):
            continue
        u2, v2 = float(u @ u), float(v @ v)
        center = a + np.array([u2 * v[1] - v2 * u[1], u[0] * v2 - v[0] * u2]) / (2 * det)
        consider(center)
    if best_center is None:
        raise ArithmeticError("nonfinite enclosing circle calculation")
    center = best_center + anchor
    # Recheck after restoring the original coordinate system.
    radius = float(np.max(np.linalg.norm(np.asarray(vertices) - center, axis=1)))
    return {"center": center.tolist(), "radius_m": radius}


def solve_halfplanes(A, b, tolerance_m=DEFAULT_TOL_M):
    """Solve a 2-D closed intersection A @ point <= b.

    Status is BOUNDED, EMPTY, UNBOUNDED, or NUMERICAL_ERROR. Vertices of a
    bounded region are counterclockwise, starting at the lexicographic minimum.
    A point/segment has dimension 0/1 and area 0. Non-bounded fields are null.
    Malformed inputs raise ValueError rather than returning a geometric status.
    """
    tolerance_m = _number(tolerance_m, "tolerance_m")
    if tolerance_m < 1e-9:
        raise ValueError("tolerance_m must be >= 1e-9 m (LP feasibility tolerance)")
    A, b = np.asarray(A, dtype=float), np.asarray(b, dtype=float)
    if A.size == 0 and A.shape == (0,):
        A = A.reshape(0, 2)
    if A.ndim != 2 or A.shape[1] != 2 or b.shape != (len(A),):
        raise ValueError("A must have shape (n, 2) and b shape (n,)")
    if not np.isfinite(A).all() or not np.isfinite(b).all():
        raise ValueError("halfplanes must be finite")
    sizes = np.hypot(A[:, 0], A[:, 1])
    zero = sizes == 0
    if np.any(b[zero] < 0):
        return _result("EMPTY", tolerance_m)
    A, b = A[~zero] / sizes[~zero, None], b[~zero] / sizes[~zero]
    if not len(A):
        return _result("UNBOUNDED", tolerance_m)
    if not np.isfinite(A).all() or not np.isfinite(b).all():
        return _result("NUMERICAL_ERROR", tolerance_m, "normalization overflow")

    def lp(objective):
        # Both coordinates are unrestricted, including negative coordinates.
        return linprog(objective, A_ub=A, b_ub=b, bounds=[(None, None)] * 2,
                       method="highs", options={"primal_feasibility_tolerance": 1e-9,
                                                "dual_feasibility_tolerance": 1e-9})
    try:
        feasible = lp([0, 0])
        if feasible.status == 2:
            return _result("EMPTY", tolerance_m)
        if not feasible.success:
            return _result("NUMERICAL_ERROR", tolerance_m, feasible.message)
        extrema = []
        objectives = ([1, 0], [-1, 0], [0, 1], [0, -1])
        for objective in objectives:
            optimum = lp(objective)
            if optimum.status == 3:
                return _result("UNBOUNDED", tolerance_m)
            if not optimum.success:
                return _result("NUMERICAL_ERROR", tolerance_m, optimum.message)
            extrema.append(optimum.x)

        # Translation reduces cancellation for large absolute coordinates.
        origin = feasible.x
        shifted_b = b - A @ origin
        candidates = []
        for i, j in combinations(range(len(A)), 2):
            det = _cross(A[i], A[j])
            if abs(det) <= 1e-14:
                continue
            point = np.array([shifted_b[i] * A[j, 1] - A[i, 1] * shifted_b[j],
                              A[i, 0] * shifted_b[j] - shifted_b[i] * A[j, 0]]) / det
            if np.isfinite(point).all() and np.all(A @ point - shifted_b <= tolerance_m):
                candidates.append(point)
        if not candidates:
            return _result("NUMERICAL_ERROR", tolerance_m, "no reliable boundary intersections")
        vertices = _hull(candidates, tolerance_m) + origin
        if not np.isfinite(vertices).all() or np.max(A @ vertices.T - b[:, None]) > tolerance_m:
            return _result("NUMERICAL_ERROR", tolerance_m, "vertex feasibility check failed")
        # Independent LP extrema detect major omissions in the vertex result.
        for objective, extremum in zip(objectives, extrema):
            error = abs(float(np.min(vertices @ objective) - extremum @ objective))
            if error > 10 * tolerance_m:
                return _result("NUMERICAL_ERROR", tolerance_m, "vertices disagree with LP bounds")

        dimension = min(len(vertices) - 1, 2)
        local = vertices - vertices[0]
        area = abs(sum(_cross(a, c) for a, c in zip(local, np.roll(local, -1, axis=0)))) / 2
        distances = np.linalg.norm(vertices[:, None, :] - vertices[None, :, :], axis=2)
        i, j = np.unravel_index(np.argmax(distances), distances.shape)
        diameter = float(distances[i, j])
        center = vertices[i] + (vertices[j] - vertices[i]) / 2
        max_distance = float(np.max(np.linalg.norm(vertices - center, axis=1)))
        gap = max_distance - diameter / 2
        enclosing_circle = minimum_enclosing_circle(vertices)
        result = _result("BOUNDED", tolerance_m)
        result.update({
            "vertices": vertices.tolist(), "dimension": dimension, "area_m2": float(area),
            "diameter_m": diameter, "diameter_pair": [vertices[i].tolist(), vertices[j].tolist()],
            "diameter_circle": {"center": center.tolist(), "radius_m": diameter / 2,
                                "max_vertex_distance_m": max_distance, "coverage_gap_m": gap},
            "diameter_circle_covers": bool(gap <= tolerance_m),
            "minimum_enclosing_circle": enclosing_circle,
        })
        return result
    except (ArithmeticError, ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
        return _result("NUMERICAL_ERROR", tolerance_m, str(exc))


def solve_bearings(observations, error_deg=1.0, tolerance_m=DEFAULT_TOL_M):
    """Question 1 entry point; no simulator or network access required."""
    A, b = bearing_halfplanes(observations, error_deg)
    result = solve_halfplanes(A, b, tolerance_m)
    result.update({"error_deg": float(error_deg), "observation_count": len(observations),
                   "region_model": "intersection_of_forward_bearing_wedges"})
    return result
