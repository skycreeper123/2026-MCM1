"""Question 2 second detection point selection for an omnidirectional source.

The safe candidate region is continuous and guaranteed: every candidate is at
most the minimum possible reception radius from every point in a conservative
outer localization polygon. Bearing-interval envelopes bound every possible
direction response. Candidate search remains finite, not globally optimal.
"""

from dataclasses import asdict, dataclass, replace
from itertools import combinations
import math
import time

import numpy as np

from B.q1.geometry import bearing_halfplanes, minimum_enclosing_circle, solve_halfplanes


@dataclass(frozen=True)
class Q2Config:
    target_center_x: float = 0.0
    target_center_y: float = 0.0
    target_radius_m: float = 1800.0
    maximum_reception_radius_m: float = 1500.0
    minimum_reception_radius_m: float = 1000.0
    safety_margin_m: float = 0.1
    near_radius_m: float = 5.0
    movement_speed_mps: float = 5.0
    movement_weight_m_per_s: float = 0.0
    circle_sides: int = 128
    coarse_spacing_m: float = 75.0
    fine_spacing_m: float = 15.0
    max_coarse_candidates: int = 64
    max_fine_candidates: int = 49  # per refinement start
    refinement_starts: int = 5
    response_bound_tolerance_m: float = 0.5
    max_response_intervals: int = 32  # preliminary screening
    shortlist_response_intervals: int = 128
    final_response_intervals: int = 256
    maximum_refinement_intervals: int = 1024
    final_bound_tolerance_m: float = 0.05
    polish_starts: int = 2
    near_best_relative_tolerance: float = 0.05
    candidate_region_mode: str = "fixed"  # fixed guarantee or first-reception information
    near_best_epsilon_m: float = 10.0
    repeated_position_tolerance_m: float = 1e-6
    calculation_time_limit_s: float | None = None


def _finite_number(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return value


def _point(value, name):
    if not isinstance(value, dict) or set(value) != {"x", "y"}:
        raise ValueError(f"{name} must contain exactly x and y")
    return np.array([_finite_number(value["x"], f"{name}.x"),
                     _finite_number(value["y"], f"{name}.y")])


def _validate_config(config):
    _finite_number(config.target_center_x, "target_center_x")
    _finite_number(config.target_center_y, "target_center_y")
    numeric_positive = (
        "target_radius_m", "maximum_reception_radius_m", "minimum_reception_radius_m",
        "movement_speed_mps", "circle_sides", "coarse_spacing_m", "fine_spacing_m",
        "max_coarse_candidates", "max_fine_candidates",
        "response_bound_tolerance_m", "max_response_intervals", "near_best_epsilon_m",
        "refinement_starts", "shortlist_response_intervals", "final_response_intervals",
        "maximum_refinement_intervals",
        "final_bound_tolerance_m", "polish_starts",
    )
    for name in numeric_positive:
        if _finite_number(getattr(config, name), name) <= 0:
            raise ValueError(f"{name} must be positive")
    if config.circle_sides < 16 or int(config.circle_sides) != config.circle_sides:
        raise ValueError("circle_sides must be an integer >= 16")
    for name in ("max_coarse_candidates", "max_fine_candidates", "max_response_intervals",
                 "refinement_starts", "shortlist_response_intervals", "final_response_intervals",
                 "maximum_refinement_intervals", "polish_starts"):
        if int(getattr(config, name)) != getattr(config, name):
            raise ValueError(f"{name} must be an integer")
    safety_margin = _finite_number(config.safety_margin_m, "safety_margin_m")
    near_radius = _finite_number(config.near_radius_m, "near_radius_m")
    movement_weight = _finite_number(config.movement_weight_m_per_s, "movement_weight_m_per_s")
    repeated_tolerance = _finite_number(
        config.repeated_position_tolerance_m, "repeated_position_tolerance_m"
    )
    if not 0 <= safety_margin < config.minimum_reception_radius_m:
        raise ValueError("safety_margin_m must be in [0, minimum_reception_radius_m)")
    if not 0 <= near_radius < config.minimum_reception_radius_m:
        raise ValueError("near_radius_m must be in [0, minimum_reception_radius_m)")
    if movement_weight < 0:
        raise ValueError("movement_weight_m_per_s must be nonnegative")
    if repeated_tolerance < 0:
        raise ValueError("repeated_position_tolerance_m must be nonnegative")
    if config.maximum_reception_radius_m < config.minimum_reception_radius_m:
        raise ValueError("maximum_reception_radius_m must be at least minimum_reception_radius_m")
    if _finite_number(config.near_best_relative_tolerance, "near_best_relative_tolerance") < 0:
        raise ValueError("near_best_relative_tolerance must be nonnegative")
    if config.candidate_region_mode not in ("fixed", "information"):
        raise ValueError("candidate_region_mode must be fixed or information")
    if config.calculation_time_limit_s is not None:
        if _finite_number(config.calculation_time_limit_s, "calculation_time_limit_s") <= 0:
            raise ValueError("calculation_time_limit_s must be positive or None")


def circle_outer_halfplanes(center, radius_m, sides=128):
    """Circumscribed regular polygon A@x<=b that contains a closed disk."""
    center = np.asarray(center, dtype=float)
    radius_m = _finite_number(radius_m, "radius_m")
    if center.shape != (2,) or not np.isfinite(center).all() or radius_m <= 0:
        raise ValueError("center must be a finite 2-D point and radius_m positive")
    if isinstance(sides, bool) or int(sides) != sides or sides < 16:
        raise ValueError("sides must be an integer >= 16")
    angles = 2 * math.pi * np.arange(int(sides)) / int(sides)
    A = np.column_stack((np.cos(angles), np.sin(angles)))
    return A, radius_m + A @ center


def build_initial_outer_region(first_observation, error_deg=1.0, config=None):
    """Outer polygon for wedge ∩ target disk ∩ first-station 1500m disk."""
    config = config or Q2Config()
    _validate_config(config)
    if not isinstance(first_observation, dict) or set(first_observation) != {"position", "svd_deg"}:
        raise ValueError("first_observation requires exactly position and svd_deg")
    station = _point(first_observation["position"], "first_observation.position")
    bearing = _finite_number(first_observation["svd_deg"], "first_observation.svd_deg")
    target_center = np.array([config.target_center_x, config.target_center_y], dtype=float)
    A_wedge, b_wedge = bearing_halfplanes(
        [{"position": {"x": station[0], "y": station[1]}, "svd_deg": bearing}], error_deg
    )
    A_target, b_target = circle_outer_halfplanes(target_center, config.target_radius_m, config.circle_sides)
    A_receive, b_receive = circle_outer_halfplanes(
        station, config.maximum_reception_radius_m, config.circle_sides
    )
    A = np.vstack((A_wedge, A_target, A_receive))
    b = np.concatenate((b_wedge, b_target, b_receive))
    result = solve_halfplanes(A, b)
    return result, A, b


def safe_candidate_region(vertices, guaranteed_radius_m=999.9):
    """Describe ∩_v B(v, guaranteed_radius_m) exactly by membership and a witness.

    For a convex source region, max distance from a candidate to the region is
    attained at a vertex. The intersection is nonempty iff the vertices' MEC
    radius does not exceed guaranteed_radius_m.
    """
    points = np.asarray(vertices, dtype=float)
    guaranteed_radius_m = _finite_number(guaranteed_radius_m, "guaranteed_radius_m")
    if guaranteed_radius_m <= 0 or points.ndim != 2 or points.shape[1] != 2 or not len(points):
        raise ValueError("vertices must be a nonempty finite (n, 2) array")
    if not np.isfinite(points).all():
        raise ValueError("vertices must be finite")
    mec = minimum_enclosing_circle(points)
    lower = np.max(points - guaranteed_radius_m, axis=0)
    upper = np.min(points + guaranteed_radius_m, axis=0)
    nonempty = mec["radius_m"] <= guaranteed_radius_m + 1e-9
    return {
        "nonempty": bool(nonempty),
        "definition": "intersection_of_closed_disks_centered_at_outer_region_vertices",
        "guaranteed_radius_m": guaranteed_radius_m,
        "witness_center": mec["center"] if nonempty else None,
        "witness_max_distance_m": mec["radius_m"] if nonempty else None,
        "witness_margin_m": guaranteed_radius_m - mec["radius_m"] if nonempty else None,
        "bounding_box": {"x_min": float(lower[0]), "x_max": float(upper[0]),
                         "y_min": float(lower[1]), "y_max": float(upper[1])},
    }


def is_safe_candidate(candidate, vertices, guaranteed_radius_m, tolerance_m=1e-9):
    candidate = np.asarray(candidate, dtype=float)
    vertices = np.asarray(vertices, dtype=float)
    if candidate.shape != (2,) or vertices.ndim != 2 or vertices.shape[1] != 2:
        raise ValueError("candidate must be (2,) and vertices (n, 2)")
    return bool(np.max(np.linalg.norm(vertices - candidate, axis=1))
                <= guaranteed_radius_m + tolerance_m)


def _clip_halfplane(polygon, normal, bound, tolerance_m=1e-8):
    polygon = [np.asarray(point, dtype=float) for point in polygon]
    output = []
    for p, q in zip(polygon, polygon[1:] + polygon[:1]):
        fp, fq = float(normal @ p - bound), float(normal @ q - bound)
        p_inside, q_inside = fp <= tolerance_m, fq <= tolerance_m
        if p_inside:
            output.append(p)
        if p_inside != q_inside:
            denominator = fp - fq
            if denominator != 0:
                output.append(p + (q - p) * fp / denominator)
    if not output:
        return np.empty((0, 2))
    unique = []
    for point in output:
        if not unique or np.linalg.norm(point - unique[-1]) > tolerance_m:
            unique.append(point)
    if len(unique) > 1 and np.linalg.norm(unique[0] - unique[-1]) <= tolerance_m:
        unique.pop()
    return np.asarray(unique)


def _clip_with_bearing(vertices, station, bearing_deg, error_deg):
    observation = {"position": {"x": float(station[0]), "y": float(station[1])},
                   "svd_deg": float(bearing_deg)}
    A, b = bearing_halfplanes([observation], error_deg)
    polygon = np.asarray(vertices, dtype=float)
    for normal, bound in zip(A, b):
        polygon = _clip_halfplane(polygon, normal, bound)
        if not len(polygon):
            break
    return polygon


def clip_convex_polygon(vertices, A, b, tolerance_m=1e-8):
    """Clip a convex polygon by public ``A @ x <= b`` constraints.

    This is the supported incremental-update primitive used by Q3.  It keeps
    Q2's clipping tolerances and avoids downstream modules importing private
    helpers.
    """
    polygon = np.asarray(vertices, dtype=float)
    A = np.asarray(A, dtype=float)
    b = np.asarray(b, dtype=float)
    tolerance_m = _finite_number(tolerance_m, "tolerance_m")
    if polygon.ndim != 2 or polygon.shape[1] != 2 or not len(polygon) \
            or not np.isfinite(polygon).all():
        raise ValueError("vertices must be a nonempty finite (n, 2) array")
    if A.ndim != 2 or A.shape[1] != 2 or b.shape != (len(A),) \
            or not np.isfinite(A).all() or not np.isfinite(b).all():
        raise ValueError("A and b must contain matching finite 2-D constraints")
    if tolerance_m < 0:
        raise ValueError("tolerance_m must be nonnegative")
    for normal, bound in zip(A, b):
        polygon = _clip_halfplane(polygon, normal, bound, tolerance_m)
        if not len(polygon):
            break
    return polygon


def _deduplicate(points, tolerance_m=1e-7):
    unique = []
    buckets = {}
    for point in points:
        point = np.asarray(point, dtype=float)
        cell = tuple(np.floor(point/tolerance_m).astype(np.int64))
        neighbors = (buckets.get((cell[0]+i, cell[1]+j), []) for i in (-1, 0, 1) for j in (-1, 0, 1))
        if any(np.linalg.norm(point-old) <= tolerance_m for group in neighbors for old in group):
            continue
        unique.append(point)
        buckets.setdefault(cell, []).append(point)
    return unique


def _diverse_subset(points, limit, anchor=None):
    points = _deduplicate(points)
    if len(points) <= limit:
        return points
    array = np.asarray(points)
    if anchor is None:
        anchor = np.mean(array, axis=0)
    first = int(np.argmin(np.linalg.norm(array - anchor, axis=1)))
    selected = [first]
    distances = np.sum((array-array[first])**2, axis=1)
    distances[first] = -1
    while len(selected) < limit:
        index = int(np.argmax(distances))
        selected.append(index)
        distances = np.minimum(distances, np.sum((array-array[index])**2, axis=1))
        distances[selected] = -1
    return [array[i] for i in selected]


def _grid(bounds, spacing, anchor):
    lower = np.array([bounds["x_min"], bounds["y_min"]])
    upper = np.array([bounds["x_max"], bounds["y_max"]])
    if np.any(lower > upper):
        return []
    axes = []
    for lo, hi, base in zip(lower, upper, anchor):
        start = base + math.ceil((lo - base) / spacing) * spacing
        axes.append(np.arange(start, hi + spacing * 1e-9, spacing))
    return [np.array([x, y]) for x in axes[0] for y in axes[1]]


def _candidate_points(vertices, safe_region, current, spacing, limit, used_positions,
                      repeated_position_tolerance_m, movement_weight=0., bearing_deg=0.,
                      membership=None):
    radius = safe_region["guaranteed_radius_m"]
    witness = np.asarray(safe_region["witness_center"])
    allowed = membership or (lambda p: is_safe_candidate(p, vertices, radius))
    # Analytic radial boundary representatives, including BOTH corridor sides.
    # The region is an intersection of disks, hence convex about the witness.
    features = []
    for angle in np.deg2rad(bearing_deg + np.arange(0., 360., 30.)):
        direction = np.array([math.cos(angle), math.sin(angle)])
        lo, hi = 0., 4*radius
        for _ in range(36):
            mid = (lo+hi)/2
            if allowed(witness+mid*direction): lo = mid
            else: hi = mid
        features.append(witness + max(0., lo-1e-6)*direction)
    raw = [witness, np.mean(vertices, axis=0), current] + features
    raw.extend(_grid(safe_region["bounding_box"], spacing, witness))
    filtered = [p for p in _deduplicate(raw)
                if allowed(p)
                and not any(np.linalg.norm(p - old) <= repeated_position_tolerance_m
                            for old in used_positions)]
    if len(filtered) <= limit:
        return filtered
    # Distance quotas are only appropriate when distance enters the objective.
    priority = [p for p in features if all(np.linalg.norm(p-q) > repeated_position_tolerance_m
                                         for q in used_positions)]
    if movement_weight > 0:
        priority = sorted(filtered, key=lambda p: (np.linalg.norm(p-current), p[0], p[1]))[:max(1, limit//3)] + priority
    priority = _deduplicate(priority)[:max(1, limit//2)]
    others = _diverse_subset(filtered, limit, witness)
    return _deduplicate(priority + others)[:limit]


def _cover_radius(points):
    """Vectorized support-circle enumeration; recheck every point of the cover.

    Same 1/2/3-support definition as Q1, with translated coordinates for stability.
    Keeping this local leaves the independent Q1 implementation unchanged.
    """
    points = np.asarray(points, dtype=float)
    p = points-points[0]
    centers = [*p]
    pairs = list(combinations(range(len(p)), 2))
    if pairs:
        ij = np.asarray(pairs)
        centers.extend((p[ij[:, 0]]+p[ij[:, 1]])/2)
    triples = list(combinations(range(len(p)), 3))
    if triples:
        ijk = np.asarray(triples)
        a = p[ijk[:, 0]]
        u, v = p[ijk[:, 1]]-a, p[ijk[:, 2]]-a
        det = u[:, 0]*v[:, 1]-u[:, 1]*v[:, 0]
        valid = np.abs(det) > 1e-14*np.linalg.norm(u, axis=1)*np.linalg.norm(v, axis=1)
        a, u, v, det = a[valid], u[valid], v[valid], det[valid]
        u2, v2 = np.sum(u*u, axis=1), np.sum(v*v, axis=1)
        centers.extend(a+np.column_stack((u2*v[:, 1]-v2*u[:, 1], u[:, 0]*v2-v[:, 0]*u2))/(2*det[:, None]))
    centers = np.asarray(centers)
    radii = np.linalg.norm(centers[:, None]-p[None], axis=2).max(axis=1)
    center = centers[int(np.argmin(radii))]+points[0]
    return float(np.linalg.norm(points-center, axis=1).max())


def _expired(deadline):
    return deadline is not None and time.monotonic() >= deadline


def _response_domain(vertices, candidate, error_deg):
    """An unwrapped interval containing all possible measured bearings.

    If vertex rays lie in an open semicircle, convex combinations stay in that
    cone. Otherwise use the full circle (also handles a station inside P).
    """
    relative = np.asarray(vertices) - candidate
    if np.min(np.linalg.norm(relative, axis=1)) <= 1e-7:
        return 0.0, 360.0
    angles = np.sort(np.degrees(np.arctan2(relative[:, 1], relative[:, 0])) % 360)
    gaps = np.diff(np.r_[angles, angles[0] + 360])
    index = int(np.argmax(gaps))
    width = 360.0 - gaps[index]
    if width >= 180.0 - 1e-8:
        return 0.0, 360.0
    start = angles[(index + 1) % len(angles)]
    # Include rounding margin and measurement error at both ends.
    return float(start - error_deg - 1e-8), float(start + width + error_deg + 1e-8)


def response_radius_bound(vertices, candidate, error_deg=1.0, config=None,
                          deadline=None, initial_radius=None):
    """Bound MEC(P intersect W(candidate, theta, delta)) for EVERY theta.

    For theta in [mid-h,mid+h], W(theta,delta) is contained in
    W(mid,delta+h). Its intersection with P supplies a covering circle for all
    responses in that interval. The maximum envelope radius is therefore an
    upper bound, even if refinement stops early. Numerical margins are explicit;
    this is floating-point geometry, not an interval-arithmetic certificate.
    """
    config = config or Q2Config()
    vertices = np.asarray(vertices, dtype=float)
    candidate = np.asarray(candidate, dtype=float)
    initial_radius = (minimum_enclosing_circle(vertices)['radius_m']
                      if initial_radius is None else initial_radius)
    numeric_margin = 1e-6
    whole_upper = initial_radius + numeric_margin
    lo, hi = _response_domain(vertices, candidate, error_deg)
    # Start with a valid bound BEFORE evaluating or checking time.
    cells = [(lo, hi, whole_upper)]
    sampled_max = 0.0
    evaluations = 0

    def interval(a, b):
        nonlocal sampled_max, evaluations
        mid = (a + b) / 2
        halfwidth = error_deg + (b - a) / 2
        if halfwidth >= 90:
            upper = whole_upper
        else:
            outer = _clip_with_bearing(vertices, candidate, mid, halfwidth)
            upper = (min(whole_upper, _cover_radius(outer) + numeric_margin)
                     if len(outer) else 0.0)
        exact = _clip_with_bearing(vertices, candidate, mid, error_deg)
        if len(exact):
            sampled_max = max(sampled_max, _cover_radius(exact))
        evaluations += 1
        return a, b, upper

    while len(cells) < config.max_response_intervals and not _expired(deadline):
        k = max(range(len(cells)), key=lambda i: cells[i][2])
        a, b, upper = cells[k]
        if upper - sampled_max <= config.response_bound_tolerance_m:
            break
        midpoint = (a + b) / 2
        if midpoint == a or midpoint == b:
            break
        left = interval(a, midpoint)
        if _expired(deadline):
            # Keep the unsplit parent: a partial partition is not a bound.
            break
        right = interval(midpoint, b)
        cells[k:k+1] = [left, right]
    upper = max(c[2] for c in cells)
    return {
        'worst_updated_cover_radius_m': float(upper),
        'sampled_direction_radius_m': float(sampled_max),
        'response_bound_gap_m': float(max(0., upper - sampled_max)),
        'response_bound_converged': bool(upper - sampled_max <= config.response_bound_tolerance_m),
        'response_interval_count': len(cells),
        'response_evaluations': evaluations,
        'radius_bound_scope': 'all_direction_responses_over_initial_outer_polygon',
        'numerical_margin_m': numeric_margin,
    }


def _safe_fallback(vertices, safe, used, tolerance):
    """Construct a safe unused point without depending on a source/grid sample."""
    center = np.asarray(safe['witness_center'])
    radius = safe['guaranteed_radius_m']
    slack = radius - float(np.linalg.norm(vertices-center, axis=1).max())
    points = [center]
    # The ball of radius slack around the witness lies in the safe region.
    if slack > 2*tolerance:
        for angle in np.linspace(0, 2*math.pi, 2*len(used)+8, endpoint=False):
            points.append(center + slack/2*np.array([math.cos(angle), math.sin(angle)]))
    return next((p for p in points if is_safe_candidate(p, vertices, radius)
                 and all(np.linalg.norm(p-old) > tolerance for old in used)), None)


def _score_candidate(candidate, vertices, current, error_deg, config, deadline, initial_radius):
    bound = response_radius_bound(vertices, candidate, error_deg, config, deadline, initial_radius)
    distance = float(np.linalg.norm(candidate-current))
    move_time = distance/config.movement_speed_mps
    return {'position': {'x': float(candidate[0]), 'y': float(candidate[1])},
            'movement_distance_m': distance, 'movement_time_s': move_time,
            'objective_m': bound['worst_updated_cover_radius_m'] + config.movement_weight_m_per_s*move_time,
            **bound}


def choose_second_detection(first_observation, error_deg=1.0, current_position=None,
                            used_positions=None, config=None):
    """Return a safe second point, using precision-first conservative ranking.

    calculation_time_limit_s is a cooperative whole-call budget: geometry and
    individual operations cannot be preempted. Expiry skips further ranking and
    returns the best safe point available, with elapsed time and overrun exposed.
    """
    started = time.monotonic()
    config = config or Q2Config()
    _validate_config(config)
    deadline = started + config.calculation_time_limit_s if config.calculation_time_limit_s is not None else None
    error_deg = _finite_number(error_deg, 'error_deg')
    if not 0 < error_deg < 90:
        raise ValueError('error_deg must be strictly between 0 and 90 degrees')
    # Validate caller inputs even if the geometry later has no solution.
    if not isinstance(first_observation, dict) or set(first_observation) != {'position', 'svd_deg'}:
        raise ValueError('first_observation requires exactly position and svd_deg')
    station = _point(first_observation['position'], 'first_observation.position')
    _finite_number(first_observation['svd_deg'], 'first_observation.svd_deg')
    current = station if current_position is None else _point(current_position, 'current_position')
    used = [station]
    if used_positions is not None:
        if not isinstance(used_positions, (list, tuple)):
            raise ValueError('used_positions must be an array or None')
        used.extend(_point(p, f'used_positions[{i}]') for i, p in enumerate(used_positions))

    def finish(result):
        elapsed = time.monotonic() - started
        result.update(elapsed_s=elapsed, timed_out=_expired(deadline),
                      budget_overrun_s=max(0., elapsed-config.calculation_time_limit_s)
                      if deadline is not None else 0.,
                      time_budget_semantics='cooperative_whole_call_not_hard_deadline')
        return result

    initial, _, _ = build_initial_outer_region(first_observation, error_deg, config)
    if initial['status'] != 'BOUNDED':
        return finish({'status': 'GEOMETRY_ERROR', 'reason': initial['status'],
                       'initial_region': initial, 'selected': None})
    vertices = np.asarray(initial['vertices'])
    initial_radius = initial['minimum_enclosing_circle']['radius_m']
    guaranteed_radius = config.minimum_reception_radius_m-config.safety_margin_m
    safe = safe_candidate_region(vertices, guaranteed_radius)
    if config.candidate_region_mode == 'information' and safe['nonempty']:
        from .regions import information_candidate_region
        safe = information_candidate_region(vertices, station, config, safe)
    if config.candidate_region_mode == 'information':
        from .regions import is_information_candidate
        allowed = lambda p: is_information_candidate(p, vertices, station, config)
    else:
        allowed = lambda p: is_safe_candidate(p, vertices, guaranteed_radius)
    base = {'initial_region': initial, 'safe_candidate_region': safe, 'config': asdict(config),
            'error_deg': error_deg, 'algorithm_version': 3,
            'ranking_is_discrete_approximation': True,
            'ranking_description': 'finite_candidate_search_with_continuous_response_upper_bounds',
            'reception_guarantee_scope': 'omnidirectional_source_only'}
    if not safe['nonempty']:
        return finish({'status': 'NO_CANDIDATE', 'reason': 'SAFE_REGION_EMPTY', 'selected': None, **base})
    fallback = _safe_fallback(vertices, safe, used, config.repeated_position_tolerance_m)
    if fallback is None:
        return finish({'status': 'NO_CANDIDATE', 'reason': 'NO_UNUSED_SAFE_WITNESS', 'selected': None, **base})
    # Universal initial-region cover is available even with zero ranking budget.
    fallback_score = _score_candidate(fallback, vertices, current, error_deg, config,
                                      -math.inf, initial_radius)
    # Store one score per position. Re-evaluation replaces the coarse bound;
    # stale, less accurate scores must not participate in the final ranking.
    point_key = lambda p: tuple(np.round(p, 7))
    scored = {point_key(fallback): fallback_score}
    ranked_count = 0
    refinement_log = []

    def score_points(points, interval_limit=None):
        nonlocal ranked_count
        stage_config = config if interval_limit is None else replace(
            config, max_response_intervals=interval_limit,
            response_bound_tolerance_m=min(config.response_bound_tolerance_m, config.final_bound_tolerance_m))
        for p in points:
            if _expired(deadline):
                break
            score = _score_candidate(p, vertices, current, error_deg, stage_config, deadline, initial_radius)
            old = scored.get(point_key(p))
            # Timeout may leave this evaluation less refined than its predecessor.
            if old is None or score['worst_updated_cover_radius_m'] <= old['worst_updated_cover_radius_m']:
                score['scoring_interval_limit'] = stage_config.max_response_intervals
                scored[point_key(p)] = score
            ranked_count += 1

    if not _expired(deadline):
        coarse = _candidate_points(vertices, safe, current, config.coarse_spacing_m,
                                   config.max_coarse_candidates, used, config.repeated_position_tolerance_m,
                                   config.movement_weight_m_per_s, float(first_observation['svd_deg']), allowed)
        score_points(_deduplicate([fallback] + coarse))
    key = lambda r: (r['objective_m'], r['movement_distance_m'], r['position']['x'], r['position']['y'])
    aspoint = lambda r: np.array([r['position']['x'], r['position']['y']])
    # Loose coarse upper bounds can conceal a good basin. Audit leaders under
    # BOTH sampled lower objective and conservative upper objective first.
    lower_key = lambda r: r['sampled_direction_radius_m'] + config.movement_weight_m_per_s*r['movement_time_s']
    coarse_leaders = sorted(scored.values(), key=key)[:5] + sorted(scored.values(), key=lower_key)[:5]
    score_points(_deduplicate([aspoint(r) for r in coarse_leaders]), config.shortlist_response_intervals)
    starts = []
    for record in sorted(scored.values(), key=key):
        p = aspoint(record)
        if all(np.linalg.norm(p-q) >= config.coarse_spacing_m for q in starts):
            starts.append(p)
        if len(starts) >= config.refinement_starts:
            break
    if not _expired(deadline):
        for best_point in starts:
            if _expired(deadline): break
            local_bounds = {'x_min': best_point[0]-config.coarse_spacing_m,
                            'x_max': best_point[0]+config.coarse_spacing_m,
                            'y_min': best_point[1]-config.coarse_spacing_m,
                            'y_max': best_point[1]+config.coarse_spacing_m}
            raw = _grid(local_bounds, config.fine_spacing_m, best_point)
            fine = _diverse_subset([p for p in raw if allowed(p)
                                   and all(np.linalg.norm(p-old) > config.repeated_position_tolerance_m for old in used)],
                                   config.max_fine_candidates, best_point)
            score_points([p for p in fine if point_key(p) not in scored])

    # Screen all positions cheaply; spend precision on promising positions.
    # Re-sort after EACH stage, including any change caused by tighter bounds.
    for count, limit in ((5, config.shortlist_response_intervals), (3, config.final_response_intervals)):
        before = min(scored.values(), key=key)['position']
        leaders = sorted(scored.values(), key=key)[:count]
        score_points([aspoint(r) for r in leaders
                      if not r['response_bound_converged'] or r.get('scoring_interval_limit', 0) < limit], limit)
        refinement_log.append({'interval_limit': limit, 'leader_before': before,
                               'leader_after': min(scored.values(), key=key)['position']})
    # Small projected pattern searches remove fine-grid alignment error. Every
    # trial still passes the exact reception predicate and duplicate filter.
    # No physical source samples enter this derivative-free polishing stage.
    polish_seeds = sorted(scored.values(), key=key)[:config.polish_starts]
    directions = [np.array([math.cos(a), math.sin(a)]) for a in np.arange(8)*math.pi/4]
    for seed in polish_seeds:
        p = aspoint(seed)
        for step in (config.fine_spacing_m, config.fine_spacing_m/2, config.fine_spacing_m/4):
            for _ in range(2):
                if _expired(deadline): break
                trials = []
                for d in directions:
                    q = p+step*d
                    if not allowed(q):
                        lo, hi = 0., 1.
                        for _ in range(24):
                            mid = (lo+hi)/2
                            if allowed(p+mid*step*d): lo = mid
                            else: hi = mid
                        q = p+max(0., lo-1e-8)*step*d
                    if (np.linalg.norm(q-p) > 1e-5 and
                        all(np.linalg.norm(q-old) > config.repeated_position_tolerance_m for old in used)):
                        trials.append(q)
                score_points([q for q in _deduplicate(trials) if point_key(q) not in scored], config.final_response_intervals)
                neighborhood = [scored[point_key(q)] for q in trials if point_key(q) in scored]
                candidate = min([scored[point_key(p)]]+neighborhood, key=key)
                if candidate['objective_m'] >= scored[point_key(p)]['objective_m']-1e-6:
                    break
                p = aspoint(candidate)
    # A selected nonconverged score receives extra work, up to a disclosed cap.
    # Never claim convergence merely because the cap or time budget was reached.
    while not _expired(deadline):
        leader = min(scored.values(), key=key)
        previous = leader.get('scoring_interval_limit', 1)
        if leader['response_bound_converged'] or previous >= config.maximum_refinement_intervals:
            break
        score_points([aspoint(leader)], min(config.maximum_refinement_intervals, max(2*previous, config.final_response_intervals)))
        if scored[point_key(aspoint(leader))].get('scoring_interval_limit', 1) <= previous:
            break
    best = min(scored.values(), key=key)
    # C_good is an accuracy set even when lambda > 0; use U, not J.
    min_radius = min(r['worst_updated_cover_radius_m'] for r in scored.values())
    cloud = [aspoint(r) for r in scored.values()
             if r['worst_updated_cover_radius_m'] <= (1+config.near_best_relative_tolerance)*min_radius]
    return finish({'status': 'OK', 'selected': best,
                   'selection_mode': 'safe_fallback' if best is fallback_score else 'response_bound_search',
                   'near_best_candidate_cloud': [{'x': float(p[0]), 'y': float(p[1])} for p in cloud],
                   'near_best_radius_threshold_m': (1+config.near_best_relative_tolerance)*min_radius,
                   'candidate_scores': sorted(scored.values(), key=key),
                   'refinement_starts': [p.tolist() for p in starts],
                   'response_refinement_log': refinement_log,
                   'evaluated_candidate_count': ranked_count,
                   'unique_evaluated_candidate_count': len(scored), **base})
