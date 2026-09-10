"""Question 2 second detection point selection for an omnidirectional source.

The safe candidate region is continuous and guaranteed: every candidate is at
most the minimum possible reception radius from every point in a conservative
outer localization polygon. Ranking inside that region is a documented finite
scenario approximation, not a proof of the continuous minimax optimum.
"""

from dataclasses import asdict, dataclass
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
    movement_weight_m_per_s: float = 1.0
    circle_sides: int = 128
    coarse_spacing_m: float = 100.0
    fine_spacing_m: float = 20.0
    scenario_spacing_m: float = 100.0
    max_coarse_candidates: int = 24
    max_fine_candidates: int = 24
    max_source_scenarios: int = 32
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
        "scenario_spacing_m", "max_coarse_candidates", "max_fine_candidates",
        "max_source_scenarios", "near_best_epsilon_m",
    )
    for name in numeric_positive:
        if _finite_number(getattr(config, name), name) <= 0:
            raise ValueError(f"{name} must be positive")
    if config.circle_sides < 16 or int(config.circle_sides) != config.circle_sides:
        raise ValueError("circle_sides must be an integer >= 16")
    for name in ("max_coarse_candidates", "max_fine_candidates", "max_source_scenarios"):
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


def _deduplicate(points, tolerance_m=1e-7):
    unique = []
    for point in points:
        point = np.asarray(point, dtype=float)
        if not any(np.linalg.norm(point - old) <= tolerance_m for old in unique):
            unique.append(point)
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
    remaining = set(range(len(points))) - {first}
    while remaining and len(selected) < limit:
        index = max(remaining, key=lambda i: (min(np.linalg.norm(array[i] - array[j]) for j in selected), -i))
        selected.append(index)
        remaining.remove(index)
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
                      repeated_position_tolerance_m):
    radius = safe_region["guaranteed_radius_m"]
    witness = np.asarray(safe_region["witness_center"])
    raw = [witness, np.mean(vertices, axis=0), current]
    raw.extend(_grid(safe_region["bounding_box"], spacing, witness))
    filtered = [p for p in _deduplicate(raw)
                if is_safe_candidate(p, vertices, radius)
                and not any(np.linalg.norm(p - old) <= repeated_position_tolerance_m
                            for old in used_positions)]
    if len(filtered) <= limit:
        return filtered
    closest = sorted(filtered, key=lambda p: (np.linalg.norm(p - current), p[0], p[1]))[:max(1, limit // 3)]
    return _deduplicate(closest + _diverse_subset(filtered, limit - len(closest), witness))[:limit]


def _physical_source(point, station, target_center, config, A_wedge, b_wedge):
    return (np.linalg.norm(point - target_center) <= config.target_radius_m + 1e-8
            and config.near_radius_m < np.linalg.norm(point - station) <= config.maximum_reception_radius_m + 1e-8
            and np.all(A_wedge @ point <= b_wedge + 1e-8))


def _center_ray_scenarios(station, bearing_deg, target_center, config):
    angle = math.radians(bearing_deg)
    direction = np.array([math.cos(angle), math.sin(angle)])
    relative = station - target_center
    discriminant = float((relative @ direction) ** 2 - (relative @ relative - config.target_radius_m ** 2))
    if discriminant < 0:
        return []
    root = math.sqrt(max(0.0, discriminant))
    lo = max(config.near_radius_m + 1e-6, -float(relative @ direction) - root)
    hi = min(config.maximum_reception_radius_m, -float(relative @ direction) + root)
    if lo > hi:
        return []
    return [station + t * direction for t in np.linspace(lo, hi, 9)]


def _source_scenarios(vertices, station, bearing_deg, error_deg, config):
    target_center = np.array([config.target_center_x, config.target_center_y])
    observation = {"position": {"x": station[0], "y": station[1]}, "svd_deg": bearing_deg}
    A_wedge, b_wedge = bearing_halfplanes([observation], error_deg)
    raw = [np.mean(vertices, axis=0)]
    raw.extend(_center_ray_scenarios(station, bearing_deg, target_center, config))
    for p, q in zip(vertices, np.roll(vertices, -1, axis=0)):
        raw.extend(p + fraction * (q - p) for fraction in (0, 0.25, 0.5, 0.75))
    bounds = {"x_min": float(np.min(vertices[:, 0])), "x_max": float(np.max(vertices[:, 0])),
              "y_min": float(np.min(vertices[:, 1])), "y_max": float(np.max(vertices[:, 1]))}
    raw.extend(_grid(bounds, config.scenario_spacing_m, np.mean(vertices, axis=0)))
    feasible = [p for p in _deduplicate(raw)
                if _physical_source(p, station, target_center, config, A_wedge, b_wedge)]
    return _diverse_subset(feasible, config.max_source_scenarios)


def _score_candidate(candidate, source_scenarios, vertices, current, error_deg, config, deadline):
    worst_radius, worst_case, evaluations, near_scenarios = -math.inf, None, 0, 0
    for source_index, source in enumerate(source_scenarios):
        if deadline is not None and time.monotonic() >= deadline:
            return None
        distance = float(np.linalg.norm(source - candidate))
        if distance <= config.near_radius_m:
            radii = [(None, 0.0)]
            near_scenarios += 1
        else:
            true_bearing = math.degrees(math.atan2(source[1] - candidate[1], source[0] - candidate[0]))
            radii = []
            for error in (-error_deg, 0.0, error_deg):
                updated = _clip_with_bearing(vertices, candidate, true_bearing + error, error_deg)
                radius = math.inf if not len(updated) else minimum_enclosing_circle(updated)["radius_m"]
                radii.append((error, radius))
        for error, radius in radii:
            evaluations += 1
            if radius > worst_radius:
                worst_radius = radius
                worst_case = {"source_scenario_index": source_index,
                              "bearing_error_deg": error, "updated_cover_radius_m": radius}
    movement_distance = float(np.linalg.norm(candidate - current))
    movement_time = movement_distance / config.movement_speed_mps
    return {
        "position": {"x": float(candidate[0]), "y": float(candidate[1])},
        "worst_updated_cover_radius_m": worst_radius,
        "movement_distance_m": movement_distance,
        "movement_time_s": movement_time,
        "objective_m": worst_radius + config.movement_weight_m_per_s * movement_time,
        "worst_case": worst_case,
        "scenario_evaluations": evaluations,
        "near_source_scenarios": near_scenarios,
    }


def choose_second_detection(first_observation, error_deg=1.0, current_position=None,
                            used_positions=None, config=None):
    """Return a safe second point and approximate minimax ranking diagnostics.

    The returned point has a continuous reception guarantee for an
    omnidirectional source under the configured 1000m minimum radius. The
    objective is evaluated on finite source/error scenarios and is approximate.
    """
    config = config or Q2Config()
    _validate_config(config)
    error_deg = _finite_number(error_deg, "error_deg")
    if not 0 < error_deg < 90:
        raise ValueError("error_deg must be strictly between 0 and 90 degrees")
    initial, _, _ = build_initial_outer_region(first_observation, error_deg, config)
    if initial["status"] != "BOUNDED":
        return {"status": "GEOMETRY_ERROR", "reason": initial["status"],
                "initial_region": initial, "selected": None}
    vertices = np.asarray(initial["vertices"])
    station = _point(first_observation["position"], "first_observation.position")
    bearing_deg = _finite_number(first_observation["svd_deg"], "first_observation.svd_deg")
    current = station if current_position is None else _point(current_position, "current_position")
    used = [station]
    if used_positions is not None:
        if not isinstance(used_positions, (list, tuple)):
            raise ValueError("used_positions must be an array or None")
        used.extend(_point(point, f"used_positions[{i}]") for i, point in enumerate(used_positions))
    guaranteed_radius = config.minimum_reception_radius_m - config.safety_margin_m
    safe = safe_candidate_region(vertices, guaranteed_radius)
    base = {"initial_region": initial, "safe_candidate_region": safe,
            "config": asdict(config), "error_deg": error_deg,
            "ranking_is_discrete_approximation": True,
            "reception_guarantee_scope": "omnidirectional_source_only"}
    if not safe["nonempty"]:
        return {"status": "NO_CANDIDATE", "reason": "SAFE_REGION_EMPTY", "selected": None, **base}
    scenarios = _source_scenarios(vertices, station, bearing_deg, error_deg, config)
    if not scenarios:
        return {"status": "NO_CANDIDATE", "reason": "NO_PHYSICAL_SOURCE_SCENARIOS", "selected": None, **base}
    deadline = (time.monotonic() + config.calculation_time_limit_s
                if config.calculation_time_limit_s is not None else None)
    coarse = _candidate_points(vertices, safe, current, config.coarse_spacing_m,
                               config.max_coarse_candidates, used,
                               config.repeated_position_tolerance_m)
    scored = []
    timed_out = False
    for candidate in coarse:
        score = _score_candidate(candidate, scenarios, vertices, current, error_deg, config, deadline)
        if score is None:
            timed_out = True
            break
        scored.append(score)
    if not scored:
        return {"status": "NO_CANDIDATE", "reason": "NO_EVALUATED_CANDIDATE", "selected": None,
                "timed_out": timed_out, **base}
    coarse_best = min(scored, key=lambda row: (row["objective_m"], row["movement_distance_m"],
                                                row["position"]["x"], row["position"]["y"]))
    best_point = np.array([coarse_best["position"]["x"], coarse_best["position"]["y"]])
    local_bounds = {"x_min": best_point[0] - config.coarse_spacing_m,
                    "x_max": best_point[0] + config.coarse_spacing_m,
                    "y_min": best_point[1] - config.coarse_spacing_m,
                    "y_max": best_point[1] + config.coarse_spacing_m}
    fine_raw = _grid(local_bounds, config.fine_spacing_m, best_point)
    fine = [p for p in _diverse_subset(
        [p for p in fine_raw if is_safe_candidate(p, vertices, guaranteed_radius)
         and not any(np.linalg.norm(p - old) <= config.repeated_position_tolerance_m for old in used)],
        config.max_fine_candidates, best_point)
        if not any(np.linalg.norm(p - np.array([row["position"]["x"], row["position"]["y"]]))
                   <= config.repeated_position_tolerance_m for row in scored)]
    if not timed_out:
        for candidate in fine:
            score = _score_candidate(candidate, scenarios, vertices, current, error_deg, config, deadline)
            if score is None:
                timed_out = True
                break
            scored.append(score)
    best = min(scored, key=lambda row: (row["objective_m"], row["movement_distance_m"],
                                         row["position"]["x"], row["position"]["y"]))
    threshold = best["objective_m"] + config.near_best_epsilon_m
    cloud = [row["position"] for row in scored if row["objective_m"] <= threshold]
    return {"status": "OK", "selected": best, "near_best_candidate_cloud": cloud,
            "source_scenario_count": len(scenarios), "evaluated_candidate_count": len(scored),
            "timed_out": timed_out, **base}
