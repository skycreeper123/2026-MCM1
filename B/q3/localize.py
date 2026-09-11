"""Cumulative localization and certified local clear plans for Q3 V2."""

from dataclasses import asdict, dataclass
import math
import time

import numpy as np

from B.q1.geometry import bearing_halfplanes, minimum_enclosing_circle
from B.q2 import (
    Q2Config,
    circle_outer_halfplanes,
    clip_convex_polygon,
    is_safe_candidate,
    response_radius_bound,
    safe_candidate_region,
)
from B.q2.selection import build_initial_outer_region

from .geometry import distance


@dataclass(frozen=True)
class ClearPlan:
    kind: str
    points: tuple[tuple[float, float], ...]
    guaranteed: bool
    cover_radius_m: float
    route_distance_m: float
    completion_upper_s: float
    cell_size_m: tuple[float, float] | None = None
    orientation_deg: float | None = None

    def as_dict(self):
        return asdict(self)


def _vertices(value):
    vertices = np.asarray(value, dtype=float)
    if vertices.ndim != 2 or vertices.shape[1] != 2 or not len(vertices) \
            or not np.isfinite(vertices).all():
        raise ValueError("vertices must be a nonempty finite (n, 2) array")
    return vertices


def _observation(position, bearing_deg):
    position = tuple(float(value) for value in position)
    if len(position) != 2 or not all(math.isfinite(value) for value in position):
        raise ValueError("position must be a finite 2-D point")
    if isinstance(bearing_deg, bool) or not isinstance(bearing_deg, (int, float)) \
            or not math.isfinite(bearing_deg):
        raise ValueError("bearing_deg must be finite")
    return {"position": {"x": position[0], "y": position[1]},
            "svd_deg": float(bearing_deg) % 360.0}


def initialize_outer_region(observation, error_deg=1.005, q2_config=None):
    """Build Q3's first conservative outer polygon with Q2's public model."""
    config = q2_config or Q2Config()
    result, _, _ = build_initial_outer_region(observation, error_deg, config)
    return result


def update_outer_region(previous_vertices, direction_observation, error_deg=1.005,
                        maximum_receive_radius_m=1500.0, circle_sides=128,
                        geometry_tolerance_m=1e-8):
    """Incrementally add one direction wedge and its receive-circle outer bound."""
    vertices = _vertices(previous_vertices)
    if not isinstance(direction_observation, dict) or set(direction_observation) != {"position", "svd_deg"}:
        raise ValueError("direction_observation requires position and svd_deg")
    position_dict = direction_observation["position"]
    if not isinstance(position_dict, dict) or set(position_dict) != {"x", "y"}:
        raise ValueError("direction_observation.position requires x and y")
    position = np.array([float(position_dict["x"]), float(position_dict["y"])])
    if not np.isfinite(position).all():
        raise ValueError("direction_observation.position must be finite")
    bearing = direction_observation["svd_deg"]
    if isinstance(bearing, bool) or not isinstance(bearing, (int, float)) or not math.isfinite(bearing):
        raise ValueError("direction_observation.svd_deg must be finite")
    A_wedge, b_wedge = bearing_halfplanes([direction_observation], error_deg)
    A_receive, b_receive = circle_outer_halfplanes(
        position, maximum_receive_radius_m, circle_sides
    )
    updated = clip_convex_polygon(
        vertices, np.vstack((A_wedge, A_receive)),
        np.concatenate((b_wedge, b_receive)), geometry_tolerance_m
    )
    if not len(updated):
        return {"status": "EMPTY", "vertices": None, "minimum_enclosing_circle": None}
    mec = minimum_enclosing_circle(updated)
    center = np.asarray(mec["center"], dtype=float)
    verified_radius = float(np.linalg.norm(updated - center, axis=1).max())
    return {
        "status": "BOUNDED",
        "vertices": updated,
        "minimum_enclosing_circle": {
            "center": center,
            "radius_m": verified_radius,
            "method": mec.get("method"),
        },
    }


def verified_enclosing_circle(vertices):
    vertices = _vertices(vertices)
    result = minimum_enclosing_circle(vertices)
    center = np.asarray(result["center"], dtype=float)
    radius = float(np.linalg.norm(vertices - center, axis=1).max())
    return center, radius


def plan_route_distance(points, current_position, continuation_target=None):
    points = tuple(tuple(map(float, point)) for point in points)
    current = tuple(map(float, current_position))
    total = 0.0
    for point in points:
        total += distance(current, point)
        current = point
    if continuation_target is not None:
        total += distance(current, continuation_target)
    return total


def _plan_seconds(points, current_position, continuation_target, speed_mps,
                  failed_clear_duration_s, successful_clear_duration_s):
    route = plan_route_distance(points, current_position, continuation_target)
    count = len(points)
    action = ((count - 1) * failed_clear_duration_s + successful_clear_duration_s)
    return route, route / speed_mps + action


def _rectangle_plan(vertices, orientation_deg, current_position, continuation_target,
                    cell_limit_m, clear_radius_m, numerical_margin_m,
                    speed_mps, failed_clear_duration_s, successful_clear_duration_s):
    angle = math.radians(orientation_deg)
    u = np.array([math.cos(angle), math.sin(angle)])
    n = np.array([-u[1], u[0]])
    projected_u = vertices @ u
    projected_n = vertices @ n
    raw_length = float(projected_u.max() - projected_u.min())
    raw_width = float(projected_n.max() - projected_n.min())
    count_u = max(1, math.ceil(raw_length / cell_limit_m))
    count_n = max(1, math.ceil(raw_width / cell_limit_m))
    u0, u1 = float(projected_u.min() - numerical_margin_m), float(projected_u.max() + numerical_margin_m)
    n0, n1 = float(projected_n.min() - numerical_margin_m), float(projected_n.max() + numerical_margin_m)
    length, width = u1 - u0, n1 - n0
    size_u, size_n = length / count_u, width / count_n
    cover_radius = math.hypot(size_u / 2, size_n / 2)
    if cover_radius > clear_radius_m:
        return None
    us = [u0 + (index + 0.5) * size_u for index in range(count_u)]
    ns = [n0 + (index + 0.5) * size_n for index in range(count_n)]
    points = []
    for row, n_value in enumerate(ns):
        values = us if row % 2 == 0 else reversed(us)
        points.extend(tuple(u_value * u + n_value * n) for u_value in values)
    variants = (tuple(points), tuple(reversed(points)))
    candidates = []
    for variant in variants:
        route, seconds = _plan_seconds(
            variant, current_position, continuation_target, speed_mps,
            failed_clear_duration_s, successful_clear_duration_s
        )
        candidates.append(ClearPlan(
            "RECTANGLE_GRID", variant, True, cover_radius, route, seconds,
            (size_u, size_n), float(orientation_deg) % 180.0,
        ))
    return min(candidates, key=lambda plan: (plan.completion_upper_s, plan.route_distance_m))


def build_cover_plan(vertices, current_position, continuation_target=None, max_points=8,
                     preferred_orientations_deg=(), cell_limit_m=28.0,
                     clear_radius_m=19.9, numerical_margin_m=1e-7,
                     speed_mps=5.0, failed_clear_duration_s=3.0,
                     successful_clear_duration_s=5.0):
    """Return the cheapest certified MEC/grid termination plan within ``max_points``."""
    vertices = _vertices(vertices)
    plans = []
    center, radius = verified_enclosing_circle(vertices)
    if radius + numerical_margin_m <= clear_radius_m and max_points >= 1:
        points = ((float(center[0]), float(center[1])),)
        route, seconds = _plan_seconds(
            points, current_position, continuation_target, speed_mps,
            failed_clear_duration_s, successful_clear_duration_s
        )
        plans.append(ClearPlan(
            "MEC_SINGLE", points, True, radius + numerical_margin_m,
            route, seconds,
        ))
    orientations = [0.0]
    orientations.extend(float(value) for value in preferred_orientations_deg)
    for first, second in zip(vertices, np.roll(vertices, -1, axis=0)):
        edge = second - first
        if np.linalg.norm(edge) > numerical_margin_m:
            orientations.append(math.degrees(math.atan2(edge[1], edge[0])))
    unique = []
    for angle in orientations:
        normalized = angle % 180.0
        if all(abs(((normalized - old + 90) % 180) - 90) > 1e-7 for old in unique):
            unique.append(normalized)
    for angle in unique:
        plan = _rectangle_plan(
            vertices, angle, current_position, continuation_target, cell_limit_m,
            clear_radius_m, numerical_margin_m, speed_mps,
            failed_clear_duration_s, successful_clear_duration_s,
        )
        if plan is not None and len(plan.points) <= max_points:
            plans.append(plan)
    if not plans:
        return None
    return min(plans, key=lambda plan: (
        plan.completion_upper_s, len(plan.points), plan.route_distance_m, plan.kind
    ))


def _unused(point, used_positions, tolerance_m):
    return all(distance(point, old) > tolerance_m for old in used_positions)


def choose_detection_from_region(vertices, current_position, used_positions,
                                 anchor_bearing_deg, error_deg=1.005,
                                 continuation_target=None, q2_config=None,
                                 deadline_monotonic=None, candidate_limit=12):
    """Choose a safe new station from a cumulative outer polygon.

    Ranking uses Q2 V3's continuous direction-response radius bound.  This is
    the V2-local adapter; it does not claim the completion-time bound described
    for the later ``V2_time_score`` stage.
    """
    started = time.monotonic()
    vertices = _vertices(vertices)
    current = np.asarray(current_position, dtype=float)
    used = tuple(tuple(map(float, point)) for point in used_positions)
    config = q2_config or Q2Config(
        movement_weight_m_per_s=1.0,
        max_response_intervals=32,
        response_bound_tolerance_m=0.5,
    )
    safe_radius = config.minimum_reception_radius_m - config.safety_margin_m
    safe = safe_candidate_region(vertices, safe_radius)
    if not safe["nonempty"]:
        return {"status": "NO_CANDIDATE", "reason": "SAFE_REGION_EMPTY"}
    witness = np.asarray(safe["witness_center"], dtype=float)
    candidates = [witness]
    if is_safe_candidate(current, vertices, safe_radius):
        candidates.append(current)
    if continuation_target is not None:
        continuation = np.asarray(continuation_target, dtype=float)
        if is_safe_candidate(continuation, vertices, safe_radius):
            candidates.append(continuation)
    for angle_deg in anchor_bearing_deg + np.arange(0.0, 360.0, 30.0):
        direction = np.array([math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))])
        lo, hi = 0.0, 2.0 * safe_radius
        for _ in range(36):
            mid = (lo + hi) / 2
            if is_safe_candidate(witness + mid * direction, vertices, safe_radius):
                lo = mid
            else:
                hi = mid
        candidates.append(witness + max(0.0, lo - 1e-6) * direction)
    unique = []
    for candidate in candidates:
        point = (float(candidate[0]), float(candidate[1]))
        if _unused(point, used, config.repeated_position_tolerance_m) \
                and is_safe_candidate(candidate, vertices, safe_radius) \
                and all(distance(point, old) > 1e-7 for old in unique):
            unique.append(point)
    if not unique:
        return {"status": "NO_CANDIDATE", "reason": "NO_UNUSED_SAFE_WITNESS"}
    # Retain spatially varied points deterministically when callers lower the cap.
    unique = unique[:max(1, int(candidate_limit))]
    initial_radius = verified_enclosing_circle(vertices)[1]
    scores = []
    for point in unique:
        if deadline_monotonic is not None and time.monotonic() >= deadline_monotonic:
            break
        bound = response_radius_bound(
            vertices, point, error_deg, config, deadline_monotonic, initial_radius
        )
        move_distance = distance(current_position, point)
        move_time = move_distance / config.movement_speed_mps
        scores.append({
            "position": {"x": point[0], "y": point[1]},
            "movement_distance_m": move_distance,
            "movement_time_s": move_time,
            "objective_m": bound["worst_updated_cover_radius_m"]
                           + config.movement_weight_m_per_s * move_time,
            **bound,
        })
    if not scores:
        return {"status": "NO_CANDIDATE", "reason": "PLANNING_BUDGET_EXHAUSTED"}
    selected = min(scores, key=lambda row: (
        row["objective_m"], row["movement_distance_m"],
        row["position"]["x"], row["position"]["y"],
    ))
    return {
        "status": "OK",
        "selected": selected,
        "candidate_scores": scores,
        "safe_candidate_region": safe,
        "selection_mode": "v2_local_cumulative_response_bound",
        "elapsed_s": time.monotonic() - started,
        "timed_out": deadline_monotonic is not None and time.monotonic() >= deadline_monotonic,
        "ranking_is_discrete_approximation": True,
        "response_bound_is_continuous": True,
    }
