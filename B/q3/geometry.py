"""Guaranteed station and clearing geometries for Question 3."""

import math


Point = tuple[float, float]


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return value


def _point(value, name="point") -> Point:
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f"{name} must contain exactly two coordinates")
    return _finite(value[0], f"{name}[0]"), _finite(value[1], f"{name}[1]")


def distance(a, b) -> float:
    ax, ay = _point(a, "a")
    bx, by = _point(b, "b")
    return math.hypot(ax - bx, ay - by)


def seven_station_route(ring_radius_m=1150.0) -> tuple[Point, ...]:
    """Return origin followed by the six hexagonal ring stations."""
    radius = _finite(ring_radius_m, "ring_radius_m")
    if radius <= 0:
        raise ValueError("ring_radius_m must be positive")
    ring = tuple(
        (radius * math.cos(k * math.pi / 3), radius * math.sin(k * math.pi / 3))
        for k in range(6)
    )
    return ((0.0, 0.0),) + ring


def route_length(points) -> float:
    points = tuple(_point(point, f"points[{i}]") for i, point in enumerate(points))
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def minimum_station_distance(point, stations=None) -> float:
    stations = seven_station_route() if stations is None else tuple(stations)
    if not stations:
        raise ValueError("stations must be nonempty")
    return min(distance(point, station) for station in stations)


def q3_coverage_certificate(target_radius_m=1800.0, guaranteed_receive_radius_m=1000.0,
                            ring_radius_m=1150.0):
    """Return the analytic endpoint certificate for the seven-station cover."""
    target = _finite(target_radius_m, "target_radius_m")
    receive = _finite(guaranteed_receive_radius_m, "guaranteed_receive_radius_m")
    ring = _finite(ring_radius_m, "ring_radius_m")
    if target <= 0 or receive <= 0 or ring <= 0:
        raise ValueError("radii must be positive")
    endpoint_radii = (receive, target)
    endpoint_distances = tuple(math.sqrt(
        radius ** 2 + ring ** 2 - 2 * ring * radius * math.cos(math.pi / 6)
    ) for radius in endpoint_radii)
    worst = max(endpoint_distances)
    return {
        "station_count": 7,
        "ring_radius_m": ring,
        "center_covered_radius_m": receive,
        "outer_annulus_radial_interval_m": endpoint_radii,
        "endpoint_worst_distances_m": endpoint_distances,
        "certified_worst_distance_m": worst,
        "guaranteed_receive_radius_m": receive,
        "covered": worst <= receive,
        "route_length_m": route_length(seven_station_route(ring)),
    }


def strip_clear_points(anchor_position, bearing_deg, max_range_m=1500.0,
                       step_m=20.0, offsets_m=(-15.0, 15.0)) -> tuple[Point, ...]:
    """Return the two-row snake that covers a first-bearing wedge within 20 m."""
    anchor_x, anchor_y = _point(anchor_position, "anchor_position")
    bearing = math.radians(_finite(bearing_deg, "bearing_deg") % 360.0)
    max_range = _finite(max_range_m, "max_range_m")
    step = _finite(step_m, "step_m")
    offsets = tuple(_finite(value, f"offsets_m[{i}]") for i, value in enumerate(offsets_m))
    if max_range <= 0 or step <= 0:
        raise ValueError("max_range_m and step_m must be positive")
    if len(offsets) != 2 or not offsets[0] < offsets[1]:
        raise ValueError("offsets_m must contain two increasing offsets")
    intervals = max_range / step
    if not math.isclose(intervals, round(intervals), abs_tol=1e-12):
        raise ValueError("max_range_m must be an integer multiple of step_m")
    u = (math.cos(bearing), math.sin(bearing))
    n = (-u[1], u[0])

    def point(k, offset):
        return (anchor_x + step * k * u[0] + offset * n[0],
                anchor_y + step * k * u[1] + offset * n[1])

    final_k = int(round(intervals))
    first_row = tuple(point(k, offsets[0]) for k in range(final_k + 1))
    second_row = tuple(point(k, offsets[1]) for k in range(final_k, -1, -1))
    return first_row + second_row


def strip_coverage_bound_m(angle_half_width_deg=1.005, max_range_m=1500.0,
                           step_m=20.0, offsets_m=(-15.0, 15.0)) -> float:
    """Compute the farthest distance to the nearest ideal strip point."""
    half_width = math.radians(_finite(angle_half_width_deg, "angle_half_width_deg"))
    max_range = _finite(max_range_m, "max_range_m")
    step = _finite(step_m, "step_m")
    offsets = sorted(_finite(value, f"offsets_m[{i}]") for i, value in enumerate(offsets_m))
    if not 0 < half_width < math.pi / 2 or max_range <= 0 or step <= 0 or not offsets:
        raise ValueError("invalid strip geometry")
    lateral_limit = max_range * math.sin(half_width)
    candidates = [-lateral_limit, lateral_limit]
    candidates.extend((a + b) / 2 for a, b in zip(offsets, offsets[1:]))
    lateral_gap = max(min(abs(z - offset) for offset in offsets) for z in candidates)
    return math.hypot(step / 2, lateral_gap)
