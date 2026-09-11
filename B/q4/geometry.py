"""Q4 geometry. Reception certificates never use an outward hull tolerance."""

from dataclasses import dataclass, field, replace
from fractions import Fraction
from pathlib import Path
import hashlib
import json
import math

import numpy as np

from B.q3.geometry import strip_clear_points, strip_coverage_bound_m
from B.q3.localize import (ClearPlan, build_cover_plan, initialize_outer_region,
                          update_outer_region, verify_cover_certificate,
                          plan_route_distance)
from .verify_q4_framework import check_certificate, check_exact_geometry


ROUTE = (0, 6, 7, 8, 1, 2, 3, 4, 5, 15, 16, 17, 18, 19, 20, 9, 10, 11, 12, 13, 14)


@dataclass(frozen=True)
class StationCover:
    stations: tuple
    route: tuple
    network_id: str
    certificate: dict


def _fallback_cover(reason):
    # Actual binary64 lattice is an affine image of the ideal equilateral
    # lattice. Bound its y-scale exactly, without relying on sampled sources.
    # Dyadic height makes every small-integer multiple exactly representable.
    height = round((990 * math.sqrt(3) / 2) * 2**30) / 2**30
    ratio_squared = Fraction(height)**2 / Fraction(3 * 990**2, 4)
    if not Fraction(999999999, 1000000000)**2 < ratio_squared < Fraction(1000000001, 1000000000)**2:
        raise ValueError("Cannot certify fallback lattice scale")
    indexed = [(i, j) for i in range(-4, 5) for j in range(-4, 5)
               if i*i + i*j + j*j <= 7]
    indexed.sort(key=lambda ij: (ij != (0, 0), ij))
    v = tuple((990 * (i+j/2), height*j) for i, j in indexed)
    if len(v) != 31:
        raise ValueError("Invalid fallback lattice")
    # In inverse coordinates |g| <= 1800/(1-1e-9). Its containing
    # equilateral triangle has vertices <= |g|+990 < 2790.000002.
    # The next excluded lattice shell is sqrt(9)*990 = 2970.
    # Thus all 3 vertices are retained; actual distance <= 990*(1+1e-9).
    # Multiplication by integer j (|j|<=3) has no rounding for this height;
    # nevertheless verify the exact affine representation of sent coordinates.
    for (i, j), (x, y) in zip(indexed, v):
        if Fraction(y) != Fraction(height)*j or Fraction(x) != Fraction(990)*(i+Fraction(j, 2)):
            raise ValueError("Fallback sent coordinates are not exact lattice points")
    remaining, route = set(range(1, 31)), [0]
    while remaining:
        k = min(remaining, key=lambda k: (math.dist(v[route[-1]], v[k]), k))
        route.append(k)
        remaining.remove(k)
    return StationCover(v, tuple(route), "q4-lattice31-" + hashlib.sha256(repr(v).encode()).hexdigest(),
                        {"passed": True, "fallback": True, "reason": reason,
                         "method": "exact affine triangular lattice; next excluded shell 2970m",
                         "max_provider_distance_m": 990.000001})


def load_and_verify_station_cover(path=None, allow_fallback=True):
    if not __debug__:
        raise RuntimeError("Certificate verifier requires Python without -O")
    path = Path(path) if path else Path(__file__).with_name("Q4_COVERAGE_CERTIFICATE.json")
    try:
        cert = json.loads(path.read_text(encoding="utf-8"))
        if cert["target_radius_m"] != 1800 or cert["distance_limit_m"] != 999:
            raise ValueError("Unexpected physical model in station certificate")
        floating, exact = check_certificate(cert), check_exact_geometry(cert)
        v = tuple(tuple(map(float, p)) for p in cert["stations_m"])
        return StationCover(v, ROUTE, hashlib.sha256(repr(v).encode()).hexdigest(),
                            {"passed": True, "floating": floating, "exact": exact, "fallback": False})
    except (OSError, ValueError, KeyError, TypeError, AssertionError, IndexError) as exc:
        if not allow_fallback:
            raise ValueError("Station coverage certificate failed") from exc
        return _fallback_cover(f"{type(exc).__name__}: {exc}")


def rational_point(p):
    return tuple(x if isinstance(x, Fraction) else Fraction(float(x)) for x in p)


def cross(a, b, p):
    return (b[0]-a[0])*(p[1]-a[1]) - (b[1]-a[1])*(p[0]-a[0])


def exact_hull(points):
    points = sorted(set(map(rational_point, points)))
    if len(points) <= 1:
        return tuple(points)
    def half(seq):
        result = []
        for p in seq:
            while len(result) >= 2 and cross(result[-2], result[-1], p) <= 0:
                result.pop()
            result.append(p)
        return result
    return tuple(half(points)[:-1] + half(reversed(points))[:-1])


def hull_contains(hull, point):
    p = rational_point(point)
    if not hull:
        return False
    if len(hull) == 1:
        return p == hull[0]
    if len(hull) == 2:
        a, b = hull
        return cross(a, b, p) == 0 and all(min(a[k], b[k]) <= p[k] <= max(a[k], b[k]) for k in (0, 1))
    return all(cross(a, b, p) >= 0 for a, b in zip(hull, hull[1:]+hull[:1]))


@dataclass
class SourceRecord:
    channel: int
    state: str = "UNKNOWN"
    source_type: str = "UNDETERMINED"
    anchor: dict | None = None
    observations: list = field(default_factory=list)
    vertices: object = None
    positive_points: list = field(default_factory=list)
    hull: tuple = ()
    region_version: int = 0
    hull_version: int = 0
    used_positions: set = field(default_factory=set)
    local_measurements: int = 0
    local_travel_m: float = 0.0
    last_local_position: tuple | None = None
    clear_certificate: object = None
    pending_pair: object = None


def update_positive_hull(record, observation):
    if observation["result"] in {"direction", "near"}:
        p = tuple(observation["position"])
        if p not in record.positive_points:
            record.positive_points.append(p)
            record.hull = exact_hull(record.positive_points)
            record.hull_version += 1
    return record.hull


def update_source_region(record, observation):
    """Negative observations do not cut a directional source's outer polygon."""
    if observation["result"] != "direction":
        return record.vertices
    x, y = observation["position"]
    obs = {"position": {"x": x, "y": y}, "svd_deg": observation["svd_deg"]}
    if record.anchor is None:
        record.anchor = obs
    result = (initialize_outer_region(obs, error_deg=1.005) if record.vertices is None
              else update_outer_region(record.vertices, obs, error_deg=1.005))
    if result["status"] != "BOUNDED":
        raise ValueError("Inconsistent direction observations: empty feasible region")
    record.vertices = np.asarray(result["vertices"], dtype=float)
    record.region_version += 1
    return record.vertices


def min_polygon_distance_squared(vertices, point):
    """Exact min distance to the WHOLE convex polygon (including its interior)."""
    hull, p = exact_hull(vertices), rational_point(point)
    if hull_contains(hull, p):
        return Fraction(0)
    values = []
    for a, b in zip(hull, hull[1:]+hull[:1]):
        delta, rel = tuple(b[k]-a[k] for k in (0, 1)), tuple(p[k]-a[k] for k in (0, 1))
        length = sum(x*x for x in delta)
        t = max(Fraction(0), min(Fraction(1), sum(x*y for x, y in zip(delta, rel))/length)) if length else 0
        values.append(sum((rel[k]-t*delta[k])**2 for k in (0, 1)))
    return min(values)


def distance_safe(vertices, p, radius=999.9):
    p, limit = rational_point(p), Fraction(float(radius))**2
    return all(sum((x-y)**2 for x, y in zip(rational_point(v), p)) <= limit for v in vertices)


def classify_q4_reception(record, point):
    if hull_contains(record.hull, point):
        return "POSITIVE_HULL"
    if record.vertices is not None:
        if distance_safe(record.vertices, point):
            return "DISTANCE_ONLY"
        if min_polygon_distance_squared(record.vertices, point) > 1500**2:
            return "GUARANTEED_NO_SIGNAL"
    return "UNCERTAIN"


@dataclass(frozen=True)
class PairTask:
    points: tuple
    region_version: int
    hull_version: int
    certificate: str = "PAIR_AT_LEAST_ONE"


def build_pair_probe(record, midpoint, offset, current=None):
    if record.vertices is None:
        return None
    points = tuple(tuple(float(midpoint[k]) + sign*float(offset[k]) for k in (0, 1)) for sign in (1, -1))
    if points[0] == points[1] or any(p in record.used_positions for p in points):
        return None
    actual_mid = tuple((Fraction(points[0][k])+Fraction(points[1][k]))/2 for k in (0, 1))
    if not hull_contains(record.hull, actual_mid) or not all(distance_safe(record.vertices, p) for p in points):
        return None
    if current is not None and math.dist(current, points[1]) < math.dist(current, points[0]):
        points = points[::-1]
    return PairTask(points, record.region_version, record.hull_version)


def hull_candidates(record):
    h = record.hull
    candidates = []
    if len(h) >= 2:
        for a, b in zip(h, h[1:]+h[:1]):
            for t in (Fraction(1, 4), Fraction(1, 2), Fraction(3, 4)):
                candidates.append(tuple(float((1-t)*a[k]+t*b[k]) for k in (0, 1)))
    if len(h) >= 3:
        center = tuple(sum(p[k] for p in h)/len(h) for k in (0, 1))
        candidates.append(tuple(map(float, center)))
        candidates.extend(tuple(float((a[k]+center[k])/2) for k in (0, 1)) for a in h)
        lower = [min(p[k] for p in h) for k in (0, 1)]
        upper = [max(p[k] for p in h) for k in (0, 1)]
        candidates.extend(tuple(float(lower[k]+(upper[k]-lower[k])*ts[k]) for k in (0, 1))
                          for ts in ((a, b) for a in (Fraction(1, 4), Fraction(1, 2), Fraction(3, 4))
                                     for b in (Fraction(1, 4), Fraction(1, 2), Fraction(3, 4))))
    return tuple(dict.fromkeys(p for p in candidates if p not in record.used_positions and hull_contains(h, p)))


def reprice(plan, current, continuation=None):
    choices = (plan.points, plan.points[::-1])
    points = min(choices, key=lambda ps: plan_route_distance(ps, current, continuation))
    length = plan_route_distance(points, current, continuation)
    return replace(plan, points=points, route_distance_m=length,
                   completion_upper_s=length/5 + 3*(len(points)-1)+5)


def build_clear_plan(record, current, continuation=None):
    if record.anchor is None:
        raise ValueError("A direction anchor is required for non-near clearing")
    plan = None
    try:
        if record.vertices is not None:
            plan = build_cover_plan(record.vertices, current, continuation, max_points=152,
                                    preferred_orientations_deg=(record.anchor["svd_deg"],))
            if plan is not None and not verify_cover_certificate(record.vertices, plan):
                plan = None
    except (ValueError, ArithmeticError, np.linalg.LinAlgError):
        plan = None
    if plan is None:
        a = record.anchor
        points = strip_clear_points((a["position"]["x"], a["position"]["y"]), a["svd_deg"])
        radius = strip_coverage_bound_m()
        if len(points) != 152 or radius >= 19.9:
            raise ValueError("Invalid original strip certificate")
        plan = ClearPlan("STRIP", points, True, radius, 0, 0,
                         cover_certificate={"method": "FIRST_BEARING_STRIP", "anchor": a,
                                            "half_width_deg": 1.005, "radius_m": radius})
    return reprice(plan, current, continuation)
