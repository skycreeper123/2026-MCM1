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
CLEAR_ROUTE_RISK_CAP_RATIO = 1.25


@dataclass(frozen=True)
class StationCover:
    stations: tuple
    route: tuple
    network_id: str
    certificate: dict
    provider_sets: tuple = ()


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
                         "max_provider_distance_m": 990.000001}, ())


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
        providers = []
        def collect(node):
            if "providers" in node:
                providers.append(frozenset(map(int, node["providers"])))
            else:
                for child in node["children"]:
                    collect(child)
        for tree in cert["trees"]:
            collect(tree)
        return StationCover(v, ROUTE, hashlib.sha256(repr(v).encode()).hexdigest(),
                            {"passed": True, "floating": floating, "exact": exact, "fallback": False},
                            tuple(providers))
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
    remaining_version: int = 0
    hull_version: int = 0
    used_positions: set = field(default_factory=set)
    local_measurements: int = 0
    local_travel_m: float = 0.0  # positive marginal detour actually settled
    reserved_detour_m: float = 0.0
    consecutive_uncertain_no_signal: int = 0
    last_local_position: tuple | None = None
    clear_certificate: object = None
    pending_pair: object = None
    failed_clear_disks: list = field(default_factory=list)
    belief: object = None
    dynamic_candidates: tuple = ()
    dynamic_cache_origin: tuple | None = None
    dynamic_cache_target: tuple | None = None
    dynamic_cache_version: int = -1
    absence_certificate: dict | None = None
    base_clear_plan: object = None
    base_clear_version: int = -1
    sparse_removed_cells: dict = field(default_factory=dict)
    sparse_processed_disks: int = 0
    clear_requests: int = 0
    discovered_virtual_s: float | None = None
    cleared_virtual_s: float | None = None


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
    def length(points):
        path = (tuple(current),)+tuple(points)+(() if continuation is None else (tuple(continuation),))
        return sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    points = min(choices, key=length)
    route_length = length(points)
    return replace(plan, points=points, route_distance_m=route_length,
                   completion_upper_s=route_length/5 + 3*(len(points)-1)+5)


def remaining_contains(record, point):
    """Membership in P_plus minus all certified failed-clear disks."""
    return (record.vertices is not None and hull_contains(exact_hull(record.vertices), point)
            and all(sum((a-b)**2 for a, b in zip(rational_point(point), rational_point(center))) > 20**2
                    for center in record.failed_clear_disks))


def update_cover_after_failed_clear(record, point):
    point = tuple(map(float, point))
    if point not in record.failed_clear_disks:
        record.failed_clear_disks.append(point)
        record.remaining_version += 1
    return record.remaining_version


def _grid_cell_corners(center, certificate):
    angle = math.radians(certificate["orientation_deg"])
    u, n = (math.cos(angle), math.sin(angle)), (-math.sin(angle), math.cos(angle))
    u0, u1, n0, n1 = certificate["bounds_un"]
    nu, nn = certificate["counts_un"]
    du, dn = (u1-u0)/nu, (n1-n0)/nn
    return tuple((center[0]+su*du*u[0]/2+sn*dn*n[0]/2,
                  center[1]+su*du*u[1]/2+sn*dn*n[1]/2)
                 for su in (-1, 1) for sn in (-1, 1))


def _inside_failed_disk(corners, center):
    if any(math.dist(p, center) > 20 for p in corners):
        return False
    q, limit = rational_point(center), Fraction(20)**2
    return all(sum((a-b)**2 for a, b in zip(rational_point(p), q)) <= limit for p in corners)


def _sparsify_grid(record, plan):
    if not record.failed_clear_disks or plan.kind != "RECTANGLE_GRID":
        return plan
    parent = dict(plan.cover_certificate)
    # Same P_plus uses the same certified base grid. Process each new failed
    # disk once instead of rechecking every old disk against every cell.
    for disk in record.failed_clear_disks[record.sparse_processed_disks:]:
        for point in plan.points:
            if point not in record.sparse_removed_cells and _inside_failed_disk(_grid_cell_corners(point, parent), disk):
                record.sparse_removed_cells[point] = disk
    record.sparse_processed_disks = len(record.failed_clear_disks)
    retained = [point for point in plan.points if point not in record.sparse_removed_cells]
    removed = [{"center": point, "failed_disk": disk}
               for point, disk in record.sparse_removed_cells.items()]
    if not removed:
        return plan
    if not retained:
        raise ValueError("Failed-clear disks exclude the entire certified region")
    return replace(plan, kind="SPARSE_REMAINING_GRID", points=tuple(retained),
                   cover_certificate={"method": "SPARSE_REMAINING_GRID",
                                      "parent": parent, "removed_cells": removed,
                                      "failed_clear_disks": tuple(record.failed_clear_disks),
                                      "remaining_version": record.remaining_version})


def _filter_failed_strip(record, plan):
    if not record.failed_clear_disks or plan.kind != "STRIP":
        return plan
    removed = tuple(p for p in plan.points if p in record.failed_clear_disks)
    if not removed:
        return plan
    retained = tuple(p for p in plan.points if p not in record.failed_clear_disks)
    if not retained:
        raise ValueError("Failed-clear disks exclude the entire strip")
    return replace(plan, kind="FILTERED_REMAINING_STRIP", points=retained,
                   cover_certificate={"method": "FILTERED_REMAINING_STRIP",
                                      "parent": plan.cover_certificate,
                                      "removed_points": removed,
                                      "failed_clear_disks": tuple(record.failed_clear_disks),
                                      "remaining_version": record.remaining_version})


def verify_remaining_cover_certificate(record, plan, safe_radius_m=19.9):
    cert = plan.cover_certificate or {}
    if cert.get("method") == "FILTERED_REMAINING_STRIP":
        parent = cert.get("parent", {})
        if (cert.get("remaining_version") != record.remaining_version or
                parent.get("method") != "FIRST_BEARING_STRIP" or
                parent.get("half_width_deg") != 1.005):
            return False
        a = parent["anchor"]
        expected = set(strip_clear_points((a["position"]["x"], a["position"]["y"]), a["svd_deg"]))
        removed = set(map(tuple, cert.get("removed_points", ())))
        retained = set(plan.points)
        return (retained.isdisjoint(removed) and retained | removed == expected and
                removed <= set(record.failed_clear_disks) and plan.cover_radius_m <= safe_radius_m)
    if cert.get("method") != "SPARSE_REMAINING_GRID":
        if cert.get("method") == "FIRST_BEARING_STRIP":
            return (plan.point_count > 0 and plan.cover_radius_m <= safe_radius_m and
                    cert.get("half_width_deg") == 1.005)
        return verify_cover_certificate(record.vertices, plan, safe_radius_m)
    if (cert.get("remaining_version") != record.remaining_version or
            tuple(map(tuple, cert.get("failed_clear_disks", ()))) != tuple(record.failed_clear_disks)):
        return False
    parent = cert.get("parent", {})
    if parent.get("method") != "ENCLOSING_RECTANGLE_CELL_HALF_DIAGONAL":
        return False
    angle = math.radians(parent["orientation_deg"])
    u, n = np.array([math.cos(angle), math.sin(angle)]), np.array([-math.sin(angle), math.cos(angle)])
    u0, u1, n0, n1 = parent["bounds_un"]
    nu, nn = parent["counts_un"]
    du, dn = (u1-u0)/nu, (n1-n0)/nn
    if max(du, dn) > 28 or math.hypot(du/2, dn/2) > safe_radius_m:
        return False
    expected = {(i, j) for i in range(nu) for j in range(nn)}
    def index(point):
        a, b = np.asarray(point)@u, np.asarray(point)@n
        i, j = round((a-u0)/du-0.5), round((b-n0)/dn-0.5)
        return (i, j) if (i, j) in expected else None
    retained = [index(p) for p in plan.points]
    removed = cert.get("removed_cells", ())
    removed_indices = [index(row["center"]) for row in removed]
    if (None in retained or None in removed_indices or len(set(retained)) != len(retained)
            or len(set(removed_indices)) != len(removed_indices)
            or set(retained) & set(removed_indices)
            or set(retained) | set(removed_indices) != expected):
        return False
    disks = set(record.failed_clear_disks)
    for row in removed:
        disk = tuple(row["failed_disk"])
        if disk not in disks or not _inside_failed_disk(_grid_cell_corners(row["center"], parent), disk):
            return False
    return True


def build_clear_route_variants(record, plan, current, continuation=None):
    """Reorder one certified point set by route and soft hit likelihood."""
    if len(plan.points) <= 1:
        return (reprice(plan, current, continuation),)
    points = tuple(plan.points)
    variants = [points, points[::-1]]
    remaining, nearest, here = list(points), [], current
    while remaining:
        point = min(remaining, key=lambda p: (math.dist(here, p), p))
        nearest.append(point)
        remaining.remove(point)
        here = point
    variants.extend((tuple(nearest), tuple(reversed(nearest))))
    try:
        from .belief import hit_probabilities
        probability = dict(zip(points, hit_probabilities(record, points)))
        remaining, priority, here = list(points), [], current
        while remaining:
            point = max(remaining, key=lambda p: (probability[p]/(1e-6+math.dist(here, p)/5),
                                                   -math.dist(here, p)))
            priority.append(point)
            remaining.remove(point)
            here = point
        variants.extend((tuple(priority), tuple(reversed(priority))))
    except (ValueError, ArithmeticError):
        pass
    return tuple(reprice(replace(plan, points=v), current, continuation)
                 for v in dict.fromkeys(variants))


def select_clear_route(record, plan, current, continuation=None):
    variants = build_clear_route_variants(record, plan, current, continuation)
    try:
        from .belief import expected_clear_cost
        # Expected early-hit time is the primary objective, but a posterior can
        # be noisy in the tail.  Treat a 25% excess over the shortest certified
        # completion bound as a hard risk veto instead of blending incompatible
        # expected and worst-case quantities into one arbitrary score.
        best_upper = min(candidate.completion_upper_s for candidate in variants)
        admissible = tuple(candidate for candidate in variants
                           if candidate.completion_upper_s
                           <= CLEAR_ROUTE_RISK_CAP_RATIO*best_upper + 1e-9)
        scored = [(expected_clear_cost(record, candidate, current, continuation), candidate)
                  for candidate in admissible]
        return min(scored, key=lambda row: (row[0] if row[0] is not None else row[1].completion_upper_s,
                                            row[1].completion_upper_s))[1]
    except (ValueError, ArithmeticError):
        return min(variants, key=lambda candidate: candidate.completion_upper_s)


def build_clear_plan(record, current, continuation=None):
    if record.anchor is None:
        raise ValueError("A direction anchor is required for non-near clearing")
    plan = record.base_clear_plan if record.base_clear_version == record.region_version else None
    try:
        if record.vertices is not None:
            if plan is None:
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
    if record.base_clear_version != record.region_version:
        record.base_clear_plan, record.base_clear_version = plan, record.region_version
        record.sparse_removed_cells.clear()
        record.sparse_processed_disks = 0
    plan = _filter_failed_strip(record, _sparsify_grid(record, plan))
    if not verify_remaining_cover_certificate(record, plan):
        raise ValueError("Independent P_remain cover verification failed")
    return select_clear_route(record, plan, current, continuation)
