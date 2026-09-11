"""Build/check finite geometric certificates for the proposed Q4 framework.

This is a design verifier, not a robot controller or a simulator score.
Run from the project root: python -m B.q4.verify_q4_framework
"""

from collections import Counter
from fractions import Fraction
from itertools import combinations
from pathlib import Path
import json
import math
import time

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, milp
from scipy.spatial import ConvexHull, Delaunay


OUT = Path(__file__).resolve().parent
GEOMETRY_TOL_M = 1e-7
CERTIFICATE_RANGE_M = 999.0


def stations():
    return np.array([(0.0, 0.0)] + [
        (995 * math.cos(k * math.pi / 4), 995 * math.sin(k * math.pi / 4))
        for k in range(8)
    ] + [
        (1864 * math.cos(k * math.pi / 6), 1864 * math.sin(k * math.pi / 6))
        for k in range(12)
    ])


def cross(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def area(poly):
    return sum(cross(a, b) for a, b in zip(poly, np.roll(poly, -1, axis=0))) / 2


def inside(poly, points):
    return all(cross(b-a, p-a) / np.linalg.norm(b-a) >= -GEOMETRY_TOL_M
               for a, b in zip(poly, np.roll(poly, -1, axis=0)) for p in points)


def split_triangle(t):
    a, b, c = t
    ab, bc, ca = (a+b)/2, (b+c)/2, (c+a)/2
    return [np.array(s) for s in [(a, ab, ca), (ab, b, bc),
                                (ca, bc, c), (ab, bc, ca)]]


def make_certificate(v):
    roots = []
    for ids in Delaunay(v).simplices:
        ids = list(map(int, ids))
        if area(v[ids]) < 0:
            ids[1], ids[2] = ids[2], ids[1]
        roots.append(ids)

    def visit(t, depth):
        distances = np.linalg.norm(v[:, None, :] - t[None, :, :], axis=2).max(axis=1)
        ids = np.where(distances <= CERTIFICATE_RANGE_M)[0]
        if len(ids) >= 3:
            hull = ConvexHull(v[ids])
            provider_ids = ids[hull.vertices]
            if inside(v[provider_ids], t):
                return {"providers": provider_ids.tolist()}
        if depth >= 12:
            raise RuntimeError("Uncertified cell: do not enable this station network")
        return {"children": [visit(child, depth+1) for child in split_triangle(t)]}

    return {"schema": 1, "station_count": len(v), "stations_m": v.tolist(),
            "target_radius_m": 1800.0, "distance_limit_m": CERTIFICATE_RANGE_M,
            "geometry_tolerance_m": GEOMETRY_TOL_M,
            "root_triangles": roots, "trees": [visit(v[ids], 0) for ids in roots]}


def check_certificate(cert):
    """Recheck stored subdivisions with edge tests, not sampled source positions."""
    v = np.array(cert["stations_m"], dtype=float)
    assert np.array_equal(v, stations())
    roots = cert["root_triangles"]
    hull_ids = list(map(int, ConvexHull(v).vertices))
    hull = v[hull_ids]
    assert area(hull) > 0
    inradius = min(cross(b-a, -a)/np.linalg.norm(b-a)
                   for a, b in zip(hull, np.roll(hull, -1, axis=0)))
    assert inradius > 1800
    edges = Counter()
    root_area = 0.0
    for ids in roots:
        t = v[ids]
        assert area(t) > 0 and inside(hull, t)
        root_area += area(t)
        for a, b in zip(ids, ids[1:] + ids[:1]):
            edges[(a, b)] += 1
    boundary = set(zip(hull_ids, hull_ids[1:] + hull_ids[:1]))
    for edge, count in edges.items():
        reverse = (edge[1], edge[0])
        assert count == 1
        assert (edge in boundary and edges[reverse] == 0) or edges[reverse] == 1
    assert all(edges[e] == 1 for e in boundary)
    assert abs(root_area - area(hull)) < 1e-6
    leaf_count, depth_max, max_distance = 0, 0, 0.0

    def visit(t, node, depth):
        nonlocal leaf_count, depth_max, max_distance
        depth_max = max(depth_max, depth)
        if "providers" in node:
            provider = v[node["providers"]]
            assert area(provider) > 0 and inside(provider, t)
            d = float(np.linalg.norm(provider[:, None, :] - t[None, :, :], axis=2).max())
            assert d <= cert["distance_limit_m"] + 1e-8
            max_distance = max(max_distance, d)
            leaf_count += 1
        else:
            assert len(node["children"]) == 4
            for sub, child in zip(split_triangle(t), node["children"]):
                visit(sub, child, depth+1)

    assert len(roots) == len(cert["trees"])
    for ids, tree in zip(roots, cert["trees"]):
        visit(v[ids], tree, 0)
    return {"passed": True, "stations": len(v), "root_triangles": len(roots),
            "leaf_triangles": leaf_count, "maximum_subdivision_depth": depth_max,
            "max_provider_distance_m": max_distance,
            "outer_polygon_inradius_m": inradius,
            "boundary_margin_m": inradius - 1800,
            "range_margin_m": 1000 - max_distance,
            "scope": "continuous triangle containment and vertex-distance certificate; floating point"}


def check_exact_geometry(cert):
    """Exact rational checks of the stored station coordinates and dyadic cells."""
    v = [tuple(Fraction(float(x)) for x in p) for p in cert["stations_m"]]
    def sub(a, b):
        return (a[0]-b[0], a[1]-b[1])
    def det(a, b):
        return a[0]*b[1]-a[1]*b[0]
    def orient(a, b, p):
        return det(sub(b, a), sub(p, a))
    def norm2(a):
        return a[0]*a[0]+a[1]*a[1]
    def mid(a, b):
        return ((a[0]+b[0])/2, (a[1]+b[1])/2)
    def contains(poly, points):
        return all(orient(a, b, p) >= 0 for a, b in
                   zip(poly, poly[1:]+poly[:1]) for p in points)
    outer_ids = list(map(int, ConvexHull(np.array(cert["stations_m"])).vertices))
    outer = [v[i] for i in outer_ids]
    assert contains(outer, v)
    for a, b in zip(outer, outer[1:]+outer[:1]):
        q = orient(a, b, (Fraction(0), Fraction(0)))
        assert q > 0 and q*q >= 1800**2 * norm2(sub(b, a))
    roots = cert["root_triangles"]
    assert sum(orient(*[v[i] for i in ids]) for ids in roots) == sum(
        det(a, b) for a, b in zip(outer, outer[1:]+outer[:1]))
    mesh_edges = list(set(tuple(sorted(e)) for ids in roots
                          for e in zip(ids, ids[1:]+ids[:1])))
    for e, f in combinations(mesh_edges, 2):
        if set(e) & set(f):
            continue
        a, b = (v[i] for i in e)
        c, d = (v[i] for i in f)
        assert not (orient(a,b,c)*orient(a,b,d) < 0
                    and orient(c,d,a)*orient(c,d,b) < 0)
    leaves = 0
    def visit(t, node):
        nonlocal leaves
        assert orient(*t) > 0
        if "providers" in node:
            poly = [v[i] for i in node["providers"]]
            assert contains(poly, t)
            assert all(norm2(sub(a,b)) <= 999**2 for a in poly for b in t)
            leaves += 1
            return
        a,b,c = t
        ab,bc,ca = mid(a,b),mid(b,c),mid(c,a)
        children = [(a,ab,ca),(ab,b,bc),(ca,bc,c),(ab,bc,ca)]
        for child, node_child in zip(children, node["children"]):
            visit(child,node_child)
    for ids, tree in zip(roots, cert["trees"]):
        visit(tuple(v[i] for i in ids),tree)
    return {"passed": True, "leaf_triangles": leaves,
            "arithmetic": "exact fractions of stored binary64 station coordinates",
            "scope": "disk containment, noncrossing mesh, dyadic subdivision, provider hull containment, squared distances <= 999^2"}


def solve_station_route(v, seconds=25):
    """Open Hamiltonian path from station 0 via a zero-cost dummy node.

    A connected optimum of a subtour relaxation is also a full TSP optimum.
    Time-limited outcomes retain a valid heuristic path and report the gap.
    """
    n = len(v)
    dummy = n
    edges = list(combinations(range(n+1), 2))
    cost = np.array([0.0 if b == dummy else np.linalg.norm(v[a]-v[b]) for a, b in edges])
    degree = np.zeros((n+1, len(edges)))
    for j, (a, b) in enumerate(edges):
        degree[a, j] = degree[b, j] = 1
    fixed = np.zeros(len(edges))
    fixed[edges.index((0, dummy))] = 1
    rows = list(degree) + [fixed]
    lower, upper = [2.0]*(n+1)+[1.0], [2.0]*(n+1)+[1.0]
    # Always keep a legal route available.
    route, remaining = [0], set(range(1, n))
    while remaining:
        j = min(remaining, key=lambda j: (np.linalg.norm(v[route[-1]]-v[j]), j))
        route.append(j)
        remaining.remove(j)
    def length(r):
        return sum(float(np.linalg.norm(v[a]-v[b])) for a, b in zip(r, r[1:]))
    improved = True
    while improved:
        improved = False
        for a in range(1, n-1):
            for b in range(a+1, n):
                candidate = route[:a] + route[a:b+1][::-1] + route[b+1:]
                if length(candidate) < length(route)-1e-8:
                    route, improved = candidate, True
    deadline = time.monotonic() + seconds
    rounds, lower_bound, optimal = 0, 0.0, False
    while time.monotonic() < deadline:
        result = milp(cost, integrality=np.ones(len(edges)), bounds=Bounds(0, 1),
                      constraints=LinearConstraint(np.array(rows), lower, upper),
                      options={"time_limit": max(0.1, deadline-time.monotonic()),
                               "mip_rel_gap": 1e-8})
        rounds += 1
        bound = getattr(result, "mip_dual_bound", None)
        if bound is not None and math.isfinite(bound):
            lower_bound = max(lower_bound, float(bound))
        if result.x is None:
            break
        adj = {k: [] for k in range(n+1)}
        for value, (a, b) in zip(result.x, edges):
            if value > 0.5:
                adj[a].append(b)
                adj[b].append(a)
        if any(len(a) != 2 for a in adj.values()):
            break
        unseen, components = set(range(n+1)), []
        while unseen:
            stack, comp = [next(iter(unseen))], set()
            while stack:
                k = stack.pop()
                if k in comp:
                    continue
                comp.add(k)
                stack.extend(b for b in adj[k] if b not in comp)
            unseen -= comp
            components.append(comp)
        if len(components) == 1:
            found, prev, cur = [0], dummy, 0
            while True:
                nxt = next(k for k in adj[cur] if k != prev)
                if nxt == dummy:
                    break
                found.append(nxt)
                prev, cur = cur, nxt
            assert sorted(found) == list(range(n))
            if length(found) < length(route)+1e-8:
                route = found
            optimal = result.status == 0
            break
        for comp in components:
            rows.append(np.array([float(a in comp and b in comp) for a, b in edges]))
            lower.append(-np.inf)
            upper.append(float(len(comp)-1))
    total = length(route)
    return {"station_ids": route, "length_m": total, "lower_bound_m": lower_bound,
            "relative_gap": max(0.0, (total-lower_bound)/total),
            "solver_optimal": optimal, "cut_rounds": rounds,
            "scope": "fixed 21 stations, start 0, free endpoint, station movement only"}


def check_reception_lemmas():
    rng = np.random.default_rng(20260911)
    for _ in range(10000):
        phi = rng.uniform(-math.pi, math.pi)
        u = np.array([math.cos(phi), math.sin(phi)])
        r = rng.uniform(1000, 1500)
        angles = phi + rng.uniform(-math.pi/2, math.pi/2, 4)
        radii = rng.uniform(0, r, 4)
        positive = np.column_stack((np.cos(angles), np.sin(angles))) * radii[:, None]
        q = rng.dirichlet(np.ones(4)) @ positive
        assert np.linalg.norm(q) <= r+1e-8 and q@u >= -1e-8
    pair_cases = 0
    for _ in range(10000):
        phi = rng.uniform(-math.pi, math.pi)
        u = np.array([math.cos(phi), math.sin(phi)])
        a = rng.uniform(0, 700)*u
        w = rng.uniform(-200, 200, 2)
        q1, q2 = a+w, a-w
        if max(np.linalg.norm(q1), np.linalg.norm(q2)) <= 999.9:
            assert max(q1@u, q2@u) >= -1e-8
            pair_cases += 1
    return {"positive_hull_cases": 10000, "paired_probe_cases": pair_cases,
            "passed": True, "scope": "synthetic checks supplement the algebraic proofs"}


def main():
    cert = make_certificate(stations())
    certificate_path = OUT / "Q4_COVERAGE_CERTIFICATE.json"
    certificate_path.write_text(json.dumps(cert, ensure_ascii=False, indent=2), encoding="utf-8")
    checks = check_certificate(json.loads(certificate_path.read_text(encoding="utf-8")))
    route = solve_station_route(stations())
    report = {"coverage": checks, "exact_geometry": check_exact_geometry(cert), "station_route": route,
              "reception_lemmas": check_reception_lemmas(),
              "baseline_scan_measure_upper": 21*20,
              "baseline_clear_request_upper": 16*152,
              "baseline_virtual_upper_s": route["length_m"]/5 + 21*20*6 + 16*1070,
              "not_run": "Q4 end-to-end controller, official practice or formal tests"}
    (OUT / "Q4_FRAMEWORK_CHECKS.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
