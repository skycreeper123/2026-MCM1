"""Bounded Q4 measure-then-clear policy trees, including negative responses."""

from dataclasses import dataclass
import math
import time

import numpy as np

from B.q1.geometry import bearing_halfplanes
from B.q2 import clip_convex_polygon, safe_candidate_region
from B.q3.localize import (_bearing_domain, _rectangle_plan, build_cover_plan,
                          spatial_candidate_subset, verified_enclosing_circle,
                          verify_cover_certificate)
from .geometry import (PairTask, build_pair_probe, classify_q4_reception,
                       hull_candidates, reprice)
from .belief import expected_clear_cost, response_probabilities


@dataclass(frozen=True)
class MeasureTask:
    points: tuple
    reception: str
    pair: PairTask | None = None
    origin: str = "static"


@dataclass(frozen=True)
class BranchCostBound:
    upper_s: float
    direction_upper_s: float
    direct_upper_s: float
    expected_s: float
    direct_expected_s: float
    expected_saving_s: float
    response_probabilities: dict | None
    includes_no_signal: bool
    intervals_total: int
    intervals_fallback: int
    eligible: bool
    scope: str = "finite measure-then-certified-clear policy; not arbitrary future replanning"


def _direction_cost(record, point, continuation, backup, intervals, deadline):
    whole = reprice(backup, point, continuation).completion_upper_s
    near = 5 + (math.dist(point, continuation)/5 if continuation is not None else 0)
    lo, hi = _bearing_domain(record.vertices, point, 1.005)
    boundaries = np.linspace(lo, hi, intervals+1)
    costs, unfinished = [near], 0
    for i, (a, b) in enumerate(zip(boundaries[:-1], boundaries[1:])):
        if time.monotonic() >= deadline:
            # All remaining intervals are explicitly represented by one whole-P
            # fallback, NOT discarded. Keep their count for auditing.
            costs.append(whole)
            unfinished += intervals-i
            break
        try:
            width = 1.005+(b-a)/2
            if width >= 90:
                poly = record.vertices
            else:
                obs = {"position": {"x": point[0], "y": point[1]}, "svd_deg": float((a+b)/2 % 360)}
                A, rhs = bearing_halfplanes([obs], width)
                poly = clip_convex_polygon(record.vertices, A, rhs)
            if not len(poly):
                # Numerically empty envelopes are not used to rule out responses.
                costs.append(whole)
                unfinished += 1
                continue
            # A small set can use one certified enclosing-circle clear.
            _, radius = verified_enclosing_circle(poly)
            if radius+1e-7 <= 19.9:
                plan = build_cover_plan(poly, point, continuation, max_points=1)
            else:
                plan = _rectangle_plan(poly, record.anchor["svd_deg"], point, continuation,
                                       28, 19.9, 1e-7, 5, 3, 5)
            cost = (plan.completion_upper_s if plan is not None and plan.point_count <= 152 and
                    verify_cover_certificate(poly, plan) else whole)
            costs.append(min(whole, cost))
        except (ValueError, ArithmeticError, np.linalg.LinAlgError):
            costs.append(whole)
            unfinished += 1
    return max(costs), unfinished


def score_measure_policy(record, task, current, receiver_channel, continuation,
                         backup, config, deadline_monotonic=math.inf, station=False,
                         response_intervals=None):
    direct = reprice(backup, current, continuation).completion_upper_s
    direct_expected = expected_clear_cost(record, backup, current, continuation)
    if direct_expected is None:
        direct_expected = direct
    s = task.points[0]
    switch = int(receiver_channel != record.channel)
    overhead = (0 if station else math.dist(current, s)/5) + 5 + switch
    intervals = response_intervals or config.response_intervals
    positive, unfinished = _direction_cost(record, s, continuation, backup,
                                           intervals, deadline_monotonic)
    direction = overhead + positive
    guaranteed = task.reception in {"POSITIVE_HULL", "PAIR_AT_LEAST_ONE"}
    total_intervals = intervals
    probabilities = response_probabilities(record, s)
    near_cost = 5 + (math.dist(s, continuation)/5 if continuation is not None else 0)
    no_signal_cost = reprice(backup, s, continuation).completion_upper_s
    if probabilities is None:
        probabilities = ({"near": 0.0, "direction": 1.0, "no_signal": 0.0}
                         if guaranteed else {"near": 0.0, "direction": 0.5, "no_signal": 0.5})
    if guaranteed:
        positive_mass = probabilities["near"]+probabilities["direction"]
        probabilities = ({"near": probabilities["near"]/positive_mass,
                          "direction": probabilities["direction"]/positive_mass,
                          "no_signal": 0.0} if positive_mass > 0 else
                         {"near": 0.0, "direction": 1.0, "no_signal": 0.0})
    if task.pair is not None:
        other = task.points[1]
        positive2, unfinished2 = _direction_cost(record, other, continuation, backup,
                                                intervals, deadline_monotonic)
        negative = math.dist(s, other)/5 + 5 + positive2
        upper = overhead + max(positive, negative)
        expected = overhead + probabilities["near"]*near_cost + probabilities["direction"]*positive + probabilities["no_signal"]*negative
        unfinished += unfinished2
        total_intervals *= 2
    elif guaranteed:
        upper = direction
        expected = overhead + probabilities["near"]*near_cost + probabilities["direction"]*positive
    else:
        upper = overhead + max(positive, no_signal_cost)
        expected = overhead + probabilities["near"]*near_cost + probabilities["direction"]*positive + probabilities["no_signal"]*no_signal_cost
    saving = direct_expected-expected
    if station:
        eligible = saving >= config.guaranteed_saving_s and upper <= direct+config.uncertain_extra_s
    elif guaranteed:
        eligible = saving >= config.guaranteed_saving_s and upper <= direct+config.uncertain_extra_s
    else:
        eligible = (saving >= config.uncertain_saving_s and
                    direction <= direct-config.uncertain_saving_s and
                    upper <= direct+config.uncertain_extra_s)
    return BranchCostBound(upper, direction, direct, expected, direct_expected, saving,
                           probabilities, not guaranteed or task.pair is not None,
                           total_intervals, unfinished, eligible)


def marginal_detour(current, points, continuation=None):
    path = (current,)+tuple(points)+(() if continuation is None else (continuation,))
    with_task = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    without = math.dist(current, continuation) if continuation is not None else 0.0
    return max(0.0, with_task-without)


def candidate_tasks(record, current, continuation, backup, future_stations, limit=24,
                    deadline_monotonic=math.inf, dynamic_candidates=()):
    guaranteed = list(hull_candidates(record))
    center = tuple(map(float, np.mean(record.vertices, axis=0)))
    pool = guaranteed + list(dynamic_candidates) + [center, backup.entry_point] + list(future_stations)
    # Reuse Q2's distance-intersection witness/bounding box as a candidate
    # generator ONLY; orientation/reception is certified separately below.
    safe_region = safe_candidate_region(record.vertices, 999.9)
    if safe_region["nonempty"]:
        pool.append(tuple(map(float, safe_region["witness_center"])))
        box = safe_region["bounding_box"]
        pool.extend((float(x), float(y)) for x in np.linspace(box["x_min"], box["x_max"], 3)
                    for y in np.linspace(box["y_min"], box["y_max"], 3))
    # Deterministic distance-only pool around the region centre; certificates
    # are ALWAYS recomputed for Q4 rather than inherited from Q2's omni model.
    angle = math.radians(record.anchor["svd_deg"])
    for radius in (100, 300, 600):
        for k in range(8):
            theta = angle+k*math.pi/4
            pool.append((center[0]+radius*math.cos(theta), center[1]+radius*math.sin(theta)))
    pool = [p for p in dict.fromkeys(pool) if p not in record.used_positions]
    pool = spatial_candidate_subset(pool, max(1, limit//2), center, current,
                                    record.anchor["svd_deg"], continuation)
    tasks = []
    for p in pool:
        certificate = classify_q4_reception(record, p)
        if certificate != "GUARANTEED_NO_SIGNAL":
            origin = "dynamic_route" if p in dynamic_candidates else ("future_station" if p in future_stations else "static")
            tasks.append(MeasureTask((p,), certificate, origin=origin))
    pairs = []
    for midpoint in record.positive_points + guaranteed:
        if time.monotonic() >= deadline_monotonic:
            break
        for length in (50, 100, 200, 400):
            for k in range(4):
                if time.monotonic() >= deadline_monotonic:
                    break
                theta = angle+k*math.pi/4
                pair = build_pair_probe(record, midpoint, (length*math.cos(theta), length*math.sin(theta)), current)
                if pair is not None and pair.points not in {p.points for p in pairs}:
                    pairs.append(pair)
    pairs.sort(key=lambda pair: (math.dist(current, pair.points[0])+math.dist(*pair.points), pair.points))
    tasks.extend(MeasureTask(pair.points, pair.certificate, pair, "symmetric_pair")
                 for pair in pairs[:limit-len(tasks)])
    # Evaluate reception-guaranteed candidates before uncertain probes.
    tasks.sort(key=lambda t: (t.reception not in {"POSITIVE_HULL", "PAIR_AT_LEAST_ONE"},
                              math.dist(current, t.points[0])))
    return tasks[:limit]
