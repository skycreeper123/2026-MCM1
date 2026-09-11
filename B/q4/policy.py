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


@dataclass(frozen=True)
class MeasureTask:
    points: tuple
    reception: str
    pair: PairTask | None = None


@dataclass(frozen=True)
class BranchCostBound:
    upper_s: float
    direction_upper_s: float
    direct_upper_s: float
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
                         backup, config, deadline_monotonic=math.inf, station=False):
    direct = reprice(backup, current, continuation).completion_upper_s
    s = task.points[0]
    switch = int(receiver_channel != record.channel)
    overhead = (0 if station else math.dist(current, s)/5) + 5 + switch
    positive, unfinished = _direction_cost(record, s, continuation, backup,
                                           config.response_intervals, deadline_monotonic)
    direction = overhead + positive
    guaranteed = task.reception in {"POSITIVE_HULL", "PAIR_AT_LEAST_ONE"}
    total_intervals = config.response_intervals
    if task.pair is not None:
        other = task.points[1]
        positive2, unfinished2 = _direction_cost(record, other, continuation, backup,
                                                config.response_intervals, deadline_monotonic)
        negative = math.dist(s, other)/5 + 5 + positive2
        upper = overhead + max(positive, negative)
        unfinished += unfinished2
        total_intervals *= 2
    elif guaranteed:
        upper = direction
    else:
        upper = overhead + max(positive, reprice(backup, s, continuation).completion_upper_s)
    if station:
        # Incoming discovery leg belongs to the fixed skeleton, not this remeasure.
        direct = reprice(backup, s, continuation).completion_upper_s
        eligible = direction <= direct-1
    elif guaranteed:
        eligible = upper <= direct-config.guaranteed_saving_s
    else:
        eligible = (direction <= direct-config.uncertain_saving_s and
                    upper <= direct+config.uncertain_extra_s)
    return BranchCostBound(upper, direction, direct, not guaranteed or task.pair is not None,
                           total_intervals, unfinished, eligible)


def candidate_tasks(record, current, continuation, backup, future_stations, limit=24,
                    deadline_monotonic=math.inf):
    guaranteed = list(hull_candidates(record))
    center = tuple(map(float, np.mean(record.vertices, axis=0)))
    pool = guaranteed + [current, center, backup.entry_point] + list(future_stations)
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
            tasks.append(MeasureTask((p,), certificate))
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
    tasks.extend(MeasureTask(pair.points, pair.certificate, pair) for pair in pairs[:limit-len(tasks)])
    # Evaluate reception-guaranteed candidates before uncertain probes.
    tasks.sort(key=lambda t: (t.reception not in {"POSITIVE_HULL", "PAIR_AT_LEAST_ONE"},
                              math.dist(current, t.points[0])))
    return tasks[:limit]
