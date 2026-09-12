"""Certificate-shielded finite-horizon belief rollout for Q4 clearing.

The rollout layer may propose a probabilistic CLEAR point, but it never uses
that proposal as a completion certificate. Every miss is followed by the
existing exact remaining-region update and the certified cover remains the
leaf policy, so optimization failure cannot weaken correctness.
"""

from dataclasses import dataclass
import math
import time

import numpy as np

from .geometry import build_clear_route_variants, reprice


@dataclass(frozen=True)
class RolloutClearDecision:
    point: tuple[float, float]
    sequence: tuple[tuple[float, float], ...]
    expected_s: float
    direct_expected_s: float
    expected_saving_s: float
    tail_s: float
    direct_tail_s: float
    completion_upper_s: float
    direct_upper_s: float
    first_hit_probability: float
    horizon: int


@dataclass(frozen=True)
class RolloutSearchResult:
    decision: RolloutClearDecision | None
    candidates_evaluated: int
    sequences_evaluated: int
    timed_out: bool


def _weighted_quantile(values, weights, quantile):
    order = np.argsort(values, kind="stable")
    cumulative = np.cumsum(weights[order])
    index = min(len(order)-1, int(np.searchsorted(cumulative, quantile, side="left")))
    return float(values[order[index]])


def _certified_scenario_costs(record, plan, start, continuation):
    belief = record.belief
    if belief is None or not belief.active or not plan.points:
        return None
    positions = belief.positions
    points = np.asarray(plan.points, dtype=float)
    hit = np.linalg.norm(positions[:, None, :]-points[None, :, :], axis=2) <= 20.0+1e-9
    if not np.all(np.any(hit, axis=1)):
        return None
    first = np.argmax(hit, axis=1)
    legs = np.empty(len(points), dtype=float)
    legs[0] = np.linalg.norm(points[0]-np.asarray(start, dtype=float))
    if len(points) > 1:
        legs[1:] = np.linalg.norm(points[1:]-points[:-1], axis=1)
    cumulative = np.cumsum(legs)
    connectors = (np.linalg.norm(points-np.asarray(continuation, dtype=float), axis=1)/5.0
                  if continuation is not None else np.zeros(len(points)))
    return cumulative[first]/5.0 + 3.0*first + 5.0 + connectors[first]


def _candidate_centres(record, plan, current, limit):
    belief = record.belief
    positions, weights = belief.positions, belief.weights
    distance = np.linalg.norm(positions[:, None, :]-positions[None, :, :], axis=2)
    covered_mass = (distance <= 20.0+1e-9) @ weights
    distance_from_robot = np.linalg.norm(positions-np.asarray(current, dtype=float), axis=1)
    order = np.lexsort((distance_from_robot, -covered_mass))
    pool = []
    for index in order[:max(4*limit, limit)]:
        neighbours = distance[index] <= 20.0+1e-9
        centroid = np.average(positions[neighbours], axis=0, weights=weights[neighbours])
        pool.extend((tuple(map(float, centroid)), tuple(map(float, positions[index]))))
    pool.extend(tuple(map(float, p)) for p in plan.points[:min(4, len(plan.points))])
    pool.append(tuple(map(float, current)))

    unique = []
    seen = set()
    for point in pool:
        key = tuple(round(value, 6) for value in point)
        if key in seen or not all(math.isfinite(value) for value in point):
            continue
        seen.add(key)
        mass = float(weights[np.linalg.norm(positions-np.asarray(point), axis=1) <= 20.0+1e-9].sum())
        if mass > 0:
            unique.append((point, mass, math.dist(current, point)))
    unique.sort(key=lambda row: (-row[1], row[2], row[0]))
    selected = []
    for point, _, _ in unique:
        if all(math.dist(point, other) > 5.0 for other in selected):
            selected.append(point)
        if len(selected) >= limit:
            break
    return tuple(selected)


def _sequence_statistics(record, plan, current, continuation, sequence):
    belief = record.belief
    weights, positions = belief.weights, belief.positions
    probes = np.asarray(sequence, dtype=float)
    probe_hit = np.linalg.norm(positions[:, None, :]-probes[None, :, :], axis=2) <= 20.0+1e-9
    any_probe = np.any(probe_hit, axis=1)
    first_probe = np.argmax(probe_hit, axis=1)
    legs = np.empty(len(probes), dtype=float)
    legs[0] = np.linalg.norm(probes[0]-np.asarray(current, dtype=float))
    if len(probes) > 1:
        legs[1:] = np.linalg.norm(probes[1:]-probes[:-1], axis=1)
    cumulative = np.cumsum(legs)
    costs = np.empty(len(positions), dtype=float)
    if np.any(any_probe):
        index = first_probe[any_probe]
        connectors = (np.linalg.norm(probes[index]-np.asarray(continuation, dtype=float), axis=1)/5.0
                      if continuation is not None else np.zeros(len(index)))
        costs[any_probe] = cumulative[index]/5.0 + 3.0*index + 5.0 + connectors
    residual = ~any_probe
    fallback_plan = min(build_clear_route_variants(
        record, plan, sequence[-1], continuation),
        key=lambda candidate: candidate.completion_upper_s)
    if np.any(residual):
        fallback = _certified_scenario_costs(record, fallback_plan, sequence[-1], continuation)
        if fallback is None:
            return None
        costs[residual] = cumulative[-1]/5.0 + 3.0*len(sequence) + fallback[residual]
    expected = float(np.sum(costs*weights))
    tail = _weighted_quantile(costs, weights, 0.95)
    upper = cumulative[-1]/5.0 + 3.0*len(sequence) + fallback_plan.completion_upper_s
    return expected, tail, upper


def plan_belief_rollout_clear(record, plan, current, continuation=None, *,
                              horizon=2, candidate_limit=8, beam_width=8,
                              minimum_saving_s=1.0, upper_ratio=1.5,
                              tail_ratio=1.25,
                              deadline_monotonic=math.inf):
    """Choose one root CLEAR action from a bounded posterior scenario tree."""
    belief = record.belief
    if (belief is None or not belief.active or plan.point_count <= 1 or
            time.monotonic() >= deadline_monotonic):
        return RolloutSearchResult(None, 0, 0, time.monotonic() >= deadline_monotonic)
    direct_costs = _certified_scenario_costs(record, plan, current, continuation)
    if direct_costs is None:
        return RolloutSearchResult(None, 0, 0, False)
    direct_expected = float(np.sum(direct_costs*belief.weights))
    direct_tail = _weighted_quantile(direct_costs, belief.weights, 0.95)
    direct_upper = reprice(plan, current, continuation).completion_upper_s
    candidates = _candidate_centres(record, plan, current, candidate_limit)
    if not candidates:
        return RolloutSearchResult(None, 0, 0, False)

    beam = ((),)
    best = None
    evaluated = 0
    timed_out = False
    for _ in range(horizon):
        expanded = []
        for prefix in beam:
            for point in candidates:
                if point in prefix or any(math.dist(point, old) <= 5.0 for old in prefix):
                    continue
                if time.monotonic() >= deadline_monotonic:
                    timed_out = True
                    break
                sequence = prefix+(point,)
                statistics = _sequence_statistics(record, plan, current, continuation, sequence)
                evaluated += 1
                if statistics is None:
                    continue
                expected, tail, upper = statistics
                if upper > upper_ratio*direct_upper or tail > tail_ratio*direct_tail:
                    continue
                expanded.append((expected, tail, sequence, upper))
                saving = direct_expected-expected
                if saving >= minimum_saving_s and (best is None or (expected, tail) < (best[0], best[1])):
                    best = expected, tail, sequence, upper
            if timed_out:
                break
        if not expanded or timed_out:
            break
        expanded.sort(key=lambda row: (row[0], row[1], row[2]))
        beam = tuple(row[2] for row in expanded[:beam_width])

    if best is None:
        return RolloutSearchResult(None, len(candidates), evaluated, timed_out)
    expected, tail, sequence, upper = best
    first = sequence[0]
    first_mass = float(belief.weights[
        np.linalg.norm(belief.positions-np.asarray(first), axis=1) <= 20.0+1e-9].sum())
    decision = RolloutClearDecision(
        first, sequence, expected, direct_expected, direct_expected-expected,
        tail, direct_tail, upper, direct_upper, first_mass, len(sequence))
    return RolloutSearchResult(decision, len(candidates), evaluated, timed_out)
