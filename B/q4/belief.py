"""Soft Q4 scenario belief used only for ordering and expected-cost estimates.

The deterministic geometry in :mod:`B.q4.geometry` remains authoritative.
An empty or degenerate belief therefore disables probabilistic scoring instead
of changing feasibility, discovery, covering, or completion certificates.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class BeliefState:
    positions: np.ndarray
    radii_m: np.ndarray
    directional: np.ndarray
    emit_deg: np.ndarray
    weights: np.ndarray
    target_count: int
    minimum_count: int
    generated: int

    @property
    def count(self):
        return int(len(self.positions))

    @property
    def active(self):
        return self.count >= self.minimum_count and self.weights.sum() > 0

    def summary(self):
        return {"count": self.count, "target_count": self.target_count,
                "minimum_count": self.minimum_count, "active": self.active,
                "effective_sample_size": (float(1/np.square(self.weights).sum())
                                          if self.active else 0.0)}


def _inside_convex(vertices, points):
    edges = np.roll(vertices, -1, axis=0)-vertices
    rel = points[:, None, :]-vertices[None, :, :]
    cross = edges[None, :, 0]*rel[:, :, 1]-edges[None, :, 1]*rel[:, :, 0]
    return np.all(cross >= -1e-7*np.maximum(1, np.linalg.norm(edges, axis=1))[None, :], axis=1)


def _sample_polygon(vertices, count, rng):
    """Area-uniform fan triangulation of a convex polygon."""
    if len(vertices) < 3 or count <= 0:
        return np.empty((0, 2))
    triangles = np.stack([np.repeat(vertices[:1], len(vertices)-2, axis=0),
                          vertices[1:-1], vertices[2:]], axis=1)
    areas = np.abs(np.cross(triangles[:, 1]-triangles[:, 0],
                            triangles[:, 2]-triangles[:, 0]))/2
    if areas.sum() <= 0:
        return np.empty((0, 2))
    selected = rng.choice(len(triangles), count, p=areas/areas.sum())
    u, v = rng.random(count), rng.random(count)
    fold = u+v > 1
    u[fold], v[fold] = 1-u[fold], 1-v[fold]
    t = triangles[selected]
    return t[:, 0]+u[:, None]*(t[:, 1]-t[:, 0])+v[:, None]*(t[:, 2]-t[:, 0])


def _angle_error(a, b):
    return np.abs((a-b+180) % 360-180)


def _consistent(record, positions, radii, directional, emit_deg):
    keep = np.ones(len(positions), dtype=bool)
    for obs in record.observations:
        point = np.asarray(obs["position"], dtype=float)
        delta = point-positions
        distance = np.linalg.norm(delta, axis=1)
        emit = np.column_stack((np.cos(np.radians(emit_deg)), np.sin(np.radians(emit_deg))))
        front = np.sum(emit*delta, axis=1) >= -1e-10
        receive = (distance <= radii+1e-9) & (~directional | front)
        if obs["result"] == "no_signal":
            keep &= ~receive
        elif obs["result"] == "near":
            keep &= receive & (distance <= 5+1e-9)
        elif obs["result"] == "direction":
            bearing = np.degrees(np.arctan2(positions[:, 1]-point[1],
                                             positions[:, 0]-point[0])) % 360
            keep &= receive & (distance > 5-1e-9) & (_angle_error(bearing, obs["svd_deg"]) <= 1.005+1e-8)
    for failed in record.failed_clear_disks:
        keep &= np.linalg.norm(positions-np.asarray(failed), axis=1) > 20
    return keep


def update_belief_scenarios(record, target_count=512, minimum_count=24):
    """Reweight, resample and replenish scenarios consistent with observations.

    The RNG seed is a stable function of observable state only. This makes
    replay logs deterministic and prevents hidden truth from leaking in.
    """
    if record.vertices is None:
        record.belief = BeliefState(np.empty((0, 2)), np.empty(0), np.empty(0, bool),
                                    np.empty(0), np.empty(0), target_count, minimum_count, 0)
        return record.belief
    seed = (record.channel*1_000_003 + record.region_version*10_007 +
            record.remaining_version*101 + len(record.observations)) & 0xffffffff
    rng = np.random.default_rng(seed)
    accepted = []
    generated = 0
    # Keep consistent old particles first (Bayesian reweight/resampling step).
    old = record.belief
    if old is not None and old.count:
        mask = _consistent(record, old.positions, old.radii_m, old.directional, old.emit_deg)
        if np.any(mask):
            indices = np.flatnonzero(mask)
            weights = old.weights[indices]
            weights = weights/weights.sum()
            take = min(target_count//2, max(len(indices), minimum_count))
            chosen = rng.choice(indices, take, replace=True, p=weights)
            # Local rejuvenation is attempted below as fresh conditional draws;
            # unchanged resamples preserve strict observation consistency.
            accepted.append((old.positions[chosen], old.radii_m[chosen],
                             old.directional[chosen], old.emit_deg[chosen]))
    have = sum(len(x[0]) for x in accepted)
    attempts = 0
    while have < target_count and attempts < 24:
        batch = max(1024, (target_count-have)*8)
        positions = _sample_polygon(np.asarray(record.vertices, float), batch, rng)
        radii = rng.uniform(1000, 1500, len(positions))
        directional = rng.random(len(positions)) < 0.5
        emit = rng.uniform(0, 360, len(positions))
        mask = _consistent(record, positions, radii, directional, emit)
        if np.any(mask):
            accepted.append((positions[mask], radii[mask], directional[mask], emit[mask]))
            have += int(mask.sum())
        generated += batch
        attempts += 1
    if accepted:
        positions = np.concatenate([x[0] for x in accepted])[:target_count]
        radii = np.concatenate([x[1] for x in accepted])[:target_count]
        directional = np.concatenate([x[2] for x in accepted])[:target_count]
        emit = np.concatenate([x[3] for x in accepted])[:target_count]
    else:
        positions, radii = np.empty((0, 2)), np.empty(0)
        directional, emit = np.empty(0, bool), np.empty(0)
    weights = np.full(len(positions), 1/len(positions)) if len(positions) else np.empty(0)
    record.belief = BeliefState(positions, radii, directional, emit, weights,
                                target_count, minimum_count, generated)
    return record.belief


def response_probabilities(record, point):
    belief = record.belief
    if belief is None or not belief.active:
        return None
    point = np.asarray(point, float)
    delta = point-belief.positions
    distance = np.linalg.norm(delta, axis=1)
    emit = np.column_stack((np.cos(np.radians(belief.emit_deg)),
                            np.sin(np.radians(belief.emit_deg))))
    receive = ((distance <= belief.radii_m) &
               (~belief.directional | (np.sum(emit*delta, axis=1) >= 0)))
    near = receive & (distance <= 5)
    return {"near": float(belief.weights[near].sum()),
            "direction": float(belief.weights[receive & ~near].sum()),
            "no_signal": float(belief.weights[~receive].sum())}


def hit_probabilities(record, points):
    belief = record.belief
    if belief is None or not belief.active:
        return tuple(0.0 for _ in points)
    return tuple(float(belief.weights[np.linalg.norm(belief.positions-np.asarray(p), axis=1) <= 20].sum())
                 for p in points)


def expected_clear_cost(record, plan, current, continuation=None):
    """Expected early-hit route cost. Returns None when belief is inactive."""
    belief = record.belief
    if belief is None or not belief.active or not plan.points:
        return None
    points = np.asarray(plan.points, float)
    cumulative = np.empty(len(points))
    previous, travelled = np.asarray(current, float), 0.0
    for i, point in enumerate(points):
        travelled += float(np.linalg.norm(point-previous))
        cumulative[i], previous = travelled, point
    costs = np.empty(belief.count)
    for i, source in enumerate(belief.positions):
        hit = np.flatnonzero(np.linalg.norm(points-source, axis=1) <= 20)
        if not len(hit):
            return None
        k = int(hit[0])
        connector = math.dist(tuple(points[k]), continuation)/5 if continuation is not None else 0
        costs[i] = cumulative[k]/5 + 3*k + 5 + connector
    return float(np.sum(costs*belief.weights))
