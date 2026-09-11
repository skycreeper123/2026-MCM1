"""Finite full-route insertion, 2-opt and bounded prefix search for Q4."""

from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class RouteTask:
    key: tuple
    points: tuple
    kind: str
    channel: int | None = None
    channels: tuple = ()


def task_route_cost(tasks, current, receiver_channel):
    """Connector counted once. CLEAR never changes the receiver's channel."""
    cost, point, channel = 0.0, current, receiver_channel
    for task in tasks:
        for q in task.points:
            cost += math.dist(point, q)/5
            point = q
        if task.kind == "CLEAR":
            cost += 3*(len(task.points)-1)+5
        else:
            for ch in (task.channels if task.kind == "STATION" else (task.channel,)):
                cost += 5 + int(channel != ch)
                channel = ch
    return cost


def optimize_service_route(stations, clears, current, receiver_channel,
                           deadline=math.inf, depth=3, width=12):
    """Always retain all discovery duties and all known-source clear backups.

    Insertion/2-opt preserve station order. Beam search reorders a bounded
    service prefix; each node is scored with its complete fallback suffix.
    """
    route = list(stations) + list(clears)
    def cost(r):
        return task_route_cost(r, current, receiver_channel)
    # Nearest full-cost insertion of each service into the discovery skeleton.
    for task in clears:
        if time.monotonic() >= deadline:
            break
        base = [x for x in route if x.key != task.key]
        candidates = [base[:i]+[task]+base[i:] for i in range(len(base)+1)]
        route = min(candidates, key=cost)
    # Only reverse stretches containing clears: no discovery duty reordered.
    best = cost(route)
    for i in range(len(route)):
        for j in range(i+1, len(route)):
            if time.monotonic() >= deadline:
                return tuple(route)
            if any(t.kind == "STATION" for t in route[i:j+1]):
                break
            trial = route[:i]+route[i:j+1][::-1]+route[j+1:]
            value = cost(trial)
            if value < best:
                route, best = trial, value
    beam = [([], list(route))]
    for _ in range(depth):
        expanded = []
        for prefix, suffix in beam:
            if time.monotonic() >= deadline:
                return tuple(route)
            # First station or any clear can be next. Later stations cannot
            # leapfrog an unfinished earlier discovery duty.
            eligible = [i for i, t in enumerate(suffix) if t.kind != "STATION"]
            eligible += next(([i] for i, t in enumerate(suffix) if t.kind == "STATION"), [])
            for i in eligible:
                p, s = prefix+[suffix[i]], suffix[:i]+suffix[i+1:]
                expanded.append((p, s))
                value = cost(p+s)
                if value < best:
                    route, best = p+s, value
        if not expanded:
            break
        beam = sorted(expanded, key=lambda ps: cost(ps[0]+ps[1]))[:width]
    return tuple(route)
