"""Offline mixed worlds. Hidden truth lives here, never in Q4Planner."""

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time

import numpy as np

from B.q3.localize import verify_cover_certificate
from .geometry import load_and_verify_station_cover
from .planner import Q4Config, Q4Planner


@dataclass
class Source:
    channel: int
    position: tuple
    radius_m: float = 1000.0
    direction_deg: float | None = None
    phase: float = 0.0
    cleared: bool = False

    def receives(self, point):
        if self.cleared or math.dist(point, self.position) > self.radius_m:
            return False
        if self.direction_deg is None:
            return True
        angle = math.radians(self.direction_deg)
        return math.cos(angle)*(point[0]-self.position[0])+math.sin(angle)*(point[1]-self.position[1]) >= 0

    def measure(self, point):
        if not self.receives(point):
            return "no_signal", None
        if math.dist(point, self.position) <= 5:
            return "near", None
        true_bearing = math.degrees(math.atan2(self.position[1]-point[1], self.position[0]-point[0]))
        # Fixed, bounded spatial error field; repeating a position yields the
        # same error and never supplies independent noise reduction.
        error = math.sin(point[0]/317 + point[1]/239 + self.phase)
        return "direction", round((true_bearing+error) % 360, 2) % 360


def mixed_world(seed, count=10):
    rng = np.random.default_rng(seed)
    channels = rng.choice(np.arange(1, 21), size=count, replace=False)
    result = []
    for i, ch in enumerate(channels):
        angle = 2*math.pi*i/count
        # Explicit outward/tangent boundary sources, centre source, and random
        # interior sources. Minimum range is deliberately frequent.
        radius = 1800 if i < 3 else (0 if i == 3 else 1800*math.sqrt(rng.random()))
        position = (radius*math.cos(angle), radius*math.sin(angle))
        direction = math.degrees(angle)+(90 if i == 1 else 0) if i % 3 != 2 else None
        result.append(Source(int(ch), position, 1000 if i % 2 == 0 else float(rng.uniform(1000, 1500)),
                             direction, float(rng.uniform(0, 2*math.pi))))
    return result


def truth_in_region(vertices, point, tolerance_m=1e-6):
    # Numerical audit only; not a planner certificate or exclusion predicate.
    v, p = np.asarray(vertices), np.asarray(point)
    edges = np.roll(v, -1, axis=0)-v
    determinants = edges[:, 0]*(p-v)[:, 1]-edges[:, 1]*(p-v)[:, 0]
    return bool(np.all(determinants >= -tolerance_m*np.maximum(1, np.linalg.norm(edges, axis=1))))


def run_world(sources, config=None, cover=None, max_actions=6000):
    start = time.monotonic()
    planner = Q4Planner(config, cover)
    world = {source.channel: source for source in sources}
    audited_versions, audited_plans = set(), set()
    first_service_cursor = None
    actual_position, actual_channel, actual_virtual = (0.0, 0.0), 1, 0.0
    for index in range(max_actions):
        action = planner.propose_action()
        if action is None:
            raise AssertionError("Planner stopped without EXIT")
        if action.kind == "EXIT":
            planner.apply_exit_result(action)
            break
        source = world.get(action.channel)
        actual_virtual += math.dist(actual_position, action.position)/5
        actual_position = action.position
        if action.kind == "MEASURE":
            actual_virtual += 5 + int(actual_channel != action.channel)
            actual_channel = action.channel
            result, bearing = source.measure(action.position) if source else ("no_signal", None)
            if action.reception in {"POSITIVE_HULL", "PAIR_SECOND_GUARANTEED"}:
                if result == "no_signal":
                    raise AssertionError("False reception certificate")
            planner.apply_measure_result(action, result, bearing)
        else:
            if first_service_cursor is None:
                first_service_cursor = planner.station_cursor
            record = planner.channels[action.channel]
            plan = planner.active_clear[1]
            key = (action.channel, id(plan))
            if key not in audited_plans:
                if plan.kind not in {"STRIP", "NEAR"} and not verify_cover_certificate(record.vertices, plan):
                    raise AssertionError("Invalid clear certificate")
                if source is None or min(math.dist(source.position, p) for p in plan.points) > 20:
                    raise AssertionError("Clear plan does not cover true source")
                audited_plans.add(key)
            success = source is not None and not source.cleared and math.dist(source.position, action.position) <= 20
            if success:
                source.cleared = True
            actual_virtual += 5 if success else 3
            planner.apply_clear_result(action, "success" if success else "no_target_in_range")
        for ch, record in planner.channels.items():
            key = (ch, record.region_version)
            if record.vertices is not None and key not in audited_versions:
                if not truth_in_region(record.vertices, world[ch].position):
                    raise AssertionError(f"True source excluded on channel {ch}, version {record.region_version}")
                audited_versions.add(key)
    else:
        raise AssertionError("Finite-action cap exhausted")
    summary = planner.summary()
    if not math.isclose(actual_virtual, summary["estimated_virtual_time_s"], abs_tol=1e-7):
        raise AssertionError("Controller cost differs from independent environment accounting")
    summary.update({"offline_all_cleared": all(s.cleared for s in sources), "source_count": len(sources),
                    "config": asdict(planner.config),
                    "clearance_rate": sum(s.cleared for s in sources)/len(sources),
                    "real_elapsed_s": time.monotonic()-start, "actions_including_exit": index+1,
                    "audited_region_versions": len(audited_versions), "audited_clear_plans": len(audited_plans),
                    "first_clear_station_cursor": first_service_cursor})
    if not summary["offline_all_cleared"] or not planner.has_completion_certificate():
        raise AssertionError(f"Incomplete world: {summary}")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=[11, 29, 47])
    parser.add_argument("--counts", type=int, nargs="+", default=[10, 13, 16])
    parser.add_argument("--planning-total-s", type=float, default=60)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if any(n < 1 or n > 16 for n in args.counts):
        parser.error("counts must be 1..16")
    cover = load_and_verify_station_cover()
    results = []
    for seed in args.seeds:
        for count in args.counts:
            result = run_world(mixed_world(seed, count), Q4Config(planning_total_s=args.planning_total_s), cover)
            results.append({"seed": seed, **result})
            print(f"seed={seed} n={count}: cleared={result['cleared_count']} T={result['estimated_virtual_time_s']:.2f}s real={result['real_elapsed_s']:.2f}s", flush=True)
    report = {"scope": "offline synthetic mixed worlds; not official simulator results", "runs": results}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report


if __name__ == "__main__":
    main()
