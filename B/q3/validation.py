"""Seeded, paired V3/V4 Q3 validation. Hidden truth belongs ONLY to this harness.

Run: python -m B.q3.validation --cases 24
Optional one-factor matrix: python -m B.q3.validation --cases 6 --matrix
"""

import argparse
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import time

import numpy as np

from B.q2 import is_safe_candidate
from .localize import verify_cover_certificate
from .planner import Q3Config, Q3Planner


def make_cases(count=24, seed=20260911):
    cases = []
    for k in range(count):
        rng = random.Random(seed + k)
        source_count = 10 + k % 7
        channels = rng.sample(range(1, 21), source_count)
        sources = []
        for j, channel in enumerate(channels):
            radius = 1800*math.sqrt(rng.random())
            angle = rng.uniform(0, 2*math.pi)
            if k % 4 == 0:
                radius, angle = 1800.0, (j+0.5)*2*math.pi/source_count
            if k % 4 == 1 and j == 0:
                radius = 0.0  # near at the first coverage station
            sources.append({"channel": channel,
                            "position": (radius*math.cos(angle), radius*math.sin(angle)),
                            "receive_radius_m": 1000.0 if k % 4 == 0 else rng.uniform(1000, 1500)})
        cases.append({"case_id": k, "seed": seed+k, "sources": sources,
                      "error_mode": ("positive", "negative", "alternating", "position_fixed")[k % 4]})
    return cases


def inside_region(vertices, point, tolerance=1e-6):
    vertices, point = np.asarray(vertices), np.asarray(point)
    if len(vertices) == 1:
        return np.linalg.norm(vertices[0]-point) <= tolerance
    edges = np.roll(vertices, -1, axis=0)-vertices
    norms = np.linalg.norm(edges, axis=1)
    if len(vertices) == 2:
        t = np.dot(point-vertices[0], edges[0])/max(norms[0]**2, 1e-30)
        return np.linalg.norm(point-(vertices[0]+np.clip(t, 0, 1)*edges[0])) <= tolerance
    delta = point-vertices
    cross = (edges[:, 0]*delta[:, 1]-edges[:, 1]*delta[:, 0])/np.maximum(norms, 1e-30)
    return bool(np.all(cross >= -tolerance) or np.all(cross <= tolerance))


def run_case(case, config, keep_trace=False):
    planner = Q3Planner(config)
    sources = {s["channel"]: s for s in case["sources"]}
    cleared, trace = set(), []
    safety_checks = truth_checks = certificate_checks = 0
    started = time.perf_counter()
    for index in range(10000):
        action = planner.propose_action()
        if action is not planner.propose_action():
            raise AssertionError("Pending action changed without a response")
        source = sources.get(action.channel)
        record = planner.channels.get(action.channel)
        if action.kind == "EXIT":
            planner.apply_exit_result(action)
            break
        if action.kind == "MEASURE":
            if action.purpose == "LOCALIZE":
                safety_checks += 1
                assert is_safe_candidate(action.position, record.outer_vertices,
                                         config.minimum_receive_radius_m-0.1), action
                assert all(math.dist(action.position, p) > config.repeated_position_tolerance_m
                           for p in record.measured_positions), action
            d = math.dist(action.position, source["position"]) if source else math.inf
            bearing = None
            if source is None or action.channel in cleared or d > source["receive_radius_m"]:
                result = "no_signal"
            elif d <= 5:
                result = "near"
            else:
                result = "direction"
                true = math.degrees(math.atan2(source["position"][1]-action.position[1],
                                               source["position"][0]-action.position[0]))
                mode = case["error_mode"]
                error = (1 if mode == "positive" else -1 if mode == "negative" else
                         (1 if len(record.measured_positions) % 2 else -1) if mode == "alternating" else
                         math.sin(action.position[0]*0.013 + action.position[1]*0.017 + action.channel))
                bearing = round((true+error) % 360, 2) % 360
            planner.apply_measure_result(action, result, bearing)
            if source and record.geometry_valid and record.outer_vertices is not None:
                truth_checks += 1
                assert inside_region(record.outer_vertices, source["position"]), (case["case_id"], action)
        else:
            if config.strategy in ("v3_global", "v4_cooperative"):
                assert planner.station_index == len(planner.stations), "service before discovery completion"
            if record.service_plan is not None and record.near_position is None:
                certificate_checks += 1
                assert verify_cover_certificate(record.outer_vertices, record.service_plan.cover,
                                                config.safe_clear_radius_m)
            if source and action.channel not in cleared and math.dist(action.position, source["position"]) <= 20:
                result = "success"
                cleared.add(action.channel)
            else:
                result = "no_target_in_range"
            planner.apply_clear_result(action, result)
        if keep_trace:
            trace.append({"action": action.as_dict(), "result": result,
                          "region_version": record.region_version,
                          "vertices": record.outer_vertices.tolist() if record.outer_vertices is not None else None})
    else:
        raise AssertionError("Action limit exhausted")
    summary = planner.summary()
    assert len(cleared) == len(sources) and summary["completion_certificate"], summary
    assert not planner.pending_sources
    summary.update({"case_id": case["case_id"], "source_count": len(sources),
                    "wall_time_s": time.perf_counter()-started,
                    "safety_checks": safety_checks, "truth_checks": truth_checks,
                    "certificate_checks": certificate_checks,
                    "service_order": [t["channel"] for t in planner.task_history if t["kind"] == "CLEAR"]})
    if keep_trace:
        summary["trace"] = trace
    return summary


def aggregate(rows):
    averages = np.asarray([r["average_clear_time_s"] for r in rows])
    source_times = [
        value
        for row in rows
        for value in row["source_service_times_s"].values()
    ]
    return {"case_count": len(rows), "source_count": sum(r["source_count"] for r in rows),
            "clear_rate": sum(r["cleared_count"] for r in rows)/sum(r["source_count"] for r in rows),
            "mean_average_clear_time_s": float(averages.mean()),
            "weighted_average_clear_time_s": sum(r["virtual_time_s"] for r in rows)/sum(r["source_count"] for r in rows),
            "p95_average_clear_time_s": float(np.percentile(averages, 95)),
            "max_average_clear_time_s": float(averages.max()),
            "p95_source_service_time_s": float(np.percentile(source_times, 95)),
            "max_source_service_time_s": float(max(source_times)),
            **{key: sum(r[key] for r in rows) for key in
               ("virtual_time_s", "distance_m", "failed_clear_count", "fallback_source_count",
                "fallback_clear_count", "measure_count", "localize_measure_count",
                "opportunistic_revisit_count", "localization_inbound_distance_m",
                "planning_time_s", "wall_time_s",
                "safety_checks", "truth_checks", "certificate_checks")},
            "local_cover_plan_point_counts": [n for r in rows for n in r["local_cover_plan_point_counts"]]}


def configurations(matrix=False):
    # V3 is frozen as the paired baseline; every V4 variant uses the same maps.
    variants = {
        "v3_global": Q3Config(strategy="v3_global"),
        "v4_cooperative": Q3Config(strategy="v4_cooperative"),
    }
    if matrix:
        for field, values in {
            "max_optimized_clear_attempts_per_source": (8, 12, 24),
            "max_extra_measurements_per_source": (3, 4),
            "quality_max_clear_points": (3, 6),
            "opportunistic_min_net_saving_s": (0.0, 5.0),
            "q2_candidate_limit": (12, 24),
        }.items():
            for value in values:
                variants[f"{field}={value}"] = replace(Q3Config(), **{field: value})
    return variants


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=int, default=24)
    parser.add_argument("--seed", type=int, default=20260911)
    parser.add_argument("--matrix", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.cases < 1:
        parser.error("cases must be positive")
    cases = make_cases(args.cases, args.seed)
    variants = configurations(args.matrix)
    rows = {name: [] for name in variants}
    input_hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in Path(__file__).parent.glob("*.py")}
    if args.output is None:
        args.output = Path(__file__).with_name("validation_results_v4")
        if args.matrix:
            args.output = args.output / "matrix"
    args.output.mkdir(parents=True, exist_ok=True)
    for case in cases:
        for name, config in variants.items():
            result = run_case(case, config)
            rows[name].append(result)
            print(f"case={case['case_id']:02d} {name} cleared={result['cleared_count']} "
                  f"average={result['average_clear_time_s']:.3f}s failures={result['failed_clear_count']} "
                  f"fallback={result['fallback_source_count']}", flush=True)
    summaries = {name: aggregate(value) for name, value in rows.items()}
    old, new = summaries["v3_global"], summaries["v4_cooperative"]
    time_metrics = (
        "weighted_average_clear_time_s",
        "p95_source_service_time_s",
        "max_source_service_time_s",
    )
    performance_pass = (
        new["clear_rate"] >= old["clear_rate"]
        and new["failed_clear_count"] < old["failed_clear_count"]
        and new["fallback_source_count"] <= old["fallback_source_count"]
        and any(new[key] < 0.95 * old[key] for key in time_metrics)
        and all(new[key] <= 1.10 * old[key] for key in time_metrics)
    )
    result = {"algorithm_version": 4, "python": platform.python_version(),
              "scope": "paired synthetic offline worlds; not official simulator runs",
              "baseline": "v3_global with current shared geometry fixes",
              "cases": cases, "configs": {k: asdict(v) for k, v in variants.items()},
              "input_sha256": input_hashes,
              "rows": rows, "aggregate": summaries,
              "performance_acceptance": performance_pass,
              "performance_thresholds": "one of weighted/P95-source/max-source time improves >=5%; the others regress <=10%; fewer failures; no increased fallback",
              "correctness_acceptance": True}
    (args.output/"paired_results.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"aggregate": {k: {f: v for f, v in a.items() if f != "local_cover_plan_point_counts"}
                                   for k, a in summaries.items()},
                      "performance_acceptance": performance_pass}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
