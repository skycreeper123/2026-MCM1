"""Compare two offline-simulator Q4 result files on paired seeds."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path
from typing import Any


COUNT_FIELDS = (
    "distance_m",
    "measure_count",
    "switch_count",
    "clear_count",
    "failed_clear_count",
)


def _nearest_rank(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(probability * len(ordered)) - 1)]


def _summary(document: dict[str, Any]) -> dict[str, Any]:
    records = document["records"]
    times = [float(record["simulator"]["virtual_time_s"]) for record in records]
    result: dict[str, Any] = {
        "full_clear_count": document["full_clear_count"],
        "cleared_sources": document["cleared_sources"],
        "total_sources": document["total_sources"],
        "mean_virtual_time_s": document["mean_virtual_time_s"],
        "pooled_average_clear_time_s": document["pooled_average_clear_time_s"],
        "median_virtual_time_s": statistics.median(times),
        "p90_virtual_time_s": _nearest_rank(times, 0.90),
        "p95_virtual_time_s": _nearest_rank(times, 0.95),
        "worst_virtual_time_s": document["worst_virtual_time_s"],
        "total_wall_time_s": document["total_wall_time_s"],
    }
    for field in COUNT_FIELDS:
        result[f"total_{field}"] = sum(
            float(record["simulator"][field]) for record in records
        )
    return result


def compare(baseline: dict[str, Any], candidate: dict[str, Any]) -> dict[str, Any]:
    base_records = {record["seed"]: record for record in baseline["records"]}
    candidate_records = {record["seed"]: record for record in candidate["records"]}
    if base_records.keys() != candidate_records.keys():
        raise ValueError("baseline and candidate must contain the same seeds")

    base_summary = _summary(baseline)
    candidate_summary = _summary(candidate)
    numeric_fields = [
        key
        for key, value in base_summary.items()
        if isinstance(value, (int, float)) and key not in {"full_clear_count", "cleared_sources", "total_sources"}
    ]
    paired = []
    for seed in sorted(base_records):
        base_time = float(base_records[seed]["simulator"]["virtual_time_s"])
        candidate_time = float(candidate_records[seed]["simulator"]["virtual_time_s"])
        paired.append(
            {
                "seed": seed,
                "source_count": base_records[seed]["source_count"],
                "baseline_virtual_time_s": base_time,
                "candidate_virtual_time_s": candidate_time,
                "delta_virtual_time_s": candidate_time - base_time,
            }
        )

    delta = {
        field: candidate_summary[field] - base_summary[field] for field in numeric_fields
    }
    delta_percent = {
        field: 100.0 * delta[field] / base_summary[field]
        for field in numeric_fields
        if base_summary[field] != 0
    }
    rollout_fields = (
        "rollout_searches",
        "rollout_candidates",
        "rollout_sequences",
        "rollout_timeouts",
        "rollout_probes",
        "rollout_probe_hits",
        "rollout_probe_misses",
    )
    rollout_totals = {
        field: sum(
            int(record["planner"]["metrics"].get(field, 0))
            for record in candidate["records"]
        )
        for field in rollout_fields
    }
    rollout_totals["probe_hit_rate"] = (
        rollout_totals["rollout_probe_hits"] / rollout_totals["rollout_probes"]
        if rollout_totals["rollout_probes"]
        else None
    )

    ordered_delta = sorted(paired, key=lambda item: item["delta_virtual_time_s"])
    return {
        "baseline": base_summary,
        "candidate": candidate_summary,
        "candidate_minus_baseline": delta,
        "candidate_minus_baseline_percent": delta_percent,
        "paired_seed_outcomes": {
            "wins": sum(item["delta_virtual_time_s"] < 0 for item in paired),
            "losses": sum(item["delta_virtual_time_s"] > 0 for item in paired),
            "ties": sum(item["delta_virtual_time_s"] == 0 for item in paired),
            "five_largest_improvements": ordered_delta[:5],
            "five_largest_regressions": ordered_delta[-5:][::-1],
        },
        "candidate_rollout_totals": rollout_totals,
        "paired_records": paired,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
    candidate = json.loads(args.candidate.read_text(encoding="utf-8"))
    report = compare(baseline, candidate)
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output is None:
        print(rendered, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")


if __name__ == "__main__":
    main()
