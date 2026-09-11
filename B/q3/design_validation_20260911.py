"""Offline evidence for the Q3 design summary; never connects to a simulator.

When the local practice trace is available, reorders its B0 actions without
inventing responses.  It also probes the Q2 V3 public API on synthetic geometry
and checks the covering constants used by Q3.  This is a design check, not an
official simulator benchmark.
Run from project root: python -m B.q3.design_validation_20260911
"""

import hashlib
import json
import math
from pathlib import Path
import platform
import time

import numpy as np

from B.q1.geometry import bearing_halfplanes, minimum_enclosing_circle
from B.q2 import (
    Q2Config,
    choose_second_detection,
    circle_outer_halfplanes,
    clip_convex_polygon,
)


ROOT = Path(__file__).resolve().parents[2]
LOG = ROOT / "B/q3/runs/q3-20260911-013311-3cf0cbb7.jsonl"
OUT = Path(__file__).with_name("Q3_DESIGN_CHECKS_20260911.json")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summarize_actions(actions):
    position, tuning = (0.0, 0.0), 1
    length, measures, switches, failures, successes = 0.0, 0, 0, 0, 0
    for action, succeeded in actions:
        if action["kind"] == "EXIT":
            continue
        length += math.dist(position, action["position"])
        position = action["position"]
        if action["kind"] == "MEASURE":
            measures += 1
            switches += tuning != action["channel"]
            tuning = action["channel"]
        else:
            successes += succeeded
            failures += not succeeded
    total = length / 5 + 5 * measures + switches + 3 * failures + 5 * successes
    return dict(distance_m=length, measure_count=measures, switch_count=switches,
                failed_clear_count=failures, cleared_count=successes,
                virtual_time_s=total, average_clear_time_s=total / successes)


def trace_batch_check():
    # Read only local public action summaries, never simulator hidden state.
    if not LOG.exists():
        return dict(
            status="SKIPPED",
            reason="local practice trace is unavailable",
            is_official_rerun=False,
            uses_hidden_truth=False,
        )
    records = [json.loads(line) for line in LOG.read_text(encoding="utf-8").splitlines()]
    rows = [r for r in records if r.get("event") == "planner_advanced"]
    if not rows:
        # JsonlRecorder uses 'kind' or 'event' depending on implementation version.
        rows = [r for r in records if "action" in r and "planner_summary" in r]
    original, by_station = [], {}
    previous_successes, station = 0, None
    for row in rows:
        action = row["action"]
        now = row["planner_summary"]["cleared_count"]
        succeeded = now > previous_successes
        previous_successes = now
        pair = (action, succeeded)
        original.append(pair)
        if action["kind"] == "MEASURE":
            station = action["station_id"]
            assert station is not None
            by_station.setdefault(station, {"measures": [], "clears": []})["measures"].append(pair)
        elif action["kind"] == "CLEAR":
            assert action["reason"] == "FIRST_BEARING_STRIP", "Only non-near trace is supported"
            by_station[station]["clears"].append(pair)
    assert len(rows) == 684 and previous_successes == 13
    reordered = []
    for station in sorted(by_station):
        group = by_station[station]
        # FIFO queue: same clear sequence, just move every station measurement first.
        reordered.extend(group["measures"])
        reordered.extend(group["clears"])
        assert len({pair[0]["channel"] for pair in group["measures"]}) == len(group["measures"])
    before, after = summarize_actions(original), summarize_actions(reordered)
    assert abs(before["virtual_time_s"] - 8134.791492) < 1e-5
    for key in ("measure_count", "failed_clear_count", "cleared_count"):
        assert before[key] == after[key]
    for channel in range(1, 21):
        assert [x for x in original if x[0]["channel"] == channel] == [
            x for x in reordered if x[0]["channel"] == channel]
    return dict(
        method="counterfactual FIFO station batching, same per-channel positions and outcomes",
        assumptions="stationary independent channels; fixed same-position bearings; no near in this trace",
        is_official_rerun=False, uses_hidden_truth=False,
        baseline=before, batch_fifo=after,
        saving_s=before["virtual_time_s"] - after["virtual_time_s"],
        saving_fraction=1 - after["virtual_time_s"] / before["virtual_time_s"],
    )


def q2_checks(movement_weight):
    first = {"position": {"x": 0.0, "y": 0.0}, "svd_deg": 0.0}
    cfg = Q2Config(calculation_time_limit_s=5.0, movement_weight_m_per_s=movement_weight)
    start = time.perf_counter()
    result = choose_second_detection(first, error_deg=1.005, config=cfg)
    elapsed = time.perf_counter() - start
    assert result["status"] == "OK", result["status"]
    assert result["algorithm_version"] == 3
    selected = result["selected"]["position"]
    point = np.array([selected["x"], selected["y"]])
    vertices = np.array(result["initial_region"]["vertices"])
    dmax = float(np.max(np.linalg.norm(vertices - point, axis=1)))
    assert dmax < 1000
    # A public stress set, not a random estimate or a continuous minimax certificate.
    radii = []
    worst = None
    for distance in (10, 100, 250, 500, 750, 1000, 1250, 1500):
        for source_angle in (-1.005, 0, 1.005):
            phi = math.radians(source_angle)
            source = distance * np.array([math.cos(phi), math.sin(phi)])
            for error in (-1.005, 0, 1.005):
                relative = source - point
                if np.linalg.norm(relative) <= 5:
                    radius = 0.0
                else:
                    bearing = math.degrees(math.atan2(relative[1], relative[0])) + error
                    A, b = bearing_halfplanes([{"position": selected, "svd_deg": bearing}], 1.005)
                    updated = clip_convex_polygon(vertices, A, b)
                    A_receive, b_receive = circle_outer_halfplanes(point, 1500, 128)
                    updated = clip_convex_polygon(updated, A_receive, b_receive)
                    assert len(updated)
                    radius = minimum_enclosing_circle(updated)["radius_m"]
                radii.append(radius)
                if worst is None or radius > worst["radius_m"]:
                    worst = dict(source_range_m=distance, source_angle_deg=source_angle,
                                 second_error_deg=error, radius_m=radius)
    upper = result["selected"]["worst_updated_cover_radius_m"]
    assert max(radii) <= upper + 1e-6
    return dict(first_observation=first, error_deg=1.005,
                algorithm_version=result["algorithm_version"],
                movement_weight_m_per_s=movement_weight, selection_elapsed_s=elapsed,
                selection_mode=result["selection_mode"],
                selected=result["selected"], evaluated_candidates=result["evaluated_candidate_count"],
                timed_out=result["timed_out"], vertex_distance_max_m=dmax,
                public_cases=len(radii), direct_clear_cases=sum(r <= 19.9 for r in radii),
                largest_radius_case=worst,
                limitation="finite conservative 1.005-degree scenarios, not simulator error distribution or official-case performance; second reception outer disk included")


def saved_v3_evidence_audit():
    evidence_dir = ROOT / "B/q2/validation_results_v3"
    acceptance_path = evidence_dir / "acceptance_summary.json"
    audit_path = evidence_dir / "saved_evidence_audit.json"
    acceptance = json.loads(acceptance_path.read_text(encoding="utf-8"))
    audit = json.loads(audit_path.read_text(encoding="utf-8"))
    assert acceptance["failures"] == 0
    assert acceptance["safety_violations"] == 0
    assert acceptance["truth_violations"] == 0
    assert acceptance["upper_bound_violations"] == 0
    assert audit["failures"] == 0
    return dict(
        algorithm_version=3,
        acceptance_sha256=sha(acceptance_path),
        audit_sha256=sha(audit_path),
        random_cases=acceptance["random_cases"],
        boundary_cases=acceptance["boundary_cases"],
        failures=acceptance["failures"],
        safety_violations=acceptance["safety_violations"],
        truth_violations=acceptance["truth_violations"],
        upper_bound_violations=acceptance["upper_bound_violations"],
        independent_search_passed=acceptance["independent_search_passed"],
        saved_audit_failures=audit["failures"],
        limitation="This audits the committed Q2 V3 summaries; q2_public_probes are the current-workspace execution check.",
    )


def cover_checks():
    z = 1500 * math.sin(math.radians(1.005))
    outer_radius = 1500 / math.cos(math.pi / 128)
    outer_z = outer_radius * math.sin(math.radians(1.005))
    outer_strip_bound = math.hypot(max(10, outer_radius - 1500), max(15, outer_z - 15))
    assert outer_strip_bound < 19.9
    bound = math.hypot(1500 / 102, 13.2)
    assert max(13.2, z - 13.2) == 13.2 and bound < 19.9
    per_source = (3000 + 4 * 13.2) / 5 + 103 * 3 + 5
    # The proposed unpruned rectangle grid uses midpoint cells, at most 28m per side.
    grid_bound = math.sqrt(2) * 14
    assert grid_bound < 19.9
    # Example showing that deleting an outside-disk centre loses an inside source.
    source, center = (1799.0, 0.0), (1805.0, 0.0)
    return dict(strip152_outer_polygon=dict(circle_sides=128, outer_radius_m=outer_radius,
                                           cover_radius_bound_m=outer_strip_bound),
                strip104=dict(points=104, lateral_bound_m=z, cover_radius_bound_m=bound,
                             closed_route_m=3052.8, per_source_bound_s=per_source,
                             serial_baseline_bound_s=1380 + 840 + 16 * per_source,
                             serial_baseline_request_bound=2 + 140 + 16 * 104),
                rectangle_grid=dict(max_cell_side_m=28, cover_radius_bound_m=grid_bound),
                bad_pruning_counterexample=dict(source=source, clear_point=center,
                                               distance_m=math.dist(source, center)),
                stage_batch_with_152_fallback=dict(
                    per_source_extra_route_cap_m=4000, max_additional_measures=3,
                    max_optimized_clear_attempts=8,
                    virtual_bound_s=19340 + 16 * (4000 / 5 + 3 * 6 + 8 * 5),
                    request_bound=2574 + 16 * (3 + 8)))


def main():
    tracked_inputs = ["B/q1/geometry.py", "B/q2/selection.py", "B/q3/planner.py",
                      "B/q3/geometry.py", "B/q3/localize.py", "B/q3/runner.py",
                      "B/q3/Q3_OPTIMIZATION_SPEC_V2.md"]
    output = dict(python=platform.python_version(), created_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                  input_sha256={name: sha(ROOT / name) for name in tracked_inputs},
                  source_log_sha256=sha(LOG) if LOG.exists() else None,
                  script_sha256=sha(Path(__file__)),
                  trace_counterfactual=trace_batch_check(),
                  q2_public_probes=[q2_checks(0.0), q2_checks(1.0)],
                  saved_q2_evidence_audit=saved_v3_evidence_audit(),
                  analytic_constants=cover_checks())
    OUT.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(output, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
