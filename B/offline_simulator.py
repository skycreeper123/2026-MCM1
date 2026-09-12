"""Offline judge for CUMCM 2026 problem B planners.

This script mimics the official robot API rules closely enough for local
iteration.  It can generate hidden Q3/Q4 scenarios, answer MEASURE/CLEAR
actions, drive the existing planner classes, and summarize batch results.

Examples:
    python -m B.offline_simulator --self-test
    python -m B.offline_simulator --problem q3 --runs 20 --seed 20260912
    python -m B.offline_simulator --problem q4 --runs 20 --seed 20260912 --jsonl tmp/q4_offline.jsonl
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import importlib
import importlib.util
import json
import math
from pathlib import Path
import random
import sys
import time
from typing import Any


TARGET_RADIUS_M = 1800.0
MIN_RECEIVE_RADIUS_M = 1000.0
MAX_RECEIVE_RADIUS_M = 1500.0
CLEAR_RADIUS_M = 20.0
NEAR_RADIUS_M = 5.0
MOVE_SPEED_MPS = 5.0
MEASURE_SECONDS = 5.0
SWITCH_SECONDS = 1.0
CLEAR_SUCCESS_SECONDS = 5.0
CLEAR_FAIL_SECONDS = 3.0
MAX_COORDINATE_ABS_M = 2_000_000.0
SIMULATOR_SCHEMA_VERSION = 2


@dataclass(frozen=True)
class Source:
    channel: int
    x: float
    y: float
    receive_radius_m: float
    kind: str = "omni"
    direction_deg: float | None = None

    @property
    def position(self) -> tuple[float, float]:
        return (self.x, self.y)


@dataclass(frozen=True)
class Scenario:
    problem: str
    seed: int
    sources: tuple[Source, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "problem": self.problem,
            "seed": self.seed,
            "sources": [asdict(source) for source in self.sources],
        }


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _bearing_deg(origin: tuple[float, float], target: tuple[float, float]) -> float:
    return math.degrees(math.atan2(target[1] - origin[1], target[0] - origin[0])) % 360.0


def _sample_disk(rng: random.Random, radius: float) -> tuple[float, float]:
    r = radius * math.sqrt(rng.random())
    angle = rng.random() * 2.0 * math.pi
    return (r * math.cos(angle), r * math.sin(angle))


def generate_scenario(problem: str, seed: int, source_count: int | None = None) -> Scenario:
    """Generate one hidden scenario with unique source channels."""
    if problem not in {"q3", "q4"}:
        raise ValueError("problem must be q3 or q4")
    rng = random.Random(seed)
    count = source_count if source_count is not None else rng.randint(10, 16)
    if not 10 <= count <= 16:
        raise ValueError("source_count must be between 10 and 16 for Q3/Q4")
    channels = rng.sample(range(1, 21), count)
    sources = []
    for channel in channels:
        x, y = _sample_disk(rng, TARGET_RADIUS_M)
        receive_radius = rng.uniform(MIN_RECEIVE_RADIUS_M, MAX_RECEIVE_RADIUS_M)
        if problem == "q4" and rng.random() < 0.5:
            sources.append(Source(
                channel=channel,
                x=x,
                y=y,
                receive_radius_m=receive_radius,
                kind="directional",
                direction_deg=rng.random() * 360.0,
            ))
        else:
            sources.append(Source(
                channel=channel,
                x=x,
                y=y,
                receive_radius_m=receive_radius,
                kind="omni",
            ))
    return Scenario(problem=problem, seed=seed, sources=tuple(sources))


class OfflineSimulator:
    """Official-like local judge with hidden sources and deterministic noise."""

    def __init__(self, scenario: Scenario, angle_error_deg: float = 1.0):
        self.scenario = scenario
        self.angle_error_deg = float(angle_error_deg)
        self.sources_by_channel = {source.channel: source for source in scenario.sources}
        self.cleared_channels: set[int] = set()
        self.position = (0.0, 0.0)
        self.receiver_channel = 1
        self.virtual_time_s = 0.0
        self.distance_m = 0.0
        self.measure_count = 0
        self.switch_count = 0
        self.clear_count = 0
        self.failed_clear_count = 0
        self.entered = False
        self.exited = False

    def enter(self) -> dict[str, Any]:
        if self.entered:
            raise RuntimeError("ENTER may only be called once")
        self.entered = True
        return {
            "accepted": True,
            "real_timestamp_ms": int(time.time() * 1000),
            "virtual_time_s": self.virtual_time_s,
            "max_virtual_duration_s": 360000.0,
            "max_real_duration_s": 1200.0,
            "remaining_real_duration_s": 1200.0,
        }

    def _validate_action(self, position: tuple[float, float], channel: int) -> tuple[tuple[float, float], int]:
        if not self.entered or self.exited:
            raise RuntimeError("MEASURE/CLEAR requires an active ENTER session")
        if len(position) != 2:
            raise ValueError("position must contain exactly two coordinates")
        normalized = (float(position[0]), float(position[1]))
        if not all(math.isfinite(value) for value in normalized):
            raise ValueError("position coordinates must be finite")
        if any(abs(value) > MAX_COORDINATE_ABS_M for value in normalized):
            raise ValueError("position coordinate exceeds the official absolute limit")
        normalized_channel = int(channel)
        if normalized_channel != channel or not 1 <= normalized_channel <= 20:
            raise ValueError("channel must be an integer from 1 to 20")
        return normalized, normalized_channel

    def _move_and_tune(self, position: tuple[float, float], channel: int, measure: bool) -> None:
        travelled = _distance(self.position, position)
        self.distance_m += travelled
        self.virtual_time_s += travelled / MOVE_SPEED_MPS
        self.position = position
        if self.receiver_channel != channel:
            self.receiver_channel = channel
            self.switch_count += 1
            self.virtual_time_s += SWITCH_SECONDS
        if measure:
            self.measure_count += 1
            self.virtual_time_s += MEASURE_SECONDS

    def _can_receive(self, source: Source, position: tuple[float, float]) -> bool:
        if source.channel in self.cleared_channels:
            return False
        if _distance(source.position, position) > source.receive_radius_m + 1e-9:
            return False
        if source.kind != "directional":
            return True
        assert source.direction_deg is not None
        emit = (math.cos(math.radians(source.direction_deg)),
                math.sin(math.radians(source.direction_deg)))
        robot = (position[0] - source.x, position[1] - source.y)
        return emit[0] * robot[0] + emit[1] * robot[1] >= -1e-9

    def _fixed_bearing_error(self, source: Source, position: tuple[float, float]) -> float:
        # Stable for the same source and effectively same measurement point.
        key = (self.scenario.seed, source.channel, round(position[0], 3), round(position[1], 3))
        rng = random.Random(repr(key))
        return rng.uniform(-self.angle_error_deg, self.angle_error_deg)

    def measure(self, position: tuple[float, float], channel: int) -> dict[str, Any]:
        position, channel = self._validate_action(position, channel)
        self._move_and_tune(position, channel, measure=True)
        source = self.sources_by_channel.get(channel)
        if source is None or not self._can_receive(source, position):
            return self._response(measure_result="no_signal")
        if _distance(source.position, position) <= NEAR_RADIUS_M + 1e-9:
            return self._response(measure_result="near")
        bearing = (_bearing_deg(position, source.position)
                   + self._fixed_bearing_error(source, position)) % 360.0
        return self._response(measure_result="direction", svd_deg=bearing)

    def clear(self, position: tuple[float, float], channel: int) -> dict[str, Any]:
        position, channel = self._validate_action(position, channel)
        travelled = _distance(self.position, position)
        self.distance_m += travelled
        self.virtual_time_s += travelled / MOVE_SPEED_MPS
        self.position = position
        self.clear_count += 1
        source = self.sources_by_channel.get(channel)
        if source is not None and source.channel not in self.cleared_channels \
                and _distance(source.position, position) <= CLEAR_RADIUS_M + 1e-9:
            self.cleared_channels.add(source.channel)
            self.virtual_time_s += CLEAR_SUCCESS_SECONDS
            return self._response(clear_result="success")
        self.failed_clear_count += 1
        self.virtual_time_s += CLEAR_FAIL_SECONDS
        return self._response(clear_result="no_target_in_range")

    def exit(self, reason: str = "offline_exit") -> dict[str, Any]:
        if not self.entered or self.exited:
            raise RuntimeError("EXIT requires an active ENTER session")
        self.exited = True
        return self._response(exit_reason=reason)

    def _response(self, **fields: Any) -> dict[str, Any]:
        return {
            "accepted": True,
            "real_timestamp_ms": int(time.time() * 1000),
            "virtual_time_s": self.virtual_time_s,
            **fields,
        }

    def summary(self) -> dict[str, Any]:
        total = len(self.scenario.sources)
        cleared = len(self.cleared_channels)
        return {
            "simulator_schema_version": SIMULATOR_SCHEMA_VERSION,
            "problem": self.scenario.problem,
            "seed": self.scenario.seed,
            "source_count": total,
            "cleared_count": cleared,
            "clear_ratio": cleared / total if total else 1.0,
            "missed_channels": sorted(set(self.sources_by_channel) - self.cleared_channels),
            "virtual_time_s": self.virtual_time_s,
            "average_clear_time_s": self.virtual_time_s / cleared if cleared else None,
            "distance_m": self.distance_m,
            "measure_count": self.measure_count,
            "switch_count": self.switch_count,
            "clear_count": self.clear_count,
            "failed_clear_count": self.failed_clear_count,
        }


def _load_q3_v5_package():
    """Load the independently packaged B/q3 V5 tree as B.q3."""
    import B

    here = Path(__file__).resolve().parent
    repo = here.parent
    v5_dir = here / "q3 V5"
    init_path = v5_dir / "__init__.py"
    planner_path = (v5_dir / "planner.py").resolve()
    dependencies = repo / ".modeling-deps"
    if dependencies.is_dir() and str(dependencies) not in sys.path:
        sys.path.insert(0, str(dependencies))

    existing = sys.modules.get("B.q3")
    if existing is not None:
        existing_file = getattr(existing, "__file__", None)
        if existing_file and Path(existing_file).resolve() == init_path.resolve():
            return existing
        raise RuntimeError(
            "B.q3 was already loaded from another implementation; "
            "run each Q3 implementation in a separate process"
        )

    spec = importlib.util.spec_from_file_location(
        "B.q3", init_path, submodule_search_locations=[str(v5_dir)]
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("could not construct the Q3 V5 package loader")
    package = importlib.util.module_from_spec(spec)
    sys.modules["B.q3"] = package
    B.q3 = package
    spec.loader.exec_module(package)
    planner = importlib.import_module("B.q3.planner")
    if Path(planner.__file__).resolve() != planner_path:
        raise RuntimeError(f"Q3 V5 loader resolved the wrong planner: {planner.__file__}")
    return package


def _add_compatible_local_dependencies() -> None:
    """Expose the repository's CPython 3.12 dependency bundle when compatible."""
    if sys.version_info[:2] != (3, 12):
        return
    dependencies = Path(__file__).resolve().parent.parent / ".modeling-deps"
    if dependencies.is_dir() and str(dependencies) not in sys.path:
        sys.path.insert(0, str(dependencies))


def _load_planner(
    problem: str,
    strategy: str | None,
    q3_implementation: str = "default",
):
    if problem == "q3":
        _add_compatible_local_dependencies()
        if q3_implementation == "v5":
            _load_q3_v5_package()
        elif q3_implementation != "default":
            raise ValueError("q3_implementation must be default or v5")
        module = importlib.import_module("B.q3.planner")
        config = module.Q3Config(strategy=strategy) if strategy else module.Q3Config()
        return module.Q3Planner(config)
    if problem == "q4":
        module = importlib.import_module("B.q4.planner")
        return module.Q4Planner()
    raise ValueError("problem must be q3 or q4")


def run_planner_offline(
    problem: str,
    scenario: Scenario,
    strategy: str | None = None,
    q3_implementation: str = "default",
    max_actions: int = 6000,
    planning_budget_s: float = 0.5,
    trace_path: Path | None = None,
) -> dict[str, Any]:
    """Drive a repository planner against the local simulator."""
    started_at = time.perf_counter()
    planner = _load_planner(problem, strategy, q3_implementation)
    simulator = OfflineSimulator(scenario)
    simulator.enter()
    trace_file = None
    try:
        if trace_path is not None:
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            trace_file = trace_path.open("a", encoding="utf-8", newline="\n")
        exhausted_actions = True
        for index in range(1, max_actions + 1):
            action = planner.propose_action(time.monotonic() + planning_budget_s)
            if action is None:
                exhausted_actions = False
                break
            row = {"index": index, "action": _action_dict(action)}
            if action.kind == "EXIT":
                response = simulator.exit(getattr(action, "reason", "offline_exit"))
                planner.apply_exit_result(action)
                row["response"] = response
                _write_trace(trace_file, scenario, row, planner, simulator)
                exhausted_actions = False
                break
            if action.kind == "MEASURE":
                response = simulator.measure(action.position, action.channel)
                planner.apply_measure_result(
                    action,
                    response["measure_result"],
                    response.get("svd_deg"),
                )
            elif action.kind == "CLEAR":
                response = simulator.clear(action.position, action.channel)
                planner.apply_clear_result(action, response["clear_result"])
            else:
                raise RuntimeError(f"unsupported action kind {action.kind!r}")
            row["response"] = response
            _write_trace(trace_file, scenario, row, planner, simulator)
        else:
            if hasattr(planner, "mark_time_budget_incomplete"):
                planner.mark_time_budget_incomplete()
        planner_summary = planner.summary() if hasattr(planner, "summary") else {}
        sim_summary = simulator.summary()
        return {
            "problem": problem,
            "seed": scenario.seed,
            "source_count": len(scenario.sources),
            "simulator": sim_summary,
            "planner": planner_summary,
            "full_clear": len(simulator.cleared_channels) == len(scenario.sources),
            "actions_limited": exhausted_actions,
            "wall_time_s": time.perf_counter() - started_at,
            "planner_module": type(planner).__module__,
            "planner_file": str(Path(sys.modules[type(planner).__module__].__file__).resolve()),
        }
    finally:
        if trace_file is not None:
            trace_file.close()


def _action_dict(action: Any) -> dict[str, Any]:
    if hasattr(action, "as_dict"):
        return action.as_dict()
    if hasattr(action, "__dict__"):
        return dict(action.__dict__)
    return {"repr": repr(action)}


def _write_trace(trace_file, scenario: Scenario, row: dict[str, Any], planner: Any, simulator: OfflineSimulator) -> None:
    if trace_file is None:
        return
    trace_file.write(json.dumps({
        "scenario": {"problem": scenario.problem, "seed": scenario.seed},
        **row,
        "simulator_summary": simulator.summary(),
        "planner_summary": planner.summary() if hasattr(planner, "summary") else None,
    }, ensure_ascii=False, separators=(",", ":")))
    trace_file.write("\n")
    trace_file.flush()


def run_batch(args: argparse.Namespace) -> dict[str, Any]:
    records = []
    jsonl_path = Path(args.jsonl) if args.jsonl else None
    if jsonl_path and jsonl_path.exists():
        if not args.force:
            raise FileExistsError(f"trace already exists: {jsonl_path}; pass --force to replace")
        jsonl_path.unlink()
    for offset in range(args.runs):
        seed = args.seed + offset
        scenario = generate_scenario(args.problem, seed, args.sources)
        try:
            record = run_planner_offline(
                args.problem,
                scenario,
                strategy=args.strategy,
                q3_implementation=args.q3_implementation,
                max_actions=args.max_actions,
                planning_budget_s=args.planning_budget_s,
                trace_path=jsonl_path,
            )
        except Exception as exc:  # Keep batch runs useful while tuning planners.
            record = {
                "problem": args.problem,
                "seed": seed,
                "source_count": len(scenario.sources),
                "full_clear": False,
                "error": f"{type(exc).__name__}: {exc}",
            }
        records.append(record)
    cleared = [r.get("simulator", {}).get("cleared_count", 0) for r in records]
    totals = [r.get("source_count", 0) for r in records]
    times = [r.get("simulator", {}).get("virtual_time_s") for r in records
             if r.get("simulator", {}).get("virtual_time_s") is not None]
    full_clear_count = sum(bool(r.get("full_clear")) for r in records)
    total_virtual_time = sum(times)
    total_cleared = sum(cleared)
    average_times = [r.get("simulator", {}).get("average_clear_time_s") for r in records
                     if r.get("simulator", {}).get("average_clear_time_s") is not None]
    wall_times = [r.get("wall_time_s") for r in records if r.get("wall_time_s") is not None]
    return {
        "simulator_schema_version": SIMULATOR_SCHEMA_VERSION,
        "problem": args.problem,
        "runs": args.runs,
        "seed_start": args.seed,
        "strategy": args.strategy,
        "q3_implementation": args.q3_implementation,
        "full_clear_count": full_clear_count,
        "full_clear_rate": full_clear_count / args.runs if args.runs else None,
        "cleared_sources": total_cleared,
        "total_sources": sum(totals),
        "overall_clear_ratio": sum(cleared) / sum(totals) if sum(totals) else None,
        "mean_virtual_time_s": total_virtual_time / len(times) if times else None,
        "pooled_average_clear_time_s": total_virtual_time / total_cleared if total_cleared else None,
        "mean_run_average_clear_time_s": sum(average_times) / len(average_times) if average_times else None,
        "worst_virtual_time_s": max(times) if times else None,
        "total_wall_time_s": sum(wall_times),
        "error_count": sum("error" in r for r in records),
        "actions_limited_count": sum(bool(r.get("actions_limited")) for r in records),
        "errors": [r for r in records if "error" in r],
        "records": records if args.verbose else None,
    }


def self_test() -> None:
    q3 = Scenario("q3", 1, (Source(1, 100.0, 0.0, 1000.0),))
    sim = OfflineSimulator(q3)
    sim.enter()
    assert sim.measure((0.0, 0.0), 1)["measure_result"] == "direction"
    assert sim.clear((100.0, 0.0), 1)["clear_result"] == "success"
    assert sim.measure((0.0, 0.0), 1)["measure_result"] == "no_signal"

    active = OfflineSimulator(q3)
    active.enter()
    try:
        active.measure((MAX_COORDINATE_ABS_M + 1.0, 0.0), 1)
    except ValueError:
        pass
    else:
        raise AssertionError("coordinate bounds must be enforced")

    q4 = Scenario("q4", 2, (Source(2, 0.0, 0.0, 1000.0, "directional", 0.0),))
    sim = OfflineSimulator(q4)
    sim.enter()
    assert sim.measure((500.0, 0.0), 2)["measure_result"] == "direction"
    assert sim.measure((-500.0, 0.0), 2)["measure_result"] == "no_signal"
    assert sim.clear((0.0, 0.0), 2)["clear_result"] == "success"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline local judge for B Q3/Q4 planners.")
    parser.add_argument("--problem", choices=("q3", "q4"), default="q3")
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--seed", type=int, default=20260912)
    parser.add_argument("--sources", type=int, default=None)
    parser.add_argument("--strategy", default=None, help="Q3 strategy name; ignored for Q4.")
    parser.add_argument(
        "--q3-implementation",
        choices=("default", "v5"),
        default="default",
        help="Load B.q3 (default) or the independent B/q3 V5 package.",
    )
    parser.add_argument("--max-actions", type=int, default=6000)
    parser.add_argument("--planning-budget-s", type=float, default=0.5)
    parser.add_argument("--jsonl", default=None, help="Optional action trace path.")
    parser.add_argument("--output", default=None, help="Optional batch-summary JSON path.")
    parser.add_argument("--force", action="store_true", help="Allow replacing --output.")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.self_test:
        self_test()
        print("offline_simulator self-test passed")
        return 0
    summary = run_batch(args)
    rendered = json.dumps(
        {k: v for k, v in summary.items() if v is not None},
        ensure_ascii=False,
        indent=2,
    )
    if args.output:
        output_path = Path(args.output)
        if output_path.exists() and not args.force:
            parser.error(f"output already exists: {output_path}; pass --force to replace")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = output_path.with_name(output_path.name + ".tmp")
        temporary.write_text(rendered + "\n", encoding="utf-8")
        temporary.replace(output_path)
    print(rendered)
    return 0 if not summary["errors"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
