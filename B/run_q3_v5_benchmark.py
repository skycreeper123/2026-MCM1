"""Run a reproducible, paired, source-count-stratified Q3 V4/V5 benchmark."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any


REPO = Path(__file__).resolve().parent.parent
if sys.version_info[:2] == (3, 12):
    dependencies = REPO / ".modeling-deps"
    if dependencies.is_dir():
        sys.path.insert(0, str(dependencies))

from B.analyze_q3_v5_benchmark import analyze, render_report


def _planner_python(explicit: Path | None) -> Path:
    if explicit is not None:
        candidate = explicit.expanduser().resolve()
    elif sys.version_info[:2] == (3, 12):
        candidate = Path(sys.executable).resolve()
    else:
        candidate = (
            Path(os.environ["USERPROFILE"])
            / ".cache"
            / "codex-runtimes"
            / "codex-primary-runtime"
            / "dependencies"
            / "python"
            / "python.exe"
        )
    if not candidate.is_file():
        raise FileNotFoundError(
            "compatible Python 3.12 runtime not found; pass --planner-python PATH"
        )
    version = subprocess.run(
        [str(candidate), "-c", "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if version != "3.12":
        raise RuntimeError(f"planner runtime must be Python 3.12, got {version}: {candidate}")
    return candidate


def _run_variant(
    *,
    python: Path,
    output_dir: Path,
    variant: str,
    implementation: str,
    strategy: str,
    runs_per_count: int,
    seed_base: int,
    force: bool,
) -> list[dict[str, Any]]:
    completed = []
    for source_count in range(10, 17):
        seed_start = seed_base + 100 * source_count
        output = output_dir / f"{variant}_sources_{source_count}.json"
        command = [
            str(python),
            "-m",
            "B.offline_simulator",
            "--problem",
            "q3",
            "--runs",
            str(runs_per_count),
            "--seed",
            str(seed_start),
            "--sources",
            str(source_count),
            "--strategy",
            strategy,
            "--q3-implementation",
            implementation,
            "--verbose",
            "--output",
            str(output),
        ]
        if force:
            command.append("--force")
        print(f"[{variant}] sources={source_count}, seeds={seed_start}..{seed_start + runs_per_count - 1}", flush=True)
        result = subprocess.run(
            command,
            cwd=REPO,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        if result.returncode:
            raise RuntimeError(
                f"{variant} sources={source_count} failed with code {result.returncode}:\n{result.stderr}"
            )
        completed.append(
            {
                "source_count": source_count,
                "seed_start": seed_start,
                "runs": runs_per_count,
                "result": str(output.resolve()),
            }
        )
    return completed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs-per-count", type=int, default=5)
    parser.add_argument("--seed-base", type=int, default=20262000)
    parser.add_argument("--planner-python", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if args.runs_per_count < 1:
        parser.error("--runs-per-count must be at least 1")

    output_dir = args.output_dir or (
        REPO / "B" / "offline_results" / f"q3_v5_paired_{datetime.now():%Y%m%d_%H%M%S}"
    )
    output_dir = output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()) and not args.force:
        parser.error(f"output directory is not empty: {output_dir}; pass --force to replace matching files")
    output_dir.mkdir(parents=True, exist_ok=True)
    python = _planner_python(args.planner_python)

    jobs = [
        {
            "variant": "v4",
            "implementation": "default",
            "strategy": "v4_cooperative",
        },
        {
            "variant": "v5",
            "implementation": "v5",
            "strategy": "v5_dynamic",
        },
    ]
    print(f"paired scenarios: {7 * args.runs_per_count}; planner runtime: {python}", flush=True)
    with ThreadPoolExecutor(max_workers=2) as executor:
        futures = [
            executor.submit(
                _run_variant,
                python=python,
                output_dir=output_dir,
                runs_per_count=args.runs_per_count,
                seed_base=args.seed_base,
                force=args.force,
                **job,
            )
            for job in jobs
        ]
        completed = [future.result() for future in futures]

    result = analyze(output_dir)
    analysis_path = output_dir / "analysis.json"
    report_path = output_dir / "REPORT.md"
    analysis_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    report_path.write_text(render_report(result, output_dir) + "\n", encoding="utf-8")
    manifest = {
        "runs_per_source_count": args.runs_per_count,
        "source_counts": list(range(10, 17)),
        "seed_base": args.seed_base,
        "planner_python": str(python),
        "completed": completed,
        "analysis": str(analysis_path),
        "report": str(report_path),
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    primary = result["primary"]
    print(
        f"done: V4={primary['v4_pooled_average_clear_time_s']:.3f} s/source, "
        f"V5={primary['v5_pooled_average_clear_time_s']:.3f} s/source, "
        f"change={primary['pooled_percent_change']:+.3f}%"
    )
    print(report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
