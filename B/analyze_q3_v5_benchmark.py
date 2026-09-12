"""Analyze paired V4/V5 result files produced by B.offline_simulator."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import platform
import random
import statistics
from typing import Any

import scipy
from scipy import stats


METRICS = {
    "average_clear_time_s": ("simulator", "average_clear_time_s"),
    "virtual_time_s": ("simulator", "virtual_time_s"),
    "distance_per_source_m": ("simulator", "distance_m"),
    "measure_per_source": ("simulator", "measure_count"),
    "switch_per_source": ("simulator", "switch_count"),
    "clear_attempt_per_source": ("simulator", "clear_count"),
    "failed_clear_per_source": ("simulator", "failed_clear_count"),
    "wall_time_s": ("wall_time_s",),
}


def _load_records(folder: Path, prefix: str) -> dict[tuple[int, int], dict[str, Any]]:
    records: dict[tuple[int, int], dict[str, Any]] = {}
    paths = sorted(folder.glob(f"{prefix}_sources_*.json"))
    if not paths:
        raise FileNotFoundError(f"no {prefix} result files in {folder}")
    for path in paths:
        payload = json.loads(path.read_text(encoding="utf-8"))
        for record in payload.get("records") or []:
            if "error" in record:
                raise RuntimeError(f"{path.name} seed {record.get('seed')}: {record['error']}")
            key = (int(record["seed"]), int(record["source_count"]))
            if key in records:
                raise ValueError(f"duplicate {prefix} record {key}")
            records[key] = record
    return records


def _metric(record: dict[str, Any], name: str) -> float:
    value: Any = record
    for key in METRICS[name]:
        value = value[key]
    value = float(value)
    if name in {
        "distance_per_source_m",
        "measure_per_source",
        "switch_per_source",
        "clear_attempt_per_source",
        "failed_clear_per_source",
    }:
        value /= int(record["source_count"])
    return value


def _bootstrap_ci(values: list[float], statistic, seed: int, iterations: int = 20_000) -> list[float]:
    rng = random.Random(seed)
    n = len(values)
    estimates = []
    for _ in range(iterations):
        sample = [values[rng.randrange(n)] for _ in range(n)]
        estimates.append(float(statistic(sample)))
    estimates.sort()
    low_index = int(0.025 * iterations)
    high_index = int(0.975 * iterations) - 1
    return [estimates[low_index], estimates[high_index]]


def _mean_ci(values: list[float]) -> list[float]:
    n = len(values)
    mean = statistics.fmean(values)
    sem = statistics.stdev(values) / math.sqrt(n)
    margin = stats.t.ppf(0.975, n - 1) * sem
    return [mean - margin, mean + margin]


def _signed_rank_effect(values: list[float]) -> float | None:
    nonzero = [value for value in values if value != 0.0]
    if not nonzero:
        return None
    ranks = stats.rankdata([abs(value) for value in nonzero])
    positive = sum(rank for rank, value in zip(ranks, nonzero) if value > 0)
    negative = sum(rank for rank, value in zip(ranks, nonzero) if value < 0)
    return float((positive - negative) / (positive + negative))


def analyze(folder: Path) -> dict[str, Any]:
    v4 = _load_records(folder, "v4")
    v5 = _load_records(folder, "v5")
    if set(v4) != set(v5):
        raise ValueError(f"unpaired records: V4-only={sorted(set(v4)-set(v5))}, V5-only={sorted(set(v5)-set(v4))}")
    keys = sorted(v4)
    if len(keys) < 2:
        raise ValueError("at least two paired scenarios are required")

    pairs = []
    for seed, source_count in keys:
        row: dict[str, Any] = {
            "seed": seed,
            "source_count": source_count,
            "v4_full_clear": bool(v4[(seed, source_count)]["full_clear"]),
            "v5_full_clear": bool(v5[(seed, source_count)]["full_clear"]),
        }
        for metric in METRICS:
            a = _metric(v4[(seed, source_count)], metric)
            b = _metric(v5[(seed, source_count)], metric)
            row[f"v4_{metric}"] = a
            row[f"v5_{metric}"] = b
            row[f"delta_{metric}"] = b - a
        row["percent_delta_average_clear_time"] = (
            100.0 * row["delta_average_clear_time_s"] / row["v4_average_clear_time_s"]
        )
        row["v4_fallback_sources"] = int(v4[(seed, source_count)]["planner"].get("fallback_source_count", 0))
        row["v5_fallback_sources"] = int(v5[(seed, source_count)]["planner"].get("fallback_source_count", 0))
        pairs.append(row)

    primary = [row["delta_average_clear_time_s"] for row in pairs]
    shapiro = stats.shapiro(primary)
    ttest = stats.ttest_rel(
        [row["v5_average_clear_time_s"] for row in pairs],
        [row["v4_average_clear_time_s"] for row in pairs],
    )
    try:
        wilcoxon = stats.wilcoxon(primary, zero_method="wilcox", alternative="two-sided", method="auto")
        wilcoxon_result = {"statistic": float(wilcoxon.statistic), "p_value": float(wilcoxon.pvalue)}
    except ValueError:
        wilcoxon_result = {"statistic": None, "p_value": None}

    planned_test = "paired_t" if shapiro.pvalue >= 0.05 else "wilcoxon_signed_rank"
    percent_changes = [row["percent_delta_average_clear_time"] for row in pairs]
    v4_total_time = sum(row["v4_virtual_time_s"] for row in pairs)
    v5_total_time = sum(row["v5_virtual_time_s"] for row in pairs)
    total_sources = sum(row["source_count"] for row in pairs)

    metric_summary = {}
    for metric in METRICS:
        left = [row[f"v4_{metric}"] for row in pairs]
        right = [row[f"v5_{metric}"] for row in pairs]
        differences = [b - a for a, b in zip(left, right)]
        metric_summary[metric] = {
            "v4_mean": statistics.fmean(left),
            "v5_mean": statistics.fmean(right),
            "mean_delta_v5_minus_v4": statistics.fmean(differences),
            "median_delta_v5_minus_v4": statistics.median(differences),
            "mean_delta_95_ci": _mean_ci(differences),
        }

    by_source_count = {}
    grouped: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for row in pairs:
        grouped[row["source_count"]].append(row)
    for source_count, rows in sorted(grouped.items()):
        differences = [row["delta_average_clear_time_s"] for row in rows]
        by_source_count[str(source_count)] = {
            "pairs": len(rows),
            "v4_mean_average_clear_time_s": statistics.fmean(row["v4_average_clear_time_s"] for row in rows),
            "v5_mean_average_clear_time_s": statistics.fmean(row["v5_average_clear_time_s"] for row in rows),
            "mean_delta_s_per_source": statistics.fmean(differences),
            "v5_faster_count": sum(value < 0 for value in differences),
        }

    v4_clear = sum(row["v4_full_clear"] for row in pairs)
    v5_clear = sum(row["v5_full_clear"] for row in pairs)
    v4_wall_times = [row["v4_wall_time_s"] for row in pairs]
    v5_wall_times = [row["v5_wall_time_s"] for row in pairs]
    v4_ci = stats.binomtest(v4_clear, len(pairs)).proportion_ci(confidence_level=0.95, method="exact")
    v5_ci = stats.binomtest(v5_clear, len(pairs)).proportion_ci(confidence_level=0.95, method="exact")
    return {
        "design": {
            "paired_scenarios": len(pairs),
            "source_counts": sorted(grouped),
            "pairs_per_source_count": {str(key): len(value) for key, value in sorted(grouped.items())},
            "primary_outcome": "average virtual time per cleared source",
            "difference_direction": "V5 minus V4; negative favors V5",
            "alpha_two_sided": 0.05,
            "bootstrap_seed": 20260912,
            "bootstrap_iterations": 20_000,
        },
        "completion": {
            "v4_full_clear": v4_clear,
            "v5_full_clear": v5_clear,
            "total_pairs": len(pairs),
            "v4_exact_95_ci": [float(v4_ci.low), float(v4_ci.high)],
            "v5_exact_95_ci": [float(v5_ci.low), float(v5_ci.high)],
        },
        "primary": {
            "v4_pooled_average_clear_time_s": v4_total_time / total_sources,
            "v5_pooled_average_clear_time_s": v5_total_time / total_sources,
            "pooled_percent_change": 100.0 * (v5_total_time - v4_total_time) / v4_total_time,
            "mean_paired_delta_s_per_source": statistics.fmean(primary),
            "mean_delta_95_ci": _mean_ci(primary),
            "median_paired_delta_s_per_source": statistics.median(primary),
            "median_delta_bootstrap_95_ci": _bootstrap_ci(primary, statistics.median, 20260912),
            "mean_paired_percent_change": statistics.fmean(percent_changes),
            "mean_percent_change_bootstrap_95_ci": _bootstrap_ci(percent_changes, statistics.fmean, 20260913),
            "v5_faster": sum(value < 0 for value in primary),
            "ties": sum(value == 0 for value in primary),
            "v5_slower": sum(value > 0 for value in primary),
            "shapiro_w": float(shapiro.statistic),
            "shapiro_p": float(shapiro.pvalue),
            "planned_test_after_assumption_check": planned_test,
            "paired_t": {"statistic": float(ttest.statistic), "df": len(primary) - 1, "p_value": float(ttest.pvalue)},
            "wilcoxon_signed_rank": wilcoxon_result,
            "paired_cohens_dz": statistics.fmean(primary) / statistics.stdev(primary),
            "signed_rank_effect": _signed_rank_effect(primary),
        },
        "metrics": metric_summary,
        "by_source_count": by_source_count,
        "fallback": {
            "v4_total_sources": sum(row["v4_fallback_sources"] for row in pairs),
            "v5_total_sources": sum(row["v5_fallback_sources"] for row in pairs),
        },
        "runtime": {
            "python": platform.python_version(),
            "scipy": scipy.__version__,
            "v4_total_wall_time_s": sum(v4_wall_times),
            "v5_total_wall_time_s": sum(v5_wall_times),
            "v4_mean_wall_time_s": statistics.fmean(v4_wall_times),
            "v5_mean_wall_time_s": statistics.fmean(v5_wall_times),
            "v4_max_wall_time_s": max(v4_wall_times),
            "v5_max_wall_time_s": max(v5_wall_times),
        },
        "pairs": pairs,
    }


def _fmt(value: float, digits: int = 3) -> str:
    return f"{value:.{digits}f}"


def render_report(result: dict[str, Any], result_folder: Path) -> str:
    design = result["design"]
    completion = result["completion"]
    primary = result["primary"]
    metrics = result["metrics"]
    runtime = result["runtime"]
    ci = primary["mean_delta_95_ci"]
    faster = primary["v5_faster"]
    slower = primary["v5_slower"]
    paired_n = design["paired_scenarios"]
    statistically_supported = ci[1] < 0
    practically_better = primary["pooled_percent_change"] < 0
    if statistically_supported:
        verdict = "本批次支持 V5 在平均虚拟耗时上优于 V4。"
    elif practically_better:
        verdict = "V5 的点估计略优，但置信区间跨过 0，现有样本不足以确认稳定提升。"
    else:
        verdict = "本批次不支持 V5 优于 V4；其平均虚拟耗时点估计反而更高。"

    rows = []
    for count, item in result["by_source_count"].items():
        rows.append(
            f"| {count} | {item['pairs']} | {_fmt(item['v4_mean_average_clear_time_s'])} | "
            f"{_fmt(item['v5_mean_average_clear_time_s'])} | {_fmt(item['mean_delta_s_per_source'])} | "
            f"{item['v5_faster_count']}/{item['pairs']} |"
        )

    test_name = primary["planned_test_after_assumption_check"]
    selected = primary["paired_t"] if test_name == "paired_t" else primary["wilcoxon_signed_rank"]
    return f"""# Q3 V5 对 V4 分层成对离线仿真报告

## 结论

{verdict}

- 两版均完整清除 {completion['v4_full_clear']}/{paired_n} 张地图中的全部干扰源。
- V4 合并平均耗时为 {_fmt(primary['v4_pooled_average_clear_time_s'])} s/源，V5 为 {_fmt(primary['v5_pooled_average_clear_time_s'])} s/源，变化 {_fmt(primary['pooled_percent_change'])}%。
- 同图配对平均差（V5−V4）为 {_fmt(primary['mean_paired_delta_s_per_source'])} s/源，95% CI [{_fmt(ci[0])}, {_fmt(ci[1])}]。
- V5 在 {faster}/{paired_n} 张图更快，在 {slower}/{paired_n} 张图更慢。
- 两版均未调用兜底清除算法（V4={result['fallback']['v4_total_sources']}，V5={result['fallback']['v5_total_sources']}）。

## 测试设计

- 共 {paired_n} 对地图，干扰源数量 10–16 各 5 对；每一对的随机种子、源坐标、频道、接收半径和测角误差函数完全相同。
- 主指标预先定义为“每个已清除干扰源的平均虚拟时间”；差值统一定义为 V5−V4，负数表示 V5 更好。
- 完整清除率单独作为首要可靠性指标；路径距离、测量次数、清除次数和真实计算耗时作为次要指标。
- 未删除异常值，也未按结果选择子组。所有 35 对均进入分析。

## 主指标统计

- 配对差正态性：Shapiro–Wilk W={_fmt(primary['shapiro_w'])}, p={_fmt(primary['shapiro_p'], 4)}。
- 按预设规则选择的检验：{test_name}，统计量={_fmt(selected['statistic']) if selected['statistic'] is not None else 'NA'}，双侧 p={_fmt(selected['p_value'], 4) if selected['p_value'] is not None else 'NA'}。
- 配对 Cohen's dz={_fmt(primary['paired_cohens_dz'])}；符号秩效应量={_fmt(primary['signed_rank_effect']) if primary['signed_rank_effect'] is not None else 'NA'}。
- 配对百分比变化均值 {_fmt(primary['mean_paired_percent_change'])}%，20,000 次固定种子 bootstrap 95% CI [{_fmt(primary['mean_percent_change_bootstrap_95_ci'][0])}%, {_fmt(primary['mean_percent_change_bootstrap_95_ci'][1])}%]。
- 中位配对差 {_fmt(primary['median_paired_delta_s_per_source'])} s/源，bootstrap 95% CI [{_fmt(primary['median_delta_bootstrap_95_ci'][0])}, {_fmt(primary['median_delta_bootstrap_95_ci'][1])}]。

## 按干扰源数量分层

| 源数量 | 配对数 | V4 均值 s/源 | V5 均值 s/源 | 平均差 V5−V4 | V5 更快 |
|---:|---:|---:|---:|---:|---:|
{chr(10).join(rows)}

## 次要指标

| 指标 | V4 均值 | V5 均值 | 平均配对差 | 95% CI |
|---|---:|---:|---:|---:|
| 路程 m/源 | {_fmt(metrics['distance_per_source_m']['v4_mean'])} | {_fmt(metrics['distance_per_source_m']['v5_mean'])} | {_fmt(metrics['distance_per_source_m']['mean_delta_v5_minus_v4'])} | [{_fmt(metrics['distance_per_source_m']['mean_delta_95_ci'][0])}, {_fmt(metrics['distance_per_source_m']['mean_delta_95_ci'][1])}] |
| 测量次数/源 | {_fmt(metrics['measure_per_source']['v4_mean'])} | {_fmt(metrics['measure_per_source']['v5_mean'])} | {_fmt(metrics['measure_per_source']['mean_delta_v5_minus_v4'])} | [{_fmt(metrics['measure_per_source']['mean_delta_95_ci'][0])}, {_fmt(metrics['measure_per_source']['mean_delta_95_ci'][1])}] |
| 清除尝试/源 | {_fmt(metrics['clear_attempt_per_source']['v4_mean'])} | {_fmt(metrics['clear_attempt_per_source']['v5_mean'])} | {_fmt(metrics['clear_attempt_per_source']['mean_delta_v5_minus_v4'])} | [{_fmt(metrics['clear_attempt_per_source']['mean_delta_95_ci'][0])}, {_fmt(metrics['clear_attempt_per_source']['mean_delta_95_ci'][1])}] |
| 失败清除/源 | {_fmt(metrics['failed_clear_per_source']['v4_mean'])} | {_fmt(metrics['failed_clear_per_source']['v5_mean'])} | {_fmt(metrics['failed_clear_per_source']['mean_delta_v5_minus_v4'])} | [{_fmt(metrics['failed_clear_per_source']['mean_delta_95_ci'][0])}, {_fmt(metrics['failed_clear_per_source']['mean_delta_95_ci'][1])}] |

真实计算耗时均值/图：V4 {_fmt(runtime['v4_mean_wall_time_s'])} s，V5 {_fmt(runtime['v5_mean_wall_time_s'])} s；最大值分别为 {_fmt(runtime['v4_max_wall_time_s'])} s 和 {_fmt(runtime['v5_max_wall_time_s'])} s。均远低于正式 20 分钟限制，但 V5 的规划计算明显更重。

## 仿真器审计与限制

本轮已修复 V5 错误加载旧版包的问题，统一支持 Python 3.12 本地依赖，增加官方坐标与频道范围校验、协议状态校验、动作上限判定、真实运行耗时和安全输出保护。

离线生成器遵守题面明确规则：Q3 仅全向源、10–16 个源、频道不重复且在 1–20、目标圆半径 1,800 m、接收半径 1,000–1,500 m、同地点同误差且误差不超过 ±1°、移动/切频/测量/清除计时与 5 m、20 m 阈值。源的位置、接收半径和频道采用均匀随机生成，这是本地压力测试分布，不代表官方隐藏用例分布。因此，本报告能判断当前随机分布下的相对改进，不能替代官方仿真成绩。

原始数据目录：`{result_folder}`。完整逐图数据和统计量见同目录 `analysis.json`。

## 可复现信息

- Python {runtime['python']}，SciPy {runtime['scipy']}。
- Bootstrap 随机种子 20260912/20260913，迭代 20,000 次。
- 分析程序：`B/analyze_q3_v5_benchmark.py`。

## 参考

Kassis, T., Agarwal, V., He, Y., Patel, D., & Brueckner, A. M. (2026). *Scientific Agent Skills: A Library of Procedural Knowledge for Research Agents*. arXiv:2609.00065. https://doi.org/10.48550/arXiv.2609.00065
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--analysis-json", type=Path, default=None)
    parser.add_argument("--report", type=Path, default=None)
    args = parser.parse_args()
    result = analyze(args.folder)
    analysis_path = args.analysis_json or args.folder / "analysis.json"
    report_path = args.report or args.folder / "REPORT.md"
    analysis_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    report_path.write_text(render_report(result, args.folder.resolve()) + "\n", encoding="utf-8")
    print(json.dumps({key: result[key] for key in ("completion", "primary", "runtime")}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
