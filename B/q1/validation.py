"""Reproduce Q1 paper figures and numerical evidence: python -m B.q1.validation."""

import hashlib
import json
import math
import platform
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import scipy

from .geometry import bearing_halfplanes, solve_bearings, solve_halfplanes

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "validation_results"


def clip(poly, normal, bound):
    """Independent edge clipping reference; no boundary-intersection enumeration."""
    result = []
    for p, q in zip(poly, poly[1:] + poly[:1]):
        fp, fq = float(normal @ p - bound), float(normal @ q - bound)
        if fp <= 0:
            result.append(p)
        if (fp <= 0) != (fq <= 0):
            result.append(p + (q - p) * fp / (fp - fq))
    return result


def random_validation():
    rng = np.random.default_rng(20260910)
    rows = []
    for index in range(40):
        source = rng.uniform(-500, 500, 2)
        observations = []
        for angle in np.deg2rad([0, 120, 240]):
            p = source + rng.uniform(100, 900) * np.array([math.cos(angle), math.sin(angle)])
            theta = math.degrees(math.atan2(*(source - p)[::-1])) + rng.uniform(-1, 1)
            observations.append({"position": dict(zip(("x", "y"), p)), "svd_deg": theta})
        result = solve_bearings(observations)
        assert result["status"] == "BOUNDED", result
        A, b = bearing_halfplanes(observations)
        poly = [np.array(p, float) for p in [(-10000, -10000), (10000, -10000),
                                            (10000, 10000), (-10000, 10000)]]
        for n, bound in zip(A, b):
            poly = clip(poly, n, bound)
        assert poly and np.max(np.abs(poly)) < 9999, "Reference box is active"
        vertices = np.asarray(result["vertices"])
        reference_diameter = max(float(np.linalg.norm(p - q)) for p in poly for q in poly)
        distances = np.linalg.norm(vertices[:, None] - np.asarray(poly)[None, :], axis=2)
        mismatch = float(max(distances.min(axis=0).max(), distances.min(axis=1).max()))
        error = abs(result["diameter_m"] - reference_diameter)
        residual = float(np.max(A @ source - b))
        assert error < 1e-6 and mismatch < 1e-6 and residual <= 1e-7
        rows.append({"case": index + 1, "source": source.tolist(), "observations": observations,
                     "diameter_m": result["diameter_m"], "reference_diameter_m": reference_diameter,
                     "diameter_error_m": error, "vertex_matching_error_m": mismatch,
                     "source_max_residual_m": residual, "reference_box_inactive": True})
    return rows


def save_figure(fig, name):
    fig.savefig(OUT / (name + ".png"), dpi=300, bbox_inches="tight", facecolor="white")
    fig.savefig(OUT / (name + ".svg"), bbox_inches="tight", facecolor="white")
    plt.close(fig)


def case_figures(cases):
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.6))
    for ax, case, tag in zip(axes, cases, ("A", "B")):
        r = case["result"]
        # Source-relative coordinates preserve distances and keep tick labels short.
        source = np.array([case["input"]["true_source"]["x"], case["input"]["true_source"]["y"]])
        vertices = np.array(r["vertices"]) - source
        pair = np.array(r["diameter_pair"]) - source
        c = np.array(r["diameter_circle"]["center"]) - source
        radius = r["diameter_m"] / 2
        ax.fill(*vertices.T, color="#D8EAF3", ec="#216782", lw=1.8, label="定位区域", zorder=2)
        ax.add_patch(Circle(c, radius, fill=False, color="#C24543", ls="--", lw=1.6, label="直径圆", zorder=3))
        ax.plot(*pair.T, "o-", color="#C24543", lw=2, ms=5, label="最远点对", zorder=4)
        ax.scatter(*c, marker="+", s=65, color="#C24543", zorder=5, label="直径圆心")
        ax.scatter(0, 0, marker="*", s=100, color="#152C3D", zorder=6, label="合成真实源")
        for k, v in enumerate(vertices):
            direction = v - vertices.mean(axis=0)
            offset = direction / max(np.linalg.norm(direction), 1e-15) * 11
            ax.annotate(f"$v_{k+1}$", v, xytext=offset, textcoords="offset points", fontsize=10,
                        ha="center", va="center")
        outside = r["diameter_circle"]["outside_vertex_indices"]
        if outside:
            ax.scatter(*vertices[outside].T, color="#D68012", s=45, zorder=7, label="圆外顶点")
            far = vertices[np.argmax(np.linalg.norm(vertices - c, axis=1))]
            edge = c + (far - c) * radius / np.linalg.norm(far - c)
            ax.plot(*np.array([edge, far]).T, color="#D68012", lw=3, zorder=5)
        extent = max(radius * 1.6, np.max(np.abs(vertices - c)) * 1.35)
        ax.set_xlim(c[0] - extent, c[0] + extent)
        ax.set_ylim(c[1] - extent, c[1] + extent)
        ax.set_aspect("equal")
        ax.set_xlabel("相对真实源的 x 坐标 / m")
        ax.set_ylabel("相对真实源的 y 坐标 / m")
        verdict = "覆盖" if r["diameter_circle_covers"] else "不覆盖"
        gap = r["diameter_circle"]["coverage_gap_m"]
        ax.set_title(f"({tag.lower()}) 案例 {tag}：{verdict}\nD = {r['diameter_m']:.4f} m，g = {gap:.4f} m", pad=14)
        ax.grid(alpha=.18)
    handles, labels = axes[1].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=6, frameon=False, fontsize=9)
    fig.subplots_adjust(bottom=.20, wspace=.25, top=.86)
    save_figure(fig, "q1_diameter_coverage")

    fig, axes = plt.subplots(1, 2, figsize=(11, 5.4))
    for ax, case, tag in zip(axes, cases, ("A", "B")):
        source = np.array([case["input"]["true_source"]["x"], case["input"]["true_source"]["y"]])
        for i, obs in enumerate(case["input"]["observations"]):
            p = np.array([obs["position"]["x"], obs["position"]["y"]])
            length = np.linalg.norm(source - p) * 1.08
            theta = math.radians(obs["svd_deg"])
            directions = np.deg2rad([obs["svd_deg"]-1, obs["svd_deg"]+1])
            wedge = np.vstack([p, p + length*np.array([np.cos(directions), np.sin(directions)]).T])
            ax.fill(*wedge.T, alpha=.13, color="#216782")
            q = p + length*np.array([math.cos(theta), math.sin(theta)])
            ax.plot(*np.array([p, q]).T, color="#216782", alpha=.5, lw=.8)
            ax.scatter(*p, color="#216782", s=22)
            ax.annotate(f"$S_{i+1}$", p, xytext=(5, 6), textcoords="offset points", fontsize=9)
        ax.scatter(*source, marker="*", color="#C24543", s=85)
        ax.set(xlabel="x / m", ylabel="y / m", title=f"案例 {tag}：六测点与 ±1° 角域")
        ax.set_aspect("equal")
        ax.margins(.16)
        ax.grid(alpha=.18)
    fig.tight_layout()
    save_figure(fig, "q1_station_layout")


def main():
    OUT.mkdir(exist_ok=True)
    plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["Microsoft YaHei", "DejaVu Sans"],
                         "axes.unicode_minus": False, "font.size": 10, "svg.fonttype": "path"})
    cases = []
    for filename in ("multi_station_covers.json", "multi_station_not_covers.json"):
        path = ROOT / "examples" / filename
        data = json.loads(path.read_text(encoding="utf-8"))
        result = solve_bearings(data["observations"], data["error_deg"])
        source = np.array([data["true_source"]["x"], data["true_source"]["y"]])
        errors = []
        for obs in data["observations"]:
            p = np.array([obs["position"]["x"], obs["position"]["y"]])
            truth = math.degrees(math.atan2(*(source-p)[::-1]))
            errors.append((obs["svd_deg"] - truth + 180) % 360 - 180)
        A, b = bearing_halfplanes(data["observations"])
        residual = float(np.max(A @ source - b))
        assert max(map(abs, errors)) <= 1 and residual <= 1e-7
        assert result["status"] == "BOUNDED"
        assert result["diameter_circle_covers"] == data["expected_diameter_circle_covers"]
        cases.append({"filename": filename, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                      "input": data, "actual_errors_deg": errors, "source_max_residual_m": residual,
                      "result": result})
    random_rows = random_validation()
    analytic = []
    h = math.sqrt(3)
    for name, A, b, expected, covered in [
        ("rectangle", [[1,0],[-1,0],[0,1],[0,-1]], [4,0,3,0], 5., True),
        ("equilateral_triangle", [[0,-1],[-h,1],[h,1]], [0,0,2*h], 2., False)]:
        result = solve_halfplanes(A,b)
        assert abs(result["diameter_m"] - expected) < 1e-7
        assert result["diameter_circle_covers"] == covered
        analytic.append({"name": name, "expected_diameter_m": expected, "result": result})
    report = {"python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__,
              "matplotlib": matplotlib.__version__, "geometry_sha256": hashlib.sha256((ROOT/'geometry.py').read_bytes()).hexdigest(),
              "seed": 20260910, "tolerance_m": 1e-7, "analytic": analytic, "cases": cases,
              "random_validation": random_rows,
              "max_diameter_error_m": max(r["diameter_error_m"] for r in random_rows),
              "max_vertex_matching_error_m": max(r["vertex_matching_error_m"] for r in random_rows)}
    (OUT / "metrics.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    case_figures(cases)
    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.plot(range(1,41), [r["diameter_error_m"] for r in random_rows], "o-", ms=3, lw=1, label="直径绝对误差")
    ax.plot(range(1,41), [r["vertex_matching_error_m"] for r in random_rows], "s-", ms=3, lw=1, label="双向顶点匹配误差")
    ax.set(xlabel="随机案例编号", ylabel="误差 / m", title="40 组测向案例：与独立逐边裁剪结果比较")
    ax.ticklabel_format(axis="y", style="sci", scilimits=(0,0))
    ax.grid(alpha=.2)
    ax.legend(frameon=False, loc="upper center", bbox_to_anchor=(.5, -.20), ncol=2)
    fig.tight_layout()
    save_figure(fig, "q1_random_validation")
    table = "| 案例 | 测点数 | 顶点数 | D / m | D/2 / m | 最大顶点距离 / m | g / m | 覆盖 |\n|---|---:|---:|---:|---:|---:|---:|---|\n"
    for tag, case in zip(("A", "B"), cases):
        r = case["result"]
        circle = r["diameter_circle"]
        table += f"| {tag} | 6 | {len(r['vertices'])} | {r['diameter_m']:.4f} | {circle['radius_m']:.4f} | {circle['max_vertex_distance_m']:.4f} | {max(0., circle['coverage_gap_m']):.4f} | {'是' if r['diameter_circle_covers'] else '否'} |\n"
    text = r"""# Q1 结果验证与可视化

## 验证数据与实验设置

题目未提供问题一的固定数值观测数据，本文采用满足示向误差约束的合成数据进行验证。设干扰源真实位置为 $G$、检测点为 $S_i$，由真实方位叠加误差构造示向度：

$$
\hat\theta_i=\operatorname{atan2}(G_y-S_{iy},G_x-S_{ix})+e_i,\qquad |e_i|\le 1^\circ.
$$

上式方位角统一换算为度并按 $360^\circ$ 归一化。真实源仅用于核验；定位算法输入为检测点坐标、示向度及统一误差界。定位区域取全部前向角域的交集，不额外裁入目标圆或接收圆。几何比较容差取 $\varepsilon=10^{-7}$ m。

对于得到的顶点集 $V$，记最远点对为 $a,b$，区域直径为 $D=\|a-b\|$，直径圆心为 $o=(a+b)/2$，定义覆盖差值

$$
g=\max_{v\in V}\|v-o\|-\frac D2.
$$

计算中以 $g\le\varepsilon$ 判定覆盖。该判据具有几何依据：圆盘为凸集，包含全部顶点等价于包含其凸包，即整个定位多边形。

## 解析结果及随机交叉验证

首先采用可解析的几何区域检验直径与覆盖计算。对于顶点为 $(0,0),(4,0),(4,3),(0,3)$ 的矩形，程序得到直径 $5$ m，直径圆覆盖全部区域；对于边长为 $2$ m 的等边三角形，程序得到直径 $2$ m，而第三顶点到对应边中点的距离为 $\sqrt3$ m，大于圆半径 $1$ m，因此判定不覆盖。两项结果均与解析值一致。这两个案例用于检验几何子程序，多测点角域构造另由下述合成测向案例验证。

进一步采用固定随机种子 20260910 生成 40 组三测点观测。真实源在 $[-500,500]^2$ m 内均匀采样；检测点沿相对于真实源的 $0^\circ,120^\circ,240^\circ$ 三个方向布设，距离在 $[100,900]$ m 内均匀采样，示向误差在 $[-1^\circ,1^\circ]$ 内均匀采样。该随机分布仅为验证设置，不代表题目中的误差分布假设。

以独立实现的逐边多边形裁剪算法作为参照，比较所得直径与顶点集。参照算法从 $[-10000,10000]^2$ m 的方框开始裁剪，并逐例确认最终顶点均严格位于框内，故该方框不改变参照区域。主算法不使用此方框。顶点误差采用两集合间双向最近邻距离的最大值，以避免顶点编号不同的影响。两实现共用角域到半平面的转换，故该比较检验求交与直径计算；已知真源的约束核验补充检查模型一致性。

"""
    text += f"40 组案例均得到有界区域，且全部保留真实源。最大直径绝对差为 **{report['max_diameter_error_m']:.6e} m**，最大双向顶点匹配误差为 **{report['max_vertex_matching_error_m']:.6e} m**，均小于预设交叉验证阈值 $10^{{-6}}$ m。误差分布见图 V2。\n\n"
    text += "## 多检测点典型案例\n\n选取两组各含六个检测点的合成观测，真实源均设为 $(250,-150)$ m，分别展示直径圆覆盖与不覆盖的情况。依据文件中保存的坐标与示向度重新核算实际角误差，以计入数据的小数舍入影响，两组误差均满足 ±1° 约束。检测点布局见补充图 V3，局部定位结果见图 V1。\n\n表 V1 六测点定位区域与直径圆覆盖结果\n\n" + table
    text += "\n注：表中长度保留四位小数，案例 A 的覆盖差值在数值容差内为零；完整未舍入结果见配套数据文件。\n\n![图 V1](q1_diameter_coverage.png)\n\n图 V1 六测点定位区域及直径圆覆盖判定。左右分别为覆盖与不覆盖案例；坐标以合成真实源为原点作平移，单位为米，两图均采用等比例坐标轴，但显示范围不同。浅蓝色为定位区域，红色实线为最远点对连线，红色虚线为直径圆，星号为合成真实源，橙色点为圆外顶点。\n\n"
    rb = cases[1]["result"]
    text += f"案例 A 的全部顶点均在直径圆内或圆上，因此该圆覆盖整个定位区域。案例 B 的最大顶点距离超过圆半径 {rb['diameter_circle']['coverage_gap_m']:.4f} m，因而不能覆盖。该六测点反例说明，区域直径描述任意两点间的最大距离，却不能单独保证半径为 $D/2$ 的圆覆盖整个区域，必须进一步检查顶点覆盖条件。若存在任意半径为 $D/2$ 的覆盖圆，它必须同时包含相距 $D$ 的端点 $a,b$，其圆心只能为二者中点，故本反例也排除了通过更换圆心实现同半径覆盖的可能。\n\n"
    text += "解析案例与随机交叉验证支持算法在上述测试范围及数值容差下的正确性。有限样本验证不能替代一般性数学证明，也不构成对任意病态浮点输入的精确认证。\n\n## 补充验证图\n\n![图 V2](q1_random_validation.png)\n\n图 V2 40 组随机测向案例的直径与顶点匹配误差。\n\n![图 V3](q1_station_layout.png)\n\n图 V3 两组六测点合成案例的全局布局。蓝色点为检测点，细线为示向方向，浅色扇形为 ±1° 角域在示意长度内的显示，星号为真实源；扇形显示长度不作为定位约束。\n"
    (OUT / "Q1结果验证.md").write_text(text, encoding="utf-8")
    print(json.dumps({k: report[k] for k in ("max_diameter_error_m", "max_vertex_matching_error_m")}, indent=2))
    print("Artifacts:", OUT)


if __name__ == "__main__":
    main()
