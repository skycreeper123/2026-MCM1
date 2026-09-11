"""Reproducible offline Q2 validation: python -m B.q2.validation --cases 30.

Synthetic experiments, not official simulator results. Independent validation
sources and 21 error levels are never passed to the selection algorithm.
"""
import argparse
import csv
from dataclasses import asdict, replace
import hashlib
from itertools import combinations
import json
import math
from pathlib import Path
import platform
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import scipy
from scipy.spatial import ConvexHull

from .legacy_selection_v2 import Q2Config, choose_second_detection
from .selection import _clip_with_bearing
from B.q1.geometry import minimum_enclosing_circle

ROOT = Path(__file__).resolve().parent
NAMES = ['精度优先（修复版）', '时间折中 λ=1', '安全见证点', '随机安全点', '沿示向前进']
COLORS = ['#176B87', '#DF7B39', '#708D50', '#9075AA', '#A56D64']


def xy(p):
    return np.array([p['x'], p['y']], float)


def reference_update(vertices, station, bearing):
    """Boundary intersection enumeration, independent of production clipping.

    Builds wedge normals directly; ConvexHull recovers the ordered intersection.
    """
    if len(vertices) < 3 or np.linalg.matrix_rank(vertices-vertices[0], tol=1e-8) < 2:
        # A segment requires endpoint constraints too. Its two opposing edge
        # normals alone describe an infinite line, which invalidates the audit.
        distances = np.linalg.norm(vertices[:, None]-vertices[None], axis=2)
        i, j = np.unravel_index(np.argmax(distances), distances.shape)
        a, c = vertices[i], vertices[j]
        direction = (c-a)/distances[i, j] if distances[i, j] > 1e-12 else np.array([1., 0.])
        normal = np.array([-direction[1], direction[0]])
        normals = np.array([-direction, direction, normal, -normal])
        bounds = np.array([-direction@a, direction@c, normal@a, -normal@a])
    else:
        edge = np.roll(vertices, -1, axis=0) - vertices
        normals = np.column_stack((edge[:, 1], -edge[:, 0]))
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        bounds = np.sum(normals * vertices, axis=1)
    lo, hi = np.deg2rad([bearing - 1, bearing + 1])
    wedge = np.array([[np.sin(lo), -np.cos(lo)], [-np.sin(hi), np.cos(hi)]])
    A = np.vstack((normals, wedge))
    b = np.r_[bounds, wedge @ station]
    pairs = np.array(list(combinations(range(len(A)), 2)))
    a, c = A[pairs[:, 0]], A[pairs[:, 1]]
    d, e = b[pairs[:, 0]], b[pairs[:, 1]]
    det = a[:, 0] * c[:, 1] - a[:, 1] * c[:, 0]
    keep = np.abs(det) > 1e-12
    a, c, d, e, det = a[keep], c[keep], d[keep], e[keep], det[keep]
    p = np.column_stack((d*c[:, 1]-a[:, 1]*e, a[:, 0]*e-d*c[:, 0]))/det[:, None]
    p = p[np.all(p @ A.T <= b + 1e-7, axis=1)]
    p = np.unique(np.round(p, 9), axis=0)
    if len(p) >= 3:
        if np.linalg.matrix_rank(p-p[0], tol=1e-8) < 2:
            axis = int(np.argmax(np.ptp(p, axis=0)))
            p = p[[np.argmin(p[:, axis]), np.argmax(p[:, axis])]]
        else:
            p = p[ConvexHull(p).vertices]
    return p, A, b


def reference_circle(p):
    """Enumerate all 1/2/3-point supported disks; no production MEC call."""
    if not len(p):
        raise ValueError('Empty validation intersection')
    origin = p.mean(axis=0)
    q = p-origin
    centers = list(q)
    centers += [(a+b)/2 for a, b in combinations(q, 2)]
    for a, b, c in combinations(q, 3):
        M = 2*np.array([b-a, c-a])
        if abs(np.linalg.det(M)) > 1e-10:
            centers.append(np.linalg.solve(M, [b@b-a@a, c@c-a@a]))
    centers = np.asarray(centers)
    radii = np.linalg.norm(centers[:, None]-q[None], axis=2).max(axis=1)
    k = np.argmin(radii)
    return centers[k]+origin, float(radii[k])


def make_cases(count, rng):
    cases = []
    for i in range(count):
        if i == 0:
            g, s, err = np.array([0., 0.]), np.array([-1000., 0.]), .5
        elif i == 1:
            g, s, err = np.array([1790., 0.]), np.array([1790., -1400.]), -.7
        else:
            angle = rng.uniform(0, 2*np.pi)
            radius = [rng.uniform(0, 600), rng.uniform(600, 1500), rng.uniform(1750, 1800)][i % 3]
            g = radius*np.array([np.cos(angle), np.sin(angle)])
            distance = [5.01, 1000., 1499.9, rng.uniform(10, 1490)][i % 4]
            theta = rng.uniform(0, 2*np.pi)
            s = g-distance*np.array([np.cos(theta), np.sin(theta)])
            err = [-1., 0., 1., rng.uniform(-1, 1)][i % 4]
        bearing = np.rad2deg(np.arctan2(*(g-s)[::-1]))+err
        cases.append({'case': i+1, 'source': g.tolist(), 'first_error_deg': err,
                      'first': {'position': dict(zip(['x', 'y'], s)), 'svd_deg': float(bearing % 360)}})
    return cases


def validation_sources(case, count, rng):
    """Polar rejection sampling of the exact physical region, no optimizer grid."""
    s = xy(case['first']['position'])
    points = [np.array(case['source'])]
    for _ in range(200):
        a = np.deg2rad(case['first']['svd_deg']+rng.uniform(-1, 1, 1000))
        r = np.sqrt(rng.uniform(5**2+1e-6, 1500**2, 1000))
        p = s+r[:, None]*np.column_stack((np.cos(a), np.sin(a)))
        points.extend(p[np.linalg.norm(p, axis=1) <= 1800])
        if len(points) >= count:
            return np.asarray(points[:count])
    raise RuntimeError('Insufficient validation sources; preserve failing case')


def baseline_points(result, case, rng):
    v = np.asarray(result['initial_region']['vertices'])
    safe = result['safe_candidate_region']
    s = xy(case['first']['position'])
    witness = np.array(safe['witness_center'])
    def allowed(p):
        return np.linalg.norm(p-s) > 1e-6 and np.linalg.norm(v-p, axis=1).max() <= 999.9+1e-9
    box = safe['bounding_box']
    random_point = None
    for _ in range(10000):
        p = rng.uniform([box['x_min'], box['y_min']], [box['x_max'], box['y_max']])
        if allowed(p):
            random_point = p
            break
    angle = np.deg2rad(case['first']['svd_deg'])
    ray = np.array([np.cos(angle), np.sin(angle)])
    forward = next((s+d*ray for d in np.arange(1., 2001.) if allowed(s+d*ray)), None)
    return [witness if allowed(witness) else None, random_point, forward]


def evaluate(case, result, point, sources, errors, crosscheck=False):
    v = np.asarray(result['initial_region']['vertices'])
    initial_radius = result['initial_region']['minimum_enclosing_circle']['radius_m']
    rows = []
    for j, g in enumerate(sources):
        distance = float(np.linalg.norm(point-g))
        near = distance <= 5
        for e in ([None] if near else errors):
            row = {'source_index': j, 'error_deg': e, 'near': near, 'distance_m': distance}
            if near:
                row.update(radius_m=None, diameter_m=None, area_m2=None,
                           center_error_m=None, compression=None, truth_violation_m=0.,
                           radius_reference_difference_m=None, vertex_difference_m=None)
            else:
                bearing = np.rad2deg(np.arctan2(*(g-point)[::-1]))+e
                p, A, b = reference_update(v, point, bearing)
                center, radius = reference_circle(p)
                diameter = np.linalg.norm(p[:, None]-p[None], axis=2).max()
                local = p-p[0]
                area = abs(np.sum(local[:, 0]*np.roll(local[:, 1], -1)-local[:, 1]*np.roll(local[:, 0], -1)))/2
                rd, vd = None, None
                if crosscheck and j < 3 and e in (-1., 0., 1.):
                    production = _clip_with_bearing(v, point, bearing, 1.)
                    rd = abs(minimum_enclosing_circle(production)['radius_m']-radius)
                    distances = np.linalg.norm(production[:, None]-p[None], axis=2)
                    vd = max(distances.min(axis=0).max(), distances.min(axis=1).max())
                row.update(radius_m=radius, diameter_m=float(diameter), area_m2=float(area),
                           center_error_m=float(np.linalg.norm(center-g)), compression=1-radius/initial_radius,
                           truth_violation_m=max(0., float(np.max(A@g-b))),
                           radius_reference_difference_m=rd, vertex_difference_m=vd)
            rows.append(row)
    radii = [r['radius_m'] for r in rows if not r['near']]
    return {'point': point.tolist(), 'movement_time_s': float(np.linalg.norm(point-xy(case['first']['position']))/5),
            'safety_margin_m': float(1000-np.linalg.norm(v-point, axis=1).max()),
            'worst_radius_m': max(radii) if radii else None, 'rows': rows}


def dump(path, value):
    def convert(x):
        if isinstance(x, np.ndarray): return x.tolist()
        if isinstance(x, np.generic): return x.item()
        raise TypeError(type(x).__name__)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, default=convert, allow_nan=False), encoding='utf-8')


def save(fig, out, name):
    for ext in ('png', 'svg', 'pdf'):
        fig.savefig(out/f'{name}.{ext}', dpi=300, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def figures(records, sensitivity, weights, out):
    plt.rcParams.update({'font.family': 'Microsoft YaHei', 'axes.unicode_minus': False,
                         'svg.fonttype': 'path', 'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False})
    examples = [r for r in records if r.get('evaluations')][:2]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5), layout='constrained')
    for ax, rec in zip(axes, examples):
        result = rec['result']; v = np.array(result['initial_region']['vertices'])
        box = result['safe_candidate_region']['bounding_box']
        X, Y = np.meshgrid(np.linspace(box['x_min'], box['x_max'], 350), np.linspace(box['y_min'], box['y_max'], 350))
        field = np.max(np.sqrt((X[..., None]-v[:, 0])**2+(Y[..., None]-v[:, 1])**2), axis=-1)
        ax.contourf(X, Y, field, levels=[0, 999.9], colors=['#DDEEDC'])
        ax.contour(X, Y, field, levels=[999.9], colors=['#708D50'], linewidths=1)
        ax.fill(*v.T, color='#ACD5E5', label='初始定位外包', zorder=2)
        s = xy(rec['case']['first']['position'])
        ax.scatter(*s, marker='s', color='#263747', label='首测点', zorder=5)
        for k, ev in enumerate(rec['evaluations']):
            if ev:
                ax.scatter(*ev['point'], s=70 if k == 0 else 30, marker='*' if k == 0 else 'o', color=COLORS[k], label=NAMES[k], zorder=6)
        ax.plot([], [], color='#708D50', lw=8, alpha=.35, label='安全区域（边界数值绘制）')
        ax.set(title=f"案例 {rec['case']['case']}：候选区域与选点", xlabel='x / m', ylabel='y / m', aspect='equal')
        ax.grid(alpha=.15)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=4, fontsize=8)
    save(fig, out, 'q2_01_candidate_region')

    fig, axes = plt.subplots(2, 2, figsize=(12, 9), layout='constrained')
    for column, rec in enumerate(examples):
        v = np.array(rec['result']['initial_region']['vertices'])
        point = np.array(rec['evaluations'][0]['point']); g = np.array(rec['case']['source'])
        bearing = np.rad2deg(np.arctan2(*(g-point)[::-1]))+.37
        p, _, _ = reference_update(v, point, bearing)
        center, radius = reference_circle(p)
        for ax in axes[:, column]:
            ax.fill(*(v-g).T, color='#D7E9EF', label='第一次观测后')
            ax.fill(*(p-g).T, color='#DF7B39', alpha=.7, label='第二次观测后')
            ax.scatter(0, 0, marker='*', color='#243747', s=70, label='合成真实源', zorder=5)
            ax.set(xlabel='相对真源 x / m', ylabel='相对真源 y / m', aspect='equal'); ax.grid(alpha=.15)
        axes[0, column].set_title(f"案例 {rec['case']['case']}：全局区域对比")
        midpoint = (v.min(axis=0)+v.max(axis=0))/2-g
        global_extent = max(np.ptp(v, axis=0))*0.57
        axes[0, column].set_xlim(midpoint[0]-global_extent, midpoint[0]+global_extent)
        axes[0, column].set_ylim(midpoint[1]-global_extent, midpoint[1]+global_extent)
        ax = axes[1, column]
        ax.add_patch(Circle(center-g, radius, fill=False, ls='--', color='#176B87', label='更新最小包围圆'))
        extent = max(radius*1.3, 5)
        ax.set_xlim(center[0]-g[0]-extent, center[0]-g[0]+extent)
        ax.set_ylim(center[1]-g[1]-extent, center[1]-g[1]+extent)
        r1 = rec['result']['initial_region']['minimum_enclosing_circle']['radius_m']
        ax.set_title(f'局部放大：R1={r1:.1f} m → R2={radius:.1f} m；压缩 {1-radius/r1:.1%}')
    handles, labels = axes[1, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=4, fontsize=9)
    save(fig, out, 'q2_02_before_after')

    valid = [r for r in records if r.get('evaluations')]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout='constrained')
    for ax, key, title in zip(axes, ['worst_radius_m', 'movement_time_s'], ['独立测试样本最坏半径 / m', '移动时间 / s']):
        values = [[r['evaluations'][k][key] for r in valid if r['evaluations'][k] and r['evaluations'][k][key] is not None] for k in range(5)]
        ax.boxplot(values, tick_labels=NAMES, showfliers=True)
        ax.tick_params(axis='x', labelrotation=20); ax.set_ylabel(title); ax.grid(axis='y', alpha=.2)
    fig.suptitle('策略对比：每个首次观测案例贡献一个统计值')
    save(fig, out, 'q2_03_strategy_comparison')

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout='constrained')
    axes[0].bar([r['case']['case'] for r in valid], [r['evaluations'][0]['safety_margin_m'] for r in valid], color=COLORS[0])
    axes[0].axhline(.1, color=COLORS[1], ls='--', label='设计裕量 0.1 m')
    axes[0].set(xlabel='案例编号', ylabel='相对 1000 m 的安全裕量 / m'); axes[0].legend()
    axes[0].set_yscale('log')
    axes[0].set_ylim(.05, 2000)
    prediction = [r['result']['selected']['worst_updated_cover_radius_m'] for r in valid]
    measured = [r['evaluations'][0]['worst_radius_m'] for r in valid]
    axes[1].scatter(prediction, measured, color=COLORS[0])
    maximum = max(prediction+measured)*1.05
    axes[1].plot([0, maximum], [0, maximum], '--', color=COLORS[1], label='上界 = 独立评估')
    axes[1].set(xlabel='连续响应半径保守上界 / m', ylabel='独立测试样本最坏半径 / m')
    axes[1].legend(); axes[1].grid(alpha=.2)
    save(fig, out, 'q2_04_reliability')

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout='constrained')
    for case_id in sorted(set(r['case'] for r in sensitivity)):
        group = [r for r in sensitivity if r['case'] == case_id]
        for ax, key in zip(axes, ['worst_radius_m', 'elapsed_s']):
            ax.plot([r['level'] for r in group], [r[key] for r in group], 'o-', label=f'案例 {case_id}')
    axes[0].set(ylabel='独立测试样本最坏半径 / m', xlabel='联合离散精度配置')
    axes[1].set(ylabel='选点计算耗时 / s', xlabel='联合离散精度配置')
    for ax in axes: ax.legend(); ax.grid(alpha=.2)
    fig.suptitle('联合加密敏感性（不能分离单个参数的影响）')
    save(fig, out, 'q2_05_resolution')

    case_ids = sorted(set(r['case'] for r in weights))
    fig, axes = plt.subplots(1, len(case_ids), figsize=(14, 4.8), layout='constrained', squeeze=False)
    for ax, case_id in zip(axes[0], case_ids):
        group = [r for r in weights if r['case'] == case_id]
        ax.plot([r['movement_time_s'] for r in group], [r['worst_radius_m'] for r in group], 'o-', label=f'案例 {case_id}')
        grouped = {}
        for r in group:
            key = (round(r['movement_time_s'], 5), round(r['worst_radius_m'], 5))
            grouped.setdefault(key, []).append(r['weight'])
        for i, (point, values) in enumerate(grouped.items()):
            offset = (-6, -16) if i % 2 == 0 else (5, 8)
            ax.annotate('λ='+','.join(f'{v:g}' for v in values), point, xytext=offset,
                        textcoords='offset points', fontsize=8, ha='right' if i % 2 == 0 else 'left')
        ax.margins(x=.25, y=.2)
        ax.set(xlabel='移动时间 / s', ylabel='独立测试样本最坏半径 / m', title=f'案例 {case_id}')
        ax.grid(alpha=.2)
    fig.suptitle('移动权重与定位精度的权衡（各子图独立刻度）')
    save(fig, out, 'q2_06_weight_tradeoff')


def write_tables(records, out):
    summary = []
    for k, name in enumerate(NAMES):
        evs = [r['evaluations'][k] for r in records if r.get('evaluations') and r['evaluations'][k]]
        worst = [e['worst_radius_m'] for e in evs if e['worst_radius_m'] is not None]
        rows = [row for e in evs for row in e['rows'] if not row['near']]
        summary.append({'策略': name, '有效案例数': len(evs), '案例最坏半径中位数_m': float(np.median(worst)),
                        '案例最坏半径P90_m': float(np.percentile(worst, 90)), '测试最大半径_m': max(worst),
                        '平均半径压缩率': float(np.mean([r['compression'] for r in rows])),
                        '移动时间中位数_s': float(np.median([e['movement_time_s'] for e in evs])),
                        '方向样本半径不超过20m比例': float(np.mean([r['radius_m'] <= 20 for r in rows])),
                        'near源场景比例': sum(sum(r['near'] for r in e['rows']) for e in evs)/sum(len(set(r['source_index'] for r in e['rows'])) for e in evs)})
    all_rows = [row for r in records if r.get('evaluations') for e in r['evaluations'] if e for row in e['rows']]
    checks = [r for r in all_rows if r['radius_reference_difference_m'] is not None]
    safety = [e['safety_margin_m'] for r in records if r.get('evaluations') for e in r['evaluations'] if e]
    times = [r['elapsed_s'] for r in records]
    gaps = [r['evaluations'][0]['worst_radius_m']-r['result']['selected']['worst_updated_cover_radius_m']
            for r in records if r.get('evaluations') and r['evaluations'][0]['worst_radius_m'] is not None]
    reliability = {'首次观测案例数': len(records), '当前策略失败数': sum(r['result']['status'] != 'OK' for r in records),
                   '当前策略超时数': sum(r['result'].get('timed_out', False) for r in records),
                   '安全裕量小于0.1m次数_容差1e-7': sum(x < .1-1e-7 for x in safety),
                   '真源约束违规次数_容差1e-6': sum(r['truth_violation_m'] > 1e-6 for r in all_rows),
                   '独立几何交叉检查次数': len(checks),
                   '最大包围半径计算差_m': max(r['radius_reference_difference_m'] for r in checks),
                   '最大顶点集合差_m': max(r['vertex_difference_m'] for r in checks),
                   '计算耗时中位数_s': float(np.median(times)), '计算耗时P95_s': float(np.percentile(times, 95)),
                   '计算耗时最大值_s': max(times)}
    reliability.update({'独立最坏半径超过上界的案例数_容差1e-6': sum(g > 1e-6 for g in gaps),
                        '独立最坏半径减上界的最大差_m': max(gaps),
                        '上界达到细化容差案例数': sum(r['result']['selected']['response_bound_converged'] for r in records if r['result']['status'] == 'OK'),
                        '安全后备选点案例数': sum(r['result'].get('selection_mode') == 'safe_fallback' for r in records)})
    for name, data in [('strategy_summary', summary), ('reliability_summary', [reliability])]:
        with (out/f'{name}.csv').open('w', encoding='utf-8-sig', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(data[0])); writer.writeheader(); writer.writerows(data)
    lines = ['# Q2 结果验证', '', '本文件及六组 PNG / SVG / PDF 图由 validation.py 自动生成。', '',
             f'本轮 {len(records)} 个合成首次观测案例，每例独立源位置见 metrics.json，第二误差使用 21 个等距取值。',
             '随机安全基线每案例独立抽取一个点。所有策略共享验证场景；真实源仅用于验证，未传给选点算法。', '',
             '|策略|案例最坏半径中位数 / m|P90 / m|移动时间中位数 / s|', '|---|---:|---:|---:|']
    lines += [f"|{r['策略']}|{r['案例最坏半径中位数_m']:.3f}|{r['案例最坏半径P90_m']:.3f}|{r['移动时间中位数_s']:.3f}|" for r in summary]
    lines += ['', '## 正确性与可靠性', '']+[f'- {k}：{v}' for k, v in reliability.items()]
    lines += ['', '## 解释范围', '',
              f'- 独立评估中有 {sum(g > 1e-6 for g in gaps)} 个案例的样本最坏半径超过连续响应上界，评估值减上界的最大差为 {max(gaps):.6f} m；负值表示上界保守。',
              '- 上界来自角度区间外包的包含关系，覆盖连续方向响应；候选点搜索仍是有限近似，并非全局最优。实现使用浮点容差，未采用严格区间算术。',
              '- 这是离线合成实验，不是官方模拟器成绩。',
              '- 每案例的最坏值只覆盖本次独立源样本与误差网格。21点误差网格仍非连续枚举。',
              '- near 单列；无线电几何半径留空，不记为零。半径≤20m比例仅以方向反馈样本为分母。',
              '- 几何验证独立生成第二角域并枚举边界交点，独立枚举支撑圆；初始外包仍来自被测算法，因此不是整个链路完全独立。',
              '- 敏感性图为联合参数加密，使用前3个案例；权重图同样使用前3个案例，不能代表总体最优权重。',
              '- 安全区域边界通过密网格等值线可视化，收信验证直接检查所有外包顶点。',
              '- 默认未设计算预算，零超时不代表硬时限保证。预算为覆盖全调用的协作式预算，超限返回安全后备点；边界与预算回归见 tests 和 repair_validation.py。',
              '- 不能仅凭一次初始区域到更新区域的半径下降判定策略优于基线，请结合策略对照表。']
    lines += ['', '## 检验图', '']
    captions = [
        ('q2_01_candidate_region', '图1 候选区域与第二测点', '绿色区域为保守安全候选区域，蓝色为初始定位外包。两个案例分别展示普通与目标圆边界附近的情况。'),
        ('q2_02_before_after', '图2 第二次检测前后定位区域', '上排为全局视图，下排为局部放大。展示使用生成首次观测的真源，第二示向误差固定为 +0.37°；不是独立测试的最坏场景。'),
        ('q2_03_strategy_comparison', '图3 五种策略的定位效果与移动时间', '每个首次观测案例贡献一个样本最坏半径和一个移动时间。箱体为四分位区间，横线为中位数，须线取1.5倍四分位距范围内的数据，圆点为其外样本。'),
        ('q2_04_reliability', '图4 收信安全裕量与半径上界', '左图纵轴采用对数刻度；右图参考线上方表示独立评估超过保守上界，应视为异常。'),
        ('q2_05_resolution', '图5 联合离散加密敏感性', '粗、默认、细三档同时改变外包边数、候选网格与响应区间上限，完整配置保存在 metrics.json。单独区间加密检验见 repair_validation.py。'),
        ('q2_06_weight_tradeoff', '图6 移动权重与定位精度权衡', 'λ 为移动权重，单位 m/s；三个子图使用独立刻度。相同点对应多个权重时合并标注。'),
    ]
    for filename, title, caption in captions:
        lines += [f'### {title}', '', f'![{title}]({filename}.png)', '', caption, '',
                  f'[SVG 矢量图]({filename}.svg) · [PDF 矢量图]({filename}.pdf)', '']
    lines += ['## 原始结果与复现', '',
              '[策略统计表](strategy_summary.csv) · [可靠性统计表](reliability_summary.csv) · [原始结果与配置](metrics.json) · [运行说明](../VALIDATION_README.md)', '']
    (out/'Q2结果验证.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    return summary, reliability


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=int, default=30)
    parser.add_argument('--sources', type=int, default=16)
    parser.add_argument('--seed', type=int, default=20260911)
    parser.add_argument('--output', type=Path, default=ROOT/'validation_results_v2')
    parser.add_argument('--replot', action='store_true', help='Regenerate figures and tables from saved metrics, without rerunning selection')
    args = parser.parse_args()
    if args.cases < 3 or args.sources < 2: parser.error('cases >= 3 and sources >= 2 required')
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=True)
    if args.replot:
        data = json.loads((out/'metrics.json').read_text(encoding='utf-8'))
        if data['metadata'].get('algorithm_version') != 2:
            parser.error('Use version 2 results; historical version 1 figures must not be relabelled.')
        summary, reliability = write_tables(data['records'], out)
        data.update(strategy_summary=summary, reliability_summary=reliability)
        data['metadata']['render_program_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        dump(out/'metrics.json', data)
        figures(data['records'], data['sensitivity'], data['weights'], out)
        print(f'Regenerated figures and tables in {out}', flush=True)
        return
    rng = np.random.default_rng(args.seed)
    cases = make_cases(args.cases, rng); errors = np.linspace(-1, 1, 21).tolist()
    records = []
    for case in cases:
        start = time.perf_counter(); result = choose_second_detection(case['first'])
        elapsed = time.perf_counter()-start
        rec = {'case': case, 'result': result, 'elapsed_s': elapsed}
        if result['status'] == 'OK':
            sources = validation_sources(case, args.sources, rng)
            pure = choose_second_detection(case['first'], config=Q2Config(movement_weight_m_per_s=1))
            points = [xy(result['selected']['position']), xy(pure['selected']['position']) if pure['status'] == 'OK' else None]
            points += baseline_points(result, case, rng)
            rec.update(validation_sources=sources.tolist(), tradeoff_result=pure,
                       evaluations=[evaluate(case, result, p, sources, errors, crosscheck=(k == 0)) if p is not None else None for k, p in enumerate(points)])
        records.append(rec)
        if case['case'] % 10 == 0 or case['case'] == args.cases:
            dump(out/'checkpoint.json', records)
        print(f"Case {case['case']}/{args.cases}: {result['status']}", flush=True)
    sensitivity, weights = [], []
    configs = [('粗', Q2Config(circle_sides=64, coarse_spacing_m=200, fine_spacing_m=40, max_coarse_candidates=12, max_fine_candidates=12, max_response_intervals=32)),
               ('默认', Q2Config()),
               ('细', Q2Config(circle_sides=256, coarse_spacing_m=50, fine_spacing_m=10, max_coarse_candidates=48, max_fine_candidates=48, max_response_intervals=128))]
    for rec in records[:3]:
        if not rec.get('evaluations'): continue
        case = rec['case']; sources = np.array(rec['validation_sources'])
        for level, config in configs:
            start = time.perf_counter(); r = choose_second_detection(case['first'], config=config); elapsed = time.perf_counter()-start
            if r['status'] != 'OK': raise RuntimeError(f'Sensitivity selection failed: {r}')
            ev = evaluate(case, r, xy(r['selected']['position']), sources, errors)
            sensitivity.append({'case': case['case'], 'level': level, 'config': asdict(config), 'elapsed_s': elapsed,
                                'worst_radius_m': ev['worst_radius_m'], 'result': r, 'evaluation': ev})
        for weight in (0., .25, .5, 1., 2.):
            r = choose_second_detection(case['first'], config=replace(Q2Config(), movement_weight_m_per_s=weight))
            if r['status'] != 'OK': raise RuntimeError(f'Weight selection failed: {r}')
            ev = evaluate(case, r, xy(r['selected']['position']), sources, errors)
            weights.append({'case': case['case'], 'weight': weight, 'movement_time_s': ev['movement_time_s'],
                            'worst_radius_m': ev['worst_radius_m'], 'result': r, 'evaluation': ev})
        print(f"Sensitivity case {case['case']} complete", flush=True)
    summary, reliability = write_tables(records, out)
    metadata = {'algorithm_version': 2, 'seed': args.seed, 'case_count': args.cases, 'sources_per_case': args.sources, 'errors_deg': errors,
                'python': platform.python_version(), 'numpy': np.__version__, 'scipy': scipy.__version__, 'matplotlib': matplotlib.__version__,
                'hashes': {str(p.relative_to(ROOT.parent)): hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__), ROOT/'selection.py', ROOT.parent/'q1'/'geometry.py']}}
    dump(out/'metrics.json', {'metadata': metadata, 'records': records, 'sensitivity': sensitivity, 'weights': weights,
                             'strategy_summary': summary, 'reliability_summary': reliability})
    if (reliability['当前策略失败数'] or reliability['安全裕量小于0.1m次数_容差1e-7']
            or reliability['真源约束违规次数_容差1e-6']
            or reliability['独立最坏半径超过上界的案例数_容差1e-6']):
        raise RuntimeError('Validation failed; evidence saved to metrics.json')
    figures(records, sensitivity, weights, out)
    print(json.dumps(reliability, ensure_ascii=False), flush=True)
    print(f'Saved to {out}', flush=True)


if __name__ == '__main__':
    main()
