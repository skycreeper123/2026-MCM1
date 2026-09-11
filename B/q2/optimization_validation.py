"""Version 3 paired experiments, independent search, and publication figures.

python -m B.q2.optimization_validation --cases 200 --workers 4
All synthetic observations; hidden source positions enter only the validator.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, replace
import csv
import hashlib
import json
import math
from pathlib import Path
import platform
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.tri as mtri
from matplotlib.colors import ListedColormap
import numpy as np
from scipy.stats import wilcoxon

from .selection import (Q2Config, choose_second_detection, response_radius_bound,
                        is_safe_candidate, _grid)
from .legacy_selection_v2 import choose_second_detection as choose_legacy
from .regions import analyze_candidate_region, geometric_features, classify_reception
from .validation import xy, dump, evaluate, baseline_points, save

ROOT = Path(__file__).resolve().parent
SEEDS = [20260911, 20260912, 20260913, 20260914, 20260915]
CHECK_CONFIG = Q2Config(max_response_intervals=256, response_bound_tolerance_m=.05)


def hashes():
    return {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
            for name in ('selection.py', 'regions.py', 'legacy_selection_v2.py', 'optimization_validation.py', 'validation.py')}


def make_stratified_cases(count):
    cases = []
    for k, seed in enumerate(SEEDS):
        rng = np.random.default_rng(seed)
        size = count//5 + (k < count % 5)
        for j in range(size):
            angle, heading = rng.uniform(0, 2*np.pi, 2)
            radius = (rng.uniform(0, 600), rng.uniform(600, 1500), rng.uniform(1750, 1800), 1800.)[j % 4]
            distance = (5.01, rng.uniform(10, 999), 1000., rng.uniform(1000, 1490), 1500.)[(j//4+k) % 5]
            error = (-1., -.999999, 0., .999999, 1., rng.uniform(-1, 1))[(j//3+k) % 6]
            g = radius*np.array([np.cos(angle), np.sin(angle)])
            s = g-distance*np.array([np.cos(heading), np.sin(heading)])
            cases.append({'case': len(cases)+1, 'seed': seed, 'stratum': 'random',
                          'source': g.tolist(), 'first_error_deg': error,
                          'first': {'position': dict(zip(('x', 'y'), s.tolist())),
                                    'svd_deg': float((np.degrees(heading)+error) % 360)}})
    # Additional extreme tangencies; separate from the 200 random cases.
    for j, angle in enumerate(np.linspace(0, 2*np.pi, 10, endpoint=False)):
        for distance in (1000., 1500.):
            g = 1800*np.array([np.cos(angle), np.sin(angle)])
            s = g-distance*np.array([-np.sin(angle), np.cos(angle)])
            cases.append({'case': len(cases)+1, 'seed': SEEDS[j % 5], 'stratum': 'tangent',
                          'source': g.tolist(), 'first_error_deg': -1.,
                          'first': {'position': dict(zip(('x', 'y'), s.tolist())),
                                    'svd_deg': float((np.degrees(angle)+89) % 360)}})
    return cases


def independent_sources(case, result, count=12):
    """Uniform triangle sampling in P+, rejected against exact physical disks."""
    rng = np.random.default_rng(case['seed']+7919*case['case'])
    v = np.asarray(result['initial_region']['vertices'])
    first = xy(case['first']['position'])
    points = [np.asarray(case['source'])]
    if len(v) < 3 or result['initial_region']['area_m2'] < 1e-10:
        return np.asarray(points)
    triangles = np.array([[v[0], a, b] for a, b in zip(v[1:-1], v[2:])])
    e1, e2 = triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0]
    areas = np.abs(e1[:, 0]*e2[:, 1]-e1[:, 1]*e2[:, 0])
    for _ in range(20):
        t = triangles[rng.choice(len(triangles), 512, p=areas/areas.sum())]
        a, b = np.sqrt(rng.random(512)), rng.random(512)
        p = (1-a[:, None])*t[:, 0] + (a*(1-b))[:, None]*t[:, 1] + (a*b)[:, None]*t[:, 2]
        d = np.linalg.norm(p-first, axis=1)
        p = p[(np.linalg.norm(p, axis=1) <= 1800+1e-8) & (d <= 1500+1e-8) & (d > 5.)]
        points.extend(p)
        if len(points) >= count: break
    return np.asarray(points[:count])


def compact_result(r):
    return {k: v for k, v in r.items() if k not in ('candidate_scores', 'near_best_candidate_cloud')}


def paired_case(case):
    old = choose_legacy(case['first'])
    new = choose_second_detection(case['first'])
    if old['status'] != 'OK' or new['status'] != 'OK':
        return {'case': case, 'failure': True, 'old': compact_result(old), 'new': compact_result(new)}
    v = np.asarray(new['initial_region']['vertices'])
    sources = independent_sources(case, new)
    row = {'case': case, 'failure': False, 'old': compact_result(old), 'new': compact_result(new),
           'validation_sources': sources.tolist(), 'evaluation': {}}
    for name, r in (('old', old), ('new', new)):
        p = xy(r['selected']['position'])
        precise = response_radius_bound(v, p, config=CHECK_CONFIG)
        # An independent half-plane-intersection enumeration and support-circle
        # implementation evaluate the same sources and 21 errors for both points.
        ev = evaluate(case, new, p, sources, np.linspace(-1, 1, 21).tolist(), crosscheck=True)
        directions = [x for x in ev['rows'] if not x['near']]
        row['evaluation'][name] = {
            'common_precision_bound': precise, 'physical_worst_radius_m': ev['worst_radius_m'],
            'physical_worst_diameter_m': max((x['diameter_m'] for x in directions), default=None),
            'physical_worst_area_m2': max((x['area_m2'] for x in directions), default=None),
            'radius_compression': 1-ev['worst_radius_m']/new['initial_region']['minimum_enclosing_circle']['radius_m'] if directions else None,
            'safety_margin_m': ev['safety_margin_m'],
            'truth_violation_m': max(x['truth_violation_m'] for x in ev['rows']),
            'upper_bound_violation_m': max(0., ev['worst_radius_m']-r['selected']['worst_updated_cover_radius_m']) if directions else 0.,
            'independent_mec_difference_m': max((x['radius_reference_difference_m'] or 0. for x in directions), default=0.),
            'near_responses': sum(x['near'] for x in ev['rows']), 'direction_responses': len(directions),
            'features': geometric_features(p, xy(case['first']['position']), case['first']['svd_deg'], v)}
    row['bound_improvement_m'] = (row['evaluation']['old']['common_precision_bound']['worst_updated_cover_radius_m']
                                  -row['evaluation']['new']['common_precision_bound']['worst_updated_cover_radius_m'])
    row['original_reported_bound_improvement_m'] = old['selected']['worst_updated_cover_radius_m']-new['selected']['worst_updated_cover_radius_m']
    # Fix the point and all other parameters when checking 128 -> 256 stability.
    row['stability_128'] = response_radius_bound(v, xy(new['selected']['position']),
                                                config=replace(CHECK_CONFIG, max_response_intervals=128))
    return row


def run_paired(cases, workers, out):
    records = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(paired_case, c) for c in cases]
        for f in as_completed(futures):
            records.append(f.result())
            if len(records) % 10 == 0:
                print(f'Paired experiments: {len(records)}/{len(cases)}', flush=True)
                dump(out/'paired_checkpoint.json', {'hashes': hashes(), 'records': records})
    records.sort(key=lambda r: r['case']['case'])
    data = {'metadata': {'hashes': hashes(), 'seeds': SEEDS, 'config': asdict(Q2Config()),
                         'common_scoring_config': asdict(CHECK_CONFIG), 'python': platform.python_version(),
                         'independent_errors_deg': np.linspace(-1, 1, 21).tolist(),
                         'requested_sources_per_case': 12, 'synthetic_data': True}, 'records': records}
    dump(out/'paired_results.json', data)
    return data


def independent_search(record, spacing=25.):
    """A separately aligned full grid, without production candidate quotas.

    All feasible grid positions receive a conservative 32-interval screen;
    twelve leaders are rescored with a common 256-interval/0.05m evaluator.
    It is an independent spatial audit, not a global certificate.
    """
    result = record['new']
    v = np.asarray(result['initial_region']['vertices'])
    box = result['safe_candidate_region']['bounding_box']
    points = [p for p in _grid(box, spacing, np.array([box['x_min']+spacing/3, box['y_min']+spacing/3]))
              if is_safe_candidate(p, v, 999.9)]
    # Always audit the production point too, so numerical differences cannot
    # make a worse reference grid look like a better optimum.
    points.append(xy(result['selected']['position']))
    scores = [response_radius_bound(v, p, config=replace(CHECK_CONFIG, max_response_intervals=32))['worst_updated_cover_radius_m'] for p in points]
    leaders = np.argsort(scores)[:12]
    precise = [(response_radius_bound(v, points[i], config=CHECK_CONFIG), points[i]) for i in leaders]
    # Refine the best independent grid locations at 5m spacing (all positions).
    for _, p in sorted(precise, key=lambda x: x[0]['worst_updated_cover_radius_m'])[:3]:
        bounds = dict(zip(('x_min', 'x_max', 'y_min', 'y_max'), (p[0]-spacing, p[0]+spacing, p[1]-spacing, p[1]+spacing)))
        local = [q for q in _grid(bounds, 5., p) if is_safe_candidate(q, v, 999.9)]
        local_scores = [response_radius_bound(v, q, config=replace(CHECK_CONFIG, max_response_intervals=32))['worst_updated_cover_radius_m'] for q in local]
        for i in np.argsort(local_scores)[:3]:
            precise.append((response_radius_bound(v, local[i], config=CHECK_CONFIG), local[i]))
    bound, p = min(precise, key=lambda x: x[0]['worst_updated_cover_radius_m'])
    recommended = record['evaluation']['new']['common_precision_bound']['worst_updated_cover_radius_m']
    difference = max(0., recommended-bound['worst_updated_cover_radius_m'])
    tolerance = max(2., .02*bound['worst_updated_cover_radius_m'])
    return {'case': record['case']['case'], 'grid_spacing_m': spacing, 'grid_count': len(points),
            'reference_point': p.tolist(), 'reference_bound': bound,
            'recommended_common_bound_m': recommended, 'regret_m': difference,
            'acceptance_tolerance_m': tolerance, 'passed': difference <= tolerance}


def run_references(data, workers, out):
    random = [r for r in data['records'] if r['case']['stratum'] == 'random' and not r['failure']]
    # One wide and one narrow case per seed, chosen by INITIAL region radius,
    # before looking at old/new improvement; prevents favorable-case selection.
    chosen = []
    for seed in SEEDS:
        group = sorted([r for r in random if r['case']['seed'] == seed],
                       key=lambda r: r['new']['initial_region']['minimum_enclosing_circle']['radius_m'])
        chosen.extend([group[0], group[-1]])
    refs = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(independent_search, r) for r in chosen]
        for f in as_completed(futures):
            refs.append(f.result())
            print(f"Independent grid checks: {len(refs)}/{len(chosen)}; latest regret={refs[-1]['regret_m']:.4f} m", flush=True)
            dump(out/'independent_search.json', {'records': refs,
                 'paired_input_sha256': hashlib.sha256((out/'paired_results.json').read_bytes()).hexdigest(),
                 'spatial_search': 'full_25m_grid_then_5m_local_grid', 'scoring_config': asdict(CHECK_CONFIG)})
    return refs


def summarize(data, references=None):
    valid = [r for r in data['records'] if not r['failure']]
    random = [r for r in valid if r['case']['stratum'] == 'random']
    delta = np.array([r['bound_improvement_m'] for r in random])
    old = np.array([r['evaluation']['old']['common_precision_bound']['worst_updated_cover_radius_m'] for r in random])
    new = old-delta
    rng = np.random.default_rng(71203)
    boot = np.array([np.mean(rng.choice(delta, len(delta), replace=True)) for _ in range(10000)])
    s = {'random_cases': len(random), 'boundary_cases': len(valid)-len(random),
         'failures': sum(r['failure'] for r in data['records']),
         'common_old_median_radius_m': float(np.median(old)), 'common_new_median_radius_m': float(np.median(new)),
         'mean_paired_improvement_m': float(delta.mean()), 'median_paired_improvement_m': float(np.median(delta)),
         'mean_improvement_bootstrap_95ci_m': np.quantile(boot, [.025, .975]).tolist(),
         'wilcoxon_one_sided_p': float(wilcoxon(delta, alternative='greater').pvalue) if np.any(delta) else 1.,
         'improved_cases': int(np.sum(delta > .05)), 'tied_within_005m_cases': int(np.sum(np.abs(delta) <= .05)),
         'worse_cases': int(np.sum(delta < -.05)), 'largest_regression_m': float(max(0., -delta.min())),
         'old_converged_cases': sum(r['old']['selected']['response_bound_converged'] for r in valid),
         'new_converged_cases': sum(r['new']['selected']['response_bound_converged'] for r in valid),
         'old_max_gap_m': max(r['old']['selected']['response_bound_gap_m'] for r in valid),
         'new_max_gap_m': max(r['new']['selected']['response_bound_gap_m'] for r in valid),
         'old_median_elapsed_s': float(np.median([r['old']['elapsed_s'] for r in random])),
         'new_median_elapsed_s': float(np.median([r['new']['elapsed_s'] for r in random])),
         'safety_violations': sum(r['evaluation']['new']['safety_margin_m'] < .1-1e-7 for r in valid),
         'truth_violations': sum(r['evaluation']['new']['truth_violation_m'] > 1e-6 for r in valid),
         'upper_bound_violations': sum(r['evaluation']['new']['upper_bound_violation_m'] > 1e-6 for r in valid),
         'max_independent_mec_difference_m': max(r['evaluation']['new']['independent_mec_difference_m'] for r in valid),
         'independent_direction_responses': sum(r['evaluation']['new']['direction_responses'] for r in valid),
         'near_responses': sum(r['evaluation']['new']['near_responses'] for r in valid),
         'stability_128_256_max_change_m': max(r['stability_128']['worst_updated_cover_radius_m']-r['evaluation']['new']['common_precision_bound']['worst_updated_cover_radius_m'] for r in valid),
         'per_seed_mean_improvement_m': {str(seed): float(np.mean([r['bound_improvement_m'] for r in random if r['case']['seed'] == seed])) for seed in SEEDS}}
    physical = [r for r in random if all(r['evaluation'][k]['physical_worst_radius_m'] is not None for k in ('old', 'new'))]
    physical_delta = np.array([r['evaluation']['old']['physical_worst_radius_m']-r['evaluation']['new']['physical_worst_radius_m'] for r in physical])
    s.update(physical_paired_cases=len(physical), physical_mean_improvement_m=float(physical_delta.mean()),
             physical_improved_cases=int(np.sum(physical_delta > .05)), physical_worse_cases=int(np.sum(physical_delta < -.05)))
    if references is not None:
        s.update(independent_search_cases=len(references), independent_search_passed=sum(r['passed'] for r in references),
                 independent_search_max_regret_m=max(r['regret_m'] for r in references))
    return s


def assert_acceptance(summary, require_reference=False):
    assert not any(summary[k] for k in ('failures', 'safety_violations', 'truth_violations', 'upper_bound_violations')), summary
    assert summary['mean_improvement_bootstrap_95ci_m'][0] > 0, summary
    assert summary['wilcoxon_one_sided_p'] < .05, summary
    assert all(x > 0 for x in summary['per_seed_mean_improvement_m'].values()), summary
    assert summary['new_max_gap_m'] <= .5+1e-7, summary
    assert summary['max_independent_mec_difference_m'] <= 1e-6, summary
    assert summary['physical_mean_improvement_m'] > 0, summary
    if require_reference:
        assert summary['random_cases'] >= 200 and summary['boundary_cases'] >= 20, summary
        assert summary['independent_search_cases'] >= 10, summary
        assert summary['independent_search_passed'] == summary['independent_search_cases'], summary


def audit_saved_evidence(data, out):
    """Check initial membership independently, including near-only cases."""
    from scipy.optimize import linprog
    from .regions import is_information_candidate
    checked = 0
    for r in data['records']:
        assert not r['failure']
        vertices = np.asarray(r['new']['initial_region']['vertices'])
        origin = vertices[0]
        A = np.vstack(((vertices-origin).T/1000, np.ones(len(vertices))))
        for source in r['validation_sources']:
            point = np.asarray(source)
            fit = linprog(np.zeros(len(vertices)), A_eq=A, b_eq=np.r_[(point-origin)/1000, 1.],
                          bounds=(0, None), method='highs', options={'primal_feasibility_tolerance': 1e-9})
            assert fit.success and np.linalg.norm(fit.x@vertices-point) <= 1e-6, ('Initial source containment', r['case']['case'], source)
            checked += 1
    nodes = 0
    visual = out/'visualization_data.json'
    if visual.exists():
        for item in json.loads(visual.read_text(encoding='utf-8'))['maps'].values():
            vertices = np.asarray(item['result']['initial_region']['vertices'])
            config = Q2Config(**item['result']['config'])
            for p in item['field']['points']:
                p = np.asarray(p)
                safe = (is_information_candidate(p, vertices, xy(item['first']['position']), config)
                        if config.candidate_region_mode == 'information' else is_safe_candidate(p, vertices, 999.9))
                assert safe, ('Unsafe heatmap node', item['name'], p)
                nodes += 1
    audit = {'initial_source_membership_checks': checked, 'safe_heatmap_node_checks': nodes,
             'failures': 0, 'paired_sha256': hashlib.sha256((out/'paired_results.json').read_bytes()).hexdigest()}
    dump(out/'saved_evidence_audit.json', audit)
    return audit


def map_experiment(name, first, mode, spacing):
    result = choose_second_detection(first, config=Q2Config(candidate_region_mode=mode))
    field = analyze_candidate_region(result, first, spacing_m=spacing,
                                     interval_limit=256 if name == 'narrow_fixed' else 128,
                                     tolerance_m=.05 if name == 'narrow_fixed' else .1)
    return {'name': name, 'first': first, 'result': result, 'field': field}


def sensitivity_experiment(parameter, value, first):
    r = choose_second_detection(first, config=replace(Q2Config(), **{parameter: value}))
    precise = response_radius_bound(np.asarray(r['initial_region']['vertices']), xy(r['selected']['position']), config=CHECK_CONFIG)
    return {'parameter': parameter, 'value': value, 'radius_m': precise['worst_updated_cover_radius_m'],
            'movement_time_s': r['selected']['movement_time_s'], 'elapsed_s': r['elapsed_s'],
            'position': r['selected']['position']}


def make_visuals(data, references, out):
    plt.rcParams.update({'font.family': 'Microsoft YaHei', 'axes.unicode_minus': False,
                         'svg.fonttype': 'path', 'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False})
    wide = {'position': {'x': -1000., 'y': 0.}, 'svd_deg': .5}
    narrow = {'position': {'x': 1790., 'y': -1400.}, 'svd_deg': 89.3}
    maps, sensitivity = [], []
    with ProcessPoolExecutor(max_workers=4) as pool:
        tasks = {pool.submit(map_experiment, *args): ('map', args[0]) for args in
                 [('wide_fixed', wide, 'fixed', 30.), ('narrow_fixed', narrow, 'fixed', 40.),
                  ('wide_information', wide, 'information', 30.), ('wide_grid_check', wide, 'fixed', 45.)]}
        for parameter, values in [('coarse_spacing_m', [50., 100.]), ('fine_spacing_m', [10., 20.]), ('circle_sides', [64, 256])]:
            for value in values:
                tasks[pool.submit(sensitivity_experiment, parameter, value, wide)] = ('sensitivity', f'{parameter}={value}')
        for future in as_completed(tasks):
            kind, name = tasks[future]
            item = future.result()
            (maps if kind == 'map' else sensitivity).append(item)
            print(f'Visualization evidence complete: {name}', flush=True)
            dump(out/'visualization_checkpoint.json', {'maps': maps, 'sensitivity': sensitivity})
    maps = {m['name']: m for m in maps}
    base = maps['wide_fixed']['result']
    base_precise = response_radius_bound(np.asarray(base['initial_region']['vertices']), xy(base['selected']['position']), config=CHECK_CONFIG)
    for parameter in ('coarse_spacing_m', 'fine_spacing_m', 'circle_sides'):
        sensitivity.append({'parameter': parameter, 'value': base['config'][parameter],
                            'radius_m': base_precise['worst_updated_cover_radius_m'],
                            'movement_time_s': base['selected']['movement_time_s'], 'elapsed_s': base['elapsed_s'],
                            'position': base['selected']['position']})
    dump(out/'visualization_data.json', {'hashes': hashes(), 'maps': maps, 'sensitivity': sensitivity})
    render_visuals(data, references, maps, sensitivity, out)


def render_visuals(data, references, maps, sensitivity, out):
    plt.rcParams.update({'font.family': 'Microsoft YaHei', 'axes.unicode_minus': False,
                         'svg.fonttype': 'path', 'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    from matplotlib.colors import LogNorm
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.6), layout='constrained')
    for ax, name, title in zip(axes, ['wide_fixed', 'narrow_fixed'], ['大范围初始区域', '边界附近初始区域']):
        item = maps[name]; field, r = item['field'], item['result']
        p, u = np.array(field['points']), np.array(field['radius_upper_m'])
        tri = mtri.Triangulation(*p.T, np.array(field['triangles']))
        heat = ax.tripcolor(tri, u, shading='gouraud', cmap='viridis', norm=LogNorm(vmin=u.min(), vmax=u.max()))
        for component in field['components']:
            boundary = np.array(component['boundary'])
            ax.plot(*boundary.T, color='#E95663', lw=2)
            if component['recommended_point'] is not None:
                ax.scatter(*component['recommended_point'], marker='*', color='white', edgecolor='#233848', s=120, zorder=8)
        v = np.array(r['initial_region']['vertices'])
        ax.plot(*np.vstack((v, v[0])).T, color='white', lw=1.5)
        ax.scatter(*xy(item['first']['position']), color='#172A3A', marker='s', s=30)
        ax.set(title=f'{title}｜5%近优区 {field["good_area_approx_m2"] / 1e4:.2f} 万m²', xlabel='x / m', ylabel='y / m')
        ax.set_aspect('equal')
        fig.colorbar(heat, ax=ax, label='连续响应半径上界 U(s) / m', shrink=.75)
    fig.suptitle('整个保证收信区域的精度分布；红线为数值 5% 近优边界', fontsize=14)
    from matplotlib.lines import Line2D
    fig.legend(handles=[Line2D([], [], marker='s', color='#172A3A', ls='', label='首次测点'),
                        Line2D([], [], marker='*', markerfacecolor='white', markeredgecolor='#233848', ls='', markersize=12, label='各连通区已评分推荐点'),
                        Line2D([], [], color='#89969E', lw=2, label='白线：初始定位外包')],
               loc='outside lower center', ncols=3, frameon=False)
    save(fig, out, 'q2_v3_01_good_regions')

    random = [r for r in data['records'] if not r['failure'] and r['case']['stratum'] == 'random']
    old = np.array([r['evaluation']['old']['common_precision_bound']['worst_updated_cover_radius_m'] for r in random])
    new = np.array([r['evaluation']['new']['common_precision_bound']['worst_updated_cover_radius_m'] for r in random])
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.6), layout='constrained')
    axes[0].scatter(old, new, s=17, color='#176B87', alpha=.7)
    axes[0].plot([0, max(old)*1.05], [0, max(old)*1.05], '--', color='#89969E')
    axes[0].set(xlabel='旧版同精度半径上界 / m', ylabel='新版同精度半径上界 / m', title='200 个配对案例；线下为改善')
    axes[1].hist(old-new, bins=24, color='#176B87', edgecolor='white')
    axes[1].axvline(0, color='#C44E52', lw=1)
    axes[1].set(xlabel='旧版 − 新版 / m', ylabel='案例数', title='测点本身的精度改善')
    seed_means = [np.mean([r['bound_improvement_m'] for r in random if r['case']['seed'] == seed]) for seed in SEEDS]
    axes[2].bar([str(x)[-4:] for x in SEEDS], seed_means, color='#708D50')
    axes[2].axhline(0, color='#89969E', lw=.8)
    axes[2].set(xlabel='随机种子末四位', ylabel='平均配对改善 / m', title='5 个独立随机种子')
    fig.suptitle('统一使用 256 区间、0.05 m 容差复评，排除评分精度不一致的影响', fontsize=13)
    save(fig, out, 'q2_v3_02_paired_improvement')

    valid = [r for r in data['records'] if not r['failure']]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.3), layout='constrained')
    for name, label, color in [('old', '旧版', '#D18B47'), ('new', '新版', '#176B87')]:
        gaps = sorted(r[name]['selected']['response_bound_gap_m'] for r in valid)
        axes[0].plot(gaps, np.arange(1, len(gaps)+1)/len(gaps), label=label, color=color)
    axes[0].axvline(.5, ls='--', color='#C44E52', label='0.5 m 验收线')
    axes[0].set(xlabel='响应半径上下界间隙 / m', ylabel='累计案例比例', title='分级加密收敛性'); axes[0].legend()
    changes = [r['stability_128']['worst_updated_cover_radius_m']-r['evaluation']['new']['common_precision_bound']['worst_updated_cover_radius_m'] for r in valid]
    axes[1].hist(changes, bins=20, color='#176B87', edgecolor='white')
    axes[1].set(xlabel='固定测点：U128 − U256 / m', ylabel='案例数', title='仅改变角度区间上限')
    if references:
        axes[2].bar(np.arange(len(references)), [r['regret_m'] for r in references], color='#708D50')
        axes[2].plot(np.arange(len(references)), [r['acceptance_tolerance_m'] for r in references], '--', color='#C44E52', label='max(2 m, 2%)')
        axes[2].set(xlabel='独立复核案例', ylabel='相对独立搜索的差距 / m', title='全域25 m网格 + 局部5 m网格'); axes[2].legend()
    save(fig, out, 'q2_v3_03_reliability')

    item = maps['wide_fixed']; f, r = item['field'], item['result']; vertices = np.array(r['initial_region']['vertices'])
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.4), layout='constrained')
    angles = np.array([x['probe_min_crossing_angle_deg'] or 0. for x in f['features']])
    lateral = np.array([x['corridor_transverse_distance_m'] for x in f['features']])
    sc = axes[0].scatter(angles, f['radius_upper_m'], c=lateral, cmap='viridis', s=6, alpha=.7)
    axes[0].set(xlabel='源探针最小交会锐角 / °', ylabel='U(s) / m', title='近乎平行的交会放大误差')
    fig.colorbar(sc, ax=axes[0], label='到首次示向中心线距离 / m', shrink=.75)
    chosen_lateral = [r['evaluation']['new']['features']['corridor_transverse_distance_m'] for r in random]
    axes[1].hist(chosen_lateral, bins=18, color='#176B87', edgecolor='white')
    axes[1].set(xlabel='推荐位置横向距离 / m', ylabel='案例数', title='200 例推荐点的侧向特征')
    _, _, forward = baseline_points(r, {'first': item['first']}, np.random.default_rng(103))
    points = [xy(r['selected']['position']), np.array(r['safe_candidate_region']['witness_center']), forward]
    values = [response_radius_bound(vertices, p, config=CHECK_CONFIG)['worst_updated_cover_radius_m'] for p in points]
    axes[2].bar(['侧向推荐', '安全见证点', '沿示向前进'], values, color=['#176B87', '#708D50', '#D18B47'])
    axes[2].set(ylabel='相同精度 U(s) / m', title='典型几何策略对比')
    save(fig, out, 'q2_v3_04_geometry_features')

    fig, axes = plt.subplots(1, 2, figsize=(12.5, 5), layout='constrained')
    first = xy(item['first']['position'])
    xs, ys = np.linspace(first[0]-1800, first[0]+3000, 130), np.linspace(-2400, 2400, 130)
    labels = {'guaranteed_reception': 0, 'uncertain': 1, 'guaranteed_no_signal': 2}
    classes = np.array([[labels[classify_reception(np.array([x, y]), vertices)['classification']] for x in xs] for y in ys])
    axes[0].pcolormesh(xs, ys, classes, cmap=ListedColormap(['#BBD9AD', '#F7E5AE', '#DFDFE2']), shading='nearest', vmin=0, vmax=2)
    from matplotlib.patches import Patch
    axes[0].legend(handles=[Patch(color=c, label=l) for c,l in zip(['#BBD9AD', '#F7E5AE', '#DFDFE2'], ['保证收信', '不确定', '保证无信号'])], loc='lower left')
    axes[0].plot(*np.vstack((vertices, vertices[0])).T, color='#176B87', lw=2)
    axes[0].set(title='安全区外仍可能收信', xlabel='x / m', ylabel='y / m'); axes[0].set_aspect('equal')
    for name, color, label in [('wide_information', '#B7CDE4', '首次收信信息 C1'), ('wide_fixed', '#BBD9AD', '固定 999.9 m')]:
        field = maps[name]['field']; p = np.array(field['points'])
        from scipy.spatial import ConvexHull
        hull = p[ConvexHull(p).vertices]
        axes[1].fill(*hull.T, color=color, label=label, alpha=.8)
        axes[1].scatter(*xy(maps[name]['result']['selected']['position']), marker='*', color='#176B87' if name == 'wide_fixed' else '#D18B47', s=150, zorder=5)
    axes[1].plot(*np.vstack((vertices, vertices[0])).T, color='#176B87', lw=1.5)
    axes[1].set(title='首次收信信息扩展候选区域', xlabel='x / m', ylabel='y / m'); axes[1].set_aspect('equal'); axes[1].legend(loc='lower left')
    save(fig, out, 'q2_v3_05_reception_regions')

    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2), layout='constrained')
    sensitivity_limits = (min(x['radius_m'] for x in sensitivity)-.03, max(x['radius_m'] for x in sensitivity)+.03)
    for ax, parameter, title in zip(axes, ['coarse_spacing_m', 'fine_spacing_m', 'circle_sides'], ['粗网格间距 / m', '细网格间距 / m', '圆外包边数']):
        rows = sorted([x for x in sensitivity if x['parameter'] == parameter], key=lambda x: x['value'])
        ax.plot([x['value'] for x in rows], [x['radius_m'] for x in rows], 'o-', color='#176B87')
        delta = max(x['radius_m'] for x in rows)-min(x['radius_m'] for x in rows)
        ax.set(xlabel=title, ylabel='同精度复评的 U(s) / m', title=f'其余参数固定；变化范围 {delta:.3f} m', ylim=sensitivity_limits)
        ax.ticklabel_format(axis='y', style='plain', useOffset=False)
    fig.suptitle('单因素敏感性：统一纵轴；小于0.05 m的变化不作精度优劣判断', fontsize=13)
    save(fig, out, 'q2_v3_06_sensitivity')


def write_report(data, summary, out):
    columns = ['case', 'seed', 'stratum', 'old_common_radius_m', 'new_common_radius_m', 'improvement_m',
               'old_physical_radius_m', 'new_physical_radius_m', 'new_diameter_m', 'new_area_m2', 'radius_compression',
               'old_elapsed_s', 'new_elapsed_s', 'movement_distance_m', 'movement_time_s',
               'response_gap_m', 'safety_margin_m', 'transverse_distance_m', 'probe_min_angle_deg']
    with (out/'paired_metrics.csv').open('w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=columns); writer.writeheader()
        for r in data['records']:
            if r['failure']: continue
            ev = r['evaluation']['new']; selected = r['new']['selected']
            writer.writerow(dict(zip(columns, [r['case']['case'], r['case']['seed'], r['case']['stratum'],
                r['evaluation']['old']['common_precision_bound']['worst_updated_cover_radius_m'],
                ev['common_precision_bound']['worst_updated_cover_radius_m'], r['bound_improvement_m'],
                r['evaluation']['old']['physical_worst_radius_m'], ev['physical_worst_radius_m'], ev['physical_worst_diameter_m'], ev['physical_worst_area_m2'], ev['radius_compression'],
                r['old']['elapsed_s'], r['new']['elapsed_s'], selected['movement_distance_m'], selected['movement_time_s'],
                selected['response_bound_gap_m'], ev['safety_margin_m'], ev['features']['corridor_transverse_distance_m'], ev['features']['probe_min_crossing_angle_deg']])))
    s = summary
    lines = ['# Q2 按优化建议升级后的验证报告', '',
        '所有案例均为离线合成观测，真实源只传入独立验证器，不参与选点。旧版为优化前冻结的版本2；历史版本1/2产物保留。', '',
        '## 配对检验', '',
        f"200 个分层随机观测（5 个种子）及 20 个附加极限切向边界案例。实际完成 {s['random_cases']} + {s['boundary_cases']} 例。",
        '新旧推荐位置均以同一初始外包、256 个角度区间上限、0.05 m 容差复评。此比较隔离了选点变化与上界加密的影响。', '',
        '| 指标 | 结果 |', '| --- | --- |',
        f"| 旧 / 新半径上界中位数 | {s['common_old_median_radius_m']:.3f} / {s['common_new_median_radius_m']:.3f} m |",
        f"| 配对改善均值 / 中位数 | {s['mean_paired_improvement_m']:.3f} / {s['median_paired_improvement_m']:.3f} m |",
        f"| 平均改善的 bootstrap 95% 区间 | [{s['mean_improvement_bootstrap_95ci_m'][0]:.3f}, {s['mean_improvement_bootstrap_95ci_m'][1]:.3f}] m |",
        f"| 单侧配对 Wilcoxon p 值 | {s['wilcoxon_one_sided_p']:.3g} |",
        f"| 改善 / 持平 / 退步（±0.05 m） | {s['improved_cases']} / {s['tied_within_005m_cases']} / {s['worse_cases']} |",
        f"| 最大退步 | {s['largest_regression_m']:.3f} m |",
        f"| 旧 / 新最大上下界间隙 | {s['old_max_gap_m']:.3f} / {s['new_max_gap_m']:.3f} m |",
        f"| 旧 / 新计算时间中位数 | {s['old_median_elapsed_s']:.3f} / {s['new_median_elapsed_s']:.3f} s |",
        f"| 失败 / 收信安全违规 / 真源遗漏 / 上界违规 | {s['failures']} / {s['safety_violations']} / {s['truth_violations']} / {s['upper_bound_violations']} |",
        f"| 独立 direction 响应检验次数 | {s['independent_direction_responses']} |",
        f"| 两种 MEC 实现的最大差异 | {s['max_independent_mec_difference_m']:.3g} m |", '',
        '逐种子平均改善：' + '；'.join(f'{seed}: {value:.3f} m' for seed, value in s['per_seed_mean_improvement_m'].items()) + '。', '',
        f"独立物理响应复核：双方均有direction统计的 {s['physical_paired_cases']} 个随机案例，采样最坏半径平均下降 {s['physical_mean_improvement_m']:.3f} m；其中 {s['physical_improved_cases']} 例改善、{s['physical_worse_cases']} 例退步，其余在±0.05 m内。near单列，未以0 m补入。该采样最坏值是独立检验，不是连续最坏情形证明。", '',
        '统计区间针对这组分层合成实验，不能外推为所有场景必然改善。提高精度的代价是更多计算时间；CSV 明确保留退步案例。', '',
        '## 独立搜索与稳定性', '']
    if 'independent_search_cases' in s:
        lines += [f"各随机种子预先按初始区域半径选取最窄、最宽各一例，共 {s['independent_search_cases']} 例。独立执行全域25 m网格、前12名精评分及前三处5 m局部网格。通过 {s['independent_search_passed']}/{s['independent_search_cases']}，最大差距 {s['independent_search_max_regret_m']:.3f} m，验收阈值为 max(2 m, 参考值的2%)。", '']
    lines += [f"固定新版推荐点仅改变角度上限，128→256 最大半径变化 {s['stability_128_256_max_change_m']:.3f} m；这是评分稳定性，未冒称位置全局最优。", '',
        '## 连续候选区域与几何特征', '',
        '固定保证收信区是圆盘交；5%近优区域按整个区域的 U(s) 采样构建安全三角网格并提取等值线。边界、面积、形心为数值近似；推荐点逐点评分和收信核验。网格三角形均在凸安全区内，但插值不构成每个内部点的精度证明。阈值相对于采样最小上界，不是已证明的全局最优值。', '',
        '交会角图使用顶点、边中点和中心源探针，报告探针最小锐角，不能解释成连续源集上的严格最小交会角。接近0°或180°时误差放大；侧向点通常更有效，但还受源不确定范围和收信区域限制。', '']
    visual_path = out/'visualization_data.json'
    if visual_path.exists():
        visual = json.loads(visual_path.read_text(encoding='utf-8'))
        maps = visual['maps']
        lines += ['| 案例 | 安全区面积 m² | 5%近优区面积 m² | 连通区数 | 网格 m |', '| --- | ---: | ---: | ---: | ---: |']
        for name, item in maps.items():
            f = item['field']; lines.append(f"| {name} | {f['safe_area_approx_m2']:.1f} | {f['good_area_approx_m2']:.1f} | {len(f['components'])} | {f['spacing_m']:g} |")
            dump(out/f'{name}_candidate_region.json', f)
        lines += ['', '连通区边界坐标、面积、形心和推荐点详见各 `*_candidate_region.json`。形心可能不在非凸近优区内，应采用已经评分的推荐点。', '',
                  '| 保证方案 | 推荐 U(s) m | 移动距离 m | 计算时间 s |', '| --- | ---: | ---: | ---: |']
        for name in ('wide_fixed', 'wide_information'):
            r = maps[name]['result']; p = r['selected']
            lines.append(f"| {name} | {p['worst_updated_cover_radius_m']:.3f} | {p['movement_distance_m']:.1f} | {r['elapsed_s']:.3f} |")
        lines += ['', 'C₁ 使用第一次已经收信的信息以及同一源接收半径不变的假设。固定主方案预留0.1 m裕量；C₁按题面定义使用闭集保证，不额外预留该裕量。C₁须检查多边形顶点、边与1000 m圆交点、圆弧极值点，不能只检查顶点。', '']
        grid = maps['wide_grid_check']['field']; fine = maps['wide_fixed']['field']
        lines += [f"近优区域45→30 m网格复核：面积 {grid['good_area_approx_m2']:.1f}→{fine['good_area_approx_m2']:.1f} m²；采样最小上界 {grid['sampled_min_radius_upper_m']:.3f}→{fine['sampled_min_radius_upper_m']:.3f} m。面积随网格变化，未将小数精度误作空间认证。", '']
    lines += ['## 指标、边界与复现', '',
        '主指标为最小包围圆半径，直接对应半径20 m清除条件。CSV同时报告独立响应的最大直径、最大面积、半径压缩率、移动距离和移动时间；这些最大值可能来自不同响应。D(P)≤2R_MEC(P)，但D≤40 m不充分保证R_MEC≤20 m。near不当作0 m几何半径。', '',
        '敏感性图固定其余参数且统一纵轴。细网格和圆边数引起的变化小于0.05 m评分容差，不能据此断言其中某一参数严格最优；改变圆边数还会改变初始外包集合。', '',
        '初始区域退化为线段时，独立验证器现保留两端约束，避免把有限线段误作无限直线；新增点/线段回归测试。首轮未达验收的诊断结果位于 `diagnostic_first_pass/`，不属于最终通过证据。', '',
        '```powershell', 'python -m unittest discover -s B/q2/tests -v',
        'python -m B.q2.optimization_validation --cases 200 --workers 4',
        'python -m B.q2.optimization_validation --phase check', '```', '',
        '完整参数、输入、源验证样本、种子、代码哈希和结果保存在 `paired_results.json`；逐例指标见 `paired_metrics.csv`，独立搜索见 `independent_search.json`。本机并行运行的耗时用于说明计算成本，不能视为单核实时承诺。', '',
        '## 可视化', '']
    for name, title in [('01_good_regions', '近优区域热力图'), ('02_paired_improvement', '配对精度改善'), ('03_reliability', '收敛性与独立搜索'), ('04_geometry_features', '交会角与侧向特征'), ('05_reception_regions', '三类收信区域与信息扩展'), ('06_sensitivity', '单因素敏感性')]:
        lines += [f'![{title}](q2_v3_{name}.png)', '']
    (out/'Q2优化验证报告.md').write_text('\n'.join(lines), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=int, default=200)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--phase', choices=['all', 'paired', 'reference', 'visuals', 'replot', 'report', 'check'], default='all')
    parser.add_argument('--output', type=Path, default=ROOT/'validation_results_v3')
    args = parser.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    if args.phase in ('all', 'paired'):
        data = run_paired(make_stratified_cases(args.cases), args.workers, out)
    else:
        data = json.loads((out/'paired_results.json').read_text(encoding='utf-8'))
        for name in ('selection.py', 'regions.py', 'legacy_selection_v2.py', 'validation.py'):
            assert data['metadata']['hashes'][name] == hashes()[name], f'Stale evidence: {name}'
    refs = None
    if args.phase in ('all', 'reference'):
        refs = run_references(data, args.workers, out)
    elif (out/'independent_search.json').exists():
        reference_data = json.loads((out/'independent_search.json').read_text(encoding='utf-8'))
        assert reference_data['paired_input_sha256'] == hashlib.sha256((out/'paired_results.json').read_bytes()).hexdigest(), 'Stale independent search input'
        refs = reference_data['records']
    summary = summarize(data, refs)
    dump(out/'acceptance_summary.json', summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    if args.phase in ('all', 'visuals'):
        make_visuals(data, refs, out)
    if args.phase == 'replot':
        visual = json.loads((out/'visualization_data.json').read_text(encoding='utf-8'))
        render_visuals(data, refs, visual['maps'], visual['sensitivity'], out)
    if args.phase in ('all', 'visuals', 'replot', 'report'):
        write_report(data, summary, out)
    if args.phase in ('all', 'check'):
        print(json.dumps(audit_saved_evidence(data, out), indent=2), flush=True)
    assert_acceptance(summary, require_reference=args.phase in ('all', 'check', 'reference'))


if __name__ == '__main__':
    main()
