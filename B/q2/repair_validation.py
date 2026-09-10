"""Version 2 repair acceptance: paired old inputs and new boundary cases.

python -m B.q2.repair_validation
Writes separate evidence; never overwrites version 1 audit or validation data.
"""
from dataclasses import replace
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from .selection import Q2Config, choose_second_detection, response_radius_bound
from .validation import evaluate, xy, dump


ROOT = Path(__file__).resolve().parent


def check_acceptance(summary):
    print(json.dumps(summary, ensure_ascii=False, indent=2, default=lambda x: x.item()), flush=True)
    assert not any(summary[k] for k in ('paired_failures', 'paired_upper_bound_violations', 'boundary_failures',
                                       'boundary_safety_violations', 'boundary_cloud_violations',
                                       'boundary_truth_failures', 'boundary_bound_violations')), summary
    assert summary['new_radius_median_m'] < summary['old_radius_median_m'], summary
    assert summary['refinement_monotone_cases'] == 10, summary
    assert summary['budget_safe_returns'] == 3, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-saved', action='store_true', help='Check already-computed evidence and algorithm hashes')
    args = parser.parse_args()
    out = ROOT/'validation_results_v2'
    out.mkdir(exist_ok=True)
    if args.check_saved:
        data = json.loads((out/'repair_acceptance.json').read_text(encoding='utf-8'))
        for name in ('selection.py', 'validation.py'):
            current = hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
            if name == 'validation.py' and current != data['hashes'][name]:
                # Plot-only revisions are explicitly tracked separately from
                # the validator version that produced the numerical evidence.
                metadata = json.loads((out/'metrics.json').read_text(encoding='utf-8'))['metadata']
                computed = next(h for p, h in metadata['hashes'].items() if p.endswith('validation.py'))
                assert computed == data['hashes'][name] and current == metadata.get('render_program_sha256'), name
            else:
                assert current == data['hashes'][name], name
        check_acceptance(data['summary'])
        return
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (ROOT/'selection.py', ROOT/'validation.py', Path(__file__))}
    old = json.loads((ROOT/'validation_results/metrics.json').read_text(encoding='utf-8'))
    paired = []
    for record in old['records']:
        r = choose_second_detection(record['case']['first'])
        if r['status'] != 'OK':
            paired.append({'case': record['case']['case'], 'failure': r})
            continue
        ev = evaluate(record['case'], r, xy(r['selected']['position']),
                      np.array(record['validation_sources']), old['metadata']['errors_deg'], True)
        paired.append({'case': record['case']['case'], 'result': r,
                       'new_radius_m': ev['worst_radius_m'],
                       'old_default_radius_m': record['evaluations'][0]['worst_radius_m'],
                       'old_precision_radius_m': record['evaluations'][1]['worst_radius_m'],
                       'new_movement_s': ev['movement_time_s'],
                       'old_movement_s': record['evaluations'][0]['movement_time_s'],
                       'bound_gap_m': ev['worst_radius_m']-r['selected']['worst_updated_cover_radius_m'],
                       'truth_violation_m': max(row['truth_violation_m'] for row in ev['rows'])})
        if record['case']['case'] % 5 == 0:
            print(f"Paired old cases: {record['case']['case']}/30", flush=True)

    boundary = []
    for angle in (0., 13., 44., 77., 89.99, 137., 180., 203., 244., 277., 315., 359.99):
        a = math.radians(angle)
        g = 1800*np.array([math.cos(a), math.sin(a)])
        for side in (-1, 1):
            tangent = side*np.array([-math.sin(a), math.cos(a)])
            for distance in (1000., 1499.9):
                station = g-distance*tangent
                for offset in (1., .999999, .9999):
                    error = -side*offset
                    first = {'position': dict(zip(['x', 'y'], station.tolist())),
                             'svd_deg': (angle+side*90+error) % 360}
                    r = choose_second_detection(first)
                    row = {'angle_deg': angle, 'side': side, 'first_distance_m': distance,
                           'first_error_deg': error, 'source': g.tolist(), 'first': first, 'result': r}
                    if r['status'] == 'OK':
                        p = xy(r['selected']['position'])
                        v = np.array(r['initial_region']['vertices'])
                        row['safety_margin_m'] = 1000-float(np.linalg.norm(v-p, axis=1).max())
                        row['true_distance_m'] = float(np.linalg.norm(p-g))
                        row['cloud_min_margin_m'] = min(1000-float(np.linalg.norm(v-xy(q), axis=1).max())
                                                        for q in r['near_best_candidate_cloud'])
                        # Check membership using independent convex combinations
                        # via linear programming; also handles segment/point P.
                        from scipy.optimize import linprog
                        lp = linprog(np.zeros(len(v)), A_eq=np.vstack((v.T, np.ones(len(v)))),
                                     b_eq=np.r_[g, 1.], bounds=(0, None), method='highs')
                        row['truth_in_initial_polygon'] = bool(lp.success)
                        # At tangent cases the real physical region can be a point.
                        # Independently assess 41 errors when the initial polygon
                        # is two-dimensional; degenerate geometry is unit-tested.
                        if r['initial_region']['dimension'] == 2:
                            ev = evaluate({'first': first}, r, p, np.array([g]), np.linspace(-1, 1, 41).tolist())
                            row['truth_violation_m'] = max(x['truth_violation_m'] for x in ev['rows'])
                            row['bound_violation_m'] = (ev['worst_radius_m']-r['selected']['worst_updated_cover_radius_m']
                                                        if ev['worst_radius_m'] is not None else 0.)
                    boundary.append(row)
        print(f'Boundary cases: {len(boundary)}/144', flush=True)
        dump(out/'repair_checkpoint.json', {'paired': paired, 'boundary': boundary})

    refinement = []
    for record in paired[:10]:
        r = record['result']
        bounds = [response_radius_bound(np.array(r['initial_region']['vertices']), xy(r['selected']['position']),
                                        config=Q2Config(max_response_intervals=n, response_bound_tolerance_m=.05))
                  for n in (16, 32, 64, 128)]
        values = [b['worst_updated_cover_radius_m'] for b in bounds]
        refinement.append({'case': record['case'], 'interval_limits': [16, 32, 64, 128],
                           'bounds': bounds, 'monotone': all(b <= a+1e-6 for a,b in zip(values, values[1:]))})

    budgets = []
    first = old['records'][0]['case']['first']
    for budget in (1e-6, .05, .5):
        r = choose_second_detection(first, config=Q2Config(calculation_time_limit_s=budget))
        budgets.append({'budget_s': budget, 'result': r,
                        'safe': r['status'] == 'OK' and
                        np.linalg.norm(np.array(r['initial_region']['vertices'])-xy(r['selected']['position']), axis=1).max() <= 999.9+1e-7})

    valid = [p for p in paired if 'failure' not in p]
    summary = {'paired_cases': len(paired), 'paired_failures': len(paired)-len(valid),
               'new_radius_median_m': float(np.median([r['new_radius_m'] for r in valid])),
               'old_radius_median_m': float(np.median([r['old_default_radius_m'] for r in valid])),
               'old_precision_radius_median_m': float(np.median([r['old_precision_radius_m'] for r in valid])),
               'new_better_than_old_default_cases': sum(r['new_radius_m'] < r['old_default_radius_m']-1e-6 for r in valid),
               'paired_upper_bound_violations': sum(r['bound_gap_m'] > 1e-6 for r in valid),
               'boundary_cases': len(boundary),
               'boundary_failures': sum(r['result']['status'] != 'OK' for r in boundary),
               'boundary_safety_violations': sum(r.get('safety_margin_m', -1) < .1-1e-7 for r in boundary),
               'boundary_cloud_violations': sum(r.get('cloud_min_margin_m', -1) < .1-1e-7 for r in boundary),
               'boundary_truth_failures': sum(not r.get('truth_in_initial_polygon', False) or r.get('truth_violation_m', 0) > 1e-6 for r in boundary),
               'boundary_bound_violations': sum(r.get('bound_violation_m', 0) > 1e-6 for r in boundary),
               'refinement_monotone_cases': sum(r['monotone'] for r in refinement),
               'budget_safe_returns': sum(r['safe'] for r in budgets)}
    dump(out/'repair_acceptance.json', {'hashes': hashes, 'summary': summary, 'paired': paired,
                                       'boundary': boundary, 'refinement': refinement, 'budgets': budgets})
    check_acceptance(summary)


if __name__ == '__main__':
    main()
