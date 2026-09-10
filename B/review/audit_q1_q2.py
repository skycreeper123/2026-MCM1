"""Read-only algorithm audit; writes separate evidence next to this script."""
import hashlib
import json
import math
from pathlib import Path
import platform
import time

import numpy as np
import scipy

from B.q2.selection import Q2Config, choose_second_detection
from B.q2.validation import evaluate, xy


def main():
    root = Path(__file__).resolve().parents[2]
    saved = json.loads((root / 'B/q2/validation_results/metrics.json').read_text(encoding='utf-8'))
    replay = []
    for rec in saved['records']:
        start = time.perf_counter()
        result = choose_second_detection(rec['case']['first'])
        elapsed = time.perf_counter() - start
        row = {'case': rec['case']['case'], 'status': result['status'], 'elapsed_s': elapsed}
        if result['status'] == 'OK':
            p = xy(result['selected']['position'])
            ev = evaluate(rec['case'], result, p, np.array(rec['validation_sources']),
                          saved['metadata']['errors_deg'], crosscheck=True)
            vertices = np.array(result['initial_region']['vertices'])
            cloud = [xy(q) for q in result['near_best_candidate_cloud']]
            row.update(
                position_change_m=float(np.linalg.norm(p-xy(rec['result']['selected']['position']))),
                radius_change_m=abs(ev['worst_radius_m']-rec['evaluations'][0]['worst_radius_m']),
                worst_radius_m=ev['worst_radius_m'],
                prediction_gap_m=ev['worst_radius_m']-result['selected']['worst_updated_cover_radius_m'],
                safety_margin_m=ev['safety_margin_m'],
                cloud_min_margin_m=min(1000-float(np.linalg.norm(vertices-q, axis=1).max()) for q in cloud),
                max_truth_violation_m=max(r['truth_violation_m'] for r in ev['rows']))
        replay.append(row)
        print(f"Replay {row['case']}: {row['status']}", flush=True)

    adversarial = []
    angle = math.radians(13)
    g = 1800*np.array([math.cos(angle), math.sin(angle)])
    s = g-1000*np.array([-math.sin(angle), math.cos(angle)])
    for error in (-1., -.999999, -.9999, -.99, -.9, 0.):
        obs = {'position': dict(zip(['x', 'y'], s.tolist())), 'svd_deg': 103.+error}
        r = choose_second_detection(obs)
        adversarial.append({'source': g.tolist(), 'source_radius_m': float(np.linalg.norm(g)),
                            'first_distance_m': float(np.linalg.norm(g-s)), 'first_error_deg': error,
                            'observation': obs, 'status': r['status'], 'reason': r.get('reason'),
                            'safe_region': r.get('safe_candidate_region'),
                            'selected': r.get('selected')})

    obs = saved['records'][0]['case']['first']
    start = time.perf_counter()
    limited = choose_second_detection(obs, config=Q2Config(calculation_time_limit_s=1e-6))
    deadline = {'configured_s': 1e-6, 'elapsed_s': time.perf_counter()-start,
                'status': limited['status'], 'reason': limited.get('reason'),
                'timed_out': limited.get('timed_out')}

    paired = []
    for k, name in enumerate(['current', 'precision', 'witness', 'random', 'forward']):
        pairs = [(r['evaluations'][0], r['evaluations'][k]) for r in saved['records']
                 if r.get('evaluations') and r['evaluations'][k]]
        diff = [a['worst_radius_m']-b['worst_radius_m'] for a,b in pairs]
        paired.append({'baseline': name, 'cases': len(pairs),
                       'current_radius_wins': sum(d < -1e-6 for d in diff),
                       'current_radius_losses': sum(d > 1e-6 for d in diff),
                       'median_current_minus_baseline_radius_m': float(np.median(diff)),
                       'median_current_minus_baseline_movement_s': float(np.median([
                           a['movement_time_s']-b['movement_time_s'] for a,b in pairs]))})

    q1 = json.loads((root/'B/q1/validation_results/metrics.json').read_text(encoding='utf-8'))
    output = {'environment': {'python': platform.python_version(), 'numpy': np.__version__, 'scipy': scipy.__version__},
              'q1_geometry_hash_matches_saved': hashlib.sha256((root/'B/q1/geometry.py').read_bytes()).hexdigest()==q1['geometry_sha256'],
              'q2_algorithm_hashes_match_saved': {p: hashlib.sha256((root/'B'/p).read_bytes()).hexdigest()==h
                  for p,h in saved['metadata']['hashes'].items() if not p.endswith('validation.py')},
              'replay': replay, 'adversarial': adversarial, 'deadline_probe': deadline, 'paired_saved_cases': paired}
    path = Path(__file__).with_name('audit_evidence.json')
    path.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'adversarial': [{'error': r['first_error_deg'], 'status':r['status'], 'reason':r['reason']} for r in adversarial],
                      'max_replay_radius_difference_m': max(r.get('radius_change_m',0) for r in replay),
                      'paired': paired, 'deadline':deadline}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
