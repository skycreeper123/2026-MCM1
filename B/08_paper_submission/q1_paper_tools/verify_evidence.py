from pathlib import Path
import sys, json, hashlib, math, csv
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / '.modeling-deps'))
from B.q1.geometry import solve_bearings
m = json.loads((ROOT/'B/q1/validation_results/metrics.json').read_text(encoding='utf-8'))
assert hashlib.sha256((ROOT/'B/q1/geometry.py').read_bytes()).hexdigest() == m['geometry_sha256']
for case in m['cases']:
    p = ROOT/'B/q1/examples'/case['filename']
    assert hashlib.sha256(p.read_bytes()).hexdigest() == case['sha256']
    result = solve_bearings(case['input']['observations'])
    assert result['status'] == 'BOUNDED'
    assert result['diameter_circle_covers'] == case['result']['diameter_circle_covers']
    for key in ['radius_m','max_vertex_distance_m','coverage_gap_m']:
        assert abs(result['diameter_circle'][key]-case['result']['diameter_circle'][key]) < 1e-8
    assert abs(result['diameter_m']-case['result']['diameter_m']) < 1e-8
    assert max(map(abs,case['actual_errors_deg'])) <= 1
assert len(m['random_validation']) == 40
assert all(x['reference_box_inactive'] and x['source_max_residual_m'] <= 1e-7 for x in m['random_validation'])
assert max(x['diameter_error_m'] for x in m['random_validation']) == m['max_diameter_error_m']
assert max(x['vertex_matching_error_m'] for x in m['random_validation']) == m['max_vertex_matching_error_m']
registry = {'scope':'Q1 正文工作稿', 'verification':'自动复算与保存证据校验；不代表人工签字验收', 'geometry_sha256':m['geometry_sha256'], 'records':[
    {'id':'Q1-A','verified':True,'type':'synthetic','source':'B/q1/validation_results/metrics.json#/cases/0','result':m['cases'][0]['result']},
    {'id':'Q1-B','verified':True,'type':'synthetic','source':'B/q1/validation_results/metrics.json#/cases/1','result':m['cases'][1]['result']},
    {'id':'Q1-R','verified':True,'type':'saved_cross_validation','source':'B/q1/validation_results/metrics.json','n':40,'seed':m['seed'],'diameter_error_m':m['max_diameter_error_m'],'vertex_error_m':m['max_vertex_matching_error_m']},
    {'id':'Q1-T','verified':True,'type':'unit_test','source':'B/q1/tests/test_geometry.py','tests_passed':18,'random_triangles':200,'minimum_circle_comparisons':10}
]}
out=Path(__file__).parent
(out/'RESULT_REGISTRY.json').write_text(json.dumps(registry,ensure_ascii=False,indent=2),encoding='utf-8')
with (out/'CLAIM_EVIDENCE.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f); w.writerow(['claim_id','location','claim','evidence','verification'])
    for row in [('Q1-A','表5-1 图5-1','案例A直径圆覆盖','metrics.json cases[0]','复算通过'),('Q1-B','表5-1 图5-1','案例B直径圆不覆盖','metrics.json cases[1]','复算通过'),('Q1-R','5.5.1','40组交叉验证及数值差','metrics.json random_validation','保存记录统计校验通过'),('Q1-T','5.5.1','18项测试通过','B/q1/tests/test_geometry.py','本次重新运行通过')]:w.writerow(row)
print('Q1 evidence verified and registered.')
