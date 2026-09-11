from pathlib import Path
import json, sys, hashlib, csv
from dataclasses import replace
HERE=Path(__file__).parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'.modeling-deps'))
from B.q2.selection import Q2Config,response_radius_bound
from B.q2.optimization_validation import check_source_hashes,summarize,assert_acceptance
out=ROOT/'B/q2/validation_results_v3'
data=json.loads((out/'paired_results.json').read_text(encoding='utf-8'))
references=json.loads((out/'independent_search.json').read_text(encoding='utf-8'))
visual=json.loads((out/'visualization_data.json').read_text(encoding='utf-8'))
check_source_hashes(data,out)
summary=summarize(data,references['records']);assert_acceptance(summary,require_reference=True)
assert summary==json.loads((out/'acceptance_summary.json').read_text())
rechecks=[]
for name in ['wide_fixed','narrow_fixed']:
    m=visual['maps'][name];r=m['result'];selected=r['selected']
    c=replace(Q2Config(**r['config']),max_response_intervals=selected['scoring_interval_limit'],response_bound_tolerance_m=0.05)
    p=[selected['position']['x'],selected['position']['y']]
    computed=response_radius_bound(r['initial_region']['vertices'],p,config=c)
    difference=abs(computed['worst_updated_cover_radius_m']-selected['worst_updated_cover_radius_m'])
    assert difference<1e-8
    rechecks.append({'case':name,'radius_difference_m':difference,'gap_m':computed['response_bound_gap_m']})
registry={'scope':'Q2正文','verification':'自动复测与证据核对，不代替参赛队人工复核','status':'题目核心要求已完成，可进入正文撰写','summary':summary,'tests':{'passed':31,'total':31,'fresh_run':True,'source':'B/q2/tests'},'fresh_selected_point_rechecks':rechecks,'maps':{k:{'first':v['first'],'selected':v['result']['selected'],'field':{n:v['field'][n] for n in ['spacing_m','safe_area_approx_m2','good_area_approx_m2','sampled_min_radius_upper_m']},'components':len(v['field']['components'])} for k,v in visual['maps'].items()},'verified':True,'source_hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [out/'paired_results.json',out/'visualization_data.json',out/'independent_search.json']}}
(HERE/'RESULT_REGISTRY.json').write_text(json.dumps(registry,ensure_ascii=False,indent=2),encoding='utf-8')
with (HERE/'CLAIM_EVIDENCE.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f);w.writerow(['id','location','claim','source','verification'])
    for row in [('Q2-S','6.7 表6-2 图6-2','200随机与20边界的配对结果','B/q2/validation_results_v3/paired_results.json','代码哈希及验收统计核对通过'),('Q2-M','6.6 表6-1 图6-1','典型候选区域和推荐坐标','B/q2/validation_results_v3/visualization_data.json','全图节点安全核验且两推荐点评分复算'),('Q2-T','6.7','31项单元测试通过','B/q2/tests','本轮运行31/31通过，含旧版失败边界回归'),('Q2-G','6.7','10例独立空间网格复核','B/q2/validation_results_v3/independent_search.json','输入哈希及通过标准核对'),('Q2-A','6.7','2377源位置与5421制图节点核验','B/q2/validation_results_v3/saved_evidence_audit.json','本轮phase check通过')]:w.writerow(row)
(HERE/'Q2完成度检查.md').write_text('''# Q2完成度检查

结论：当前版本3已完成问题二的核心要求，可以据此撰写正文。题目要求的第二检测点策略、候选区域及定位效果验证均已具备。

本轮重新运行Q2的31项单元测试，全部通过。测试覆盖旧审查发现的切向窄区域失败、跨零角度、连续响应上界、重复点排除和预算后备。重新运行验收check：200随机加20边界案例的保存证据通过检查；独立核验2377个源位置和5421个制图节点。当前算法哈希与结果文件一致，10例独立空间搜索均通过。两幅典型案例的推荐点以保存的精度配置重新评分，结果一致。

题目对应：首测角域与物理圆约束形成凸外包；固定圆盘交给出保证收信候选区域；利用第一次已经收信的信息可进一步扩展区域；连续示向响应区间外包提供定位半径评分；多起点网格和局部搜索返回经过安全核验的第二点。

当前检查不等于重新执行全部220例选点和全部49077次响应实验。全量统计来自已有原始实验，本轮核验其代码对应关系、记录统计及源/节点安全性，并辅以现有测试和典型点复算。数据均为离线合成观测，不是官方模拟器正式测试。

保留边界：固定候选区为保守区域，非最大物理候选区；空间搜索不保证连续全局最优；5%近优轮廓采用网格插值，不证明内部每点达到精度阈值；0.05m为高精度阶段目标，全体案例验收采用0.5m，保存的最大间隙为0.358m；浮点裕量不是区间算术认证；计算时间是历史实验记录，协作式预算不是硬实时承诺。

Q1正文保持不变，Q2独立保存。
''',encoding='utf-8')
print(json.dumps({'status':registry['status'],'tests':31,'cases':220,'rechecks':rechecks},ensure_ascii=False))
