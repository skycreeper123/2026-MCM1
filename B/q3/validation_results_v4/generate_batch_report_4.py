"""Rebuild batch 4 and its comparison from read-only saved run artifacts."""
import collections
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import re
import statistics as st

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'Q3_BATCH_TEST_DATA'
RUNS = ROOT / 'runs'
TZ = dt.timezone(dt.timedelta(hours=8))

def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))

def stamp(s):
    return dt.datetime.fromisoformat(s.replace('Z', '+00:00'))

def bj(s):
    return stamp(s).astimezone(TZ).strftime('%H:%M:%S')

def fmt(v, n=3):
    return f'{v:,.{n}f}'

def table(head, rows):
    return '\n'.join(['| ' + ' | '.join(head) + ' |', '| ' + ' | '.join(['---']*len(head)) + ' |'] +
                     ['| ' + ' | '.join(map(str, row)) + ' |' for row in rows])

def load_run(path):
    s = read(path)
    log = path.with_name(path.name.replace('.summary.json', '.jsonl'))
    es = [json.loads(line) for line in log.read_text(encoding='utf-8').splitlines() if line.strip()]
    p = s['planner_summary']
    responses = [e for e in es if e['event'] == 'response_received']
    acts = [e for e in es if e['event'] == 'planner_advanced']
    retries = [e for e in es if e['event'] == 'transport_retry']
    config = es[0]['planner_config']
    assert es[-1]['event'] == 'run_finished'
    assert es[-1]['planner_summary'] == p
    assert all(s[k] is True for k in ['completed', 'task_completed', 'exit_accepted', 'entered'])
    assert s['error'] is None and s['exit_response']['accepted'] is True
    assert p['completion_certificate'] and p['completion_reason'] == 'ALL_CHANNELS_RESOLVED'
    assert p['channel_counts']['UNKNOWN'] == p['channel_counts']['DETECTED'] == 0
    assert p['channel_counts']['CLEARED'] == p['cleared_count']
    assert sum(p['channel_counts'].values()) == config['channel_max']-config['channel_min']+1
    assert not p['pending_sources']
    assert p['fallback_clear_count'] == p['fallback_source_count'] == 0
    assert len(acts) == s['action_count'] == p['measure_count'] + p['clear_attempt_count'] + 1
    assert len(responses) == s['action_count'] + 1
    assert all(200 <= e['http_status'] < 300 and e['response']['accepted'] for e in responses)
    paths = collections.Counter(e['path'] for e in responses)
    assert paths == {'/enter': 1, '/exit': 1, '/measure': p['measure_count'], '/clear': p['clear_attempt_count']}
    clear = collections.Counter(e['response']['clear_result'] for e in responses if e['path'] == '/clear')
    assert clear['success'] == p['cleared_count']
    assert clear['no_target_in_range'] == p['failed_clear_count']
    assert p['clear_attempt_count'] == p['cleared_count'] + p['failed_clear_count']
    assert p['measure_count'] == p['coverage_measure_count'] + p['localize_measure_count'] + p.get('opportunistic_revisit_count', 0)
    assert math.isclose(p['average_clear_time_s'], p['virtual_time_s']/p['cleared_count'], abs_tol=1e-9)
    components = {
        '行驶': p['distance_m']/config['speed_mps'],
        '测量': p['measure_count']*config['measure_duration_s'],
        '切换': p['switch_count']*config['switch_duration_s'],
        '成功清除': p['cleared_count']*config['successful_clear_duration_s'],
        '失败清除': p['failed_clear_count']*config['failed_clear_duration_s'],
    }
    assert math.isclose(sum(components.values()), p['virtual_time_s'], abs_tol=1e-6)
    gap = abs(p['virtual_time_s']-s['exit_response']['virtual_time_s'])
    assert gap < 0.001
    return dict(file=path.name, log=log.name, summary=s, p=p, config=config,
                start=es[0]['recorded_at_utc'], end=es[-1]['recorded_at_utc'],
                wall=(stamp(es[-1]['recorded_at_utc'])-stamp(es[0]['recorded_at_utc'])).total_seconds(),
                retries=len(retries), retry_errors=dict(collections.Counter(e.get('error', '未提供') for e in retries)),
                responses=len(responses), components=components, virtual_gap_s=gap)

all_summaries = [(f, read(f)) for f in RUNS.glob('*.summary.json')]
v4_candidates = [(f,s) for f,s in all_summaries if s['planner_summary'].get('strategy') == 'v4_cooperative']
current=[]
for f in sorted(DATA.glob('*.result.json'), key=lambda f: read(f)['window_started_at_utc']):
    official=read(f)
    end_ms=round(stamp(official['ended_at_utc']).timestamp()*1000)
    candidates=sorted((abs(s['exit_response']['real_timestamp_ms']-end_ms), path.name, path)
                      for path,s in v4_candidates)
    assert candidates[0][0] <= 1 and candidates[1][0] > 1000
    r=load_run(candidates[0][2])
    assert r['p']['cleared_count'] == official['jammer_count']
    assert official['problem_no'] == 3 and official['directional_jammer_count'] == 0
    assert official['omnidirectional_jammer_count'] == official['jammer_count']
    base=f.name.removesuffix('.result.json')
    assert (DATA/(base+'.jlog')).exists() and (DATA/(base+'.psum')).exists()
    r.update(case=official['case_code'], official=official, result_file=f.name, match_gap_ms=candidates[0][0])
    current.append(r)
assert len(current)==12 and len({r['file'] for r in current})==12

old_text=(ROOT/'Q3_BATCH_TEST_REPORT_3.md').read_text(encoding='utf-8')
old_table=[]
for line in old_text.splitlines():
    if re.match(r'^\| [1-5] \| [A-Z0-9]{4}-',line):
        old_table.append([c.strip() for c in line.strip('|').split('|')])
assert len(old_table)==5
old_paths=sorted(f for f,s in all_summaries if s['planner_summary'].get('strategy')=='v3_global')
assert len(old_paths)==5
previous=[]
for path, row in zip(old_paths,old_table):
    r=load_run(path); p=r['p']
    assert row[2]==f"{p['cleared_count']}/{p['cleared_count']}"
    for index,key in [(3,'virtual_time_s'),(4,'average_clear_time_s'),(5,'distance_m'),(9,'planning_time_s')]:
        assert abs(float(row[index].replace(',',''))-p[key]) <= 0.000501
    assert row[6]==f"{p['measure_count']}（{p['coverage_measure_count']}/{p['localize_measure_count']}）"
    assert int(row[7])==p['switch_count']
    assert row[8]==f"{p['clear_attempt_count']}（{p['failed_clear_count']}）"
    assert int(row[10])==r['summary']['action_count'] and int(row[11])==r['retries']
    r['case']=row[1];previous.append(r)

def total(rs,k):
    return sum(r['p'].get(k,0) for r in rs)

def values(rs,k):
    return [r['p'][k] for r in rs]

def per(rs,k):
    return total(rs,k)/total(rs,'cleared_count')

def aggregate(rs):
    times=values(rs,'virtual_time_s'); av=values(rs,'average_clear_time_s')
    return dict(runs=len(rs),sources=total(rs,'cleared_count'),weighted_time=per(rs,'virtual_time_s'),
                mean=st.mean(av),median=st.median(av),sd=st.stdev(av),cv=st.stdev(av)/st.mean(av)*100,
                minimum=min(av),maximum=max(av),time_mean=st.mean(times),time_median=st.median(times),
                time_sd=st.stdev(times),time_min=min(times),time_max=max(times),
                actions=sum(r['summary']['action_count'] for r in rs),responses=sum(r['responses'] for r in rs),
                retries=sum(r['retries'] for r in rs),retry_runs=sum(r['retries']>0 for r in rs),
                clear_success_pct=total(rs,'cleared_count')/total(rs,'clear_attempt_count')*100)

A=aggregate(previous); B=aggregate(current)
N=B['sources']; K=len(current)
def delta(a,b):
    return (b/a-1)*100

basic_rows=[]; operation_rows=[]; mapping=[]
for i,r in enumerate(current,1):
    p=r['p']
    basic_rows.append([i,r['case'],f"{p['cleared_count']}/{p['cleared_count']}",fmt(p['virtual_time_s']),fmt(p['average_clear_time_s']),fmt(p['distance_m'])])
    operation_rows.append([i,f"{p['measure_count']}（{p['coverage_measure_count']}/{p['localize_measure_count']}/{p['opportunistic_revisit_count']}）",p['switch_count'],f"{p['clear_attempt_count']}（{p['failed_clear_count']}）",fmt(p['planning_time_s']),r['summary']['action_count'],r['retries'],int(r['wall'])])
    mapping.append([i,r['case'],r['file'],bj(r['official']['ended_at_utc']),r['match_gap_ms']])

stat_rows=[['干扰源总数 / 成功清除数',f'{N} / {N}'],['场次完成 / 正常退出',f'{K}/{K} / {K}/{K}'],
           ['虚拟时间合计（s）',fmt(total(current,'virtual_time_s'))]]
for label,key in [('每场虚拟时间均值（s）','time_mean'),('每场虚拟时间中位数（s）','time_median'),('每场虚拟时间样本标准差（s）','time_sd'),('单源平均时间的场次均值（s/源）','mean'),('单源平均时间中位数（s/源）','median'),('单源平均时间样本标准差（s/源）','sd'),('加权单源平均时间（s/源）','weighted_time')]:
    stat_rows.append([label,fmt(B[key])])
stat_rows.extend([['每场虚拟时间范围（s）',f"{fmt(B['time_min'])}—{fmt(B['time_max'])}"],['单源平均时间范围（s/源）',f"{fmt(B['minimum'])}—{fmt(B['maximum'])}"],['单源平均时间变异系数',fmt(B['cv'],2)+'%']])

volume_rows=[]
for label,key in [('路程（m）','distance_m'),('测量总次数','measure_count'),('覆盖测量','coverage_measure_count'),('自适应定位测量','localize_measure_count'),('固定站点机会复测','opportunistic_revisit_count'),('信道切换','switch_count'),('清除尝试','clear_attempt_count'),('失败清除','failed_clear_count'),('规划计算耗时（s）','planning_time_s')]:
    t=total(current,key);volume_rows.append([label,fmt(t),fmt(t/K),fmt(t/N)])
volume_rows.append(['动作数',B['actions'],fmt(B['actions']/K),fmt(B['actions']/N)])
time_rows=[]
for c in current[0]['components']:
    t=sum(r['components'][c] for r in current)
    time_rows.append([c,fmt(t),fmt(t/N),fmt(t/total(current,'virtual_time_s')*100,2)+'%'])

best=min(current,key=lambda r:r['p']['average_clear_time_s']);worst=max(current,key=lambda r:r['p']['average_clear_time_s'])
services=[v for r in current for v in r['p']['source_service_times_s'].values()]
assert len(services)==N
def percentile(xs,q):
    a=sorted(xs); z=(len(a)-1)*q; i=int(z); j=min(i+1,len(a)-1)
    return a[i]+(a[j]-a[i])*(z-i)
groups=[]
for count in sorted({r['p']['cleared_count'] for r in current}):
    rs=[r for r in current if r['p']['cleared_count']==count]
    groups.append([count,len(rs),fmt(st.mean(values(rs,'average_clear_time_s'))),fmt(per(rs,'distance_m')),fmt(per(rs,'measure_count'))])

report=f'''# B题第三问 v4_cooperative 第四轮仿真结果记录

## 1. 本轮结论

2026-09-11 本轮 12 场练习仿真全部完成，156 个全向干扰源全部清除，均正常退出，无 fallback、无最终运行错误。加权单源平均时间为 **{fmt(B['weighted_time'])} s/源**，场次均值为 **{fmt(B['mean'])} s/源**。本报告记录批次实际表现，不代表定向干扰源场景已获验证。

## 2. 数据范围与核对方法

- 算法标识取自运行日志：`v4_cooperative`。
- 官方结果窗口：北京时间 {bj(current[0]['official']['window_started_at_utc'])}—{bj(current[-1]['official']['ended_at_utc'])}。
- 本地运行日志范围：北京时间 {bj(current[0]['start'])}—{bj(current[-1]['end'])}；单场跨度 {int(min(r['wall'] for r in current))}—{int(max(r['wall'] for r in current))} s，时间戳精度为秒。
- 官方文件目录：`B/q3(改良)/q3/Q3_BATCH_TEST_DATA`，12 组 `.result.json`、`.jlog`、`.psum`。
- 本地摘要与行为日志：同级 `runs` 下对应的 12 份 `.summary.json` 和 12 份 `.jsonl`。
- 使用 `.result.json` 的 `ended_at_utc` 与本地退出响应的 `real_timestamp_ms` 匹配，逐场差值 0—1 ms，且官方干扰源数与本地成功清除数相等；得到唯一的一一对应关系。未使用复制后文件修改时间作为匹配依据。
- `.result.json` 提供场次、数量和时间元数据；性能与完成状态来自本地摘要和实际接口响应。官方 `.jlog`、`.psum` 仅核对配套存在，本次没有解密其内容，也未执行签名校验。
- 摘要中的 `jsonl_log` 保留原机器 `F:` 盘路径；本次实际读取本目录同名日志，不依赖该历史路径。

12 场均核验：`task_completed=true`、`completion_certificate=true`、`completion_reason=ALL_CHANNELS_RESOLVED`、`exit_accepted=true`；未知信道和已检测未清除信道均为 0，待处理源为空。已清除与不存在信道合计均为 20。

## 3. 分场结果

### 3.1 清除、时间与路径

{table(['场次','案例编号','干扰源/清除数','虚拟时间（s）','单源平均时间（s/源）','路程（m）'],basic_rows)}

### 3.2 测量、清除与运行开销

{table(['场次','测量总数（覆盖/自适应定位/机会复测）','切换','清除尝试（失败）','规划耗时（s）','动作数','传输重试','日志跨度（s）'],operation_rows)}

测量三类互斥，逐场满足“总测量 = 覆盖 + 自适应定位 + 机会复测”。动作数包含退出动作，不包含进入请求；信道切换由测量动作携带，不作为额外接口动作累计。清除失败指接口正常返回 `no_target_in_range`，不属于网络失败。

## 4. 总体统计

### 4.1 完成与时间

{table(['指标','结果'],stat_rows)}

### 4.2 路径、动作与规划

{table(['指标','总量','每场均值','每源均值'],volume_rows)}

- 清除尝试成功比例：{N}/{total(current,'clear_attempt_count')} = **{fmt(B['clear_success_pct'],2)}%**。
- 零失败清除场次：{sum(r['p']['failed_clear_count']==0 for r in current)}/{K}；fallback 清除次数和启用 fallback 场次均为 0。
- 规划耗时每场范围：{fmt(min(values(current,'planning_time_s')))}—{fmt(max(values(current,'planning_time_s')))} s。
- 覆盖、自适应定位、机会复测分别占总测量 {fmt(total(current,'coverage_measure_count')/total(current,'measure_count')*100,2)}%、{fmt(total(current,'localize_measure_count')/total(current,'measure_count')*100,2)}%、{fmt(total(current,'opportunistic_revisit_count')/total(current,'measure_count')*100,2)}%。

### 4.3 虚拟时间构成

日志配置为速度 5 m/s、每次测量 5 s、切换 1 s、成功清除 5 s、失败清除 3 s；12 场均按各自配置核算一致。

{table(['构成','总时间（s）','每源时间（s/源）','占比'],time_rows)}

虚拟时间 = 路程/速度 + 测量时间 + 切换时间 + 成功清除时间 + 失败清除时间。规划计算为实际计算耗时，单列记录，不叠加到虚拟时间中。本地重算与退出响应的虚拟时间最大差值为 {max(r['virtual_gap_s'] for r in current):.9f} s。

## 5. 波动与典型场次

- 最低单源平均时间：{best['case']}，{best['p']['cleared_count']} 个源，{fmt(best['p']['average_clear_time_s'])} s/源。
- 最高单源平均时间：{worst['case']}，{worst['p']['cleared_count']} 个源，{fmt(worst['p']['average_clear_time_s'])} s/源。
- 单源平均时间极差 {fmt(B['maximum']-B['minimum'])} s/源。总体清除成功并不意味着每场效率相同。

按本轮干扰源数量分组的描述统计如下。每组样本少、地图不同，不能据此认定源数量是性能差异的原因。

{table(['每场源数','场次数','场次平均时间均值（s/源）','加权路程（m/源）','测量（次/源）'],groups)}

本轮另记录“从首次检测到该源被清除”的服务时间，包含排队等待：156 个源合并均值 {fmt(st.mean(services))} s，中位数 {fmt(st.median(services))} s，P95 {fmt(percentile(services,.95))} s，最大值 {fmt(max(services))} s。这与“全场总虚拟时间/清除数”含义不同，不能互相替代；各源等待区间可能重叠，服务时间也不能求和作为总任务时间。

## 6. 通信与运行稳定性

- 共 {B['actions']} 个动作、{B['responses']} 个成功 HTTP 响应（含每场进入请求）；所有响应均为 2xx 且 `accepted=true`。
- {B['retry_runs']}/{K} 场有传输重试，共 {B['retries']} 次；最大单场 {max(r['retries'] for r in current)} 次，全部恢复。
- 没有最终错误、未完成场次或未接受退出；失败清除已单独计入行为成本。
- 响应中的成功清除数与摘要、官方干扰源总数逐场相等；行为日志末尾摘要与独立摘要完全一致。

## 7. 场次与文件对应记录

{table(['场次','案例编号','本地摘要文件','官方结束时间（北京时间）','退出时间差（ms）'],mapping)}

同名 `.jsonl` 即该场行为日志。官方文件名包含对应案例编号。

## 8. 统计口径与复现

- 所有统计保留全部 12 场，无异常场次剔除、无插补。
- 单场单源平均时间 = 该场总虚拟时间/成功清除数；场次均值为 12 个单场值的算术平均。
- 加权指标 = 12 场对应总量/156。两种均值分母不同，均予以保留。
- 标准差采用样本标准差（分母 `n-1`）；P95 使用排序后的线性插值。
- 计算使用原始精度，展示通常保留三位小数，展示值求和可能有末位舍入差。
- 复现脚本：`validation_results_v4/generate_batch_report_4.py`；核对结果：`validation_results_v4/batch_report_4_audit.json`。
- 与第三轮的对比另见 `Q3_BATCH_COMPARISON_3_VS_4.md`。原始日志及第三轮报告均未修改。
'''

comparison_rows=[]
def comp(label,a,b,unit=''):
    comparison_rows.append([label,fmt(a)+unit,fmt(b)+unit,fmt(delta(a,b),2)+'%'])
for label,key in [('加权单源平均时间（s/源）','weighted_time'),('单源平均时间场次均值（s/源）','mean'),('单源平均时间中位数（s/源）','median'),('最小单源平均时间（s/源）','minimum'),('最大单源平均时间（s/源）','maximum'),('单源平均时间样本标准差（s/源）','sd'),('每场总虚拟时间均值（s）','time_mean')]:
    comp(label,A[key],B[key])
for label,key in [('路程（m/源）','distance_m'),('总测量（次/源）','measure_count'),('覆盖测量（次/源）','coverage_measure_count'),('切换（次/源）','switch_count'),('清除尝试（次/源）','clear_attempt_count'),('失败清除（次/源）','failed_clear_count'),('规划耗时（s/源）','planning_time_s')]:
    comp(label,per(previous,key),per(current,key))
comp('每场规划耗时（s）',total(previous,'planning_time_s')/len(previous),total(current,'planning_time_s')/K)
comp('动作数（次/源）',A['actions']/A['sources'],B['actions']/N)

contribution=[]
for c in current[0]['components']:
    a=sum(r['components'][c] for r in previous)/A['sources']; b=sum(r['components'][c] for r in current)/N
    contribution.append([c,fmt(a),fmt(b),f'{b-a:+.3f}'])
contribution.append(['合计',fmt(A['weighted_time']),fmt(B['weighted_time']),f"{B['weighted_time']-A['weighted_time']:+.3f}"])
lower_than_old_min=sum(r['p']['average_clear_time_s']<A['minimum'] for r in current)
higher_than_old_max=sum(r['p']['average_clear_time_s']>A['maximum'] for r in current)
comparison=f'''# B题第三问第三轮与第四轮仿真对比报告

## 1. 主要结论

与 `Q3_BATCH_TEST_REPORT_3.md` 的 `v3_global` 五场相比，本轮 `v4_cooperative` 十二场保持 100% 场次完成和干扰源清除：加权单源平均时间由 **{fmt(A['weighted_time'])} 降至 {fmt(B['weighted_time'])} s/源（{delta(A['weighted_time'],B['weighted_time']):.2f}%）**，每源路程下降 **{-delta(per(previous,'distance_m'),per(current,'distance_m')):.2f}%**，每源失败清除下降 **{-delta(per(previous,'failed_clear_count'),per(current,'failed_clear_count')):.2f}%**，每场规划耗时下降 **{-delta(total(previous,'planning_time_s')/5,total(current,'planning_time_s')/12):.2f}%**。

同时，每源总测量增加 **{delta(per(previous,'measure_count'),per(current,'measure_count')):.2f}%**，每源切换增加 **{delta(per(previous,'switch_count'),per(current,'switch_count')):.2f}%**；单源时间场间波动增大，最慢场次也比旧批次更慢。因此本轮表现为“总体时间与路程下降，但测量、切换开销及场间波动上升”。这属于不同场景批次的观察，不能把变化全部归因于算法版本。

## 2. 数据基础与可比性

{table(['项目','第三轮：v3_global','第四轮：v4_cooperative'],[
['测试日期','2026-09-11','2026-09-11'],
['本地日志范围（北京时间）','17:55:04—17:58:53',bj(current[0]['start'])+'—'+bj(current[-1]['end'])],
['场次','5','12'],['干扰源总数','68','156'],['每场平均源数','13.600','13.000'],
['每场源数范围','12—16','10—16'],['场景类型','全向干扰源','全向干扰源'],
['场次完整清除 / 正常退出','5/5 / 5/5','12/12 / 12/12'],
['干扰源清除','68/68','156/156'],['fallback 场次','0/5','0/12'],
['本轮报告','Q3_BATCH_TEST_REPORT_3.md','Q3_BATCH_TEST_REPORT_4.md']])}

第三轮按既有报告中的五场顺序与 `runs` 中五组 `v3_global` 日志核对；每场源数、时间、路程、测量、切换、清除、规划、动作和重试均与报告在展示精度内一致。第三轮案例编号沿用既有报告，本次没有重新读取其官方加密结果。第四轮按官方结束时间与本地退出响应唯一匹配，时间差不超过 1 ms，详见第四轮记录。

两批案例编号无重合，不能做同图逐场配对比较；样本量和源数构成也不同。对性能优先使用每源指标，同时列出每场统计；不直接比较总路程、总动作等随批次规模增长的总量。两轮日志中的速度和各类操作时间参数相同，虚拟时间分解具有相同口径。

## 3. 核心指标对比

{table(['指标','第三轮','第四轮','相对变化'],comparison_rows)}

相对变化 =（第四轮/第三轮 − 1）×100%；负号表示数值下降，是否有利需结合指标含义。标准差和极值仅描述这些样本，并未控制场景难度。

清除尝试成功比例从 {fmt(A['clear_success_pct'],2)}% 升至 {fmt(B['clear_success_pct'],2)}%，提高 {fmt(B['clear_success_pct']-A['clear_success_pct'],2)} 个百分点。零失败清除场次从 {sum(r['p']['failed_clear_count']==0 for r in previous)}/5 增至 {sum(r['p']['failed_clear_count']==0 for r in current)}/12。

## 4. 时间减少体现在哪些成本上

按两轮日志的实际配置重算，虚拟时间 = 路程/5 + 测量数×5 + 切换数×1 + 成功清除数×5 + 失败清除数×3。

{table(['时间构成','第三轮（s/源）','第四轮（s/源）','变化（s/源）'],contribution)}

每源行驶时间减少 {fmt(per(previous,'distance_m')/5-per(current,'distance_m')/5)} s，失败清除时间减少 {fmt((per(previous,'failed_clear_count')-per(current,'failed_clear_count'))*3)} s；新增测量与切换时间分别抵消 {fmt((per(current,'measure_count')-per(previous,'measure_count'))*5)} s 和 {fmt(per(current,'switch_count')-per(previous,'switch_count'))} s，合计净减少 {fmt(A['weighted_time']-B['weighted_time'])} s/源。

这是对观测成本的算术分解，说明批次时间差主要对应行驶距离变化；它不能单独证明某项规划机制造成了路径缩短。规划计算耗时不计入上述虚拟时间，应作为实际计算开销单独考察。

## 5. 测量统计口径及代价

{table(['类别','第三轮总次数','第四轮总次数','第三轮（次/源）','第四轮（次/源）'],[
['覆盖测量',375,963,fmt(375/68),fmt(963/156)],
['原始 localize_measure_count',88,72,fmt(88/68),fmt(72/156)],
['固定站点机会复测','未单列',198,'未单列',fmt(198/156)],
['非覆盖测量（总测量−覆盖）',88,270,fmt(88/68),fmt(270/156)],
['总测量',463,1233,fmt(463/68),fmt(1233/156)]])}

第三轮满足“总测量 = 覆盖 + 定位”；第四轮满足“总测量 = 覆盖 + 自适应定位 + 固定站点机会复测”。因此，单看 `localize_measure_count` 从 88 次降到 72 次，不能得出全部定位信息采集工作量减少的结论。用统一的“非覆盖测量”口径，实际从 {fmt(88/68)} 增至 {fmt(270/156)} 次/源（{delta(88/68,270/156):+.2f}%）。旧版没有单列复测字段，不据此推断旧版同类行为次数为零。

新版机会复测与自适应定位如何各自影响路线，应在同图条件下关闭/开启对应机制进行消融验证；本报告不以本批数据推断单项机制的因果贡献。

## 6. 波动、较慢场次与样本构成

- 单源时间变异系数：第三轮 {fmt(A['cv'],2)}%，第四轮 {fmt(B['cv'],2)}%。
- 第四轮 {lower_than_old_min}/12 场低于第三轮最快值 {fmt(A['minimum'])} s/源，但仍有 {higher_than_old_max}/12 场高于第三轮最慢值 {fmt(A['maximum'])} s/源。
- 第四轮最高值来自 {worst['case']}：{fmt(worst['p']['average_clear_time_s'])} s/源；第三轮最高值为 {fmt(A['maximum'])} s/源，新批次最高值增加 {fmt(delta(A['maximum'],B['maximum']),2)}%。
- 第四轮出现第三轮未覆盖的 10 源和 11 源场景；全场固定扫描等开销除以较少源数时，每源指标可能变大。第四轮分组表显示样本构成值得关注，但不能据此认定差异只由源数造成。

新批次均值下降与离散程度上升同时存在，不能表述为“所有场景均变快”或“性能稳定性全面改善”。清除完成的可靠性与时间表现的稳定性是两类不同指标。

## 7. 通信与计算开销

{table(['指标','第三轮','第四轮'],[
['成功 HTTP 响应',A['responses'],B['responses']],['非 2xx / 最终错误','0 / 0','0 / 0'],
['传输重试次数',A['retries'],B['retries']],['发生重试的场次',f"{A['retry_runs']}/5",f"{B['retry_runs']}/12"],
['每场传输重试',fmt(A['retries']/5),fmt(B['retries']/12)],
['规划耗时总量（s）',fmt(total(previous,'planning_time_s')),fmt(total(current,'planning_time_s'))],
['每场规划耗时（s）',fmt(total(previous,'planning_time_s')/5),fmt(total(current,'planning_time_s')/12)],
['单场日志跨度（s，秒精度）','13—19',f"{int(min(r['wall'] for r in current))}—{int(max(r['wall'] for r in current))}"]])}

两轮重试均恢复。第四轮重试更普遍，但运行环境和连接状态未受控，不能把重试增加直接归因于算法。每场规划计时下降可作为本次运行的计算开销结果；硬件负载未受控，不外推为跨机器的速度保证。

## 8. 可支持的结论与后续验证

1. 在本次两个全向干扰源批次中，新版维持全部清除，平均虚拟耗时、每源路程和失败清除成本更低。
2. 代价是测量、切换更多；较慢场次仍存在，场间波动增加。上述变化均需在报告和论文中同时披露。
3. 当前实验不支持定向源效果、同图必然提升、各机制独立贡献或严格因果提升幅度等结论。
4. 后续应优先使用相同场景/种子配对运行两版，覆盖不同源数并增加定向源；逐图比较时间差和路径差，再评估固定站点复测、区域访问排序与自适应定位的独立贡献。

## 9. 复现说明

使用两轮原始摘要和 JSONL 重新计算，第三轮展示值已与 `Q3_BATCH_TEST_REPORT_3.md` 核对；第四轮来源为 `Q3_BATCH_TEST_DATA` 与 `runs` 的 12 组匹配记录。所有场次保留，使用原始精度，标准差为样本标准差。未进行同图配对检验，也未报告统计显著性。

详见 `Q3_BATCH_TEST_REPORT_4.md` 的逐场记录与 `validation_results_v4/batch_report_4_audit.json` 的核对明细。可使用 `validation_results_v4/generate_batch_report_4.py` 复算，两份报告均为派生文件。
'''

for key in ['speed_mps','measure_duration_s','switch_duration_s','successful_clear_duration_s','failed_clear_duration_s']:
    assert len({r['config'][key] for r in previous+current})==1
assert not ({r['case'] for r in previous}&{r['case'] for r in current})
audit={'date':'2026-09-11','previous':A,'current':B,'checks':'All assertions passed',
       'rows':[dict(case=r['case'],summary=r['file'],result=r['result_file'],match_gap_ms=r['match_gap_ms'],
                    virtual_gap_s=r['virtual_gap_s'],retries=r['retries'],retry_errors=r['retry_errors']) for r in current],
       'input_sha256':{str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest()
                       for f in [ROOT/'Q3_BATCH_TEST_REPORT_3.md'] +
                       [RUNS/r[k] for r in previous+current for k in ['file','log']] +
                       [DATA/r['result_file'] for r in current]}}
(ROOT/'Q3_BATCH_TEST_REPORT_4.md').write_text(report,encoding='utf-8')
(ROOT/'Q3_BATCH_COMPARISON_3_VS_4.md').write_text(comparison,encoding='utf-8')
(ROOT/'validation_results_v4/batch_report_4_audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'previous':A,'current':B,'reports':['Q3_BATCH_TEST_REPORT_4.md','Q3_BATCH_COMPARISON_3_VS_4.md'],'checks':'passed'},ensure_ascii=True))
