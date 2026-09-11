# Q2 优化后算法总结（版本3）

记录日期：2026-09-11。对应程序输出 `algorithm_version=3`。本文记录原优化建议落实并通过验收后的算法，包含建模依据、实际搜索流程、默认参数、接口、验证结果与适用边界，是当前唯一保留的Q2算法summary。

本次相对版本2的核心变化是：由单起点细化改为最多5个分散起点；按精度目标修正候选保留规则；采用32→128→256并可继续至1024区间的分级评分；增加局部方向搜索、数值5%近优区域、三类收信判定和首次收信信息扩展区域C₁。

实现见 `selection.py` 和 `regions.py`；完整数值证据见 [优化验证报告](validation_results_v3/Q2优化验证报告.md)，复现见 [验证说明](VALIDATION_README.md)。目录仅保留最新版代码、文档和最终结果；旧版对照位置与评分保存在最新版`paired_results.json`中，复核时读取这些基线数据，不再执行旧算法。

## 1. 建模目标与指标

根据首测位置 S₁、首次示向度 θ₁ 和有界误差 δ=1°，先求源位置的保守外包 P⁺，再选出保证再次收信且能有效缩小定位区域的第二点 s。真实源坐标不进入算法。

主指标为所有可能第二次 direction 响应下，更新区域最小包围圆半径的保守上界 U(s)。其直接对应半径20 m圆覆盖条件。另报直径、面积、半径压缩率、移动距离和移动时间。对任何定位区域 P，有 D(P)≤2R_MEC(P)，但 D(P)≤40 m 并不足以推出 R_MEC(P)≤20 m。例如边长40 m的等边三角形的外接圆半径约23.09 m。

选点目标为 J(s)=U(s)+λ‖s-p‖/5，默认 λ=0 精度优先；λ>0 才计入移动时间成本。near 响应单列，不能解释为几何半径归零。

## 2. 首次定位外包与三类收信区域

源位置满足：

```text
G ∈ W(S₁, θ₁, δ) ∩ B(O, 1800) ∩ B(S₁, 1500) ⊆ P⁺
```

两个圆用128边外切多边形替代，利用Q1的半平面交获得凸多边形、线段或点。正常direction意味着实际站源距离大于5 m；算法仍保留包含近距部分的凸外包，以维持保守性。

固定保证收信主方案预留0.1 m裕量，ρ=999.9 m：

```text
C_safe = {s : max_{v∈vertices(P⁺)} ‖s-v‖ ≤ ρ}
```

距离函数在凸多边形上的最大值取于顶点，所以该条件覆盖 P⁺ 内所有可能源，而不是仅保证若干采样源。C_safe 是闭圆盘交，可能超出目标圆。其非空当且仅当 P⁺ 的最小包围圆半径不超过ρ，包围圆心给出安全见证点。

`classify_reception` 区分：

- 保证收信：最大顶点距离不超过999.9 m。
- 保证无信号：到整个 P⁺ 的最小距离大于1500 m；最小距离必须考虑多边形内部和边，不能只查顶点。
- 不确定：其余位置，可能收到，也可能收不到。

P⁺ 是外包，判定可以保守。不在保证收信区不等于保证无信号。

## 3. 利用首次收信的信息扩展 C₁

同一全向源的接收半径在两次检测间不变时，第一次已经收信意味着 R≥max(1000, ‖G-S₁‖)。于是可使用：

```text
C₁ = {s : ‖s-G‖ ≤ max(1000, ‖G-S₁‖), 对所有 G∈P⁺}
```

`Q2Config(candidate_region_mode="information")` 启用此方案；默认仍为固定999.9 m主方案。C₁按闭集定义判定，不额外预留固定方案的0.1 m裕量；利用固定方案的安全见证点开始搜索。

C₁不能只对 P⁺ 顶点作可变半径检查。实现最大化

```text
h(G)=‖s-G‖²−max(1000², ‖G-S₁‖²)
```

在1000 m圆内，h为凸二次函数；圆外为仿射函数；圆周上也可化为仿射形式。因此只需检查多边形顶点、边与该圆的交点、位于 P⁺ 内且背离s的圆周极值点。所有这些临界点的 h≤0 才是连续区域的正确判据。测试包含“所有顶点通过、边内部却不安全”的反例。

保证范围为题设全向源及同一有效接收半径；没有将它推广到方向性干扰源。

## 4. 连续 direction 响应的半径上界

候选点 s 的所有可能第二示向读数由顶点方位最短覆盖弧外扩δ得到；若不能落入开半圆，则采用完整360°，包括内部测点及跨0°情形。

对于读数子区间 I=[m-h,m+h]，有：

```text
P⁺ ∩ W(s, θ, δ) ⊆ P⁺ ∩ W(s, m, δ+h),  对任意 θ∈I
```

区间外包的包围圆半径给出该区间所有响应的上界；所有叶区间上界最大值为 U(s)。区间中点响应的最大半径给出采样下界 L(s)。当 δ+h≥90° 时，直接使用整个初始区域作为保守上界，随后二分收紧。

优先二分上界最大的区间。只有左右子区间均完成才替换父区间，因此预算中途用尽也不会留下响应覆盖空隙。始终加入1e-6 m数值裕量；这是浮点几何的保守计算，不是区间算术形式化认证。

包围圆的1/2/3点支撑圆枚举在Q2中作了向量化，并对全部顶点重新检查半径。Q1几何实现保持原样；另用独立实现验证两者一致。

## 5. 多起点、分级加密与最后的局部搜索

1. 在整个保证区域生成75 m粗网格，加入见证点、区域中心、当前点及12个沿首次示向旋转的径向边界特征点，包含走廊两侧。安全过滤后保留最多64个粗候选。
2. λ=0时，按边界代表性和最远点分散规则筛选，不保留近距离配额；λ>0时才给近距离候选预留约三分之一名额。
3. 全部候选先用最多32个角度区间粗评；同时将上界目标前5名和采样下界目标前5名提升至128区间，减少松上界掩盖优良区域的风险。
4. 按评分选取最多5个相距至少一个粗网格间距的起点，分别在周围±75 m范围内建立15 m细网格，每个起点最多49个候选。
5. 全部候选前5名提升至128区间，重排后前3名提升至256区间。高精度阶段容差默认0.05 m。
6. 从前2个结果执行安全约束下的局部方向搜索，步长为15、7.5、3.75 m，每级最多2次移动；超出安全区的试探方向缩回边界，位置仍逐一验证。此步骤降低网格对齐误差。
7. 对最终未收敛推荐点继续倍增加密，最多1024个区间；每次重评替换该位置的旧评分并重新排序。预算或上限耗尽时如实返回未收敛状态。

选择结果是有限候选与有限数值精度下的较优解，不声称连续全局最优。最优性另用独立对齐的全域25 m网格与局部5 m网格复核，接受差距不超过 max(2 m, 参考值的2%)。

## 6. 真正可绘制的 5% 近优区域

`near_best_candidate_cloud` 改为满足 U≤1.05·min U 的已评分点；时间折中模式中近优集仍按定位半径U定义，不混成综合目标J的集合。

连续区域由 `analyze_candidate_region(result, first, spacing_m=30)` 单独生成，避免每次选点都承担全域制图成本。它返回整个安全区域的评分节点、三角网格、5%等值边界、各连通区域面积、形心和已评分推荐点，以及区间上界间隙与几何特征。

每个三角形的顶点通过精确收信判据，凸性保证整个三角形仍在收信区域内。U值采用分片线性插值，近优边界、面积、形心是给定网格分辨率下的数值近似；未证明插值区域内每一点都满足精度阈值。阈值相对于采样最小上界，而非已知全局最优值。形心可能落在非凸近优区域外，应使用返回的已评分推荐点。

图中的保证收信范围不能解释成处处定位效果都好。近优区域用单独边界和面积标示，并报告45 m与30 m制图网格的敏感性。

## 7. 几何特征与第三问可复用信息

`geometric_features` 输出相对首次示向中心线的横向距离、左右侧标志，以及源顶点/边中点/中心探针下的最小和中位交会锐角。交会角统一折叠到[0°,90°]；接近0°对应原始交会接近0°或180°的病态情形。

探针最小交会角不是整个连续源区域的严格最小值。交会接近90°通常有利，但位置还受源范围与再次收信约束影响，不能把“交会角越大”单独当作全局最优判据。第三问可直接复用安全判据、候选评分和侧向几何特征。

## 8. 使用方式

```python
from B.q2 import Q2Config, choose_second_detection, analyze_candidate_region

first = {"position": {"x": -1000.0, "y": 0.0}, "svd_deg": 0.5}
result = choose_second_detection(first)
if result["status"] == "OK":
    print(result["selected"])
    region = analyze_candidate_region(result, first, spacing_m=30)
    print(region["components"])

expanded = choose_second_detection(
    first, config=Q2Config(candidate_region_mode="information")
)
```

函数输入仍严格校验字段、有限数值、角误差、重复测点与时间预算。先检查 `status`，可能为OK、NO_CANDIDATE、GEOMETRY_ERROR。

新增输出包括 `algorithm_version=3`、`candidate_scores`、`refinement_starts`、`response_refinement_log`、`near_best_radius_threshold_m`、`unique_evaluated_candidate_count`。同一位置只保留最新且更紧的评分；`evaluated_candidate_count`包括分级重评调用，可能大于不同位置数。`scoring_interval_limit`与实际叶区间数分开报告。

`calculation_time_limit_s`仍是整个调用的协作式预算，不能硬中断单次几何运算。耗尽时返回安全后备结果和明确的超时信息；不能用离线多秒运行结果声称满足实时预算。

## 9. 优化建议落实情况

| 原优化建议方向 | 版本3实现与验证 |
| --- | --- |
| 热力图、5%边界、连通区面积与中心 | 全安全域U场、区域JSON和PNG/SVG/PDF图；附空间分辨率限制 |
| 多起点粗细搜索、独立最优性复核 | 75/15 m、最多5起点、局部方向搜索、10例独立网格 |
| λ=0候选保留规则 | 无近点配额、双侧边界特征、空间分散性 |
| 非收敛结果二次加密 | 32→128→256→最高1024，重排、0.05 m高精度容差 |
| 好测点几何特征 | 交会角与U关系、横向距离统计、侧向/前向策略对比 |
| 三类收信位置 | 连续最大/最小距离判定，安全区外单列不确定区 |
| 信息修正区域C₁ | 全连续源区域临界点核验，面积/半径/距离/时间对比 |
| MEC、直径、面积口径 | 独立响应评估和逐例CSV，near不按零半径处理 |
| 200例、5种子、边界与敏感性 | 200随机+20切向极限、21误差、单因素分析、128/256稳定性 |
| 不引入深度学习主算法 | 全流程几何方法，接收安全和响应覆盖有包含关系依据 |

## 10. 默认参数记录

以下取自本版本 `Q2Config`；距离单位为米，时间单位为秒，示向角单位为度。表中“区间上限”是允许的最大叶区间数，达到容差时可以提前结束。

| 参数 | 默认值 | 用途 |
| --- | ---: | --- |
| `target_radius_m` | 1800 | 目标区域半径 |
| `minimum_reception_radius_m` / `maximum_reception_radius_m` | 1000 / 1500 | 全向源有效接收半径范围 |
| `safety_margin_m` | 0.1 | 固定保证方案的距离裕量 |
| `near_radius_m` | 5 | near响应距离阈值 |
| `movement_speed_mps` | 5 | 将移动距离换算为时间 |
| `movement_weight_m_per_s` | 0 | 默认精度优先 |
| `circle_sides` | 128 | 每个圆的外切多边形边数 |
| `coarse_spacing_m` / `fine_spacing_m` | 75 / 15 | 粗、细网格间距 |
| `max_coarse_candidates` | 64 | 粗候选保留上限；安全后备点可另行加入 |
| `refinement_starts` | 5 | 空间分散的细化起点上限 |
| `max_fine_candidates` | 49 | 每个细化起点的细候选上限 |
| `max_response_intervals` | 32 | 初步评分区间上限 |
| `shortlist_response_intervals` | 128 | 前列候选二次评分上限 |
| `final_response_intervals` | 256 | 最终高精度评分及局部方向搜索上限 |
| `maximum_refinement_intervals` | 1024 | 最终未收敛推荐点继续加密的上限 |
| `response_bound_tolerance_m` | 0.5 | 初步评分的上下界间隙容差 |
| `final_bound_tolerance_m` | 0.05 | 高精度阶段目标容差 |
| `polish_starts` | 2 | 最后局部方向搜索的起点数 |
| `near_best_relative_tolerance` | 0.05 | 数值近优阈值为采样最小U的1.05倍 |
| `candidate_region_mode` | `fixed` | 主方案；`information`启用C₁ |
| `repeated_position_tolerance_m` | 1e-6 | 已测位置排除容差 |
| `calculation_time_limit_s` | `None` | 默认不限制离线计算时间 |

`error_deg=1.0`是入口函数参数，不在`Q2Config`内。`near_best_epsilon_m=10.0`仍保留在配置中以兼容旧接口，但版本3的近优点云采用相对5%规则，不使用该旧加性阈值。

0.05 m是高精度阶段的计算目标，不能据此断言所有输出均已达到该精度。应读取推荐点实际的`response_bound_gap_m`和评分阶段；本次完整验收采用0.5 m的总体容差门槛。

## 11. 输入输出与复用清单

主入口：

```python
choose_second_detection(
    first_observation,
    error_deg=1.0,
    current_position=None,
    used_positions=None,
    config=None,
)
```

`first_observation`仅含`position: {x, y}`和`svd_deg`。当前位置默认首测点；首测点自动列入已测位置。附加已测位置只用于排除重复测点，不自动增加观测约束。

| 输出 | 含义 |
| --- | --- |
| `initial_region` | 首次观测与圆约束得到的定位外包 |
| `safe_candidate_region` | 当前收信保证区域的定义、见证点和包围盒；C₁包含额外假设说明 |
| `selected.position` | 推荐第二检测点 |
| `selected.worst_updated_cover_radius_m` | 连续direction响应半径上界U |
| `selected.sampled_direction_radius_m` | 已计算中点响应半径的最大值L |
| `selected.response_bound_gap_m` | 当前U−L间隙 |
| `selected.response_bound_converged` | 是否满足该条评分记录使用的容差；不代表位置搜索全局收敛 |
| `selected.movement_distance_m` / `movement_time_s` | 从当前位置出发的移动距离和时间 |
| `selected.objective_m` | 用于排名的J值 |
| `candidate_scores` | 每个位置当前保留的评分记录 |
| `refinement_starts` / `response_refinement_log` | 空间细化起点及分级评分前后首位变化 |
| `near_best_candidate_cloud` | 满足相对近优条件的已评分点；连续边界由区域分析函数另行计算 |
| `elapsed_s` / `timed_out` / `budget_overrun_s` | 实际耗时、预算耗尽状态和超时量 |

可独立复用的函数：`safe_candidate_region`、`response_radius_bound`、`classify_reception`、`is_information_candidate`、`geometric_features`、`analyze_candidate_region`。这些函数位于`selection.py`或`regions.py`，其中区域分析需要绘图库。

## 12. 本版本实际验收结果

以下数字来自已保存的[验收汇总](validation_results_v3/acceptance_summary.json)，不是预计效果。实验采用5个随机种子、200个分层随机首次观测和20个附加极限边界案例。旧版对照数据由优化前的版本2默认算法生成；两个版本的推荐位置均以相同初始外包、256区间上限和0.05 m目标容差重新评分。旧代码清理后，这些旧版位置和耗时作为已保存的历史基线使用。

| 检验指标 | 实际结果 |
| --- | --- |
| 旧 / 新同精度半径上界中位数 | 47.521 / 46.980 m |
| 配对半径上界改善均值 / 中位数 | 0.829 / 0.528 m |
| 平均改善的bootstrap 95%区间 | [0.708, 0.955] m |
| 单侧配对Wilcoxon p值 | 1.69×10⁻²⁷ |
| 改善 / 持平 / 退步，按±0.05 m区分 | 171 / 8 / 21例 |
| 最大单例退步 | 0.649 m |
| 旧 / 新最大响应上下界间隙 | 4.183 / 0.358 m |
| 收信安全违规 / 真源遗漏 / 响应上界违规 | 0 / 0 / 0 |
| 独立direction响应检验 | 49,077次；near另记40次 |
| 独立MEC实现最大差异 | 5.41×10⁻¹⁰ m |
| 独立网格最优性复核 | 10/10通过，最大差距0.556 m |
| 旧 / 新计算时间中位数 | 1.446 / 5.031 s |
| 源位置归属与热力图节点安全复核 | 2,377个源位置、5,421个节点，均通过 |
| 单元测试 | Q2清理后共31项通过（含新增基线读取测试）；Q1此前18项通过 |

5个种子的平均配对改善均为正，依次约0.762、0.821、0.872、0.703、0.985 m。独立物理响应评估中，双方均存在direction统计的193个随机案例，其采样最坏半径平均下降0.638 m；该统计不以0 m补入near，也不能替代连续响应上界。

因此，本版本的证据支持“在本次分层合成实验中，选点精度总体改善且评分可靠性提高”，不支持“所有案例必然改善”或“已求得连续全局最优”。增加计算量是此次精度改善的代价。

## 13. 可视化和复现记录

本版本保存六组PNG/SVG/PDF：

1. [全安全域U热力图与5%近优边界](validation_results_v3/q2_v3_01_good_regions.png)。
2. [200例配对改善与5种子对比](validation_results_v3/q2_v3_02_paired_improvement.png)。
3. [分级加密收敛、128/256稳定性与独立网格复核](validation_results_v3/q2_v3_03_reliability.png)。
4. [交会角、横向距离与侧向/前向策略比较](validation_results_v3/q2_v3_04_geometry_features.png)。
5. [三类收信位置与C₁扩展区域](validation_results_v3/q2_v3_05_reception_regions.png)。
6. [粗网格、细网格和圆边数的单因素敏感性](validation_results_v3/q2_v3_06_sensitivity.png)。

典型大范围案例的固定保证区域面积约439,229 m²，30 m制图网格下的5%近优面积约14,412 m²，分为两个连通区域。C₁将该案例候选区域扩展至约1,210,230 m²，但推荐半径上界仍约55.879 m；区域扩张并不自动等于推荐点精度进一步改善。各连通区的面积、形心、边界和已评分推荐点保存于`*_candidate_region.json`。

复现命令在仓库根目录执行：

```powershell
python -m pip install -r B/q2/requirements-validation.txt
python -m unittest discover -s B/q2/tests -v
python -m unittest discover -s B/q1/tests -v
python -m B.q2.optimization_validation --cases 200 --workers 4
python -m B.q2.optimization_validation --phase check
```

详细环境与分阶段命令见[验证说明](VALIDATION_README.md)。原始观测、参数、种子、源验证样本、评分和原实验代码哈希见`validation_results_v3/paired_results.json`，逐例指标见`paired_metrics.csv`。旧版文件、重复summary和未通过的中间记录已清理；最终实验的原始数值及哈希保留不变，验证辅助代码的整理记录见`validation_reorganization.json`。
