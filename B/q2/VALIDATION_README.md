# Q2 版本3优化与验证复现

最新算法说明：[Q2_ALGORITHM_SUMMARY_V3.md](Q2_ALGORITHM_SUMMARY_V3.md)。目录仅保留版本3实现、独立验证辅助函数、测试、依赖清单和最终结果。

在仓库根目录，使用 Python 3.10+ 并安装依赖：

```powershell
python -m pip install -r B/q2/requirements-validation.txt
python -m unittest discover -s B/q2/tests -v
python -m unittest discover -s B/q1/tests -v
python -m B.q2.optimization_validation --cases 200 --workers 4
python -m B.q2.optimization_validation --phase check
```

默认输出目录 `B/q2/validation_results_v3/`，可通过 `--output` 改变。完整离线验证包括：

- 200 个分层随机首次观测、5 个随机种子，各40例；另外20个目标圆/接收圆切向极限案例。距离包含5.01、1000、1500 m，首次误差包含±1°及邻近值。
- 与最新版结果中已保存的版本2推荐点进行同输入配对。两个推荐点都用同一256区间、0.05 m容差重新评分，避免仅靠加密造成“看似提升”。旧算法已删除，旧版耗时为历史记录。
- 每例最多12个独立物理源位置与21个第二误差值；极窄物理区域不足12个时如实保存实际数量。真源包含、收信安全、半径上界分别核验。独立几何采用半平面边界交点枚举和支撑圆枚举。
- 10例独立搜索：各种子按初始区域半径选最窄和最宽各一例，不按改善结果挑选；全域25 m网格加局部5 m网格，允许差距 max(2 m, 参考值的2%)。有限网格不是全局最优证明。
- 固定测点的128/256区间稳定性；固定其他参数的粗网格、细网格、圆边数敏感性；45/30 m近优区域制图网格比较。
- 全域U热力图、5%近优边界及连通区面积/形心/推荐点、交会角与横向距离、三类收信区域、首次收信信息C₁扩展方案。

长实验可分阶段运行：

```powershell
python -m B.q2.optimization_validation --phase paired --cases 200 --workers 4
python -m B.q2.optimization_validation --phase reference --workers 4
python -m B.q2.optimization_validation --phase visuals
python -m B.q2.optimization_validation --phase check
```

`--phase replot`仅用已保存的可视化数据重绘，不重新跑算法；`--phase report`重写逐例CSV及中文报告。独立搜索完成后再重绘，可靠性图才会包含完整独立复核结果。

`--phase paired`和完整流程从已有`paired_results.json`读取旧版推荐点，再运行最新版并重新评价双方。使用新输出目录时默认读取随项目保留的版本3配对结果，也可用`--baseline <paired_results.json路径>`指定。匹配依据是完整首次观测；没有旧版基线的新观测会明确报错，不能以不对应的数据进行比较。仅测试当前算法的其他观测可直接调用`choose_second_detection`。成功运行不再产生长期保留的中间checkpoint文件。

验收断言要求：无失败、无安全违规、无真源遗漏、无独立半径超过保守上界；配对平均改善bootstrap区间下限大于0、Wilcoxon单侧p<0.05、5个种子平均改善均为正；最终最大上下界间隙不超过0.5 m；独立几何差异不超过1e-6 m；独立搜索全部达到阈值。任何失败返回非零，不能忽略后宣称通过。统计改善不等于每一例必然更优。

## 输出文件

| 文件 | 内容 |
| --- | --- |
| `Q2优化验证报告.md` | 中文结论、限制、图表与复现入口 |
| `acceptance_summary.json` | 验收汇总及统计检验结果 |
| `paired_results.json` | 首次观测、独立源、参数、原始结果、代码哈希 |
| `paired_metrics.csv` | 新旧同精度指标、直径、面积、压缩率、距离、时间 |
| `independent_search.json` | 独立网格候选数、最优参考点与验收差距 |
| `visualization_data.json` | 热力图场、区域边界、几何特征、敏感性原始结果 |
| `*_candidate_region.json` | 连通区域面积、形心、边界和推荐点 |
| `q2_v3_01...06_*.png/.svg/.pdf` | 六组可导出的300dpi验证图 |
| `saved_evidence_audit.json` | 初始源归属及热力图节点安全复核 |
| `validation_reorganization.json` | 验证辅助代码整理前后的哈希及原样保留的函数清单 |

近优边界按安全三角网格插值，其面积和精度范围有分辨率误差；JSON明确标明未认证插值目标和全局最优性。安全判据本身仍作用于整个凸源外包。几何图中的最小交会角是源探针最小值。

## 清理后的文件安排

仅保留`Q2_ALGORITHM_SUMMARY_V3.md`这一份算法summary以及`validation_results_v3/`这一套最终结果。旧版结果、旧算法、旧修复入口、已完成的优化计划和中间检查点均已移除。

`selection.py`与`regions.py`仍是同一已验收的版本3算法。`validation.py`仅保留最新版使用的独立几何/输出辅助函数，其命令行转到`optimization_validation.py`。后者是当前配对检验、独立搜索、可视化和验收入口。

旧版对照的位置、评分与原始实验代码哈希仍作为最新版实验的组成部分保存在`paired_results.json`中，不保留旧版可执行实现。原实验数值文件没有重写；辅助代码整理的哈希关联单独记录，避免把文件清理冒称为重新完成数值实验。

## 本机环境

本机系统默认Python缺少数值依赖；已使用桌面提供的Python3.12和项目内 `.modeling-deps` 验证。其他机器按依赖清单安装即可。本机可按下列方式运行（解释器路径依安装位置调整）：

```powershell
$env:PYTHONPATH = (Resolve-Path .modeling-deps).Path
& 'C:\Users\but48\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -m B.q2.optimization_validation --phase check
```

绘图使用Microsoft YaHei，SVG字形转路径。实验为离线合成数据，耗时是在本机并行实验环境测得，不是官方模拟器成绩或硬实时承诺。
