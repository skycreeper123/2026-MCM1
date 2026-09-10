# Q2 版本2结果复现

在项目根目录使用安装了依赖的 Python：

```powershell
python -m pip install -r B/q2/requirements-validation.txt
python -m unittest discover -s B/q2/tests -v
python -m B.q2.validation --cases 60 --sources 24 --seed 20260912
python -m B.q2.repair_validation
```

当前算法是精度优先的连续响应上界评分；第二个对照为显式λ=1的时间折中版本，另比较见证点、随机安全点和沿示向前进。真源坐标只进入独立评估器。

新结果在 `validation_results_v2/`。`validation_results/` 和 `../review/audit_evidence.json` 是修复前历史结果，保留作版本对照；不得以当前图注或策略名称重新标记历史结果。

主验证输出六组300dpi PNG/SVG/PDF、策略及可靠性表、中文说明、记录输入/原始评估/参数/哈希的 `metrics.json`。前两例固定用于展示，其余58例采用新随机种子的分层测试。每例24个独立源、21个第二误差取值；near单列，不按零几何半径计入。敏感性和权重图仅用前三例，不能当成总体最优性证据。

`repair_validation.py` 生成 `repair_acceptance.json`：

- 原30例使用相同独立源及第二误差，与修复前默认策略和纯精度策略配对。
- 144个新增边界案例：12种圆周方位、两个切向、两个站源距离、三种接近极限的首次误差。检查正常返回、推荐点和近优点云安全性、真源包含性及可用方向反馈的半径上界。
- 前10个配对案例固定输入和测点，只增加响应区间上限，检查上界不增。
- 三档小预算检查安全返回及明确的超限状态，不冒称硬实时达标。

验证不要求每例精度都优于所有基线，但要求合法边界无选点失败、无真源遗漏、无收信安全违规、无独立半径超过保守上界。断言失败返回非零状态，不能忽略后宣称通过。

已有版本2结果时用 `python -m B.q2.validation --replot` 重绘，保留计算哈希并另记重绘哈希。`--output` 指定其他目录；同目录重跑覆盖同名产物。历史版本1结果拒绝用版本2流程重绘。

本机默认Python未装依赖，本次使用桌面提供的Python3.12及项目内 `.modeling-deps`。其他机器按依赖清单安装，无需复制本机路径。绘图使用Microsoft YaHei，SVG字形转路径。

这些是离线实验，不是官方成绩。连续响应上界有数学包含关系依据，代码使用浮点容差，候选测点搜索仍为有限近似。
