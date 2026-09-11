# B题问题1：交会定位区域直径与直径圆覆盖判定

## 1. 任务范围

本模块只解决问题1，不负责模拟器通信，也不负责选择新的检测点。

输入为若干个检测点坐标及其对同一个干扰源测得的示向度。示向度误差固定在 `[-1°, 1°]` 内。程序需要：

1. 根据所有检测结果构造交会定位区域；
2. 计算定位区域的直径；
3. 判断直径为定位区域直径的圆能否覆盖整个定位区域；
4. 额外计算最小包围圆，为问题2至问题4提供可靠的定位和清除判据。

题目图2展示的是两个检测点的一般情形，此时通常得到四边形。但题目使用“若干个检测点”，因此程序必须支持 `n >= 2`：每个检测点产生两个半平面，最终定位区域最多可能有 `2n` 条有效边。

## 2. 最终采用的算法

完整流程如下：

```text
示向度和误差
    ↓
构造 2n 个半平面约束
    ↓
枚举边界直线交点
    ↓
筛选满足全部约束的可行交点
    ↓
去重并求 Andrew 凸包
    ↓
得到凸多边形定位区域
    ├── 旋转卡壳：计算区域直径
    ├── 直径端点圆：判断能否覆盖
    └── 最小包围圆：给后续问题使用
```

推荐的具体算法组合：

| 子任务 | 正式算法 | 辅助校验算法 |
|---|---|---|
| 定位区域 | 边界交点枚举 + 全约束筛选 + Andrew 凸包 | 绘图检查所有角域 |
| 多边形直径 | 旋转卡壳 | 顶点对暴力枚举 |
| 直径圆覆盖 | 直径端点中点 + 全顶点检查 | 点积判据 |
| 最小包围圆 | 随机增量算法 | 小规模两点/三点穷举 |

检测点数量很少，交点枚举的计算量完全可接受。与复杂的双端队列半平面交相比，这种实现更容易检查和调试。

## 3. 输入与输出

### 3.1 输入

建议使用以下 JSON 结构：

```json
{
  "angle_error_deg": 1.0,
  "detections": [
    {"x": -1000.0, "y": 0.0, "bearing_deg": 0.0},
    {"x": 0.0, "y": -1000.0, "bearing_deg": 90.0}
  ]
}
```

字段说明：

| 字段 | 含义 |
|---|---|
| `x, y` | 检测点坐标，单位为米 |
| `bearing_deg` | 示向度，正东方向为0°，逆时针为正，范围归一化到 `[0°, 360°)` |
| `angle_error_deg` | 示向度最大绝对误差，本题固定为1° |

### 3.2 输出

建议输出以下字段：

```json
{
  "status": "ok",
  "vertices": [[-10.0, -10.0], [10.0, -10.0], [10.0, 10.0], [-10.0, 10.0]],
  "area": 400.0,
  "diameter": 28.284271,
  "diameter_pair": [[-10.0, -10.0], [10.0, 10.0]],
  "diameter_circle_center": [0.0, 0.0],
  "diameter_circle_radius": 14.142136,
  "diameter_circle_required_radius": 14.142136,
  "can_diameter_circle_cover": true,
  "minimum_enclosing_circle_center": [0.0, 0.0],
  "minimum_enclosing_circle_radius": 14.142136,
  "coverage_ratio": 1.0,
  "clear_ready_20m": true,
  "clear_ready_18m": true
}
```

`status` 至少支持：

| 状态 | 含义 |
|---|---|
| `ok` | 得到正常的有界凸多边形 |
| `empty` | 所有角域没有公共交集 |
| `point` | 定位区域退化为一个点 |
| `segment` | 定位区域退化为线段 |
| `unbounded` | 定位区域无界，需要增加有效检测点 |
| `invalid_input` | 输入字段、坐标或角度不合法 |

## 4. 角域的数学表示

第 `i` 个检测点为

```math
S_i=(x_i,y_i),
```

示向度为 `θ_i`，误差上限为 `δ=1°`。定义角域的两条边界方向向量：

```math
u_i^-=(\cos(\theta_i-\delta),\ \sin(\theta_i-\delta)),
```

```math
u_i^+=(\cos(\theta_i+\delta),\ \sin(\theta_i+\delta)).
```

设待判断点为 `X=(x,y)`，二维叉积定义为

```math
\operatorname{cross}(a,b)=a_xb_y-a_yb_x.
```

则 `X` 位于第 `i` 个检测角域内，当且仅当

```math
\operatorname{cross}(u_i^-,X-S_i)\geq 0,
```

```math
\operatorname{cross}(u_i^+,X-S_i)\leq 0.
```

全部检测结果对应的定位区域为

```math
\mathcal P=\bigcap_{i=1}^{n}\left(H_i^-\cap H_i^+\right).
```

实现时统一将角度转换为弧度，并使用方向向量和叉积判断，不要直接比较角度大小。该写法能够自然处理示向度跨越0°的情况，例如 `359.5° ± 1°`。

## 5. 定位区域求解

### 5.1 边界直线交点

每个检测点产生两条边界直线。边界直线写成

```math
L_i(t)=S_i+t u_i,\qquad t\in\mathbb R.
```

对于两条不平行直线

```math
L_i(t)=S_i+t u_i,
\qquad
L_j(s)=S_j+s u_j,
```

令

```math
q=S_j-S_i,
```

则交点参数为

```math
t=\frac{\operatorname{cross}(q,u_j)}{\operatorname{cross}(u_i,u_j)}.
```

当

```math
|\operatorname{cross}(u_i,u_j)|<\varepsilon
```

时，两条直线视为平行，不计算交点。

虽然这里使用的是完整直线，但交点最终必须接受全部角域约束检查，因此位于错误射线方向的点会自动被排除。

### 5.2 可行性筛选

对每个候选交点 `P`，检查它是否满足全部 `2n` 个半平面约束。考虑浮点误差，使用：

```text
cross(u_lower, P - S) >= -EPS
cross(u_upper, P - S) <=  EPS
```

仅保留全部约束都满足的点。

### 5.3 交点去重

如果两个候选点满足

```math
\|P_i-P_j\|\leq\varepsilon_{\rm merge},
```

则将其视为同一个点。可使用坐标量化哈希，也可以在点数较少时直接两两比较。

### 5.4 Andrew 凸包

将可行交点按 `(x,y)` 字典序排序，分别构造下凸壳和上凸壳，再拼接得到逆时针排列的凸包顶点。

建议删除位于同一条边内部的共线点，只保留该边的两个端点。这可以减少后续旋转卡壳和最小包围圆计算中的冗余。

### 5.5 关于无界区域

交点枚举法只直接产生有限顶点，因此必须额外检查定位区域是否无界。理论上，将所有半平面统一写成

```math
a_k^T X\leq b_k
```

后，若全部法向量 `a_k` 不能在二维平面上形成正张成，则区域无界。工程实现可采用以下任一方法：

1. 对 `x、-x、y、-y` 分别做线性规划；任何一个方向目标无界，则定位区域无界；
2. 对所有半平面法向量的极角排序，若最大循环角间隔大于或等于180°，则约束无法在所有方向封闭区域；
3. 调试阶段用大边界框裁剪，若最终区域接触边界框，则标记为疑似无界。

推荐正式实现使用方法2，方法3仅用于可视化调试。题目称结果为“多边形定位区域”，主流程可以假设正常输入得到有界区域，但程序不能静默返回一个错误的有限凸包。

不要在问题1主算法中直接加入半径1800米的目标圆。加入该圆后边界包含圆弧，结果不再是题目所说的多边形；如需使用目标区域先验，应在论文中明确说明并单独实现。

## 6. 多边形直径

对于凸多边形

```math
\mathcal P=\operatorname{conv}\{V_1,V_2,\ldots,V_m\},
```

区域直径一定由两个顶点取得：

```math
D=\max_{1\leq i<j\leq m}\|V_i-V_j\|.
```

正式算法采用旋转卡壳，复杂度为 `O(m)`。同时保留一个 `O(m^2)` 的暴力枚举函数作为单元测试基准，要求两种算法输出的直径在容差内一致。

旋转卡壳伪代码：

```text
function polygon_diameter(vertices_ccw):
    if number_of_vertices <= 2:
        directly handle degenerate case

    j = 1
    best_distance_squared = 0
    best_pair = null

    for i in [0, m-1]:
        ni = (i + 1) mod m

        while area(V[i], V[ni], V[(j+1) mod m])
              > area(V[i], V[ni], V[j]) + EPS:
            j = (j + 1) mod m

        update best using pairs (V[i], V[j]) and (V[ni], V[j])

    return sqrt(best_distance_squared), best_pair
```

内部比较尽量使用距离平方，只在最终输出时开平方。

## 7. 直径圆覆盖判定

设直径端点为 `A、B`：

```math
D=\|A-B\|.
```

如果一个半径为 `D/2` 的圆同时包含 `A、B`，那么 `A、B` 必须是这个圆的一对直径端点，圆心只能是

```math
M=\frac{A+B}{2}.
```

计算固定圆心 `M` 覆盖所有顶点所需的半径：

```math
r_{AB}=\max_i\|V_i-M\|.
```

覆盖判据为

```math
r_{AB}\leq\frac D2+\varepsilon.
```

因为圆盘是凸集，只需检查凸包顶点，无需检查边内部的点。

等价的点积判据是：对所有顶点 `V_i`，均有

```math
(V_i-A)\cdot(V_i-B)\leq\varepsilon.
```

对于两个检测点得到的四边形，如果直径端点为 `A、B`，另外两个顶点为 `C、D`，则只需检查：

```math
(C-A)\cdot(C-B)\leq0,
```

```math
(D-A)\cdot(D-B)\leq0.
```

几何上等价于

```math
\angle ACB\geq90^\circ,
\qquad
\angle ADB\geq90^\circ.
```

如果数值上存在多个几乎相同的直径顶点对，可对所有满足 `distance >= D-EPS_D` 的顶点对执行上述检查，只要存在一个可覆盖圆即可判定为可覆盖。

## 8. 最小包围圆及后续接口

最小包围圆定义为覆盖定位多边形的最小圆：

```math
(C^*,R^*)=\operatorname{MEC}(V_1,V_2,\ldots,V_m).
```

最小包围圆只可能由以下两种边界情况决定：

1. 两个顶点构成圆的直径；
2. 三个不共线顶点确定外接圆。

推荐使用随机增量最小包围圆算法。由于本题顶点很少，也可以枚举所有两点圆和三点外接圆，筛选能覆盖全部顶点的最小圆，作为校验版本。

理论上总有

```math
R^*\geq\frac D2.
```

直径圆能够覆盖定位区域，当且仅当

```math
R^*=\frac D2.
```

定义无量纲覆盖系数：

```math
K=\frac{2R^*}{D}.
```

解释如下：

| `K` | 含义 |
|---|---|
| `K = 1` | 直径圆可以覆盖 |
| `K > 1` | 直径圆不能覆盖 |
| `K` 越大 | 仅使用直径描述定位区域越不充分 |

后续问题建议使用：

```text
R* <= 20 m：理论上可以在最小包围圆圆心执行清除
R* <= 18 m：实际程序采用的保守清除阈值
R* > 18 m ：继续选择新的检测点缩小定位区域
```

问题2的选点目标应优先最小化 `R*`，而不是只最小化直径 `D`。

## 9. 数值稳定性

### 9.1 推荐容差

不要在所有位置硬编码同一个绝对容差。先定义坐标尺度：

```math
L=\max\left(1,\max_i|x_i|,\max_i|y_i|\right).
```

推荐初始值：

```text
EPS_PARALLEL = 1e-12
EPS_GEOM     = 1e-9 * L
EPS_MERGE    = 1e-8 * L
EPS_DISTANCE = 1e-8 * max(1, D)
```

最终数值应通过单元测试调整，不要用过大的容差掩盖真实错误。

### 9.2 实现要求

- 坐标和角度计算使用双精度浮点数；
- 输入角度先归一化到 `[0°,360°)`，再转换为弧度；
- 平行判断使用叉积绝对值；
- 距离比较优先使用距离平方；
- 所有顶点输出前按逆时针排列；
- 对接近零面积、接近平行和重复检测点给出警告；
- 不要重复测量同一地点来假设误差能够平均抵消，题目明确同一地点的检测误差固定。

## 10. 模块划分建议

以 Python 为例：

```text
problem1/
├── geometry.py
│   ├── cross
│   ├── dot
│   ├── line_intersection
│   ├── point_in_all_halfplanes
│   └── normalize_angle
├── localization.py
│   ├── build_halfplanes
│   ├── enumerate_feasible_intersections
│   ├── check_boundedness
│   └── solve_localization_polygon
├── convex_hull.py
│   └── andrew_hull
├── diameter.py
│   ├── diameter_bruteforce
│   └── diameter_rotating_calipers
├── minimum_circle.py
│   ├── circle_from_two_points
│   ├── circle_from_three_points
│   └── minimum_enclosing_circle
├── analysis.py
│   ├── diameter_circle_test
│   ├── coverage_ratio
│   └── calculate_metrics
├── experiments.py
├── main.py
└── tests/
```

核心数据结构建议保持简单：

```text
Point      = (x, y)
Line       = (origin, direction)
HalfPlane  = (origin, lower_direction, upper_direction)
Circle     = (center, radius)
```

## 11. 主流程伪代码

```text
function solve_problem_1(detections, delta=1 degree):
    validate input

    constraints = []
    boundary_lines = []

    for detection in detections:
        lower = direction(bearing - delta)
        upper = direction(bearing + delta)

        constraints.append(cross(lower, X-S) >= 0)
        constraints.append(cross(upper, X-S) <= 0)

        boundary_lines.append(line(S, lower))
        boundary_lines.append(line(S, upper))

    if feasible region is unbounded:
        return status = "unbounded"

    candidates = []
    for each pair of boundary lines:
        if not parallel:
            P = line intersection
            if P satisfies every constraint:
                candidates.append(P)

    candidates = merge_near_duplicate_points(candidates)
    polygon = andrew_convex_hull(candidates)

    handle empty / point / segment cases

    D, diameter_pairs = rotating_calipers(polygon)
    verify D using brute force in test mode

    can_cover = false
    best_diameter_circle = null
    for (A, B) in diameter_pairs:
        M = (A+B)/2
        required_radius = max(distance(M, V) for V in polygon)
        if required_radius <= D/2 + EPS:
            can_cover = true
            best_diameter_circle = circle(M, D/2)
            break

    C_star, R_star = minimum_enclosing_circle(polygon)
    K = 2 * R_star / D

    return polygon, D, can_cover, C_star, R_star, K
```

## 12. 实验设计

### 12.1 正确的数据生成方式

不要独立随机生成检测点和示向度，否则可能生成没有公共真实目标的矛盾数据。

建议先生成真实干扰源 `G`，再生成检测信息：

1. 设置真实目标 `G`；
2. 设置检测距离 `r_i` 和检测方向；
3. 生成检测点 `S_i`；
4. 计算从 `S_i` 指向 `G` 的真实方位角 `φ_i`；
5. 生成误差 `e_i in [-1°,1°]`；
6. 设置模拟示向度 `θ_i = φ_i + e_i`；
7. 调用问题1算法并确认 `G` 位于定位区域内。

### 12.2 参数扫描

建议扫描：

| 参数 | 建议范围 |
|---|---|
| 两条示向中心线的夹角 | `5°` 至 `175°`，步长 `5°` |
| 检测距离 | `200、500、800、1000、1500 m` |
| 距离比 `r1/r2` | `0.25` 至 `4` |
| 测角误差 | `-1°` 至 `1°`，包括四组边界组合 |
| 检测点数量 | `2、3、4、5` |

重点输出指标：

```text
交会角 gamma
定位区域面积 A
定位区域直径 D
最小包围圆半径 R*
覆盖系数 K = 2R*/D
是否可被直径圆覆盖
区域长宽比或紧致度
```

### 12.3 推荐图表

1. 交会角与定位区域直径的关系曲线；
2. 交会角与最小包围圆半径的关系曲线；
3. `(交会角, 距离比)` 对应的可覆盖/不可覆盖热力图；
4. `D` 与 `R*` 的散点图；
5. 可覆盖和不可覆盖的典型定位多边形示意图；
6. 两个检测点、三个检测点下定位区域变化的对比图。

实验中可以总结布局规律，但“是否覆盖”必须使用精确几何判据，不使用机器学习分类器代替。

## 13. 单元测试与验收标准

### 13.1 必测场景

- 两个检测点、近似正交交会；
- 两个检测点、接近平行交会；
- 两个检测点距离目标明显不对称；
- 三个及以上检测点；
- 示向度跨越0°；
- 重复检测点或平行边界；
- 空集、点、线段和无界区域；
- 可被直径圆覆盖的多边形；
- 不能被直径圆覆盖的锐角三角形型区域。

### 13.2 性质测试

每组正常数据必须满足：

1. 生成数据时的真实目标位于最终定位区域内；
2. 每个输出顶点满足全部半平面约束；
3. 输出顶点按逆时针排列，且多边形为凸多边形；
4. 暴力枚举直径与旋转卡壳直径在容差内相等；
5. 最小包围圆覆盖全部顶点；
6. `R* >= D/2 - EPS`；
7. 若 `can_diameter_circle_cover=true`，直径圆必须覆盖全部顶点；
8. 若 `R* <= 18 m`，输出 `clear_ready_18m=true`。

## 14. 复杂度

设检测点数量为 `n`，半平面边界数量为 `L=2n`，最终凸包顶点数量为 `m`。

| 步骤 | 复杂度 |
|---|---|
| 枚举边界交点 | `O(L^2)` |
| 全约束可行性筛选 | `O(L^3)` |
| Andrew 凸包 | `O(m log m)` |
| 旋转卡壳直径 | `O(m)` |
| 随机增量最小包围圆 | 期望 `O(m)` |

本题 `n` 很小，定位区域求解的实际耗时可以忽略。算法优化的主要目的不是节省运行时间，而是形成清晰、通用且可验证的论文算法。

## 15. 与后续问题的接口

问题1应向后续模块提供：

```text
polygon_vertices
area
diameter
diameter_pair
minimum_enclosing_circle_center
minimum_enclosing_circle_radius
coverage_ratio
clear_ready_18m
clear_ready_20m
```

后续决策规则建议为：

```text
if minimum_enclosing_circle_radius <= 18 m:
    在最小包围圆圆心执行清除
else:
    选择新的检测点继续缩小定位区域
```

问题2的检测点评价函数可使用：

```math
J(S_2)=R^*_{\rm new}+\lambda\frac{\|S_2-S_1\|}{5},
```

即同时考虑新定位区域的最小包围圆半径和机器狗移动时间。

强化学习或其他学习算法可以把 `D、R*、K、面积、交会角` 作为状态特征，但问题1的几何判定应始终采用上述精确算法，不使用学习模型近似。

## 16. 论文表述建议

问题1建议按以下顺序书写：

1. 定义示向度误差角域；
2. 将每个角域转换为两个线性半平面；
3. 给出交点枚举、可行性筛选与凸包算法；
4. 使用旋转卡壳计算定位区域直径；
5. 证明直径圆不一定覆盖定位区域；
6. 给出直径端点圆的精确覆盖判据；
7. 引入最小包围圆和覆盖系数；
8. 通过参数扫描分析检测几何对定位效果的影响；
9. 将最小包围圆半径作为后续问题的可靠清除条件。

核心结论应明确写为：

> 定位区域直径只能刻画区域内最远两点的距离，不能保证半径为直径一半的圆覆盖整个区域。最小包围圆半径才是判断单次清除位置是否可靠的直接指标。
