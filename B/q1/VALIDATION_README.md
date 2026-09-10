# Q1 论文验证材料

论文正文入口：[Q1 结果验证](validation_results/Q1结果验证.md)。包含实验设置、解析检验、随机交叉验证、六测点结果表、图注与结论。

## 图表安排

- `validation_results/q1_diameter_coverage`：图 V1，覆盖与不覆盖双栏主图，建议放论文正文。
- `validation_results/q1_random_validation`：图 V2，40 组随机测向案例的交叉验证误差，可放正文或附录。
- `validation_results/q1_station_layout`：图 V3，六测点全局布局，建议放附录。

每幅图提供 300 dpi PNG 和 SVG 矢量版。SVG 文字转为路径，避免换电脑后缺失中文字体。图号 V1—V3、表号 V1 为暂定编号，合并论文时统一调整。

## 复现

在仓库根目录、使用具备依赖的 Python 执行：

```powershell
python -m pip install -r B/q1/requirements-validation.txt
python -m B.q1.validation
python -m unittest discover -s B/q1/tests -v
```

绘图默认使用 Microsoft YaHei 中文字体；其他系统需安装该字体或将绘图字体设置替换为已安装的中文字体。验证程序重新生成本目录下专属 `validation_results` 内的同名产物。算法实现未作修改。

`metrics.json` 保存全部随机输入、解析及六测点完整输出、实际角误差、误差统计、软件版本和源文件校验值。所有图表及论文数字从同一轮计算生成。六测点输入取自 `examples`，不是官方模拟器数据；输入文件小数舍入后的实际误差重新计算，不直接照抄注入误差字段。

随机验证参照采用独立逐边裁剪实现，但两算法共用角域半平面转换，不能将其描述为全部处理环节都独立。40 组随机案例的误差反映数值实现间的一致性，不是干扰源定位误差，也不是对所有可能输入的误差上界。
