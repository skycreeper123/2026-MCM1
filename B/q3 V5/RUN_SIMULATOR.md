# Q3 V5 模拟器接入

## 先检查本地版本

从仓库根目录执行：

```powershell
& 'B/Q3 V5/Start-V5.ps1' -SelfCheck
```

第一行必须包含 `Q3 V5`、`strategy=v5_dynamic` 和 `B\Q3 V5\planner.py`。自检不联网，只检查加载、配置和理论界；不能将理论上界作为实际性能预测。

不要使用旧命令 `python -m B.q3.runner`，它会加载仓库另一个 Q3 目录。

完整回归测试和离线小批验证：

```powershell
& 'B/Q3 V5/Start-V5.ps1' -Tests
& 'B/Q3 V5/Start-V5.ps1' -Validate -Cases 2 -Seed 20260912
```

内置验证器比较 `v3_global` 与 `v5_dynamic`；它不是 V4/V5 对比报告。历史 V4/动态候选 14 场对比见算法 summary。验证器输出到本目录 `validation_results_v5`。

## 连接官方练习模拟器

1. 打开模拟器并登录，在设置中确认 Robot API 端口为 `2026`。
2. 确保没有其他算法客户端连接该端口。
3. 启动下面的 V5 命令；未提供队号时会在终端提示输入。
4. 在模拟器中选择“问题3练习/演练测试”。程序默认等待 API 开放 180 秒，随后完成进入、动作请求与退出。

```powershell
& 'B/Q3 V5/Start-V5.ps1'
```

也可明确指定当前登录队号：

```powershell
& 'B/Q3 V5/Start-V5.ps1' -RobotId '<实际队号>'
```

这里的占位符必须替换为真实队号。默认地址为 `http://127.0.0.1:2026`；若修改了本地端口，用 `-BaseUrl` 指定。运行器只允许本机回环地址。

不要并行运行两个分支连接同一个模拟器。本次准备工作未启动官方练习或正式测试，实际连接仍需在你准备好模拟器后执行上面的命令。

## 检查结果

- 本地行为日志和摘要默认写入 `B/Q3 V5/runs`。
- 运行配置和最终 `planner_summary.strategy` 应为 `v5_dynamic`。
- 成功标准：`task_completed=true`、`completion_certificate=true`、`completion_reason=ALL_CHANNELS_RESOLVED`、`exit_accepted=true`。
- `exit_accepted` 单独为真不能证明全部源已清除；同时对照官方干扰源数量与本地 `cleared_count`。
- 检查 `candidate_origin=dynamic_route` 可确认是否实际使用新增定位候选。
- 保留同场官方 `.result.json`、`.jlog`、`.psum` 以及本地摘要，后续用于逐场核对。

## 运行环境

`Start-V5.ps1` 优先使用本机 Codex 自带 Python 3.12，启动器自动查找仓库 `.modeling-deps` 中的 NumPy/SciPy。可用 `-PythonPath` 指定另一个兼容解释器。

在另一台机器上应先安装 `requirements.txt` 所列依赖，并确保仓库内 Q1/Q2 模块齐全。如果 `.modeling-deps` 中的本机二进制与新解释器不兼容，使用与依赖匹配的环境；不要把加载失败误认为算法失败。
