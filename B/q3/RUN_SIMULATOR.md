# 问题3模拟器演练运行手册

## 运行前检查

1. 只选择“问题3演练测试”，不要选择正式测试。
2. 模拟器已经联网登录，设置中的 Robot API 端口为 `2026`。
3. 当前没有其他机器狗程序连接该端口。
4. 从仓库根目录运行命令。

## 离线检查

```powershell
python -m unittest discover -s B\q3\tests -v
python -m B.q3.runner --self-check
python -m B.q3.runner --self-check --strategy v4_cooperative
python -m B.q3.runner --self-check --strategy v3_global
```

离线固定案例配对验证（不需要启动模拟器）：`python -m B.q3.validation --cases 24`。这类结果只验证自建环境中的算法行为，不是官方演练成绩。`--self-check` 只校验配置和理论界，不会运行案例。`v4_cooperative` 与 `v3_global` 的有限时间界刻意很宽松，不能用来预计实际成绩。

## 演练命令

```powershell
python -m B.q3.runner --robot-id <当前登录参赛队号> --strategy v4_cooperative
```

也可以省略 `--robot-id`，由程序在终端中提示输入：

```powershell
python -m B.q3.runner
```

运行器默认等待 Robot API 开放 180 秒。因此可以先启动命令，再在模拟器中点击
“问题3演练测试”。数据准备和 5 秒倒计时结束后，运行器会自动调用 `/enter`。

`v4_cooperative` 是当前默认策略：七站发现期间按价值选择固定站顺路复测，随后按区域质量联合安排安全测向与认证清除，并对全部待处理区域进行动态路线优化。`--strategy v3_global` 保留上一版全局对照，`v2_local`、`b0_batch_fifo` 和 `b0_serial` 保留旧版对照。V4 的 152 点条带仅用于异常回退。算法与参数解释见 [Q3 summary](Q3_ALGORITHM_SUMMARY.md)。

## 运行中

- 不要再启动第二个客户端。
- 不要关闭模拟器、终端或网络。
- 每个新动作使用新 `request_id`；网络错误只会用相同请求体和相同 ID 重试。
- 终端会显示动作编号、频道、结果、虚拟时间和已清除数量。
- 本地完整请求与响应写入 `B/q3/runs/*.jsonl`。

## 结束后

正常结束时应看到：

```text
Question 3 planner completed and /exit was accepted.
```

同时生成 `*.summary.json`。回到模拟器的问题3演练页面，记录案例编码、干扰源数量、
平均清除时间及行为日志大小。首次演练必须确认加密行为日志不超过 2 MB。

汇总中的 `exit_accepted` 只表示模拟器接受退出；只有 `task_completed=true` 才表示规划器在退出前持有全部频道解决证书。

如果终端报告 `TransportError`，最后一个请求的执行结果可能不确定，不要立即启动另一
客户端；先保留 JSONL 日志并检查模拟器界面。如果报告 `ProtocolError` 或
`RequestRejected`，运行器会在协议允许时尝试安全 `/exit`。

## 正式测试限制

运行器不会在 GUI 中启动测试。正式测试每次启动都会消耗机会，只有在多次演练、日志
大小检查、时间对账和代码冻结全部通过后才能使用。
