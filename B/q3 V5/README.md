# Q3 V5

当前 v4 合入动态补充顺路定位候选，独立策略标识为 `v5_dynamic`。

准备状态：52 项测试通过，自检通过，V5 两场离线测试 21/21 源清除、零 fallback；官方模拟器会话尚未启动。

- [算法 summary](Q3_ALGORITHM_SUMMARY.md)
- [模拟器接入说明](RUN_SIMULATOR.md)
- `RELEASE_VERIFICATION.json`：本次打包验证记录。
- `BUILD_MANIFEST.json`：来源与文件完整性记录。

从仓库根目录先执行：

```powershell
& 'B/Q3 V5/Start-V5.ps1' -SelfCheck
```

准备好模拟器后执行 `& 'B/Q3 V5/Start-V5.ps1'`，按提示输入队号，再进入问题3练习测试。实际运行日志保存在本目录 `runs`。
