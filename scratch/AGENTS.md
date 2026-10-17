# Scratch 目录准入

## 用途

这里是 `EXPLORE` 的默认工作区，保存尚未承诺长期维护、但需要跨对话或重启保留的分析、
脚本和小型输出。除本 AGENTS 外，目录内容由 Git 忽略。

## 准入

```text
scratch/<task-id>/
```

规则：

- 所有会影响研究判断的探索脚本默认放在这里，不依赖系统 `/tmp` 的持久性；
- 不要求探索代码具有正式 package 结构，可以快速修改；
- 每个目录必须对应一个 Active 或被后续 TASK 明确引用的任务；
- TASK 完成时在 Closeout 的 `Evidence` / `Next scientific action` 中准确说明进入正式维护、
  保留或删除；
- 删除只在用户确认准确路径后执行。

如果某段代码仍需继续探索，它属于尚未完成或后续明确引用的 TASK，而不是永久留在
scratch 的“可能有用代码”。

## 禁止

- 不让正式 `src/`、`ros2_ws/`、FORMAL Experiment 或论文材料依赖本目录。
- 不在这里保存唯一一份正式数据或论文证据。
- 不创建 `old/`、`backup/` 或通用代码坟场。

## 生命周期

任务完成时通过 Closeout 的四项默认字段决定每个对象的去向。探索内容不会自动成为
正式资产或证据；需要进入 `src/` 或 `ros2_ws/` 时，由用户确认同一 H-task 进入 `VALIDATE`，
在任务分支和 Execution Contract 下重新验证。需要 claim-level evidence 时，只有已通过 VALIDATE 的主张
才在同一 H-task 进入 `FORMAL`，并按 Frozen Protocol、holdout 和 immutable request 重新运行；阶段升级
本身不创建新 H-task。
