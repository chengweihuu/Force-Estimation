# Source 目录准入

## 用途

这里只保存已经接受、愿意维护的 Python 参考实现。用户声明目标功能、研究约束和成功标准；
Codex 负责检查现有代码、选择 owner、规划文件和测试，不要求用户先指定目录。

## 准入

### 按需职责

首个正式模块出现时再创建 package 和 `pyproject.toml`。package 内只按实际内容选择：

| 目录 | 唯一职责 | 不得包含 |
|---|---|---|
| `core/` | 理论、数学、算法和领域模型 | 文件路径、ROS、Experiment ID、画图 |
| `adapters/` | 数据格式和外部系统边界转换 | 核心公式和实验判定 |
| `evaluation/` | 指标、性质检查和判定逻辑 | 具体实验编排 |
| `workflows/` | 组合稳定组件 | 重复的数学实现 |
| `cli.py` | 参数解析和调用入口 | 算法逻辑 |

禁止创建 `common/`、`utils/`、`misc/`、`helpers/`、`old/` 或 `backup/`。无法确定职责时
不得进入本目录：EXPLORE 代码留在 `scratch/`，VALIDATE 候选代码留在同一 H-task 的分支
且不能通过 Landing
Gate。

### Landing Gate

代码进入这里前，Codex 必须确认：

1. 对应已接受的 Claim、理论章节或明确工程能力；
2. 输入输出、单位、坐标系、边界和允许近似明确；
3. owner 和依赖方向唯一；
4. 不硬编码 Task/Run/Experiment ID、Protocol version、运行目录、foreground/detached、
   tmux/systemd、carrier/host 或论文图表；Experiment identity 只能由 config/request 注入下游 binding；
5. 有快速、确定性的性质或单元测试；
6. Closeout 的 `Evidence` / `Next scientific action` 已说明旧实现和探索代码去向。

Python 与 C++ 实现相同理论时，Python 默认是正确性参考。当前 TASK 的 Execution Contract
必须声明 C++ 允许误差，
并用数学性质和小型标准输入做一致性测试。

## 禁止

- 不把 EXPLORE、FORMAL 运行脚本、Task/Experiment 编号或论文画图逻辑塞进 `core/`。
- 不使用职责不清的 `common/`、`utils/`、`misc/`、`helpers/`、`old/` 或 `backup/`。

## 生命周期

代码先在同一 H-task 的 `VALIDATE` 阶段和任务分支中受控验证，通过 Landing Gate 后才进入这里。
CLI、schema、runner、carrier、logging、测试和小型 refactor 留在当前 TASK 的 Execution Contract；
只有科学 H-task 边界变化才创建新 H-task。正式代码的删除、保留或迁移必须在 Closeout 的
`Evidence` / `Next scientific action` 中给出精确路径和 diff 摘要；Git 保存已接受的历史，不创建代码坟场。
