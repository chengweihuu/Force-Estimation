# Current Task

Status: Active

## Identity

- Task ID: H-097
- Stage: EXPLORE
- Progress: 只读源码／既有记录核对完成：JTC实际反馈初值及首点前插值与write250一致，D项由命令跳变主导。报告 `scratch/H-097/jtc_handover/JTC轨迹接管目标跳变_只读核对.md`；未修改参数或重新仿真，下一候选仅列方案。
- Research objects: 无齿隙UR5同步保持启动时目标跳变与PD误差差分；延续H-097原问题。

## Question

未来header的hold命令为何在收到后立刻改变底层位置目标，并触发原力矩保护？仅核对源码和现有记录。

## Context

- Execution request: `scratch/H-097/jtc_handover/reconcile_request.json`
- 来源已核对：`runs/H-097/R-20261027T111541Z-jtcread/inspect`；本步仅原授权5秒以内的既有记录代数核对。
- 用户批准上一轮唯一下一步：只读核对轨迹接管连续性，不仿真、不调参。
- 既有结果：`runs/H-097/R-20261027T110816Z-holdreport/analyze/同步保持修复与实验结论.md`。
- 触发记录：`runs/H-097/R-20261027T110451Z-holdabort/analyze/metrics.json`。
- 原采集：`runs/H-097/R-20261027T110131Z-holdfixed`，主窗未进入。
- 失败记忆：以上原Run与`scratch/H-097/startup_capture/execution_review.md`保留；不重试。
- 账本：`state/H-097_budget.json`，开始累计1838.151/1920 s。

## Allowed Changes

只读安装头文件、官方同版本源码、既有配置／日志；写scratch/H-097/jtc_handover/检查脚本和解释报告、Run输出、TASK与实际预算。不修改控制器／模型／PD／采集程序。必要的来源取回和离线核对经workflow执行。

## Non-goals

无ROS启动、MuJoCo求值或积分、训练、KF、参数修改、物理重采、新依赖、版本切换、CURRENT／Idea改写或Git提交。

## Budget

- Files: `scratch/H-097/jtc_handover/`、`state/TASK.md`、`state/H-097_budget.json`及本Run。
- Time / Compute: 本轮实际2.777 s；当前累计1840.928/1920 s，余额79.072 s。无新增预算；本轮只读工作完成。
- Robot runs: 0

## Done When

明确JTC接收、初始状态选择、header之前采样、position命令写入与底层PD差分的调用链；区分源码事实、既有数据验证和未观测内部量。给出不调PD的下一项最小建议，不执行新实验。

## Stop and Ask

版本或来源不符、所需文件不可得时保留缺口，不换版本或新建仿真。授权仅只读诊断；任何控制语义或启动方案实际修改须另获批准。

## Outputs

- `runs/H-097/R-20261027T111808Z-jtcmatch`

- `runs/H-097/R-20261027T111541Z-jtcread`
- `scratch/H-097/jtc_handover/`

## EXPLORE Probe Contract

复用write249/250与physical2411记录；先取版本对应源码，再逐调用核对。数值计算只使用既有状态、目标、周期和增益，不产生新的物理状态。不同启动方案只在报告比较其必要条件与风险，不宣布其已通过安全测试。

## Execution Contract

| Requirement | Owner / Interface | Inputs / Outputs / Scientific Invariants | Verification |
|---|---|---|---|
| 来源身份 | inspect_sources.py | 安装版本、生成参数默认值、4.42.1官方源码 | 版本明确匹配；不读取README |
| 命令接管解释 | 本任务报告 | 新轨迹、初始点、采样、position输出、PD | 源码行号与既有事件／write数值分别引用 |
| 数据边界 | 既有Run | 原始结果不覆盖；无新物理 | workflow命令与输出计账 |

## Information Gain

- Scientific: 既有失败记录得到同版本源码支持：新轨迹从实际反馈重选初值并在首点前插值；0.5 ms前瞻的五次插值可复现write250，D项分解显示命令跳变主导。属于已有数据的机制解释，未增加物理验证，静态偏置／重力／完整时间链仍未验收。
- Engineering: 核对已安装4.42.1版本、生成默认值、现有配置及调用链；区分JTC名义前瞻和PD实际周期。给出零额外publish的保持启动候选及需要修订的准入／窗口规则，未实施或测试候选。
