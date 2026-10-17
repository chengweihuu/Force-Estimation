# State 目录准入

这里是科研循环的当前控制面，不保存任务历史、实验日志或完整科学结论。

```text
PROJECT = 长期边界    CURRENT = 当前坐标    TASK = 唯一行动
```

## Git Bootstrap Gate

准备第一个 Active TASK 前，确认项目是有效 Git 工作树并已有初始提交。推荐：

```text
git init -b main
git add -A
git commit -m "chore: initialize research harness"
```

实际命令和提交身份由用户确认。没有有效基线时只可 THINK 和提出 bootstrap 方案，不得准备
`VALIDATE` / `FORMAL`、覆盖 CURRENT 或 Idea。`EXPLORE` 也应在基线后进行，除非用户明确批准
一次性的 bootstrap 迁移。

## THINK 与任务准备

没有 Active TASK 时，科学研究处于 THINK。Codex 可以只读检查必要上下文、讨论方案并形成
待确认计划，但不直接执行科学任务。用户确认研究方向后，只用一次修改将 TASK 从 Inactive
替换为 Active；不得在同一步执行科学任务。普通维护按下一节执行，无需激活 TASK。

未接受的讨论不写入仓库。若需要为具体科学问题跨对话保留探索代码、计算或长篇分析，应纳入
该 H-task 的 `EXPLORE`，输出到 `scratch/<task-id>/`。不得因 execution-only 内容需要持久化而
单独创建 H-task。

## 普通维护

用户明确提出的、不改变研究问题、科学规则或已接受结论的工程与文档维护，可以直接执行，
无需创建或激活 TASK。授权范围包括完成该修改所必需的检查、修复和验证，不包括顺手扩大范围。

没有 Active TASK 时，TASK 保持 Inactive；不创建维护编号、维护阶段或额外计划档案。
完成后报告改动、验证结果和剩余问题，无需任务准备、阶段合同或科研 Closeout。
Git 提交仍按用户的明确授权执行。

有 Active TASK 时，服务于当前研究的维护留在原任务内，并遵守其范围与预算；不得借维护
绕过任务约束。与当前研究无关的维护不得自行抢占主线。

涉及科学含义、冻结条件、结论接受、真实机器人运动或未授权的范围扩展时，继续执行相应
确认规则。只有科学任务边界实质变化时才新建 H-task。维护边界按改动含义判断，不按文件数量
或行数判断；普通维护不构成修改冻结记录或自动接受科学结论的授权。

## TASK 的唯一形状

Inactive 时，`TASK.md` 必须且只能是：

```markdown
# Current Task

Status: Inactive

下一步：根据 CURRENT 进入 THINK；用户确认计划后再替换本文件。
```

Active TASK 必须按以下顺序包含：

1. `Status: Active`；
2. `Identity`：Task ID、唯一 Stage、单行 `Progress`、精确研究对象；
3. `Question`；
4. `Context`：只列执行必需的指针与事实；
5. `Allowed Changes`；
6. `Non-goals`；
7. `Budget`：至少列 Files、Time / Compute、Robot runs；
8. `Done When`：可观察、可检查的成功判据；
9. `Stop and Ask`；
10. `Outputs`：明确位置；
11. Stage 所需合同；若含执行层工程，再加 `Execution Contract`；
12. `Information Gain`：分开 `Scientific` 和 `Engineering`，创建时可为 Pending，关闭前替换为实际获益。

不要使用 `Expected Outcome`：意图由 Question 表达，验收由 Done When 表达，位置由 Outputs
表达。`Progress` 只保存当前检查点并原地覆盖，不追加过程日志。`Robot runs` 默认是 0；只有
用户批准准确运动命令后才能改变。

用户只需说明研究意图、约束和可观察成功标准。Codex 从 CURRENT 和关联文档解析编号并选择
owner、目录、接口、依赖方向和验证位置；存在多个研究含义时按含义消歧，不要求用户查编号。

完整填写示例只在需要构造或审查格式时读取 `EXAMPLES.md`，不要每次任务都加载。

本规则生效前已经 Active 的 TASK 按其启动时合同关闭；不得用该过渡条款新建 `BUILD` 或
延续旧的 Formal-grade 默认要求。

## H-task 边界、Stage 与分支

| Stage | 用途 | 专属内容 | Git / 输出边界 |
|---|---|---|---|
| `EXPLORE` | 快速探索 hypothesis / mechanism，允许修改设计 | 当前 probe 的预算、失败与停止条件 | 默认 `scratch/<task-id>/`，不提交；不要求 Formal-grade binding/hash，不产生 claim-level evidence |
| `VALIDATE` | 固定比较设计并做受控验证 | Validation Contract | 需跟踪的配置/实现从干净基线建 `task/<task-id>`；结果不得追溯晋级为 claim-level evidence |
| `FORMAL` | 仅对已通过 VALIDATE 且需要 claim-level evidence 的主张按 Frozen Protocol 执行 holdout | Experiment Contract | 只运行干净的冻结 commit 和 immutable request，完整绑定环境/数据/hash |

只有 scientific hypothesis、treatment、endpoint、population、inferential unit 或 Frozen
scientific rule 发生实质变化时才创建新 H-task。进入下一 Stage 不是新 H-task 的理由；
Codex 不得自行切换 Stage，用户确认后原地更新同一 TASK 的 Stage 和合同。若上述科学要素变更，
关闭/取代当前 TASK 并准备新 H-task，不在旧 TASK 后追加历史。

CLI、schema、runner、carrier、heartbeat、ID plumbing、logging、测试和小型 refactor 默认留在
当前 TASK。如果它们超出 Allowed Changes / Budget 或需要新依赖、长期公共接口、架构抽象、
启发式、阈值或降级策略，Stop and Ask；在科学边界不变的前提下，用户批准后修订当前 TASK，
不新建 H-task。没有 Active TASK 时，按本文件的普通维护规则执行，TASK 保持 Inactive。

### Validation Contract

| Hypothesis / Claim | Treatment | Baseline | Endpoint | Population / Inferential Unit | Seed Schedule | Code / Config Revision | Failure / Stop Rule |
|---|---|---|---|---|---|---|---|

以上字段必须在观察 endpoint 前固定。观察结果后更改 treatment、endpoint、population 或
inferential unit 等 H-task 边界字段时，创建新 H-task；仅更换 baseline、扩充 seed schedule 或
修订 execution-only 代码/配置时，必须宣告新的 validation round，不得与已观察 round 静默合并；
除非它同时改变上述科学边界或 Frozen scientific rule，不因此自动新建 H-task。

### Experiment Contract

| Claim | Frozen Protocol | Holdout / Data | Metric / Decision Rule | Immutable Request / Environment Binding | Evidence Location |
|---|---|---|---|---|---|

FORMAL 执行前冻结科学 Protocol、holdout、指标、判定/排除规则和数据范围，再通过独立的
immutable request 绑定环境、依赖、数据清单、代码 commit 和 hash。结果记录与研究者接受字段见
`docs/ideas/AGENTS.md`。

### Execution Contract

| Requirement | Owner / Interface | Inputs / Outputs / Scientific Invariants | Verification |
|---|---|---|---|

任何 Stage 中若包含执行层工程，都把它写入当前 TASK 的 Execution Contract，而不创建 `BUILD`。
实现前核对输入输出、单位、坐标系、符号、边界和允许近似；验证通过后才能声称实现与理论一致。

## 执行层依赖与 identity

```text
Scientific Core → Generic Execution Contract → Experiment Binding → Carrier/Host
```

Scientific Core 不得依赖 Experiment ID、Protocol version、Task/Run ID、foreground/detached、tmux/systemd
或其他 carrier/host 细节。Experiment identity 必须由 config/request 注入 Experiment Binding；禁止
core implementation 硬编码具体 Experiment ID 或根据实验名称分支科学逻辑。Generic Execution
Contract 仅描述通用输入输出和生命周期；Carrier/Host 仅负责运输、调度和状态。

runner、carrier、host、schema、CLI、logging、测试和 ID plumbing 变化不得自动升级 Scientific
Protocol。若已冻结 FORMAL 绑定受影响，创建新 immutable request / Experiment Binding / Run 并保留旧绑定；
只有科学规则改变才创建新 Scientific Protocol version。

## 状态与知识边界

| 内容 | 唯一权威位置 |
|---|---|
| 长期项目目标与边界 | `PROJECT.md` |
| 对象指针、一句话接受状态、主要不确定性、下一项决策 | `CURRENT.md` |
| 科学主张、假设、理由和完整当前结论 | 对应 Idea |
| 失败研究记忆的可检索入口 | `docs/RESEARCH_WIKI.md`（只索引权威对象） |
| FORMAL 的冻结科学方法 | Protocol |
| FORMAL 详细结果、协议结果和研究者验证 | Experiment |
| 正式 Python 实现 | `src/` |
| 可丢弃探索 | `scratch/<task-id>/` |

CURRENT 不复制科学结论。接受的科学结论改变时，Closeout 必须在同一批准和同一提交中更新
Idea 的完整结论以及 CURRENT 的指针或一句话接受状态；任一端缺失都不得关闭。

## 内容级 Closeout

以下 Closeout 仅用于科学 TASK；普通维护按上面的规则报告。科学任务达到 Done When 后，
默认只报告四项：

1. `Result`：分别声明 `Scientific information gain` 和 `Engineering information gain`；
2. `Evidence`：只列决定性依据和需批准对象的精确内容/diff；
3. `Remaining uncertainty`；
4. `Next scientific action`：没有时明确写 None，并在需要时附上最小关闭批准请求。

若只产生工程获益，`Result` 必须原文包含：`No new scientific evidence was generated.`
`Evidence` 中的 diff、测试或验证结果不因字段名称自动成为科学 Evidence。

以下内容必须让用户批准“具体内容”，而不只是批准动作：

- CURRENT、Idea、Protocol 或其他权威知识：给出拟写入的完整文本或可读 diff；
- Experiment：给出 ProtocolOutcome、ResearcherVerification 和 ResearcherNote 的拟值；
- 代码或配置：给出准确文件列表和决定性 diff 摘要；
- 删除：给出每个准确路径；不得使用未展开的 glob；
- Git：有已接受的 tracked diff 需提交时，给出纳入提交的准确文件和拟用 commit message。

把上述必要的精确内容放入四项回复中的 `Evidence` 或 `Next scientific action`，不再维护
`Closeout Plan: Pending` 或通用处理表。

用户确认前不得移动、删除、更新长期状态或提交。确认后只执行表中列出的内容；若实际 diff
与批准内容存在实质差异，停止并重新请求批准。

执行获批 Closeout 的顺序：

1. 应用获批的知识、代码和清理动作；
2. 运行合同中的验证并检查最终 diff；
3. 将 TASK 替换为上面的 Inactive 精确形状；
4. 若存在已接受的 tracked diff，用获批的文件集合和 message 创建提交；只有未提交 scratch 的
   `EXPLORE` 不创建空提交；
5. 确认工作树干净。

关闭后不保留 TASK 历史；Git 提交保存已接受历史。FORMAL 的 Experiment 必须保留所有结果，
不能因研究者不同意解释而删除。

## Harness 缺口

若缺口会使当前结果无效，或修复需要改规则、扩大 Allowed Changes，立即 Stop and Ask。其他
一次性摩擦放入 `Remaining uncertainty`。若连续两次因同类 binding、schema、identity、carrier
或 interface 问题失败，停止逐个 patch，不做第三个局部补丁；在当前 TASK 中记录失败类别并
触发一次 execution-layer architecture review 或经用户批准的小范围 refactor。超出原范围时修订当前
TASK；只要科学边界未变，不新建 H-task，不升级 Scientific Protocol。科学判断不得机械化。

## Environment Contract and staged compute

严格性按 Stage 启用：

- `EXPLORE`：只记录当前 probe 的计算/硬件预算、停止条件和理解输出所需的最小环境信息。
  不强制依赖锁、image digest、主机指纹、artifact hash 或未实际计划的后续计算阶梯。
- `VALIDATE`：Validation Contract 在观察 endpoint 前固定 treatment、baseline、endpoint、population、
  inferential unit、seed schedule、代码/配置修订、预算、失败和停止规则。再记录可重放所需的运行模式、
  依赖参照和硬件/驱动；不默认要求 Formal-grade digest/hash。
- `FORMAL`：必须绑定 Frozen Protocol、holdout、immutable request、实际 image digest 或主机指纹、
  依赖锁、数据清单、代码 commit 和必要 hash；所有绑定必须在观察 holdout 前进入 immutable request，
  不得事后覆盖。

每个 Stage 先做当前预算内最低成本、仍可判别的失败检查。仅为实际计划的 probe / pilot / scale
写通过条件、预算和停止条件；前级失败时停止，不自动扩大。`FORMAL` 是 claim-level evidence
阶段，不是所有非零计算的默认末级。`ProtocolOutcome` 只能在 FORMAL 中来自可核验 Run 上的
预声明判定规则；LLM 建议或翻译不是判定输入，也不构成 Evidence。

Scientific Protocol 只在科学规则改变时升版。execution-only 变化可要求新 binding/request/Run，
但不得创建新 Scientific Protocol version。旧 Protocol、request、commit、Run 和不利结果都保留；
不改变数值、判定、范围或排除条件的纯排版/链接修正标记为 `Editorial correction`。

## Context state

结构化状态的最小字段和委派包格式见 `state/CONTEXT.md`。TASK 的 `Progress` 只写当前检查点；原始工具结果不得进入 `CURRENT.md`、Idea、Protocol 或长期对话摘要，通过外部产物的路径和短摘要引用。
`FORMAL` 绑定和完整性确有需要时必须附 hash；`EXPLORE` 不强制 Formal-grade hashing。

使用实验阶梯时，Context 以 `- Execution request: \`相对路径\`` 引用已批准请求，Outputs
列出精确 Run 目录。执行器只原地更新 Progress 为 `workflow: \`runs/.../workflow-state.json\``
及下一步，不复制日志或新增任务。恢复入口为 `node tools/research-workflow.mjs context`；
请求、输入或 TASK 科学合同变化后不得沿旧 Run 继续，按既有权限审阅后使用新 Run。

失败任务关闭前运行 `node tools/research-workflow.mjs wiki audit`。已有 Rejected Idea 或
Refuted/Inconclusive Experiment 必须补齐索引；Wiki 行与权威对象的拟变更放在同一次内容级
Closeout 中审批，不另开一套任务或审批流程。观察尚待解释时保留 Run 的 pending 指针，
不得为填 Wiki 捏造科学状态。普通维护没有科学失败对象时不需要这一步。
