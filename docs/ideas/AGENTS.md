# Ideas 与正式结果准入

## Idea 的职责

一个研究 Idea 是持续更新的理论入口，负责保存研究问题、Claims、理论、假设、可证伪条件、
完整当前结论和 Evidence Map。命名为 `I-NNN_short_name.md`，不按日期、讨论轮次或“最终版”
复制文件。

Idea 状态使用 `Exploring`、`Candidate`、`Accepted`、`Rejected`。Claim 在 Idea 内使用
`C-01`、`C-02`，完整引用为 `I-001:C-01`。公式应明确输入输出、单位、坐标系、符号约定、
允许近似和边界情况。

推荐骨架：

```markdown
# Idea 标题

Idea ID: I-001
Status: Exploring

## Research Question
## Core Hypothesis
## Claims

### C-01 简短、可证伪的主张

## Theory
## Assumptions
## Falsification
## Current Conclusion

## Evidence Map

| Claim ID | Claim | Protocol | Experiment | Experiment Status | Protocol Outcome | Researcher Verification | Evidence |
|---|---|---|---|---|---|---|---|
| C-01 | 简短主张 | [P-001](../protocols/P-001_xxx.md) | E-001 | Completed | Supported | Pending | [Result](../../experiments/E-001/README.md) |

## Open Questions
```

## Experiment 的两个判断层

每个 FORMAL Experiment 记录必须包含：

- `ExperimentStatus`: `Planned`、`Running`、`Completed`、`Invalid`；
- `ProtocolOutcome`: `Pending`、`Supported`、`Refuted`、`Inconclusive`、`NotEvaluated`；
- `ResearcherVerification`: `Pending`、`Accepted`、`Disputed`；
- `ResearcherNote`: 研究者接受或保留意见的短说明；
- Task、Claim、Protocol、冻结 commit、Runs、结果摘要和 Deviations。

`ProtocolOutcome` 只表示冻结规则怎样判定结果，不等于研究者已经接受科学解释。FORMAL 完成
后，Codex 可以根据冻结规则写入 `Supported`、`Refuted` 或 `Inconclusive`，但必须保持
`ResearcherVerification: Pending`，直到用户在内容级 Closeout 中确认。

LLM 可以帮助翻译 Protocol、摘要或指出可能的范围歧义，但不能决定 `ProtocolOutcome`、
替代可核验 Run、生成 Evidence，或代替 `ResearcherVerification`。

唯一合法映射：

- `Planned` / `Running` 通常对应 `ProtocolOutcome: Pending`；
- `Completed` 对应 `Supported`、`Refuted` 或 `Inconclusive`；
- `Invalid` 必须对应 `NotEvaluated`，并写清失效、排除或协议偏离原因；
- `ResearcherVerification: Disputed` 不删除或改写协议结果，只在 ResearcherNote 解释异议。

## Evidence Map

Evidence Map 是从 Idea 查看正式证据的唯一入口。每行对应一个已经规划或开始的 FORMAL
Experiment；同一 Claim 有多个 Experiment 时使用多行。所有 `Pending`、`Accepted`、
`Disputed`、`Invalid` 结果都保留，Run 不进入本表。

这里只保存短状态和链接，不复制指标、图片或实验长结论。Experiment 是详细结果的权威来源；
Idea 是科学结论的权威来源；CURRENT 只保存指针和一句话接受状态。

## Failure memory

`Rejected` Idea，以及 `Refuted` 或 `Inconclusive` 的 Experiment，都是必须保留的研究记忆，
不得因不利而删除或以“失败”替换其权威记录。为使其可检索，`docs/RESEARCH_WIKI.md` 应有一条
短索引，至少包含：权威 Idea/Experiment/Protocol 指针、状态、证据指针（若存在）、适用边界、
可复用教训和重启条件。Wiki 只索引，不复制完整科学结论，也不把失败记录伪装成 Evidence。

## 生命周期

新 H-task 只在 scientific hypothesis、treatment、endpoint、population、inferential unit 或
Frozen scientific rule 实质变化时创建，并引用具体 Idea/Claim。`EXPLORE → VALIDATE → FORMAL`
的阶段升级保留同一 H-task identity；CLI、schema、runner、carrier、logging、测试和小型 refactor
等工程工作通过当前 TASK 的 Execution Contract 管理，不创建 `BUILD` 或新 H-task。

`VALIDATE` 的受控结果可用于更新 Candidate 判断和是否进入 FORMAL，但不进入 Evidence Map，
不得追溯性地标为 claim-level evidence。FORMAL 先创建或更新 Experiment，写入协议结果并
保持研究者验证为 Pending；内容级 Closeout 在 `Evidence` / `Next scientific action` 中向用户展示：

1. Experiment 的拟议研究者验证和说明；
2. Idea Evidence Map 的准确行；
3. Idea Current Conclusion 的完整拟写文本或 diff；
4. CURRENT 的一句话状态和下一决策 diff。

用户批准后在同一 Closeout 和同一接受提交中更新相互引用的对象。默认回复仍只保留
`Result`、`Evidence`、`Remaining uncertainty`、`Next scientific action` 四项。两端引用不一致、验证仍需
确认或实际 diff 偏离批准内容时不得关闭任务。

完整示例见 `state/EXAMPLES.md`，只在构造或审查格式时读取。

## 禁止

- 不把 `ProtocolOutcome` 写成研究者已经接受的结论；
- 不隐藏不利、无结论、失效或被研究者质疑的 Experiment；
- 不删除 `Rejected` Idea，或以摘要、LLM 建议、Wiki 索引替代其失败记忆的权威对象；
- Evidence Map 不列 Run，不复制指标、图片或实验长记录；
- 不把未接受的假设伪装成 Current Conclusion。
