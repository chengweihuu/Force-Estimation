# Protocols 目录准入

## 用途

这里只保存准备进入 `FORMAL` 的冻结科学验证规则和跨实验复用的科学数据合同。
协议回答“用什么科学规则验证”，不保存运行结果，也不承载 runner、carrier、host、CLI 或
Experiment ID 等执行身份与调度细节。

## 准入

协议应明确研究主张、treatment、baseline、endpoint、population、inferential unit、
科学数据范围/holdout 规则、指标、判定/排除规则和科学停止条件。这些 Frozen scientific
rules 实质变化时，创建新 H-task 和新 Protocol version，不覆盖原冻结条件。

每份协议命名为 `P-NNN_short_name.md`，并在开头声明：

```markdown
Protocol ID: P-001
Idea ID: I-001
Claim IDs: I-001:C-01
Status: Frozen
```

Protocol 状态使用 `Draft`、`Frozen`、`Retired`。协议定义“怎样验证”，不保存运行结果；
对应 Experiment 由 Idea 的 Evidence Map 建立入口。

协议冻结后开始的 Experiment 无论结果是否有利都必须留下记录。冻结 Protocol 文件保存
自身内容 hash；改变指标、判定/排除规则、科学数据范围或其他科学语义时，必须复制为
新版本（例如 `P-001-v2`）并说明变更理由，旧版本及其 Experiment 不得打补丁或删除。

Environment Contract、依赖锁、实际数据清单、代码 commit/interface、计算资源和 carrier/host
选择由独立 immutable request / Experiment Binding 绑定，不写入 Scientific Protocol 版本身份。
执行层变化需要重跑时，创建新 request/binding/Run 并保留旧绑定；不因此升级 Protocol。

## FORMAL 前的最小反证

Protocol 只服务于已通过 `VALIDATE` 且需要 claim-level evidence 的主张。EXPLORE / VALIDATE 的
probe、pilot 和受控比较留在同一 H-task 的阶段合同中，不写进 Frozen Protocol。FORMAL Protocol
只声明实际需要的最小运行有效性检查、holdout 判定和停止条件；不要求所有 Protocol 复制
`Smoke → Pilot → Scale → Formal` 通用阶梯。有效性检查失败时停止并保留 Invalid/NotEvaluated
记录，不事后改动冻结规则或 holdout。

`ProtocolOutcome` 只能由冻结的指标、判定规则、排除条件和可核验 Run 得到。LLM 的翻译、
摘要或范围建议不属于该判定输入，也不构成 Evidence。External Reviewer 如被使用，只能提供
`plain-language` 或 `scope-check` 的 advisory-only 输出：无工具、无写权、无 Evidence 资格，
不得改写 Protocol 或触发阶段升级。

## 禁止

- `EXPLORE` / `VALIDATE` 计划、运行日志和结果不进入本目录。
- 冻结后不得直接覆盖指标、判定规则、排除条件或数据范围。
- 不得因 runner、carrier、host、schema、CLI、logging、ID plumbing 或环境变化创建新 Protocol version。

## 生命周期

协议先处于 `Draft`，确认后变为 `Frozen`；科学语义实质变更应在新 H-task 中升级版本或建立
新 Protocol。不改变数值、判定、范围或排除条件的纯排版/链接修正标记为 `Editorial correction`。停止
使用的协议标记为 `Retired`，原记录仍保留。
